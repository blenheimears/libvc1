#include "vulkan_compute.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

#if defined(LIBVC1_HAVE_VULKAN)
#include <vulkan/vulkan.h>
#include "vulkan_motion_cost_spv.h"
#endif

namespace libvc1 {

VulkanComputeContext::VulkanComputeContext() = default;

#if defined(LIBVC1_HAVE_VULKAN)

namespace {

static void vk_check(VkResult r, const char* what) {
    if (r != VK_SUCCESS)
        throw std::runtime_error(std::string(what) + " failed (VkResult=" + std::to_string(static_cast<int>(r)) + ")");
}

static uint64_t frame_fingerprint(const uint8_t* p, size_t n) {
    // This is only a cache-invalidation tag, not a content hash. Sampling the
    // frame avoids re-reading megabytes on every motion-search dispatch while
    // still detecting ordinary decoder/input-buffer reuse at the same address.
    uint64_t h = 1469598103934665603ull;
    if (!p || !n) return h;
    constexpr size_t samples = 97;
    for (size_t i = 0; i < samples; ++i) {
        const size_t pos = (i * (n - 1)) / (samples - 1);
        h ^= static_cast<uint64_t>(p[pos]);
        h *= 1099511628211ull;
    }
    h ^= static_cast<uint64_t>(n);
    h *= 1099511628211ull;
    return h;
}

static VkDeviceSize round_up4(VkDeviceSize n) { return (n + 3u) & ~VkDeviceSize(3u); }

struct PushParams {
    int32_t width;
    int32_t height;
    int32_t x0;
    int32_t y0;
    int32_t bw;
    int32_t bh;
    int32_t sample_stride;
    int32_t bilinear;
    int32_t rnd;
    int32_t metric;
    int32_t candidate_count;
    int32_t reserved;
};
static_assert(sizeof(PushParams) == 48, "shader push-constant ABI mismatch");

} // namespace

struct VulkanComputeContext::Impl {
    VkInstance instance = VK_NULL_HANDLE;
    VkPhysicalDevice physical = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    uint32_t queue_family = 0;
    VkPhysicalDeviceProperties properties{};
    VkPhysicalDeviceMemoryProperties memory_properties{};

    VkDescriptorSetLayout descriptor_layout = VK_NULL_HANDLE;
    VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;
    VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
    VkCommandPool command_pool = VK_NULL_HANDLE;
    VkCommandBuffer command_buffer = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;

    std::string name;
    int selected_index = -1;
    mutable std::mutex mutex;
    VulkanComputeCounters stats{};
    uint64_t age_counter = 0;

    struct Buffer {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        VkDeviceSize capacity = 0;
        void* mapped = nullptr;
    };

    struct FrameSlot {
        Buffer staging;
        Buffer device_buffer;
        const uint8_t* host_ptr = nullptr;
        int width = 0;
        int height = 0;
        uint64_t fingerprint = 0;
        uint64_t age = 0;
        bool valid = false;
        bool dirty = false;
    };

    static constexpr size_t kFrameSlots = 8;
    std::array<FrameSlot, kFrameSlots> frames{};
    Buffer candidate_buffer;
    Buffer result_buffer;

    ~Impl() {
        if (device != VK_NULL_HANDLE) vkDeviceWaitIdle(device);
        destroy_buffer(candidate_buffer);
        destroy_buffer(result_buffer);
        for (auto& f : frames) {
            destroy_buffer(f.staging);
            destroy_buffer(f.device_buffer);
        }
        if (fence) vkDestroyFence(device, fence, nullptr);
        if (command_pool) vkDestroyCommandPool(device, command_pool, nullptr);
        if (pipeline) vkDestroyPipeline(device, pipeline, nullptr);
        if (pipeline_layout) vkDestroyPipelineLayout(device, pipeline_layout, nullptr);
        if (descriptor_pool) vkDestroyDescriptorPool(device, descriptor_pool, nullptr);
        if (descriptor_layout) vkDestroyDescriptorSetLayout(device, descriptor_layout, nullptr);
        if (device) vkDestroyDevice(device, nullptr);
        if (instance) vkDestroyInstance(instance, nullptr);
    }

    void destroy_buffer(Buffer& b) {
        if (device == VK_NULL_HANDLE) return;
        if (b.mapped && b.memory) vkUnmapMemory(device, b.memory);
        if (b.buffer) vkDestroyBuffer(device, b.buffer, nullptr);
        if (b.memory) vkFreeMemory(device, b.memory, nullptr);
        b = {};
    }

    uint32_t memory_type(uint32_t bits, VkMemoryPropertyFlags required,
                         VkMemoryPropertyFlags preferred = 0) const {
        int fallback = -1;
        for (uint32_t i = 0; i < memory_properties.memoryTypeCount; ++i) {
            if (!(bits & (1u << i))) continue;
            const auto flags = memory_properties.memoryTypes[i].propertyFlags;
            if ((flags & required) != required) continue;
            if ((flags & preferred) == preferred) return i;
            if (fallback < 0) fallback = static_cast<int>(i);
        }
        if (fallback >= 0) return static_cast<uint32_t>(fallback);
        throw std::runtime_error("no compatible Vulkan memory type");
    }

    void create_buffer(Buffer& out, VkDeviceSize size, VkBufferUsageFlags usage,
                       VkMemoryPropertyFlags required, VkMemoryPropertyFlags preferred,
                       bool map) {
        size = std::max<VkDeviceSize>(4, round_up4(size));
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bi.size = size;
        bi.usage = usage;
        bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        vk_check(vkCreateBuffer(device, &bi, nullptr, &out.buffer), "vkCreateBuffer");
        VkMemoryRequirements req{};
        vkGetBufferMemoryRequirements(device, out.buffer, &req);
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.allocationSize = req.size;
        ai.memoryTypeIndex = memory_type(req.memoryTypeBits, required, preferred);
        try {
            vk_check(vkAllocateMemory(device, &ai, nullptr, &out.memory), "vkAllocateMemory");
            vk_check(vkBindBufferMemory(device, out.buffer, out.memory, 0), "vkBindBufferMemory");
            if (map)
                vk_check(vkMapMemory(device, out.memory, 0, VK_WHOLE_SIZE, 0, &out.mapped), "vkMapMemory");
            out.capacity = size;
        } catch (...) {
            destroy_buffer(out);
            throw;
        }
    }

    void ensure_buffer(Buffer& b, VkDeviceSize need, VkBufferUsageFlags usage,
                       VkMemoryPropertyFlags required, VkMemoryPropertyFlags preferred,
                       bool map) {
        if (b.capacity >= need && b.buffer != VK_NULL_HANDLE) return;
        VkDeviceSize cap = 4096;
        while (cap < need) {
            if (cap > std::numeric_limits<VkDeviceSize>::max() / 2) { cap = need; break; }
            cap *= 2;
        }
        destroy_buffer(b);
        create_buffer(b, cap, usage, required, preferred, map);
    }

    FrameSlot& acquire_frame(const uint8_t* data, int width, int height) {
        const size_t bytes = static_cast<size_t>(width) * static_cast<size_t>(height);
        const uint64_t fp = frame_fingerprint(data, bytes);
        for (auto& slot : frames) {
            if (slot.valid && slot.host_ptr == data && slot.width == width && slot.height == height &&
                slot.fingerprint == fp) {
                slot.age = ++age_counter;
                slot.dirty = false;
                return slot;
            }
        }
        auto* slot = &frames[0];
        for (auto& candidate : frames) {
            if (!candidate.valid) { slot = &candidate; break; }
            if (candidate.age < slot->age) slot = &candidate;
        }
        const VkDeviceSize padded = round_up4(static_cast<VkDeviceSize>(bytes));
        ensure_buffer(slot->staging, padded, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                      0, true);
        ensure_buffer(slot->device_buffer, padded,
                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0, false);
        std::memcpy(slot->staging.mapped, data, bytes);
        if (padded > bytes)
            std::memset(static_cast<uint8_t*>(slot->staging.mapped) + bytes, 0,
                        static_cast<size_t>(padded - bytes));
        slot->host_ptr = data;
        slot->width = width;
        slot->height = height;
        slot->fingerprint = fp;
        slot->age = ++age_counter;
        slot->valid = true;
        slot->dirty = true;
        ++stats.frame_uploads;
        stats.frame_upload_bytes += bytes;
        return *slot;
    }

    void initialize(int requested_device) {
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "libvc1";
        app.applicationVersion = VK_MAKE_VERSION(0, 2, 30);
        app.pEngineName = "libvc1-vulkan-compute";
        app.engineVersion = VK_MAKE_VERSION(0, 2, 30);
        app.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo ii{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        ii.pApplicationInfo = &app;
        vk_check(vkCreateInstance(&ii, nullptr, &instance), "vkCreateInstance");

        uint32_t physical_count = 0;
        vk_check(vkEnumeratePhysicalDevices(instance, &physical_count, nullptr), "vkEnumeratePhysicalDevices(count)");
        if (!physical_count) throw std::runtime_error("no Vulkan physical devices found");
        std::vector<VkPhysicalDevice> physicals(physical_count);
        vk_check(vkEnumeratePhysicalDevices(instance, &physical_count, physicals.data()), "vkEnumeratePhysicalDevices");

        struct DeviceChoice { VkPhysicalDevice dev; uint32_t qf; VkPhysicalDeviceProperties props; int ordinal; };
        std::vector<DeviceChoice> choices;
        for (uint32_t ordinal = 0; ordinal < physical_count; ++ordinal) {
            uint32_t qcount = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(physicals[ordinal], &qcount, nullptr);
            std::vector<VkQueueFamilyProperties> qprops(qcount);
            vkGetPhysicalDeviceQueueFamilyProperties(physicals[ordinal], &qcount, qprops.data());
            for (uint32_t q = 0; q < qcount; ++q) {
                if (qprops[q].queueCount && (qprops[q].queueFlags & VK_QUEUE_COMPUTE_BIT)) {
                    VkPhysicalDeviceProperties props{};
                    vkGetPhysicalDeviceProperties(physicals[ordinal], &props);
                    choices.push_back({physicals[ordinal], q, props, static_cast<int>(ordinal)});
                    break;
                }
            }
        }
        if (choices.empty()) throw std::runtime_error("no Vulkan device exposes a compute queue");

        size_t selected = 0;
        if (requested_device >= 0) {
            auto it = std::find_if(choices.begin(), choices.end(), [&](const DeviceChoice& c) { return c.ordinal == requested_device; });
            if (it == choices.end()) throw std::runtime_error("requested Vulkan device has no compute queue or does not exist");
            selected = static_cast<size_t>(std::distance(choices.begin(), it));
        } else {
            auto rank = [](VkPhysicalDeviceType t) {
                switch (t) {
                    case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: return 0;
                    case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: return 1;
                    case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: return 2;
                    case VK_PHYSICAL_DEVICE_TYPE_CPU: return 4;
                    default: return 3;
                }
            };
            for (size_t i = 1; i < choices.size(); ++i)
                if (rank(choices[i].props.deviceType) < rank(choices[selected].props.deviceType)) selected = i;
        }
        physical = choices[selected].dev;
        queue_family = choices[selected].qf;
        properties = choices[selected].props;
        selected_index = choices[selected].ordinal;
        name = properties.deviceName;
        vkGetPhysicalDeviceMemoryProperties(physical, &memory_properties);

        const float priority = 1.0f;
        VkDeviceQueueCreateInfo qi{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qi.queueFamilyIndex = queue_family;
        qi.queueCount = 1;
        qi.pQueuePriorities = &priority;
        VkDeviceCreateInfo di{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        di.queueCreateInfoCount = 1;
        di.pQueueCreateInfos = &qi;
        vk_check(vkCreateDevice(physical, &di, nullptr, &device), "vkCreateDevice");
        vkGetDeviceQueue(device, queue_family, 0, &queue);

        std::array<VkDescriptorSetLayoutBinding, 4> bindings{};
        for (uint32_t i = 0; i < bindings.size(); ++i) {
            bindings[i].binding = i;
            bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo dli{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        dli.bindingCount = static_cast<uint32_t>(bindings.size());
        dli.pBindings = bindings.data();
        vk_check(vkCreateDescriptorSetLayout(device, &dli, nullptr, &descriptor_layout), "vkCreateDescriptorSetLayout");

        VkPushConstantRange pcr{};
        pcr.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        pcr.offset = 0;
        pcr.size = sizeof(PushParams);
        VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pli.setLayoutCount = 1;
        pli.pSetLayouts = &descriptor_layout;
        pli.pushConstantRangeCount = 1;
        pli.pPushConstantRanges = &pcr;
        vk_check(vkCreatePipelineLayout(device, &pli, nullptr, &pipeline_layout), "vkCreatePipelineLayout");

        VkShaderModuleCreateInfo smi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        smi.codeSize = kVulkanMotionCostSpvSize;
        smi.pCode = kVulkanMotionCostSpv;
        VkShaderModule shader = VK_NULL_HANDLE;
        vk_check(vkCreateShaderModule(device, &smi, nullptr, &shader), "vkCreateShaderModule");
        VkPipelineShaderStageCreateInfo stage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        stage.module = shader;
        stage.pName = "main";
        VkComputePipelineCreateInfo cpi{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        cpi.stage = stage;
        cpi.layout = pipeline_layout;
        const VkResult pipeline_result = vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &cpi, nullptr, &pipeline);
        vkDestroyShaderModule(device, shader, nullptr);
        vk_check(pipeline_result, "vkCreateComputePipelines");

        VkDescriptorPoolSize pool_size{};
        pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        pool_size.descriptorCount = 4;
        VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        dpi.maxSets = 1;
        dpi.poolSizeCount = 1;
        dpi.pPoolSizes = &pool_size;
        vk_check(vkCreateDescriptorPool(device, &dpi, nullptr, &descriptor_pool), "vkCreateDescriptorPool");
        VkDescriptorSetAllocateInfo dai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dai.descriptorPool = descriptor_pool;
        dai.descriptorSetCount = 1;
        dai.pSetLayouts = &descriptor_layout;
        vk_check(vkAllocateDescriptorSets(device, &dai, &descriptor_set), "vkAllocateDescriptorSets");

        VkCommandPoolCreateInfo cpool{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        cpool.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        cpool.queueFamilyIndex = queue_family;
        vk_check(vkCreateCommandPool(device, &cpool, nullptr, &command_pool), "vkCreateCommandPool");
        VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.commandPool = command_pool;
        cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = 1;
        vk_check(vkAllocateCommandBuffers(device, &cai, &command_buffer), "vkAllocateCommandBuffers");
        VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        vk_check(vkCreateFence(device, &fi, nullptr, &fence), "vkCreateFence");
    }

    bool score(const uint8_t* cur, const uint8_t* ref, int width, int height,
               int x0, int y0, int bw, int bh, int sample_stride,
               bool bilinear, bool rnd, VulkanMotionMetric metric,
               const VulkanMotionCandidate* candidates, size_t count,
               std::vector<uint64_t>& costs, std::string* error) {
        try {
            if (!cur || !ref || width <= 0 || height <= 0 || bw <= 0 || bh <= 0 || !candidates || !count) {
                costs.clear();
                return true;
            }
            if (count > 64u * 65535u)
                throw std::runtime_error("Vulkan motion batches above 4,194,240 candidates are not supported by the shader ABI");
            if (static_cast<uint64_t>(width) * static_cast<uint64_t>(height) > static_cast<uint64_t>(std::numeric_limits<int32_t>::max()))
                throw std::runtime_error("Vulkan luma plane exceeds 32-bit shader addressing");

            std::lock_guard<std::mutex> lock(mutex);
            auto& cur_slot = acquire_frame(cur, width, height);
            auto& ref_slot = acquire_frame(ref, width, height);
            ensure_buffer(candidate_buffer, count * sizeof(VulkanMotionCandidate), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, 0, true);
            ensure_buffer(result_buffer, count * sizeof(uint32_t), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, 0, true);
            std::memcpy(candidate_buffer.mapped, candidates, count * sizeof(VulkanMotionCandidate));

            std::array<VkDescriptorBufferInfo, 4> infos{};
            infos[0] = {cur_slot.device_buffer.buffer, 0, VK_WHOLE_SIZE};
            infos[1] = {ref_slot.device_buffer.buffer, 0, VK_WHOLE_SIZE};
            infos[2] = {candidate_buffer.buffer, 0, count * sizeof(VulkanMotionCandidate)};
            infos[3] = {result_buffer.buffer, 0, count * sizeof(uint32_t)};
            std::array<VkWriteDescriptorSet, 4> writes{};
            for (uint32_t i = 0; i < writes.size(); ++i) {
                writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[i].dstSet = descriptor_set;
                writes[i].dstBinding = i;
                writes[i].descriptorCount = 1;
                writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                writes[i].pBufferInfo = &infos[i];
            }
            vkUpdateDescriptorSets(device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

            vk_check(vkResetCommandBuffer(command_buffer, 0), "vkResetCommandBuffer");
            VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
            vk_check(vkBeginCommandBuffer(command_buffer, &begin), "vkBeginCommandBuffer");

            std::array<FrameSlot*, 2> possible{{&cur_slot, &ref_slot}};
            std::array<FrameSlot*, 2> dirty{};
            size_t dirty_count = 0;
            for (auto* slot : possible) {
                if (!slot->dirty) continue;
                bool duplicate = false;
                for (size_t i = 0; i < dirty_count; ++i) if (dirty[i] == slot) duplicate = true;
                if (!duplicate) dirty[dirty_count++] = slot;
            }
            std::array<VkBufferMemoryBarrier, 2> barriers{};
            for (size_t i = 0; i < dirty_count; ++i) {
                auto* slot = dirty[i];
                const VkDeviceSize bytes = round_up4(static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height));
                VkBufferCopy copy{0, 0, bytes};
                vkCmdCopyBuffer(command_buffer, slot->staging.buffer, slot->device_buffer.buffer, 1, &copy);
                barriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
                barriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                barriers[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                barriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barriers[i].buffer = slot->device_buffer.buffer;
                barriers[i].offset = 0;
                barriers[i].size = bytes;
                slot->dirty = false;
            }
            if (dirty_count) {
                vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                     0, 0, nullptr, static_cast<uint32_t>(dirty_count), barriers.data(), 0, nullptr);
            }

            PushParams params{width, height, x0, y0, bw, bh, std::max(sample_stride, 1),
                              bilinear ? 1 : 0, rnd ? 1 : 0,
                              metric == VulkanMotionMetric::Satd ? 1 : 0,
                              static_cast<int32_t>(count), 0};
            vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
            vkCmdBindDescriptorSets(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout,
                                    0, 1, &descriptor_set, 0, nullptr);
            vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(params), &params);

            const uint32_t groups = static_cast<uint32_t>((count + 63u) / 64u);
            vkCmdDispatch(command_buffer, groups, 1, 1);

            VkBufferMemoryBarrier host_barrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
            host_barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            host_barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
            host_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            host_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            host_barrier.buffer = result_buffer.buffer;
            host_barrier.offset = 0;
            host_barrier.size = count * sizeof(uint32_t);
            vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_HOST_BIT,
                                 0, 0, nullptr, 1, &host_barrier, 0, nullptr);
            vk_check(vkEndCommandBuffer(command_buffer), "vkEndCommandBuffer");

            vk_check(vkResetFences(device, 1, &fence), "vkResetFences");
            VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            submit.commandBufferCount = 1;
            submit.pCommandBuffers = &command_buffer;
            const auto t0 = std::chrono::steady_clock::now();
            vk_check(vkQueueSubmit(queue, 1, &submit, fence), "vkQueueSubmit");
            vk_check(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX), "vkWaitForFences");
            const auto t1 = std::chrono::steady_clock::now();

            const auto* out = static_cast<const uint32_t*>(result_buffer.mapped);
            costs.resize(count);
            for (size_t i = 0; i < count; ++i) costs[i] = out[i];
            ++stats.dispatches;
            stats.candidates += count;
            stats.submit_wait_nanoseconds += static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
            return true;
        } catch (const std::exception& ex) {
            if (error) *error = ex.what();
            return false;
        }
    }
};

std::shared_ptr<VulkanComputeContext> VulkanComputeContext::create(int requested_device,
                                                                   std::string& error) {
    try {
        auto out = std::shared_ptr<VulkanComputeContext>(new VulkanComputeContext());
        out->impl_ = std::make_unique<Impl>();
        out->impl_->initialize(requested_device);
        error.clear();
        return out;
    } catch (const std::exception& ex) {
        error = ex.what();
        return {};
    }
}

VulkanComputeContext::~VulkanComputeContext() = default;
const std::string& VulkanComputeContext::device_name() const {
    static const std::string none;
    return impl_ ? impl_->name : none;
}
int VulkanComputeContext::device_index() const { return impl_ ? impl_->selected_index : -1; }

bool VulkanComputeContext::score_motion_batch(const uint8_t* cur, const uint8_t* ref,
                                               int width, int height, int x0, int y0,
                                               int bw, int bh, int sample_stride,
                                               bool bilinear, bool rnd, VulkanMotionMetric metric,
                                               const VulkanMotionCandidate* candidates,
                                               size_t candidate_count,
                                               std::vector<uint64_t>& costs,
                                               std::string* error) {
    if (!impl_) return false;
    return impl_->score(cur, ref, width, height, x0, y0, bw, bh, sample_stride,
                        bilinear, rnd, metric, candidates, candidate_count, costs, error);
}

VulkanComputeCounters VulkanComputeContext::counters() const {
    if (!impl_) return {};
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats;
}

void VulkanComputeContext::reset_counters() {
    if (!impl_) return;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stats = {};
}

#else

struct VulkanComputeContext::Impl {};
std::shared_ptr<VulkanComputeContext> VulkanComputeContext::create(int, std::string& error) {
    error = "Vulkan compute support was not compiled into this libvc1 build";
    return {};
}
VulkanComputeContext::~VulkanComputeContext() = default;
const std::string& VulkanComputeContext::device_name() const { static const std::string none; return none; }
int VulkanComputeContext::device_index() const { return -1; }
bool VulkanComputeContext::score_motion_batch(const uint8_t*, const uint8_t*, int, int, int, int,
                                               int, int, int, bool, bool, VulkanMotionMetric,
                                               const VulkanMotionCandidate*, size_t,
                                               std::vector<uint64_t>&, std::string*) { return false; }
VulkanComputeCounters VulkanComputeContext::counters() const { return {}; }
void VulkanComputeContext::reset_counters() {}

#endif

} // namespace libvc1
