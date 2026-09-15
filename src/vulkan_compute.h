#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace libvc1 {

struct VulkanMotionCandidate {
    int32_t xq = 0;
    int32_t yq = 0;
};

enum class VulkanMotionMetric : uint8_t { Sad = 0, Satd = 1 };

struct VulkanComputeCounters {
    uint64_t dispatches = 0;
    uint64_t candidates = 0;
    uint64_t frame_uploads = 0;
    uint64_t frame_upload_bytes = 0;
    uint64_t submit_wait_nanoseconds = 0;
};

class VulkanComputeContext {
public:
    static std::shared_ptr<VulkanComputeContext> create(int requested_device,
                                                         std::string& error);
    ~VulkanComputeContext();

    VulkanComputeContext(const VulkanComputeContext&) = delete;
    VulkanComputeContext& operator=(const VulkanComputeContext&) = delete;

    const std::string& device_name() const;
    int device_index() const;

    // Scores quarter-pixel luma motion candidates. Integer-pixel motion is
    // represented by xq/yq values divisible by four. The shader implements the
    // same clamped VC-1 mspel/bilinear interpolation and 4x4 SATD law as the CPU
    // path. Returns false on a recoverable Vulkan failure so callers can use the
    // existing CPU/SIMD implementation without changing the encoded bitstream.
    bool score_motion_batch(const uint8_t* cur, const uint8_t* ref,
                            int width, int height,
                            int x0, int y0, int bw, int bh,
                            int sample_stride, bool bilinear, bool rnd,
                            VulkanMotionMetric metric,
                            const VulkanMotionCandidate* candidates,
                            size_t candidate_count,
                            std::vector<uint64_t>& costs,
                            std::string* error = nullptr);

    VulkanComputeCounters counters() const;
    void reset_counters();

private:
    VulkanComputeContext();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace libvc1
