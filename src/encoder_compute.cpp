#include "encoder_internal.h"

#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace libvc1 {

void validate_compute_params(const vc1_param_t& p) {
    if (p.i_compute_backend!=VC1_COMPUTE_AUTO && p.i_compute_backend!=VC1_COMPUTE_CPU &&
        p.i_compute_backend!=VC1_COMPUTE_VULKAN)
        throw std::runtime_error("compute backend must be auto, cpu, or vulkan");
    if (p.i_compute_device < -1)
        throw std::runtime_error("compute device must be -1 or a nonnegative Vulkan physical-device ordinal");
    if (p.i_vulkan_min_batch < 1 || p.i_vulkan_min_batch > 4194240)
        throw std::runtime_error("Vulkan minimum batch must be 1..4194240 candidates");
    if (p.b_vulkan_force && p.i_compute_backend!=VC1_COMPUTE_VULKAN)
        throw std::runtime_error("Vulkan force override requires --compute vulkan / VC1_COMPUTE_VULKAN");
}

static void copy_device_name(char* dst,size_t dst_size,const std::string& name) {
    if (!dst_size) return;
    std::strncpy(dst,name.c_str(),dst_size-1);
    dst[dst_size-1]='\0';
}

void configure_compute_backend(EncoderConfig& cfg,vc1_stats_t& stats,const vc1_param_t& p) {
    cfg.vulkan_min_batch=p.i_vulkan_min_batch;
    cfg.vulkan_min_batch_sad=p.i_vulkan_min_batch;
    cfg.vulkan_min_batch_satd=p.i_vulkan_min_batch;
    cfg.vulkan_compute.reset();

    stats.i_compute_selected=VC1_COMPUTE_CPU;
    stats.i_compute_device=-1;
    stats.sz_compute_device[0]='\0';
    stats.b_compute_benchmark_ran=0;
    stats.i_compute_benchmark_device=-1;
    stats.sz_compute_benchmark_device[0]='\0';
    stats.i_compute_benchmark_sad_min_batch=0;
    stats.i_compute_benchmark_satd_min_batch=0;
    stats.f_compute_cpu_sad_candidates_per_second=0.0;
    stats.f_compute_vulkan_sad_candidates_per_second=0.0;
    stats.f_compute_cpu_satd_candidates_per_second=0.0;
    stats.f_compute_vulkan_satd_candidates_per_second=0.0;

    // Vulkan is experimental and opt-in. AUTO is retained only as a legacy
    // spelling of the safe CPU default; neither AUTO nor CPU probes a GPU.
    if (p.i_compute_backend!=VC1_COMPUTE_VULKAN) return;

    if (cfg.syntax!=StreamSyntax::Advanced) {
        if (p.b_vulkan_force)
            throw std::runtime_error("forced Vulkan compute currently supports VC-1 Advanced Profile only");
        return;
    }

    std::string error;
    auto vk=VulkanComputeContext::create(p.i_compute_device,error);
    if (!vk) {
        // Ordinary experimental opt-in is fail-safe: unavailable Vulkan simply
        // leaves the selected CPU/SIMD path in place. The explicit force option
        // makes initialization failure fatal so it can be used for smoke tests.
        if (p.b_vulkan_force)
            throw std::runtime_error(std::string("forced Vulkan compute unavailable: ")+error);
        return;
    }

    // --compute vulkan --vulkan-force bypasses benchmark rejection. Keep the
    // user's minimum-batch floor, but otherwise use Vulkan for every eligible
    // batch. This is deliberately a second, explicit opt-in.
    if (p.b_vulkan_force) {
        cfg.vulkan_compute=std::move(vk);
        stats.i_compute_selected=VC1_COMPUTE_VULKAN;
        stats.i_compute_device=cfg.vulkan_compute->device_index();
        copy_device_name(stats.sz_compute_device,sizeof(stats.sz_compute_device),cfg.vulkan_compute->device_name());
        return;
    }

    // Normal experimental Vulkan opt-in compares the real selected CPU/SIMD
    // motion-cost path against Vulkan. The benchmark also finds the first SAD
    // batch size where Vulkan wins so tiny dispatches remain on CPU.
    stats.b_compute_benchmark_ran=1;
    stats.i_compute_benchmark_device=vk->device_index();
    copy_device_name(stats.sz_compute_benchmark_device,sizeof(stats.sz_compute_benchmark_device),vk->device_name());
    ComputeBenchmarkResult bench{};
    try {
        bench=benchmark_vulkan_motion_compute(cfg,*vk,p.i_vulkan_min_batch);
    } catch (...) {
        // A benchmark-time Vulkan failure is recoverable for the ordinary
        // experimental opt-in: keep CPU selected. The force path skipped this
        // benchmark and would have surfaced initialization failures above.
        return;
    }
    stats.i_compute_benchmark_sad_min_batch=bench.sad_min_batch;
    stats.i_compute_benchmark_satd_min_batch=bench.satd_min_batch;
    stats.f_compute_cpu_sad_candidates_per_second=bench.cpu_sad_candidates_per_second;
    stats.f_compute_vulkan_sad_candidates_per_second=bench.vulkan_sad_candidates_per_second;
    stats.f_compute_cpu_satd_candidates_per_second=bench.cpu_satd_candidates_per_second;
    stats.f_compute_vulkan_satd_candidates_per_second=bench.vulkan_satd_candidates_per_second;

    if (bench.sad_min_batch<=0 && bench.satd_min_batch<=0) return;

    cfg.vulkan_min_batch_sad=bench.sad_min_batch>0?bench.sad_min_batch:0;
    cfg.vulkan_min_batch_satd=bench.satd_min_batch>0?bench.satd_min_batch:0;
    cfg.vulkan_compute=std::move(vk);
    stats.i_compute_selected=VC1_COMPUTE_VULKAN;
    stats.i_compute_device=cfg.vulkan_compute->device_index();
    copy_device_name(stats.sz_compute_device,sizeof(stats.sz_compute_device),cfg.vulkan_compute->device_name());
}

void update_compute_stats(const EncoderConfig& cfg,vc1_stats_t& stats) {
    if (!cfg.vulkan_compute) return;
    const auto c=cfg.vulkan_compute->counters();
    stats.i_compute_dispatches=c.dispatches;
    stats.i_compute_candidates=c.candidates;
    stats.i_compute_frame_uploads=c.frame_uploads;
    stats.i_compute_frame_upload_bytes=c.frame_upload_bytes;
    stats.i_compute_submit_wait_nanoseconds=c.submit_wait_nanoseconds;
}

} // namespace libvc1
