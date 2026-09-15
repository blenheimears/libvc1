#include "encoder_internal.h"
#include "vulkan_compute.h"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

using libvc1::EncoderConfig;
using libvc1::Frame;
using libvc1::Vc1Encoder;
using libvc1::VulkanComputeContext;
using libvc1::VulkanMotionCandidate;
using libvc1::VulkanMotionMetric;

static uint64_t integer_sampled_sad(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                                    int dx,int dy,int stride) {
    const int aw=std::max(0,std::min(bw,cur.width-x0));
    const int ah=std::max(0,std::min(bh,cur.height-y0));
    uint64_t sad=0;
    for(int y=0;y<ah;y+=stride) for(int x=0;x<aw;x+=stride) {
        const int rx=std::clamp(x0+x+dx,0,ref.width-1);
        const int ry=std::clamp(y0+y+dy,0,ref.height-1);
        sad+=static_cast<uint64_t>(std::abs(static_cast<int>(cur.y[static_cast<size_t>(y0+y)*cur.width+x0+x])-
                                            static_cast<int>(ref.y[static_cast<size_t>(ry)*ref.width+rx])));
    }
    return sad*static_cast<uint64_t>(stride*stride);
}

int main() {
    std::string error;
    auto vk=VulkanComputeContext::create(-1,error);
    if(!vk) {
        std::cerr<<"Vulkan runtime unavailable: "<<error<<"\n";
        return 77;
    }

    Frame cur,ref; cur.width=ref.width=48; cur.height=ref.height=40;
    cur.y.resize(static_cast<size_t>(cur.width)*cur.height);
    ref.y.resize(static_cast<size_t>(ref.width)*ref.height);
    for(int y=0;y<cur.height;++y) for(int x=0;x<cur.width;++x) {
        cur.y[static_cast<size_t>(y)*cur.width+x]=static_cast<uint8_t>((x*13+y*29+(x*y)%37+17)&255);
        ref.y[static_cast<size_t>(y)*ref.width+x]=static_cast<uint8_t>((x*7+y*19+(x*y)%53+61)&255);
    }

    EncoderConfig cfg; cfg.width=cur.width; cfg.height=cur.height; cfg.simd_dispatch.tier.fill(libvc1::SimdTier::None);
    Vc1Encoder cpu(cfg);
    const std::vector<Vc1Encoder::MotionVector> mvs={{0,0},{4,0},{-4,0},{0,4},{0,-4},{3,1},{-3,2},{7,-5},
        {-9,-7},{12,8},{-16,11},{1,3},{2,2},{-1,-3},{20,-12},{-23,17},{31,29},{-32,-27},{5,-19},{-21,6}};
    std::vector<VulkanMotionCandidate> candidates; candidates.reserve(mvs.size());
    for(auto mv:mvs) candidates.push_back({mv.xq,mv.yq});

    auto compare=[&](Vc1Encoder::ProgressiveMvMode mode,VulkanMotionMetric metric) {
        std::vector<uint64_t> got;
        if(!vk->score_motion_batch(cur.y.data(),ref.y.data(),cur.width,cur.height,5,7,16,16,1,
                                   Vc1Encoder::mv_mode_bilinear(mode),cfg.rndctrl,metric,
                                   candidates.data(),candidates.size(),got,&error)) {
            std::cerr<<"Vulkan dispatch failed: "<<error<<"\n"; return false;
        }
        if(got.size()!=mvs.size()) return false;
        for(size_t i=0;i<mvs.size();++i) {
            const uint64_t expected=metric==VulkanMotionMetric::Sad
                ?cpu.block_sad_motion(cur,ref,5,7,16,16,mvs[i],mode)
                :cpu.block_satd_motion(cur,ref,5,7,16,16,mvs[i],mode);
            if(got[i]!=expected) {
                std::cerr<<"cost mismatch at "<<i<<": got "<<got[i]<<" expected "<<expected<<"\n";
                return false;
            }
        }
        return true;
    };

    if(!compare(Vc1Encoder::ProgressiveMvMode::OneMvQpel,VulkanMotionMetric::Sad)) return 2;
    if(!compare(Vc1Encoder::ProgressiveMvMode::OneMvQpel,VulkanMotionMetric::Satd)) return 3;
    // Bilinear mode is half-pel syntax; test only legal even-qpel candidates.
    std::vector<VulkanMotionCandidate> half_candidates;
    std::vector<Vc1Encoder::MotionVector> half_mvs;
    for(auto mv:mvs) if(!(mv.xq&1) && !(mv.yq&1)) { half_mvs.push_back(mv); half_candidates.push_back({mv.xq,mv.yq}); }
    {
        std::vector<uint64_t> got;
        if(!vk->score_motion_batch(cur.y.data(),ref.y.data(),cur.width,cur.height,3,4,16,16,1,true,cfg.rndctrl,
                                   VulkanMotionMetric::Sad,half_candidates.data(),half_candidates.size(),got,&error)) return 4;
        for(size_t i=0;i<half_mvs.size();++i)
            if(got[i]!=cpu.block_sad_motion(cur,ref,3,4,16,16,half_mvs[i],Vc1Encoder::ProgressiveMvMode::OneMvHpelBilinear)) return 5;
    }
    for(int stride: {2,4}) {
        std::vector<VulkanMotionCandidate> integer_candidates;
        std::vector<Vc1Encoder::MotionVector> integer_mvs;
        for(auto mv:mvs) if((mv.xq%4)==0 && (mv.yq%4)==0) { integer_mvs.push_back(mv); integer_candidates.push_back({mv.xq,mv.yq}); }
        std::vector<uint64_t> got;
        if(!vk->score_motion_batch(cur.y.data(),ref.y.data(),cur.width,cur.height,5,7,16,16,stride,false,cfg.rndctrl,
                                   VulkanMotionMetric::Sad,integer_candidates.data(),integer_candidates.size(),got,&error)) return 6;
        for(size_t i=0;i<integer_mvs.size();++i)
            if(got[i]!=integer_sampled_sad(cur,ref,5,7,16,16,integer_mvs[i].xq/4,integer_mvs[i].yq/4,stride)) return 7;
    }
    const auto counters=vk->counters();
    if(counters.dispatches<5 || counters.candidates==0 || counters.frame_uploads<2) return 8;

    // Exercise the AUTO startup benchmark against the exact CPU motion-cost
    // implementation.  The winning backend is hardware-dependent, so only
    // require valid measurements and that benchmark traffic is removed from
    // the public runtime counters afterward.
    const auto bench=libvc1::benchmark_vulkan_motion_compute(cfg,*vk,8);
    if(!bench.ran || bench.sad_min_batch<0 || bench.satd_min_batch<0) return 9;
    if(bench.sad_min_batch>0 &&
       (!(bench.cpu_sad_candidates_per_second>0.0) || !(bench.vulkan_sad_candidates_per_second>0.0))) return 10;
    const auto reset=vk->counters();
    if(reset.dispatches!=0 || reset.candidates!=0 || reset.frame_uploads!=0 || reset.submit_wait_nanoseconds!=0) return 11;
    std::cout<<"Vulkan motion-cost parity/benchmark ok on "<<vk->device_name()
             <<" (SAD crossover="<<(bench.sad_min_batch?std::to_string(bench.sad_min_batch):std::string("cpu"))
             <<", SATD="<<(bench.satd_min_batch?"vulkan":"cpu")<<")\n";
    return 0;
}
