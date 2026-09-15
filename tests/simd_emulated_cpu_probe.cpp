#include "encoder_internal.h"

#include <cstdio>
#include <cstring>

using namespace libvc1;

static SimdTier tier_for(const char* name) {
    if (!std::strcmp(name,"v1")) return SimdTier::X86V1;
    if (!std::strcmp(name,"prescott")) return SimdTier::Prescott;
    if (!std::strcmp(name,"k10")) return SimdTier::K10;
    if (!std::strcmp(name,"conroe")) return SimdTier::Conroe;
    if (!std::strcmp(name,"penryn")) return SimdTier::Penryn;
    if (!std::strcmp(name,"v2")) return SimdTier::X86V2;
    if (!std::strcmp(name,"sandybridge")) return SimdTier::SandyBridge;
    if (!std::strcmp(name,"bulldozer")) return SimdTier::Bulldozer;
    if (!std::strcmp(name,"piledriver")) return SimdTier::Piledriver;
    if (!std::strcmp(name,"avx2-partial")) return SimdTier::Avx2Partial;
    if (!std::strcmp(name,"v3")) return SimdTier::X86V3;
    if (!std::strcmp(name,"v4")) return SimdTier::X86V4;
    return SimdTier::Mixed;
}

static bool unexpectedly_high(const char* name) {
    // These checks deliberately validate libvc1's normal runtime feature
    // detection rather than selecting/benchmarking a kernel.  The emulated
    // CPU is constrained enough that the next incompatible target must remain
    // unavailable, which catches target leakage (e.g. AVX2 in a v2 build).
    if (!std::strcmp(name,"v1")) return simd_target_available(SimdTier::Prescott);
    if (!std::strcmp(name,"prescott")) return simd_target_available(SimdTier::Conroe) || simd_target_available(SimdTier::K10);
    if (!std::strcmp(name,"k10")) return simd_target_available(SimdTier::Conroe) || simd_target_available(SimdTier::Bulldozer) || simd_target_available(SimdTier::X86V2);
    if (!std::strcmp(name,"conroe")) return simd_target_available(SimdTier::Penryn) || simd_target_available(SimdTier::K10);
    if (!std::strcmp(name,"penryn")) return simd_target_available(SimdTier::X86V2);
    if (!std::strcmp(name,"v2")) return simd_target_available(SimdTier::SandyBridge) || simd_target_available(SimdTier::K10) || simd_target_available(SimdTier::X86V3);
    if (!std::strcmp(name,"sandybridge")) return simd_target_available(SimdTier::Avx2Partial) || simd_target_available(SimdTier::X86V3);
    if (!std::strcmp(name,"bulldozer")) return simd_target_available(SimdTier::Piledriver) || simd_target_available(SimdTier::Avx2Partial) || simd_target_available(SimdTier::X86V3);
    if (!std::strcmp(name,"piledriver")) return simd_target_available(SimdTier::Avx2Partial) || simd_target_available(SimdTier::X86V3);
    if (!std::strcmp(name,"avx2-partial")) return simd_target_available(SimdTier::X86V3) || simd_target_available(SimdTier::Bulldozer) || simd_target_available(SimdTier::Piledriver);
    if (!std::strcmp(name,"v3")) return simd_target_available(SimdTier::X86V4);
    return false; // v4 is the highest target currently compiled by libvc1.
}

int main(int argc,char** argv) {
    if (argc!=2) {
        std::fprintf(stderr,"usage: %s TARGET\n",argv[0]);
        return 2;
    }
    const SimdTier expected=tier_for(argv[1]);
    if (expected==SimdTier::Mixed) return 2;
    if (!simd_target_available(expected)) {
        std::fprintf(stderr,"libvc1: emulated CPU does not expose required target %s\n",argv[1]);
        return 78;
    }
    if (unexpectedly_high(argv[1])) {
        std::fprintf(stderr,"libvc1: emulated CPU for %s exposes an incompatible higher/alternate SIMD target\n",argv[1]);
        return 3;
    }
    std::printf("libvc1: emulated CPU detection OK: %s available, incompatible higher target unavailable\n",argv[1]);
    return 0;
}
