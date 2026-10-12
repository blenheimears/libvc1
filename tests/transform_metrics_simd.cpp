#include "transform_residual_metrics.h"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <random>
#include <vector>

int main() {
    using libvc1::SimdTier;
    using namespace libvc1::transform_metrics;
    std::mt19937 rng(0x714a551u);
    // The zero/full-scale patterns stress differences up to 510 in the
    // residual gradient; odd widths and right/bottom clips must not overread.
    for (int trial=0;trial<500;++trial) {
        const int width=(trial%9==0)?80:((trial%9==1)?50:((trial%9==2)?18:64));
        const int height=(trial%7==0)?50:((trial%7==1)?18:48);
        std::vector<uint8_t> src(static_cast<size_t>(width)*height),pred(src.size());
        for (size_t n=0;n<src.size();++n) {
            switch (trial%6) {
            case 0: src[n]=static_cast<uint8_t>(rng()&255);pred[n]=static_cast<uint8_t>(rng()&255);break;
            case 1: src[n]=255;pred[n]=0;break;
            case 2: src[n]=0;pred[n]=255;break;
            case 3: src[n]=(n/width)&1 ? 255:0;pred[n]=(n%width)&1 ? 255:0;break;
            case 4: src[n]=pred[n]=static_cast<uint8_t>(rng()&255);break;
            default: src[n]=static_cast<uint8_t>(n&255);pred[n]=static_cast<uint8_t>((n*7)&255);break;
            }
        }
        for (int my=0;my<(height+15)/16;++my) for (int mx=0;mx<(width+15)/16;++mx) {
            const auto want=scalar(src.data(),pred.data(),width,height,mx,my);
            const auto disabled=calculate(src.data(),pred.data(),width,height,mx,my,SimdTier::None);
            if (!(want==disabled)) return 1;
#if defined(LIBVC1_TRANSFORM_METRICS_SSE2)
            const auto got=calculate(src.data(),pred.data(),width,height,mx,my,SimdTier::X86V1);
            if (!(want==got)) {
                std::cerr<<"SSE2 residual metric mismatch trial="<<trial<<" mb="<<mx<<","<<my<<"\n";
                return 2;
            }
#endif
#if defined(LIBVC1_TRANSFORM_METRICS_NEON)
            const auto got_neon=calculate(src.data(),pred.data(),width,height,mx,my,SimdTier::AArch64Neon);
            if (!(want==got_neon)) {
                std::cerr<<"NEON residual metric mismatch trial="<<trial<<" mb="<<mx<<","<<my<<"\n";
                return 3;
            }
#endif
        }
    }
    std::cout<<"transform heuristic SIMD/scalar parity: PASS (500 trials; full and cropped macroblocks)\n";
    return 0;
}
