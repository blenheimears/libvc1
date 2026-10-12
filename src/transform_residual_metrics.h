#pragma once

// Exact integer residual metrics for the throughput preset's transform chooser.
// The SIMD kernels deliberately use the same clipping and accumulator semantics
// as the scalar reference. Only complete 16x16 luma blocks take the SIMD path;
// partial macroblocks retain the reference loop, with no out-of-frame reads.
#include <cstdint>
#include <cstdlib>
#include "encoder_internal.h"

#if defined(__SSE2__) && (defined(__x86_64__) || defined(_M_X64) || defined(__i386__))
#include <emmintrin.h>
#define LIBVC1_TRANSFORM_METRICS_SSE2 1
#endif
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#define LIBVC1_TRANSFORM_METRICS_NEON 1
#endif

namespace libvc1::transform_metrics {

struct Metrics {
    uint64_t sad=0, hedge=0, vedge=0, grad=0;
    bool operator==(const Metrics& other) const {
        return sad==other.sad && hedge==other.hedge && vedge==other.vedge && grad==other.grad;
    }
};

inline Metrics scalar(const uint8_t* src,const uint8_t* pred,int stride,
                      int height,int mx,int my) {
    Metrics m{};
    for (int py=0;py<16 && my*16+py<height;++py) {
        for (int px=0;px<16 && mx*16+px<stride;++px) {
            const int x=mx*16+px,y=my*16+py;
            const size_t o=static_cast<size_t>(y)*stride+x;
            const int r=static_cast<int>(src[o])-static_cast<int>(pred[o]);
            m.sad+=static_cast<uint64_t>(std::abs(r));
            if ((py&7)==4 && py>0) {
                const size_t u=static_cast<size_t>(y-1)*stride+x;
                const int rp=static_cast<int>(src[u])-static_cast<int>(pred[u]);
                m.hedge+=static_cast<uint64_t>(std::abs(r-rp));
            }
            if ((px&7)==4 && px>0) {
                const size_t l=o-1;
                const int rp=static_cast<int>(src[l])-static_cast<int>(pred[l]);
                m.vedge+=static_cast<uint64_t>(std::abs(r-rp));
            }
            if (px>0) {
                const size_t l=o-1;
                const int rp=static_cast<int>(src[l])-static_cast<int>(pred[l]);
                m.grad+=static_cast<uint64_t>(std::abs(r-rp));
            }
            if (py>0) {
                const size_t u=o-stride;
                const int rp=static_cast<int>(src[u])-static_cast<int>(pred[u]);
                m.grad+=static_cast<uint64_t>(std::abs(r-rp));
            }
        }
    }
    return m;
}

#if defined(LIBVC1_TRANSFORM_METRICS_SSE2)
inline __m128i abs16(__m128i x) {
    const __m128i sign=_mm_srai_epi16(x,15);
    return _mm_sub_epi16(_mm_xor_si128(x,sign),sign);
}
inline uint64_t sum4(__m128i x) {
    alignas(16) uint32_t lane[4];
    _mm_storeu_si128(reinterpret_cast<__m128i*>(lane),x);
    return static_cast<uint64_t>(lane[0])+lane[1]+lane[2]+lane[3];
}
inline Metrics sse2(const uint8_t* src,const uint8_t* pred,int stride) {
    Metrics m{};
    const __m128i zero=_mm_setzero_si128();
    const __m128i ones=_mm_set1_epi16(1);
    const __m128i mask_first=_mm_setr_epi16(0,-1,-1,-1,-1,-1,-1,-1);
    __m128i sad_acc=zero,grad_acc=zero,hedge_acc=zero;
    __m128i prev_lo=zero,prev_hi=zero;
    for (int y=0;y<16;++y) {
        const __m128i s=_mm_loadu_si128(reinterpret_cast<const __m128i*>(src+static_cast<size_t>(y)*stride));
        const __m128i p=_mm_loadu_si128(reinterpret_cast<const __m128i*>(pred+static_cast<size_t>(y)*stride));
        const __m128i lo=_mm_sub_epi16(_mm_unpacklo_epi8(s,zero),_mm_unpacklo_epi8(p,zero));
        const __m128i hi=_mm_sub_epi16(_mm_unpackhi_epi8(s,zero),_mm_unpackhi_epi8(p,zero));
        sad_acc=_mm_add_epi64(sad_acc,_mm_sad_epu8(s,p));
        // Cross-register carry is the residual at x=7, needed for x=8.
        const __m128i left_lo=_mm_slli_si128(lo,2);
        const __m128i left_hi=_mm_or_si128(_mm_slli_si128(hi,2),_mm_srli_si128(lo,14));
        const __m128i h0=_mm_and_si128(abs16(_mm_sub_epi16(lo,left_lo)),mask_first);
        const __m128i h1=abs16(_mm_sub_epi16(hi,left_hi));
        grad_acc=_mm_add_epi32(grad_acc,_mm_add_epi32(_mm_madd_epi16(h0,ones),_mm_madd_epi16(h1,ones)));
        m.vedge+=static_cast<uint64_t>(_mm_extract_epi16(h0,4))+_mm_extract_epi16(h1,4);
        if (y>0) {
            const __m128i v0=abs16(_mm_sub_epi16(lo,prev_lo));
            const __m128i v1=abs16(_mm_sub_epi16(hi,prev_hi));
            const __m128i vsum=_mm_add_epi32(_mm_madd_epi16(v0,ones),_mm_madd_epi16(v1,ones));
            grad_acc=_mm_add_epi32(grad_acc,vsum);
            if (y==4 || y==12) hedge_acc=_mm_add_epi32(hedge_acc,vsum);
        }
        prev_lo=lo;
        prev_hi=hi;
    }
    alignas(16) uint64_t sad_lane[2];
    _mm_storeu_si128(reinterpret_cast<__m128i*>(sad_lane),sad_acc);
    m.sad=sad_lane[0]+sad_lane[1];
    m.grad=sum4(grad_acc);
    m.hedge=sum4(hedge_acc);
    return m;
}
#endif

#if defined(LIBVC1_TRANSFORM_METRICS_NEON)
inline Metrics neon(const uint8_t* src,const uint8_t* pred,int stride) {
    Metrics m{};
    uint32x4_t sad_acc=vdupq_n_u32(0),grad_acc=vdupq_n_u32(0),hedge_acc=vdupq_n_u32(0);
    int16x8_t prev_lo=vdupq_n_s16(0),prev_hi=vdupq_n_s16(0);
    for (int y=0;y<16;++y) {
        const uint8x16_t s=vld1q_u8(src+static_cast<size_t>(y)*stride);
        const uint8x16_t p=vld1q_u8(pred+static_cast<size_t>(y)*stride);
        const int16x8_t lo=vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(s),vget_low_u8(p)));
        const int16x8_t hi=vreinterpretq_s16_u16(vsubl_u8(vget_high_u8(s),vget_high_u8(p)));
        sad_acc=vpadalq_u16(sad_acc,vreinterpretq_u16_s16(vabsq_s16(lo)));
        sad_acc=vpadalq_u16(sad_acc,vreinterpretq_u16_s16(vabsq_s16(hi)));
        const int16x8_t left_lo=vextq_s16(vdupq_n_s16(0),lo,7);
        const int16x8_t left_hi=vextq_s16(lo,hi,7);
        uint16x8_t h0=vreinterpretq_u16_s16(vabsq_s16(vsubq_s16(lo,left_lo)));
        h0=vsetq_lane_u16(0,h0,0); // x=0 has no left neighbor within the macroblock.
        const uint16x8_t h1=vreinterpretq_u16_s16(vabsq_s16(vsubq_s16(hi,left_hi)));
        grad_acc=vpadalq_u16(grad_acc,h0);
        grad_acc=vpadalq_u16(grad_acc,h1);
        m.vedge+=static_cast<uint64_t>(vgetq_lane_u16(h0,4))+vgetq_lane_u16(h1,4);
        if (y>0) {
            const uint16x8_t v0=vreinterpretq_u16_s16(vabsq_s16(vsubq_s16(lo,prev_lo)));
            const uint16x8_t v1=vreinterpretq_u16_s16(vabsq_s16(vsubq_s16(hi,prev_hi)));
            grad_acc=vpadalq_u16(grad_acc,v0);
            grad_acc=vpadalq_u16(grad_acc,v1);
            if (y==4 || y==12) {
                hedge_acc=vpadalq_u16(hedge_acc,v0);
                hedge_acc=vpadalq_u16(hedge_acc,v1);
            }
        }
        prev_lo=lo;
        prev_hi=hi;
    }
    m.sad=vaddvq_u32(sad_acc);
    m.grad=vaddvq_u32(grad_acc);
    m.hedge=vaddvq_u32(hedge_acc);
    return m;
}
#endif

inline Metrics calculate(const uint8_t* src,const uint8_t* pred,int stride,
                         int height,int mx,int my,SimdTier selected) {
    if (selected!=SimdTier::None && mx*16+16<=stride && my*16+16<=height) {
        const size_t off=static_cast<size_t>(my*16)*stride+mx*16;
#if defined(LIBVC1_TRANSFORM_METRICS_SSE2)
        if (selected!=SimdTier::AArch64Neon && selected!=SimdTier::AArch64Dotprod &&
            selected!=SimdTier::AArch64Rdm) return sse2(src+off,pred+off,stride);
#endif
#if defined(LIBVC1_TRANSFORM_METRICS_NEON)
        if (selected==SimdTier::AArch64Neon || selected==SimdTier::AArch64Dotprod ||
            selected==SimdTier::AArch64Rdm) return neon(src+off,pred+off,stride);
#endif
    }
    return scalar(src,pred,stride,height,mx,my);
}
} // namespace libvc1::transform_metrics
