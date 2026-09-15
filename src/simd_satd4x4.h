#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <emmintrin.h>
#if defined(__SSSE3__)
#include <tmmintrin.h>
#endif
#if defined(__SSE4_1__) || defined(__AVX2__)
#include <smmintrin.h>
#endif
#if defined(__AVX2__)
#include <immintrin.h>
#endif

namespace libvc1::simd::detail {

static inline __m128i satd_hadamard4_epi16(__m128i v) {
    const __m128i rev=_mm_shufflelo_epi16(v,_MM_SHUFFLE(0,1,2,3));
    const __m128i sum=_mm_add_epi16(v,rev);
    const __m128i dif=_mm_sub_epi16(v,rev);
    const __m128i a0=_mm_shufflelo_epi16(sum,_MM_SHUFFLE(0,0,0,0));
    const __m128i a1=_mm_shufflelo_epi16(sum,_MM_SHUFFLE(1,1,1,1));
    const __m128i a3=_mm_shufflelo_epi16(dif,_MM_SHUFFLE(0,0,0,0));
    const __m128i a2=_mm_shufflelo_epi16(dif,_MM_SHUFFLE(1,1,1,1));
    const __m128i y0=_mm_add_epi16(a0,a1);
    const __m128i y1=_mm_add_epi16(a3,a2);
    const __m128i y2=_mm_sub_epi16(a0,a1);
    const __m128i y3=_mm_sub_epi16(a3,a2);
    const __m128i p01=_mm_unpacklo_epi16(y0,y1);
    const __m128i p23=_mm_unpacklo_epi16(y2,y3);
    return _mm_unpacklo_epi32(p01,p23);
}

static inline __m128i satd_residual4_epi16(const uint8_t* cur,const uint8_t* pred) {
    uint32_t ca=0,pa=0;
    std::memcpy(&ca,cur,sizeof(ca));
    std::memcpy(&pa,pred,sizeof(pa));
    const __m128i cb=_mm_cvtsi32_si128(static_cast<int>(ca));
    const __m128i pb=_mm_cvtsi32_si128(static_cast<int>(pa));
#if defined(__SSE4_1__)
    return _mm_sub_epi16(_mm_cvtepu8_epi16(cb),_mm_cvtepu8_epi16(pb));
#else
    const __m128i z=_mm_setzero_si128();
    return _mm_sub_epi16(_mm_unpacklo_epi8(cb,z),_mm_unpacklo_epi8(pb,z));
#endif
}

#if defined(__AVX2__)
static inline void satd_residual4_pair_avx2(const uint8_t* cur0,const uint8_t* cur1,
                                             const uint8_t* pred0,const uint8_t* pred1,
                                             __m128i& r0,__m128i& r1) {
    uint32_t c0=0,c1=0,p0=0,p1=0;
    std::memcpy(&c0,cur0,sizeof(c0)); std::memcpy(&c1,cur1,sizeof(c1));
    std::memcpy(&p0,pred0,sizeof(p0)); std::memcpy(&p1,pred1,sizeof(p1));
    const uint64_t cq=static_cast<uint64_t>(c0)|(static_cast<uint64_t>(c1)<<32);
    const uint64_t pq=static_cast<uint64_t>(p0)|(static_cast<uint64_t>(p1)<<32);
    const __m128i cb=_mm_cvtsi64_si128(static_cast<long long>(cq));
    const __m128i pb=_mm_cvtsi64_si128(static_cast<long long>(pq));
    const __m256i d=_mm256_sub_epi16(_mm256_cvtepu8_epi16(cb),_mm256_cvtepu8_epi16(pb));
    const __m128i lo=_mm256_castsi256_si128(d);
    r0=lo;
    r1=_mm_srli_si128(lo,8);
}
#endif

static inline __m128i satd_abs_epi16(__m128i v) {
#if defined(__SSSE3__)
    return _mm_abs_epi16(v);
#else
    const __m128i sign=_mm_srai_epi16(v,15);
    return _mm_sub_epi16(_mm_xor_si128(v,sign),sign);
#endif
}

static inline uint64_t satd4x4_target(const uint8_t* cur,int cur_stride,
                                      const uint8_t* pred,int pred_stride) {
    __m128i r0,r1,r2,r3;
#if defined(__AVX2__)
    satd_residual4_pair_avx2(cur+0*cur_stride,cur+1*cur_stride,
                             pred+0*pred_stride,pred+1*pred_stride,r0,r1);
    satd_residual4_pair_avx2(cur+2*cur_stride,cur+3*cur_stride,
                             pred+2*pred_stride,pred+3*pred_stride,r2,r3);
#else
    r0=satd_residual4_epi16(cur+0*cur_stride,pred+0*pred_stride);
    r1=satd_residual4_epi16(cur+1*cur_stride,pred+1*pred_stride);
    r2=satd_residual4_epi16(cur+2*cur_stride,pred+2*pred_stride);
    r3=satd_residual4_epi16(cur+3*cur_stride,pred+3*pred_stride);
#endif
    r0=satd_hadamard4_epi16(r0); r1=satd_hadamard4_epi16(r1);
    r2=satd_hadamard4_epi16(r2); r3=satd_hadamard4_epi16(r3);
    const __m128i a0=_mm_add_epi16(r0,r3),a1=_mm_add_epi16(r1,r2);
    const __m128i a2=_mm_sub_epi16(r1,r2),a3=_mm_sub_epi16(r0,r3);
    r0=_mm_add_epi16(a0,a1); r1=_mm_add_epi16(a3,a2);
    r2=_mm_sub_epi16(a0,a1); r3=_mm_sub_epi16(a3,a2);
    const __m128i ones=_mm_set1_epi16(1);
    __m128i sums=_mm_madd_epi16(satd_abs_epi16(r0),ones);
    sums=_mm_add_epi32(sums,_mm_madd_epi16(satd_abs_epi16(r1),ones));
    sums=_mm_add_epi32(sums,_mm_madd_epi16(satd_abs_epi16(r2),ones));
    sums=_mm_add_epi32(sums,_mm_madd_epi16(satd_abs_epi16(r3),ones));
#if defined(__SSSE3__)
    sums=_mm_hadd_epi32(sums,sums);
#else
    sums=_mm_add_epi32(sums,_mm_srli_si128(sums,4));
#endif
    const uint64_t tile=static_cast<uint32_t>(_mm_cvtsi128_si32(sums));
    return (tile+1)>>1;
}

} // namespace libvc1::simd::detail
