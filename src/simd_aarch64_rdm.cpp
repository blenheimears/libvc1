// Armv8.1-A +rdm (ASIMDRDM). Dispatch checks HWCAP_ASIMDRDM first.
// The exact-integer specialization requires shift divisible by 64:
// SQRDMLAH(shift/64, pixel*64, scale*8) == (pixel*scale+shift+32)>>6.
// All other inputs take the proven baseline NEON implementation.
#include "simd.h"
#include <arm_neon.h>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cstring>
namespace libvc1::simd {
namespace {
// One 16-byte transform, kept inline so that the four independent chunks
// below expose instruction-level parallelism without scalar lane extraction.
inline void rdm16(uint8_t* d,const uint8_t* s,int16x8_t multiplier,int16x8_t offset){
    const uint8x16_t pixels=vld1q_u8(s);
    const int16x8_t lo=vreinterpretq_s16_u16(vshll_n_u8(vget_low_u8(pixels),6));
    const int16x8_t hi=vreinterpretq_s16_u16(vshll_n_u8(vget_high_u8(pixels),6));
    vst1q_u8(d,vcombine_u8(vqmovun_s16(vqrdmlahq_s16(offset,lo,multiplier)),
                            vqmovun_s16(vqrdmlahq_s16(offset,hi,multiplier))));
}
} // namespace
void intensity_map_plane_aarch64_rdm(uint8_t* d,const uint8_t* s,size_t n,int scale,int shift){
    if(scale<0 || scale>95 || shift < -5000 || shift>8000 || (shift%64)!=0){
        intensity_map_plane_aarch64_neon(d,s,n,scale,shift);
        return;
    }
    if(!n)return;
    // Exact common scaling identities. Avoid two widenings and RDM entirely
    // for neutral intensity; memmove also handles overlapping planes safely.
    if(scale==64 && shift==0){if(d!=s)std::memmove(d,s,n);return;}
    if(scale==0){
        const int value=std::clamp((shift+32)>>6,0,255);
        std::memset(d,value,n);
        return;
    }
    // scale=64 maps to saturating byte addition/subtraction by shift/64.
    // This avoids widening, multiplication and narrowing for brightness-only
    // changes while preserving the signed-rounding rule exactly.
    if(scale==64){
        const int delta=shift/64;
        if(delta>=255 || delta<=-255){std::memset(d,delta>0?255:0,n);return;}
        size_t i=0;
        if(delta>0){
            const uint8x16_t add=vdupq_n_u8(static_cast<uint8_t>(delta));
            for(;i+16<=n;i+=16)vst1q_u8(d+i,vqaddq_u8(vld1q_u8(s+i),add));
        }else{
            const uint8x16_t subtract=vdupq_n_u8(static_cast<uint8_t>(-delta));
            for(;i+16<=n;i+=16)vst1q_u8(d+i,vqsubq_u8(vld1q_u8(s+i),subtract));
        }
        for(;i<n;++i)d[i]=static_cast<uint8_t>(std::clamp(int(s[i])+delta,0,255));
        return;
    }
    // Short planes are dominated by setup and packing; use the already
    // verified NEON kernel. AUTO still benchmarks both tiers independently.
    if(n<256){intensity_map_plane_aarch64_neon(d,s,n,scale,shift);return;}
    const int16x8_t multiplier=vdupq_n_s16(static_cast<int16_t>(scale*8));
    const int16x8_t offset=vdupq_n_s16(static_cast<int16_t>(shift/64));
    // Unroll the independent operations four ways to overlap two widening
    // loads, SQRDMLAH and saturation pipelines on wide intensity planes.
    size_t i=0;
    for(;i+64<=n;i+=64){
        rdm16(d+i,s+i,multiplier,offset);
        rdm16(d+i+16,s+i+16,multiplier,offset);
        rdm16(d+i+32,s+i+32,multiplier,offset);
        rdm16(d+i+48,s+i+48,multiplier,offset);
    }
    for(;i+16<=n;i+=16)rdm16(d+i,s+i,multiplier,offset);
    for(;i+8<=n;i+=8){
        const int16x8_t a=vreinterpretq_s16_u16(vshll_n_u8(vld1_u8(s+i),6));
        vst1_u8(d+i,vqmovun_s16(vqrdmlahq_s16(offset,a,multiplier)));
    }
    for(;i<n;++i){
        const int v=(scale*int(s[i])+shift+32)>>6;
        d[i]=static_cast<uint8_t>(v<0?0:v>255?255:v);
    }
}
void intensity_map_chroma_plane_aarch64_rdm(uint8_t* d,const uint8_t* s,size_t n,int scale){
    intensity_map_plane_aarch64_rdm(d,s,n,scale,128*64-128*scale);
}
} // namespace libvc1::simd
