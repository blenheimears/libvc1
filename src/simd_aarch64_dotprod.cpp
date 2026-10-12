// Armv8.2-A +dotprod (ASIMDDP). Built as a separate ISA object: callers must
// check HWCAP_ASIMDDP before entering any function in this translation unit.
// Four-tap VC-1 integer filters are expressed as centered signed-byte SDOTs.
// Unaligned 8-byte loads deliberately preserve the NEON edge-access contract.
#include "simd.h"
#include <arm_neon.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace libvc1::simd {
namespace {
inline int clamp8(int x){return std::clamp(x,0,255);}
inline int asr(int x,int n){return x>=0?x>>n:-int((uint64_t(-int64_t(x))+((1ull<<n)-1))>>n);}
inline int filter(int a,int b,int c,int d,int m){
    if(m==1)return -4*a+53*b+18*c-3*d;
    if(m==2)return -a+9*b+9*c-d;
    if(m==3)return -3*a+18*b+53*c-4*d;
    return b;
}
inline int qpel_scalar(const uint8_t* p,int s,int hm,int vm,bool rnd){
    if(!hm&&!vm)return p[0];
    if(hm&&vm){static constexpr int shifts[4]={0,5,1,5};const int sh=(shifts[hm]+shifts[vm])>>1;
        const int bias=(1<<(sh-1))+int(rnd)-1;int t[4];
        for(int k=-1;k<=2;++k)t[k+1]=asr(filter(p[k-s],p[k],p[k+s],p[k+2*s],vm)+bias,sh);
        return clamp8(asr(filter(t[0],t[1],t[2],t[3],hm)+64-int(rnd),7));
    }
    if(vm)return clamp8(asr(filter(p[-s],p[0],p[s],p[2*s],vm)+(vm==2?7+int(rnd):31+int(rnd)),vm==2?4:6));
    return clamp8(asr(filter(p[-1],p[0],p[1],p[2],hm)+(hm==2?8-int(rnd):32-int(rnd)),hm==2?4:6));
}
inline int chroma_scalar(const uint8_t* p,int s,int fx,int fy,bool rnd){
    if(!fx&&!fy)return p[0];
    const int bias=rnd?28:32;
    if(!fy)return (int(p[0])*(8-fx)*8+int(p[1])*fx*8+bias)>>6;
    if(!fx)return (int(p[0])*(8-fy)*8+int(p[s])*fy*8+bias)>>6;
    return (int(p[0])*(8-fx)*(8-fy)+int(p[1])*fx*(8-fy)+int(p[s])*(8-fx)*fy+int(p[s+1])*fx*fy+bias)>>6;
}
inline uint32_t pack(int a,int b,int c,int d){
    return uint32_t(uint8_t(a)) | (uint32_t(uint8_t(b))<<8) |
           (uint32_t(uint8_t(c))<<16) | (uint32_t(uint8_t(d))<<24);
}
// Create eight independent four-byte dot-product input groups using strictly
// 8-byte loads. XOR 0x80 changes an unsigned byte u into signed (u-128).
// Coefficients have sum 16 or 64; preloading 128*sum restores exact pixels.
// Build groups once per eight outputs. The shared helper also lets chroma
// use UDOT directly: its four coefficients are nonnegative, so an XOR to
// center unsigned bytes and a 128*coefficient correction are unnecessary.
struct DotTaps { uint8x16_t lo, hi; };
inline DotTaps interleave8(uint8x8_t a,uint8x8_t b,uint8x8_t c,uint8x8_t d){
    const uint8x16_t ab=vcombine_u8(vzip1_u8(a,b),vzip2_u8(a,b));
    const uint8x16_t cd=vcombine_u8(vzip1_u8(c,d),vzip2_u8(c,d));
    return {vreinterpretq_u8_u16(vzip1q_u16(vreinterpretq_u16_u8(ab),vreinterpretq_u16_u8(cd))),
            vreinterpretq_u8_u16(vzip2q_u16(vreinterpretq_u16_u8(ab),vreinterpretq_u16_u8(cd)))};
}
inline int32x4x2_t signed_dot8(DotTaps taps,uint32_t coeff,int coeff_sum){
    const int8x16_t k=vreinterpretq_s8_u32(vdupq_n_u32(coeff));
    const uint8x16_t sign=vdupq_n_u8(128);
    const int32x4_t base=vdupq_n_s32(128*coeff_sum);
    return {vdotq_s32(base,vreinterpretq_s8_u8(veorq_u8(taps.lo,sign)),k),
            vdotq_s32(base,vreinterpretq_s8_u8(veorq_u8(taps.hi,sign)),k)};
}
inline int32x4x2_t dot8(uint8x8_t a,uint8x8_t b,uint8x8_t c,uint8x8_t d,uint32_t coeff,int coeff_sum){
    return signed_dot8(interleave8(a,b,c,d),coeff,coeff_sum);
}
inline int32x4x2_t unsigned_dot8(uint8x8_t a,uint8x8_t b,uint8x8_t c,uint8x8_t d,uint32_t coeff){
    const DotTaps t=interleave8(a,b,c,d);
    const uint8x16_t k=vreinterpretq_u8_u32(vdupq_n_u32(coeff));
    const uint32x4_t zero=vdupq_n_u32(0);
    const uint32x4_t lo=vdotq_u32(zero,t.lo,k);
    const uint32x4_t hi=vdotq_u32(zero,t.hi,k);
    return {vreinterpretq_s32_u32(lo),vreinterpretq_s32_u32(hi)};
}
// Gather consecutive horizontal four-tap windows using two *bounded* 8-byte
// loads and two TBLs instead of four loads, four byte ZIPs, two 16-bit ZIPs.
// First load covers x-1..x+6; second covers x+2..x+9. These are exactly
// within the original 0.2.55 access envelope (including the right edge).
// Repeated positions 13..15 refer to x+7..x+9 in the second load.
alignas(16) static constexpr uint8_t kHorizontalLo[16]={
    0,1,2,3, 1,2,3,4, 2,3,4,5, 3,4,5,6};
alignas(16) static constexpr uint8_t kHorizontalHi[16]={
    4,5,6,7, 5,6,7,13, 6,7,13,14, 7,13,14,15};
inline DotTaps horizontal_taps(const uint8_t* p){
    const uint8x16_t bytes=vcombine_u8(vld1_u8(p-1),vld1_u8(p+2));
    return {vqtbl1q_u8(bytes,vld1q_u8(kHorizontalLo)),
            vqtbl1q_u8(bytes,vld1q_u8(kHorizontalHi))};
}
inline int32x4x2_t filter_vertical(const uint8_t* p,int s,int mode){
    const int a=mode==1?-4:mode==2?-1:-3;
    const int b=mode==1?53:mode==2?9:18;
    const int c=mode==1?18:mode==2?9:53;
    const int d=mode==1?-3:mode==2?-1:-4;
    return dot8(vld1_u8(p-s),vld1_u8(p),vld1_u8(p+s),vld1_u8(p+2*s),pack(a,b,c,d),mode==2?16:64);
}
inline int32x4x2_t filter_horizontal(const uint8_t* p,int mode){
    const int a=mode==1?-4:mode==2?-1:-3;
    const int b=mode==1?53:mode==2?9:18;
    const int c=mode==1?18:mode==2?9:53;
    const int d=mode==1?-3:mode==2?-1:-4;
    return signed_dot8(horizontal_taps(p),pack(a,b,c,d),mode==2?16:64);
}
inline int16x8_t narrow_shift(int32x4x2_t v,int bias,int shift){
    const int32x4_t add=vdupq_n_s32(bias);
    const int32x4_t sh=vdupq_n_s32(-shift);
    return vcombine_s16(vmovn_s32(vshlq_s32(vaddq_s32(v.val[0],add),sh)),
                        vmovn_s32(vshlq_s32(vaddq_s32(v.val[1],add),sh)));
}
inline int32x4_t filter32(int32x4_t a,int32x4_t b,int32x4_t c,int32x4_t d,int mode){
    if(mode==1){int32x4_t v=vmulq_n_s32(a,-4);v=vmlaq_n_s32(v,b,53);v=vmlaq_n_s32(v,c,18);return vmlaq_n_s32(v,d,-3);}
    if(mode==2){int32x4_t v=vnegq_s32(a);v=vmlaq_n_s32(v,b,9);v=vmlaq_n_s32(v,c,9);return vsubq_s32(v,d);}
    int32x4_t v=vmulq_n_s32(a,-3);v=vmlaq_n_s32(v,b,18);v=vmlaq_n_s32(v,c,53);return vmlaq_n_s32(v,d,-4);
}
inline uint8x8_t qpel8(const uint8_t* p,int s,int hm,int vm,bool rnd){
    if(!hm&&!vm)return vld1_u8(p);
    if(!hm || !vm){
        const int mode=hm?hm:vm;
        const int bias=hm?(mode==2?8-int(rnd):32-int(rnd)):(mode==2?7+int(rnd):31+int(rnd));
        const int16x8_t value=narrow_shift(hm?filter_horizontal(p,mode):filter_vertical(p,s,mode),bias,mode==2?4:6);
        return vqmovun_s16(value);
    }
    constexpr int shifts[4]={0,5,1,5};
    const int sh=(shifts[hm]+shifts[vm])>>1;
    const int bias=(1<<(sh-1))+int(rnd)-1;
    // The four horizontal outputs overlap heavily: the old kernel evaluated
    // 32 vertical samples for only 11 distinct positions. Two bounded eight-
    // lane dot products cover x-1..x+9; the second store overlaps five lanes.
    // No 16-byte speculative load is used at the right edge.
    alignas(16) int16_t vertical[16];
    vst1q_s16(vertical, narrow_shift(filter_vertical(p-1,s,vm),bias,sh));
    vst1q_s16(vertical+3, narrow_shift(filter_vertical(p+2,s,vm),bias,sh));
    const int16x8_t t0=vld1q_s16(vertical);
    const int16x8_t t1=vld1q_s16(vertical+1);
    const int16x8_t t2=vld1q_s16(vertical+2);
    const int16x8_t t3=vld1q_s16(vertical+3);
    const int32x4_t lo=filter32(vmovl_s16(vget_low_s16(t0)),vmovl_s16(vget_low_s16(t1)),vmovl_s16(vget_low_s16(t2)),vmovl_s16(vget_low_s16(t3)),hm);
    const int32x4_t hi=filter32(vmovl_s16(vget_high_s16(t0)),vmovl_s16(vget_high_s16(t1)),vmovl_s16(vget_high_s16(t2)),vmovl_s16(vget_high_s16(t3)),hm);
    const int32x4_t add=vdupq_n_s32(64-int(rnd));
    return vqmovun_s16(vcombine_s16(vqmovn_s32(vshrq_n_s32(vaddq_s32(lo,add),7)),
                                      vqmovn_s32(vshrq_n_s32(vaddq_s32(hi,add),7))));
}
// Hoist coefficient packing from the pixel loop. A 16-pixel UDOT path
// processes four independent 4-pixel groups with four vector dot products;
// the old two 8-pixel iterations needed twelve ZIP operations in total.
struct ChromaParams {
    int fx,fy,bias;
    uint8x16_t coefficients;
};
inline ChromaParams chroma_params(int fx,int fy,bool rnd){
    return {fx,fy,rnd?28:32,
            vreinterpretq_u8_u32(vdupq_n_u32(pack((8-fx)*(8-fy),fx*(8-fy),(8-fx)*fy,fx*fy)))};
}
inline uint8x8_t chroma_narrow(int32x4_t lo,int32x4_t hi,int bias){
    const int32x4_t add=vdupq_n_s32(bias);
    const int16x8_t values=vcombine_s16(
        vmovn_s32(vshrq_n_s32(vaddq_s32(lo,add),6)),
        vmovn_s32(vshrq_n_s32(vaddq_s32(hi,add),6)));
    return vqmovun_s16(values);
}
inline uint8x8_t chroma8_four(const uint8_t* p,int s,ChromaParams v){
    const DotTaps taps=interleave8(vld1_u8(p),vld1_u8(p+1),
                                   vld1_u8(p+s),vld1_u8(p+s+1));
    const uint32x4_t zero=vdupq_n_u32(0);
    return chroma_narrow(vreinterpretq_s32_u32(vdotq_u32(zero,taps.lo,v.coefficients)),
                         vreinterpretq_s32_u32(vdotq_u32(zero,taps.hi,v.coefficients)),v.bias);
}
inline uint8x16_t chroma16_four(const uint8_t* p,int s,ChromaParams v){
    const uint8x16_t a=vld1q_u8(p),b=vld1q_u8(p+1);
    const uint8x16_t c=vld1q_u8(p+s),d=vld1q_u8(p+s+1);
    const uint8x16_t ab0=vzip1q_u8(a,b),ab1=vzip2q_u8(a,b);
    const uint8x16_t cd0=vzip1q_u8(c,d),cd1=vzip2q_u8(c,d);
    const uint32x4_t zero=vdupq_n_u32(0);
    const uint8x8_t lo=chroma_narrow(
        vreinterpretq_s32_u32(vdotq_u32(zero,vreinterpretq_u8_u16(
            vzip1q_u16(vreinterpretq_u16_u8(ab0),vreinterpretq_u16_u8(cd0))),v.coefficients)),
        vreinterpretq_s32_u32(vdotq_u32(zero,vreinterpretq_u8_u16(
            vzip2q_u16(vreinterpretq_u16_u8(ab0),vreinterpretq_u16_u8(cd0))),v.coefficients)),v.bias);
    const uint8x8_t hi=chroma_narrow(
        vreinterpretq_s32_u32(vdotq_u32(zero,vreinterpretq_u8_u16(
            vzip1q_u16(vreinterpretq_u16_u8(ab1),vreinterpretq_u16_u8(cd1))),v.coefficients)),
        vreinterpretq_s32_u32(vdotq_u32(zero,vreinterpretq_u8_u16(
            vzip2q_u16(vreinterpretq_u16_u8(ab1),vreinterpretq_u16_u8(cd1))),v.coefficients)),v.bias);
    return vcombine_u8(lo,hi);
}
inline uint8x8_t chroma8_axis(const uint8_t* p,int s,int fx,int fy,bool rnd){
    const int fraction=fx?fx:fy;
    const uint8x8_t a=vld1_u8(p),b=vld1_u8(p+(fx?1:s));
    // The bias 28/32 on the 64-denominator expression is 3/4 on 8.
    uint16x8_t v=vmulq_n_u16(vmovl_u8(a),8-fraction);
    v=vmlaq_n_u16(v,vmovl_u8(b),fraction);
    return vmovn_u16(vshrq_n_u16(vaddq_u16(v,vdupq_n_u16(rnd?3:4)),3));
}
inline uint8x8_t chroma8_half(const uint8_t* p,int s,bool rnd){
    // Exact when fx=fy=4: ((a+b+c+d)*16 + bias)>>6.
    uint16x8_t total=vaddl_u8(vld1_u8(p),vld1_u8(p+1));
    total=vaddw_u8(total,vld1_u8(p+s));
    total=vaddw_u8(total,vld1_u8(p+s+1));
    return vmovn_u16(vshrq_n_u16(vaddq_u16(total,vdupq_n_u16(rnd?1:2)),2));
}
inline void luma_mc(uint8_t* dst,int ds,const uint8_t* ref,int rs,int w,int h,int hm,int vm,bool rnd,bool bilinear,bool avg){
    // Baseline NEON remains the bilinear implementation: no four-tap dot product
    // is necessary for a 1/2 average. The caller only selects this for qpel.
    for(int y=0;y<h;++y){uint8_t* d=dst+ptrdiff_t(y)*ds;const uint8_t* r=ref+ptrdiff_t(y)*rs;int x=0;
        for(;x+8<=w;x+=8){uint8x8_t p=qpel8(r+x,rs,hm,vm,rnd);if(avg)p=vrhadd_u8(vld1_u8(d+x),p);vst1_u8(d+x,p);}
        for(;x<w;++x){const int p=qpel_scalar(r+x,rs,hm,vm,rnd);d[x]=uint8_t(avg?(int(d[x])+p+1)>>1:p);}
    }
    (void)bilinear;
}
inline void chroma_mc(uint8_t* dst,int ds,const uint8_t* ref,int rs,int w,int h,int fx,int fy,bool rnd,bool avg){
    const ChromaParams params=chroma_params(fx,fy,rnd);
    const bool copy=!fx&&!fy,axis=(fx==0)!=(fy==0),half=fx==4&&fy==4;
    for(int y=0;y<h;++y){
        uint8_t* d=dst+ptrdiff_t(y)*ds;
        const uint8_t* r=ref+ptrdiff_t(y)*rs;
        int x=0;
        if(!copy&&!axis&&!half){
            for(;x+16<=w;x+=16){
                uint8x16_t p=chroma16_four(r+x,rs,params);
                if(avg)p=vrhaddq_u8(vld1q_u8(d+x),p);
                vst1q_u8(d+x,p);
            }
        }
        for(;x+8<=w;x+=8){
            const uint8x8_t p=copy?vld1_u8(r+x):
                 axis?chroma8_axis(r+x,rs,fx,fy,rnd):
                 half?chroma8_half(r+x,rs,rnd):chroma8_four(r+x,rs,params);
            vst1_u8(d+x,avg?vrhadd_u8(vld1_u8(d+x),p):p);
        }
        for(;x<w;++x){
            const int p=chroma_scalar(r+x,rs,fx,fy,rnd);
            d[x]=uint8_t(avg?(int(d[x])+p+1)>>1:p);
        }
    }
}
}
uint64_t block_sad_luma_qpel_aarch64_dotprod(const uint8_t* cur,int cs,const uint8_t* ref,int rs,int w,int h,int hm,int vm,bool rnd){
    uint64_t sum=0;
    for(int y=0;y<h;++y){const uint8_t* c=cur+ptrdiff_t(y)*cs,*r=ref+ptrdiff_t(y)*rs;int x=0;
        for(;x+8<=w;x+=8)sum+=vaddlv_u8(vabd_u8(vld1_u8(c+x),qpel8(r+x,rs,hm,vm,rnd)));
        for(;x<w;++x)sum+=static_cast<unsigned>(std::abs(int(c[x])-qpel_scalar(r+x,rs,hm,vm,rnd)));
    }
    return sum;
}
void luma_mc_block_aarch64_dotprod(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int hm,int vm,bool rnd,bool bilinear){
    if(bilinear){luma_mc_block_aarch64_neon(d,ds,r,rs,w,h,hm,vm,rnd,true);return;}
    luma_mc(d,ds,r,rs,w,h,hm,vm,rnd,false,false);
}
void luma_mc_block_avg_aarch64_dotprod(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int hm,int vm,bool rnd,bool bilinear){
    if(bilinear){luma_mc_block_avg_aarch64_neon(d,ds,r,rs,w,h,hm,vm,rnd,true);return;}
    luma_mc(d,ds,r,rs,w,h,hm,vm,rnd,false,true);
}
void chroma_mc_block_aarch64_dotprod(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int fx,int fy,bool rnd){
    chroma_mc(d,ds,r,rs,w,h,fx,fy,rnd,false);
}
void chroma_mc_block_avg_aarch64_dotprod(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int fx,int fy,bool rnd){
    chroma_mc(d,ds,r,rs,w,h,fx,fy,rnd,true);
}
} // namespace libvc1::simd
