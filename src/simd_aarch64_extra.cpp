// Optimized additional encoder primitives for baseline AArch64 Advanced SIMD (NEON).
// Keep VC-1 integer rounding and boundary contracts identical to scalar code.
#include "simd.h"
#include <arm_neon.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace libvc1::simd {
namespace {
inline int clamp8(int x) {return std::clamp(x,0,255);}
inline int asr(int x,int n) {return x>=0 ? x>>n : -int((uint64_t(-int64_t(x))+((1ull<<n)-1))>>n);}
inline int filter(int a,int b,int c,int d,int m){
    if(m==1)return -4*a+53*b+18*c-3*d;
    if(m==2)return -a+9*b+9*c-d;
    if(m==3)return -3*a+18*b+53*c-4*d;
    return b;
}
inline int qpel_scalar(const uint8_t* p,int s,int hm,int vm,bool rnd){
    if(!hm&&!vm)return p[0];
    if(hm&&vm){static constexpr int shifts[4]={0,5,1,5};const int sh=(shifts[hm]+shifts[vm])>>1;
        const int r1=(1<<(sh-1))+(rnd?1:0)-1;int t[4];
        for(int k=-1;k<=2;++k)t[k+1]=asr(filter(p[k-s],p[k],p[k+s],p[k+2*s],vm)+r1,sh);
        return clamp8(asr(filter(t[0],t[1],t[2],t[3],hm)+64-(rnd?1:0),7));
    }
    if(vm)return clamp8(asr(filter(p[-s],p[0],p[s],p[2*s],vm)+(vm==2?7+(rnd?1:0):31+(rnd?1:0)),vm==2?4:6));
    return clamp8(asr(filter(p[-1],p[0],p[1],p[2],hm)+(hm==2?8-(rnd?1:0):32-(rnd?1:0)),hm==2?4:6));
}
inline int bilinear_scalar(const uint8_t* p,int s,bool fx,bool fy,bool rnd){
    if(!fx&&!fy)return p[0];
    if(fx&&fy)return (int(p[0])+p[1]+p[s]+p[s+1]+(rnd?1:2))>>2;
    return (int(p[0])+p[fx?1:s]+(rnd?0:1))>>1;
}
inline int chroma_scalar(const uint8_t* p,int s,int fx,int fy,bool rnd){
    if(!fx&&!fy)return p[0];
    const int bias=rnd?28:32;
    if(!fy)return (int(p[0])*(8-fx)*8+int(p[1])*fx*8+bias)>>6;
    if(!fx)return (int(p[0])*(8-fy)*8+int(p[s])*fy*8+bias)>>6;
    return (int(p[0])*(8-fx)*(8-fy)+int(p[1])*fx*(8-fy)+int(p[s])*(8-fx)*fy+int(p[s+1])*fx*fy+bias)>>6;
}
inline uint8x8_t bilinear8(const uint8_t* p,int s,bool fx,bool fy,bool rnd){
    const uint8x8_t a=vld1_u8(p);
    if(!fx&&!fy)return a;
    // halving-add is the exact single-axis rounding law; it avoids two
    // widenings, a wide add and a narrowing for the common 1D cases.
    if(!fx)return rnd?vhadd_u8(a,vld1_u8(p+s)):vrhadd_u8(a,vld1_u8(p+s));
    if(!fy)return rnd?vhadd_u8(a,vld1_u8(p+1)):vrhadd_u8(a,vld1_u8(p+1));
    uint16x8_t v=vaddq_u16(vmovl_u8(a),vmovl_u8(vld1_u8(p+1)));
    v=vaddq_u16(v,vmovl_u8(vld1_u8(p+s)));
    v=vaddq_u16(v,vmovl_u8(vld1_u8(p+s+1)));
    return vmovn_u16(vshrq_n_u16(vaddq_u16(v,vdupq_n_u16(rnd?1:2)),2));
}
// Fractional chroma uses one multiply-accumulate per needed tap. Integer,
// horizontal-only and vertical-only paths avoid loading unused neighboring
// rows or multiplying by zero. Coefficient sum is exactly 64; 16-bit lanes
// cannot overflow (64 * 255 + 32 < 65536).
inline uint8x8_t chroma8(const uint8_t* p,int s,int fx,int fy,bool rnd){
    const uint8x8_t a=vld1_u8(p);
    if(!fx&&!fy)return a;
    uint16x8_t v;
    if(!fy){
        v=vmulq_n_u16(vmovl_u8(a),static_cast<uint16_t>(8-fx));
        v=vmlaq_n_u16(v,vmovl_u8(vld1_u8(p+1)),static_cast<uint16_t>(fx));
        return vmovn_u16(vshrq_n_u16(vaddq_u16(v,vdupq_n_u16(rnd?3:4)),3));
    }
    if(!fx){
        v=vmulq_n_u16(vmovl_u8(a),static_cast<uint16_t>(8-fy));
        v=vmlaq_n_u16(v,vmovl_u8(vld1_u8(p+s)),static_cast<uint16_t>(fy));
        return vmovn_u16(vshrq_n_u16(vaddq_u16(v,vdupq_n_u16(rnd?3:4)),3));
    }
    v=vmulq_n_u16(vmovl_u8(a),static_cast<uint16_t>((8-fx)*(8-fy)));
    v=vmlaq_n_u16(v,vmovl_u8(vld1_u8(p+1)),static_cast<uint16_t>(fx*(8-fy)));
    v=vmlaq_n_u16(v,vmovl_u8(vld1_u8(p+s)),static_cast<uint16_t>((8-fx)*fy));
    v=vmlaq_n_u16(v,vmovl_u8(vld1_u8(p+s+1)),static_cast<uint16_t>(fx*fy));
    return vmovn_u16(vshrq_n_u16(vaddq_u16(v,vdupq_n_u16(rnd?28:32)),6));
}

// True vector quarter-pel filtering. First-pass tap results fit in signed
// 16-bit lanes (worst case <= 18105); the unclipped second pass widens to
// signed 32-bit before applying the horizontal taps. Never clamp the
// intermediate: the decoder's tap rounding depends on its full signed value.
inline int16x8_t filter16(int16x8_t a,int16x8_t b,int16x8_t c,int16x8_t d,int mode){
    if(mode==1) {
        int16x8_t v=vmulq_n_s16(a,-4);
        v=vmlaq_n_s16(v,b,53);v=vmlaq_n_s16(v,c,18);
        return vmlaq_n_s16(v,d,-3);
    }
    if(mode==2){
        int16x8_t v=vnegq_s16(a);
        v=vmlaq_n_s16(v,b,9);v=vmlaq_n_s16(v,c,9);
        return vsubq_s16(v,d);
    }
    if(mode==3){
        int16x8_t v=vmulq_n_s16(a,-3);
        v=vmlaq_n_s16(v,b,18);v=vmlaq_n_s16(v,c,53);
        return vmlaq_n_s16(v,d,-4);
    }
    return b;
}
inline int16x8_t filter_vertical(const uint8_t* p,int s,int mode){
    return filter16(vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p-s))),
                    vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p))),
                    vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p+s))),
                    vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p+2*s))),mode);
}
inline int16x8_t filter_horizontal(const uint8_t* p,int mode){
    return filter16(vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p-1))),
                    vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p))),
                    vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p+1))),
                    vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p+2))),mode);
}
inline int32x4_t filter32(int32x4_t a,int32x4_t b,int32x4_t c,int32x4_t d,int mode){
    if(mode==1){
        int32x4_t v=vmulq_n_s32(a,-4);
        v=vmlaq_n_s32(v,b,53);v=vmlaq_n_s32(v,c,18);
        return vmlaq_n_s32(v,d,-3);
    }
    if(mode==2){
        int32x4_t v=vnegq_s32(a);
        v=vmlaq_n_s32(v,b,9);v=vmlaq_n_s32(v,c,9);
        return vsubq_s32(v,d);
    }
    if(mode==3){
        int32x4_t v=vmulq_n_s32(a,-3);
        v=vmlaq_n_s32(v,b,18);v=vmlaq_n_s32(v,c,53);
        return vmlaq_n_s32(v,d,-4);
    }
    return b;
}
inline uint8x8_t qpel8(const uint8_t* p,int s,int hm,int vm,bool rnd){
    if(!hm&&!vm)return vld1_u8(p);
    if(!hm || !vm){
        const int mode=hm?hm:vm;
        const int bias=hm?(mode==2?8-int(rnd):32-int(rnd)):
                          (mode==2?7+int(rnd):31+int(rnd));
        const int shift=mode==2?4:6;
        const int16x8_t raw=hm?filter_horizontal(p,mode):filter_vertical(p,s,mode);
        return vqmovun_s16(vshlq_s16(vaddq_s16(raw,vdupq_n_s16(bias)),vdupq_n_s16(-shift)));
    }
    constexpr int shifts[4]={0,5,1,5};
    const int sh=(shifts[hm]+shifts[vm])>>1;
    const int bias=(1<<(sh-1))+int(rnd)-1;
    const int16x8_t t0=vshlq_s16(vaddq_s16(filter_vertical(p-1,s,vm),vdupq_n_s16(bias)),vdupq_n_s16(-sh));
    const int16x8_t t1=vshlq_s16(vaddq_s16(filter_vertical(p,s,vm),vdupq_n_s16(bias)),vdupq_n_s16(-sh));
    const int16x8_t t2=vshlq_s16(vaddq_s16(filter_vertical(p+1,s,vm),vdupq_n_s16(bias)),vdupq_n_s16(-sh));
    const int16x8_t t3=vshlq_s16(vaddq_s16(filter_vertical(p+2,s,vm),vdupq_n_s16(bias)),vdupq_n_s16(-sh));
    const int32x4_t lo=filter32(vmovl_s16(vget_low_s16(t0)),vmovl_s16(vget_low_s16(t1)),
                              vmovl_s16(vget_low_s16(t2)),vmovl_s16(vget_low_s16(t3)),hm);
    const int32x4_t hi=filter32(vmovl_s16(vget_high_s16(t0)),vmovl_s16(vget_high_s16(t1)),
                              vmovl_s16(vget_high_s16(t2)),vmovl_s16(vget_high_s16(t3)),hm);
    return vqmovun_s16(vcombine_s16(
        vqmovn_s32(vshrq_n_s32(vaddq_s32(lo,vdupq_n_s32(64-int(rnd))),7)),
        vqmovn_s32(vshrq_n_s32(vaddq_s32(hi,vdupq_n_s32(64-int(rnd))),7))));
}
inline void inverse4(const int32x4_t s[8],int32x4_t d[8],int bias,int shift){
    const int32x4_t t1=vaddq_s32(vmulq_n_s32(vaddq_s32(s[0],s[4]),12),vdupq_n_s32(bias));
    const int32x4_t t2=vaddq_s32(vmulq_n_s32(vsubq_s32(s[0],s[4]),12),vdupq_n_s32(bias));
    const int32x4_t t3=vaddq_s32(vmulq_n_s32(s[2],16),vmulq_n_s32(s[6],6));
    const int32x4_t t4=vsubq_s32(vmulq_n_s32(s[2],6),vmulq_n_s32(s[6],16));
    const int32x4_t a=vaddq_s32(t1,t3),b=vaddq_s32(t2,t4),c=vsubq_s32(t2,t4),e=vsubq_s32(t1,t3);
    const int32x4_t o0=vaddq_s32(vaddq_s32(vmulq_n_s32(s[1],16),vmulq_n_s32(s[3],15)),vaddq_s32(vmulq_n_s32(s[5],9),vmulq_n_s32(s[7],4)));
    const int32x4_t o1=vsubq_s32(vsubq_s32(vmulq_n_s32(s[1],15),vmulq_n_s32(s[3],4)),vaddq_s32(vmulq_n_s32(s[5],16),vmulq_n_s32(s[7],9)));
    const int32x4_t o2=vaddq_s32(vsubq_s32(vmulq_n_s32(s[1],9),vmulq_n_s32(s[3],16)),vaddq_s32(vmulq_n_s32(s[5],4),vmulq_n_s32(s[7],15)));
    const int32x4_t o3=vsubq_s32(vaddq_s32(vsubq_s32(vmulq_n_s32(s[1],4),vmulq_n_s32(s[3],9)),vmulq_n_s32(s[5],15)),vmulq_n_s32(s[7],16));
    const int32x4_t outs[8]={vaddq_s32(a,o0),vaddq_s32(b,o1),vaddq_s32(c,o2),vaddq_s32(e,o3),vsubq_s32(e,o3),vsubq_s32(c,o2),vsubq_s32(b,o1),vsubq_s32(a,o0)};
    for(int i=0;i<8;++i)d[i]=vshlq_s32(vaddq_s32(outs[i],vdupq_n_s32(bias==64 && i>=4?1:0)),vdupq_n_s32(-shift));
}
} // namespace

void inverse_transform_8x8_aarch64_neon(int* block){
    int tmp[64];
    for(int col=0;col<8;col+=4){
        int32x4_t src[8],out[8];
        for(int y=0;y<8;++y)src[y]=vld1q_s32(block+y*8+col);
        inverse4(src,out,4,3);
        for(int k=0;k<8;++k){int lanes[4];vst1q_s32(lanes,out[k]);for(int lane=0;lane<4;++lane)tmp[(col+lane)*8+k]=lanes[lane];}
    }
    for(int col=0;col<8;col+=4){
        int32x4_t src[8],out[8];
        for(int y=0;y<8;++y)src[y]=vld1q_s32(tmp+y*8+col);
        inverse4(src,out,64,7);
        for(int y=0;y<8;++y)vst1q_s32(block+y*8+col,out[y]);
    }
}

uint64_t block_sad_luma_qpel_aarch64_neon(const uint8_t* cur,int cs,const uint8_t* ref,int rs,int w,int h,int hm,int vm,bool rnd){
    uint64_t sum=0;
    for(int y=0;y<h;++y){const uint8_t* c=cur+ptrdiff_t(y)*cs,*r=ref+ptrdiff_t(y)*rs;int x=0;
        for(;x+8<=w;x+=8)sum+=vaddlv_u8(vabd_u8(vld1_u8(c+x),qpel8(r+x,rs,hm,vm,rnd)));
        for(;x<w;++x)sum+=static_cast<unsigned>(std::abs(int(c[x])-qpel_scalar(r+x,rs,hm,vm,rnd)));
    }return sum;
}
uint64_t block_sad_luma_bilinear_aarch64_neon(const uint8_t* cur,int cs,const uint8_t* ref,int rs,int w,int h,bool fx,bool fy,bool rnd){
    uint64_t sum=0;
    for(int y=0;y<h;++y){const uint8_t* c=cur+ptrdiff_t(y)*cs,*r=ref+ptrdiff_t(y)*rs;int x=0;
        for(;x+8<=w;x+=8)sum+=vaddlv_u8(vabd_u8(vld1_u8(c+x),bilinear8(r+x,rs,fx,fy,rnd)));
        for(;x<w;++x)sum+=static_cast<unsigned>(std::abs(int(c[x])-bilinear_scalar(r+x,rs,fx,fy,rnd)));
    }return sum;
}
static void luma_mc(uint8_t* dst,int ds,const uint8_t* ref,int rs,int w,int h,int hm,int vm,bool rnd,bool bilinear,bool avg){
    for(int y=0;y<h;++y){uint8_t* d=dst+ptrdiff_t(y)*ds;const uint8_t* r=ref+ptrdiff_t(y)*rs;int x=0;
        for(;x+8<=w;x+=8){uint8x8_t p;
            if(bilinear)p=bilinear8(r+x,rs,hm!=0,vm!=0,rnd);
            else p=qpel8(r+x,rs,hm,vm,rnd);
            if(avg)p=vrhadd_u8(vld1_u8(d+x),p);
            vst1_u8(d+x,p);
        }
        for(;x<w;++x){const int p=bilinear?bilinear_scalar(r+x,rs,hm!=0,vm!=0,rnd):qpel_scalar(r+x,rs,hm,vm,rnd);d[x]=uint8_t(avg?(int(d[x])+p+1)>>1:p);}
    }
}
void luma_mc_block_aarch64_neon(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int hm,int vm,bool rnd,bool bilinear){luma_mc(d,ds,r,rs,w,h,hm,vm,rnd,bilinear,false);}
void luma_mc_block_avg_aarch64_neon(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int hm,int vm,bool rnd,bool bilinear){luma_mc(d,ds,r,rs,w,h,hm,vm,rnd,bilinear,true);}
static void chroma_mc(uint8_t* dst,int ds,const uint8_t* ref,int rs,int w,int h,int fx,int fy,bool rnd,bool avg){
    for(int y=0;y<h;++y){uint8_t* d=dst+ptrdiff_t(y)*ds;const uint8_t* r=ref+ptrdiff_t(y)*rs;int x=0;
        for(;x+8<=w;x+=8){uint8x8_t p=chroma8(r+x,rs,fx,fy,rnd);if(avg)p=vrhadd_u8(vld1_u8(d+x),p);vst1_u8(d+x,p);}
        for(;x<w;++x){const int p=chroma_scalar(r+x,rs,fx,fy,rnd);d[x]=uint8_t(avg?(int(d[x])+p+1)>>1:p);}
    }
}
void chroma_mc_block_aarch64_neon(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int fx,int fy,bool rnd){chroma_mc(d,ds,r,rs,w,h,fx,fy,rnd,false);}
void chroma_mc_block_avg_aarch64_neon(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int fx,int fy,bool rnd){chroma_mc(d,ds,r,rs,w,h,fx,fy,rnd,true);}
// 16 pixels per loop. The common fast path is exact: 0 <= scale <= 95,
// -5000 <= shift <= 8000 imply -4968 <= scale*pixel+shift+32 <= 32257,
// so every intermediate fits signed 16 bits. Saturate only AFTER >> 6.
static void intensity(uint8_t* d,const uint8_t* s,size_t n,int scale,int shift){
    size_t i=0;
    if(scale>=0 && scale<=95 && shift>=-5000 && shift<=8000){
        const int16x8_t add=vdupq_n_s16(static_cast<int16_t>(shift+32));
        const uint8_t multiplier=static_cast<uint8_t>(scale);
        for(;i+16<=n;i+=16){
            const uint8x16_t pixels=vld1q_u8(s+i);
            const int16x8_t lo=vreinterpretq_s16_u16(vmull_u8(vget_low_u8(pixels),vdup_n_u8(multiplier)));
            const int16x8_t hi=vreinterpretq_s16_u16(vmull_u8(vget_high_u8(pixels),vdup_n_u8(multiplier)));
            const uint8x16_t result=vcombine_u8(vqshrun_n_s16(vaddq_s16(lo,add),6),
                                               vqshrun_n_s16(vaddq_s16(hi,add),6));
            vst1q_u8(d+i,result);
        }
        if(i+8<=n){
            const int16x8_t u=vreinterpretq_s16_u16(vmull_u8(vld1_u8(s+i),vdup_n_u8(multiplier)));
            vst1_u8(d+i,vqshrun_n_s16(vaddq_s16(u,add),6));
            i+=8;
        }
        // Do not enter the 32-bit fallback for a few remaining bytes.
        for(;i<n;++i)d[i]=uint8_t(clamp8((scale*int(s[i])+shift+32)>>6));
        return;
    }
    // Full-range fallback for signed or unusually large scales/shifts.
    const int32x4_t add=vdupq_n_s32(shift+32);
    for(;i+8<=n;i+=8){
        const int16x8_t wide=vreinterpretq_s16_u16(vmovl_u8(vld1_u8(s+i)));
        const int32x4_t lo=vaddq_s32(vmulq_n_s32(vmovl_s16(vget_low_s16(wide)),scale),add);
        const int32x4_t hi=vaddq_s32(vmulq_n_s32(vmovl_s16(vget_high_s16(wide)),scale),add);
        const int16x8_t r=vcombine_s16(vqmovn_s32(vshrq_n_s32(lo,6)),vqmovn_s32(vshrq_n_s32(hi,6)));
        vst1_u8(d+i,vqmovun_s16(r));
    }
    for(;i<n;++i)d[i]=uint8_t(clamp8((scale*int(s[i])+shift+32)>>6));
}
void intensity_map_plane_aarch64_neon(uint8_t* d,const uint8_t* s,size_t n,int scale,int shift){intensity(d,s,n,scale,shift);}
void intensity_map_chroma_plane_aarch64_neon(uint8_t* d,const uint8_t* s,size_t n,int scale){intensity(d,s,n,scale,128*64-128*scale);}
void quantize_coefficients_aarch64_neon(const double* c,int* out,int count,double qs,double off,int max_level){
    // ceil/division and error comparisons must use the original scalar order:
    // SIMD division changed exact boundary decisions on some processors.
    // Avoid the old 2-lane load/store, which added overhead without vectorizing
    // the quantization decision. AUTO can still select the scalar primitive.
    for(int i=0;i<count;++i){
        const double a=std::abs(c[i]);
        if(a==0.0){out[i]=0;continue;}
        int q=int(std::ceil((a-off)/qs-0.5));q=std::clamp(q,1,max_level);
        const double err=a-(q*qs+off);
        out[i]=(err*err<a*a)?(c[i]<0?-q:q):0;
    }
}
void perceptual_stats_aarch64_neon(const uint8_t* src,const uint8_t* pred,int s,int w,int h,uint64_t* sum,uint64_t* grad,uint64_t* residual,int* mn,int* mx,int* edges,int* gn){
    uint64_t su=0,g=0,r=0;int lo=255,hi=0,e=0,n=0;
    for(int y=0;y<h;++y){const uint8_t* row=src+ptrdiff_t(y)*s;const uint8_t* prow=pred?pred+ptrdiff_t(y)*s:nullptr;int x=0;
        for(;x+16<=w;x+=16){const uint8x16_t v=vld1q_u8(row+x);
            su+=vaddlvq_u8(v);lo=std::min(lo,int(vminvq_u8(v)));hi=std::max(hi,int(vmaxvq_u8(v)));
            if(prow)r+=vaddlvq_u8(vabdq_u8(v,vld1q_u8(prow+x)));
            // Pixel 0 has no left neighbor. Avoid a read before the source row.
            if(x==0){
                // Rotate within the already-loaded vector. Lane zero is
                // ignored, so no preceding-row/out-of-bounds read is needed.
                const uint8x16_t left=vextq_u8(v,v,15);
                const uint8x16_t d=vsetq_lane_u8(0,vabdq_u8(v,left),0);
                g+=vaddlvq_u8(d);
                e+=vaddlvq_u8(vshrq_n_u8(vcgeq_u8(d,vdupq_n_u8(96)),7));
                n+=15;
            }else{const uint8x16_t d=vabdq_u8(v,vld1q_u8(row+x-1));g+=vaddlvq_u8(d);e+=vaddlvq_u8(vshrq_n_u8(vcgeq_u8(d,vdupq_n_u8(96)),7));n+=16;}
            if(y){const uint8x16_t d=vabdq_u8(v,vld1q_u8(row+x-s));g+=vaddlvq_u8(d);e+=vaddlvq_u8(vshrq_n_u8(vcgeq_u8(d,vdupq_n_u8(96)),7));n+=16;}
        }
        for(;x<w;++x){const int v=row[x];su+=v;lo=std::min(lo,v);hi=std::max(hi,v);if(prow)r+=std::abs(v-int(prow[x]));if(x){int diff=std::abs(v-int(row[x-1]));g+=diff;++n;e+=diff>=96;}if(y){int diff=std::abs(v-int(row[x-s]));g+=diff;++n;e+=diff>=96;}}
    }
    *sum=su;*grad=g;*residual=r;*mn=lo;*mx=hi;*edges=e;*gn=n;
}
} // namespace libvc1::simd
