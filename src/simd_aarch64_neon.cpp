// AArch64 Advanced SIMD (NEON) kernels. No optional ISA is required. In
// particular, don't use FP FMA: contraction can change quantizer decisions.
#include "simd.h"
#include "forward_transform_tables.h"
#include <arm_neon.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace libvc1::simd {
namespace {
inline uint64_t absdiff(uint8_t a,uint8_t b) { return a>b?a-b:b-a; }

// Reduce once per row / every 64 vectors instead of once per 16 pixels.
// The 16-bit pair accumulator is bounded by 64 * (2 * 255) = 32640;
// it cannot wrap even on high-contrast input. Samples are aligned to the
// original scalar x=0,ss,2*ss lattice, including the scalar tail.
static constexpr uint8_t kSampleMask[4][16] = {
    {255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0},
    {255,0,0,0,255,0,0,0,255,0,0,0,255,0,0,0},
    {255,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0},
    {255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};
inline uint64_t sad_row(const uint8_t* a,const uint8_t* b,int width,int ss) {
    if (ss!=1 && ss!=2 && ss!=4 && ss!=8 && ss!=16) {
        uint64_t s=0;
        for (int x=0;x<width;x+=ss) s+=absdiff(a[x],b[x]);
        return s;
    }
    // Small/subsampled blocks are cheaper without SIMD setup or wide loads.
    if(width<16) {
        uint64_t s=0;
        for(int x=0;x<width;x+=ss)s+=absdiff(a[x],b[x]);
        return s;
    }
    const uint8x16_t mask=ss==1?vdupq_n_u8(255):
        vld1q_u8(kSampleMask[ss==2?0:ss==4?1:ss==8?2:3]);
    uint64_t total=0;
    uint16x8_t pairs=vdupq_n_u16(0);
    int x=0, batched=0;
    for(;x+16<=width;x+=16) {
        uint8x16_t d=vabdq_u8(vld1q_u8(a+x),vld1q_u8(b+x));
        if(ss!=1)d=vandq_u8(d,mask);
        pairs=vpadalq_u8(pairs,d);
        if(++batched==64){total+=vaddlvq_u16(pairs);pairs=vdupq_n_u16(0);batched=0;}
    }
    total+=vaddlvq_u16(pairs);
    for(;x<width;x+=ss)total+=absdiff(a[x],b[x]);
    return total;
}

inline float64x2_t pair_double(const uint8_t* p) {
    // Avoid a speculative 8/16-byte load across the right edge of a 4/8 block.
    return vsetq_lane_f64(static_cast<double>(p[1]),
                           vdupq_n_f64(static_cast<double>(p[0])),1);
}

void forward_rect(const uint8_t* src,const uint8_t* pred,int stride,
                  int w,int h,double* coeff,bool intra) {
    alignas(16) double rows[8][8]{};
    alignas(16) double tmp[8][8]{};
    for (int y=0;y<h;++y) for (int x=0;x<w;x+=2) {
        const uint8_t* p=src+static_cast<ptrdiff_t>(y)*stride+x;
        const float64x2_t a=pair_double(p);
        const float64x2_t b=intra?vdupq_n_f64(128.0):pair_double(pred+static_cast<ptrdiff_t>(y)*stride+x);
        vst1q_f64(rows[y]+x,vsubq_f64(a,b));
    }
    for (int a=0;a<h;++a) for (int x=0;x<w;x+=2) {
        float64x2_t acc=vdupq_n_f64(0.0);
        for (int y=0;y<h;++y) {
            const double k=h==8?forward_transform::k8[a][y]:forward_transform::k4[a][y];
            acc=vaddq_f64(acc,vmulq_n_f64(vld1q_f64(rows[y]+x),k));
        }
        vst1q_f64(tmp[a]+x,acc);
    }
    std::fill(coeff,coeff+64,0.0);
    for (int a=0;a<h;++a) for (int bb=0;bb<w;bb+=2) {
        float64x2_t acc=vdupq_n_f64(0.0);
        for (int x=0;x<w;++x) {
            const double k0=w==8?forward_transform::k8[bb][x]:forward_transform::k4[bb][x];
            const double k1=w==8?forward_transform::k8[bb+1][x]:forward_transform::k4[bb+1][x];
            const float64x2_t k=vsetq_lane_f64(k1,vdupq_n_f64(k0),1);
            acc=vaddq_f64(acc,vmulq_n_f64(k,tmp[a][x]));
        }
        // 8x8 layout is transposed, matching the existing x86 kernel exactly;
        // rectangular transforms use the normal 8-stride raster layout.
        if (w==8 && h==8) {
            alignas(16) double v[2];
            vst1q_f64(v,vmulq_n_f64(acc,1024.0));
            coeff[static_cast<size_t>(bb)*8+a]=v[0];
            coeff[static_cast<size_t>(bb+1)*8+a]=v[1];
        } else {
            vst1q_f64(coeff+static_cast<size_t>(a)*8+bb,vmulq_n_f64(acc,1024.0));
        }
    }
}

} // namespace

uint64_t block_sad_aarch64_neon(const uint8_t* cur,int cs,const uint8_t* ref,int rs,
                                int w,int h,int ss) {
    if(w<=0 || h<=0 || ss<=0)return 0;
    uint64_t s=0;
    for(int y=0;y<h;y+=ss)
        s+=sad_row(cur+static_cast<ptrdiff_t>(y)*cs,ref+static_cast<ptrdiff_t>(y)*rs,w,ss);
    return ss>1?s*static_cast<uint64_t>(ss)*static_cast<uint64_t>(ss):s;
}

uint64_t frame_sad_aarch64_neon(const uint8_t* cur,int cs,const uint8_t* ref,int rs,
                                int w,int h,int dx,int dy,int ss) {
    if(w<=0 || h<=0 || ss<=0)return 0;
    uint64_t sum=0;
    for(int y=0;y<h;y+=ss) {
        const uint8_t* a=cur+static_cast<ptrdiff_t>(y)*cs;
        const uint8_t* b=ref+static_cast<ptrdiff_t>(std::clamp(y+dy,0,h-1))*rs;
        int x=0;
        // Keep the boundary clamping scalar. Center is vectorized, preserving
        // sampling phase even for negative dx and odd picture dimensions.
        for(;x<w && (static_cast<int64_t>(x)+dx<0);x+=ss) sum+=absdiff(a[x],b[0]);
        const int max_center=static_cast<int>(std::clamp<int64_t>(static_cast<int64_t>(w)-dx,0,w));
        if(x<max_center) {
            // Vectorize only if the start is on a 16-byte sampling boundary;
            // otherwise the scalar prefix puts it there.
            while(x<max_center && (x%16)!=0) { sum+=absdiff(a[x],b[x+dx]); x+=ss; }
            if(x<max_center && (ss==1||ss==2||ss==4||ss==8||ss==16)) {
                const int length=max_center-x;
                const int vec=(length/16)*16;
                if(vec>0) {sum+=sad_row(a+x,b+x+dx,vec,ss);x+=vec;}
            }
            for(;x<max_center;x+=ss)sum+=absdiff(a[x],b[x+dx]);
        }
        for(;x<w;x+=ss)sum+=absdiff(a[x],b[w-1]);
    }
    return ss>1?sum*static_cast<uint64_t>(ss)*static_cast<uint64_t>(ss):sum;
}

void forward_transform_intra8_aarch64_neon(const uint8_t* src,int stride,double* out,bool) {
    forward_rect(src,nullptr,stride,8,8,out,true);
}
void forward_transform_residual8_aarch64_neon(const uint8_t* src,const uint8_t* pred,int stride,double* out,bool) {
    forward_rect(src,pred,stride,8,8,out,false);
}
void forward_transform_residual_rect_aarch64_neon(const uint8_t* src,const uint8_t* pred,int stride,int w,int h,double* out,bool) {
    forward_rect(src,pred,stride,w,h,out,false);
}
void put_block8_aarch64_neon(uint8_t* dst,int stride,const int* block,int bias) {
    for(int y=0;y<8;++y) {
        const int* b=block+y*8;
        const int16x8_t values=vcombine_s16(vqmovn_s32(vaddq_s32(vld1q_s32(b),vdupq_n_s32(bias))),
                                             vqmovn_s32(vaddq_s32(vld1q_s32(b+4),vdupq_n_s32(bias))));
        vst1_u8(dst+static_cast<ptrdiff_t>(y)*stride,vqmovun_s16(values));
    }
}
void add_block_rect_aarch64_neon(uint8_t* dst,int stride,const int* block,int w,int h) {
    for(int y=0;y<h;++y){
        uint8_t* d=dst+static_cast<ptrdiff_t>(y)*stride;
        const int* b=block+y*8;
        int x=0;
        for(;x+8<=w;x+=8){
            const uint16x8_t pixels=vmovl_u8(vld1_u8(d+x));
            const int32x4_t lo=vaddq_s32(vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(pixels))),vld1q_s32(b+x));
            const int32x4_t hi=vaddq_s32(vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(pixels))),vld1q_s32(b+x+4));
            vst1_u8(d+x,vqmovun_s16(vcombine_s16(vqmovn_s32(lo),vqmovn_s32(hi))));
        }
        // 4x4 rectangles are common: avoid constructing a vector for just
        // four destination pixels. This also handles odd widths safely.
        for(;x<w;++x)d[x]=static_cast<uint8_t>(std::clamp(int(d[x])+b[x],0,255));
    }
}
uint64_t sum_u8_aarch64_neon(const uint8_t* src,size_t n) {
    uint64_t sum=0;size_t x=0;
    // Work on four independent 16-byte vectors to hide load dependency and
    // amortize the horizontal reduction.  Each lane holds at most 4*255.
    for(;x+64<=n;x+=64){
        const uint16x8_t a=vpaddlq_u8(vld1q_u8(src+x));
        const uint16x8_t b=vpaddlq_u8(vld1q_u8(src+x+16));
        const uint16x8_t c=vpaddlq_u8(vld1q_u8(src+x+32));
        const uint16x8_t d=vpaddlq_u8(vld1q_u8(src+x+48));
        sum+=vaddlvq_u16(vaddq_u16(vaddq_u16(a,b),vaddq_u16(c,d)));
    }
    for(;x+16<=n;x+=16)sum+=vaddlvq_u8(vld1q_u8(src+x));
    for(;x<n;++x)sum+=src[x];
    return sum;
}
// Four-pixel loads are intentionally exactly four bytes: 4x4 blocks can sit
// at the right edge of an image. Never fetch 8 bytes from these rows.
static inline uint32x2_t satd_load_rows(const uint8_t* row0,const uint8_t* row1) {
    uint32_t a,b;
    __builtin_memcpy(&a,row0,sizeof(a));
    __builtin_memcpy(&b,row1,sizeof(b));
    return vset_lane_u32(b,vdup_n_u32(a),1);
}
// The order of the horizontal outputs is [DC, coefficient 2, coefficient 1,
// coefficient 3]. This permutation is immaterial to SATD: the subsequent
// vertical transform operates on each coefficient independently and takes
// the sum of their absolute values.
static inline int16x4_t satd_hadamard_row(int16x4_t pixels) {
    const int16x4_t rev=vrev64_s16(pixels);
    const int16x4_t zipped=vzip1_s16(vadd_s16(pixels,rev),vsub_s16(pixels,rev));
    const int16x4_t swapped=vext_s16(zipped,zipped,2);
    return vzip1_s16(vadd_s16(zipped,swapped),vsub_s16(zipped,swapped));
}
uint64_t satd4x4_aarch64_neon(const uint8_t* cur,int cs,const uint8_t* pred,int ps) {
#if defined(__AARCH64EB__)
    // Packed 32-bit loads below rely on LE NEON lane order; keep BE correct.
    int16_t h[4][4];
    for(int y=0;y<4;++y){
        const uint8_t* c=cur+static_cast<ptrdiff_t>(y)*cs;
        const uint8_t* p=pred+static_cast<ptrdiff_t>(y)*ps;
        const int a=int(c[0])-p[0],b=int(c[1])-p[1],d=int(c[2])-p[2],e=int(c[3])-p[3];
        h[y][0]=a+b+d+e;h[y][1]=a+b-d-e;
        h[y][2]=a-b-d+e;h[y][3]=a-b+d-e;
    }
    const int16x4_t r0=vld1_s16(h[0]),r1=vld1_s16(h[1]),r2=vld1_s16(h[2]),r3=vld1_s16(h[3]);
#else
    const uint16x8_t c01=vmovl_u8(vreinterpret_u8_u32(satd_load_rows(cur,cur+cs)));
    const uint16x8_t p01=vmovl_u8(vreinterpret_u8_u32(satd_load_rows(pred,pred+ps)));
    const uint16x8_t c23=vmovl_u8(vreinterpret_u8_u32(satd_load_rows(cur+2*static_cast<ptrdiff_t>(cs),cur+3*static_cast<ptrdiff_t>(cs))));
    const uint16x8_t p23=vmovl_u8(vreinterpret_u8_u32(satd_load_rows(pred+2*static_cast<ptrdiff_t>(ps),pred+3*static_cast<ptrdiff_t>(ps))));
    const int16x8_t d01=vreinterpretq_s16_u16(vsubq_u16(c01,p01));
    const int16x8_t d23=vreinterpretq_s16_u16(vsubq_u16(c23,p23));
    const int16x4_t r0=satd_hadamard_row(vget_low_s16(d01));
    const int16x4_t r1=satd_hadamard_row(vget_high_s16(d01));
    const int16x4_t r2=satd_hadamard_row(vget_low_s16(d23));
    const int16x4_t r3=satd_hadamard_row(vget_high_s16(d23));
#endif
    const int16x4_t a=vadd_s16(r0,r3),b=vadd_s16(r1,r2),c=vsub_s16(r1,r2),d=vsub_s16(r0,r3);
    const int16x4_t v0=vadd_s16(a,b),v1=vadd_s16(d,c),v2=vsub_s16(a,b),v3=vsub_s16(d,c);
    const uint16x4_t total=vadd_u16(vadd_u16(vreinterpret_u16_s16(vabs_s16(v0)),vreinterpret_u16_s16(vabs_s16(v1))),
                                  vadd_u16(vreinterpret_u16_s16(vabs_s16(v2)),vreinterpret_u16_s16(vabs_s16(v3))));
    return (static_cast<uint64_t>(vaddv_u16(total))+1)>>1;
}

} // namespace libvc1::simd
