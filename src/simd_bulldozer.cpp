#include "simd.h"
#include "forward_transform_tables.h"
#include "simd_satd4x4.h"

#include <algorithm>
#include <array>
#include "simd_recon_kernels.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <immintrin.h>
#include <x86intrin.h>

namespace libvc1::simd {
namespace {

static inline uint64_t hsum128_epi64(__m128i v) {
    alignas(16) uint64_t a[2];
    _mm_store_si128(reinterpret_cast<__m128i*>(a),v);
    return a[0]+a[1];
}
static inline __m128i sample_mask128(int s) {
    if (s==2) return _mm_set1_epi16(0x00ff);
    if (s==4) return _mm_set1_epi32(0x000000ff);
    if (s==8) return _mm_set_epi64x(0x00000000000000ffLL,0x00000000000000ffLL);
    if (s==16) return _mm_cvtsi32_si128(0xff);
    return _mm_set1_epi8(static_cast<char>(0xff));
}
static inline uint64_t sad_row_sampled(const uint8_t* a,const uint8_t* b,int n,int ss) {
    uint64_t sad=0; int x=0; const __m128i mask=sample_mask128(ss);
    for (;x+16<=n;x+=16) {
        __m128i va=_mm_loadu_si128(reinterpret_cast<const __m128i*>(a+x));
        __m128i vb=_mm_loadu_si128(reinterpret_cast<const __m128i*>(b+x));
        if (ss!=1) { va=_mm_and_si128(va,mask); vb=_mm_and_si128(vb,mask); }
        sad+=hsum128_epi64(_mm_sad_epu8(va,vb));
    }
    for (;x<n;x+=ss) sad+=static_cast<uint64_t>(a[x]>b[x]?a[x]-b[x]:b[x]-a[x]);
    return sad;
}
static inline __m128d load_u8x2_as_double(const uint8_t* p) {
    uint16_t q; std::memcpy(&q,p,sizeof(q));
    const __m128i b=_mm_cvtsi32_si128(q);
    const __m128i i=_mm_cvtepu8_epi32(b);
    return _mm_cvtepi32_pd(i);
}
static inline void forward8(const double rows[8][8],double* coeff64,bool use_fma) {
    alignas(16) double tmp[8][8]{};
    for (int a=0;a<8;++a) for (int x=0;x<8;x+=2) {
        __m128d acc=_mm_setzero_pd();
        for (int y=0;y<8;++y) {const __m128d k=_mm_set1_pd(forward_transform::k8[a][y]);const __m128d r=_mm_load_pd(rows[y]+x);acc=use_fma?_mm_macc_pd(k,r,acc):_mm_add_pd(acc,_mm_mul_pd(k,r));}
        _mm_store_pd(tmp[a]+x,acc);
    }
    for (int a=0;a<8;++a) for (int bb=0;bb<8;bb+=2) {
        __m128d acc=_mm_setzero_pd();
        for (int x=0;x<8;++x) {
            const __m128d k=_mm_setr_pd(forward_transform::k8[bb][x],forward_transform::k8[bb+1][x]);
            {const __m128d v=_mm_set1_pd(tmp[a][x]);acc=use_fma?_mm_macc_pd(v,k,acc):_mm_add_pd(acc,_mm_mul_pd(v,k));}
        }
        acc=_mm_mul_pd(acc,_mm_set1_pd(1024.0));
        alignas(16) double v[2]; _mm_store_pd(v,acc);
        coeff64[static_cast<size_t>(bb)*8+a]=v[0];
        coeff64[static_cast<size_t>(bb+1)*8+a]=v[1];
    }
}
static inline int arshift_scalar(int v,int s) {
    if (v>=0) return v>>s;
    return -static_cast<int>((static_cast<unsigned long long>(-static_cast<long long>(v))+((1ull<<s)-1ull))>>s);
}
static inline int filter4_scalar(int a,int b,int c,int d,int mode) {
    if(mode==1)return -4*a+53*b+18*c-3*d;
    if(mode==2)return -a+9*b+9*c-d;
    if(mode==3)return -3*a+18*b+53*c-4*d;
    return b;
}
static inline int clamp_u8(int v){return std::clamp(v,0,255);}
static inline int luma_pred_scalar(const uint8_t* c,int stride,int hm,int vm,bool rnd) {
    if(!hm&&!vm)return *c;
    if(hm&&vm){
        static constexpr int sv[4]={0,5,1,5}; const int shift=(sv[hm]+sv[vm])>>1; const int r1=(1<<(shift-1))+(rnd?1:0)-1;
        int t[4]; for(int k=-1;k<=2;++k){const uint8_t* p=c+k; t[k+1]=arshift_scalar(filter4_scalar(p[-stride],p[0],p[stride],p[2*stride],vm)+r1,shift);} 
        return clamp_u8(arshift_scalar(filter4_scalar(t[0],t[1],t[2],t[3],hm)+64-(rnd?1:0),7));
    }
    if(vm){const int raw=filter4_scalar(c[-stride],c[0],c[stride],c[2*stride],vm); const int bias=vm==2?7+(rnd?1:0):31+(rnd?1:0); return clamp_u8(arshift_scalar(raw+bias,vm==2?4:6));}
    const int raw=filter4_scalar(c[-1],c[0],c[1],c[2],hm); const int bias=hm==2?8-(rnd?1:0):32-(rnd?1:0); return clamp_u8(arshift_scalar(raw+bias,hm==2?4:6));
}
static inline int bilinear_pred_scalar(const uint8_t* c,int stride,bool fx,bool fy,bool rnd){
    const int a=c[0]; if(!fx&&!fy)return a; if(fx&&fy)return (a+c[1]+c[stride]+c[stride+1]+(rnd?1:2))>>2; const int b=c[fx?1:stride]; return (a+b+(rnd?0:1))>>1;
}
static inline int chroma_pred_scalar(const uint8_t* c,int stride,int fx,int fy,bool rnd){
    if(!fx&&!fy) return c[0];
    const int bias=rnd?28:32;
    if(fx&&!fy)return (c[0]*(8-fx)*8+c[1]*fx*8+bias)>>6;
    if(!fx&&fy)return (c[0]*(8-fy)*8+c[stride]*fy*8+bias)>>6;
    return (c[0]*(8-fx)*(8-fy)+c[1]*fx*(8-fy)+c[stride]*(8-fx)*fy+c[stride+1]*fx*fy+bias)>>6;
}
static inline __m128i pack4_i32_to_u8(__m128i v){const __m128i p16=_mm_packs_epi32(v,v);return _mm_packus_epi16(p16,p16);}
static inline __m128i clamp_i32(__m128i v){return _mm_min_epi32(_mm_set1_epi32(255),_mm_max_epi32(_mm_setzero_si128(),v));}
static inline __m128i load_u8x4_i32(const uint8_t* p){uint32_t q;std::memcpy(&q,p,4);return _mm_cvtepu8_epi32(_mm_cvtsi32_si128(static_cast<int>(q)));}
static inline void store_u8x4(uint8_t* p,__m128i v){uint32_t q=static_cast<uint32_t>(_mm_cvtsi128_si32(pack4_i32_to_u8(clamp_i32(v))));std::memcpy(p,&q,4);}

} // namespace

uint64_t block_sad_bulldozer(const uint8_t* cur,int cs,const uint8_t* ref,int rs,int w,int h,int ss){uint64_t sad=0;for(int y=0;y<h;y+=ss)sad+=sad_row_sampled(cur+static_cast<ptrdiff_t>(y)*cs,ref+static_cast<ptrdiff_t>(y)*rs,w,ss);if(ss>1)sad*=static_cast<uint64_t>(ss*ss);return sad;}
uint64_t frame_sad_bulldozer(const uint8_t* cur,int cs,const uint8_t* ref,int rs,int w,int h,int dx,int dy,int ss){uint64_t sad=0;for(int y=0;y<h;y+=ss){const int sy=std::clamp(y+dy,0,h-1);const uint8_t* a=cur+static_cast<ptrdiff_t>(y)*cs;const uint8_t* b=ref+static_cast<ptrdiff_t>(sy)*rs;int x=0;while(x<w&&x+dx<0){sad+=std::abs(int(a[x])-int(b[0]));x+=ss;}const int last=std::min(w-1,w-1-dx);if(x<=last){const int samples=(last-x)/ss+1;const int span=(samples-1)*ss+1;const int vecspan=(span/16)*16;if(vecspan>=16){sad+=sad_row_sampled(a+x,b+x+dx,vecspan,ss);x+=vecspan;}while(x<=last){sad+=std::abs(int(a[x])-int(b[x+dx]));x+=ss;}}for(;x<w;x+=ss)sad+=std::abs(int(a[x])-int(b[w-1]));}if(ss>1)sad*=static_cast<uint64_t>(ss*ss);return sad;}

void forward_transform_intra8_bulldozer(const uint8_t* src,int stride,double* coeff64,bool use_fma){alignas(16) double rows[8][8];for(int y=0;y<8;++y)for(int x=0;x<8;x+=2){__m128d v=_mm_sub_pd(load_u8x2_as_double(src+static_cast<ptrdiff_t>(y)*stride+x),_mm_set1_pd(128.0));_mm_store_pd(rows[y]+x,v);}forward8(rows,coeff64,use_fma);}
void forward_transform_residual8_bulldozer(const uint8_t* src,const uint8_t* pred,int stride,double* coeff64,bool use_fma){alignas(16) double rows[8][8];for(int y=0;y<8;++y)for(int x=0;x<8;x+=2){__m128d v=_mm_sub_pd(load_u8x2_as_double(src+static_cast<ptrdiff_t>(y)*stride+x),load_u8x2_as_double(pred+static_cast<ptrdiff_t>(y)*stride+x));_mm_store_pd(rows[y]+x,v);}forward8(rows,coeff64,use_fma);}

void inverse_transform_8x8_bulldozer(int* b){
    std::array<int,64> temp{};
    for(int col=0;col<8;++col){
        const int* src=b+col; int* dst=temp.data()+col*8;
        int t1=12*(src[0]+src[32])+4,t2=12*(src[0]-src[32])+4,t3=16*src[16]+6*src[48],t4=6*src[16]-16*src[48];
        int t5=t1+t3,t6=t2+t4,t7=t2-t4,t8=t1-t3;
        t1=16*src[8]+15*src[24]+9*src[40]+4*src[56]; t2=15*src[8]-4*src[24]-16*src[40]-9*src[56];
        t3=9*src[8]-16*src[24]+4*src[40]+15*src[56]; t4=4*src[8]-9*src[24]+15*src[40]-16*src[56];
        dst[0]=arshift_scalar(t5+t1,3); dst[1]=arshift_scalar(t6+t2,3); dst[2]=arshift_scalar(t7+t3,3); dst[3]=arshift_scalar(t8+t4,3);
        dst[4]=arshift_scalar(t8-t4,3); dst[5]=arshift_scalar(t7-t3,3); dst[6]=arshift_scalar(t6-t2,3); dst[7]=arshift_scalar(t5-t1,3);
    }
    for(int col=0;col<8;++col){
        const int* src=temp.data()+col;
        int t1=12*(src[0]+src[32])+64,t2=12*(src[0]-src[32])+64,t3=16*src[16]+6*src[48],t4=6*src[16]-16*src[48];
        int t5=t1+t3,t6=t2+t4,t7=t2-t4,t8=t1-t3;
        t1=16*src[8]+15*src[24]+9*src[40]+4*src[56]; t2=15*src[8]-4*src[24]-16*src[40]-9*src[56];
        t3=9*src[8]-16*src[24]+4*src[40]+15*src[56]; t4=4*src[8]-9*src[24]+15*src[40]-16*src[56];
        b[0*8+col]=arshift_scalar(t5+t1,7); b[1*8+col]=arshift_scalar(t6+t2,7); b[2*8+col]=arshift_scalar(t7+t3,7); b[3*8+col]=arshift_scalar(t8+t4,7);
        b[4*8+col]=arshift_scalar(t8-t4+1,7); b[5*8+col]=arshift_scalar(t7-t3+1,7); b[6*8+col]=arshift_scalar(t6-t2+1,7); b[7*8+col]=arshift_scalar(t5-t1+1,7);
    }
}

void put_block8_bulldozer(uint8_t* dst,int stride,const int* block,int bias){const __m128i bv=_mm_set1_epi32(bias);for(int y=0;y<8;++y)for(int x=0;x<8;x+=4){__m128i v=_mm_add_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(block+y*8+x)),bv);store_u8x4(dst+static_cast<ptrdiff_t>(y)*stride+x,v);}}
void add_block_rect_bulldozer(uint8_t* dst,int stride,const int* block,int w,int h){for(int y=0;y<h;++y)for(int x=0;x<w;x+=4){__m128i p=load_u8x4_i32(dst+static_cast<ptrdiff_t>(y)*stride+x);__m128i r=_mm_loadu_si128(reinterpret_cast<const __m128i*>(block+y*8+x));store_u8x4(dst+static_cast<ptrdiff_t>(y)*stride+x,_mm_add_epi32(p,r));}}

uint64_t block_sad_luma_qpel_bulldozer(const uint8_t* cur,int cs,const uint8_t* ref,int rs,int w,int h,int hm,int vm,bool rnd){uint64_t sad=0;for(int y=0;y<h;++y)for(int x=0;x<w;++x)sad+=std::abs(int(cur[static_cast<ptrdiff_t>(y)*cs+x])-luma_pred_scalar(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,hm,vm,rnd));return sad;}
uint64_t block_sad_luma_bilinear_bulldozer(const uint8_t* cur,int cs,const uint8_t* ref,int rs,int w,int h,bool fx,bool fy,bool rnd){uint64_t sad=0;for(int y=0;y<h;++y)for(int x=0;x<w;++x)sad+=std::abs(int(cur[static_cast<ptrdiff_t>(y)*cs+x])-bilinear_pred_scalar(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,fx,fy,rnd));return sad;}
void luma_mc_block_bulldozer(uint8_t* dst,int ds,const uint8_t* ref,int rs,int w,int h,int hm,int vm,bool rnd,bool bilinear){for(int y=0;y<h;++y)for(int x=0;x<w;++x)dst[static_cast<ptrdiff_t>(y)*ds+x]=static_cast<uint8_t>(bilinear?bilinear_pred_scalar(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,hm!=0,vm!=0,rnd):luma_pred_scalar(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,hm,vm,rnd));}
void luma_mc_block_avg_bulldozer(uint8_t* dst,int ds,const uint8_t* ref,int rs,int w,int h,int hm,int vm,bool rnd,bool bilinear){for(int y=0;y<h;++y)for(int x=0;x<w;++x){uint8_t& d=dst[static_cast<ptrdiff_t>(y)*ds+x];int p=bilinear?bilinear_pred_scalar(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,hm!=0,vm!=0,rnd):luma_pred_scalar(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,hm,vm,rnd);d=static_cast<uint8_t>((int(d)+p+1)>>1);}}
void chroma_mc_block_bulldozer(uint8_t* dst,int ds,const uint8_t* ref,int rs,int w,int h,int fx,int fy,bool rnd){for(int y=0;y<h;++y)for(int x=0;x<w;++x)dst[static_cast<ptrdiff_t>(y)*ds+x]=static_cast<uint8_t>(chroma_pred_scalar(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,fx,fy,rnd));}
void chroma_mc_block_avg_bulldozer(uint8_t* dst,int ds,const uint8_t* ref,int rs,int w,int h,int fx,int fy,bool rnd){for(int y=0;y<h;++y)for(int x=0;x<w;++x){uint8_t& d=dst[static_cast<ptrdiff_t>(y)*ds+x];int p=chroma_pred_scalar(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,fx,fy,rnd);d=static_cast<uint8_t>((int(d)+p+1)>>1);}}

void intensity_map_plane_bulldozer(uint8_t* dst,const uint8_t* src,size_t n,int scale,int shift){size_t i=0;const __m128i sv=_mm_set1_epi32(scale),sh=_mm_set1_epi32(shift+32);for(;i+4<=n;i+=4){__m128i v=load_u8x4_i32(src+i);v=_mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(v,sv),sh),6);store_u8x4(dst+i,v);}for(;i<n;++i)dst[i]=static_cast<uint8_t>(clamp_u8((scale*int(src[i])+shift+32)>>6));}
void intensity_map_chroma_plane_bulldozer(uint8_t* dst,const uint8_t* src,size_t n,int scale){size_t i=0;const __m128i sv=_mm_set1_epi32(scale),bias=_mm_set1_epi32(128*64+32),c128=_mm_set1_epi32(128);for(;i+4<=n;i+=4){__m128i v=_mm_sub_epi32(load_u8x4_i32(src+i),c128);v=_mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(v,sv),bias),6);store_u8x4(dst+i,v);}for(;i<n;++i)dst[i]=static_cast<uint8_t>(clamp_u8((scale*(int(src[i])-128)+128*64+32)>>6));}

void forward_transform_residual_rect_bulldozer(const uint8_t* src,const uint8_t* pred,int stride,int w,int h,double* coeff,bool use_fma){alignas(16) double rows[8][8]{};alignas(16) double tmp[8][8]{};for(int y=0;y<h;++y)for(int x=0;x<w;x+=2)_mm_store_pd(rows[y]+x,_mm_sub_pd(load_u8x2_as_double(src+static_cast<ptrdiff_t>(y)*stride+x),load_u8x2_as_double(pred+static_cast<ptrdiff_t>(y)*stride+x)));for(int a=0;a<h;++a)for(int x=0;x<w;x+=2){__m128d acc=_mm_setzero_pd();for(int y=0;y<h;++y){double k=h==8?forward_transform::k8[a][y]:forward_transform::k4[a][y];{const __m128d kv=_mm_set1_pd(k);const __m128d r=_mm_load_pd(rows[y]+x);acc=use_fma?_mm_macc_pd(kv,r,acc):_mm_add_pd(acc,_mm_mul_pd(kv,r));}}_mm_store_pd(tmp[a]+x,acc);}std::fill(coeff,coeff+64,0.0);for(int a=0;a<h;++a)for(int bb=0;bb<w;bb+=2){__m128d acc=_mm_setzero_pd();for(int x=0;x<w;++x){__m128d k=w==8?_mm_setr_pd(forward_transform::k8[bb][x],forward_transform::k8[bb+1][x]):_mm_setr_pd(forward_transform::k4[bb][x],forward_transform::k4[bb+1][x]);{const __m128d v=_mm_set1_pd(tmp[a][x]);acc=use_fma?_mm_macc_pd(v,k,acc):_mm_add_pd(acc,_mm_mul_pd(v,k));}}acc=_mm_mul_pd(acc,_mm_set1_pd(1024.0));_mm_storeu_pd(coeff+static_cast<size_t>(a)*8+bb,acc);}}

void quantize_coefficients_bulldozer(const double* coeff,int* out,int count,double qs,double off,int max_level){const __m128d signbit=_mm_set1_pd(-0.0),qv=_mm_set1_pd(qs),ov=_mm_set1_pd(off),one=_mm_set1_pd(1.0),mx=_mm_set1_pd(double(max_level)),zero=_mm_setzero_pd(),half=_mm_set1_pd(0.5);int i=0;for(;i+2<=count;i+=2){__m128d c=_mm_loadu_pd(coeff+i),a=_mm_andnot_pd(signbit,c);__m128d n=_mm_div_pd(_mm_sub_pd(a,ov),qv);n=_mm_ceil_pd(_mm_sub_pd(n,half));n=_mm_min_pd(mx,_mm_max_pd(one,n));__m128d recon=_mm_add_pd(_mm_mul_pd(n,qv),ov),e=_mm_sub_pd(a,recon),err=_mm_mul_pd(e,e),err0=_mm_mul_pd(a,a),keep=_mm_cmplt_pd(err,err0),neg=_mm_cmplt_pd(c,zero),signedn=_mm_blendv_pd(n,_mm_sub_pd(zero,n),neg),selected=_mm_blendv_pd(zero,signedn,keep);__m128i qi=_mm_cvttpd_epi32(selected);out[i]=_mm_cvtsi128_si32(qi);out[i+1]=_mm_extract_epi32(qi,1);}for(;i<count;++i){double c=coeff[i],a=std::abs(c);if(a==0){out[i]=0;continue;}int n=int(std::ceil((a-off)/qs-0.5));n=std::clamp(n,1,max_level);double e=a-(double(n)*qs+off);out[i]=(e*e<a*a)?(c<0?-n:n):0;}}

uint64_t sum_u8_bulldozer(const uint8_t* src,size_t count){uint64_t total=0;const __m128i z=_mm_setzero_si128();size_t i=0;for(;i+16<=count;i+=16)total+=hsum128_epi64(_mm_sad_epu8(_mm_loadu_si128(reinterpret_cast<const __m128i*>(src+i)),z));for(;i<count;++i)total+=src[i];return total;}
static inline uint64_t sum_abs_u8_128(__m128i a,__m128i b){return hsum128_epi64(_mm_sad_epu8(a,b));}
static inline int count_ge_u8_128(__m128i d,int threshold,unsigned valid_mask){const __m128i t=_mm_set1_epi8(static_cast<char>(threshold));const __m128i ge=_mm_cmpeq_epi8(_mm_max_epu8(d,t),d);return __builtin_popcount(static_cast<unsigned>(_mm_movemask_epi8(ge))&valid_mask);}
void perceptual_stats_bulldozer(const uint8_t* src,const uint8_t* pred,int stride,int width,int height,uint64_t* sum,uint64_t* grad,uint64_t* residual,int* min_value,int* max_value,int* edges,int* grad_count){uint64_t s=0,g=0,r=0;int mn=255,mx=0,e=0,gn=0;const __m128i zero=_mm_setzero_si128(),low8=_mm_set_epi64x(0,-1ll),high8ff=_mm_set_epi64x(-1ll,0),lane0_clear=_mm_setr_epi8(0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1);__m128i vmin=_mm_set1_epi8(static_cast<char>(0xff)),vmax=zero;bool hv=false;for(int y=0;y<height;++y){const uint8_t* row=src+static_cast<ptrdiff_t>(y)*stride;const uint8_t* prow=pred?pred+static_cast<ptrdiff_t>(y)*stride:nullptr;int x=0;for(;x+16<=width;x+=16){__m128i v=_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+x));hv=true;vmin=_mm_min_epu8(vmin,v);vmax=_mm_max_epu8(vmax,v);s+=sum_abs_u8_128(v,zero);if(prow)r+=sum_abs_u8_128(v,_mm_loadu_si128(reinterpret_cast<const __m128i*>(prow+x)));__m128i left;unsigned valid=0xffffu;if(x==0){left=_mm_slli_si128(v,1);valid&=~1u;}else left=_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+x-1));__m128i hd=_mm_sub_epi8(_mm_max_epu8(v,left),_mm_min_epu8(v,left));if(x==0)hd=_mm_and_si128(hd,lane0_clear);g+=sum_abs_u8_128(hd,zero);e+=count_ge_u8_128(hd,96,valid);gn+=__builtin_popcount(valid);if(y>0){__m128i top=_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+x-stride));__m128i vd=_mm_sub_epi8(_mm_max_epu8(v,top),_mm_min_epu8(v,top));g+=sum_abs_u8_128(vd,zero);e+=count_ge_u8_128(vd,96,0xffffu);gn+=16;}}if(x+8<=width){__m128i v=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row+x));hv=true;vmin=_mm_min_epu8(vmin,_mm_or_si128(v,high8ff));vmax=_mm_max_epu8(vmax,v);s+=sum_abs_u8_128(v,zero);if(prow)r+=sum_abs_u8_128(v,_mm_loadl_epi64(reinterpret_cast<const __m128i*>(prow+x)));__m128i left;unsigned valid=0xffu;if(x==0){left=_mm_slli_si128(v,1);valid&=~1u;}else left=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row+x-1));__m128i hd=_mm_and_si128(_mm_sub_epi8(_mm_max_epu8(v,left),_mm_min_epu8(v,left)),low8);if(x==0)hd=_mm_and_si128(hd,lane0_clear);g+=sum_abs_u8_128(hd,zero);e+=count_ge_u8_128(hd,96,valid);gn+=__builtin_popcount(valid);if(y>0){__m128i top=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row+x-stride));__m128i vd=_mm_and_si128(_mm_sub_epi8(_mm_max_epu8(v,top),_mm_min_epu8(v,top)),low8);g+=sum_abs_u8_128(vd,zero);e+=count_ge_u8_128(vd,96,0xffu);gn+=8;}x+=8;}for(;x<width;++x){int v=row[x];s+=v;mn=std::min(mn,v);mx=std::max(mx,v);if(prow)r+=std::abs(v-int(prow[x]));if(x>0){int d=std::abs(v-int(row[x-1]));g+=d;++gn;if(d>=96)++e;}if(y>0){int d=std::abs(v-int(row[x-stride]));g+=d;++gn;if(d>=96)++e;}}}if(hv){alignas(16) uint8_t mins[16],maxs[16];_mm_store_si128(reinterpret_cast<__m128i*>(mins),vmin);_mm_store_si128(reinterpret_cast<__m128i*>(maxs),vmax);for(int i=0;i<16;++i){mn=std::min(mn,int(mins[i]));mx=std::max(mx,int(maxs[i]));}}*sum=s;*grad=g;*residual=r;*min_value=mn;*max_value=mx;*edges=e;*grad_count=gn;}

uint64_t satd4x4_bulldozer(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride){return detail::satd4x4_target(cur,cur_stride,pred,pred_stride);}


void dequant_coefficients_bulldozer(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform){detail::dequant_coefficients_target(q,out,count,double_quant,mquant,nonuniform);}
void inverse_transform_rect_bulldozer(int* block64,int type){detail::inverse_transform_rect_target(block64,type);}
void overlap_vertical8_bulldozer(int* edge,int stride){detail::overlap_vertical8_target(edge,stride);}
void overlap_horizontal8_bulldozer(int* edge,int stride){detail::overlap_horizontal8_target(edge,stride);}
bool loop_filter4_h_bulldozer(uint8_t* p,int stride,int x,int y,int pq){return detail::loop_filter4_h_target(p,stride,x,y,pq);}
bool loop_filter4_v_bulldozer(uint8_t* p,int stride,int x,int y,int pq){return detail::loop_filter4_v_target(p,stride,x,y,pq);}

} // namespace libvc1::simd
