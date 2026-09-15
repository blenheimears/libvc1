#include "simd.h"
#include "forward_transform_tables.h"
#include "simd_satd4x4.h"

#include <algorithm>
#include <array>
#include "simd_recon_kernels.h"
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <immintrin.h>

namespace libvc1::simd {
namespace {

static inline uint64_t hsum256_epi64(__m256i v) {
    alignas(32) uint64_t a[4];
    _mm256_store_si256(reinterpret_cast<__m256i*>(a), v);
    return a[0]+a[1]+a[2]+a[3];
}
static inline uint64_t hsum128_epi64(__m128i v) {
    alignas(16) uint64_t a[2];
    _mm_store_si128(reinterpret_cast<__m128i*>(a), v);
    return a[0]+a[1];
}

static inline __m256i sample_mask256(int s) {
    if (s == 2) return _mm256_set1_epi16(0x00ff);
    if (s == 4) return _mm256_set1_epi32(0x000000ff);
    if (s == 8) return _mm256_set1_epi64x(0x00000000000000ffLL);
    if (s == 16) return _mm256_set_epi64x(0,0x00000000000000ffLL,0,0x00000000000000ffLL);
    return _mm256_set1_epi8(static_cast<char>(0xff));
}
static inline __m128i sample_mask128(int s) {
    if (s == 2) return _mm_set1_epi16(0x00ff);
    if (s == 4) return _mm_set1_epi32(0x000000ff);
    if (s == 8) return _mm_set_epi64x(0x00000000000000ffLL,0x00000000000000ffLL);
    if (s == 16) return _mm_cvtsi32_si128(0xff);
    return _mm_set1_epi8(static_cast<char>(0xff));
}

static inline uint64_t sad_row_sampled(const uint8_t* a, const uint8_t* b,
                                       int n, int sample_stride) {
    uint64_t sad=0;
    int x=0;
    const __m256i m256=sample_mask256(sample_stride);
    while (x+32 <= n) {
        __m256i va=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(a+x));
        __m256i vb=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(b+x));
        if (sample_stride != 1) {
            va=_mm256_and_si256(va,m256);
            vb=_mm256_and_si256(vb,m256);
        }
        sad += hsum256_epi64(_mm256_sad_epu8(va,vb));
        x += 32;
    }
    if (x+16 <= n) {
        __m128i va=_mm_loadu_si128(reinterpret_cast<const __m128i*>(a+x));
        __m128i vb=_mm_loadu_si128(reinterpret_cast<const __m128i*>(b+x));
        if (sample_stride != 1) {
            const __m128i m128=sample_mask128(sample_stride);
            va=_mm_and_si128(va,m128);
            vb=_mm_and_si128(vb,m128);
        }
        sad += hsum128_epi64(_mm_sad_epu8(va,vb));
        x += 16;
    }
    for (;x<n;x+=sample_stride)
        sad += static_cast<uint64_t>(a[x]>b[x]?a[x]-b[x]:b[x]-a[x]);
    return sad;
}

static inline __m256d load_u8x4_as_double(const uint8_t* p) {
    uint32_t q;
    std::memcpy(&q,p,sizeof(q));
    const __m128i b=_mm_cvtsi32_si128(static_cast<int>(q));
    const __m128i i=_mm_cvtepu8_epi32(b);
    return _mm256_cvtepi32_pd(i);
}

static inline void forward8_from_rows(const __m256d lo[8], const __m256d hi[8],
                                      double* coeff64, bool use_fma) {
    alignas(32) double tmp[8][8];
    for (int a=0;a<8;++a) {
        __m256d accl=_mm256_setzero_pd();
        __m256d acch=_mm256_setzero_pd();
        for (int y=0;y<8;++y) {
            const __m256d k=_mm256_set1_pd(forward_transform::k8[a][y]);
            if (use_fma) {
                accl=_mm256_fmadd_pd(k,lo[y],accl);
                acch=_mm256_fmadd_pd(k,hi[y],acch);
            } else {
                accl=_mm256_add_pd(accl,_mm256_mul_pd(k,lo[y]));
                acch=_mm256_add_pd(acch,_mm256_mul_pd(k,hi[y]));
            }
        }
        _mm256_store_pd(tmp[a],accl);
        _mm256_store_pd(tmp[a]+4,acch);
    }
    for (int a=0;a<8;++a) {
        __m256d accl=_mm256_setzero_pd();
        __m256d acch=_mm256_setzero_pd();
        for (int x=0;x<8;++x) {
            const __m256d kl=_mm256_setr_pd(forward_transform::k8[0][x],forward_transform::k8[1][x],forward_transform::k8[2][x],forward_transform::k8[3][x]);
            const __m256d kh=_mm256_setr_pd(forward_transform::k8[4][x],forward_transform::k8[5][x],forward_transform::k8[6][x],forward_transform::k8[7][x]);
            const __m256d v=_mm256_set1_pd(tmp[a][x]);
            if (use_fma) {
                accl=_mm256_fmadd_pd(v,kl,accl);
                acch=_mm256_fmadd_pd(v,kh,acch);
            } else {
                accl=_mm256_add_pd(accl,_mm256_mul_pd(v,kl));
                acch=_mm256_add_pd(acch,_mm256_mul_pd(v,kh));
            }
        }
        alignas(32) double vl[4],vh[4];
        _mm256_store_pd(vl,accl);
        _mm256_store_pd(vh,acch);
        for (int bb=0;bb<4;++bb) coeff64[static_cast<size_t>(bb)*8+a]=1024.0*vl[bb];
        for (int bb=0;bb<4;++bb) coeff64[static_cast<size_t>(bb+4)*8+a]=1024.0*vh[bb];
    }
}

static inline void load_intra_rows(const uint8_t* src,int stride,__m256d lo[8],__m256d hi[8]) {
    const __m256d bias=_mm256_set1_pd(128.0);
    for (int y=0;y<8;++y) {
        const uint8_t* p=src+static_cast<ptrdiff_t>(y)*stride;
        lo[y]=_mm256_sub_pd(load_u8x4_as_double(p),bias);
        hi[y]=_mm256_sub_pd(load_u8x4_as_double(p+4),bias);
    }
}
static inline void load_residual_rows(const uint8_t* src,const uint8_t* pred,int stride,
                                      __m256d lo[8],__m256d hi[8]) {
    for (int y=0;y<8;++y) {
        const uint8_t* a=src+static_cast<ptrdiff_t>(y)*stride;
        const uint8_t* b=pred+static_cast<ptrdiff_t>(y)*stride;
        lo[y]=_mm256_sub_pd(load_u8x4_as_double(a),load_u8x4_as_double(b));
        hi[y]=_mm256_sub_pd(load_u8x4_as_double(a+4),load_u8x4_as_double(b+4));
    }
}

} // namespace

uint64_t block_sad_x86_64_v3(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride) {
    uint64_t sad=0;
    for (int y=0;y<height;y+=sample_stride)
        sad += sad_row_sampled(cur+static_cast<ptrdiff_t>(y)*cur_stride,
                               ref+static_cast<ptrdiff_t>(y)*ref_stride,
                               width,sample_stride);
    if (sample_stride>1) sad*=static_cast<uint64_t>(sample_stride*sample_stride);
    return sad;
}

uint64_t frame_sad_x86_64_v3(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride) {
    uint64_t sad=0;
    for (int y=0;y<height;y+=sample_stride) {
        const int sy=std::clamp(y+dy,0,height-1);
        const uint8_t* a=cur+static_cast<ptrdiff_t>(y)*cur_stride;
        const uint8_t* b=ref+static_cast<ptrdiff_t>(sy)*ref_stride;
        int x=0;
        while (x<width && x+dx<0) {
            sad += static_cast<uint64_t>(a[x]>b[0]?a[x]-b[0]:b[0]-a[x]);
            x += sample_stride;
        }
        const int last_unclamped=std::min(width-1,width-1-dx);
        if (x<=last_unclamped) {
            const int samples=(last_unclamped-x)/sample_stride+1;
            const int span=(samples-1)*sample_stride+1;
            const int vecspan=(span/32)*32;
            if (vecspan>=32) {
                sad += sad_row_sampled(a+x,b+x+dx,vecspan,sample_stride);
                x += vecspan;
            }
            while (x<=last_unclamped) {
                sad += static_cast<uint64_t>(a[x]>b[x+dx]?a[x]-b[x+dx]:b[x+dx]-a[x]);
                x += sample_stride;
            }
        }
        for (;x<width;x+=sample_stride)
            sad += static_cast<uint64_t>(a[x]>b[width-1]?a[x]-b[width-1]:b[width-1]-a[x]);
    }
    if (sample_stride>1) sad*=static_cast<uint64_t>(sample_stride*sample_stride);
    return sad;
}

void forward_transform_intra8_x86_64_v3(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma) {
    __m256d lo[8],hi[8];
    load_intra_rows(src,stride,lo,hi);
    forward8_from_rows(lo,hi,coeff64,use_fma);
}

void forward_transform_residual8_x86_64_v3(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma) {
    __m256d lo[8],hi[8];
    load_residual_rows(src,pred,stride,lo,hi);
    forward8_from_rows(lo,hi,coeff64,use_fma);
}


static inline __m256i mul_i32(__m256i v,int c) {
    return _mm256_mullo_epi32(v,_mm256_set1_epi32(c));
}

void inverse_transform_8x8_x86_64_v3(int* block64) {
    // First 1-D pass: process all eight input columns in parallel.  stage[v]
    // contains vertical output v for all eight horizontal positions.
    const __m256i r0=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+0));
    const __m256i r1=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+8));
    const __m256i r2=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+16));
    const __m256i r3=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+24));
    const __m256i r4=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+32));
    const __m256i r5=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+40));
    const __m256i r6=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+48));
    const __m256i r7=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+56));
    const __m256i four=_mm256_set1_epi32(4);
    __m256i t1=_mm256_add_epi32(mul_i32(_mm256_add_epi32(r0,r4),12),four);
    __m256i t2=_mm256_add_epi32(mul_i32(_mm256_sub_epi32(r0,r4),12),four);
    __m256i t3=_mm256_add_epi32(mul_i32(r2,16),mul_i32(r6,6));
    __m256i t4=_mm256_sub_epi32(mul_i32(r2,6),mul_i32(r6,16));
    const __m256i t5=_mm256_add_epi32(t1,t3);
    const __m256i t6=_mm256_add_epi32(t2,t4);
    const __m256i t7=_mm256_sub_epi32(t2,t4);
    const __m256i t8=_mm256_sub_epi32(t1,t3);
    t1=_mm256_add_epi32(_mm256_add_epi32(mul_i32(r1,16),mul_i32(r3,15)),
                        _mm256_add_epi32(mul_i32(r5,9),mul_i32(r7,4)));
    t2=_mm256_sub_epi32(_mm256_sub_epi32(mul_i32(r1,15),mul_i32(r3,4)),
                        _mm256_add_epi32(mul_i32(r5,16),mul_i32(r7,9)));
    t3=_mm256_add_epi32(_mm256_sub_epi32(mul_i32(r1,9),mul_i32(r3,16)),
                        _mm256_add_epi32(mul_i32(r5,4),mul_i32(r7,15)));
    t4=_mm256_sub_epi32(_mm256_add_epi32(_mm256_sub_epi32(mul_i32(r1,4),mul_i32(r3,9)),mul_i32(r5,15)),
                        mul_i32(r7,16));
    alignas(32) int stage[8][8];
    _mm256_store_si256(reinterpret_cast<__m256i*>(stage[0]),_mm256_srai_epi32(_mm256_add_epi32(t5,t1),3));
    _mm256_store_si256(reinterpret_cast<__m256i*>(stage[1]),_mm256_srai_epi32(_mm256_add_epi32(t6,t2),3));
    _mm256_store_si256(reinterpret_cast<__m256i*>(stage[2]),_mm256_srai_epi32(_mm256_add_epi32(t7,t3),3));
    _mm256_store_si256(reinterpret_cast<__m256i*>(stage[3]),_mm256_srai_epi32(_mm256_add_epi32(t8,t4),3));
    _mm256_store_si256(reinterpret_cast<__m256i*>(stage[4]),_mm256_srai_epi32(_mm256_sub_epi32(t8,t4),3));
    _mm256_store_si256(reinterpret_cast<__m256i*>(stage[5]),_mm256_srai_epi32(_mm256_sub_epi32(t7,t3),3));
    _mm256_store_si256(reinterpret_cast<__m256i*>(stage[6]),_mm256_srai_epi32(_mm256_sub_epi32(t6,t2),3));
    _mm256_store_si256(reinterpret_cast<__m256i*>(stage[7]),_mm256_srai_epi32(_mm256_sub_epi32(t5,t1),3));

    // Second pass: lanes are the eight first-pass outputs.  Build each input
    // horizontal position as a lane vector, then apply the same integer math.
    auto col=[&](int x) {
        return _mm256_setr_epi32(stage[0][x],stage[1][x],stage[2][x],stage[3][x],
                                 stage[4][x],stage[5][x],stage[6][x],stage[7][x]);
    };
    const __m256i c0=col(0),c1=col(1),c2=col(2),c3=col(3),c4=col(4),c5=col(5),c6=col(6),c7=col(7);
    const __m256i sixty4=_mm256_set1_epi32(64);
    t1=_mm256_add_epi32(mul_i32(_mm256_add_epi32(c0,c4),12),sixty4);
    t2=_mm256_add_epi32(mul_i32(_mm256_sub_epi32(c0,c4),12),sixty4);
    t3=_mm256_add_epi32(mul_i32(c2,16),mul_i32(c6,6));
    t4=_mm256_sub_epi32(mul_i32(c2,6),mul_i32(c6,16));
    const __m256i u5=_mm256_add_epi32(t1,t3);
    const __m256i u6=_mm256_add_epi32(t2,t4);
    const __m256i u7=_mm256_sub_epi32(t2,t4);
    const __m256i u8=_mm256_sub_epi32(t1,t3);
    t1=_mm256_add_epi32(_mm256_add_epi32(mul_i32(c1,16),mul_i32(c3,15)),
                        _mm256_add_epi32(mul_i32(c5,9),mul_i32(c7,4)));
    t2=_mm256_sub_epi32(_mm256_sub_epi32(mul_i32(c1,15),mul_i32(c3,4)),
                        _mm256_add_epi32(mul_i32(c5,16),mul_i32(c7,9)));
    t3=_mm256_add_epi32(_mm256_sub_epi32(mul_i32(c1,9),mul_i32(c3,16)),
                        _mm256_add_epi32(mul_i32(c5,4),mul_i32(c7,15)));
    t4=_mm256_sub_epi32(_mm256_add_epi32(_mm256_sub_epi32(mul_i32(c1,4),mul_i32(c3,9)),mul_i32(c5,15)),
                        mul_i32(c7,16));
    const __m256i one=_mm256_set1_epi32(1);
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(block64+0), _mm256_srai_epi32(_mm256_add_epi32(u5,t1),7));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(block64+8), _mm256_srai_epi32(_mm256_add_epi32(u6,t2),7));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(block64+16),_mm256_srai_epi32(_mm256_add_epi32(u7,t3),7));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(block64+24),_mm256_srai_epi32(_mm256_add_epi32(u8,t4),7));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(block64+32),_mm256_srai_epi32(_mm256_add_epi32(_mm256_sub_epi32(u8,t4),one),7));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(block64+40),_mm256_srai_epi32(_mm256_add_epi32(_mm256_sub_epi32(u7,t3),one),7));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(block64+48),_mm256_srai_epi32(_mm256_add_epi32(_mm256_sub_epi32(u6,t2),one),7));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(block64+56),_mm256_srai_epi32(_mm256_add_epi32(_mm256_sub_epi32(u5,t1),one),7));
}

static inline __m128i pack8_i32_to_u8(__m256i v) {
    const __m128i lo=_mm256_castsi256_si128(v);
    const __m128i hi=_mm256_extracti128_si256(v,1);
    const __m128i p16=_mm_packs_epi32(lo,hi);
    return _mm_packus_epi16(p16,p16);
}

void put_block8_x86_64_v3(uint8_t* dst,int stride,const int* block64,int bias) {
    const __m256i zero=_mm256_setzero_si256(),mx=_mm256_set1_epi32(255),bv=_mm256_set1_epi32(bias);
    for (int y=0;y<8;++y) {
        __m256i v=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+y*8));
        v=_mm256_add_epi32(v,bv);
        v=_mm256_min_epi32(mx,_mm256_max_epi32(zero,v));
        const __m128i p=pack8_i32_to_u8(v);
        _mm_storel_epi64(reinterpret_cast<__m128i*>(dst+static_cast<ptrdiff_t>(y)*stride),p);
    }
}

void add_block_rect_x86_64_v3(uint8_t* dst,int stride,const int* block64,int width,int height) {
    const __m256i zero=_mm256_setzero_si256(),mx=_mm256_set1_epi32(255);
    for (int y=0;y<height;++y) {
        if (width==8) {
            uint64_t q; std::memcpy(&q,dst+static_cast<ptrdiff_t>(y)*stride,sizeof(q));
            const __m128i b=_mm_cvtsi64_si128(static_cast<long long>(q));
            __m256i pixels=_mm256_cvtepu8_epi32(b);
            __m256i r=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(block64+y*8));
            r=_mm256_add_epi32(r,pixels);
            r=_mm256_min_epi32(mx,_mm256_max_epi32(zero,r));
            const __m128i packed=pack8_i32_to_u8(r);
            _mm_storel_epi64(reinterpret_cast<__m128i*>(dst+static_cast<ptrdiff_t>(y)*stride),packed);
        } else {
            uint32_t q; std::memcpy(&q,dst+static_cast<ptrdiff_t>(y)*stride,sizeof(q));
            const __m128i b=_mm_cvtsi32_si128(static_cast<int>(q));
            __m128i pixels=_mm_cvtepu8_epi32(b);
            __m128i r=_mm_loadu_si128(reinterpret_cast<const __m128i*>(block64+y*8));
            r=_mm_add_epi32(r,pixels);
            r=_mm_min_epi32(_mm_set1_epi32(255),_mm_max_epi32(_mm_setzero_si128(),r));
            const __m128i p16=_mm_packs_epi32(r,r);
            const __m128i p8=_mm_packus_epi16(p16,p16);
            q=static_cast<uint32_t>(_mm_cvtsi128_si32(p8));
            std::memcpy(dst+static_cast<ptrdiff_t>(y)*stride,&q,sizeof(q));
        }
    }
}


namespace {

static inline __m256i load_u8x8_i32(const uint8_t* p) {
    const __m128i q=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(p));
    return _mm256_cvtepu8_epi32(q);
}

static inline __m256i filter4_i32(__m256i a,__m256i b,__m256i c,__m256i d,int mode) {
    if (mode==1) {
        return _mm256_add_epi32(
            _mm256_add_epi32(_mm256_mullo_epi32(a,_mm256_set1_epi32(-4)),
                             _mm256_mullo_epi32(b,_mm256_set1_epi32(53))),
            _mm256_add_epi32(_mm256_mullo_epi32(c,_mm256_set1_epi32(18)),
                             _mm256_mullo_epi32(d,_mm256_set1_epi32(-3))));
    }
    if (mode==2) {
        return _mm256_add_epi32(
            _mm256_sub_epi32(_mm256_mullo_epi32(b,_mm256_set1_epi32(9)),a),
            _mm256_sub_epi32(_mm256_mullo_epi32(c,_mm256_set1_epi32(9)),d));
    }
    if (mode==3) {
        return _mm256_add_epi32(
            _mm256_add_epi32(_mm256_mullo_epi32(a,_mm256_set1_epi32(-3)),
                             _mm256_mullo_epi32(b,_mm256_set1_epi32(18))),
            _mm256_add_epi32(_mm256_mullo_epi32(c,_mm256_set1_epi32(53)),
                             _mm256_mullo_epi32(d,_mm256_set1_epi32(-4))));
    }
    return b;
}

static inline __m256i arshift_i32(__m256i v,int shift) {
    switch (shift) {
        case 1:return _mm256_srai_epi32(v,1);
        case 2:return _mm256_srai_epi32(v,2);
        case 3:return _mm256_srai_epi32(v,3);
        case 4:return _mm256_srai_epi32(v,4);
        case 5:return _mm256_srai_epi32(v,5);
        case 6:return _mm256_srai_epi32(v,6);
        case 7:return _mm256_srai_epi32(v,7);
        default:return v;
    }
}

static inline __m256i clamp_u8_i32(__m256i v) {
    return _mm256_min_epi32(_mm256_set1_epi32(255),
                            _mm256_max_epi32(_mm256_setzero_si256(),v));
}

static inline uint64_t hsum8_i32(__m256i v) {
    __m128i lo=_mm256_castsi256_si128(v);
    __m128i hi=_mm256_extracti128_si256(v,1);
    __m128i s=_mm_add_epi32(lo,hi);
    s=_mm_hadd_epi32(s,s);
    s=_mm_hadd_epi32(s,s);
    return static_cast<uint64_t>(static_cast<uint32_t>(_mm_cvtsi128_si32(s)));
}

static inline uint64_t sad_pred8(const uint8_t* cur,__m256i pred) {
    const __m256i cv=load_u8x8_i32(cur);
    return hsum8_i32(_mm256_abs_epi32(_mm256_sub_epi32(cv,pred)));
}

static inline void store_pred8(uint8_t* dst,__m256i pred) {
    const __m128i p=pack8_i32_to_u8(clamp_u8_i32(pred));
    _mm_storel_epi64(reinterpret_cast<__m128i*>(dst),p);
}

static inline void avg_store_pred8(uint8_t* dst,__m256i pred) {
    pred=clamp_u8_i32(pred);
    const __m256i old=load_u8x8_i32(dst);
    const __m256i avg=_mm256_srli_epi32(_mm256_add_epi32(_mm256_add_epi32(old,pred),_mm256_set1_epi32(1)),1);
    store_pred8(dst,avg);
}

static inline __m256i luma_pred8_1d(const uint8_t* center,int stride,int hm,int vm,bool rnd) {
    if (!hm && !vm) return load_u8x8_i32(center);
    if (vm) {
        const __m256i a=load_u8x8_i32(center-stride);
        const __m256i b=load_u8x8_i32(center);
        const __m256i c=load_u8x8_i32(center+stride);
        const __m256i d=load_u8x8_i32(center+2*stride);
        __m256i raw=filter4_i32(a,b,c,d,vm);
        const int bias=(vm==2)?(7+(rnd?1:0)):(31+(rnd?1:0));
        return clamp_u8_i32(arshift_i32(_mm256_add_epi32(raw,_mm256_set1_epi32(bias)),vm==2?4:6));
    }
    const __m256i a=load_u8x8_i32(center-1);
    const __m256i b=load_u8x8_i32(center);
    const __m256i c=load_u8x8_i32(center+1);
    const __m256i d=load_u8x8_i32(center+2);
    __m256i raw=filter4_i32(a,b,c,d,hm);
    const int bias=(hm==2)?(8-(rnd?1:0)):(32-(rnd?1:0));
    return clamp_u8_i32(arshift_i32(_mm256_add_epi32(raw,_mm256_set1_epi32(bias)),hm==2?4:6));
}

static inline __m256i vertical_stage8(const uint8_t* center,int stride,int vm,int shift,int r1) {
    __m256i raw=filter4_i32(load_u8x8_i32(center-stride),load_u8x8_i32(center),
                             load_u8x8_i32(center+stride),load_u8x8_i32(center+2*stride),vm);
    return arshift_i32(_mm256_add_epi32(raw,_mm256_set1_epi32(r1)),shift);
}

static inline int filter4_scalar(int a,int b,int c,int d,int mode) {
    if (mode==1) return -4*a+53*b+18*c-3*d;
    if (mode==2) return -a+9*b+9*c-d;
    if (mode==3) return -3*a+18*b+53*c-4*d;
    return b;
}

static inline int arshift_scalar(int v,int s) {
    if (v>=0) return v>>s;
    return -static_cast<int>((static_cast<unsigned>(-static_cast<int64_t>(v)) + ((1u<<s)-1u))>>s);
}

static inline __m256i luma_pred8_2d(const uint8_t* center,int stride,int hm,int vm,bool rnd) {
    static constexpr int shift_value[4]={0,5,1,5};
    const int shift=(shift_value[hm]+shift_value[vm])>>1;
    const int r1=(1<<(shift-1))+(rnd?1:0)-1;
    alignas(32) int t[11];
    // SIMD the eight central vertical filter locations (-1..6), then compute
    // only the three right halo values scalar. This avoids unsafe byte overread
    // at right/bottom picture borders while leaving the hot body vectorized.
    const __m256i v=vertical_stage8(center-1,stride,vm,shift,r1);
    _mm256_store_si256(reinterpret_cast<__m256i*>(t),v);
    for (int pos=7;pos<=9;++pos) {
        const uint8_t* p=center+pos;
        const int raw=filter4_scalar(p[-stride],p[0],p[stride],p[2*stride],vm);
        t[pos+1]=arshift_scalar(raw+r1,shift);
    }
    return clamp_u8_i32(arshift_i32(_mm256_add_epi32(
        filter4_i32(_mm256_loadu_si256(reinterpret_cast<const __m256i*>(t+0)),
                    _mm256_loadu_si256(reinterpret_cast<const __m256i*>(t+1)),
                    _mm256_loadu_si256(reinterpret_cast<const __m256i*>(t+2)),
                    _mm256_loadu_si256(reinterpret_cast<const __m256i*>(t+3)),hm),
        _mm256_set1_epi32(64-(rnd?1:0))),7));
}

static inline __m256i luma_pred8_qpel(const uint8_t* center,int stride,int hm,int vm,bool rnd) {
    if (hm && vm) return luma_pred8_2d(center,stride,hm,vm,rnd);
    return luma_pred8_1d(center,stride,hm,vm,rnd);
}

static inline __m256i luma_pred8_bilinear(const uint8_t* center,int stride,bool fx,bool fy,bool rnd) {
    const __m256i a=load_u8x8_i32(center);
    if (!fx && !fy) return a;
    if (fx && fy) {
        const __m256i b=load_u8x8_i32(center+1);
        const __m256i c=load_u8x8_i32(center+stride);
        const __m256i d=load_u8x8_i32(center+stride+1);
        return _mm256_srli_epi32(_mm256_add_epi32(
            _mm256_add_epi32(_mm256_add_epi32(a,b),_mm256_add_epi32(c,d)),
            _mm256_set1_epi32(rnd?1:2)),2);
    }
    const __m256i b=load_u8x8_i32(center+(fx?1:stride));
    return _mm256_srli_epi32(_mm256_add_epi32(_mm256_add_epi32(a,b),_mm256_set1_epi32(rnd?0:1)),1);
}

static inline __m256i chroma_pred8(const uint8_t* center,int stride,int fx,int fy,bool rnd) {
    const __m256i a=load_u8x8_i32(center);
    if (!fx && !fy) return a;
    const __m256i rounding=_mm256_set1_epi32(rnd?28:32);

    // Do not speculatively load zero-weight neighbours.  At the visible/padded
    // right and bottom borders the scalar law is allowed to use a one-axis
    // interpolation without a sample on the other axis.  Loading all four
    // bilinear neighbours here used to read one row/column beyond the plane
    // whenever exactly one of fx/fy was zero, which can cross an allocation
    // boundary and fault despite the unused neighbour having weight zero.
    if (!fy) {
        const __m256i b=load_u8x8_i32(center+1);
        __m256i sum=_mm256_mullo_epi32(a,_mm256_set1_epi32((8-fx)*8));
        sum=_mm256_add_epi32(sum,_mm256_mullo_epi32(b,_mm256_set1_epi32(fx*8)));
        return _mm256_srli_epi32(_mm256_add_epi32(sum,rounding),6);
    }
    if (!fx) {
        const __m256i c=load_u8x8_i32(center+stride);
        __m256i sum=_mm256_mullo_epi32(a,_mm256_set1_epi32((8-fy)*8));
        sum=_mm256_add_epi32(sum,_mm256_mullo_epi32(c,_mm256_set1_epi32(fy*8)));
        return _mm256_srli_epi32(_mm256_add_epi32(sum,rounding),6);
    }

    const __m256i b=load_u8x8_i32(center+1);
    const __m256i c=load_u8x8_i32(center+stride);
    const __m256i d=load_u8x8_i32(center+stride+1);
    const int ax=(8-fx)*(8-fy),bx=fx*(8-fy),cx=(8-fx)*fy,dx=fx*fy;
    __m256i sum=_mm256_mullo_epi32(a,_mm256_set1_epi32(ax));
    sum=_mm256_add_epi32(sum,_mm256_mullo_epi32(b,_mm256_set1_epi32(bx)));
    sum=_mm256_add_epi32(sum,_mm256_mullo_epi32(c,_mm256_set1_epi32(cx)));
    sum=_mm256_add_epi32(sum,_mm256_mullo_epi32(d,_mm256_set1_epi32(dx)));
    return _mm256_srli_epi32(_mm256_add_epi32(sum,rounding),6);
}

} // namespace

uint64_t block_sad_luma_qpel_x86_64_v3(const uint8_t* cur,int cur_stride,
                                        const uint8_t* ref_center,int ref_stride,
                                        int width,int height,int hm,int vm,bool rnd) {
    uint64_t sad=0;
    for (int y=0;y<height;++y) {
        for (int x=0;x<width;x+=8) {
            const __m256i pred=luma_pred8_qpel(ref_center+static_cast<ptrdiff_t>(y)*ref_stride+x,ref_stride,hm,vm,rnd);
            sad+=sad_pred8(cur+static_cast<ptrdiff_t>(y)*cur_stride+x,pred);
        }
    }
    return sad;
}

uint64_t block_sad_luma_bilinear_x86_64_v3(const uint8_t* cur,int cur_stride,
                                            const uint8_t* ref_center,int ref_stride,
                                            int width,int height,bool fx,bool fy,bool rnd) {
    uint64_t sad=0;
    for (int y=0;y<height;++y) for (int x=0;x<width;x+=8) {
        const __m256i pred=luma_pred8_bilinear(ref_center+static_cast<ptrdiff_t>(y)*ref_stride+x,ref_stride,fx,fy,rnd);
        sad+=sad_pred8(cur+static_cast<ptrdiff_t>(y)*cur_stride+x,pred);
    }
    return sad;
}

void luma_mc_block_x86_64_v3(uint8_t* dst,int dst_stride,const uint8_t* ref_center,int ref_stride,
                             int width,int height,int hm,int vm,bool rnd,bool bilinear) {
    for (int y=0;y<height;++y) for (int x=0;x<width;x+=8) {
        const uint8_t* r=ref_center+static_cast<ptrdiff_t>(y)*ref_stride+x;
        const __m256i pred=bilinear?luma_pred8_bilinear(r,ref_stride,hm!=0,vm!=0,rnd)
                                     :luma_pred8_qpel(r,ref_stride,hm,vm,rnd);
        store_pred8(dst+static_cast<ptrdiff_t>(y)*dst_stride+x,pred);
    }
}

void luma_mc_block_avg_x86_64_v3(uint8_t* dst,int dst_stride,const uint8_t* ref_center,int ref_stride,
                                 int width,int height,int hm,int vm,bool rnd,bool bilinear) {
    for (int y=0;y<height;++y) for (int x=0;x<width;x+=8) {
        const uint8_t* r=ref_center+static_cast<ptrdiff_t>(y)*ref_stride+x;
        const __m256i pred=bilinear?luma_pred8_bilinear(r,ref_stride,hm!=0,vm!=0,rnd)
                                     :luma_pred8_qpel(r,ref_stride,hm,vm,rnd);
        avg_store_pred8(dst+static_cast<ptrdiff_t>(y)*dst_stride+x,pred);
    }
}

void chroma_mc_block_x86_64_v3(uint8_t* dst,int dst_stride,const uint8_t* ref_center,int ref_stride,
                               int width,int height,int fx,int fy,bool rnd) {
    for (int y=0;y<height;++y) for (int x=0;x<width;x+=8)
        store_pred8(dst+static_cast<ptrdiff_t>(y)*dst_stride+x,
                    chroma_pred8(ref_center+static_cast<ptrdiff_t>(y)*ref_stride+x,ref_stride,fx,fy,rnd));
}

void chroma_mc_block_avg_x86_64_v3(uint8_t* dst,int dst_stride,const uint8_t* ref_center,int ref_stride,
                                   int width,int height,int fx,int fy,bool rnd) {
    for (int y=0;y<height;++y) for (int x=0;x<width;x+=8)
        avg_store_pred8(dst+static_cast<ptrdiff_t>(y)*dst_stride+x,
                        chroma_pred8(ref_center+static_cast<ptrdiff_t>(y)*ref_stride+x,ref_stride,fx,fy,rnd));
}

void intensity_map_plane_x86_64_v3(uint8_t* dst,const uint8_t* src,size_t count,int scale,int shift) {
    size_t i=0;
    const __m256i sv=_mm256_set1_epi32(scale),sh=_mm256_set1_epi32(shift+32);
    for (;i+8<=count;i+=8) {
        __m256i v=load_u8x8_i32(src+i);
        v=_mm256_srai_epi32(_mm256_add_epi32(_mm256_mullo_epi32(v,sv),sh),6);
        store_pred8(dst+i,v);
    }
    for (;i<count;++i) dst[i]=static_cast<uint8_t>(std::clamp((scale*static_cast<int>(src[i])+shift+32)>>6,0,255));
}

void intensity_map_chroma_plane_x86_64_v3(uint8_t* dst,const uint8_t* src,size_t count,int scale) {
    size_t i=0;
    const __m256i sv=_mm256_set1_epi32(scale),bias=_mm256_set1_epi32(128*64+32),c128=_mm256_set1_epi32(128);
    for (;i+8<=count;i+=8) {
        __m256i v=_mm256_sub_epi32(load_u8x8_i32(src+i),c128);
        v=_mm256_srai_epi32(_mm256_add_epi32(_mm256_mullo_epi32(v,sv),bias),6);
        store_pred8(dst+i,v);
    }
    for (;i<count;++i) dst[i]=static_cast<uint8_t>(std::clamp((scale*(static_cast<int>(src[i])-128)+128*64+32)>>6,0,255));
}

void forward_transform_residual_rect_x86_64_v3(const uint8_t* src,const uint8_t* pred,int stride,
                                                int width,int height,double* coeff64,bool use_fma) {
    alignas(32) double rows[8][8]{};
    alignas(32) double tmp[8][8]{};
    for (int y=0;y<height;++y) {
        for (int x=0;x<width;x+=4) {
            const __m256d a=load_u8x4_as_double(src+static_cast<ptrdiff_t>(y)*stride+x);
            const __m256d b=load_u8x4_as_double(pred+static_cast<ptrdiff_t>(y)*stride+x);
            _mm256_store_pd(rows[y]+x,_mm256_sub_pd(a,b));
        }
    }
    const double (*vm8)[8]=forward_transform::k8;
    for (int a=0;a<height;++a) {
        for (int x=0;x<width;x+=4) {
            __m256d acc=_mm256_setzero_pd();
            for (int y=0;y<height;++y) {
                const double k=height==8?vm8[a][y]:forward_transform::k4[a][y];
                const __m256d r=_mm256_load_pd(rows[y]+x);
                const __m256d kv=_mm256_set1_pd(k);
                acc=use_fma?_mm256_fmadd_pd(kv,r,acc):_mm256_add_pd(acc,_mm256_mul_pd(kv,r));
            }
            _mm256_store_pd(tmp[a]+x,acc);
        }
    }
    std::fill(coeff64,coeff64+64,0.0);
    for (int a=0;a<height;++a) {
        for (int bb=0;bb<width;bb+=4) {
            __m256d acc=_mm256_setzero_pd();
            for (int x=0;x<width;++x) {
                __m256d kv;
                if (width==8) kv=_mm256_setr_pd(forward_transform::k8[bb+0][x],forward_transform::k8[bb+1][x],forward_transform::k8[bb+2][x],forward_transform::k8[bb+3][x]);
                else kv=_mm256_setr_pd(forward_transform::k4[0][x],forward_transform::k4[1][x],forward_transform::k4[2][x],forward_transform::k4[3][x]);
                const __m256d tv=_mm256_set1_pd(tmp[a][x]);
                acc=use_fma?_mm256_fmadd_pd(tv,kv,acc):_mm256_add_pd(acc,_mm256_mul_pd(tv,kv));
            }
            acc=_mm256_mul_pd(acc,_mm256_set1_pd(1024.0));
            _mm256_storeu_pd(coeff64+static_cast<size_t>(a)*8+bb,acc);
        }
    }
}


void quantize_coefficients_x86_64_v3(const double* coeff,int* out,int count,
                                     double qscale,double offset,int max_level) {
    const __m256d signbit=_mm256_set1_pd(-0.0);
    const __m256d qv=_mm256_set1_pd(qscale),ov=_mm256_set1_pd(offset);
    const __m256d one=_mm256_set1_pd(1.0),mx=_mm256_set1_pd(static_cast<double>(max_level));
    const __m256d zero=_mm256_setzero_pd();
    int i=0;
    for (;i+4<=count;i+=4) {
        const __m256d c=_mm256_loadu_pd(coeff+i);
        const __m256d a=_mm256_andnot_pd(signbit,c);
        __m256d n=_mm256_div_pd(_mm256_sub_pd(a,ov),qv);
        // scalar_quantize_level() computes llround() and then evaluates the
        // adjacent lower level first; exact half-step ties therefore settle on
        // the lower magnitude. ceil(x-0.5) expresses that same decision law
        // directly and avoids the round-to-even drift of _mm256_round_pd().
        n=_mm256_ceil_pd(_mm256_sub_pd(n,_mm256_set1_pd(0.5)));
        n=_mm256_min_pd(mx,_mm256_max_pd(one,n));
        const __m256d recon=_mm256_add_pd(_mm256_mul_pd(n,qv),ov);
        const __m256d e=_mm256_sub_pd(a,recon);
        const __m256d err=_mm256_mul_pd(e,e);
        const __m256d err0=_mm256_mul_pd(a,a);
        const __m256d keep=_mm256_cmp_pd(err,err0,_CMP_LT_OQ);
        const __m256d neg=_mm256_cmp_pd(c,zero,_CMP_LT_OQ);
        const __m256d signedn=_mm256_blendv_pd(n,_mm256_sub_pd(zero,n),neg);
        const __m256d selected=_mm256_blendv_pd(zero,signedn,keep);
        const __m128i qi=_mm256_cvttpd_epi32(selected);
        _mm_storeu_si128(reinterpret_cast<__m128i*>(out+i),qi);
    }
    for (;i<count;++i) {
        const double c=coeff[i],a=std::abs(c);
        if (a==0.0) { out[i]=0; continue; }
        int n=static_cast<int>(std::ceil((a-offset)/qscale-0.5));
        n=std::clamp(n,1,max_level);
        const double e=a-(static_cast<double>(n)*qscale+offset);
        out[i]=(e*e<a*a)?(c<0?-n:n):0;
    }
}


uint64_t sum_u8_x86_64_v3(const uint8_t* src,size_t count) {
    uint64_t total=0;
    const __m256i zero=_mm256_setzero_si256();
    size_t i=0;
    for (;i+32<=count;i+=32) {
        const __m256i v=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(src+i));
        total += hsum256_epi64(_mm256_sad_epu8(v,zero));
    }
    if (i+16<=count) {
        const __m128i v=_mm_loadu_si128(reinterpret_cast<const __m128i*>(src+i));
        total += hsum128_epi64(_mm_sad_epu8(v,_mm_setzero_si128()));
        i+=16;
    }
    for (;i<count;++i) total+=src[i];
    return total;
}

static inline uint64_t sum_abs_u8_128(__m128i a,__m128i b) {
    return hsum128_epi64(_mm_sad_epu8(a,b));
}
static inline int count_ge_u8_128(__m128i d,int threshold,unsigned valid_mask) {
    const __m128i t=_mm_set1_epi8(static_cast<char>(threshold));
    const __m128i ge=_mm_cmpeq_epi8(_mm_max_epu8(d,t),d);
    const unsigned bits=static_cast<unsigned>(_mm_movemask_epi8(ge)) & valid_mask;
    return __builtin_popcount(bits);
}

void perceptual_stats_x86_64_v3(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count) {
    uint64_t s=0,g=0,r=0;
    int mn=255,mx=0,e=0,gn=0;
    const __m128i zero=_mm_setzero_si128();
    const __m128i low8=_mm_set_epi64x(0,-1ll);
    const __m128i high8ff=_mm_set_epi64x(-1ll,0);
    const __m128i lane0_clear=_mm_setr_epi8(0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1);
    __m128i vmin=_mm_set1_epi8(static_cast<char>(0xff));
    __m128i vmax=zero;
    bool have_vector=false;

    for (int y=0;y<height;++y) {
        const uint8_t* row=src+static_cast<ptrdiff_t>(y)*stride;
        const uint8_t* prow=pred?pred+static_cast<ptrdiff_t>(y)*stride:nullptr;
        int x=0;
        for (;x+16<=width;x+=16) {
            const __m128i v=_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+x));
            have_vector=true;
            vmin=_mm_min_epu8(vmin,v); vmax=_mm_max_epu8(vmax,v);
            s += sum_abs_u8_128(v,zero);
            if (prow) r += sum_abs_u8_128(v,_mm_loadu_si128(reinterpret_cast<const __m128i*>(prow+x)));

            __m128i left;
            unsigned valid=0xffffu;
            if (x==0) {
                left=_mm_slli_si128(v,1);
                valid &= ~1u;
            } else {
                left=_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+x-1));
            }
            __m128i hd=_mm_sub_epi8(_mm_max_epu8(v,left),_mm_min_epu8(v,left));
            if (x==0) hd=_mm_and_si128(hd,lane0_clear);
            g += sum_abs_u8_128(hd,zero);
            e += count_ge_u8_128(hd,96,valid);
            gn += __builtin_popcount(valid);

            if (y>0) {
                const __m128i top=_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+x-stride));
                const __m128i vd=_mm_sub_epi8(_mm_max_epu8(v,top),_mm_min_epu8(v,top));
                g += sum_abs_u8_128(vd,zero);
                e += count_ge_u8_128(vd,96,0xffffu);
                gn += 16;
            }
        }
        // Chroma and 4-point transform regions are commonly eight pixels wide.
        // The first implementation sent all of those regions down the scalar
        // tail, which left a large fraction of AQ/perceptual work unvectorized.
        if (x+8<=width) {
            const __m128i v=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row+x));
            have_vector=true;
            vmin=_mm_min_epu8(vmin,_mm_or_si128(v,high8ff));
            vmax=_mm_max_epu8(vmax,v);
            s += sum_abs_u8_128(v,zero);
            if (prow) r += sum_abs_u8_128(v,_mm_loadl_epi64(reinterpret_cast<const __m128i*>(prow+x)));

            __m128i left;
            unsigned valid=0xffu;
            if (x==0) {
                left=_mm_slli_si128(v,1);
                valid &= ~1u;
            } else {
                left=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row+x-1));
            }
            __m128i hd=_mm_sub_epi8(_mm_max_epu8(v,left),_mm_min_epu8(v,left));
            hd=_mm_and_si128(hd,low8);
            if (x==0) hd=_mm_and_si128(hd,lane0_clear);
            g += sum_abs_u8_128(hd,zero);
            e += count_ge_u8_128(hd,96,valid);
            gn += __builtin_popcount(valid);

            if (y>0) {
                const __m128i top=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row+x-stride));
                const __m128i vd=_mm_and_si128(
                    _mm_sub_epi8(_mm_max_epu8(v,top),_mm_min_epu8(v,top)),low8);
                g += sum_abs_u8_128(vd,zero);
                e += count_ge_u8_128(vd,96,0xffu);
                gn += 8;
            }
            x+=8;
        }
        for (;x<width;++x) {
            const int v=row[x]; s+=v; mn=std::min(mn,v); mx=std::max(mx,v);
            if (prow) r+=static_cast<uint64_t>(std::abs(v-static_cast<int>(prow[x])));
            if (x>0) { const int d=std::abs(v-static_cast<int>(row[x-1])); g+=d; ++gn; if (d>=96) ++e; }
            if (y>0) { const int d=std::abs(v-static_cast<int>(row[x-stride])); g+=d; ++gn; if (d>=96) ++e; }
        }
    }
    if (have_vector) {
        alignas(16) uint8_t mins[16],maxs[16];
        _mm_store_si128(reinterpret_cast<__m128i*>(mins),vmin);
        _mm_store_si128(reinterpret_cast<__m128i*>(maxs),vmax);
        for (int i=0;i<16;++i) { mn=std::min(mn,static_cast<int>(mins[i])); mx=std::max(mx,static_cast<int>(maxs[i])); }
    }
    *sum=s; *grad=g; *residual=r; *min_value=mn; *max_value=mx; *edges=e; *grad_count=gn;
}

uint64_t satd4x4_x86_64_v3(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride){return detail::satd4x4_target(cur,cur_stride,pred,pred_stride);}


void dequant_coefficients_x86_64_v3(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform){detail::dequant_coefficients_target(q,out,count,double_quant,mquant,nonuniform);}
void inverse_transform_rect_x86_64_v3(int* block64,int type){detail::inverse_transform_rect_target(block64,type);}
void overlap_vertical8_x86_64_v3(int* edge,int stride){detail::overlap_vertical8_target(edge,stride);}
void overlap_horizontal8_x86_64_v3(int* edge,int stride){detail::overlap_horizontal8_target(edge,stride);}
bool loop_filter4_h_x86_64_v3(uint8_t* p,int stride,int x,int y,int pq){return detail::loop_filter4_h_target(p,stride,x,y,pq);}
bool loop_filter4_v_x86_64_v3(uint8_t* p,int stride,int x,int y,int pq){return detail::loop_filter4_v_target(p,stride,x,y,pq);}

} // namespace libvc1::simd
