#pragma once
#include <immintrin.h>
#include <algorithm>
#include <cstdint>
#include <cstring>

namespace libvc1::simd::detail {

static inline __m128i abs_epi32_sse2(__m128i v){const __m128i s=_mm_srai_epi32(v,31);return _mm_sub_epi32(_mm_xor_si128(v,s),s);}
static inline __m128i min_epi32_sse2(__m128i a,__m128i b){const __m128i m=_mm_cmpgt_epi32(a,b);return _mm_or_si128(_mm_and_si128(m,b),_mm_andnot_si128(m,a));}
static inline __m128i select_epi32(__m128i m,__m128i a,__m128i b){return _mm_or_si128(_mm_and_si128(m,a),_mm_andnot_si128(m,b));}
static inline __m128i load_u8x4_i32_recon(const uint8_t* p){uint32_t q;std::memcpy(&q,p,4);const __m128i b=_mm_cvtsi32_si128(static_cast<int>(q)),z=_mm_setzero_si128();return _mm_unpacklo_epi16(_mm_unpacklo_epi8(b,z),z);}
static inline void store_i32x4_u8_recon(uint8_t* p,__m128i v){
    const __m128i z=_mm_setzero_si128(),hi=_mm_set1_epi32(255);
    v=_mm_andnot_si128(_mm_cmpgt_epi32(z,v),v);
    const __m128i over=_mm_cmpgt_epi32(v,hi);v=select_epi32(over,hi,v);
    const __m128i p16=_mm_packs_epi32(v,v),p8=_mm_packus_epi16(p16,p16);
    const uint32_t q=static_cast<uint32_t>(_mm_cvtsi128_si32(p8));std::memcpy(p,&q,4);
}

static inline void transpose4_epi32(__m128i& r0,__m128i& r1,__m128i& r2,__m128i& r3){
    const __m128i t0=_mm_unpacklo_epi32(r0,r1),t1=_mm_unpackhi_epi32(r0,r1);
    const __m128i t2=_mm_unpacklo_epi32(r2,r3),t3=_mm_unpackhi_epi32(r2,r3);
    r0=_mm_unpacklo_epi64(t0,t2); r1=_mm_unpackhi_epi64(t0,t2);
    r2=_mm_unpacklo_epi64(t1,t3); r3=_mm_unpackhi_epi64(t1,t3);
}
static inline __m128i mul17(__m128i x){return _mm_add_epi32(_mm_slli_epi32(x,4),x);}
static inline __m128i mul22(__m128i x){return _mm_add_epi32(_mm_add_epi32(_mm_slli_epi32(x,4),_mm_slli_epi32(x,2)),_mm_slli_epi32(x,1));}
static inline __m128i mul10(__m128i x){return _mm_add_epi32(_mm_slli_epi32(x,3),_mm_slli_epi32(x,1));}
static inline __m128i mul12(__m128i x){return _mm_add_epi32(_mm_slli_epi32(x,3),_mm_slli_epi32(x,2));}
static inline __m128i mul16(__m128i x){return _mm_slli_epi32(x,4);}
static inline __m128i mul6(__m128i x){return _mm_add_epi32(_mm_slli_epi32(x,2),_mm_slli_epi32(x,1));}
static inline __m128i mul15(__m128i x){return _mm_sub_epi32(_mm_slli_epi32(x,4),x);}
static inline __m128i mul9(__m128i x){return _mm_add_epi32(_mm_slli_epi32(x,3),x);}
static inline __m128i mul4(__m128i x){return _mm_slli_epi32(x,2);}

static inline void dequant_coefficients_target(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform){
    int i=0;
#if defined(__AVX2__)
    const __m256i vdq=_mm256_set1_epi32(double_quant),vmq256=_mm256_set1_epi32(mquant),zero256=_mm256_setzero_si256();
    for(;i+8<=count;i+=8){
        const __m256i v=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(q+i));
        __m256i r=_mm256_mullo_epi32(v,vdq);
        if(nonuniform){const __m256i pos=_mm256_cmpgt_epi32(v,zero256),neg=_mm256_cmpgt_epi32(zero256,v);const __m256i adj=_mm256_sub_epi32(_mm256_and_si256(pos,vmq256),_mm256_and_si256(neg,vmq256));r=_mm256_add_epi32(r,adj);}
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(out+i),r);
    }
#endif
    const __m128i zero=_mm_setzero_si128();
    const __m128i mult16=_mm_setr_epi16(static_cast<short>(double_quant),0,static_cast<short>(double_quant),0,static_cast<short>(double_quant),0,static_cast<short>(double_quant),0);
    const __m128i vmq=_mm_set1_epi32(mquant);
    for(;i+4<=count;i+=4){
        const __m128i v=_mm_loadu_si128(reinterpret_cast<const __m128i*>(q+i));
        const __m128i q16=_mm_packs_epi32(v,v);
        const __m128i pairs=_mm_unpacklo_epi16(q16,zero);
        __m128i r=_mm_madd_epi16(pairs,mult16);
        if(nonuniform){const __m128i pos=_mm_cmpgt_epi32(v,zero),neg=_mm_cmpgt_epi32(zero,v);const __m128i adj=_mm_sub_epi32(_mm_and_si128(pos,vmq),_mm_and_si128(neg,vmq));r=_mm_add_epi32(r,adj);}
        _mm_storeu_si128(reinterpret_cast<__m128i*>(out+i),r);
    }
    for(;i<count;++i){if(!q[i]){out[i]=0;continue;}long v=static_cast<long>(q[i])*double_quant;if(nonuniform)v+=q[i]>0?mquant:-mquant;out[i]=static_cast<int>(v);}
}

static inline void inv8_row_scalar(const int* src,int* dst,int round){
    int t1=12*(src[0]+src[4])+round,t2=12*(src[0]-src[4])+round;
    int t3=16*src[2]+6*src[6],t4=6*src[2]-16*src[6];
    int t5=t1+t3,t6=t2+t4,t7=t2-t4,t8=t1-t3;
    t1=16*src[1]+15*src[3]+9*src[5]+4*src[7];
    t2=15*src[1]-4*src[3]-16*src[5]-9*src[7];
    t3=9*src[1]-16*src[3]+4*src[5]+15*src[7];
    t4=4*src[1]-9*src[3]+15*src[5]-16*src[7];
    dst[0]=(t5+t1)>>3;dst[1]=(t6+t2)>>3;dst[2]=(t7+t3)>>3;dst[3]=(t8+t4)>>3;
    dst[4]=(t8-t4)>>3;dst[5]=(t7-t3)>>3;dst[6]=(t6-t2)>>3;dst[7]=(t5-t1)>>3;
}
static inline void inv4_row_scalar(const int* src,int* dst){
    int t1=17*(src[0]+src[2])+4,t2=17*(src[0]-src[2])+4;
    int t3=22*src[1]+10*src[3],t4=22*src[3]-10*src[1];
    dst[0]=(t1+t3)>>3;dst[1]=(t2-t4)>>3;dst[2]=(t2+t4)>>3;dst[3]=(t1-t3)>>3;
}

static inline void inverse_transform_rect_target(int* a,int type){
    alignas(32) int tmp[64]{};
    if(type==1){ // 8x4
        for(int y=0;y<4;++y) inv8_row_scalar(a+y*8,tmp+y*8,4);
#if defined(__AVX2__)
        const __m256i r0=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(tmp+0));
        const __m256i r1=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(tmp+8));
        const __m256i r2=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(tmp+16));
        const __m256i r3=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(tmp+24));
        const __m256i c64=_mm256_set1_epi32(64);
        const __m256i t1=_mm256_add_epi32(_mm256_mullo_epi32(_mm256_add_epi32(r0,r2),_mm256_set1_epi32(17)),c64);
        const __m256i t2=_mm256_add_epi32(_mm256_mullo_epi32(_mm256_sub_epi32(r0,r2),_mm256_set1_epi32(17)),c64);
        const __m256i t3=_mm256_add_epi32(_mm256_mullo_epi32(r1,_mm256_set1_epi32(22)),_mm256_mullo_epi32(r3,_mm256_set1_epi32(10)));
        const __m256i t4=_mm256_sub_epi32(_mm256_mullo_epi32(r3,_mm256_set1_epi32(22)),_mm256_mullo_epi32(r1,_mm256_set1_epi32(10)));
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(a+0),_mm256_srai_epi32(_mm256_add_epi32(t1,t3),7));
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(a+8),_mm256_srai_epi32(_mm256_sub_epi32(t2,t4),7));
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(a+16),_mm256_srai_epi32(_mm256_add_epi32(t2,t4),7));
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(a+24),_mm256_srai_epi32(_mm256_sub_epi32(t1,t3),7));
        return;
#else
        for(int x=0;x<8;x+=4){
            const __m128i r0=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+x));
            const __m128i r1=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+8+x));
            const __m128i r2=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+16+x));
            const __m128i r3=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+24+x));
            const __m128i c64=_mm_set1_epi32(64);
            const __m128i t1=_mm_add_epi32(mul17(_mm_add_epi32(r0,r2)),c64),t2=_mm_add_epi32(mul17(_mm_sub_epi32(r0,r2)),c64);
            const __m128i t3=_mm_add_epi32(mul22(r1),mul10(r3)),t4=_mm_sub_epi32(mul22(r3),mul10(r1));
            _mm_storeu_si128(reinterpret_cast<__m128i*>(a+x),_mm_srai_epi32(_mm_add_epi32(t1,t3),7));
            _mm_storeu_si128(reinterpret_cast<__m128i*>(a+8+x),_mm_srai_epi32(_mm_sub_epi32(t2,t4),7));
            _mm_storeu_si128(reinterpret_cast<__m128i*>(a+16+x),_mm_srai_epi32(_mm_add_epi32(t2,t4),7));
            _mm_storeu_si128(reinterpret_cast<__m128i*>(a+24+x),_mm_srai_epi32(_mm_sub_epi32(t1,t3),7));
        } return;
#endif
    }
    const int rows=type==2?8:4;
    for(int y=0;y<rows;++y) inv4_row_scalar(a+y*8,tmp+y*8);
    if(type==3){ // 4x4
        const __m128i r0=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+0)),r1=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+8));
        const __m128i r2=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+16)),r3=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+24)),c64=_mm_set1_epi32(64);
        const __m128i t1=_mm_add_epi32(mul17(_mm_add_epi32(r0,r2)),c64),t2=_mm_add_epi32(mul17(_mm_sub_epi32(r0,r2)),c64);
        const __m128i t3=_mm_add_epi32(mul22(r1),mul10(r3)),t4=_mm_sub_epi32(mul22(r3),mul10(r1));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(a+0),_mm_srai_epi32(_mm_add_epi32(t1,t3),7));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(a+8),_mm_srai_epi32(_mm_sub_epi32(t2,t4),7));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(a+16),_mm_srai_epi32(_mm_add_epi32(t2,t4),7));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(a+24),_mm_srai_epi32(_mm_sub_epi32(t1,t3),7)); return;
    }
    // 4x8: one four-lane vector covers all columns.
    const __m128i r0=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+0)),r1=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+8));
    const __m128i r2=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+16)),r3=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+24));
    const __m128i r4=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+32)),r5=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+40));
    const __m128i r6=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+48)),r7=_mm_loadu_si128(reinterpret_cast<const __m128i*>(tmp+56));
    const __m128i c64=_mm_set1_epi32(64),one=_mm_set1_epi32(1);
    const __m128i t1=_mm_add_epi32(mul12(_mm_add_epi32(r0,r4)),c64),t2=_mm_add_epi32(mul12(_mm_sub_epi32(r0,r4)),c64);
    const __m128i t3=_mm_add_epi32(mul16(r2),mul6(r6)),t4=_mm_sub_epi32(mul6(r2),mul16(r6));
    const __m128i t5=_mm_add_epi32(t1,t3),t6=_mm_add_epi32(t2,t4),t7=_mm_sub_epi32(t2,t4),t8=_mm_sub_epi32(t1,t3);
    const __m128i u1=_mm_add_epi32(_mm_add_epi32(mul16(r1),mul15(r3)),_mm_add_epi32(mul9(r5),mul4(r7)));
    const __m128i u2=_mm_sub_epi32(_mm_sub_epi32(mul15(r1),mul4(r3)),_mm_add_epi32(mul16(r5),mul9(r7)));
    const __m128i u3=_mm_add_epi32(_mm_sub_epi32(mul9(r1),mul16(r3)),_mm_add_epi32(mul4(r5),mul15(r7)));
    const __m128i u4=_mm_sub_epi32(_mm_add_epi32(mul4(r1),mul15(r5)),_mm_add_epi32(mul9(r3),mul16(r7)));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(a+0),_mm_srai_epi32(_mm_add_epi32(t5,u1),7));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(a+8),_mm_srai_epi32(_mm_add_epi32(t6,u2),7));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(a+16),_mm_srai_epi32(_mm_add_epi32(t7,u3),7));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(a+24),_mm_srai_epi32(_mm_add_epi32(t8,u4),7));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(a+32),_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(t8,u4),one),7));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(a+40),_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(t7,u3),one),7));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(a+48),_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(t6,u2),one),7));
    _mm_storeu_si128(reinterpret_cast<__m128i*>(a+56),_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(t5,u1),one),7));
}

static inline void overlap_vertical8_target(int* edge,int stride){
    const __m128i r1=_mm_setr_epi32(4,3,4,3),r2=_mm_setr_epi32(3,4,3,4);
    for(int g=0;g<8;g+=4){
        __m128i x0=_mm_loadu_si128(reinterpret_cast<const __m128i*>(edge+g*stride));
        __m128i x1=_mm_loadu_si128(reinterpret_cast<const __m128i*>(edge+(g+1)*stride));
        __m128i x2=_mm_loadu_si128(reinterpret_cast<const __m128i*>(edge+(g+2)*stride));
        __m128i x3=_mm_loadu_si128(reinterpret_cast<const __m128i*>(edge+(g+3)*stride));
        transpose4_epi32(x0,x1,x2,x3); // a,b,c,d across four rows
        const __m128i d1=_mm_sub_epi32(x0,x3),d2=_mm_add_epi32(d1,_mm_sub_epi32(x1,x2));
        __m128i o0=_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_slli_epi32(x0,3),d1),r1),3);
        __m128i o1=_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_slli_epi32(x1,3),d2),r2),3);
        __m128i o2=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_slli_epi32(x2,3),d2),r1),3);
        __m128i o3=_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_slli_epi32(x3,3),d1),r2),3);
        transpose4_epi32(o0,o1,o2,o3);
        _mm_storeu_si128(reinterpret_cast<__m128i*>(edge+g*stride),o0);_mm_storeu_si128(reinterpret_cast<__m128i*>(edge+(g+1)*stride),o1);
        _mm_storeu_si128(reinterpret_cast<__m128i*>(edge+(g+2)*stride),o2);_mm_storeu_si128(reinterpret_cast<__m128i*>(edge+(g+3)*stride),o3);
    }
}
static inline void overlap_horizontal8_target(int* edge,int stride){
    const __m128i r1=_mm_setr_epi32(4,3,4,3),r2=_mm_setr_epi32(3,4,3,4);
    for(int x=0;x<8;x+=4){
        const __m128i a=_mm_loadu_si128(reinterpret_cast<const __m128i*>(edge-2*stride+x)),b=_mm_loadu_si128(reinterpret_cast<const __m128i*>(edge-stride+x));
        const __m128i c=_mm_loadu_si128(reinterpret_cast<const __m128i*>(edge+x)),d=_mm_loadu_si128(reinterpret_cast<const __m128i*>(edge+stride+x));
        const __m128i d1=_mm_sub_epi32(a,d),d2=_mm_add_epi32(d1,_mm_sub_epi32(b,c));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(edge-2*stride+x),_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_slli_epi32(a,3),d1),r1),3));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(edge-stride+x),_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_slli_epi32(b,3),d2),r2),3));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(edge+x),_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_slli_epi32(c,3),d2),r1),3));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(edge+stride+x),_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_slli_epi32(d,3),d1),r2),3));
    }
}

static inline __m128i loop_filter_delta4(__m128i sm4,__m128i sm3,__m128i sm2,__m128i sm1,__m128i s0,__m128i s1,__m128i s2,__m128i s3,int pq,__m128i* eligible_out){
    const __m128i four=_mm_set1_epi32(4);
    __m128i a0=_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_slli_epi32(_mm_sub_epi32(sm2,s1),1),_mm_add_epi32(_mm_slli_epi32(_mm_sub_epi32(sm1,s0),2),_mm_sub_epi32(sm1,s0))),four),3);
    const __m128i a0abs=abs_epi32_sse2(a0);
    const __m128i a1=abs_epi32_sse2(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_slli_epi32(_mm_sub_epi32(sm4,sm1),1),_mm_add_epi32(_mm_slli_epi32(_mm_sub_epi32(sm3,sm2),2),_mm_sub_epi32(sm3,sm2))),four),3));
    const __m128i a2=abs_epi32_sse2(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_slli_epi32(_mm_sub_epi32(s0,s3),1),_mm_add_epi32(_mm_slli_epi32(_mm_sub_epi32(s1,s2),2),_mm_sub_epi32(s1,s2))),four),3));
    const __m128i pqv=_mm_set1_epi32(pq),zero=_mm_setzero_si128();
    const __m128i lt_pq=_mm_cmpgt_epi32(pqv,a0abs),lt1=_mm_cmpgt_epi32(a0abs,a1),lt2=_mm_cmpgt_epi32(a0abs,a2);
    const __m128i edge=_mm_sub_epi32(sm1,s0),limit=_mm_srli_epi32(abs_epi32_sse2(edge),1);
    const __m128i nz=_mm_cmpgt_epi32(limit,zero);
    const __m128i eligible=_mm_and_si128(_mm_and_si128(lt_pq,_mm_or_si128(lt1,lt2)),nz);*eligible_out=eligible;
    const __m128i amin=min_epi32_sse2(a1,a2),diff=_mm_sub_epi32(a0abs,amin);
    __m128i delta=_mm_srai_epi32(_mm_add_epi32(_mm_slli_epi32(diff,2),diff),3);
    delta=min_epi32_sse2(delta,limit);
    const __m128i signa=_mm_srai_epi32(a0,31),signe=_mm_srai_epi32(edge,31),sign_mismatch=_mm_xor_si128(signa,signe);
    const __m128i modmask=_mm_and_si128(eligible,sign_mismatch);
    delta=_mm_and_si128(delta,modmask);
    delta=select_epi32(signe,_mm_sub_epi32(zero,delta),delta);
    return delta;
}
static inline bool loop_filter4_h_target(uint8_t* p,int stride,int x,int y,int pq){
    __m128i s[8];for(int k=-4;k<=3;++k)s[k+4]=load_u8x4_i32_recon(p+static_cast<ptrdiff_t>(y+k)*stride+x);
    __m128i eligible;const __m128i d=loop_filter_delta4(s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],pq,&eligible);
    if(!(_mm_movemask_ps(_mm_castsi128_ps(eligible))&4))return false;
    __m128i pv=load_u8x4_i32_recon(p+static_cast<ptrdiff_t>(y-1)*stride+x),qv=load_u8x4_i32_recon(p+static_cast<ptrdiff_t>(y)*stride+x);
    store_i32x4_u8_recon(p+static_cast<ptrdiff_t>(y-1)*stride+x,_mm_sub_epi32(pv,d));store_i32x4_u8_recon(p+static_cast<ptrdiff_t>(y)*stride+x,_mm_add_epi32(qv,d));return true;
}
static inline bool loop_filter4_v_target(uint8_t* p,int stride,int x,int y,int pq){
    __m128i s[8];for(int k=-4;k<=3;++k)s[k+4]=_mm_setr_epi32(p[static_cast<ptrdiff_t>(y+0)*stride+x+k],p[static_cast<ptrdiff_t>(y+1)*stride+x+k],p[static_cast<ptrdiff_t>(y+2)*stride+x+k],p[static_cast<ptrdiff_t>(y+3)*stride+x+k]);
    __m128i eligible;const __m128i d=loop_filter_delta4(s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],pq,&eligible);
    if(!(_mm_movemask_ps(_mm_castsi128_ps(eligible))&4))return false;
    alignas(16) int dv[4];_mm_store_si128(reinterpret_cast<__m128i*>(dv),d);
    for(int i=0;i<4;++i){int a=p[static_cast<ptrdiff_t>(y+i)*stride+x-1]-dv[i],b=p[static_cast<ptrdiff_t>(y+i)*stride+x]+dv[i];p[static_cast<ptrdiff_t>(y+i)*stride+x-1]=static_cast<uint8_t>(std::clamp(a,0,255));p[static_cast<ptrdiff_t>(y+i)*stride+x]=static_cast<uint8_t>(std::clamp(b,0,255));}return true;
}

} // namespace libvc1::simd::detail
