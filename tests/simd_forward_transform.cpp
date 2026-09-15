#include "simd.h"
#include "forward_transform_tables.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace {

using namespace libvc1;

bool cpu_supported() {
#if (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
#if defined(LIBVC1_TEST_SIMD_V1)
    return true;
#elif defined(LIBVC1_TEST_SIMD_PRESCOTT)
    return __builtin_cpu_supports("sse3");
#elif defined(LIBVC1_TEST_SIMD_K10)
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("sse4a") && __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("popcnt");
#elif defined(LIBVC1_TEST_SIMD_CONROE)
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3");
#elif defined(LIBVC1_TEST_SIMD_PENRYN)
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") && __builtin_cpu_supports("sse4.1");
#elif defined(LIBVC1_TEST_SIMD_V2)
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("popcnt");
#elif defined(LIBVC1_TEST_SIMD_SANDYBRIDGE)
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("avx");
#elif defined(LIBVC1_TEST_SIMD_BULLDOZER)
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") && __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") && __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("avx") && __builtin_cpu_supports("xop") && __builtin_cpu_supports("fma4");
#elif defined(LIBVC1_TEST_SIMD_PILEDRIVER)
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") && __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") && __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("avx") && __builtin_cpu_supports("xop") && __builtin_cpu_supports("fma4") && __builtin_cpu_supports("fma") && __builtin_cpu_supports("f16c") && __builtin_cpu_supports("bmi");
#elif defined(LIBVC1_TEST_SIMD_AVX2_PARTIAL)
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") && __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") && __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("avx") && __builtin_cpu_supports("avx2");
#elif defined(LIBVC1_TEST_SIMD_V4)
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("avx") && __builtin_cpu_supports("avx2") &&
           __builtin_cpu_supports("bmi") && __builtin_cpu_supports("bmi2") &&
           __builtin_cpu_supports("f16c") && __builtin_cpu_supports("fma") &&
           __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("movbe") &&
           __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("avx512f") &&
           __builtin_cpu_supports("avx512bw") && __builtin_cpu_supports("avx512cd") &&
           __builtin_cpu_supports("avx512dq") && __builtin_cpu_supports("avx512vl");
#else
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("avx") && __builtin_cpu_supports("avx2") &&
           __builtin_cpu_supports("bmi") && __builtin_cpu_supports("bmi2") &&
           __builtin_cpu_supports("f16c") && __builtin_cpu_supports("fma") &&
           __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("movbe") &&
           __builtin_cpu_supports("popcnt");
#endif
#else
    return false;
#endif
}

bool close_coeff(const std::array<double,64>& a,const std::array<double,64>& b) {
    for (size_t i=0;i<a.size();++i) {
        const double tol=1e-7*std::max({1.0,std::abs(a[i]),std::abs(b[i])});
        if (std::abs(a[i]-b[i])>tol) {
            std::fprintf(stderr,"coefficient %zu differs: scalar=%.17g simd=%.17g\n",i,a[i],b[i]);
            return false;
        }
    }
    return true;
}

std::array<double,64> scalar8(const uint8_t* src,const uint8_t* pred,int stride,bool intra) {
    double r[8][8]{},tmp[8][8]{},xf[8][8]{};
    for(int y=0;y<8;++y) for(int x=0;x<8;++x)
        r[y][x]=intra?static_cast<int>(src[static_cast<ptrdiff_t>(y)*stride+x])-128:
                      static_cast<int>(src[static_cast<ptrdiff_t>(y)*stride+x])-static_cast<int>(pred[static_cast<ptrdiff_t>(y)*stride+x]);
    for(int a=0;a<8;++a) for(int x=0;x<8;++x) for(int y=0;y<8;++y)
        tmp[a][x]+=forward_transform::k8[a][y]*r[y][x];
    for(int a=0;a<8;++a) for(int b=0;b<8;++b) for(int x=0;x<8;++x)
        xf[a][b]+=tmp[a][x]*forward_transform::k8[b][x];
    std::array<double,64> out{};
    for(int row=0;row<8;++row) for(int col=0;col<8;++col)
        out[static_cast<size_t>(row)*8+col]=1024.0*xf[col][row];
    return out;
}

std::array<double,64> scalar_rect(const uint8_t* src,const uint8_t* pred,int stride,int w,int h) {
    double r[8][8]{},tmp[8][8]{},xf[8][8]{};
    for(int y=0;y<h;++y) for(int x=0;x<w;++x)
        r[y][x]=static_cast<int>(src[static_cast<ptrdiff_t>(y)*stride+x])-static_cast<int>(pred[static_cast<ptrdiff_t>(y)*stride+x]);
    for(int a=0;a<h;++a) for(int x=0;x<w;++x) for(int y=0;y<h;++y)
        tmp[a][x]+=(h==8?forward_transform::k8[a][y]:forward_transform::k4[a][y])*r[y][x];
    for(int a=0;a<h;++a) for(int b=0;b<w;++b) for(int x=0;x<w;++x)
        xf[a][b]+=tmp[a][x]*(w==8?forward_transform::k8[b][x]:forward_transform::k4[b][x]);
    std::array<double,64> out{};
    for(int row=0;row<h;++row) for(int col=0;col<w;++col)
        out[static_cast<size_t>(row)*8+col]=1024.0*xf[row][col];
    return out;
}

void simd_intra(const uint8_t* src,int stride,double* out,bool fma) {
#if defined(LIBVC1_TEST_SIMD_V1)
    libvc1::simd::forward_transform_intra8_x86_64_v1(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_PRESCOTT)
    libvc1::simd::forward_transform_intra8_prescott(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_K10)
    libvc1::simd::forward_transform_intra8_k10(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_CONROE)
    libvc1::simd::forward_transform_intra8_conroe(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_PENRYN)
    libvc1::simd::forward_transform_intra8_penryn(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_V2)
    libvc1::simd::forward_transform_intra8_x86_64_v2(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_SANDYBRIDGE)
    libvc1::simd::forward_transform_intra8_sandybridge(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_BULLDOZER)
    libvc1::simd::forward_transform_intra8_bulldozer(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_PILEDRIVER)
    libvc1::simd::forward_transform_intra8_piledriver(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_AVX2_PARTIAL)
    libvc1::simd::forward_transform_intra8_avx2_partial(src,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_V4)
    libvc1::simd::forward_transform_intra8_x86_64_v4(src,stride,out,fma);
#else
    libvc1::simd::forward_transform_intra8_x86_64_v3(src,stride,out,fma);
#endif
}
void simd_residual(const uint8_t* src,const uint8_t* pred,int stride,double* out,bool fma) {
#if defined(LIBVC1_TEST_SIMD_V1)
    libvc1::simd::forward_transform_residual8_x86_64_v1(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_PRESCOTT)
    libvc1::simd::forward_transform_residual8_prescott(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_K10)
    libvc1::simd::forward_transform_residual8_k10(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_CONROE)
    libvc1::simd::forward_transform_residual8_conroe(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_PENRYN)
    libvc1::simd::forward_transform_residual8_penryn(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_V2)
    libvc1::simd::forward_transform_residual8_x86_64_v2(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_SANDYBRIDGE)
    libvc1::simd::forward_transform_residual8_sandybridge(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_BULLDOZER)
    libvc1::simd::forward_transform_residual8_bulldozer(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_PILEDRIVER)
    libvc1::simd::forward_transform_residual8_piledriver(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_AVX2_PARTIAL)
    libvc1::simd::forward_transform_residual8_avx2_partial(src,pred,stride,out,fma);
#elif defined(LIBVC1_TEST_SIMD_V4)
    libvc1::simd::forward_transform_residual8_x86_64_v4(src,pred,stride,out,fma);
#else
    libvc1::simd::forward_transform_residual8_x86_64_v3(src,pred,stride,out,fma);
#endif
}
void simd_rect(const uint8_t* src,const uint8_t* pred,int stride,int w,int h,double* out,bool fma) {
#if defined(LIBVC1_TEST_SIMD_V1)
    libvc1::simd::forward_transform_residual_rect_x86_64_v1(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_PRESCOTT)
    libvc1::simd::forward_transform_residual_rect_prescott(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_K10)
    libvc1::simd::forward_transform_residual_rect_k10(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_CONROE)
    libvc1::simd::forward_transform_residual_rect_conroe(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_PENRYN)
    libvc1::simd::forward_transform_residual_rect_penryn(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_V2)
    libvc1::simd::forward_transform_residual_rect_x86_64_v2(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_SANDYBRIDGE)
    libvc1::simd::forward_transform_residual_rect_sandybridge(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_BULLDOZER)
    libvc1::simd::forward_transform_residual_rect_bulldozer(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_PILEDRIVER)
    libvc1::simd::forward_transform_residual_rect_piledriver(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_AVX2_PARTIAL)
    libvc1::simd::forward_transform_residual_rect_avx2_partial(src,pred,stride,w,h,out,fma);
#elif defined(LIBVC1_TEST_SIMD_V4)
    libvc1::simd::forward_transform_residual_rect_x86_64_v4(src,pred,stride,w,h,out,fma);
#else
    libvc1::simd::forward_transform_residual_rect_x86_64_v3(src,pred,stride,w,h,out,fma);
#endif
}

} // namespace

int main() {
    if (!cpu_supported()) return 77;
    constexpr int stride=19;
    std::array<uint8_t,stride*12> src{},pred{};
    uint32_t state=0x91f00d5u;
    for(int trial=0;trial<96;++trial) {
        for(size_t i=0;i<src.size();++i) {
            state=state*1664525u+1013904223u; src[i]=static_cast<uint8_t>(state>>24);
            state=state*1664525u+1013904223u; pred[i]=static_cast<uint8_t>(state>>24);
        }
        const uint8_t* s=src.data()+stride+3;
        const uint8_t* p=pred.data()+stride+3;
        for(bool fma : {false,true}) {
            std::array<double,64> got{};
            simd_intra(s,stride,got.data(),fma);
            if(!close_coeff(scalar8(s,nullptr,stride,true),got)) return 2;
            got.fill(0.0);
            simd_residual(s,p,stride,got.data(),fma);
            if(!close_coeff(scalar8(s,p,stride,false),got)) return 3;
            for(const auto shape : {std::array<int,2>{8,4},std::array<int,2>{4,8},std::array<int,2>{4,4}}) {
                got.fill(12345.0); // kernel must also zero inactive coefficient slots
                simd_rect(s,p,stride,shape[0],shape[1],got.data(),fma);
                if(!close_coeff(scalar_rect(s,p,stride,shape[0],shape[1]),got)) return 4;
            }
        }
    }
    return 0;
}
