#include "simd.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

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

uint64_t scalar_frame_sad(const uint8_t* cur,int cur_stride,
                          const uint8_t* ref,int ref_stride,
                          int width,int height,int dx,int dy,int sample_stride) {
    uint64_t sad=0;
    for (int y=0;y<height;y+=sample_stride) {
        const int sy=std::clamp(y+dy,0,height-1);
        for (int x=0;x<width;x+=sample_stride) {
            const int sx=std::clamp(x+dx,0,width-1);
            sad+=static_cast<uint64_t>(std::abs(static_cast<int>(cur[static_cast<std::ptrdiff_t>(y)*cur_stride+x])-
                                                static_cast<int>(ref[static_cast<std::ptrdiff_t>(sy)*ref_stride+sx])));
        }
    }
    if (sample_stride>1) sad*=static_cast<uint64_t>(sample_stride*sample_stride);
    return sad;
}

uint64_t simd_frame_sad(const uint8_t* cur,int cur_stride,
                        const uint8_t* ref,int ref_stride,
                        int width,int height,int dx,int dy,int sample_stride) {
#if defined(LIBVC1_TEST_SIMD_V1)
    return libvc1::simd::frame_sad_x86_64_v1(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_PRESCOTT)
    return libvc1::simd::frame_sad_prescott(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_K10)
    return libvc1::simd::frame_sad_k10(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_CONROE)
    return libvc1::simd::frame_sad_conroe(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_PENRYN)
    return libvc1::simd::frame_sad_penryn(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_V2)
    return libvc1::simd::frame_sad_x86_64_v2(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_SANDYBRIDGE)
    return libvc1::simd::frame_sad_sandybridge(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_BULLDOZER)
    return libvc1::simd::frame_sad_bulldozer(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_PILEDRIVER)
    return libvc1::simd::frame_sad_piledriver(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_AVX2_PARTIAL)
    return libvc1::simd::frame_sad_avx2_partial(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#elif defined(LIBVC1_TEST_SIMD_V4)
    return libvc1::simd::frame_sad_x86_64_v4(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#else
    return libvc1::simd::frame_sad_x86_64_v3(cur,cur_stride,ref,ref_stride,width,height,dx,dy,sample_stride);
#endif
}


uint64_t scalar_satd4x4(const uint8_t* cur,int cs,const uint8_t* pred,int ps) {
    int d[4][4],t[4][4];
    for(int y=0;y<4;++y) for(int x=0;x<4;++x) d[y][x]=int(cur[y*cs+x])-int(pred[y*ps+x]);
    for(int y=0;y<4;++y){const int a0=d[y][0]+d[y][3],a1=d[y][1]+d[y][2],a2=d[y][1]-d[y][2],a3=d[y][0]-d[y][3];t[y][0]=a0+a1;t[y][1]=a3+a2;t[y][2]=a0-a1;t[y][3]=a3-a2;}
    uint64_t z=0;for(int x=0;x<4;++x){const int a0=t[0][x]+t[3][x],a1=t[1][x]+t[2][x],a2=t[1][x]-t[2][x],a3=t[0][x]-t[3][x];z+=std::abs(a0+a1)+std::abs(a3+a2)+std::abs(a0-a1)+std::abs(a3-a2);}return (z+1)>>1;
}
uint64_t simd_satd4x4(const uint8_t* cur,int cs,const uint8_t* pred,int ps) {
#if defined(LIBVC1_TEST_SIMD_V1)
    return libvc1::simd::satd4x4_x86_64_v1(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_PRESCOTT)
    return libvc1::simd::satd4x4_prescott(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_K10)
    return libvc1::simd::satd4x4_k10(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_CONROE)
    return libvc1::simd::satd4x4_conroe(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_PENRYN)
    return libvc1::simd::satd4x4_penryn(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_V2)
    return libvc1::simd::satd4x4_x86_64_v2(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_SANDYBRIDGE)
    return libvc1::simd::satd4x4_sandybridge(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_BULLDOZER)
    return libvc1::simd::satd4x4_bulldozer(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_PILEDRIVER)
    return libvc1::simd::satd4x4_piledriver(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_AVX2_PARTIAL)
    return libvc1::simd::satd4x4_avx2_partial(cur,cs,pred,ps);
#elif defined(LIBVC1_TEST_SIMD_V4)
    return libvc1::simd::satd4x4_x86_64_v4(cur,cs,pred,ps);
#else
    return libvc1::simd::satd4x4_x86_64_v3(cur,cs,pred,ps);
#endif
}

}

int main() {
    if (!cpu_supported()) return 77;
    constexpr std::array<int,5> sample_strides{{1,2,4,8,16}};
    constexpr std::array<int,13> offsets{{-32,-17,-8,-3,-1,0,1,3,8,17,31,32,37}};
    for (const int width : {159,160,319,320,321}) {
        for (const int height : {95,96,239,240,241}) {
            std::vector<uint8_t> cur(static_cast<size_t>(width)*height);
            std::vector<uint8_t> ref(static_cast<size_t>(width)*height);
            uint32_t state=0x13579bdu ^ static_cast<uint32_t>(width*65537+height);
            auto fill=[&](std::vector<uint8_t>& p) {
                for (auto& v:p) { state=state*1664525u+1013904223u; v=static_cast<uint8_t>(state>>24); }
            };
            fill(cur); fill(ref);
            for (const int ss:sample_strides) for (const int dy:offsets) for (const int dx:offsets) {
                const auto scalar=scalar_frame_sad(cur.data(),width,ref.data(),width,width,height,dx,dy,ss);
                const auto simd=simd_frame_sad(cur.data(),width,ref.data(),width,width,height,dx,dy,ss);
                if (scalar!=simd) {
                    std::fprintf(stderr,"frame SAD mismatch %dx%d stride=%d dx=%d dy=%d scalar=%llu simd=%llu\n",
                                 width,height,ss,dx,dy,
                                 static_cast<unsigned long long>(scalar),static_cast<unsigned long long>(simd));
                    return 1;
                }
            }
        }
    }
    // The same ISA-specific object also carries the staged-ME 4x4 SATD kernel.
    // Exercise varied strides and unaligned offsets so forced-QEMU ISA tests catch
    // both illegal-instruction leakage and arithmetic mismatches in this hot path.
    {
        std::array<uint8_t,512> a{},b{}; uint32_t st=0x2468aceu;
        for(auto& v:a){st=st*1664525u+1013904223u;v=static_cast<uint8_t>(st>>24);} for(auto& v:b){st=st*1664525u+1013904223u;v=static_cast<uint8_t>(st>>24);}
        for(int cs:{4,7,16,31}) for(int ps:{4,9,16,29}) for(int off=0;off<11;++off){
            const uint8_t* ap=a.data()+off; const uint8_t* bp=b.data()+13-off;
            const auto scalar=scalar_satd4x4(ap,cs,bp,ps); const auto simd=simd_satd4x4(ap,cs,bp,ps);
            if(scalar!=simd){std::fprintf(stderr,"SATD4x4 mismatch cs=%d ps=%d off=%d scalar=%llu simd=%llu\n",cs,ps,off,(unsigned long long)scalar,(unsigned long long)simd);return 2;}
        }
    }
    return 0;
}
