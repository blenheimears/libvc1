#include "simd.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>

namespace {
struct GuardedTail {
    uint8_t* mapping=nullptr;
    uint8_t* data=nullptr;
    size_t page=0;
    size_t bytes=0;
    explicit GuardedTail(size_t n) : bytes(n) {
        const long ps=sysconf(_SC_PAGESIZE);
        if (ps<=0 || n>static_cast<size_t>(ps)) return;
        page=static_cast<size_t>(ps);
        void* p=mmap(nullptr,page*2,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);
        if (p==MAP_FAILED) return;
        mapping=static_cast<uint8_t*>(p);
        if (mprotect(mapping+page,page,PROT_NONE)!=0) {
            munmap(mapping,page*2); mapping=nullptr; return;
        }
        data=mapping+page-bytes;
    }
    ~GuardedTail() { if (mapping) munmap(mapping,page*2); }
    explicit operator bool() const { return data!=nullptr; }
};

static uint8_t chroma_scalar(uint8_t a,uint8_t b,uint8_t c,uint8_t d,int fx,int fy,bool rnd) {
    const int value=static_cast<int>(a)*(8-fx)*(8-fy)
                   +static_cast<int>(b)*fx*(8-fy)
                   +static_cast<int>(c)*(8-fx)*fy
                   +static_cast<int>(d)*fx*fy;
    return static_cast<uint8_t>((value+(rnd?28:32))>>6);
}
}

#if defined(LIBVC1_TEST_SIMD_V1)
# define CHROMA_MC libvc1::simd::chroma_mc_block_x86_64_v1
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_x86_64_v1
#elif defined(LIBVC1_TEST_SIMD_PRESCOTT)
# define CHROMA_MC libvc1::simd::chroma_mc_block_prescott
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_prescott
#elif defined(LIBVC1_TEST_SIMD_K10)
# define CHROMA_MC libvc1::simd::chroma_mc_block_k10
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_k10
#elif defined(LIBVC1_TEST_SIMD_CONROE)
# define CHROMA_MC libvc1::simd::chroma_mc_block_conroe
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_conroe
#elif defined(LIBVC1_TEST_SIMD_PENRYN)
# define CHROMA_MC libvc1::simd::chroma_mc_block_penryn
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_penryn
#elif defined(LIBVC1_TEST_SIMD_V2)
# define CHROMA_MC libvc1::simd::chroma_mc_block_x86_64_v2
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_x86_64_v2
#elif defined(LIBVC1_TEST_SIMD_SANDYBRIDGE)
# define CHROMA_MC libvc1::simd::chroma_mc_block_sandybridge
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_sandybridge
#elif defined(LIBVC1_TEST_SIMD_BULLDOZER)
# define CHROMA_MC libvc1::simd::chroma_mc_block_bulldozer
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_bulldozer
#elif defined(LIBVC1_TEST_SIMD_PILEDRIVER)
# define CHROMA_MC libvc1::simd::chroma_mc_block_piledriver
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_piledriver
#elif defined(LIBVC1_TEST_SIMD_AVX2_PARTIAL)
# define CHROMA_MC libvc1::simd::chroma_mc_block_avx2_partial
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_avx2_partial
#elif defined(LIBVC1_TEST_SIMD_V4)
# define CHROMA_MC libvc1::simd::chroma_mc_block_x86_64_v4
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_x86_64_v4
#else
# define CHROMA_MC libvc1::simd::chroma_mc_block_x86_64_v3
# define CHROMA_MC_AVG libvc1::simd::chroma_mc_block_avg_x86_64_v3
#endif

int main() {
#if defined(__x86_64__) || defined(__amd64__)
# if defined(__GNUC__) || defined(__clang__)
    __builtin_cpu_init();
# if defined(LIBVC1_TEST_SIMD_V1)
    /* SSE2 is mandatory in x86-64 mode. */
# elif defined(LIBVC1_TEST_SIMD_PRESCOTT)
    if (!__builtin_cpu_supports("sse3")) return 77;
# elif defined(LIBVC1_TEST_SIMD_K10)
    if (!(__builtin_cpu_supports("sse3") && __builtin_cpu_supports("sse4a") && __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("popcnt"))) return 77;
# elif defined(LIBVC1_TEST_SIMD_CONROE)
    if (!__builtin_cpu_supports("sse3") || !__builtin_cpu_supports("ssse3")) return 77;
# elif defined(LIBVC1_TEST_SIMD_PENRYN)
    if (!__builtin_cpu_supports("sse3") || !__builtin_cpu_supports("ssse3") || !__builtin_cpu_supports("sse4.1")) return 77;
# elif defined(LIBVC1_TEST_SIMD_V2)
    if (!__builtin_cpu_supports("sse3") || !__builtin_cpu_supports("ssse3") || !__builtin_cpu_supports("sse4.1") || !__builtin_cpu_supports("sse4.2") || !__builtin_cpu_supports("popcnt")) return 77;
# elif defined(LIBVC1_TEST_SIMD_SANDYBRIDGE)
    if (!__builtin_cpu_supports("sse3") || !__builtin_cpu_supports("ssse3") || !__builtin_cpu_supports("sse4.1") || !__builtin_cpu_supports("sse4.2") || !__builtin_cpu_supports("popcnt") || !__builtin_cpu_supports("avx")) return 77;
# elif defined(LIBVC1_TEST_SIMD_BULLDOZER)
    if (!(__builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") && __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") && __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("avx") && __builtin_cpu_supports("xop") && __builtin_cpu_supports("fma4"))) return 77;
# elif defined(LIBVC1_TEST_SIMD_PILEDRIVER)
    if (!(__builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") && __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") && __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("avx") && __builtin_cpu_supports("xop") && __builtin_cpu_supports("fma4") && __builtin_cpu_supports("fma") && __builtin_cpu_supports("f16c") && __builtin_cpu_supports("bmi"))) return 77;
# elif defined(LIBVC1_TEST_SIMD_AVX2_PARTIAL)
    if (!(__builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") && __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") && __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("avx") && __builtin_cpu_supports("avx2"))) return 77;
# elif defined(LIBVC1_TEST_SIMD_V4)
    if (!__builtin_cpu_supports("avx512f") || !__builtin_cpu_supports("avx512bw") || !__builtin_cpu_supports("avx512vl")) return 77;
# else
    if (!__builtin_cpu_supports("avx2")) return 77;
# endif
# endif
#endif
    // Horizontal-only interpolation at the final row. Exactly nine source
    // bytes are legal: a[0..7] and b[1..8]. Any load from center+stride must
    // hit the guard page.
    GuardedTail h(9);
    if (!h) return 2;
    for (int i=0;i<9;++i) h.data[i]=static_cast<uint8_t>(20+i*7);
    uint8_t out[8]{};
    CHROMA_MC(out,8,h.data,9,8,1,4,0,false);
    for (int x=0;x<8;++x) {
        const uint8_t want=chroma_scalar(h.data[x],h.data[x+1],0,0,4,0,false);
        if (out[x]!=want) { std::fprintf(stderr,"horizontal mismatch x=%d got=%u want=%u\n",x,out[x],want); return 3; }
    }
    std::memset(out,37,sizeof(out));
    CHROMA_MC_AVG(out,8,h.data,9,8,1,4,0,false);
    for (int x=0;x<8;++x) {
        const uint8_t pred=chroma_scalar(h.data[x],h.data[x+1],0,0,4,0,false);
        const uint8_t want=static_cast<uint8_t>((37+pred+1)>>1);
        if (out[x]!=want) { std::fprintf(stderr,"horizontal avg mismatch x=%d got=%u want=%u\n",x,out[x],want); return 4; }
    }

    // Vertical-only interpolation at the final column. Exactly two 8-byte
    // rows are legal. Any center+1 / center+stride+1 vector load reaches one
    // byte into the guard page even though those horizontal weights are zero.
    GuardedTail v(16);
    if (!v) return 5;
    for (int i=0;i<16;++i) v.data[i]=static_cast<uint8_t>(11+i*5);
    CHROMA_MC(out,8,v.data,8,8,1,0,6,true);
    for (int x=0;x<8;++x) {
        const uint8_t want=chroma_scalar(v.data[x],0,v.data[8+x],0,0,6,true);
        if (out[x]!=want) { std::fprintf(stderr,"vertical mismatch x=%d got=%u want=%u\n",x,out[x],want); return 6; }
    }
    std::memset(out,91,sizeof(out));
    CHROMA_MC_AVG(out,8,v.data,8,8,1,0,6,true);
    for (int x=0;x<8;++x) {
        const uint8_t pred=chroma_scalar(v.data[x],0,v.data[8+x],0,0,6,true);
        const uint8_t want=static_cast<uint8_t>((91+pred+1)>>1);
        if (out[x]!=want) { std::fprintf(stderr,"vertical avg mismatch x=%d got=%u want=%u\n",x,out[x],want); return 7; }
    }
    return 0;
}
