#include "encoder_internal.h"
#include "forward_transform_tables.h"

#include <cmath>
#include <cstring>

namespace libvc1 {

bool cpu_has_x86_64_v1() {
#if defined(LIBVC1_HAVE_X86_64_V1) && (defined(__x86_64__) || defined(__amd64__))
    return true;
#else
    return false;
#endif
}

bool cpu_has_x86_64_v2() {
#if defined(LIBVC1_HAVE_X86_64_V2) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("popcnt");
#else
    return false;
#endif
}

bool cpu_has_x86_64_v3() {
#if defined(LIBVC1_HAVE_X86_64_V3) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("avx") && __builtin_cpu_supports("avx2") &&
           __builtin_cpu_supports("bmi") && __builtin_cpu_supports("bmi2") &&
           __builtin_cpu_supports("f16c") && __builtin_cpu_supports("fma") &&
           __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("movbe") &&
           __builtin_cpu_supports("popcnt");
#else
    return false;
#endif
}

bool cpu_has_x86_64_v4() {
#if defined(LIBVC1_HAVE_X86_64_V4) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return cpu_has_x86_64_v3() && __builtin_cpu_supports("avx512f") &&
           __builtin_cpu_supports("avx512bw") && __builtin_cpu_supports("avx512cd") &&
           __builtin_cpu_supports("avx512dq") && __builtin_cpu_supports("avx512vl");
#else
    return false;
#endif
}

bool cpu_has_prescott() {
#if defined(LIBVC1_HAVE_PRESCOTT) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return __builtin_cpu_supports("sse3");
#else
    return false;
#endif
}

bool cpu_has_conroe() {
#if defined(LIBVC1_HAVE_CONROE) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3");
#else
    return false;
#endif
}

bool cpu_has_penryn() {
#if defined(LIBVC1_HAVE_PENRYN) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") && __builtin_cpu_supports("sse4.1");
#else
    return false;
#endif
}

bool cpu_has_sandybridge() {
#if defined(LIBVC1_HAVE_SANDYBRIDGE) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("avx");
#else
    return false;
#endif
}

bool cpu_has_k10() {
#if defined(LIBVC1_HAVE_K10) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("sse4a") &&
           __builtin_cpu_supports("lzcnt") && __builtin_cpu_supports("popcnt");
#else
    return false;
#endif
}

bool cpu_has_bulldozer() {
#if defined(LIBVC1_HAVE_BULLDOZER) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    /* FMA4 is deliberately a hard CPUID requirement.  Do not infer it from
       vendor/family/model: CPUs that hide or omit the FMA4 flag must never
       execute these kernels, including under --benchmark-all. */
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("lzcnt") &&
           __builtin_cpu_supports("avx") && __builtin_cpu_supports("xop") &&
           __builtin_cpu_supports("fma4");
#else
    return false;
#endif
}

bool cpu_has_piledriver() {
#if defined(LIBVC1_HAVE_PILEDRIVER) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    /* Piledriver is also FMA4-gated even though its fused transform candidate
       uses FMA3.  This prevents later AMD/Intel CPUs from matching the target. */
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("lzcnt") &&
           __builtin_cpu_supports("avx") && __builtin_cpu_supports("xop") &&
           __builtin_cpu_supports("fma4") && __builtin_cpu_supports("fma") &&
           __builtin_cpu_supports("f16c") && __builtin_cpu_supports("bmi");
#else
    return false;
#endif
}

bool cpu_has_avx2_partial() {
#if defined(LIBVC1_HAVE_AVX2_PARTIAL) && (defined(__x86_64__) || defined(__amd64__)) && (defined(__GNUC__) || defined(__clang__))
    __builtin_cpu_init();
    return __builtin_cpu_supports("sse3") && __builtin_cpu_supports("ssse3") &&
           __builtin_cpu_supports("sse4.1") && __builtin_cpu_supports("sse4.2") &&
           __builtin_cpu_supports("popcnt") && __builtin_cpu_supports("avx") &&
           __builtin_cpu_supports("avx2");
#else
    return false;
#endif
}

namespace {
enum class FmaFlavor : uint8_t { None=0, Fma3, Fma4 };
struct SimdTargetRecord {
    SimdTier tier;
    int benchmark_slot;
    const char* name;
    FmaFlavor fma_flavor;
    bool implemented;
    bool (*cpu_available)();
};

/* Registry order is the exact-throughput tie preference order. */
static const std::array<SimdTargetRecord,12>& simd_target_records() {
    static const std::array<SimdTargetRecord,12> records{{
        {SimdTier::X86V1,       VC1_SIMD_BENCH_X86_64_V1,   "x86-64-v1",   FmaFlavor::None, true, &cpu_has_x86_64_v1},
        {SimdTier::Prescott,    VC1_SIMD_BENCH_PRESCOTT,    "prescott",    FmaFlavor::None, true, &cpu_has_prescott},
        {SimdTier::K10,         VC1_SIMD_BENCH_K10,         "k10",         FmaFlavor::None, true, &cpu_has_k10},
        {SimdTier::Conroe,      VC1_SIMD_BENCH_CONROE,      "conroe",      FmaFlavor::None, true, &cpu_has_conroe},
        {SimdTier::Penryn,      VC1_SIMD_BENCH_PENRYN,      "penryn",      FmaFlavor::None, true, &cpu_has_penryn},
        {SimdTier::X86V2,       VC1_SIMD_BENCH_X86_64_V2,   "x86-64-v2",   FmaFlavor::None, true, &cpu_has_x86_64_v2},
        {SimdTier::SandyBridge, VC1_SIMD_BENCH_SANDYBRIDGE, "sandybridge", FmaFlavor::None, true, &cpu_has_sandybridge},
        {SimdTier::Bulldozer,   VC1_SIMD_BENCH_BULLDOZER,   "bulldozer",   FmaFlavor::Fma4, true, &cpu_has_bulldozer},
        {SimdTier::Piledriver,  VC1_SIMD_BENCH_PILEDRIVER,  "piledriver",  FmaFlavor::Fma3, true, &cpu_has_piledriver},
        {SimdTier::Avx2Partial, VC1_SIMD_BENCH_AVX2_PARTIAL,"avx2-partial",FmaFlavor::None, true, &cpu_has_avx2_partial},
        {SimdTier::X86V3,       VC1_SIMD_BENCH_X86_64_V3,   "x86-64-v3",   FmaFlavor::Fma3, true, &cpu_has_x86_64_v3},
        {SimdTier::X86V4,       VC1_SIMD_BENCH_X86_64_V4,   "x86-64-v4",   FmaFlavor::Fma3, true, &cpu_has_x86_64_v4},
    }};
    return records;
}

static const SimdTargetRecord* simd_target_record(SimdTier tier) {
    for (const auto& r : simd_target_records()) if (r.tier==tier) return &r;
    return nullptr;
}
} // namespace

const char* simd_target_name(SimdTier tier) {
    if (tier==SimdTier::None) return "none";
    if (tier==SimdTier::Mixed) return "mixed";
    const auto* r=simd_target_record(tier);
    return r?r->name:"unknown";
}

bool simd_target_implemented(SimdTier tier) {
    if (tier==SimdTier::None) return true;
    const auto* r=simd_target_record(tier);
    return r && r->implemented;
}

bool simd_target_available(SimdTier tier) {
    if (tier==SimdTier::None) return true;
    const auto* r=simd_target_record(tier);
    return r && r->implemented && r->cpu_available && r->cpu_available();
}

bool simd_target_auto_eligible(SimdTier tier,bool benchmark_all) {
    if (!simd_target_available(tier)) return false;
    if (benchmark_all) return true;
    /* Generic ISA levels keep their historical AUTO behavior.  Intermediate
       targets enter AUTO only when they are the highest useful feature band:
       lower intermediate implementations are intentionally skipped. */
    switch (tier) {
      case SimdTier::Prescott:
        return cpu_has_prescott() && !cpu_has_conroe();
      case SimdTier::Conroe:
        return cpu_has_conroe() && !cpu_has_penryn();
      case SimdTier::Penryn:
        return cpu_has_penryn() && !cpu_has_x86_64_v2();
      case SimdTier::SandyBridge:
        return cpu_has_sandybridge() && !cpu_has_avx2_partial() && !cpu_has_x86_64_v3();
      case SimdTier::K10:
        return cpu_has_k10() && !cpu_has_x86_64_v2() && !cpu_has_bulldozer();
      case SimdTier::Bulldozer:
        return cpu_has_bulldozer() && !cpu_has_piledriver();
      case SimdTier::Piledriver:
        return cpu_has_piledriver();
      case SimdTier::Avx2Partial:
        return cpu_has_avx2_partial() && !cpu_has_x86_64_v3();
      default:
        return true;
    }
}

bool simd_target_fma_capable(SimdTier tier) {
    const auto* r=simd_target_record(tier);
    return r && r->implemented && r->fma_flavor!=FmaFlavor::None;
}

int simd_target_benchmark_slot(SimdTier tier) {
    if (tier==SimdTier::None) return VC1_SIMD_BENCH_NONE;
    const auto* r=simd_target_record(tier);
    return r?r->benchmark_slot:-1;
}

const char* simd_primitive_name(SimdPrimitive primitive) {
    static constexpr const char* names[kSimdPrimitiveCount] = {
        "frame-sad","block-sad","forward-intra8","forward-residual8","inverse8x8",
        "put-block8","add-block-rect","luma-qpel-sad","luma-bilinear-sad","luma-mc",
        "luma-mc-avg","chroma-mc","chroma-mc-avg","intensity-luma","intensity-chroma",
        "forward-residual-rect","quantize","perceptual-stats","sum-u8","satd4x4"
    };
    const size_t i=static_cast<size_t>(primitive);
    return i<kSimdPrimitiveCount?names[i]:"unknown";
}

namespace {

static int simd_benchmark_total_ms() {
    constexpr int kDefaultMs=2400;
    const char* e=std::getenv("LIBVC1_SIMD_BENCHMARK_MS");
    if (!e || !*e) e=std::getenv("VC1BD_SIMD_BENCHMARK_MS");
    if (!e || !*e) return kDefaultMs;
    char* end=nullptr;
    const long v=std::strtol(e,&end,10);
    if (!end || *end!='\0' || v<100 || v>15000) return kDefaultMs;
    return static_cast<int>(v);
}

static inline int arshift_scalar(int v,int s) {
    if (v>=0) return v>>s;
    return -static_cast<int>((static_cast<unsigned long long>(-static_cast<long long>(v)) + ((1ull<<s)-1ull))>>s);
}
static inline int filter4_scalar(int a,int b,int c,int d,int mode) {
    if (mode==1) return -4*a+53*b+18*c-3*d;
    if (mode==2) return -a+9*b+9*c-d;
    if (mode==3) return -3*a+18*b+53*c-4*d;
    return b;
}
static inline int clamp_u8(int v) { return std::clamp(v,0,255); }

static uint64_t block_sad_none(const uint8_t* a,int as,const uint8_t* b,int bs,int w,int h,int ss) {
    uint64_t sad=0;
    for (int y=0;y<h;y+=ss) for (int x=0;x<w;x+=ss)
        sad+=static_cast<uint64_t>(std::abs(static_cast<int>(a[static_cast<ptrdiff_t>(y)*as+x])-static_cast<int>(b[static_cast<ptrdiff_t>(y)*bs+x])));
    if (ss>1) sad*=static_cast<uint64_t>(ss*ss);
    return sad;
}
static uint64_t frame_sad_none(const uint8_t* a,int as,const uint8_t* b,int bs,int w,int h,int dx,int dy,int ss) {
    uint64_t sad=0;
    for (int y=0;y<h;y+=ss) {
        const int sy=std::clamp(y+dy,0,h-1);
        for (int x=0;x<w;x+=ss) {
            const int sx=std::clamp(x+dx,0,w-1);
            sad+=static_cast<uint64_t>(std::abs(static_cast<int>(a[static_cast<ptrdiff_t>(y)*as+x])-static_cast<int>(b[static_cast<ptrdiff_t>(sy)*bs+sx])));
        }
    }
    if (ss>1) sad*=static_cast<uint64_t>(ss*ss);
    return sad;
}

// The compatibility benchmark intentionally uses the same canonical forward-transform
// tables as scalar encoder and SIMD kernels; numerical parity still comes from
// independent scalar versus vector evaluation order.


static void forward8_none(const uint8_t* src,const uint8_t* pred,int stride,double* out,bool intra) {
    double r[8][8]{},tmp[8][8]{},xf[8][8]{};
    for(int y=0;y<8;++y) for(int x=0;x<8;++x)
        r[y][x]=intra?static_cast<int>(src[static_cast<ptrdiff_t>(y)*stride+x])-128:
                      static_cast<int>(src[static_cast<ptrdiff_t>(y)*stride+x])-static_cast<int>(pred[static_cast<ptrdiff_t>(y)*stride+x]);
    for(int a=0;a<8;++a) for(int x=0;x<8;++x) for(int y=0;y<8;++y) tmp[a][x]+=forward_transform::k8[a][y]*r[y][x];
    for(int a=0;a<8;++a) for(int b=0;b<8;++b) for(int x=0;x<8;++x) xf[a][b]+=tmp[a][x]*forward_transform::k8[b][x];
    for(int row=0;row<8;++row) for(int col=0;col<8;++col) out[row*8+col]=1024.0*xf[col][row];
}
static void forward_rect_none(const uint8_t* src,const uint8_t* pred,int stride,int w,int h,double* out) {
    double r[8][8]{},tmp[8][8]{},xf[8][8]{};
    for(int y=0;y<h;++y) for(int x=0;x<w;++x) r[y][x]=static_cast<int>(src[static_cast<ptrdiff_t>(y)*stride+x])-static_cast<int>(pred[static_cast<ptrdiff_t>(y)*stride+x]);
    for(int a=0;a<h;++a) for(int x=0;x<w;++x) for(int y=0;y<h;++y) tmp[a][x]+=(h==8?forward_transform::k8[a][y]:forward_transform::k4[a][y])*r[y][x];
    for(int a=0;a<h;++a) for(int b=0;b<w;++b) for(int x=0;x<w;++x) xf[a][b]+=tmp[a][x]*(w==8?forward_transform::k8[b][x]:forward_transform::k4[b][x]);
    std::fill(out,out+64,0.0);
    for(int row=0;row<h;++row) for(int col=0;col<w;++col) out[row*8+col]=1024.0*xf[row][col];
}
static void inverse8_none(int* block) {
    int temp[64]{};
    for(int col=0;col<8;++col){const int* src=block+col;int* dst=temp+col*8;int t1=12*(src[0]+src[32])+4,t2=12*(src[0]-src[32])+4,t3=16*src[16]+6*src[48],t4=6*src[16]-16*src[48];int t5=t1+t3,t6=t2+t4,t7=t2-t4,t8=t1-t3;t1=16*src[8]+15*src[24]+9*src[40]+4*src[56];t2=15*src[8]-4*src[24]-16*src[40]-9*src[56];t3=9*src[8]-16*src[24]+4*src[40]+15*src[56];t4=4*src[8]-9*src[24]+15*src[40]-16*src[56];dst[0]=arshift_scalar(t5+t1,3);dst[1]=arshift_scalar(t6+t2,3);dst[2]=arshift_scalar(t7+t3,3);dst[3]=arshift_scalar(t8+t4,3);dst[4]=arshift_scalar(t8-t4,3);dst[5]=arshift_scalar(t7-t3,3);dst[6]=arshift_scalar(t6-t2,3);dst[7]=arshift_scalar(t5-t1,3);}
    for(int col=0;col<8;++col){const int* src=temp+col;int t1=12*(src[0]+src[32])+64,t2=12*(src[0]-src[32])+64,t3=16*src[16]+6*src[48],t4=6*src[16]-16*src[48];int t5=t1+t3,t6=t2+t4,t7=t2-t4,t8=t1-t3;t1=16*src[8]+15*src[24]+9*src[40]+4*src[56];t2=15*src[8]-4*src[24]-16*src[40]-9*src[56];t3=9*src[8]-16*src[24]+4*src[40]+15*src[56];t4=4*src[8]-9*src[24]+15*src[40]-16*src[56];block[0*8+col]=arshift_scalar(t5+t1,7);block[1*8+col]=arshift_scalar(t6+t2,7);block[2*8+col]=arshift_scalar(t7+t3,7);block[3*8+col]=arshift_scalar(t8+t4,7);block[4*8+col]=arshift_scalar(t8-t4+1,7);block[5*8+col]=arshift_scalar(t7-t3+1,7);block[6*8+col]=arshift_scalar(t6-t2+1,7);block[7*8+col]=arshift_scalar(t5-t1+1,7);}
}
static void put8_none(uint8_t* d,int ds,const int* b,int bias){for(int y=0;y<8;++y)for(int x=0;x<8;++x)d[static_cast<ptrdiff_t>(y)*ds+x]=static_cast<uint8_t>(clamp_u8(bias+b[y*8+x]));}
static void addrect_none(uint8_t* d,int ds,const int* b,int w,int h){for(int y=0;y<h;++y)for(int x=0;x<w;++x)d[static_cast<ptrdiff_t>(y)*ds+x]=static_cast<uint8_t>(clamp_u8(static_cast<int>(d[static_cast<ptrdiff_t>(y)*ds+x])+b[y*8+x]));}

static int luma_qpel_pixel(const uint8_t* c,int s,int hm,int vm,bool rnd) {
    if(!hm&&!vm) return c[0];
    if(hm&&vm){static constexpr int sv[4]={0,5,1,5};const int sh=(sv[hm]+sv[vm])>>1;const int r1=(1<<(sh-1))+(rnd?1:0)-1;int t[4];for(int k=-1;k<=2;++k)t[k+1]=arshift_scalar(filter4_scalar(c[k-s],c[k],c[k+s],c[k+2*s],vm)+r1,sh);return clamp_u8(arshift_scalar(filter4_scalar(t[0],t[1],t[2],t[3],hm)+64-(rnd?1:0),7));}
    if(vm){const int raw=filter4_scalar(c[-s],c[0],c[s],c[2*s],vm);return clamp_u8(arshift_scalar(raw+((vm==2)?(7+(rnd?1:0)):(31+(rnd?1:0))),vm==2?4:6));}
    const int raw=filter4_scalar(c[-1],c[0],c[1],c[2],hm);return clamp_u8(arshift_scalar(raw+((hm==2)?(8-(rnd?1:0)):(32-(rnd?1:0))),hm==2?4:6));
}
static int luma_bilinear_pixel(const uint8_t* c,int s,bool fx,bool fy,bool rnd){const int a=c[0];if(!fx&&!fy)return a;if(fx&&fy)return (a+c[1]+c[s]+c[s+1]+(rnd?1:2))>>2;return (a+c[fx?1:s]+(rnd?0:1))>>1;}
static int chroma_pixel(const uint8_t* c,int s,int fx,int fy,bool rnd){const int a=c[0];if(!fx&&!fy)return a;const int round=rnd?28:32;if(!fy)return ((8-fx)*8*a+fx*8*c[1]+round)>>6;if(!fx)return ((8-fy)*8*a+fy*8*c[s]+round)>>6;return ((8-fx)*(8-fy)*a+fx*(8-fy)*c[1]+(8-fx)*fy*c[s]+fx*fy*c[s+1]+round)>>6;}
static uint64_t luma_qpel_sad_none(const uint8_t* cur,int cs,const uint8_t* ref,int rs,int w,int h,int hm,int vm,bool rnd){uint64_t z=0;for(int y=0;y<h;++y)for(int x=0;x<w;++x)z+=std::abs(static_cast<int>(cur[static_cast<ptrdiff_t>(y)*cs+x])-luma_qpel_pixel(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,hm,vm,rnd));return z;}
static uint64_t luma_bilinear_sad_none(const uint8_t* cur,int cs,const uint8_t* ref,int rs,int w,int h,bool fx,bool fy,bool rnd){uint64_t z=0;for(int y=0;y<h;++y)for(int x=0;x<w;++x)z+=std::abs(static_cast<int>(cur[static_cast<ptrdiff_t>(y)*cs+x])-luma_bilinear_pixel(ref+static_cast<ptrdiff_t>(y)*rs+x,rs,fx,fy,rnd));return z;}
static void luma_mc_none(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int hm,int vm,bool rnd,bool bilinear,bool avg){for(int y=0;y<h;++y)for(int x=0;x<w;++x){const int p=bilinear?luma_bilinear_pixel(r+static_cast<ptrdiff_t>(y)*rs+x,rs,hm!=0,vm!=0,rnd):luma_qpel_pixel(r+static_cast<ptrdiff_t>(y)*rs+x,rs,hm,vm,rnd);uint8_t& q=d[static_cast<ptrdiff_t>(y)*ds+x];q=static_cast<uint8_t>(avg?((static_cast<int>(q)+p+1)>>1):p);}}
static void chroma_mc_none(uint8_t* d,int ds,const uint8_t* r,int rs,int w,int h,int fx,int fy,bool rnd,bool avg){for(int y=0;y<h;++y)for(int x=0;x<w;++x){const int p=chroma_pixel(r+static_cast<ptrdiff_t>(y)*rs+x,rs,fx,fy,rnd);uint8_t& q=d[static_cast<ptrdiff_t>(y)*ds+x];q=static_cast<uint8_t>(avg?((static_cast<int>(q)+p+1)>>1):p);}}
static void intensity_none(uint8_t* d,const uint8_t* s,size_t n,int scale,int shift){for(size_t i=0;i<n;++i)d[i]=static_cast<uint8_t>(clamp_u8((scale*static_cast<int>(s[i])+shift+32)>>6));}
static void intensity_chroma_none(uint8_t* d,const uint8_t* s,size_t n,int scale){for(size_t i=0;i<n;++i)d[i]=static_cast<uint8_t>(clamp_u8((scale*(static_cast<int>(s[i])-128)+128*64+32)>>6));}
static void quant_none(const double* c,int* out,int n,double qs,double off,int mx){for(int i=0;i<n;++i){const double a=std::abs(c[i]);if(a==0.0){out[i]=0;continue;}int q=static_cast<int>(std::ceil((a-off)/qs-0.5));q=std::clamp(q,1,mx);const double e=a-(q*qs+off);out[i]=(e*e<a*a)?(c[i]<0?-q:q):0;}}
static uint64_t sum_none(const uint8_t* p,size_t n){uint64_t z=0;for(size_t i=0;i<n;++i)z+=p[i];return z;}
static void perceptual_none(const uint8_t* src,const uint8_t* pred,int s,int w,int h,uint64_t* sum,uint64_t* grad,uint64_t* residual,int* mn,int* mx,int* edges,int* gn){uint64_t su=0,g=0,r=0;int lo=255,hi=0,e=0,n=0;for(int y=0;y<h;++y)for(int x=0;x<w;++x){const int v=src[static_cast<ptrdiff_t>(y)*s+x];su+=v;lo=std::min(lo,v);hi=std::max(hi,v);if(pred)r+=std::abs(v-static_cast<int>(pred[static_cast<ptrdiff_t>(y)*s+x]));if(x){const int d=std::abs(v-static_cast<int>(src[static_cast<ptrdiff_t>(y)*s+x-1]));g+=d;++n;if(d>=96)++e;}if(y){const int d=std::abs(v-static_cast<int>(src[static_cast<ptrdiff_t>(y-1)*s+x]));g+=d;++n;if(d>=96)++e;}}*sum=su;*grad=g;*residual=r;*mn=lo;*mx=hi;*edges=e;*gn=n;}

static uint64_t satd4x4_none(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride) {
    int d[4][4],t[4][4];
    for(int y=0;y<4;++y) for(int x=0;x<4;++x)
        d[y][x]=static_cast<int>(cur[static_cast<ptrdiff_t>(y)*cur_stride+x])-static_cast<int>(pred[static_cast<ptrdiff_t>(y)*pred_stride+x]);
    for(int y=0;y<4;++y){
        const int a0=d[y][0]+d[y][3],a1=d[y][1]+d[y][2];
        const int a2=d[y][1]-d[y][2],a3=d[y][0]-d[y][3];
        t[y][0]=a0+a1;t[y][1]=a3+a2;t[y][2]=a0-a1;t[y][3]=a3-a2;
    }
    uint64_t total=0;
    for(int x=0;x<4;++x){
        const int a0=t[0][x]+t[3][x],a1=t[1][x]+t[2][x];
        const int a2=t[1][x]-t[2][x],a3=t[0][x]-t[3][x];
        total+=static_cast<uint64_t>(std::abs(a0+a1)); total+=static_cast<uint64_t>(std::abs(a3+a2));
        total+=static_cast<uint64_t>(std::abs(a0-a1)); total+=static_cast<uint64_t>(std::abs(a3-a2));
    }
    return (total+1)>>1;
}

struct BenchData {
    static constexpr int S=64;
    alignas(64) std::array<uint8_t,S*S> a{},b{},d{};
    alignas(64) std::array<int,64> ib{},io{};
    alignas(64) std::array<double,64> c{},co{};
    uint64_t stat_sum=0,stat_grad=0,stat_residual=0;
    int stat_min=0,stat_max=0,stat_edges=0,stat_grad_n=0;
    BenchData(){uint32_t x=0x1234567u;for(size_t i=0;i<a.size();++i){x=x*1664525u+1013904223u;a[i]=static_cast<uint8_t>(x>>24);x=x*1664525u+1013904223u;b[i]=static_cast<uint8_t>(x>>24);d[i]=static_cast<uint8_t>((i*13+71)&255);}for(int i=0;i<64;++i){ib[i]=(i*37%511)-255;c[i]=std::sin(i*0.41)*700.0;}}
};

static bool close_coeff(const double* a,const double* b,int n){for(int i=0;i<n;++i){const double tol=1e-7*std::max({1.0,std::abs(a[i]),std::abs(b[i])});if(std::abs(a[i]-b[i])>tol)return false;}return true;}

static void run_primitive(SimdPrimitive p,SimdTier t,bool fma,BenchData& x,uint64_t& sink) {
    constexpr int S=BenchData::S; const uint8_t* rp=x.b.data()+2*S+2; const uint8_t* cp=x.a.data()+2*S+2; uint8_t* dp=x.d.data()+2*S+2;
    switch(p){
      case SimdPrimitive::FrameSad:
        if(t==SimdTier::None)sink+=frame_sad_none(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)sink+=simd::frame_sad_x86_64_v1(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)sink+=simd::frame_sad_prescott(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)sink+=simd::frame_sad_k10(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)sink+=simd::frame_sad_conroe(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)sink+=simd::frame_sad_x86_64_v2(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)sink+=simd::frame_sad_penryn(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)sink+=simd::frame_sad_sandybridge(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)sink+=simd::frame_sad_bulldozer(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)sink+=simd::frame_sad_piledriver(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)sink+=simd::frame_sad_avx2_partial(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)sink+=simd::frame_sad_x86_64_v3(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else sink+=simd::frame_sad_x86_64_v4(x.a.data(),S,x.b.data(),S,S,S,3,-2,2);
#endif
        break;
      case SimdPrimitive::BlockSad:
        if(t==SimdTier::None)sink+=block_sad_none(cp,S,rp,S,32,32,1);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)sink+=simd::block_sad_x86_64_v1(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)sink+=simd::block_sad_prescott(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)sink+=simd::block_sad_k10(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)sink+=simd::block_sad_conroe(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)sink+=simd::block_sad_x86_64_v2(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)sink+=simd::block_sad_penryn(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)sink+=simd::block_sad_sandybridge(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)sink+=simd::block_sad_bulldozer(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)sink+=simd::block_sad_piledriver(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)sink+=simd::block_sad_avx2_partial(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)sink+=simd::block_sad_x86_64_v3(cp,S,rp,S,32,32,1);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else sink+=simd::block_sad_x86_64_v4(cp,S,rp,S,32,32,1);
#endif
        break;
      case SimdPrimitive::ForwardIntra8:
        if(t==SimdTier::None)forward8_none(cp,nullptr,S,x.co.data(),true);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::forward_transform_intra8_x86_64_v1(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::forward_transform_intra8_prescott(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::forward_transform_intra8_k10(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::forward_transform_intra8_conroe(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::forward_transform_intra8_x86_64_v2(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::forward_transform_intra8_penryn(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::forward_transform_intra8_sandybridge(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::forward_transform_intra8_bulldozer(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::forward_transform_intra8_piledriver(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::forward_transform_intra8_avx2_partial(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::forward_transform_intra8_x86_64_v3(cp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::forward_transform_intra8_x86_64_v4(cp,S,x.co.data(),fma);
#endif
        sink+=static_cast<uint64_t>(std::llabs(static_cast<long long>(x.co[7]))); break;
      case SimdPrimitive::ForwardResidual8:
        if(t==SimdTier::None)forward8_none(cp,rp,S,x.co.data(),false);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::forward_transform_residual8_x86_64_v1(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::forward_transform_residual8_prescott(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::forward_transform_residual8_k10(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::forward_transform_residual8_conroe(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::forward_transform_residual8_x86_64_v2(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::forward_transform_residual8_penryn(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::forward_transform_residual8_sandybridge(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::forward_transform_residual8_bulldozer(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::forward_transform_residual8_piledriver(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::forward_transform_residual8_avx2_partial(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::forward_transform_residual8_x86_64_v3(cp,rp,S,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::forward_transform_residual8_x86_64_v4(cp,rp,S,x.co.data(),fma);
#endif
        sink+=static_cast<uint64_t>(std::llabs(static_cast<long long>(x.co[11]))); break;
      case SimdPrimitive::Inverse8x8:
        std::copy(x.ib.begin(),x.ib.end(),x.io.begin()); if(t==SimdTier::None)inverse8_none(x.io.data());
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::inverse_transform_8x8_x86_64_v1(x.io.data());
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::inverse_transform_8x8_prescott(x.io.data());
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::inverse_transform_8x8_k10(x.io.data());
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::inverse_transform_8x8_conroe(x.io.data());
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::inverse_transform_8x8_x86_64_v2(x.io.data());
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::inverse_transform_8x8_penryn(x.io.data());
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::inverse_transform_8x8_sandybridge(x.io.data());
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::inverse_transform_8x8_bulldozer(x.io.data());
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::inverse_transform_8x8_piledriver(x.io.data());
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::inverse_transform_8x8_avx2_partial(x.io.data());
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::inverse_transform_8x8_x86_64_v3(x.io.data());
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::inverse_transform_8x8_x86_64_v4(x.io.data());
#endif
        sink+=static_cast<uint32_t>(x.io[17]); break;
      case SimdPrimitive::PutBlock8:
        if(t==SimdTier::None)put8_none(dp,S,x.ib.data(),128);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::put_block8_x86_64_v1(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::put_block8_prescott(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::put_block8_k10(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::put_block8_conroe(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::put_block8_x86_64_v2(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::put_block8_penryn(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::put_block8_sandybridge(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::put_block8_bulldozer(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::put_block8_piledriver(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::put_block8_avx2_partial(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::put_block8_x86_64_v3(dp,S,x.ib.data(),128);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::put_block8_x86_64_v4(dp,S,x.ib.data(),128);
#endif
        sink+=dp[7]; break;
      case SimdPrimitive::AddBlockRect:
        std::memcpy(dp,cp,8); if(t==SimdTier::None)addrect_none(dp,S,x.ib.data(),8,8);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::add_block_rect_x86_64_v1(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::add_block_rect_prescott(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::add_block_rect_k10(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::add_block_rect_conroe(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::add_block_rect_x86_64_v2(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::add_block_rect_penryn(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::add_block_rect_sandybridge(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::add_block_rect_bulldozer(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::add_block_rect_piledriver(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::add_block_rect_avx2_partial(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::add_block_rect_x86_64_v3(dp,S,x.ib.data(),8,8);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::add_block_rect_x86_64_v4(dp,S,x.ib.data(),8,8);
#endif
        sink+=dp[3]; break;
      case SimdPrimitive::LumaQpelSad:
        if(t==SimdTier::None)sink+=luma_qpel_sad_none(cp,S,rp,S,16,16,1,3,false);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)sink+=simd::block_sad_luma_qpel_x86_64_v1(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)sink+=simd::block_sad_luma_qpel_prescott(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)sink+=simd::block_sad_luma_qpel_k10(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)sink+=simd::block_sad_luma_qpel_conroe(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)sink+=simd::block_sad_luma_qpel_x86_64_v2(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)sink+=simd::block_sad_luma_qpel_penryn(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)sink+=simd::block_sad_luma_qpel_sandybridge(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)sink+=simd::block_sad_luma_qpel_bulldozer(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)sink+=simd::block_sad_luma_qpel_piledriver(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)sink+=simd::block_sad_luma_qpel_avx2_partial(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)sink+=simd::block_sad_luma_qpel_x86_64_v3(cp,S,rp,S,16,16,1,3,false);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else sink+=simd::block_sad_luma_qpel_x86_64_v4(cp,S,rp,S,16,16,1,3,false);
#endif
        break;
      case SimdPrimitive::LumaBilinearSad:
        if(t==SimdTier::None)sink+=luma_bilinear_sad_none(cp,S,rp,S,16,16,true,true,false);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)sink+=simd::block_sad_luma_bilinear_x86_64_v1(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)sink+=simd::block_sad_luma_bilinear_prescott(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)sink+=simd::block_sad_luma_bilinear_k10(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)sink+=simd::block_sad_luma_bilinear_conroe(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)sink+=simd::block_sad_luma_bilinear_x86_64_v2(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)sink+=simd::block_sad_luma_bilinear_penryn(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)sink+=simd::block_sad_luma_bilinear_sandybridge(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)sink+=simd::block_sad_luma_bilinear_bulldozer(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)sink+=simd::block_sad_luma_bilinear_piledriver(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)sink+=simd::block_sad_luma_bilinear_avx2_partial(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)sink+=simd::block_sad_luma_bilinear_x86_64_v3(cp,S,rp,S,16,16,true,true,false);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else sink+=simd::block_sad_luma_bilinear_x86_64_v4(cp,S,rp,S,16,16,true,true,false);
#endif
        break;
      case SimdPrimitive::LumaMc: case SimdPrimitive::LumaMcAvg: {
        const bool avg=p==SimdPrimitive::LumaMcAvg; if(avg)std::memcpy(dp,cp,16);
        if(t==SimdTier::None)luma_mc_none(dp,S,rp,S,16,16,1,3,false,false,avg);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1){if(avg)simd::luma_mc_block_avg_x86_64_v1(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_x86_64_v1(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott){if(avg)simd::luma_mc_block_avg_prescott(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_prescott(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10){if(avg)simd::luma_mc_block_avg_k10(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_k10(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe){if(avg)simd::luma_mc_block_avg_conroe(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_conroe(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2){if(avg)simd::luma_mc_block_avg_x86_64_v2(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_x86_64_v2(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn){if(avg)simd::luma_mc_block_avg_penryn(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_penryn(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge){if(avg)simd::luma_mc_block_avg_sandybridge(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_sandybridge(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer){if(avg)simd::luma_mc_block_avg_bulldozer(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_bulldozer(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver){if(avg)simd::luma_mc_block_avg_piledriver(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_piledriver(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial){if(avg)simd::luma_mc_block_avg_avx2_partial(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_avx2_partial(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3){if(avg)simd::luma_mc_block_avg_x86_64_v3(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_x86_64_v3(dp,S,rp,S,16,16,1,3,false,false);}
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else {if(avg)simd::luma_mc_block_avg_x86_64_v4(dp,S,rp,S,16,16,1,3,false,false);else simd::luma_mc_block_x86_64_v4(dp,S,rp,S,16,16,1,3,false,false);}
#endif
        sink+=dp[12]; break; }
      case SimdPrimitive::ChromaMc: case SimdPrimitive::ChromaMcAvg: {
        const bool avg=p==SimdPrimitive::ChromaMcAvg; if(avg)std::memcpy(dp,cp,8);
        if(t==SimdTier::None)chroma_mc_none(dp,S,rp,S,8,8,3,5,false,avg);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1){if(avg)simd::chroma_mc_block_avg_x86_64_v1(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_x86_64_v1(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott){if(avg)simd::chroma_mc_block_avg_prescott(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_prescott(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10){if(avg)simd::chroma_mc_block_avg_k10(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_k10(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe){if(avg)simd::chroma_mc_block_avg_conroe(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_conroe(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2){if(avg)simd::chroma_mc_block_avg_x86_64_v2(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_x86_64_v2(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn){if(avg)simd::chroma_mc_block_avg_penryn(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_penryn(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge){if(avg)simd::chroma_mc_block_avg_sandybridge(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_sandybridge(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer){if(avg)simd::chroma_mc_block_avg_bulldozer(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_bulldozer(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver){if(avg)simd::chroma_mc_block_avg_piledriver(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_piledriver(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial){if(avg)simd::chroma_mc_block_avg_avx2_partial(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_avx2_partial(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3){if(avg)simd::chroma_mc_block_avg_x86_64_v3(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_x86_64_v3(dp,S,rp,S,8,8,3,5,false);}
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else {if(avg)simd::chroma_mc_block_avg_x86_64_v4(dp,S,rp,S,8,8,3,5,false);else simd::chroma_mc_block_x86_64_v4(dp,S,rp,S,8,8,3,5,false);}
#endif
        sink+=dp[5]; break; }
      case SimdPrimitive::IntensityLuma:
        if(t==SimdTier::None)intensity_none(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::intensity_map_plane_x86_64_v1(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::intensity_map_plane_prescott(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::intensity_map_plane_k10(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::intensity_map_plane_conroe(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::intensity_map_plane_x86_64_v2(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::intensity_map_plane_penryn(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::intensity_map_plane_sandybridge(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::intensity_map_plane_bulldozer(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::intensity_map_plane_piledriver(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::intensity_map_plane_avx2_partial(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::intensity_map_plane_x86_64_v3(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::intensity_map_plane_x86_64_v4(x.d.data(),x.a.data(),x.a.size(),71,-3*64);
#endif
        sink+=x.d[123]; break;
      case SimdPrimitive::IntensityChroma:
        if(t==SimdTier::None)intensity_chroma_none(x.d.data(),x.a.data(),x.a.size(),71);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::intensity_map_chroma_plane_x86_64_v1(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::intensity_map_chroma_plane_prescott(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::intensity_map_chroma_plane_k10(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::intensity_map_chroma_plane_conroe(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::intensity_map_chroma_plane_x86_64_v2(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::intensity_map_chroma_plane_penryn(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::intensity_map_chroma_plane_sandybridge(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::intensity_map_chroma_plane_bulldozer(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::intensity_map_chroma_plane_piledriver(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::intensity_map_chroma_plane_avx2_partial(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::intensity_map_chroma_plane_x86_64_v3(x.d.data(),x.a.data(),x.a.size(),71);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::intensity_map_chroma_plane_x86_64_v4(x.d.data(),x.a.data(),x.a.size(),71);
#endif
        sink+=x.d[321]; break;
      case SimdPrimitive::ForwardResidualRect:
        if(t==SimdTier::None)forward_rect_none(cp,rp,S,4,8,x.co.data());
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::forward_transform_residual_rect_x86_64_v1(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::forward_transform_residual_rect_prescott(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::forward_transform_residual_rect_k10(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::forward_transform_residual_rect_conroe(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::forward_transform_residual_rect_x86_64_v2(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::forward_transform_residual_rect_penryn(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::forward_transform_residual_rect_sandybridge(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::forward_transform_residual_rect_bulldozer(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::forward_transform_residual_rect_piledriver(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::forward_transform_residual_rect_avx2_partial(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::forward_transform_residual_rect_x86_64_v3(cp,rp,S,4,8,x.co.data(),fma);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::forward_transform_residual_rect_x86_64_v4(cp,rp,S,4,8,x.co.data(),fma);
#endif
        sink+=static_cast<uint64_t>(std::llabs(static_cast<long long>(x.co[9]))); break;
      case SimdPrimitive::Quantize:
        if(t==SimdTier::None)quant_none(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::quantize_coefficients_x86_64_v1(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::quantize_coefficients_prescott(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::quantize_coefficients_k10(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::quantize_coefficients_conroe(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::quantize_coefficients_x86_64_v2(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::quantize_coefficients_penryn(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::quantize_coefficients_sandybridge(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::quantize_coefficients_bulldozer(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::quantize_coefficients_piledriver(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::quantize_coefficients_avx2_partial(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::quantize_coefficients_x86_64_v3(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::quantize_coefficients_x86_64_v4(x.c.data(),x.io.data(),64,18.0,9.0,2047);
#endif
        sink+=static_cast<uint32_t>(x.io[23]); break;
      case SimdPrimitive::PerceptualStats: {
        if(t==SimdTier::None)perceptual_none(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)simd::perceptual_stats_x86_64_v1(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)simd::perceptual_stats_prescott(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)simd::perceptual_stats_k10(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)simd::perceptual_stats_conroe(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)simd::perceptual_stats_x86_64_v2(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)simd::perceptual_stats_penryn(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)simd::perceptual_stats_sandybridge(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)simd::perceptual_stats_bulldozer(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)simd::perceptual_stats_piledriver(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)simd::perceptual_stats_avx2_partial(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)simd::perceptual_stats_x86_64_v3(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else simd::perceptual_stats_x86_64_v4(cp,rp,S,16,16,&x.stat_sum,&x.stat_grad,&x.stat_residual,&x.stat_min,&x.stat_max,&x.stat_edges,&x.stat_grad_n);
#endif
        sink+=x.stat_sum+x.stat_grad+x.stat_residual+static_cast<unsigned>(x.stat_min+x.stat_max+x.stat_edges+x.stat_grad_n); break;}
      case SimdPrimitive::Satd4x4:
        if(t==SimdTier::None)sink+=satd4x4_none(cp,S,rp,S);
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)sink+=simd::satd4x4_x86_64_v1(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)sink+=simd::satd4x4_prescott(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)sink+=simd::satd4x4_k10(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)sink+=simd::satd4x4_conroe(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)sink+=simd::satd4x4_penryn(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)sink+=simd::satd4x4_x86_64_v2(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)sink+=simd::satd4x4_sandybridge(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)sink+=simd::satd4x4_bulldozer(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)sink+=simd::satd4x4_piledriver(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)sink+=simd::satd4x4_avx2_partial(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)sink+=simd::satd4x4_x86_64_v3(cp,S,rp,S);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else sink+=simd::satd4x4_x86_64_v4(cp,S,rp,S);
#endif
        break;
      case SimdPrimitive::SumU8:
        if(t==SimdTier::None)sink+=sum_none(x.a.data(),x.a.size());
#if defined(LIBVC1_HAVE_X86_64_V1)
        else if(t==SimdTier::X86V1)sink+=simd::sum_u8_x86_64_v1(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        else if(t==SimdTier::Prescott)sink+=simd::sum_u8_prescott(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_K10)
        else if(t==SimdTier::K10)sink+=simd::sum_u8_k10(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_CONROE)
        else if(t==SimdTier::Conroe)sink+=simd::sum_u8_conroe(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        else if(t==SimdTier::X86V2)sink+=simd::sum_u8_x86_64_v2(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        else if(t==SimdTier::Penryn)sink+=simd::sum_u8_penryn(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        else if(t==SimdTier::SandyBridge)sink+=simd::sum_u8_sandybridge(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        else if(t==SimdTier::Bulldozer)sink+=simd::sum_u8_bulldozer(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        else if(t==SimdTier::Piledriver)sink+=simd::sum_u8_piledriver(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        else if(t==SimdTier::Avx2Partial)sink+=simd::sum_u8_avx2_partial(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        else if(t==SimdTier::X86V3)sink+=simd::sum_u8_x86_64_v3(x.a.data(),x.a.size());
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
        else sink+=simd::sum_u8_x86_64_v4(x.a.data(),x.a.size());
#endif
        break;
      default: break;
    }
}

static bool compatible(SimdPrimitive p,SimdTier t,bool fma) {
    if(t==SimdTier::None) return true;
    BenchData a,b; uint64_t sa=0,sb=0;
    run_primitive(p,SimdTier::None,false,a,sa);
    run_primitive(p,t,fma,b,sb);
    switch(p){
      case SimdPrimitive::ForwardIntra8: case SimdPrimitive::ForwardResidual8: case SimdPrimitive::ForwardResidualRect:
        return close_coeff(a.co.data(),b.co.data(),64);
      case SimdPrimitive::Inverse8x8: case SimdPrimitive::Quantize:
        return a.io==b.io;
      case SimdPrimitive::PutBlock8: case SimdPrimitive::AddBlockRect: case SimdPrimitive::LumaMc: case SimdPrimitive::LumaMcAvg:
      case SimdPrimitive::ChromaMc: case SimdPrimitive::ChromaMcAvg: case SimdPrimitive::IntensityLuma: case SimdPrimitive::IntensityChroma:
        return a.d==b.d;
      case SimdPrimitive::PerceptualStats:
        return a.stat_sum==b.stat_sum && a.stat_grad==b.stat_grad && a.stat_residual==b.stat_residual &&
               a.stat_min==b.stat_min && a.stat_max==b.stat_max && a.stat_edges==b.stat_edges && a.stat_grad_n==b.stat_grad_n;
      default: return sa==sb;
    }
}

static double benchmark_one(SimdPrimitive p,SimdTier t,bool fma,int ms) {
    BenchData d; uint64_t sink=0,iterations=0;
    for(int i=0;i<8;++i) run_primitive(p,t,fma,d,sink);
    const auto start=std::chrono::steady_clock::now();
    const auto deadline=start+std::chrono::milliseconds(ms);
    do { for(int k=0;k<8;++k)run_primitive(p,t,fma,d,sink); iterations+=8; } while(std::chrono::steady_clock::now()<deadline);
    static volatile uint64_t keep=0; keep^=sink;
    const double sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    return sec>0.0?static_cast<double>(iterations)/sec:0.0;
}

static SimdTier summarize(const SimdDispatch& d) {
    const SimdTier first=d.tier[0]; for(SimdTier t:d.tier) if(t!=first)return SimdTier::Mixed; return first;
}

} // namespace

SimdTier choose_fastest_simd_target(const std::array<double,VC1_SIMD_BENCH_TARGET_COUNT>& rates) {
    SimdTier best=SimdTier::None;
    double best_rate=rates[VC1_SIMD_BENCH_NONE];
    for (const auto& target : simd_target_records()) {
        const double r=rates[static_cast<size_t>(target.benchmark_slot)];
        if (r>best_rate) { best=target.tier; best_rate=r; }
    }
    return best;
}

SimdBenchmarkResult auto_select_simd(bool allow_fma,bool benchmark_all) {
    SimdBenchmarkResult result;
    const int total=simd_benchmark_total_ms();
    int candidates=1;
    for (const auto& target : simd_target_records()) if (simd_target_auto_eligible(target.tier,benchmark_all)) ++candidates;
    const int slice=std::max(4,total/std::max(1,static_cast<int>(kSimdPrimitiveCount)*candidates));
    std::array<long double,VC1_SIMD_BENCH_TARGET_COUNT> logsum{};
    std::array<int,VC1_SIMD_BENCH_TARGET_COUNT> nrate{};
    for(size_t pi=0;pi<kSimdPrimitiveCount;++pi){
        const auto p=static_cast<SimdPrimitive>(pi);
        std::array<bool,VC1_SIMD_BENCH_TARGET_COUNT> target_fma{};
        const double none_rate=benchmark_one(p,SimdTier::None,false,slice);
        result.primitive_units_per_second[pi][VC1_SIMD_BENCH_NONE]=none_rate;
        if(none_rate>0){logsum[VC1_SIMD_BENCH_NONE]+=std::log(none_rate);++nrate[VC1_SIMD_BENCH_NONE];}
        for (const auto& target : simd_target_records()) {
            if (!simd_target_auto_eligible(target.tier,benchmark_all)) continue;
            if(!compatible(p,target.tier,false)) continue;
            double r=benchmark_one(p,target.tier,false,slice);
            bool chosen_fma=false;
            // AUTO must not let benchmark noise choose between fused and unfused
            // forward-transform arithmetic.  FMA changes floating-point rounding
            // enough to move coefficients across quantizer/RDO boundaries on rare
            // blocks, so two otherwise identical AUTO runs could emit different
            // bitstreams depending on which microbenchmark happened to win.
            // Keep AUTO on the bit-exact non-fused transform kernels.  FMA remains
            // available through an explicitly forced SIMD target/override.
            (void)allow_fma;
            const int sl=target.benchmark_slot;
            result.primitive_units_per_second[pi][static_cast<size_t>(sl)]=r;
            target_fma[static_cast<size_t>(sl)]=chosen_fma;
            if(r>0){logsum[static_cast<size_t>(sl)]+=std::log(r);++nrate[static_cast<size_t>(sl)];}
        }
        const SimdTier best=choose_fastest_simd_target(result.primitive_units_per_second[pi]);
        result.dispatch.tier[pi]=best;
        const int best_slot=simd_target_benchmark_slot(best);
        result.dispatch.fma[pi]=best_slot>=0 && target_fma[static_cast<size_t>(best_slot)];
    }
    auto gmean=[&](int i){return nrate[static_cast<size_t>(i)]?std::exp(static_cast<double>(logsum[static_cast<size_t>(i)]/nrate[static_cast<size_t>(i)])):0.0;};
    result.scalar_units_per_second=gmean(VC1_SIMD_BENCH_NONE);
    result.v1_units_per_second=gmean(VC1_SIMD_BENCH_X86_64_V1);
    result.v2_units_per_second=gmean(VC1_SIMD_BENCH_X86_64_V2);
    result.v3_units_per_second=gmean(VC1_SIMD_BENCH_X86_64_V3);
    result.v4_units_per_second=gmean(VC1_SIMD_BENCH_X86_64_V4);
    result.tier=summarize(result.dispatch);
    return result;
}

} // namespace libvc1
