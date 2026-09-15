#include "encoder_internal.h"
#include <cstdio>
#include <cstdlib>

using namespace libvc1;

static const char* tier_name(SimdTier t) {
    switch (t) {
        case SimdTier::None: return "none";
        case SimdTier::Mixed: return "mixed";
        case SimdTier::X86V1: return "x86-64-v1";
        case SimdTier::X86V2: return "x86-64-v2";
        case SimdTier::X86V3: return "x86-64-v3";
        case SimdTier::X86V4: return "x86-64-v4";
        case SimdTier::Prescott: return "prescott";
        case SimdTier::K10: return "k10";
        case SimdTier::Conroe: return "conroe";
        case SimdTier::Penryn: return "penryn";
        case SimdTier::SandyBridge: return "sandybridge";
        case SimdTier::Bulldozer: return "bulldozer";
        case SimdTier::Piledriver: return "piledriver";
        case SimdTier::Avx2Partial: return "avx2-partial";
    }
    return "invalid";
}

static const char* primitive_name(size_t i) {
    static constexpr const char* names[] = {
        "frame-sad","block-sad","forward-intra8","forward-residual8","inverse8x8",
        "put-block8","add-block-rect","luma-qpel-sad","luma-bilinear-sad","luma-mc",
        "luma-mc-avg","chroma-mc","chroma-mc-avg","intensity-luma","intensity-chroma",
        "forward-residual-rect","quantize","perceptual-stats","sum-u8"
    };
    return i < sizeof(names)/sizeof(names[0]) ? names[i] : "unknown";
}

int main() {
#if defined(_WIN32)
    if (!std::getenv("LIBVC1_SIMD_BENCHMARK_MS")) _putenv_s("LIBVC1_SIMD_BENCHMARK_MS", "400");
#else
    if (!std::getenv("LIBVC1_SIMD_BENCHMARK_MS")) setenv("LIBVC1_SIMD_BENCHMARK_MS", "400", 1);
#endif
    /* Regression for the old 1% hysteresis: the strictly faster v4 result must win. */
    std::array<double,VC1_SIMD_BENCH_TARGET_COUNT> synthetic{};
    synthetic[VC1_SIMD_BENCH_NONE]=72146941.12;
    synthetic[VC1_SIMD_BENCH_X86_64_V3]=148072264.61;
    synthetic[VC1_SIMD_BENCH_X86_64_V4]=148793942.90;
    if (choose_fastest_simd_target(synthetic)!=SimdTier::X86V4) return 20;
    /* Exact v3/v4 ties deliberately prefer the earlier registry target (v3). */
    synthetic[VC1_SIMD_BENCH_X86_64_V4]=synthetic[VC1_SIMD_BENCH_X86_64_V3];
    if (choose_fastest_simd_target(synthetic)!=SimdTier::X86V3) return 21;
    synthetic.fill(0.0); synthetic[VC1_SIMD_BENCH_NONE]=72146941.12; synthetic[VC1_SIMD_BENCH_X86_64_V1]=72146941.12;
    if (choose_fastest_simd_target(synthetic)!=SimdTier::None) return 22;
    synthetic.fill(0.0); synthetic[VC1_SIMD_BENCH_X86_64_V1]=90000000.0; synthetic[VC1_SIMD_BENCH_X86_64_V2]=90000000.0;
    if (choose_fastest_simd_target(synthetic)!=SimdTier::X86V1) return 28;

    const bool v1=simd_target_available(SimdTier::X86V1);
    const bool prescott=simd_target_available(SimdTier::Prescott);
    const bool k10=simd_target_available(SimdTier::K10);
    const bool conroe=simd_target_available(SimdTier::Conroe);
    const bool penryn=simd_target_available(SimdTier::Penryn);
    const bool v2=simd_target_available(SimdTier::X86V2);
    const bool sandy=simd_target_available(SimdTier::SandyBridge);
    const bool bulldozer=simd_target_available(SimdTier::Bulldozer);
    const bool piledriver=simd_target_available(SimdTier::Piledriver);
    const bool avx2partial=simd_target_available(SimdTier::Avx2Partial);
    const bool v3=simd_target_available(SimdTier::X86V3);
    const bool v4=simd_target_available(SimdTier::X86V4);
    struct Target { SimdTier tier; int slot; bool avail; int err; };
    const Target named[] = {
        {SimdTier::Prescott,VC1_SIMD_BENCH_PRESCOTT,prescott,31},
        {SimdTier::K10,VC1_SIMD_BENCH_K10,k10,32},
        {SimdTier::Conroe,VC1_SIMD_BENCH_CONROE,conroe,33},
        {SimdTier::Penryn,VC1_SIMD_BENCH_PENRYN,penryn,34},
        {SimdTier::SandyBridge,VC1_SIMD_BENCH_SANDYBRIDGE,sandy,35},
        {SimdTier::Bulldozer,VC1_SIMD_BENCH_BULLDOZER,bulldozer,36},
        {SimdTier::Piledriver,VC1_SIMD_BENCH_PILEDRIVER,piledriver,37},
        {SimdTier::Avx2Partial,VC1_SIMD_BENCH_AVX2_PARTIAL,avx2partial,38},
    };
    for (const auto& m : named) {
        if (!simd_target_implemented(m.tier)) return 26;
        if (simd_target_benchmark_slot(m.tier)!=m.slot) return 27;
    }
    // K10/SSE4a is a gap target, not a preference over a complete generic v2.
    if (k10 && v2 && simd_target_auto_eligible(SimdTier::K10,false)) return 76;
    const auto r=auto_select_simd(true,false);
    const auto rall=auto_select_simd(true,true);
    if (!(r.scalar_units_per_second>0.0)) return 2;

    // The three forward-transform primitives have dedicated v1/v2/v3/v4 kernels.
    // Their benchmark rates must be present whenever the CPU exposes the tier;
    // an n/a here means the compatibility gate rejected a stale/broken kernel.
    constexpr SimdPrimitive forward_primitives[] = {
        SimdPrimitive::ForwardIntra8,
        SimdPrimitive::ForwardResidual8,
        SimdPrimitive::ForwardResidualRect,
    };
    for (const SimdPrimitive p : forward_primitives) {
        const size_t i=static_cast<size_t>(p);
        if (v1 && !(r.primitive_units_per_second[i][VC1_SIMD_BENCH_X86_64_V1]>0.0)) return 29;
        if (v2 && !(r.primitive_units_per_second[i][VC1_SIMD_BENCH_X86_64_V2]>0.0)) return 23;
        if (v3 && !(r.primitive_units_per_second[i][VC1_SIMD_BENCH_X86_64_V3]>0.0)) return 24;
        if (v4 && !(r.primitive_units_per_second[i][VC1_SIMD_BENCH_X86_64_V4]>0.0)) return 25;
        for(const auto& m:named){
            const bool eligible=simd_target_auto_eligible(m.tier,false);
            if(m.avail && !(rall.primitive_units_per_second[i][m.slot]>0.0)) return m.err;
            if(!m.avail && rall.primitive_units_per_second[i][m.slot]!=0.0) return m.err+10;
            if(!eligible && r.primitive_units_per_second[i][m.slot]!=0.0) return m.err+20;
            if(eligible && m.avail && !(r.primitive_units_per_second[i][m.slot]>0.0)) return m.err+30;
        }
    }
    // benchmark-all widens policy only; it must never bypass ISA compatibility.
    // This is particularly important for FMA4: Intel and Zen-class CPUs where
    // CPUID does not expose FMA4 must never execute Bulldozer/Piledriver code.
    for (const auto& m : named) {
        if (!m.avail) {
            if (simd_target_auto_eligible(m.tier,true)) return m.err+40;
            for (size_t i=0;i<kSimdPrimitiveCount;++i)
                if (rall.primitive_units_per_second[i][m.slot]!=0.0) return m.err+50;
        }
    }
    for (size_t i=0;i<kSimdPrimitiveCount;++i) {
        const SimdTier t=r.dispatch.tier[i];
        if (t==SimdTier::Mixed) return 3;
        if (t==SimdTier::X86V1 && !v1) return 30;
        if ((t==SimdTier::Prescott || t==SimdTier::K10 || t==SimdTier::Conroe || t==SimdTier::Penryn ||
             t==SimdTier::SandyBridge || t==SimdTier::Bulldozer || t==SimdTier::Piledriver ||
             t==SimdTier::Avx2Partial) && !simd_target_auto_eligible(t,false)) return 75;
        if (t==SimdTier::X86V2 && !v2) return 4;
        if (t==SimdTier::X86V3 && !v3) return 4;
        if (t==SimdTier::X86V4 && !v4) return 5;
        if (r.dispatch.fma[i] && !(t==SimdTier::Bulldozer || t==SimdTier::Piledriver || t==SimdTier::X86V3 || t==SimdTier::X86V4)) return 6;
        const auto expected=choose_fastest_simd_target(r.primitive_units_per_second[i]);
        if (t!=expected) return 7;
        const char* fused = r.dispatch.fma[i] ? (t==SimdTier::Bulldozer ? "+fma4" : "+fma3") : "";
        std::printf("primitive[%zu] %s=%s%s\n",i,primitive_name(i),tier_name(t),fused);
    }
    std::printf("summary=%s none=%.2f v1=%.2f v2=%.2f v3=%.2f v4=%.2f\n",tier_name(r.tier),r.scalar_units_per_second,r.v1_units_per_second,r.v2_units_per_second,r.v3_units_per_second,r.v4_units_per_second);
    return 0;
}
