#include "encoder_internal.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>

using namespace libvc1;

static Frame frame16(uint8_t yv=128,uint8_t uv=128,uint8_t vv=128) {
    Frame f;
    f.width=16; f.height=16;
    f.y.assign(16*16,yv);
    f.u.assign(8*8,uv);
    f.v.assign(8*8,vv);
    return f;
}

int main() {
    EncoderConfig cfg;
    cfg.width=16; cfg.height=16; cfg.display_width=16; cfg.display_height=16;
    cfg.pqindex=8; cfg.motion_search_range=8; cfg.motion_local_search_range=8;
    cfg.syntax=StreamSyntax::Advanced;
    cfg.me_quality=VC1_ME_RD;
    Vc1Encoder enc(cfg);

    // SATD must distinguish transform-friendly from spectrally scattered
    // residuals even when SAD is exactly equal.
    Frame cur=frame16(128), flat=frame16(127), noisy=frame16(128);
    uint32_t state=1;
    for (auto& px:noisy.y) {
        state=state*1664525u+1013904223u;
        px=(state&0x80000000u)?127:129;
    }
    const Vc1Encoder::MotionVector z{};
    const auto sf=enc.block_sad_motion(cur,flat,0,0,16,16,z,Vc1Encoder::ProgressiveMvMode::OneMvQpel);
    const auto sn=enc.block_sad_motion(cur,noisy,0,0,16,16,z,Vc1Encoder::ProgressiveMvMode::OneMvQpel);
    if (sf!=256 || sn!=256) { std::fprintf(stderr,"SAD fixture mismatch: %llu/%llu\n",(unsigned long long)sf,(unsigned long long)sn); return 1; }
    const auto tf=enc.block_satd_motion(cur,flat,0,0,16,16,z,Vc1Encoder::ProgressiveMvMode::OneMvQpel);
    const auto tn=enc.block_satd_motion(cur,noisy,0,0,16,16,z,Vc1Encoder::ProgressiveMvMode::OneMvQpel);
    if (!(tn>tf)) { std::fprintf(stderr,"SATD failed to distinguish residual structure: %llu/%llu\n",(unsigned long long)tf,(unsigned long long)tn); return 2; }

    // The full finalist score must include chroma. Luma is byte-identical in
    // these two current pictures; only U/V differ from the prediction.
    Frame same=frame16(128,128,128), chroma=frame16(128,210,40), pred=frame16();
    const auto rd_same=enc.motion_mb_rd_cost(same,same,0,0,z,Vc1Encoder::ProgressiveMvMode::OneMvQpel,1,pred);
    const auto rd_chroma=enc.motion_mb_rd_cost(chroma,same,0,0,z,Vc1Encoder::ProgressiveMvMode::OneMvQpel,1,pred);
    if (!(rd_chroma>rd_same)) { std::fprintf(stderr,"full RD finalist ignored chroma\n"); return 3; }

    // SAD quality is the exact compatibility path: staged refinement delegates
    // to the historical SAD-only refiner byte-for-byte in its vector/score law.
    cfg.me_quality=VC1_ME_SAD;
    Vc1Encoder sadenc(cfg);
    Frame ref=frame16(), shifted=frame16();
    for (int y=0;y<16;++y) for (int x=0;x<16;++x)
        ref.y[static_cast<size_t>(y)*16+x]=static_cast<uint8_t>((x*13+y*7+(x*y)%31)&255);
    for (int y=0;y<16;++y) for (int x=0;x<16;++x)
        shifted.y[static_cast<size_t>(y)*16+x]=ref.y[static_cast<size_t>(y)*16+std::min(15,x+1)];
    uint64_t old_sad=0,staged_sad=0;
    const auto old_mv=sadenc.refine_motion_block(shifted,ref,0,0,16,16,z,8,Vc1Encoder::ProgressiveMvMode::OneMvQpel,&old_sad);
    Vc1Encoder::PredictorInfo pi{};
    const auto staged_mv=sadenc.refine_motion_block_staged(shifted,ref,0,0,16,16,z,8,Vc1Encoder::ProgressiveMvMode::OneMvQpel,pi,true,nullptr,&staged_sad);
    if (old_mv.xq!=staged_mv.xq || old_mv.yq!=staged_mv.yq || old_sad!=staged_sad) {
        std::fprintf(stderr,"SAD compatibility path diverged\n"); return 4;
    }


    // The optimized finalist path must be bit-exact with scalar SATD.  Exercise
    // real fractional interpolation too: SIMD mode first renders the luma
    // prediction through the selected MC kernel and then uses the SSE2 4x4
    // Hadamard accumulator, while the scalar encoder keeps the historical
    // sample-at-a-time reference path.
    if (cpu_has_x86_64_v3()) {
        EncoderConfig scfg=cfg;
        scfg.width=32; scfg.height=32; scfg.display_width=32; scfg.display_height=32;
        scfg.simd_dispatch=SimdDispatch{};
        EncoderConfig vcfg=scfg;
        vcfg.simd_dispatch.tier[static_cast<size_t>(SimdPrimitive::LumaMc)]=SimdTier::X86V3;
        Vc1Encoder senc(scfg),venc(vcfg);
        Frame c32,r32; c32.width=r32.width=32; c32.height=r32.height=32;
        c32.y.resize(32*32); r32.y.resize(32*32); c32.u.resize(16*16,128); c32.v.resize(16*16,128); r32.u=c32.u; r32.v=c32.v;
        uint32_t q=7;
        for(size_t i=0;i<c32.y.size();++i){q=q*1664525u+1013904223u;c32.y[i]=static_cast<uint8_t>(q>>24);q=q*1664525u+1013904223u;r32.y[i]=static_cast<uint8_t>(q>>24);}
        for(int yq=-7;yq<=7;++yq) for(int xq=-7;xq<=7;++xq) {
            Vc1Encoder::MotionVector mv{xq,yq};
            const auto a=senc.block_satd_motion(c32,r32,8,8,16,16,mv,Vc1Encoder::ProgressiveMvMode::OneMvQpel);
            const auto b=venc.block_satd_motion(c32,r32,8,8,16,16,mv,Vc1Encoder::ProgressiveMvMode::OneMvQpel);
            if(a!=b){std::fprintf(stderr,"SIMD SATD mismatch at (%d,%d): %llu/%llu\n",xq,yq,(unsigned long long)a,(unsigned long long)b);return 5;}
        }
        for(int yq=-6;yq<=6;yq+=2) for(int xq=-6;xq<=6;xq+=2) {
            Vc1Encoder::MotionVector mv{xq,yq};
            const auto a=senc.block_satd_motion(c32,r32,8,8,16,16,mv,Vc1Encoder::ProgressiveMvMode::OneMvHpelBilinear);
            const auto b=venc.block_satd_motion(c32,r32,8,8,16,16,mv,Vc1Encoder::ProgressiveMvMode::OneMvHpelBilinear);
            if(a!=b){std::fprintf(stderr,"SIMD bilinear SATD mismatch at (%d,%d): %llu/%llu\n",xq,yq,(unsigned long long)a,(unsigned long long)b);return 6;}
        }
    }


    // Distant-match confidence gate: absolute-good remains unconditional;
    // relative improvement is accepted only below the configured MAE ceiling.
    cfg.distant_match_max_mae=12.0;
    Vc1Encoder gateenc(cfg);
    if (gateenc.distant_match_decision(5u*256u,40u*256u,0,0)!=VC1_DISTANT_MATCH_ABSOLUTE_GOOD) return 7;
    if (gateenc.distant_match_decision(11u*256u,20u*256u,0,0)!=VC1_DISTANT_MATCH_MATERIAL_IMPROVEMENT) return 8;
    if (gateenc.distant_match_decision(29u*256u,40u*256u,0,0)!=VC1_DISTANT_MATCH_REJECTED) return 9;
    if (gateenc.distant_match_decision(11u*256u,13u*256u,0,0)!=VC1_DISTANT_MATCH_REJECTED) return 10;

    std::puts("staged motion estimation ok");
    return 0;
}
