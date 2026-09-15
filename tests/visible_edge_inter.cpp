#include "encoder_internal.h"
#include "forward_transform_tables.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

using libvc1::EncoderConfig;
using libvc1::Frame;
using libvc1::Vc1Encoder;

static std::array<double,64> reference_8x8(const std::array<double,64>& residual) {
    double tmp[8][8]{};
    double xform[8][8]{};
    for (int a=0;a<8;++a)
        for (int x=0;x<8;++x)
            for (int y=0;y<8;++y)
                tmp[a][x] += libvc1::forward_transform::k8[a][y]*residual[static_cast<size_t>(y)*8+x];
    for (int a=0;a<8;++a)
        for (int bb=0;bb<8;++bb)
            for (int x=0;x<8;++x)
                xform[a][bb] += tmp[a][x]*libvc1::forward_transform::k8[bb][x];
    std::array<double,64> out{};
    for (int row=0;row<8;++row)
        for (int col=0;col<8;++col)
            out[static_cast<size_t>(row)*8+col]=1024.0*xform[col][row];
    return out;
}

static bool close(double a,double b) {
    const double scale=std::max({1.0,std::abs(a),std::abs(b)});
    return std::abs(a-b)<=1e-10*scale;
}

int main() {
    EncoderConfig c;
    c.width=18; c.height=18;
    c.motion_search_range=2; c.motion_local_search_range=2;
    c.fade_compensation=false; c.adaptive_quality=false; c.trellis=0;
    c.variable_transforms=false; c.loop_filter=false; c.overlap=false;
    Vc1Encoder enc(c);

    Frame src, pred;
    src.width=pred.width=18; src.height=pred.height=18;
    src.y.assign(18*18,100); pred.y.assign(18*18,90);
    src.u.assign(9*9,128); pred.u.assign(9*9,128);
    src.v.assign(9*9,128); pred.v.assign(9*9,128);

    // The bottom-right MB has only 2x2 visible luma samples. Motion costs must
    // therefore be four ten-level differences, not a replicated 16x16 block.
    const uint64_t expected_sad=4u*10u;
    if (enc.mb_sad(src,pred,1,1,0,0,1)!=expected_sad) {
        std::fprintf(stderr,"bottom-right mb_sad included hidden pixels\n");
        return 1;
    }
    if (enc.block_sad_qpel(src,pred,16,16,16,16,{0,0})!=expected_sad) {
        std::fprintf(stderr,"bottom-right qpel SAD included hidden pixels\n");
        return 2;
    }
    if (enc.block_sad_motion(src,pred,16,16,16,16,{0,0},Vc1Encoder::ProgressiveMvMode::OneMvHpelBilinear)!=expected_sad) {
        std::fprintf(stderr,"bottom-right bilinear SAD included hidden pixels\n");
        return 3;
    }

    // Inter residual coding must zero-pad the delta itself. With only a 2x2
    // visible corner, the other 60 transform inputs are zero residual rather
    // than copies of the last visible source/prediction sample.
    std::array<double,64> residual{};
    for (int y=0;y<2;++y) for (int x=0;x<2;++x) residual[static_cast<size_t>(y)*8+x]=10.0;
    const auto expected_coeff=reference_8x8(residual);
    const auto got_coeff=enc.forward_transform_residual_part(src.y,pred.y,18,18,16,16,8,8);
    for (size_t i=0;i<64;++i) if (!close(expected_coeff[i],got_coeff[i])) {
        std::fprintf(stderr,"visible-edge residual transform mismatch at %zu: %.17g != %.17g\n",i,got_coeff[i],expected_coeff[i]);
        return 4;
    }

    // A parent entirely outside the display crop has no source delta and must
    // never consume coefficient bits merely because the coded MB is padded.
    const auto invisible=enc.quantize_inter_part(src.y,pred.y,18,18,24,16,8,8,false,c.pqindex,nullptr);
    if (std::any_of(invisible.begin(),invisible.end(),[](int v){return v!=0;})) {
        std::fprintf(stderr,"fully hidden inter parent produced coefficients\n");
        return 5;
    }

    // P/B intra *analysis* uses only visible samples. Keep actual block_mean()
    // edge-extension behavior separate because coded intra padding still needs
    // a deterministic source extension.
    std::fill(src.y.begin(),src.y.end(),0);
    src.y[16*18+16]=10; src.y[16*18+17]=20;
    src.y[17*18+16]=30; src.y[17*18+17]=40;
    const int visible_mean=Vc1Encoder::visible_block_mean(src.y,18,18,16,16);
    const int coded_mean=Vc1Encoder::block_mean(src.y,18,18,16,16);
    if (visible_mean!=25) {
        std::fprintf(stderr,"visible edge mean is %d, expected 25\n",visible_mean);
        return 6;
    }
    if (coded_mean==visible_mean) {
        std::fprintf(stderr,"test fixture does not distinguish visible and padded means\n");
        return 7;
    }

    // Regressions for the reported mode-selection symptom. These deterministic
    // fixtures used to choose P-intra solely on bottom/right edge macroblocks
    // because the spatial proxy's mean was dominated by replicated hidden
    // samples. The visible-only proxy keeps the temporal mode when the visible
    // samples do not justify an intra escape.
    auto random_frame=[](uint32_t seed) {
        Frame f; f.width=18; f.height=18;
        f.y.resize(18*18); f.u.assign(9*9,128); f.v.assign(9*9,128);
        uint32_t state=seed;
        for (auto& v:f.y) { state=state*1664525u+1013904223u; v=static_cast<uint8_t>(state>>24); }
        return f;
    };
    auto perturb=[](Frame& ref,const Frame& cur,uint32_t seed,int amp) {
        ref=cur; uint32_t state=seed;
        for (int y=0;y<18;++y) for (int x=0;x<18;++x) {
            state=state*1103515245u+12345u;
            const int d=static_cast<int>((state>>24)%static_cast<uint32_t>(2*amp+1))-amp;
            const size_t off=static_cast<size_t>(y)*18+x;
            ref.y[off]=static_cast<uint8_t>(std::clamp(static_cast<int>(cur.y[off])+d,0,255));
        }
    };
    auto p_edge_ok=[&](uint32_t seed,size_t edge_pos) {
        Frame cur=random_frame(seed), ref; perturb(ref,cur,seed^0xa5a5a5a5u,180);
        const auto a=enc.analyze_p_picture_core(cur,ref,Vc1Encoder::ProgressiveMvMode::OneMvQpel,true,nullptr);
        return edge_pos<a.use_intra.size() && a.use_intra[edge_pos]==0;
    };
    if (!p_edge_ok(814,2)) { std::fprintf(stderr,"bottom-edge P macroblock regressed to intra\n"); return 8; }
    if (!p_edge_ok(3950,1)) { std::fprintf(stderr,"right-edge P macroblock regressed to intra\n"); return 9; }
    if (!p_edge_ok(1214,3)) { std::fprintf(stderr,"bottom-right P macroblock regressed to intra\n"); return 10; }

    // The B-intra proxy uses the same visible-only mean. Verify both a bottom
    // and a right edge case without requiring any unsupported SIMD target.
    std::vector<Vc1Encoder::MotionVector> anchor_mvs(4);
    std::vector<uint8_t> anchor_4mv(4,0);
    Vc1Encoder::IntensityComp no_ic{};
    auto b_edge_ok=[&](uint32_t seed,size_t edge_pos) {
        Frame cur=random_frame(seed), past, future;
        perturb(past,cur,seed^0xa5a5a5a5u,255);
        perturb(future,cur,seed^0x5a5a5a5au,255);
        const auto a=enc.analyze_b_picture_mode(cur,past,future,anchor_mvs,anchor_4mv,no_ic,1,2,
                                                Vc1Encoder::ProgressiveMvMode::OneMvQpel);
        return edge_pos<a.modes.size() && a.modes[edge_pos]!=Vc1Encoder::BMbMode::Intra;
    };
    if (!b_edge_ok(1,2)) { std::fprintf(stderr,"bottom-edge B macroblock regressed to intra\n"); return 11; }
    if (!b_edge_ok(89,1)) { std::fprintf(stderr,"right-edge B macroblock regressed to intra\n"); return 12; }


    // Intra transforms must use black luma padding, not edge replication.  The
    // visible 2x2 corner is 100; all 60 hidden inputs are sample value zero.
    std::array<double,64> intra_samples{};
    intra_samples.fill(-128.0);
    for (int y=0;y<2;++y) for (int x=0;x<2;++x)
        intra_samples[static_cast<size_t>(y)*8+x]=-28.0;
    const auto expected_intra_coeff=reference_8x8(intra_samples);
    std::vector<uint8_t> intra_plane(18*18,100);
    const auto got_intra_coeff=enc.forward_transform(intra_plane,18,18,16,16,0);
    for (size_t i=0;i<64;++i) if (!close(expected_intra_coeff[i],got_intra_coeff[i])) {
        std::fprintf(stderr,"visible-edge intra transform did not zero hidden luma at %zu\n",i);
        return 13;
    }
    if (Vc1Encoder::padded_block_mean(intra_plane,18,18,16,16,0)!=6) {
        std::fprintf(stderr,"visible-edge intra DC mean did not use black hidden padding\n");
        return 14;
    }

    // When the display edge lands four samples into an 8x8 parent, variable
    // transform RDO must split at that edge instead of ever considering 8x8.
    auto check_edge_transform=[&](int w,int h,bool need_x4,bool need_y4) {
        EncoderConfig ec=c; ec.width=w; ec.height=h; ec.variable_transforms=true;
        Vc1Encoder e(ec);
        Frame a,b; a.width=b.width=w; a.height=b.height=h;
        a.y.resize(static_cast<size_t>(w)*h); b.y.resize(a.y.size());
        a.u.assign(static_cast<size_t>(w/2)*(h/2),128); b.u=a.u;
        a.v=a.u; b.v=a.u;
        for (int y=0;y<h;++y) for (int x=0;x<w;++x) {
            a.y[static_cast<size_t>(y)*w+x]=static_cast<uint8_t>((17*x+29*y+91)&255);
            b.y[static_cast<size_t>(y)*w+x]=static_cast<uint8_t>((3*x+5*y+7)&255);
        }
        const int mx=(w-1)/16,my=(h-1)/16;
        const auto d=e.choose_transform_mb(a,b,mx,my,0,false,6,false,nullptr);
        for (const auto& parent:d.parents) {
            if (need_x4 && (parent.type==Vc1Encoder::TransformType::T8x8 || parent.type==Vc1Encoder::TransformType::T8x4)) return false;
            if (need_y4 && (parent.type==Vc1Encoder::TransformType::T8x8 || parent.type==Vc1Encoder::TransformType::T4x8)) return false;
        }
        return true;
    };
    if (!check_edge_transform(20,16,true,false)) {
        std::fprintf(stderr,"right crop edge allowed an 8-wide transform to cross hidden samples\n");
        return 15;
    }
    if (!check_edge_transform(16,20,false,true)) {
        std::fprintf(stderr,"bottom crop edge allowed an 8-high transform to cross hidden samples\n");
        return 16;
    }
    if (!check_edge_transform(20,20,true,true)) {
        std::fprintf(stderr,"bottom-right crop edge allowed a transform to cross hidden samples\n");
        return 17;
    }

    // Edge macroblocks now retain both motion-vector components.  Reference
    // padding/clamping handles prediction outside the displayed raster instead
    // of forcing horizontal or vertical motion to zero.

    // Verify real P/B analysis retains useful horizontal motion on a partial
    // right-edge macroblock. This fixture's best temporal match is one pixel
    // to the left; clamping x to zero used to create a visible edge discontinuity.
    {
        EncoderConfig ec=c; ec.width=30; ec.height=16; ec.display_width=30; ec.display_height=16;
        ec.motion_search_range=2; ec.motion_local_search_range=2; ec.me_quality=VC1_ME_SAD;
        ec.debug_disable_b_intra=true; ec.variable_transforms=true;
        Vc1Encoder e(ec);
        Frame r,cur; r.width=cur.width=30; r.height=cur.height=16;
        r.y.resize(30*16); cur.y.resize(30*16);
        r.u.assign(15*8,128); r.v=r.u; cur.u=r.u; cur.v=r.u;
        for (int yy=0;yy<16;++yy) for (int xx=0;xx<30;++xx)
            r.y[static_cast<size_t>(yy)*30+xx]=static_cast<uint8_t>((37*xx+19*yy+11)&255);
        for (int yy=0;yy<16;++yy) for (int xx=0;xx<30;++xx)
            cur.y[static_cast<size_t>(yy)*30+xx]=r.y[static_cast<size_t>(yy)*30+std::max(0,xx-1)];
        const auto pa=e.analyze_p_picture_core(cur,r,Vc1Encoder::ProgressiveMvMode::OneMvQpel,false,nullptr);
        if (pa.mvs.size()!=2 || pa.mvs[1].xq==0) {
            std::fprintf(stderr,"P right-edge macroblock lost useful horizontal motion\n");
            return 18;
        }
        std::vector<Vc1Encoder::MotionVector> anchors(2);
        std::vector<uint8_t> anchor4(2,0);
        Vc1Encoder::IntensityComp ic{};
        const auto ba=e.analyze_b_picture_mode(cur,r,r,anchors,anchor4,ic,1,2,Vc1Encoder::ProgressiveMvMode::OneMvQpel);
        if (ba.forward_mvs.size()!=2 || ba.backward_mvs.size()!=2 ||
            (ba.forward_mvs[1].xq==0 && ba.backward_mvs[1].xq==0)) {
            std::fprintf(stderr,"B right-edge macroblock had both horizontal motion fields forced to zero\n");
            return 19;
        }
    }

    return 0;
}
