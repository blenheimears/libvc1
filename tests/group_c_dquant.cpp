#include <libvc1.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

static int run_case(bool dquant, int qp, bool aq, uint64_t *changed_out) {
    constexpr int W=96,H=64;
    vc1_param_t p{};
    if (vc1_param_default(&p)<0) return 1;
    p.i_width=W; p.i_height=H; p.i_fps_num=24; p.i_fps_den=1;
    p.i_threads=1; p.b_intra_gop_parallelism=1; p.i_keyint_max=24; p.i_bframes=0; p.i_motion_search_range=0;
    p.b_scene_cut=0; p.b_dquant=dquant?1:0; p.b_debug_stats=1;
    p.b_adaptive_quality=aq?1:0;
    p.i_rc_method=VC1_RC_CQP; p.i_qp_constant=qp; p.i_simd=VC1_SIMD_SCALAR;
    vc1_t *e=vc1_encoder_open(&p);
    if (!e) { std::fprintf(stderr,"open failed: %s\n",vc1_encoder_last_error(nullptr)); return 2; }

    std::vector<uint8_t> y(W*H),u((W/2)*(H/2),128),v((W/2)*(H/2),128);
    // Keep this DQUANT mechanics fixture outside the dim-luma AQ class.
    // 0.1.78 intentionally protects dim textured blocks, so a Y=120 source is
    // no longer a valid "unprotected" coarsening fixture.
    std::fill(y.begin(),y.end(),static_cast<uint8_t>(170));

    auto submit=[&](int64_t pts)->int {
        vc1_picture_t in{}; vc1_picture_init(&in); in.i_pts=pts; in.img.i_plane=3;
        in.img.plane[0]=y.data(); in.img.plane[1]=u.data(); in.img.plane[2]=v.data();
        in.img.i_stride[0]=W; in.img.i_stride[1]=W/2; in.img.i_stride[2]=W/2;
        vc1_au_t *au=nullptr; int n=0; vc1_picture_t out{};
        if (vc1_encoder_encode(e,&au,&n,&out,&in)<0) return -1;
        return 0;
    };
    if (submit(0)<0) { vc1_encoder_close(e); return 3; }
    // Add a small central patch inside each macroblock for the P picture.  The
    // 4x4 +50 patch has low average prediction error, local range below AQ's
    // contrast threshold, no >=96 hard edges, and a quiet surrounding ring.
    // It is therefore deliberately unprotected while still coefficient-bearing.
    for (int my=0;my<H;my+=16) for (int mx=0;mx<W;mx+=16)
        for (int yy=my+6;yy<std::min(my+10,H);++yy)
            for (int xx=mx+6;xx<std::min(mx+10,W);++xx)
                y[static_cast<size_t>(yy)*W+xx]=220;
    if (submit(1)<0) { vc1_encoder_close(e); return 3; }

    bool saw_p=false;
    uint64_t changed=0;
    while (vc1_encoder_delayed_frames(e)>0) {
        vc1_au_t *au=nullptr; int n=0; vc1_picture_t out{};
        if (vc1_encoder_encode(e,&au,&n,&out,nullptr)<0) { vc1_encoder_close(e); return 4; }
        if (!n) continue;
        if (au->i_type==VC1_TYPE_P) {
            saw_p=true;
            changed=au->i_debug_dquant_macroblocks;
            if (dquant && aq && qp>2) {
                if (changed==0 || au->i_debug_mquant_min<qp || au->i_debug_mquant_max<=qp ||
                    !(au->f_debug_mquant_mean>static_cast<double>(qp))) {
                    std::fprintf(stderr,"qp=%d changed=%llu min=%d max=%d mean=%.6f\n",qp,(unsigned long long)changed,au->i_debug_mquant_min,au->i_debug_mquant_max,au->f_debug_mquant_mean);
                    vc1_encoder_close(e); return 5; }
            } else if (changed!=0) {
                vc1_encoder_close(e); return 6;
            }
        }
    }
    vc1_stats_t st{};
    if (vc1_encoder_stats(e,&st)<0 || !saw_p) { vc1_encoder_close(e); return 7; }
    const bool expect_changes=dquant && aq && qp>2;
    if (expect_changes ? (st.i_dquant_macroblocks==0) : (st.i_dquant_macroblocks!=0)) { vc1_encoder_close(e); return 8; }
    vc1_encoder_close(e);
    *changed_out=changed;
    return 0;
}

int main() {
    if (LIBVC1_API_VERSION<11) return 10;
    vc1_param_t p{}; if (vc1_param_default(&p)<0 || p.b_dquant!=1) return 11;
    uint64_t on=0,off=0;
    int r=run_case(true,6,true,&on); if (r) return 20+r;
    r=run_case(false,6,true,&off); if (r) return 40+r;
    uint64_t lowq=0,noaq=0;
    r=run_case(true,2,true,&lowq); if (r) return 60+r;
    r=run_case(true,6,false,&noaq); if (r) return 80+r;
    if (lowq!=0 || noaq!=0) return 100;
    std::printf("Group-C DQUANT runtime ok (%llu changed MBs; PQ2/no-AQ protected)\n",(unsigned long long)on);
    return 0;
}
