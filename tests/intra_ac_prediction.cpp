#include <libvc1.h>
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

static void fill_frame(std::vector<uint8_t>& y,int w,int h,int frame) {
    static constexpr int wave[8]={-32,-24,-12,4,18,28,36,42};
    for (int yy=0;yy<h;++yy) for (int x=0;x<w;++x) {
        const int bx=x/8;
        int v=70+bx*7+(frame&1)+wave[x&7];
        if (v<16) v=16; else if (v>235) v=235;
        y[static_cast<size_t>(yy)*w+x]=static_cast<uint8_t>(v);
    }
}

static int encode(int threads,std::vector<uint8_t>& bytes) {
    constexpr int w=128,h=128,frames=6;
    vc1_param_t p{};
    if (vc1_param_default(&p)<0) return 1;
    p.i_width=w; p.i_height=h; p.i_fps_num=24; p.i_fps_den=1;
    p.i_profile=VC1_PROFILE_ADVANCED;
    p.i_threads=threads; p.i_keyint_max=24; p.i_bframes=0;
    p.b_scene_cut=0; p.b_variable_transforms=0; p.i_trellis=0;
    p.b_adaptive_quality=0; p.b_loop_filter=0; p.i_simd=VC1_SIMD_SCALAR;
    p.b_intra_only=1; p.b_ac_coding=1; p.i_ac_mode=VC1_AC_AUTO;
    p.i_rc_method=VC1_RC_CQP; p.i_qp_constant=8;
    vc1_t* e=vc1_encoder_open(&p);
    if (!e) { std::fprintf(stderr,"open: %s\n",vc1_encoder_last_error(nullptr)); return 2; }

    std::vector<uint8_t> y(static_cast<size_t>(w)*h),u(static_cast<size_t>(w/2)*(h/2),128),v=u;
    int output=0;
    auto consume=[&](vc1_au_t* au,int n) {
        if (!n || !au) return;
        bytes.insert(bytes.end(),au->p_payload,au->p_payload+au->i_payload);
        ++output;
    };
    for (int f=0;f<frames;++f) {
        fill_frame(y,w,h,f);
        vc1_picture_t in{},out{}; vc1_picture_init(&in); vc1_picture_init(&out);
        in.i_pts=f; in.i_type=VC1_TYPE_AUTO; in.img.i_plane=3;
        in.img.plane[0]=y.data(); in.img.plane[1]=u.data(); in.img.plane[2]=v.data();
        in.img.i_stride[0]=w; in.img.i_stride[1]=w/2; in.img.i_stride[2]=w/2;
        vc1_au_t* au=nullptr; int n=0;
        if (vc1_encoder_encode(e,&au,&n,&out,&in)<0) {
            std::fprintf(stderr,"encode: %s\n",vc1_encoder_last_error(e)); vc1_encoder_close(e); return 3;
        }
        consume(au,n);
    }
    while (vc1_encoder_delayed_frames(e)>0) {
        vc1_au_t* au=nullptr; int n=0; vc1_picture_t out{}; vc1_picture_init(&out);
        if (vc1_encoder_encode(e,&au,&n,&out,nullptr)<0) {
            std::fprintf(stderr,"flush: %s\n",vc1_encoder_last_error(e)); vc1_encoder_close(e); return 4;
        }
        if (!n) { std::fprintf(stderr,"flush made no progress\n"); vc1_encoder_close(e); return 5; }
        consume(au,n);
    }
    vc1_encoder_close(e);
    if (output!=frames) { std::fprintf(stderr,"output=%d\n",output); return 6; }
    return 0;
}

int main() {
    std::vector<uint8_t> one,four;
    if (int rc=encode(1,one)) return rc;
    if (int rc=encode(4,four)) return 10+rc;
    if (one!=four) { std::fprintf(stderr,"ACPRED output differs across thread counts\n"); return 20; }
    // This repeated directional-AC fixture is ~6.8 kB without ACPRED at the
    // same CQP settings. Keep generous headroom while still detecting a return
    // to hard-wired ACPRED=0 or a broken directional residual path.
    if (one.size()>=2500) {
        std::fprintf(stderr,"ACPRED fixture too large: %zu bytes\n",one.size());
        return 21;
    }
    std::printf("intra AC prediction fixture: %zu bytes, deterministic\n",one.size());
    return 0;
}
