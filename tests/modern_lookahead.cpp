#include <libvc1.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

static void fill_frame(std::vector<uint8_t>& y,int w,int h,int frame) {
    const int gop=frame<12?0:(frame<24?1:(frame<36?2:3));
    const int k=frame%12;
    const bool hard=gop==3;
    for (int yy=0;yy<h;++yy) for (int x=0;x<w;++x) {
        int v;
        if (hard) {
            v=48+(((x*19+yy*23+k*29+(x*yy)%131)^((x*3+k*11)+(yy*7)))%176);
        } else {
            v=80+(((x/6+yy/6+k/3+gop)&1)*28)+((x*3+yy*5+k*7)%13);
        }
        y[static_cast<size_t>(yy)*w+x]=static_cast<uint8_t>(v<16?16:(v>235?235:v));
    }
}

int main() {
    constexpr int w=96,h=64,frames=42;
    vc1_param_t p{};
    if (vc1_param_default(&p)<0) return 1;
    p.i_width=w; p.i_height=h; p.i_fps_num=24; p.i_fps_den=1;
    p.i_threads=1; p.i_keyint_max=24; p.i_bframes=2; p.i_motion_search_range=4;
    p.b_scene_cut=0; p.b_fixed_gop_grid=1; p.b_variable_transforms=0; p.i_trellis=0;
    p.b_adaptive_quality=0; p.b_loop_filter=0; p.i_simd=VC1_SIMD_SCALAR;
    p.i_rc_method=VC1_RC_ABR; p.i_bitrate=1400000; p.i_vbv_buffer_size=2800000;
    p.b_rc_maximize=0;
    vc1_t* e=vc1_encoder_open(&p);
    if (!e) { std::fprintf(stderr,"open: %s\n",vc1_encoder_last_error(nullptr)); return 2; }

    std::vector<uint8_t> y(static_cast<size_t>(w)*h),u(static_cast<size_t>(w/2)*(h/2),128),v=u;
    std::array<double,4> target{{0,0,0,0}};
    std::array<uint64_t,4> actual{{0,0,0,0}};
    int output=0;
    auto consume=[&](vc1_au_t* au,int n) {
        if (!n || !au) return;
        const uint64_t d=au->i_display_order;
        const size_t gi=d<12?0:(d<24?1:(d<36?2:3));
        target[gi]+=au->f_rc_target_bits;
        actual[gi]+=static_cast<uint64_t>(au->i_payload)*8ull;
        ++output;
    };

    for (int f=0;f<frames;++f) {
        fill_frame(y,w,h,f);
        vc1_picture_t in{},out{}; vc1_picture_init(&in); vc1_picture_init(&out);
        in.i_pts=f; in.i_type=(f==12 || f==24 || f==36)?VC1_TYPE_I:VC1_TYPE_AUTO; in.img.i_plane=3;
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
    vc1_stats_t st{}; vc1_encoder_stats(e,&st); vc1_encoder_close(e);
    if (output!=frames) { std::fprintf(stderr,"output=%d\n",output); return 6; }

    const std::array<double,4> nominal{{700000.0,700000.0,700000.0,350000.0}};
    const double combined=target[0]+target[1]+target[2]+target[3];
    const double nominal_total=nominal[0]+nominal[1]+nominal[2]+nominal[3];
    // G0 cannot see the hard tail yet and should remain nominal. G1/G2 can
    // see it in the sliding +2-GOP window and reserve target bits; the short
    // hard G3 then spends only that saved credit. A non-overlapping 3-GOP
    // planner would miss this boundary case entirely.
    if (!(std::abs(target[0]-nominal[0])<nominal[0]*0.02 &&
          target[1]<nominal[1]*0.98 && target[2]<nominal[2]*0.98 &&
          target[3]>nominal[3]*1.15 && target[3]<=nominal[3]*1.605)) {
        std::fprintf(stderr,"sliding lookahead did not reserve across boundary: %.0f %.0f %.0f %.0f\n",
                     target[0],target[1],target[2],target[3]); return 7;
    }
    if (std::abs(combined-nominal_total)>nominal_total*0.02) {
        std::fprintf(stderr,"lookahead changed combined target: %.0f vs %.0f\n",combined,nominal_total); return 8;
    }
    if (!(actual[3]>actual[0] && actual[3]>actual[1] && actual[3]>actual[2])) {
        std::fprintf(stderr,"hard GOP was not allowed more coded bits: %llu %llu %llu %llu\n",
                     (unsigned long long)actual[0],(unsigned long long)actual[1],
                     (unsigned long long)actual[2],(unsigned long long)actual[3]); return 10;
    }
    if (st.i_hrd_underflows!=0) { std::fprintf(stderr,"HRD underflows=%llu\n",(unsigned long long)st.i_hrd_underflows); return 9; }
    std::printf("modern lookahead targets %.0f/%.0f/%.0f/%.0f bits; actual %llu/%llu/%llu/%llu\n",
                target[0],target[1],target[2],target[3],(unsigned long long)actual[0],
                (unsigned long long)actual[1],(unsigned long long)actual[2],(unsigned long long)actual[3]);
    return 0;
}
