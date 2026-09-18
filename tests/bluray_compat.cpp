#include <libvc1.h>
#include "../src/encoder_two_pass.h"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
vc1_param_t base() {
    vc1_param_t p{};
    if (vc1_param_default(&p)<0) throw std::runtime_error("vc1_param_default failed");
    p.i_profile=VC1_PROFILE_ADVANCED;
    p.i_width=1920; p.i_height=1080;
    p.i_fps_num=24; p.i_fps_den=1;
    p.i_scan_mode=VC1_SCAN_PROGRESSIVE;
    p.i_threads=1; p.i_bframes=0; p.b_intra_only=1;
    p.i_simd=VC1_SIMD_NONE;
    p.i_rc_method=VC1_RC_CQP; p.i_qp_constant=20;
    return p;
}

struct BitReader {
    const std::vector<uint8_t>& b;
    size_t bit=0;
    uint64_t bits(int n) {
        if (n<0 || n>64 || bit+static_cast<size_t>(n)>b.size()*8u)
            throw std::runtime_error("sequence header truncated");
        uint64_t v=0;
        for (int i=0;i<n;++i) {
            v=(v<<1) | ((b[bit>>3] >> (7-(bit&7))) & 1u);
            ++bit;
        }
        return v;
    }
};

std::vector<uint8_t> unescape_sequence(const vc1_au_t& h) {
    if (!h.p_payload || h.i_payload<5 || h.p_payload[0]!=0 || h.p_payload[1]!=0 ||
        h.p_payload[2]!=1 || h.p_payload[3]!=0x0f)
        throw std::runtime_error("missing Advanced sequence start code");
    std::vector<uint8_t> out;
    int zeros=0;
    for (size_t i=4;i<h.i_payload;++i) {
        const uint8_t v=h.p_payload[i];
        if (zeros>=2 && v==0x03) { zeros=0; continue; }
        out.push_back(v);
        if (v==0) ++zeros; else zeros=0;
    }
    return out;
}

struct FrameRateSignal {
    bool display_ext=false;
    bool framerate_flag=false;
    int nr=0;
    int dr=0;
};

FrameRateSignal sequence_rate(vc1_param_t p) {
    vc1_t* e=vc1_encoder_open(&p);
    if (!e) throw std::runtime_error(std::string("open failed: ")+vc1_encoder_last_error(nullptr));
    vc1_au_t* h=nullptr; int n=0;
    if (vc1_encoder_headers(e,&h,&n)<=0 || n!=1) {
        const std::string err=vc1_encoder_last_error(e);
        vc1_encoder_close(e);
        throw std::runtime_error("headers failed: "+err);
    }
    const auto rbdu=unescape_sequence(*h);
    vc1_encoder_close(e);
    BitReader br{rbdu};
    if (br.bits(2)!=3) throw std::runtime_error("not Advanced Profile");
    (void)br.bits(3); // LEVEL
    (void)br.bits(2); // COLORDIFF_FORMAT
    (void)br.bits(3); // FRMRTQ_POSTPROC
    (void)br.bits(5); // BITRTQ_POSTPROC
    (void)br.bits(1); // POSTPROCFLAG
    (void)br.bits(12); (void)br.bits(12); // coded width/height
    for (int i=0;i<6;++i) (void)br.bits(1); // PULLDOWN..PSF
    FrameRateSignal r;
    r.display_ext=br.bits(1)!=0;
    if (!r.display_ext) return r;
    (void)br.bits(14); (void)br.bits(14); // display width/height
    if (br.bits(1)) { // ASPECT_RATIO_FLAG
        const int ar=static_cast<int>(br.bits(4));
        if (ar==15) { (void)br.bits(8); (void)br.bits(8); }
    }
    r.framerate_flag=br.bits(1)!=0;
    if (r.framerate_flag) {
        const bool explicit_rate=br.bits(1)!=0; // FRAMERATEIND
        if (explicit_rate) throw std::runtime_error("Blu-ray rate unexpectedly used FRAMERATEEXP");
        r.nr=static_cast<int>(br.bits(8));
        r.dr=static_cast<int>(br.bits(4));
    }
    return r;
}

void require_rate(vc1_param_t p,int nr,int dr,const char* what) {
    const auto r=sequence_rate(p);
    if (!r.display_ext || !r.framerate_flag || r.nr!=nr || r.dr!=dr) {
        char msg[256];
        std::snprintf(msg,sizeof(msg),"%s sequence rate mismatch: DISPLAY_EXT=%d FRAMERATE_FLAG=%d NR=%d DR=%d",
                      what,r.display_ext?1:0,r.framerate_flag?1:0,r.nr,r.dr);
        throw std::runtime_error(msg);
    }
}

bool accepts(vc1_param_t p) {
    vc1_t* e=vc1_encoder_open(&p);
    if (!e) return false;
    vc1_encoder_close(e);
    return true;
}

void require_accept(vc1_param_t p,const char* what) {
    vc1_t* e=vc1_encoder_open(&p);
    if (!e) throw std::runtime_error(std::string(what)+" rejected: "+vc1_encoder_last_error(nullptr));
    vc1_encoder_close(e);
}

void require_reject(vc1_param_t p,const char* what) {
    if (accepts(p)) throw std::runtime_error(std::string(what)+" unexpectedly accepted");
}
void require_reject_reason(vc1_param_t p,const char* needle) {
    vc1_t* e=vc1_encoder_open(&p);
    if (e) { vc1_encoder_close(e); throw std::runtime_error("Blu-ray pass 2 limit unexpectedly accepted"); }
    if (std::string(vc1_encoder_last_error(nullptr)).find(needle)==std::string::npos)
        throw std::runtime_error(std::string("wrong rejection reason for Blu-ray pass 2: ")+
                                 vc1_encoder_last_error(nullptr));
}
}

int main() {
    try {
        // Generic VC-1 is no longer constrained to the Blu-ray application subset.
        auto p=base(); p.i_width=2048; p.i_height=1088;
        require_accept(p,"generic >1080 picture");
        p=base(); p.i_fps_num=30;
        require_accept(p,"generic 1080p30");
        p=base(); p.i_fps_num=60;
        require_accept(p,"generic 1080p60/AP@L4");
        p=base(); p.i_width=1919; p.i_height=1079;
        require_accept(p,"generic odd Advanced picture");
        p=base(); p.i_keyint_max=999;
        require_accept(p,"generic long keyframe interval");
        p=base(); p.i_rc_method=VC1_RC_ABR; p.i_bitrate=41000000ull; p.i_vbv_buffer_size=31000000ull;
        require_accept(p,"generic >Blu-ray bitrate/VBV");

        // Legal Blu-ray codec-subset combinations. No container state is involved.
        p=base(); p.b_bluray_compat=1;
        require_accept(p,"Blu-ray 1080p24");
        p=base(); p.b_bluray_compat=1; p.i_fps_num=24000; p.i_fps_den=1001;
        require_accept(p,"Blu-ray 1080p23.976");
        p=base(); p.b_bluray_compat=1; p.i_scan_mode=VC1_SCAN_INTERLACED_TFF; p.i_fps_num=30000; p.i_fps_den=1001;
        require_accept(p,"Blu-ray 1080i59.94 fields");
        p=base(); p.b_bluray_compat=1; p.i_width=1280; p.i_height=720; p.i_fps_num=60000; p.i_fps_den=1001;
        require_accept(p,"Blu-ray 720p59.94");
        p=base(); p.b_bluray_compat=1; p.i_width=720; p.i_height=576; p.i_scan_mode=VC1_SCAN_INTERLACED_BFF; p.i_fps_num=25;
        require_accept(p,"Blu-ray 576i50 fields");
        p=base(); p.b_bluray_compat=1; p.i_keyint_max=24;
        require_accept(p,"Blu-ray one-second keyint");

        // Hardware-oriented Blu-ray output carries explicit Advanced-profile
        // frame timing instead of relying solely on MPEG-2 transport PTS.
        p=base(); p.b_bluray_compat=1;
        require_rate(p,1,1,"Blu-ray 1080p24");
        p=base(); p.b_bluray_compat=1; p.i_fps_num=24000; p.i_fps_den=1001;
        require_rate(p,1,2,"Blu-ray 1080p23.976");
        p=base(); p.b_bluray_compat=1; p.i_scan_mode=VC1_SCAN_INTERLACED_TFF; p.i_fps_num=30000; p.i_fps_den=1001;
        require_rate(p,3,2,"Blu-ray 1080i59.94 fields");
        p=base(); p.b_bluray_compat=1; p.i_width=1280; p.i_height=720; p.i_fps_num=60000; p.i_fps_den=1001;
        require_rate(p,5,2,"Blu-ray 720p59.94");
        p=base(); p.b_bluray_compat=1; p.i_width=1280; p.i_height=720; p.i_fps_num=50;
        require_rate(p,4,1,"Blu-ray 720p50");
        p=base(); p.b_bluray_compat=1; p.i_width=720; p.i_height=576; p.i_scan_mode=VC1_SCAN_INTERLACED_BFF; p.i_fps_num=25;
        require_rate(p,2,1,"Blu-ray 576i50 fields");

        // Generic aligned Advanced Profile remains transport-timestamp driven.
        p=base();
        if (sequence_rate(p).display_ext)
            throw std::runtime_error("generic aligned Advanced stream unexpectedly gained DISPLAY_EXT");

        // Blu-ray restrictions are opt-in and cover codec parameters only.
        p=base(); p.b_bluray_compat=1; p.i_width=2048; p.i_height=1088;
        require_reject(p,"Blu-ray >1080 picture");
        p=base(); p.b_bluray_compat=1; p.i_fps_num=30;
        require_reject(p,"Blu-ray 1080p30");
        p=base(); p.b_bluray_compat=1; p.i_fps_num=60;
        require_reject(p,"Blu-ray 1080p60");
        p=base(); p.b_bluray_compat=1; p.i_width=1919; p.i_height=1079;
        require_reject(p,"Blu-ray odd dimensions");
        p=base(); p.b_bluray_compat=1; p.i_width=1280; p.i_height=720; p.i_fps_num=25;
        require_reject(p,"Blu-ray 720p25");
        p=base(); p.b_bluray_compat=1; p.i_keyint_max=25;
        require_reject(p,"Blu-ray >one-second keyint");
        p=base(); p.b_bluray_compat=1; p.i_rc_method=VC1_RC_ABR; p.i_bitrate=41000000ull; p.i_vbv_buffer_size=30000000ull;
        require_reject(p,"Blu-ray >40M bitrate");
        p.i_two_pass=2; p.psz_two_pass_stats_file="missing-two-pass-stats";
        require_reject_reason(p,"bitrate");
        p=base(); p.b_bluray_compat=1; p.i_rc_method=VC1_RC_ABR; p.i_bitrate=38000000ull; p.i_vbv_buffer_size=30000001ull;
        require_reject(p,"Blu-ray >30M VBV");
        p.i_two_pass=2; p.psz_two_pass_stats_file="missing-two-pass-stats";
        require_reject_reason(p,"VBV");
        p=base(); p.b_bluray_compat=1; p.i_profile=VC1_PROFILE_MAIN;
        require_reject(p,"Blu-ray Main Profile");

        // A valid single-picture stats fixture exercises *pass-2 opening*,
        // including the measured whole-film planner and signaled HRD rate.
        // Bitrate, peak and buffer are independently changeable between passes.
        p=base(); p.b_bluray_compat=1; p.i_rc_method=VC1_RC_ABR;
        p.i_bitrate=20000000ull; p.i_vbv_buffer_size=30000000ull;
        p.i_two_pass=2;
        const std::string stats_path="libvc1-bluray-peak-regression.stats";
        p.psz_two_pass_stats_file=stats_path.c_str();
        {
            std::ofstream f(stats_path,std::ios::binary|std::ios::trunc);
            if (!f) throw std::runtime_error("cannot create Blu-ray stats test fixture");
            f<<"LIBVC1_TWO_PASS 2 "<<vc1_twopass::signature(p)<<"\n"
             <<"F 0 0 0 I 100000 10 1000 8160 0 42 20 10\n"
             <<"END 1 100000\n";
        }
        auto check_hrd=[&](uint64_t lower_rate,uint64_t upper_rate,
                           uint64_t lower_buffer,uint64_t upper_buffer) {
            vc1_t* e=vc1_encoder_open(&p);
            if (!e) throw std::runtime_error(std::string("Blu-ray 2pass open failed: ")+vc1_encoder_last_error(nullptr));
            vc1_stats_t st{};
            const int rc=vc1_encoder_stats(e,&st);
            vc1_encoder_close(e);
            if (rc<0 || st.i_hrd_rate_bits<lower_rate || st.i_hrd_rate_bits>upper_rate ||
                st.i_hrd_buffer_bits<lower_buffer || st.i_hrd_buffer_bits>upper_buffer)
                throw std::runtime_error("Blu-ray average/peak/buffer are not independent in HRD");
        };
        // 20M average MUST NOT become the HRD transmission rate: default peak 40M.
        check_hrd(39990000ull,40000000ull,29990000ull,30000000ull);
        p.i_peak_bitrate=30000000ull;
        check_hrd(29990000ull,30000000ull,29990000ull,30000000ull);
        p.i_vbv_buffer_size=20000000ull;
        check_hrd(29990000ull,30000000ull,19990000ull,20000000ull);
        p.i_peak_bitrate=40000001ull;
        require_reject_reason(p,"peak bitrate");
        p.i_peak_bitrate=19000000ull;
        require_reject_reason(p,"exceeds peak");
        p.i_peak_bitrate=40000000ull; p.i_vbv_buffer_size=30000001ull;
        require_reject_reason(p,"VBV");
        std::remove(stats_path.c_str());

        // Existing unrestricted non-Blu-ray pass 2 must reject, not ignore,
        // an explicit peak rate. One-pass may use a separate peak.
        p=base(); p.i_rc_method=VC1_RC_ABR; p.i_bitrate=20000000ull;
        p.i_peak_bitrate=40000000ull;
        require_accept(p,"one-pass independent peak");
        p.i_two_pass=2; p.psz_two_pass_stats_file="missing-two-pass-stats";
        require_reject_reason(p,"--max-bitrate requires --bluray-compat");

        std::puts("Blu-ray compatibility, independent average/peak/buffer, and timing ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr,"Blu-ray compatibility test failed: %s\n",e.what());
        return 1;
    }
}
