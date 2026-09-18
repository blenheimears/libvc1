#include "encoder_bluray_constraints.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

uint64_t resolved_peak_bitrate(const vc1_param_t& p) {
    if (p.i_peak_bitrate) return p.i_peak_bitrate;
    return p.b_bluray_compat && p.i_two_pass!=0 ? 40000000ull : p.i_bitrate;
}

static bool fps_equals(const vc1_param_t& p,int num,int den=1) {
    const int64_t a=static_cast<int64_t>(p.i_fps_num);
    const int64_t b=static_cast<int64_t>(p.i_fps_den);
    return a*static_cast<int64_t>(den)==static_cast<int64_t>(num)*b;
}

uint64_t bluray_keyint_limit(const vc1_param_t& p) {
    const long double fps=static_cast<long double>(p.i_fps_num)/p.i_fps_den;
    return static_cast<uint64_t>(std::max<long long>(1,std::llround(fps)));
}

void validate_bluray_compat(const vc1_param_t& p,int advanced_level) {
    if (!p.b_bluray_compat) return;
    if (p.i_profile!=VC1_PROFILE_ADVANCED)
        throw std::runtime_error("--bluray-compat requires VC-1 Advanced Profile");
    if (advanced_level>3)
        throw std::runtime_error("--bluray-compat requires VC-1 Advanced Profile Level 3 or lower");
    if ((p.i_width&1) || (p.i_height&1))
        throw std::runtime_error("--bluray-compat does not permit odd picture dimensions");
    if (p.i_rc_method==VC1_RC_ABR) {
        if (p.i_bitrate>40000000ull)
            throw std::runtime_error("--bluray-compat average bitrate exceeds 40 Mbit/s");
        if (resolved_peak_bitrate(p)>40000000ull)
            throw std::runtime_error("--bluray-compat peak bitrate exceeds 40 Mbit/s");
        if (p.i_rc_method==VC1_RC_ABR && p.i_bitrate>resolved_peak_bitrate(p))
            throw std::runtime_error("Blu-ray average target bitrate exceeds peak bitrate");
        if (p.i_vbv_buffer_size>30000000ull)
            throw std::runtime_error("--bluray-compat VBV exceeds 30 Mbit");
    }

    const bool progressive=p.i_scan_mode==VC1_SCAN_PROGRESSIVE;
    const bool interlaced=!progressive;
    bool legal=false;
    if ((p.i_width==1920 || p.i_width==1440) && p.i_height==1080) {
        legal = (progressive && (fps_equals(p,24000,1001) || fps_equals(p,24))) ||
                (interlaced && (fps_equals(p,30000,1001) || fps_equals(p,25)));
    } else if (p.i_width==1280 && p.i_height==720) {
        legal = progressive && (fps_equals(p,60000,1001) || fps_equals(p,50) ||
                                fps_equals(p,24000,1001) || fps_equals(p,24));
    } else if (p.i_width==720 && p.i_height==480) {
        legal = interlaced && fps_equals(p,30000,1001);
    } else if (p.i_width==720 && p.i_height==576) {
        legal = interlaced && fps_equals(p,25);
    }
    if (!legal)
        throw std::runtime_error("resolution/frame-rate/scan combination is outside the Blu-ray VC-1 video subset");

    const uint64_t max_keyint=bluray_keyint_limit(p);
    if (p.i_keyint_max>0 && static_cast<uint64_t>(p.i_keyint_max)>max_keyint)
        throw std::runtime_error("--bluray-compat keyframe interval exceeds approximately one second");
}

