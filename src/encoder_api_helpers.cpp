#include "encoder_internal.h"
#include <cctype>
#include <cstring>

extern "C" {
const char *vc1_version_str(void) { return LIBVC1_VERSION_STRING; }
const char *vc1_simd_primitive_name(vc1_simd_primitive_e primitive) {
    const int i=static_cast<int>(primitive);
    if (i<0 || i>=VC1_SIMD_PRIMITIVE_COUNT) return "unknown";
    return libvc1::simd_primitive_name(static_cast<libvc1::SimdPrimitive>(i));
}
const char *vc1_speed_profile_operation_name(vc1_speed_profile_operation_e operation) {
    static const char* names[VC1_SPEED_PROFILE_OPERATION_COUNT]={
        "GOP encode attempts", "scene-change detection", "rate control",
        "I-picture coding", "P-picture coding", "B-picture coding",
        "P-picture analysis", "B-picture analysis", "local motion search",
        "long-range motion search", "subpixel/RD motion refinement",
        "intensity/fade compensation", "motion compensation", "transform RDO",
        "adaptive/perceptual AQ",
        "DQUANT selection", "trellis quantization", "entropy coding",
        "overlap smoothing", "loop filtering", "macroblock debug metrics"
    };
    const int i=static_cast<int>(operation);
    return i>=0 && i<VC1_SPEED_PROFILE_OPERATION_COUNT?names[i]:"unknown";
}

int vc1_param_default(vc1_param_t *p) {
    if (!p) return -1;
    std::memset(p,0,sizeof(*p));
    p->i_struct_size=sizeof(*p); p->i_api_version=LIBVC1_API_VERSION;
    p->i_fps_num=24; p->i_fps_den=1; p->i_profile=VC1_PROFILE_ADVANCED; p->b_bluray_compat=0; p->i_scan_mode=VC1_SCAN_PROGRESSIVE;
    p->i_threads=static_cast<int>(std::max(1u,std::thread::hardware_concurrency())); p->b_intra_gop_parallelism=0;
    p->i_keyint_max=0; p->i_bframes=2; p->i_motion_search_range=1024; p->i_motion_local_search_range=32; p->i_me_quality=VC1_ME_SATD; p->i_long_range_search_mode=VC1_LONG_RANGE_COMPARE; p->f_distant_match_max_mae=255.0; p->f_scene_threshold=28.0;
    p->b_scene_cut=1; p->f_scene_cut_min_interval=0.25; p->b_fixed_gop_grid=0; p->b_fade_compensation=1; p->b_loop_filter=1; p->b_overlap=1; p->b_variable_transforms=1; p->b_dquant=1;
    p->i_trellis=1; p->b_adaptive_quality=1; p->f_aq_strength=1.0; p->b_intra_only=0; p->b_skip_identical_frames=1;
    p->b_ac_coding=1; p->i_ac_mode=VC1_AC_AUTO; p->i_ac_y_table=0; p->i_ac_c_table=0;
    p->i_rc_method=VC1_RC_ABR; p->i_bitrate=38000000ull; p->i_vbv_buffer_size=30000000ull;
    p->i_qp_constant=2; p->b_qp_half=0; p->b_halfqp=1; p->i_quantizer_type=VC1_QUANTIZER_AUTO; p->b_rc_maximize=0;
    p->f_rc_i_weight=5.0; p->f_rc_p_weight=1.0; p->f_rc_b_weight=0.70;
    p->f_rc_residual_threshold=8.0; p->f_rc_residual_width=24.0; p->f_rc_residual_max_q_boost=0.0; p->f_inter_intra_threshold=0.80;
    p->i_compute_backend=VC1_COMPUTE_CPU; p->i_compute_device=-1; p->i_vulkan_min_batch=8; p->b_vulkan_force=0;
    p->i_simd=VC1_SIMD_AUTO; p->b_simd_fma=1; p->b_simd_benchmark_all=1;
    for (int i=0;i<VC1_SIMD_PRIMITIVE_COUNT;++i) p->i_simd_primitive[i]=VC1_SIMD_AUTO;
    p->b_recon=0; p->b_transform_info=0; p->b_debug_stats=0; p->b_debug_macroblock_stats=0; p->b_speed_profile=0; p->b_debug_disable_p_intra=0; p->b_debug_disable_b_intra=0; p->b_emit_sequence_header=1;
    return 0;
}

int vc1_param_apply_profile(vc1_param_t *p,const char *profile) {
    if (!p || !profile) return -1;
    std::string s(profile); for (char& c:s) c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (s=="bluray") { p->i_profile=VC1_PROFILE_ADVANCED; p->b_bluray_compat=1; return 0; }
    if (s=="advanced" || s=="ap") { p->i_profile=VC1_PROFILE_ADVANCED; return 0; }
    if (s=="main" || s=="mp" || s=="wmv9" || s=="wmv3") {
        p->i_profile=VC1_PROFILE_MAIN;
        if (p->i_rc_method==VC1_RC_ABR && p->i_bitrate==38000000ull) p->i_bitrate=20000000ull;
        return 0;
    }
    return -1;
}

void vc1_picture_init(vc1_picture_t *pic) {
    if (!pic) return;
    std::memset(pic,0,sizeof(*pic));
    pic->i_struct_size=sizeof(*pic);
    pic->i_type=VC1_TYPE_AUTO;
}

} // extern C
