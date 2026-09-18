#include <libvc1.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    vc1_param_t p;
    if (vc1_param_default(&p) < 0) return 1;
    if (LIBVC1_API_VERSION < 40 || p.i_scan_mode != VC1_SCAN_PROGRESSIVE || p.b_bluray_compat != 0 || p.i_simd != VC1_SIMD_AUTO || p.b_simd_fma != 1 || p.b_simd_benchmark_all != 1 || VC1_SIMD_SCALAR != VC1_SIMD_NONE || p.b_rc_maximize != 0 || p.f_rc_i_weight != 5.0 || p.f_rc_p_weight != 1.0 || p.f_rc_b_weight != 0.70 || p.f_rc_residual_threshold != 8.0 || p.f_rc_residual_width != 24.0 || p.f_rc_residual_max_q_boost != 0.0 || p.f_inter_intra_threshold != 0.80 || p.i_compute_backend != VC1_COMPUTE_CPU || p.i_compute_device != -1 || p.i_vulkan_min_batch != 8 || p.b_vulkan_force != 0 || p.b_intra_gop_parallelism != 0 || p.b_fixed_gop_grid != 0 || p.b_scene_cut != 1 || p.f_scene_cut_min_interval != 0.25 || p.i_motion_search_range != 1024 || p.i_motion_local_search_range != 32 || p.i_me_quality != VC1_ME_SATD || p.f_distant_match_max_mae != 255.0 || p.b_debug_stats != 0 || p.b_debug_macroblock_stats != 0 || p.b_speed_profile != 0 || p.b_debug_disable_p_intra != 0 || p.b_debug_disable_b_intra != 0 || p.b_dquant != 1 || p.b_skip_identical_frames != 1 || p.i_qp_constant != 2 || p.i_peak_bitrate != 0 || p.i_two_pass != 0 || p.psz_two_pass_stats_file != NULL || p.b_two_pass_dynamic_weights != 1 || p.f_two_pass_dynamic_strength != 1.0) return 8;
    if (LIBVC1_VERSION_MAJOR != 0 || LIBVC1_VERSION_MINOR != 2 || LIBVC1_VERSION_PATCH != 48 ||
        strcmp(LIBVC1_VERSION_STRING,"0.2.48") != 0) return 13;
    for (int i=0;i<VC1_SIMD_PRIMITIVE_COUNT;++i) if (p.i_simd_primitive[i] != VC1_SIMD_AUTO) return 14;
    if (VC1_SIMD_PRIMITIVE_COUNT != 20 || VC1_SIMD_BENCH_TARGET_COUNT != 13 || VC1_SIMD_BENCH_TIER_COUNT != VC1_SIMD_BENCH_TARGET_COUNT || strcmp(vc1_simd_primitive_name(VC1_SIMD_PRIMITIVE_FRAME_SAD),"frame-sad") != 0) return 15;

    /* API v39 policy: neither the default CPU mode nor legacy AUTO may probe or
       select experimental Vulkan. The force bit is valid only with VULKAN. */
    vc1_param_t legacy=p; legacy.i_width=16; legacy.i_height=16; legacy.i_threads=1;
    legacy.i_bframes=0; legacy.b_intra_only=1; legacy.i_rc_method=VC1_RC_CQP; legacy.i_qp_constant=9;
    legacy.i_compute_backend=VC1_COMPUTE_AUTO;
    vc1_t *legacy_e=vc1_encoder_open(&legacy); if(!legacy_e) return 18;
    vc1_stats_t legacy_st; if(vc1_encoder_stats(legacy_e,&legacy_st)<0 || legacy_st.i_compute_selected!=VC1_COMPUTE_CPU || legacy_st.b_compute_benchmark_ran!=0) return 19;
    vc1_encoder_close(legacy_e);
    vc1_param_t bad=p; bad.i_width=16; bad.i_height=16; bad.i_threads=1; bad.i_bframes=0; bad.b_intra_only=1;
    bad.i_rc_method=VC1_RC_CQP; bad.i_qp_constant=9; bad.b_vulkan_force=1;
    vc1_t *bad_e=vc1_encoder_open(&bad); if(bad_e){vc1_encoder_close(bad_e);return 20;}

    if (vc1_param_default(&p) < 0) return 11;
    p.i_width=16; p.i_height=16; p.i_fps_num=24; p.i_fps_den=1;
    p.i_threads=1; p.i_bframes=0; p.b_intra_only=1; p.b_debug_stats=1; p.b_debug_macroblock_stats=1;
    p.f_rc_i_weight=5.0; p.f_rc_p_weight=2.0; p.f_rc_b_weight=0.5;
    p.i_rc_method=VC1_RC_CQP; p.i_qp_constant=9; p.i_compute_backend=VC1_COMPUTE_CPU; p.i_simd=VC1_SIMD_SCALAR;
    vc1_t *e=vc1_encoder_open(&p);
    if (!e) { fprintf(stderr,"open: %s\n",vc1_encoder_last_error(NULL)); return 2; }
    vc1_param_t actual; if (vc1_encoder_parameters(e,&actual)<0 || actual.i_motion_search_range!=1024 || actual.i_motion_local_search_range!=32 || actual.i_me_quality!=VC1_ME_SATD || actual.f_rc_i_weight!=5.0 || actual.f_rc_p_weight!=2.0 || actual.f_rc_b_weight!=0.5 || actual.i_compute_backend!=VC1_COMPUTE_CPU || actual.i_compute_device!=-1) return 16;
    vc1_au_t *h=NULL; int nh=0;
    if (vc1_encoder_headers(e,&h,&nh)<=0 || nh!=1 || h->i_payload<8) return 3;
    uint8_t y[256],u[64],v[64]; memset(y,96,sizeof(y)); memset(u,128,sizeof(u)); memset(v,128,sizeof(v));
    vc1_picture_t in,out; vc1_picture_init(&in); vc1_picture_init(&out);
    in.i_pts=7; in.img.i_plane=3; in.img.plane[0]=y;in.img.plane[1]=u;in.img.plane[2]=v;
    in.img.i_stride[0]=16;in.img.i_stride[1]=8;in.img.i_stride[2]=8;
    vc1_au_t *au=NULL; int nau=0;
    if (vc1_encoder_encode(e,&au,&nau,&out,&in)<0) return 4;
    while (vc1_encoder_delayed_frames(e)>0) {
        if (vc1_encoder_encode(e,&au,&nau,&out,NULL)<0) return 5;
        if (nau) break;
    }
    if (!nau || !au || au->i_type!=VC1_TYPE_I || !au->b_keyframe || out.i_pts!=7) return 6;
    if (au->i_debug_gop_index!=0 || au->i_debug_frame_in_gop!=0 || au->i_debug_gop_frames!=1 ||
        !(au->f_debug_qscale>0.0) || au->i_debug_encode_trials<1 || out.img.i_plane!=3) return 12;
    if (au->i_debug_macroblocks!=1 || !au->p_debug_macroblocks ||
        au->p_debug_macroblocks[0].i_mode!=VC1_MB_DEBUG_I || !au->p_debug_macroblocks[0].b_intra ||
        au->p_debug_macroblocks[0].i_local_bits==0 || au->p_debug_macroblocks[0].i_field_index!=-1 ||
        au->p_debug_macroblocks[0].f_mean_y!=96.0 || !(au->p_debug_macroblocks[0].f_recon_psnr_y_db>0.0)) return 17;
    vc1_stats_t st; if(vc1_encoder_stats(e,&st)<0 || st.i_i_frames!=1 || st.i_compute_selected!=VC1_COMPUTE_CPU || st.i_compute_device!=-1 || st.b_compute_benchmark_ran!=0) return 7;
    vc1_encoder_close(e);
    puts("libvc1 C API ok");
    return 0;
}
