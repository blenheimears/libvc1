#include <libvc1.hpp>
#include <array>
#include <iostream>
int main() {
    vc1_param_t p{}; if(vc1_param_default(&p)<0) return 1; if(LIBVC1_API_VERSION<40||p.i_scan_mode!=VC1_SCAN_PROGRESSIVE||p.b_bluray_compat!=0||p.b_rc_maximize!=0||p.f_rc_i_weight!=5.0||p.f_rc_p_weight!=1.0||p.f_rc_b_weight!=0.70||p.f_rc_residual_threshold!=8.0||p.f_rc_residual_width!=24.0||p.f_rc_residual_max_q_boost!=0.0||p.f_inter_intra_threshold!=0.80||p.i_compute_backend!=VC1_COMPUTE_CPU||p.i_compute_device!=-1||p.i_vulkan_min_batch!=8||p.b_vulkan_force!=0||p.b_intra_gop_parallelism!=0||p.b_fixed_gop_grid!=0||p.b_scene_cut!=1||p.f_scene_cut_min_interval!=0.25||p.i_motion_search_range!=1024||p.i_motion_local_search_range!=32||p.i_me_quality!=VC1_ME_SATD||p.f_distant_match_max_mae!=255.0||p.b_debug_stats!=0||p.b_debug_macroblock_stats!=0||p.b_speed_profile!=0||p.b_debug_disable_p_intra!=0||p.b_debug_disable_b_intra!=0||p.b_dquant!=1||p.i_qp_constant!=2)return 5;
    p.i_width=16;p.i_height=16;p.i_threads=1;p.i_bframes=0;p.b_intra_only=1;p.i_rc_method=VC1_RC_CQP;p.i_qp_constant=9;p.i_simd=VC1_SIMD_SCALAR;
    if(vc1_param_apply_profile(&p,"wmv9")<0) return 2;
    libvc1::Encoder e(p);
    vc1_param_t actual{}; if(vc1_encoder_parameters(e.get(),&actual)<0 || actual.i_motion_search_range!=255 || actual.i_motion_local_search_range!=32 || actual.i_me_quality!=VC1_ME_SATD) return 6;
    vc1_au_t* hdr=nullptr;int nh=0;if(vc1_encoder_headers(e.get(),&hdr,&nh)<=0||nh!=1||hdr->i_payload!=4)return 3;
    std::array<uint8_t,256> y{};std::array<uint8_t,64> u{},v{};y.fill(64);u.fill(128);v.fill(128);
    vc1_picture_t in{},out{};vc1_picture_init(&in);vc1_picture_init(&out);in.i_pts=11;in.img.i_plane=3;in.img.plane[0]=y.data();in.img.plane[1]=u.data();in.img.plane[2]=v.data();in.img.i_stride[0]=16;in.img.i_stride[1]=8;in.img.i_stride[2]=8;
    vc1_au_t* au=nullptr;int n=0;e.encode(&au,&n,&out,&in);while(vc1_encoder_delayed_frames(e.get())>0&&n==0)e.encode(&au,&n,&out,nullptr);if(n!=1||!au||out.i_pts!=11)return 4;
    std::cout<<"libvc1 C++ API ok\n";return 0;
}
