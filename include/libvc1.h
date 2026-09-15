#ifndef LIBVC1_H
#define LIBVC1_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32) && !defined(LIBVC1_STATIC)
#  if defined(LIBVC1_BUILDING_SHARED)
#    define LIBVC1_API __declspec(dllexport)
#  else
#    define LIBVC1_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define LIBVC1_API __attribute__((visibility("default")))
#else
#  define LIBVC1_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define LIBVC1_API_VERSION 39
#define LIBVC1_VERSION_MAJOR 0
#define LIBVC1_VERSION_MINOR 2
#define LIBVC1_VERSION_PATCH 35
#define LIBVC1_VERSION_STRING "0.2.35"

typedef struct vc1_t vc1_t;

typedef enum vc1_profile_e {
    VC1_PROFILE_MAIN = 1,
    VC1_PROFILE_ADVANCED = 3
} vc1_profile_e;

/* Source/display scan mode. Interlaced modes currently use legal Advanced-Profile
 * progressive picture coding (FCM=0) while preserving interlace sequence metadata
 * and field display order. */
typedef enum vc1_scan_mode_e {
    VC1_SCAN_PROGRESSIVE = 0,
    VC1_SCAN_INTERLACED_TFF = 1,
    VC1_SCAN_INTERLACED_BFF = 2
} vc1_scan_mode_e;

typedef enum vc1_rc_method_e {
    VC1_RC_ABR = 0,
    VC1_RC_CQP = 1
} vc1_rc_method_e;

typedef enum vc1_ac_mode_e {
    VC1_AC_AUTO = 0,
    VC1_AC_VLC = 1,
    VC1_AC_ESC3 = 2
} vc1_ac_mode_e;

typedef enum vc1_quantizer_type_e {
    VC1_QUANTIZER_AUTO = 0,
    VC1_QUANTIZER_UNIFORM = 1,
    VC1_QUANTIZER_NONUNIFORM = 2
} vc1_quantizer_type_e;

typedef enum vc1_compute_backend_e {
    VC1_COMPUTE_AUTO = -1,
    VC1_COMPUTE_CPU = 0,
    VC1_COMPUTE_VULKAN = 1
} vc1_compute_backend_e;

typedef enum vc1_simd_e {
    VC1_SIMD_AUTO = -1,
    VC1_SIMD_NONE = 0,
    VC1_SIMD_SCALAR = VC1_SIMD_NONE, /* compatibility alias */
    VC1_SIMD_MIXED = 1, /* output/status value used by auto per-primitive dispatch */
    VC1_SIMD_X86_64_V1 = 2,
    VC1_SIMD_X86_64_V2 = 3,
    VC1_SIMD_X86_64_V3 = 4,
    VC1_SIMD_X86_64_V4 = 5,
    /* Stable named feature-band targets. Prescott/Conroe/Penryn/Sandy Bridge
       became implemented in API v26; K10/Bulldozer/Piledriver/AVX2-partial
       are added in API v27. Runtime matching is based only on CPU feature flags. */
    VC1_SIMD_PRESCOTT = 10,
    VC1_SIMD_CONROE = 11,
    VC1_SIMD_PENRYN = 12,
    VC1_SIMD_SANDYBRIDGE = 13,
    VC1_SIMD_K10 = 14,
    VC1_SIMD_BULLDOZER = 15,
    VC1_SIMD_PILEDRIVER = 16,
    VC1_SIMD_AVX2_PARTIAL = 17
} vc1_simd_e;

/* SIMD primitives independently benchmarked/selected by VC1_SIMD_AUTO. */
typedef enum vc1_simd_primitive_e {
    VC1_SIMD_PRIMITIVE_FRAME_SAD = 0,
    VC1_SIMD_PRIMITIVE_BLOCK_SAD,
    VC1_SIMD_PRIMITIVE_FORWARD_INTRA_8X8,
    VC1_SIMD_PRIMITIVE_FORWARD_RESIDUAL_8X8,
    VC1_SIMD_PRIMITIVE_INVERSE_8X8,
    VC1_SIMD_PRIMITIVE_PUT_BLOCK_8X8,
    VC1_SIMD_PRIMITIVE_ADD_BLOCK_RECT,
    VC1_SIMD_PRIMITIVE_LUMA_QPEL_SAD,
    VC1_SIMD_PRIMITIVE_LUMA_BILINEAR_SAD,
    VC1_SIMD_PRIMITIVE_LUMA_MC,
    VC1_SIMD_PRIMITIVE_LUMA_MC_AVG,
    VC1_SIMD_PRIMITIVE_CHROMA_MC,
    VC1_SIMD_PRIMITIVE_CHROMA_MC_AVG,
    VC1_SIMD_PRIMITIVE_INTENSITY_LUMA,
    VC1_SIMD_PRIMITIVE_INTENSITY_CHROMA,
    VC1_SIMD_PRIMITIVE_FORWARD_RESIDUAL_RECT,
    VC1_SIMD_PRIMITIVE_QUANTIZE,
    VC1_SIMD_PRIMITIVE_PERCEPTUAL_STATS,
    VC1_SIMD_PRIMITIVE_SUM_U8,
    VC1_SIMD_PRIMITIVE_SATD_4X4,
    VC1_SIMD_PRIMITIVE_COUNT
} vc1_simd_primitive_e;

/* Indices for vc1_stats_t::f_simd_primitive_units_per_second[][...]. */
typedef enum vc1_simd_benchmark_tier_e {
    VC1_SIMD_BENCH_NONE = 0,
    VC1_SIMD_BENCH_X86_64_V1 = 1,
    VC1_SIMD_BENCH_X86_64_V2 = 2,
    VC1_SIMD_BENCH_X86_64_V3 = 3,
    VC1_SIMD_BENCH_X86_64_V4 = 4,
    VC1_SIMD_BENCH_PRESCOTT = 5,
    VC1_SIMD_BENCH_CONROE = 6,
    VC1_SIMD_BENCH_PENRYN = 7,
    VC1_SIMD_BENCH_SANDYBRIDGE = 8,
    VC1_SIMD_BENCH_K10 = 9,
    VC1_SIMD_BENCH_BULLDOZER = 10,
    VC1_SIMD_BENCH_PILEDRIVER = 11,
    VC1_SIMD_BENCH_AVX2_PARTIAL = 12,
    VC1_SIMD_BENCH_TARGET_COUNT = 13,
    VC1_SIMD_BENCH_TIER_COUNT = VC1_SIMD_BENCH_TARGET_COUNT /* compatibility name */
} vc1_simd_benchmark_tier_e;

typedef enum vc1_speed_profile_operation_e {
    VC1_SPEED_GOP_ENCODE = 0,
    VC1_SPEED_SCENE_DETECTION,
    VC1_SPEED_RATE_CONTROL,
    VC1_SPEED_I_PICTURE,
    VC1_SPEED_P_PICTURE,
    VC1_SPEED_B_PICTURE,
    VC1_SPEED_P_ANALYSIS,
    VC1_SPEED_B_ANALYSIS,
    VC1_SPEED_MOTION_LOCAL_SEARCH,
    VC1_SPEED_MOTION_LONG_RANGE,
    VC1_SPEED_MOTION_SUBPEL,
    VC1_SPEED_INTENSITY_COMP,
    VC1_SPEED_MOTION_COMP,
    VC1_SPEED_TRANSFORM_RDO,
    VC1_SPEED_ADAPTIVE_QUANTIZATION,
    VC1_SPEED_DQUANT,
    VC1_SPEED_TRELLIS,
    VC1_SPEED_ENTROPY,
    VC1_SPEED_OVERLAP,
    VC1_SPEED_LOOP_FILTER,
    VC1_SPEED_MACROBLOCK_DEBUG,
    VC1_SPEED_PROFILE_OPERATION_COUNT
} vc1_speed_profile_operation_e;

typedef enum vc1_picture_type_e {
    VC1_TYPE_AUTO = 0,
    VC1_TYPE_I = 1,
    VC1_TYPE_P = 2,
    VC1_TYPE_B = 3
} vc1_picture_type_e;

/* API v30: per-macroblock analysis record for offline encoder tuning.  Motion
 * vectors are quarter-pixel units. i_local_bits counts syntax emitted while
 * writing this macroblock only; picture headers, shared bitplanes/tables, start
 * codes and byte/RBDU padding remain frame-level overhead.  Negative floating
 * values denote an unavailable comparison (for example the first input frame has
 * no previous-source comparison and intra blocks have no motion prediction). */
typedef enum vc1_mb_debug_mode_e {
    VC1_MB_DEBUG_I = 0,
    VC1_MB_DEBUG_P_INTER = 1,
    VC1_MB_DEBUG_P_4MV = 2,
    VC1_MB_DEBUG_P_INTRA = 3,
    VC1_MB_DEBUG_P_SKIPPED = 4,
    VC1_MB_DEBUG_B_FORWARD = 5,
    VC1_MB_DEBUG_B_BACKWARD = 6,
    VC1_MB_DEBUG_B_INTERPOLATED = 7,
    VC1_MB_DEBUG_B_DIRECT = 8,
    VC1_MB_DEBUG_B_INTRA = 9,
    VC1_MB_DEBUG_BI_INTRA = 10,
    VC1_MB_DEBUG_FIELD_P_FORWARD = 11,
    VC1_MB_DEBUG_FIELD_B_FORWARD = 12,
    VC1_MB_DEBUG_FIELD_B_BACKWARD = 13
} vc1_mb_debug_mode_e;

typedef enum vc1_distant_match_decision_e {
    VC1_DISTANT_MATCH_NONE = 0,
    VC1_DISTANT_MATCH_REJECTED = 1,
    VC1_DISTANT_MATCH_ABSOLUTE_GOOD = 2,
    VC1_DISTANT_MATCH_MATERIAL_IMPROVEMENT = 3
} vc1_distant_match_decision_e;

typedef struct vc1_mb_debug_t {
    uint32_t i_mb_x, i_mb_y;
    uint32_t i_visible_width, i_visible_height;
    vc1_mb_debug_mode_e i_mode;
    int i_field_index, i_field_parity; /* -1 for frame-coded MBs; otherwise coded field 0/1 and top=0/bottom=1 */
    int b_skipped, b_intra, b_four_mv, b_direct, b_acpred;
    int b_opposite_field_reference;
    int i_picture_q, i_mquant, i_dquant_delta;
    uint32_t i_cbp, i_coded_blocks;
    uint64_t i_local_bits;
    double f_amortized_frame_bits; /* local bits + equal share of frame/shared syntax; sums to frame bits */
    uint64_t i_estimated_transform_bits;
    uint32_t i_transform_parts[4]; /* 8x8, 8x4, 4x8, 4x4 coded partitions */
    int i_forward_mv_count, i_backward_mv_count;
    int64_t i_forward_reference_display_order, i_backward_reference_display_order; /* -1 when unused */
    int32_t i_forward_mv_xq[4], i_forward_mv_yq[4];
    int32_t i_backward_mv_xq[4], i_backward_mv_yq[4];
    double f_mean_y, f_stddev_y, f_mean_u, f_mean_v, f_activity_y;
    double f_previous_sad_y, f_previous_mse_y;
    double f_prediction_sad_y, f_prediction_mse_y, f_prediction_gain_db;
    double f_recon_mse_y, f_recon_mse_u, f_recon_mse_v, f_recon_mse_yuv;
    double f_recon_snr_y_db, f_recon_psnr_y_db;
    double f_recon_snr_yuv_db, f_recon_psnr_yuv_db;
    /* API v33 tuning diagnostics. Prediction MAE/ramp fields are -1 when no
       temporal prediction exists. Inter/intra cost ratio is <1 when the
       spatial proxy is cheaper; the configured selection threshold is emitted
       once in the CLI macroblock-statistics config record. */
    double f_prediction_mae_y;
    double f_residual_priority_position;
    double f_residual_priority_requested_q_boost;
    int i_residual_priority_applied_q_boost;
    double f_inter_intra_cost_ratio;
    /* API v35 long-range-search diagnostics. MAE values are -1 when no
       propagated/content distant candidate was evaluated for that direction. */
    double f_forward_distant_local_mae_y, f_forward_distant_candidate_mae_y;
    vc1_distant_match_decision_e i_forward_distant_match_decision;
    double f_backward_distant_local_mae_y, f_backward_distant_candidate_mae_y;
    vc1_distant_match_decision_e i_backward_distant_match_decision;
    double f_aq_dark_detail;
    double f_aq_color_luma_priority;
    double f_aq_color_chroma_priority;
    double f_aq_requested_q_boost;
} vc1_mb_debug_t;

/* Long-range motion-search policy. */
typedef enum vc1_long_range_search_mode_e {
    /* Default: always compare local UMH with propagated distant candidates. */
    VC1_LONG_RANGE_COMPARE = 0,
    /* Skip propagated/content long-range work when local UMH is already good. */
    VC1_LONG_RANGE_LOCAL_GOOD_SKIP = 1,
    /* Historical behavior: try propagated distant candidates first and short-circuit if good. */
    VC1_LONG_RANGE_LEGACY_DISTANT_FIRST = 2
} vc1_long_range_search_mode_e;

/* Motion-estimation decision depth. Candidate discovery always uses SAD; the
 * later levels progressively add coded-vector rate, SATD, and codec-aware RD. */
typedef enum vc1_me_quality_e {
    VC1_ME_SAD = 0,
    VC1_ME_RATE = 1,
    VC1_ME_SATD = 2,
    VC1_ME_RD = 3
} vc1_me_quality_e;

typedef struct vc1_param_t {
    int i_struct_size;
    int i_api_version;

    int i_width;
    int i_height;
    int i_fps_num;
    int i_fps_den;
    vc1_profile_e i_profile;
    /* API v28: enforce the Blu-ray Disc VC-1 codec subset. This affects codec
     * constraints only; it never selects or restricts an output container. */
    int b_bluray_compat;
    /* API v21: source/display scan mode. Interlaced modes require Advanced Profile. */
    vc1_scan_mode_e i_scan_mode;

    int i_threads;
    /* Enable parallel work inside a GOP (scene-cut scan and independent B-picture analysis). */
    int b_intra_gop_parallelism;
    int i_keyint_max;
    int i_bframes;
    int i_motion_search_range;
    /* Radius used by the ordinary local UMH stage before lazy long-range rescue. */
    int i_motion_local_search_range;
    /* API v29: staged ME decision depth. Normal/default is SATD; RD remains explicit HQ. */
    vc1_me_quality_e i_me_quality;
    /* Controls how local and propagated distant motion candidates are ordered/compared. */
    vc1_long_range_search_mode_e i_long_range_search_mode;
    /* API v35: maximum luma MAE for a merely-relative distant match. Distant
     * candidates at <=6 MAE remain unconditionally good; candidates above
     * this ceiling are rejected unless legacy distant-first mode is selected. */
    double f_distant_match_max_mae;
    double f_scene_threshold;
    int b_scene_cut;
    /* API v36: minimum elapsed source time between threshold scene-cut I pictures.
     * 0 disables the cooldown. Independent of i_keyint_max. */
    double f_scene_cut_min_interval;
    /* Keep scene I pictures on the old absolute keyint grid instead of resetting the interval. */
    int b_fixed_gop_grid;
    int b_fade_compensation;
    int b_loop_filter;
    /* VC-1 OVERLAP smoothing; Advanced low-Q pictures use CONDOVER/OVERFLAGS. */
    int b_overlap;
    int b_variable_transforms;
    /* Advanced-profile per-macroblock quantizer signaling/RDO (DQUANT). */
    int b_dquant;
    int i_trellis;
    int b_adaptive_quality;
    double f_aq_strength;
    int b_intra_only;
    /* Advanced Profile: emit normative skipped P pictures for exact duplicate input frames. */
    int b_skip_identical_frames;

    int b_ac_coding;
    vc1_ac_mode_e i_ac_mode;
    int i_ac_y_table;
    int i_ac_c_table;

    vc1_rc_method_e i_rc_method;
    uint64_t i_bitrate;
    uint64_t i_vbv_buffer_size;
    int i_qp_constant;
    /* CQP HALFQP flag. Legal only when i_qp_constant <= 8. */
    int b_qp_half;
    /* Permit ABR/VBV to choose legal HALFQP picture quantizers. */
    int b_halfqp;
    /* Per-picture PQUANTIZER policy. AUTO chooses uniform/nonuniform with sampled RDO. */
    vc1_quantizer_type_e i_quantizer_type;
    /* In ABR mode, bias quality toward fuller use of the requested bitrate when headroom is available. */
    int b_rc_maximize;
    /* API v31: relative ABR bitrate weights for intra, P-inter, and B-inter
     * macroblocks. Defaults are I=5.0, P=1.0, B=0.70. Since 0.2.17 these are
     * block-class weights: an intra block inside a P/B picture receives the same
     * relative share as a block in an I picture. The analyzed block mix is
     * normalized over the GOP, so scaling all three by the same factor does not
     * change the requested average bitrate. Ignored by CQP. */
    double f_rc_i_weight;
    double f_rc_p_weight;
    double f_rc_b_weight;
    /* API v33: smooth protection for poorly predicted temporal macroblocks.
     * Prediction error is luma mean absolute error (8-bit sample units). The
     * protection ramps from zero at threshold to full strength over width.
     * Strength is the maximum number of local MQUANT indices made finer. */
    double f_rc_residual_threshold;
    double f_rc_residual_width;
    double f_rc_residual_max_q_boost;
    /* API v33: P/B inter->intra hysteresis. 0.80 preserves the historical
     * requirement that the intra proxy be at least 20% cheaper. Larger values
     * make intra macroblocks easier to select. */
    double f_inter_intra_threshold;

    /* API v39: optional experimental compute acceleration. CPU is the default
     * and Vulkan is never activated automatically. VC1_COMPUTE_VULKAN opts in
     * to a startup benchmark and uses Vulkan only for batch sizes where it beats
     * the selected CPU/SIMD path. VC1_COMPUTE_AUTO is retained as a legacy alias
     * for the safe CPU default. */
    vc1_compute_backend_e i_compute_backend;
    /* Physical-device ordinal from vkEnumeratePhysicalDevices; -1 selects the
     * preferred compute-capable GPU automatically. */
    int i_compute_device;
    /* Small batches stay on the CPU/SIMD path to avoid dispatch overhead. */
    int i_vulkan_min_batch;
    /* Additional experimental override. When nonzero together with VULKAN,
     * bypass the startup benchmark decision and force eligible Vulkan work. */
    int b_vulkan_force;

    vc1_simd_e i_simd;
    int b_simd_fma;
    /* API v26: benchmark all CPU-compatible intermediate x86 targets in AUTO mode. */
    int b_simd_benchmark_all;
    /* API v20: per-primitive force overrides. VC1_SIMD_AUTO means inherit i_simd. */
    vc1_simd_e i_simd_primitive[VC1_SIMD_PRIMITIVE_COUNT];

    /* Output-side helpers. The codec remains container-agnostic. */
    int b_recon;
    int b_transform_info;
    /* Collect extended per-picture diagnostics in vc1_au_t. This is opt-in. */
    int b_debug_stats;
    /* API v30: collect vc1_mb_debug_t records for every coded macroblock. */
    int b_debug_macroblock_stats;
    /* API v34: collect low-overhead aggregate encoder speed profiling counters. */
    int b_speed_profile;
    /* API v22 diagnostic: suppress whole-macroblock intra decisions in P pictures only. */
    int b_debug_disable_p_intra;
    /* API v23 diagnostic: suppress intra macroblock selection in B pictures.
       This also prevents all-intra B slots from being promoted to BI pictures. */
    int b_debug_disable_b_intra;
    /* Emit Advanced Profile sequence headers. In Blu-ray compatibility mode,
       the header is repeated at every I-picture random-access entry point. */
    int b_emit_sequence_header;

    void *opaque;
} vc1_param_t;

typedef struct vc1_image_t {
    int i_plane;
    uint8_t *plane[3];
    int i_stride[3];
} vc1_image_t;

typedef struct vc1_picture_properties_t {
    const uint8_t *transform_map;
    size_t transform_map_size;
} vc1_picture_properties_t;

typedef struct vc1_picture_t {
    int i_struct_size;
    vc1_picture_type_e i_type;
    int i_qpplus1;
    int b_keyframe;
    /* API v18: this access unit is the Advanced Profile PTYPE=Skipped picture form. */
    int b_skipped_picture;
    int64_t i_pts;
    int64_t i_dts;
    uint64_t i_display_order;
    uint64_t i_coded_order;
    vc1_image_t img;
    vc1_picture_properties_t prop;
    void *opaque;
} vc1_picture_t;

/*
 * Access-unit storage is owned by the encoder. p_payload remains valid only
 * until the next vc1_encoder_encode() call (or encoder close), so muxers should
 * consume/copy it before submitting another picture.
 */
typedef struct vc1_au_t {
    const uint8_t *p_payload;
    size_t i_payload;
    vc1_picture_type_e i_type;
    int i_qp;
    /* API v12 picture quantizer diagnostics. */
    int b_halfqp;
    vc1_quantizer_type_e i_quantizer_type;
    double f_quant_step;
    int b_keyframe;
    /* API v18: this access unit uses Advanced Profile PTYPE=Skipped. */
    int b_skipped_picture;
    int64_t i_pts;
    int64_t i_dts;
    uint64_t i_display_order;
    uint64_t i_coded_order;

    /* Per-picture rate-control diagnostics (API v4). These are zero in CQP.
       Values describe the ABR controller's first decision and the final
       accepted access unit, so callers can tune the estimator. */
    double f_rc_complexity;
    double f_rc_predicted_bits;
    double f_rc_target_bits;
    double f_rc_allowed_bits;
    double f_rc_prediction_error_percent;
    double f_rc_vbv_before_bits;
    double f_rc_vbv_after_bits;
    uint64_t i_rc_first_actual_bits;
    int i_rc_predicted_q;
    int i_rc_retries;
    int b_rc_reencoded;

    /* Extended per-picture diagnostics (introduced in API v8; extended in API v10/v11). Populated when
       vc1_param_t::b_debug_stats is enabled; otherwise zero. */
    uint64_t i_debug_gop_index;
    uint64_t i_debug_frame_in_gop;
    uint64_t i_debug_gop_frames;
    int b_debug_scene_i;
    /* API v15: adaptive I inserted because no macroblock found a usable motion match. */
    int b_debug_motion_failure_i;
    double f_debug_qscale;
    double f_debug_rc_planned_bits;
    double f_debug_gop_budget_scale;
    double f_debug_gop_difficulty;
    double f_debug_motion_residual;
    double f_debug_mean_mv_pixels;
    double f_debug_max_mv_pixels;
    uint64_t i_debug_moved_macroblocks;
    uint64_t i_debug_fractional_chroma_macroblocks;
    uint64_t i_debug_skipped_macroblocks;
    uint64_t i_debug_explicit_macroblocks;
    uint64_t i_debug_coded_macroblocks;
    uint64_t i_debug_coded_blocks;
    uint64_t i_debug_four_mv_macroblocks;
    uint64_t i_debug_intra_macroblocks;
    /* Group-C DQUANT diagnostics (API v11). */
    uint64_t i_debug_dquant_macroblocks;
    int i_debug_mquant_min;
    int i_debug_mquant_max;
    double f_debug_mquant_mean;
    uint64_t i_debug_transform_parts[4];
    /* API v13 transform-picture diagnostics. i_debug_ttfrm is 0 when
       TTMBF=0, otherwise 1=8x8, 2=8x4, 3=4x8, 4=4x4. The exact-check
       flag is currently meaningful for B pictures. */
    int b_debug_ttmbf;
    int i_debug_ttfrm;
    int b_debug_ttfrm_exact_checked;
    uint64_t i_debug_acpred_macroblocks;
    uint64_t i_debug_b_forward;
    uint64_t i_debug_b_backward;
    uint64_t i_debug_b_interpolated;
    uint64_t i_debug_b_direct;
    int b_debug_intensity_comp;
    int i_debug_encode_trials;

    /* API v30 macroblock-analysis array. Owned by the encoder and valid only
       until the next vc1_encoder_encode() call, like p_payload. */
    const vc1_mb_debug_t *p_debug_macroblocks;
    size_t i_debug_macroblocks;
} vc1_au_t;

typedef struct vc1_stats_t {
    uint64_t i_frames;
    uint64_t i_i_frames;
    uint64_t i_p_frames;
    /* API v18: subset of P pictures represented with PTYPE=Skipped. */
    uint64_t i_skipped_pictures;
    uint64_t i_b_frames;
    uint64_t i_scene_i_frames;
    uint64_t i_motion_failure_i_frames;
    uint64_t i_ic_p_frames;
    uint64_t i_b_forward;
    uint64_t i_b_backward;
    uint64_t i_b_interpolated;
    uint64_t i_b_direct;
    uint64_t i_moved_macroblocks;
    uint64_t i_fractional_chroma_macroblocks;
    uint64_t i_skipped_macroblocks;
    uint64_t i_explicit_macroblocks;
    uint64_t i_coded_macroblocks;
    uint64_t i_coded_blocks;
    uint64_t i_four_mv_macroblocks;
    uint64_t i_intra_macroblocks;
    uint64_t i_dquant_macroblocks;
    uint64_t i_halfqp_frames;
    uint64_t i_uniform_quantizer_frames;
    uint64_t i_nonuniform_quantizer_frames;
    uint64_t i_transform_parts[4];
    uint64_t i_q_sum;
    int i_q_min;
    int i_q_max;
    uint64_t i_rate_total_bits;
    uint64_t i_rate_frames;
    uint64_t i_hrd_rate_bits;
    uint64_t i_hrd_buffer_bits;
    uint64_t i_hrd_init_bits;
    uint64_t i_hrd_pauses;
    uint64_t i_hrd_underflows;
    double f_hrd_min_after_bits;
    double f_hrd_max_pre_bits;
    uint64_t i_gops;
    vc1_simd_e i_simd_selected;
    double f_simd_scalar_units_per_second;
    double f_simd_v1_units_per_second;
    double f_simd_v2_units_per_second;
    double f_simd_v3_units_per_second;
    double f_simd_v4_units_per_second;
    /* API v20: final tier and measured throughput for every primitive.
       Tier slots are VC1_SIMD_BENCH_*; unavailable/incompatible tiers report 0. */
    vc1_simd_e i_simd_primitive_selected[VC1_SIMD_PRIMITIVE_COUNT];
    int b_simd_primitive_fma_selected[VC1_SIMD_PRIMITIVE_COUNT];
    double f_simd_primitive_units_per_second[VC1_SIMD_PRIMITIVE_COUNT][VC1_SIMD_BENCH_TIER_COUNT];
    /* API v34 aggregate profiling. Timers are cumulative across all worker
       threads and retries; nested feature timers intentionally overlap. */
    uint64_t i_speed_profile_calls[VC1_SPEED_PROFILE_OPERATION_COUNT];
    uint64_t i_speed_profile_nanoseconds[VC1_SPEED_PROFILE_OPERATION_COUNT];
    /* API v39 Vulkan compute diagnostics and startup benchmark results. */
    vc1_compute_backend_e i_compute_selected;
    int i_compute_device;
    char sz_compute_device[128];
    int b_compute_benchmark_ran;
    int i_compute_benchmark_device;
    char sz_compute_benchmark_device[128];
    int i_compute_benchmark_sad_min_batch;
    int i_compute_benchmark_satd_min_batch;
    double f_compute_cpu_sad_candidates_per_second;
    double f_compute_vulkan_sad_candidates_per_second;
    double f_compute_cpu_satd_candidates_per_second;
    double f_compute_vulkan_satd_candidates_per_second;
    uint64_t i_compute_dispatches;
    uint64_t i_compute_candidates;
    uint64_t i_compute_frame_uploads;
    uint64_t i_compute_frame_upload_bytes;
    uint64_t i_compute_submit_wait_nanoseconds;
} vc1_stats_t;

LIBVC1_API const char *vc1_version_str(void);
LIBVC1_API const char *vc1_simd_primitive_name(vc1_simd_primitive_e primitive);
LIBVC1_API const char *vc1_speed_profile_operation_name(vc1_speed_profile_operation_e operation);
LIBVC1_API int vc1_param_default(vc1_param_t *param);
LIBVC1_API int vc1_param_apply_profile(vc1_param_t *param, const char *profile);
LIBVC1_API void vc1_picture_init(vc1_picture_t *pic);

LIBVC1_API vc1_t *vc1_encoder_open(const vc1_param_t *param);
LIBVC1_API int vc1_encoder_headers(vc1_t *encoder, vc1_au_t **pp_au, int *pi_au);
/* Submit one picture, or pass pic_in == NULL to flush delayed GOP/B output. */
LIBVC1_API int vc1_encoder_encode(vc1_t *encoder, vc1_au_t **pp_au, int *pi_au,
                                  vc1_picture_t *pic_out, const vc1_picture_t *pic_in);
LIBVC1_API int vc1_encoder_delayed_frames(vc1_t *encoder);
LIBVC1_API int vc1_encoder_parameters(vc1_t *encoder, vc1_param_t *param);
LIBVC1_API int vc1_encoder_stats(vc1_t *encoder, vc1_stats_t *stats);
LIBVC1_API const char *vc1_encoder_last_error(vc1_t *encoder);
LIBVC1_API void vc1_encoder_close(vc1_t *encoder);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIBVC1_H */
