#include "encoder_internal.h"
#include "encoder_two_pass.h"
#include "encoder_bluray_constraints.h"
namespace libvc1 {
using MotionVector = Vc1Encoder::MotionVector;
using ProgressiveMvMode = Vc1Encoder::ProgressiveMvMode;
using IntensityComp = Vc1Encoder::IntensityComp;
using PAnalysis = Vc1Encoder::PAnalysis;
using TransformType = Vc1Encoder::TransformType;
using TransformSignalLevel = Vc1Encoder::TransformSignalLevel;
using TransformPicturePlan = Vc1Encoder::TransformPicturePlan;
using PEncodeResult = Vc1Encoder::PEncodeResult;
using BMbMode = Vc1Encoder::BMbMode;
using BAnalysis = Vc1Encoder::BAnalysis;
using TransformRateEstimate = Vc1Encoder::TransformRateEstimate;
using QuantizerRateEstimate = Vc1Encoder::QuantizerRateEstimate;
using BEncodeResult = Vc1Encoder::BEncodeResult;
using PredictorInfo = Vc1Encoder::PredictorInfo;
using BitplaneChoice = Vc1Encoder::BitplaneChoice;
using MvDataSyntax = Vc1Encoder::MvDataSyntax;
using MotionSearchBounds = Vc1Encoder::MotionSearchBounds;
using GlobalMotionResult = Vc1Encoder::GlobalMotionResult;
using IntegerMotionResult = Vc1Encoder::IntegerMotionResult;
using LongRangeSignature = Vc1Encoder::LongRangeSignature;
using LongRangeSignatureCell = Vc1Encoder::LongRangeSignatureCell;
using LongRangeIndex = Vc1Encoder::LongRangeIndex;
using VlcCode = Vc1Encoder::VlcCode;
using BlockCoding = Vc1Encoder::BlockCoding;
using TransformParentDecision = Vc1Encoder::TransformParentDecision;
using MbTransformDecision = Vc1Encoder::MbTransformDecision;
using MbQuantDecision = Vc1Encoder::MbQuantDecision;
using AcVlcEntry = Vc1Encoder::AcVlcEntry;
using AcTableView = Vc1Encoder::AcTableView;
using AcLookup = Vc1Encoder::AcLookup;
using DcCodePlan = Vc1Encoder::DcCodePlan;
Vc1Encoder::Vc1Encoder(EncoderConfig c) : c_(c) {
        if (c_.pqindex < 1 || c_.pqindex > 31)
            throw std::runtime_error("PQINDEX must be 1..31 in this encoder");
        if (c_.halfqp && c_.pqindex>8)
            throw std::runtime_error("HALFQP is legal only at PQINDEX 1..8");
        if (c_.ac_y_table_index < 0 || c_.ac_y_table_index > 2 ||
            c_.ac_c_table_index < 0 || c_.ac_c_table_index > 2)
            throw std::runtime_error("AC table index must be 0, 1, or 2");
        if (c_.motion_search_range < 0 || c_.motion_search_range > 1024)
            throw std::runtime_error("motion search range must be 0..1024 pixels");
        if (!(c_.scene_threshold >= 0.0 && c_.scene_threshold <= 255.0))
            throw std::runtime_error("scene threshold must be 0..255");
        if (c_.trellis < 0 || c_.trellis > 2)
            throw std::runtime_error("trellis level must be 0, 1, or 2");
        if (!(c_.aq_strength >= 0.0 && c_.aq_strength <= 3.0) || !std::isfinite(c_.aq_strength))
            throw std::runtime_error("adaptive-quality strength must be finite and in 0..3");
        // Field-picture loop filtering changes the reference picture after residual
        // reconstruction. Until the encoder mirrors that field-domain filter exactly,
        // signal LOOPFILTER=0 for interlaced streams so encoder and decoder references
        // remain identical across P chains.
        if (c_.interlaced()) c_.loop_filter=false;
        scale_ = dc_scale(c_.pqindex);
    }
} // namespace libvc1
#include <cstring>
#include <cctype>
#include <deque>
#include <future>
#include <mutex>
#include <numeric>
#include <optional>
#include <string_view>
#include <utility>
namespace {
enum class GopPictureKind { I, P, B };
struct GopEncodeParams {
    libvc1::EncoderConfig cfg;
    bool cq_set=false;              // unbounded constant-Q mode
    bool bounded_cq=false;          // one-pass ABR + global VBV ceiling
    bool bounded_maximize=false;     // optional aggressive full-buffer policy
    bool intra_gop_parallelism=false; // optional parallel scene/B analysis inside each GOP
    int intra_gop_workers=1;         // total threads a GOP job may consume, including its main thread
    int cq_value=2;
    int bounded_start_q=2;       // retry/recovery starting Q retained at the GOP boundary
    int bounded_min_q=1;         // recovery-only floor
    int bounded_preferred_q=2;   // fast mode normally returns toward this CQ target
    bool scene_cut=false;
    bool first_is_scene_i=false;  // scheduler-preclassified scene boundary at GOP start
    bool first_is_motion_failure_i=false; // no macroblock found a usable hybrid-search match
    bool intra_only=false;
    int bframes=2;
    uint64_t hrd_rate_bits=0;
    uint64_t target_rate_bits=0; // average ABR target, independent of peak HRD transmission
    uint64_t hrd_buffer_bits=0;
    // The libvc1 ABR controller does not impose a hard GOP ceiling.  It only
    // biases this GOP's nominal ABR target so easy GOPs can leave reservoir for
    // a short difficult GOP in the same planning window.
    double modern_budget_scale=1.0;
    std::vector<double> two_pass_picture_scale;
    // Relative intra/P-inter/B-inter ABR block weights. Normalized over the analyzed GOP block mix.
    double rc_i_weight=5.0;
    double rc_p_weight=1.0;
    double rc_b_weight=0.70;
    bool include_sequence=false;
    std::vector<uint8_t> sequence;
    bool keep_reconstruction=false;
    bool debug_stats=false;
    bool debug_macroblock_stats=false;
    // Debug-only temporal predecessor retained across GOP boundaries.
    std::shared_ptr<const libvc1::Frame> previous_source;
    double modern_difficulty=0.0;
    int two_pass_mode=0;
    uint64_t modern_nominal_gop_bits=0;
    uint64_t modern_target_gop_bits=0;
};
struct EncodedGopPicture {
    std::vector<uint8_t> au;
    libvc1::Frame reconstructed;
    std::vector<uint8_t> transform_map;
    uint64_t display_index=0;
    GopPictureKind kind=GopPictureKind::P;
    bool key=false;
    bool skipped_picture=false;
    int q=9;
    bool halfqp=false;
    libvc1::QuantizerType quantizer_type=libvc1::QuantizerType::Uniform;
    double quant_step=18.0;
    // libvc1 ABR rate-control diagnostics, copied to vc1_au_t for applications/--rc-stats.
    double rc_complexity=0.0;
    double pass1_mse_y=0.0,pass1_mse_uv=0.0;
    double two_pass_budget_scale=1.0;
    double two_pass_i_weight=0.0,two_pass_p_weight=0.0,two_pass_b_weight=0.0;
    double rc_predicted_bits=0.0;
    double rc_target_bits=0.0;
    double rc_allowed_bits=0.0;
    double rc_prediction_error_percent=0.0;
    double rc_vbv_before_bits=0.0;
    double rc_vbv_after_bits=0.0;
    uint64_t rc_first_actual_bits=0;
    int rc_predicted_q=0;
    int rc_retries=0;
    // Optional extended diagnostics for --debug-stats/API v8.
    bool debug_scene_i=false;
    bool debug_motion_failure_i=false;
    double debug_qscale=0.0;
    double debug_rc_planned_bits=0.0;
    double debug_gop_budget_scale=0.0;
    double debug_gop_difficulty=0.0;
    double debug_motion_residual=0.0;
    double debug_mean_mv_pixels=0.0;
    double debug_max_mv_pixels=0.0;
    uint64_t debug_moved_macroblocks=0;
    uint64_t debug_fractional_chroma_macroblocks=0;
    uint64_t debug_skipped_macroblocks=0;
    uint64_t debug_explicit_macroblocks=0;
    uint64_t debug_coded_macroblocks=0;
    uint64_t debug_coded_blocks=0;
    uint64_t debug_four_mv_macroblocks=0;
    uint64_t debug_intra_macroblocks=0;
    uint64_t debug_dquant_macroblocks=0;
    int debug_mquant_min=0;
    int debug_mquant_max=0;
    double debug_mquant_mean=0.0;
    std::array<uint64_t,4> debug_transform_parts{};
    bool debug_ttmbf=false;
    int debug_ttfrm=0; // 0=none; 1=8x8, 2=8x4, 3=4x8, 4=4x4
    bool debug_ttfrm_exact_checked=false;
    uint64_t debug_acpred_macroblocks=0;
    uint64_t debug_b_forward=0,debug_b_backward=0,debug_b_interpolated=0,debug_b_direct=0;
    bool debug_intensity_comp=false;
    int debug_encode_trials=0;
    std::vector<vc1_mb_debug_t> macroblock_debug;
};
struct EncodedGop {
    uint64_t index=0;
    uint64_t start_frame=0;
    std::vector<EncodedGopPicture> pictures; // coded order
    // ABR workers return the source GOP with their first-pass result so an
    // actually overflowing GOP can be retried after earlier GOPs establish the
    // real global VBV state. This is a move, not a second source-frame copy.
    std::vector<libvc1::Frame> retry_source;
    std::shared_ptr<const libvc1::Frame> previous_source;
    bool bounded_probe=false;
    int bounded_probe_q=0;       // Q used by the speculative one-pass GOP
    int bounded_end_q=0;         // Q state to carry into the next encoded unit
    uint64_t i_frames=0, p_frames=0, b_frames=0, skipped_pictures=0, scene_i_frames=0, motion_failure_i_frames=0;
    uint64_t ic_p_frames=0;
    uint64_t b_forward=0, b_backward=0, b_interpolated=0, b_direct=0;
    uint64_t moved_macroblocks=0, fractional_chroma_macroblocks=0;
    uint64_t skipped_macroblocks=0, explicit_macroblocks=0;
    uint64_t coded_macroblocks=0, coded_blocks=0;
    uint64_t four_mv_macroblocks=0, intra_macroblocks=0, dquant_macroblocks=0;
    uint64_t halfqp_frames=0, uniform_quantizer_frames=0, nonuniform_quantizer_frames=0;
    std::array<uint64_t,4> transform_parts{};
    uint64_t q_sum=0;
    int q_min_used=32, q_max_used=0;
    uint64_t rate_total_bits=0, rate_frames=0;
    uint64_t hrd_init_bits=0, hrd_pauses=0, hrd_underflows=0;
    double hrd_min_after_bits=0.0, hrd_max_pre_bits=0.0;
};
class BoundedCqExceeded : public std::runtime_error {
public:
    BoundedCqExceeded()
        : std::runtime_error("predictive rate control exhausted the available VBV allowance") {}
};
static double fast_intra_activity(const libvc1::Frame& f,const libvc1::EncoderConfig& cfg) {
    if (f.width<2 || f.height<2 || f.y.empty()) return 0.0;
    // This runs only for I-picture RC analysis (normally once per GOP).  The
    // old 2x2 high-resolution subsampling could alias away fine periodic detail
    // completely because it sampled one edge phase only; grass/block textures
    // could therefore report near-zero complexity.  Scan every adjacent edge.
    const int ss=1;
    uint64_t h=0,v=0;
#if defined(LIBVC1_HAVE_X86_64_V4)
    if (cfg.simd_for(libvc1::SimdPrimitive::BlockSad)==libvc1::SimdTier::X86V4) {
        h=libvc1::simd::block_sad_x86_64_v4(f.y.data(),f.width,f.y.data()+1,f.width,
                                             f.width-1,f.height,ss);
        v=libvc1::simd::block_sad_x86_64_v4(f.y.data(),f.width,f.y.data()+f.width,f.width,
                                             f.width,f.height-1,ss);
    } else
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
    if (cfg.simd_for(libvc1::SimdPrimitive::BlockSad)==libvc1::SimdTier::X86V1) {
        h=libvc1::simd::block_sad_x86_64_v1(f.y.data(),f.width,f.y.data()+1,f.width,
                                             f.width-1,f.height,ss);
        v=libvc1::simd::block_sad_x86_64_v1(f.y.data(),f.width,f.y.data()+f.width,f.width,
                                             f.width,f.height-1,ss);
    } else
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
    if (cfg.simd_for(libvc1::SimdPrimitive::BlockSad)==libvc1::SimdTier::X86V2) {
        h=libvc1::simd::block_sad_x86_64_v2(f.y.data(),f.width,f.y.data()+1,f.width,
                                             f.width-1,f.height,ss);
        v=libvc1::simd::block_sad_x86_64_v2(f.y.data(),f.width,f.y.data()+f.width,f.width,
                                             f.width,f.height-1,ss);
    } else
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
    if (cfg.simd_for(libvc1::SimdPrimitive::BlockSad)==libvc1::SimdTier::Avx2Partial) {
        h=libvc1::simd::block_sad_avx2_partial(f.y.data(),f.width,f.y.data()+1,f.width,
                                             f.width-1,f.height,ss);
        v=libvc1::simd::block_sad_avx2_partial(f.y.data(),f.width,f.y.data()+f.width,f.width,
                                             f.width,f.height-1,ss);
    } else
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
    if (cfg.simd_for(libvc1::SimdPrimitive::BlockSad)==libvc1::SimdTier::X86V3) {
        h=libvc1::simd::block_sad_x86_64_v3(f.y.data(),f.width,f.y.data()+1,f.width,
                                             f.width-1,f.height,ss);
        v=libvc1::simd::block_sad_x86_64_v3(f.y.data(),f.width,f.y.data()+f.width,f.width,
                                             f.width,f.height-1,ss);
    } else
#endif
    {
        for (int y=0;y<f.height;y+=ss)
            for (int x=0;x<f.width-1;x+=ss)
                h+=static_cast<uint64_t>(std::abs(static_cast<int>(f.y[static_cast<size_t>(y)*f.width+x])-static_cast<int>(f.y[static_cast<size_t>(y)*f.width+x+1])));
        for (int y=0;y<f.height-1;y+=ss)
            for (int x=0;x<f.width;x+=ss)
                v+=static_cast<uint64_t>(std::abs(static_cast<int>(f.y[static_cast<size_t>(y)*f.width+x])-static_cast<int>(f.y[static_cast<size_t>(y+1)*f.width+x])));
        if (ss>1) { h*=static_cast<uint64_t>(ss*ss); v*=static_cast<uint64_t>(ss*ss); }
    }
    const double hn=static_cast<double>(h)/static_cast<double>(static_cast<uint64_t>(f.width-1)*f.height);
    const double vn=static_cast<double>(v)/static_cast<double>(static_cast<uint64_t>(f.width)*(f.height-1));
    return 0.5*(hn+vn);
}
static double sparse_texture_activity(const libvc1::Frame& f) {
    if (f.width<2 || f.height<2 || f.y.empty()) return 0.0;
    // Cheap source-domain texture hint used only to decide whether AQ should
    // retain one extra local quantizer step after a very successful UMH match.
    // Compare adjacent samples at a sparse ~64x36 grid so two-pixel detail is
    // not aliased away by comparing samples several pixels apart.
    const int sx=std::max(1,f.width/64);
    const int sy=std::max(1,f.height/36);
    uint64_t sum=0,n=0;
    for (int y=0;y<f.height-1;y+=sy) {
        for (int x=0;x<f.width-1;x+=sx) {
            const size_t o=static_cast<size_t>(y)*f.width+x;
            const int v=f.y[o];
            sum+=static_cast<uint64_t>(std::abs(v-static_cast<int>(f.y[o+1])));
            sum+=static_cast<uint64_t>(std::abs(v-static_cast<int>(f.y[o+f.width])));
            n+=2;
        }
    }
    return n?static_cast<double>(sum)/static_cast<double>(n):0.0;
}
struct AbrRateControl {
    static constexpr double kQExponent=0.55;
    static constexpr double kComplexityExponent=1.15;
    static double q2_bits(GopPictureKind k,double complexity,const libvc1::EncoderConfig& cfg) {
        const double pixels=static_cast<double>(static_cast<uint64_t>(cfg.width)*cfg.height);
        const double mbs=static_cast<double>(static_cast<uint64_t>((cfg.width+15)/16)*((cfg.height+15)/16));
        const double c=std::pow(std::max(0.0,complexity),kComplexityExponent);
        if (k==GopPictureKind::I) return std::max(64.0,10.5*mbs + 0.84*pixels*c);
        if (k==GopPictureKind::P) return std::max(64.0, 1.2*mbs + 0.41*pixels*c);
        return std::max(64.0,2.2*mbs + 1.01*pixels*c);
    }
    // One-pass ABR state uses exponentially blurred complexity, a running rate
    // factor, overflow correction, per-picture-type size predictors and VBV
    // clipping.
    struct SizePredictor {
        double coeff_min=0.25;
        double coeff=1.0;
        double count=1.0;
        double decay=0.5;
        double offset=0.0;
        double predict(double qscale,double var) const {
            return std::max(32.0,(coeff*var+offset)/(std::max(1e-6,qscale)*count));
        }
        void update(double qscale,double var,double bits) {
            if (!(var>=10.0) || !(bits>0.0)) return;
            const double old_coeff=coeff/count;
            const double old_offset=offset/count;
            double new_coeff=std::max((bits*qscale-old_offset)/var,coeff_min);
            new_coeff=std::clamp(new_coeff,old_coeff/1.5,old_coeff*1.5);
            double new_offset=bits*qscale-new_coeff*var;
            if (new_offset<0.0) new_offset=0.0;
            count*=decay; coeff*=decay; offset*=decay;
            count+=1.0; coeff+=new_coeff; offset+=new_offset;
        }
    };
    struct Decision {
        int q=2;
        bool halfqp=false;
        double predicted_bits=0.0;
        double rceq=1.0;
        double var=1.0;
        double qscale=1.0;
        double target_bits=0.0;
        double planned_bits=0.0;
        double block_rate_multiplier=1.0;
        int b_reference_q=0;
    };
    struct QuantChoice { int q=2; bool halfqp=false; double qscale=1.0; };
    std::array<SizePredictor,3> pred{};
    std::array<double,3> last_qscale{{1.0,1.0,1.0}};
    std::array<bool,3> have_last{{false,false,false}};
    // Keep B quality consistent with both reference anchors.
    double previous_non_b_qscale=1.0;
    double last_non_b_qscale=1.0;
    GopPictureKind previous_non_b_kind=GopPictureKind::I;
    GopPictureKind last_non_b_kind=GopPictureKind::I;
    int non_b_qscales=0;
    double post_i_credit_bits=0.0;
    static constexpr double kIpFactor=1.40;
    static constexpr double kPbFactor=1.30;
    double qcompress=0.60;
    double rate_tolerance=1.0;
    double short_term_cplxsum=0.0;
    double short_term_cplxcount=0.0;
    double cplxr_sum=0.0;
    double wanted_bits_window=0.0;
    double actual_bits_sum=0.0;
    double planned_bits_sum=0.0;
    double gop_debt_planned_bits_sum=0.0;
    double target_per_frame=1.0;
    double allocation_scale=1.0;
    double bitrate=1.0;
    double buffer_size=1.0;
    double cbr_decay=1.0;
    uint64_t frames_done=0;
    uint64_t gop_frames_total=0;
    uint64_t gop_i_frames=0, gop_p_frames=0, gop_b_frames=0;
    double gop_weight_total_estimate=1.0;
    double i_weight=5.0, p_weight=1.0, b_weight=0.70;
    bool initialized=false;
    AbrRateControl(uint64_t rate_bits,uint64_t buffer_bits,libvc1::Rational fps) {
        bitrate=std::max(1.0,static_cast<double>(rate_bits));
        buffer_size=std::max(1.0,static_cast<double>(buffer_bits));
        target_per_frame=bitrate*static_cast<double>(fps.den)/std::max(1.0,static_cast<double>(fps.num));
        // Local-window ABR+VBV behavior when target
        // bitrate and VBV maxrate are equal: keep roughly a buffer-duration
        // amount of history instead of allowing ancient frames to dominate.
        const double frac=std::clamp(target_per_frame/buffer_size,0.0,1.0);
        cbr_decay=std::clamp(1.0-0.25*frac,0.90,1.0);
        pred[0].coeff_min=0.06; pred[0].coeff=0.19;
        pred[1].coeff_min=0.06; pred[1].coeff=0.23;
        pred[2].coeff_min=0.06; pred[2].coeff=0.23;
    }
    void set_gop_plan(uint64_t n,uint64_t ni,uint64_t np,uint64_t nb,double scale=1.0,
                      double iw=5.0,double pw=1.0,double bw=0.70) {
        gop_frames_total=n; gop_i_frames=ni; gop_p_frames=np; gop_b_frames=nb;
        i_weight=iw; p_weight=pw; b_weight=bw;
        const double weighted=i_weight*static_cast<double>(ni)+p_weight*static_cast<double>(np)+
                              b_weight*static_cast<double>(nb);
        gop_weight_total_estimate=weighted>0.0?weighted:static_cast<double>(std::max<uint64_t>(1,n));
        allocation_scale=scale; // Pass-2 whole-film planning, not a GOP peak-rate cap.
    }
    double effective_target_per_frame() const { return target_per_frame*allocation_scale; }
    static size_t type_index(GopPictureKind k) {
        return k==GopPictureKind::I?0u:(k==GopPictureKind::P?1u:2u);
    }
    double nominal_picture_weight(GopPictureKind k) const {
        return k==GopPictureKind::I?i_weight:(k==GopPictureKind::P?p_weight:b_weight);
    }
    void announce_picture_block_weight(GopPictureKind k,double actual_average_weight) {
        if (!(actual_average_weight>0.0)) actual_average_weight=nominal_picture_weight(k);
        const double old_norm=gop_weight_norm();
        gop_weight_total_estimate += actual_average_weight-nominal_picture_weight(k);
        gop_weight_total_estimate=std::max(1e-9,gop_weight_total_estimate);
        const double new_norm=gop_weight_norm();
        // Re-express already-earned nominal shares under the improved GOP
        // denominator. This makes the final sum of planned picture budgets
        // exactly equal to the GOP target once every picture's block mix is
        // known, while actual encoded bits remain untouched and appear as
        // ordinary ABR debt/credit against that corrected plan.
        const double rescale=old_norm>0.0?new_norm/old_norm:1.0;
        planned_bits_sum*=rescale;
        gop_debt_planned_bits_sum*=rescale;
        post_i_credit_bits*=rescale;
    }
    double gop_weight_norm() const {
        return static_cast<double>(std::max<uint64_t>(1,gop_frames_total))/std::max(1e-9,gop_weight_total_estimate);
    }
    double planned_picture_bits_for_weight(double picture_weight) const {
        return effective_target_per_frame()*std::max(1e-9,picture_weight)*gop_weight_norm();
    }
    double planned_picture_bits(GopPictureKind k) const {
        return planned_picture_bits_for_weight(nominal_picture_weight(k));
    }
    static double q_to_scale(double q) {
        // The VC-1 PQ-to-size response uses an empirically measured exponent.  0.55 comes from the
        // encoder's measured coded-size-vs-PQ curve gathered for 0.1.33.
        // 0.1.72 represents legal HALFQP values directly as q+0.5 so ABR's
        // predictor and overflow accounting see the actual reconstruction step.
        return std::pow(std::max(0.5,q/2.0),kQExponent);
    }
    static int scale_to_q(double s,int min_q=1) {
        const double raw=2.0*std::pow(std::max(1e-9,s),1.0/kQExponent);
        return std::clamp(static_cast<int>(std::llround(raw)),std::clamp(min_q,1,31),31);
    }
    static QuantChoice scale_to_quant(double s,int min_q,bool allow_halfqp) {
        const int minqi=std::clamp(min_q,1,31);
        QuantChoice best{minqi,false,q_to_scale(static_cast<double>(minqi))};
        double best_error=std::abs(std::log(std::max(1e-12,best.qscale)/std::max(1e-12,s)));
        for (int q=minqi;q<=31;++q) {
            const double is=q_to_scale(static_cast<double>(q));
            const double ie=std::abs(std::log(std::max(1e-12,is)/std::max(1e-12,s)));
            if (ie<best_error) { best={q,false,is}; best_error=ie; }
            if (allow_halfqp && q<=8) {
                const double hs=q_to_scale(static_cast<double>(q)+0.5);
                const double he=std::abs(std::log(std::max(1e-12,hs)/std::max(1e-12,s)));
                if (he<best_error) { best={q,true,hs}; best_error=he; }
            }
        }
        return best;
    }
    static double complexity_var(GopPictureKind k,double complexity,const libvc1::EncoderConfig& cfg) {
        // Use our cheap SIMD/post-motion metric, but express it as the estimated
        // Q2 bit cost.  This gives the linear predictor a resolution/type-aware
        // residual-complexity proxy without an extra transform pass.
        return q2_bits(k,complexity,cfg);
    }
    static double complexity_from_var(GopPictureKind k,double var,const libvc1::EncoderConfig& cfg) {
        const double pixels=static_cast<double>(static_cast<uint64_t>(cfg.width)*cfg.height);
        const double mbs=static_cast<double>(static_cast<uint64_t>((cfg.width+15)/16)*((cfg.height+15)/16));
        if (!(pixels>0.0)) return 0.0;
        const double base=(k==GopPictureKind::I?10.5:(k==GopPictureKind::P?1.2:2.2))*mbs;
        const double slope=(k==GopPictureKind::I?0.84:(k==GopPictureKind::P?0.41:1.01))*pixels;
        const double powered=std::max(0.0,(std::max(64.0,var)-base)/std::max(1e-9,slope));
        return std::pow(powered,1.0/kComplexityExponent);
    }
    double picture_intra_fraction(GopPictureKind k,double picture_weight) const {
        if (k==GopPictureKind::I) return 1.0;
        const double temporal=k==GopPictureKind::P?p_weight:b_weight;
        const double denom=i_weight-temporal;
        if (std::abs(denom)<1e-12) return 0.0;
        return std::clamp((picture_weight-temporal)/denom,0.0,1.0);
    }
    static double qvalue_from_scale(double qscale) {
        return 2.0*std::pow(std::max(1e-9,qscale),1.0/kQExponent);
    }
    double block_rate_multiplier_for_q(GopPictureKind k,double picture_weight,double qvalue) const {
        if (!(picture_weight>0.0)) picture_weight=nominal_picture_weight(k);
        const double intra_fraction=picture_intra_fraction(k,picture_weight);
        if (k==GopPictureKind::I || intra_fraction<=0.0) return 1.0;
        const double temporal_weight=k==GopPictureKind::P?p_weight:b_weight;
        const double base_qscale=q_to_scale(qvalue);
        auto class_multiplier=[&](double class_weight) {
            if (!(class_weight>picture_weight)) return 1.0;
            const double ratio=std::clamp(picture_weight/class_weight,0.05,1.0);
            const double desired=qvalue*std::pow(ratio,1.0/kQExponent);
            const int local_q=std::clamp(static_cast<int>(std::lround(desired)),1,31);
            return base_qscale/q_to_scale(static_cast<double>(local_q));
        };
        const double intra_mult=class_multiplier(i_weight);
        const double temporal_mult=class_multiplier(temporal_weight);
        return std::max(1e-9,intra_fraction*intra_mult+(1.0-intra_fraction)*temporal_mult);
    }
    double reference_bits_adjusted_complexity(GopPictureKind k,double temporal_complexity,
                                              uint64_t reference_bits,int analysis_q,bool analysis_halfqp,
                                              const libvc1::EncoderConfig& cfg,double picture_weight=0.0) const {
        if (!reference_bits) return temporal_complexity;
        const size_t ti=type_index(k);
        const double qvalue=static_cast<double>(analysis_q)+(analysis_halfqp?0.5:0.0);
        const double qscale=q_to_scale(qvalue);
        const double block_mult=block_rate_multiplier_for_q(k,picture_weight,qvalue);
        // reference_bits is an actual dry-run entropy result at the analysis
        // quantizer, including the selected P/B-intra DC/AC/CBP/MVDATA syntax.
        // Unlike the old intra-SAD proxy, this may legitimately move complexity
        // either up or down: intra can be more expensive than motion residuals,
        // but on a local cut it can also be much cheaper than every temporal mode.
        const double measured_bits=std::max(32.0,static_cast<double>(reference_bits)/block_mult);
        const auto& sp=pred[ti];
        const double needed_var=std::max(10.0,
            (measured_bits*qscale*sp.count-sp.offset)/std::max(1e-9,sp.coeff));
        return complexity_from_var(k,needed_var,cfg);
    }
    Decision choose(GopPictureKind k,double complexity,const libvc1::EncoderConfig& cfg,
                    libvc1::RateController* vbv,bool maximize,int min_q=1,double coding_law_scale=1.0,
                    double picture_block_weight=0.0,double picture_scale=1.0,bool offline=false) {
        Decision d;
        const double picture_weight=picture_block_weight>0.0?picture_block_weight:nominal_picture_weight(k);
        const double picture_plan=planned_picture_bits_for_weight(picture_weight)*picture_scale;
        const size_t ti=type_index(k);
        d.var=complexity_var(k,complexity,cfg)*std::clamp(coding_law_scale,0.85,1.15);
        auto predict_bits=[&](double picture_qscale) {
            const double qvalue=qvalue_from_scale(picture_qscale);
            return pred[ti].predict(picture_qscale,d.var)*
                block_rate_multiplier_for_q(k,picture_weight,qvalue);
        };
        short_term_cplxsum*=0.5;
        short_term_cplxcount*=0.5;
        short_term_cplxsum+=d.var;
        short_term_cplxcount+=1.0;
        const double blurred=short_term_cplxsum/std::max(1e-9,short_term_cplxcount);
        d.rceq=std::pow(std::max(1.0,blurred),1.0-qcompress);
        if (offline) {
            // Use the offline allocation without one-pass debt or Q smoothing.
            double qscale=q_to_scale(1);
            for (int iteration=0;iteration<5;++iteration) {
                const double estimate=predict_bits(qscale);
                qscale=std::clamp(qscale*estimate/std::max(64.0,picture_plan),
                                  q_to_scale(min_q),q_to_scale(31));
            }
            if (vbv) {
                // Only the explicitly requested Blu-ray mode supplies VBV
                // in pass 2. Final actual-byte validation still follows.
                const double allowed=vbv->max_picture_bits();
                if (predict_bits(qscale)>allowed)
                    qscale=std::clamp(qscale*predict_bits(qscale)/std::max(32.0,allowed),
                                      q_to_scale(min_q),q_to_scale(31));
            }
            const QuantChoice quant=scale_to_quant(qscale,min_q,cfg.allow_halfqp);
            d.q=quant.q;d.halfqp=quant.halfqp;d.qscale=quant.qscale;
            d.block_rate_multiplier=block_rate_multiplier_for_q(
                k,picture_weight,static_cast<double>(d.q)+(d.halfqp?0.5:0.0));
            d.predicted_bits=predict_bits(d.qscale);
            d.target_bits=effective_target_per_frame();
            d.planned_bits=picture_plan;
            return d;
        }
        if (!initialized) {
            wanted_bits_window=effective_target_per_frame();
            // A GOP-local worker cannot inherit a sequential cross-GOP rate factor,
            // but seeding every refresh I around PQ2 made hard GOPs restart at
            // PQ1 after the I bonus.  Instead solve the first qscale from the
            // I picture's predicted coded size and a type-weighted share of the
            // whole GOP budget.  The ordinary I/P factor is applied below, so
            // this remains a quality bonus rather than a fixed-bit allocation.
            const double seed_target=std::max(64.0,picture_plan);
            const double q1_prediction=predict_bits(1.0);
            const double seed_qscale=std::clamp(q1_prediction/seed_target,q_to_scale(1),q_to_scale(31));
            const double seed_rate_factor=d.rceq/std::max(1e-9,seed_qscale);
            cplxr_sum=wanted_bits_window/std::max(1e-9,seed_rate_factor);
            initialized=true;
        }
        const double rate_factor=wanted_bits_window/std::max(1e-9,cplxr_sum);
        double qscale=d.rceq/std::max(1e-9,rate_factor);
        // VC-1's present B predictor depends more heavily on corrective
        // residual after mode selection. Scale the fixed P/B
        // penalty back toward unity when this picture's *current* measured
        // post-prediction residual is expensive relative to its planned share.
        // Easy B pictures retain the legacy 1.30 penalty.
        double b_quality_factor=kPbFactor;
        double b_debt_strength=1.35;
        if (k==GopPictureKind::B) {
            const double plan=std::max(64.0,picture_plan);
            const double difficulty=d.var/plan;
            const double hard=std::clamp((difficulty-1.0)/3.0,0.0,1.0);
            b_quality_factor=kPbFactor-0.30*hard;
            b_debt_strength=1.35-0.80*hard;
        }
        // Reference-picture quality priorities: I
        // pictures receive the default 1.4 quality factor and disposable B
        // pictures the default 1.3 coarsening factor.  The older 1.20/1.20
        // approximation was too weak for VC-1: under VBV pressure an anchor
        // could reach PQ31 while later disposable B pictures recovered to a
        // finer Q, producing a whole-GOP reference-quality pulse.
        if (k==GopPictureKind::I) qscale/=kIpFactor;
        else if (k==GopPictureKind::B) qscale*=b_quality_factor;
        // Long-term overflow compensation: bias quality according to how far
        // actual output has drifted from target.  With ABR+VBV this is gentle;
        // the local rate-factor decay and VBV clipping do most of the work.
        if (frames_done>0) {
            const double time_done=static_cast<double>(frames_done)*target_per_frame/bitrate;
            // Measure overflow against the type-weighted GOP plan that has
            // actually elapsed, not a flat per-frame budget. A refresh I is
            // intentionally allowed a larger share; charging that legitimate
            // share as immediate debt caused transient post-I P/B Q spikes.
            const double wanted=planned_bits_sum;
            const double abr_buffer=2.0*rate_tolerance*bitrate*std::max(1.0,std::sqrt(std::max(0.0,time_done)));
            const double overflow=std::clamp(1.0+(actual_bits_sum-wanted)/std::max(1.0,abr_buffer),0.5,2.0);
            qscale*=overflow;
            // Independent GOP workers cannot share a fully sequential ABR rate factor
            // without serializing the encoder. Close each GOP's nominal
            // budget progressively instead. An expensive refresh I therefore
            // creates a small bit debt that is repaid over the remaining
            // pictures, rather than accumulating across GOPs until the global
            // VBV forces an entire later GOP to PQ31. Disposable B pictures
            // absorb most of the repayment; P anchors are protected. This is
            // arithmetic-only and adds no trial encode or analysis pass.
            if (gop_frames_total>frames_done) {
                const double debt=actual_bits_sum-gop_debt_planned_bits_sum;
                const double remaining_nominal=std::max(0.0,
                    effective_target_per_frame()*static_cast<double>(gop_frames_total)-planned_bits_sum);
                if (debt>0.0 && remaining_nominal>0.0) {
                    const double pressure=std::clamp(1.0+1.20*debt/remaining_nominal,1.0,2.25);
                    const double strength=k==GopPictureKind::B?b_debt_strength:(k==GopPictureKind::P?0.90:0.55);
                    qscale*=std::pow(pressure,strength);
                } else if (maximize && debt<0.0 && remaining_nominal>0.0) {
                    const double relief=std::clamp(1.0+0.35*debt/remaining_nominal,0.80,1.0);
                    qscale*=relief;
                }
            }
        }
        // Avoid violent quality jumps of the same picture type.  Asymmetric
        // relaxation lets overshoot correction react faster than undershoot.
        if (have_last[ti]) {
            const double step=1.60;
            qscale=std::clamp(qscale,last_qscale[ti]/step,last_qscale[ti]*step);
        }
        // Predict actual coded size and clip against the authoritative VBV when
        // available (retry/serialized path).  This is the frame-level analogue
        // of VBV qscale clipping; a true miss may still take one retry.
        double predicted=predict_bits(qscale);
        // Maximum-utilization mode deliberately spends more of the nominal
        // per-picture allowance even in speculative GOP workers. Keep this
        // independent of global VBV timing so thread count cannot change the
        // decision; ordered VBV validation will still reject a true overflow.
        if (maximize && predicted<effective_target_per_frame()*0.92) {
            qscale*=std::clamp(predicted/std::max(32.0,effective_target_per_frame()*0.92),0.65,1.0);
            predicted=predict_bits(qscale);
        }
        if (vbv) {
            const double allowed=std::max(32.0,vbv->max_picture_bits());
            if (predicted>allowed) {
                qscale*=predicted/allowed;
                predicted=predict_bits(qscale);
            }
            const double fullness=vbv->fullness();
            const double ratio=std::clamp(fullness/buffer_size,0.0,1.0);
            if (k!=GopPictureKind::B && ratio<0.50) {
                qscale/=std::clamp(2.0*ratio,0.50,1.0);
                predicted=predict_bits(qscale);
            } else if (maximize && ratio>0.80 && predicted<effective_target_per_frame()*0.75) {
                qscale*=std::clamp(predicted/std::max(32.0,effective_target_per_frame()*0.75),0.70,1.0);
                predicted=predict_bits(qscale);
            }
        }
        // If the preceding I picture used less than its intentional
        // type-weighted share, carry that unused budget into the first P
        // reference instead of letting the flat running rate-factor window
        // suppress it. This is a one-picture credit, not a permanent rate
        // factor bias, so ordinary GOP allocation later in the GOP is unchanged.
        if (k==GopPictureKind::P && non_b_qscales>0 &&
            last_non_b_kind==GopPictureKind::I && post_i_credit_bits>0.0) {
            double repair_target=picture_plan+post_i_credit_bits;
            if (vbv) repair_target=std::min(repair_target,std::max(32.0,vbv->max_picture_bits())*0.95);
            const double useful_target=std::max(64.0,repair_target*0.90);
            if (predicted<useful_target) {
                qscale*=std::clamp(predicted/useful_target,0.65,1.0);
                predicted=predict_bits(qscale);
            }
        }

        // A disposable B picture must not receive a finer qscale than the
        // reference anchors surrounding it.  Because VC-1 coded order emits
        // the future P anchor before its intervening B pictures, both anchor
        // qscales are already known here.  Apply this after all reservoir/
        // maximize adjustments so a filling VBV cannot spend recovered bits on
        // B pictures while the P/I references remain coarse.
        if (k==GopPictureKind::B && non_b_qscales>0) {
            double reference_qscale=last_non_b_qscale;
            if (non_b_qscales>1)
                reference_qscale=std::sqrt(previous_non_b_qscale*last_non_b_qscale);
            const double normal_b_qscale=reference_qscale*b_quality_factor;
            qscale=std::max(qscale,normal_b_qscale);

            // Do not let a transient GOP-start rate-factor impulse make a B
            // picture arbitrarily coarser than both references when the size
            // predictor says the normal P/B relationship already fits this
            // picture's planned share. This is deliberately predictor-only: it
            // avoids another entropy/transform pass and leaves genuinely hard B
            // pictures free to choose a coarser Q.
            const double normal_b_bits=predict_bits(normal_b_qscale);
            double b_affordable=picture_plan*1.05;
            if (vbv) b_affordable=std::min(b_affordable,std::max(32.0,vbv->max_picture_bits())*0.97);
            if (non_b_qscales>=2 && previous_non_b_kind==GopPictureKind::I &&
                last_non_b_kind==GopPictureKind::P && normal_b_bits<=b_affordable)
                qscale=normal_b_qscale;

            // Repair only pathological B/reference quality gaps.  Keep the
            // normal 1.30 reference priority and ordinary B decisions intact;
            // when a B picture is more than ten PQ steps coarser than the
            // surrounding anchors, spend at most one six-PQ recovery step,
            // and only if the already-trained size predictor says it fits the
            // picture's planned GOP share and current VBV allowance.
            const int reference_q=scale_to_q(reference_qscale,min_q);
            d.b_reference_q=reference_q;
            const int candidate_q=scale_to_q(qscale,min_q);
            if (candidate_q>reference_q+10) {
                const int repaired_q=std::max(reference_q+10,candidate_q-6);
                const double repaired_qscale=q_to_scale(repaired_q);
                const double repaired_bits=predict_bits(repaired_qscale);
                double repair_budget=picture_plan;
                if (vbv) repair_budget=std::min(repair_budget,std::max(32.0,vbv->max_picture_bits())*0.97);
                if (repaired_bits<=repair_budget)
                    qscale=repaired_qscale;
            }

            // Current-picture residual difficulty must retain a meaningful
            // influence even when the blurred ABR/debt state is still reacting
            // to earlier pictures.  If a difficult B picture is predicted to
            // receive less than 55% of its type-weighted share, pull it toward
            // that floor.  The correction is capped at eight PQ steps and may
            // approach, but never become finer than, the surrounding anchors.
            // This is predictor arithmetic only: no trial encode is introduced.
            const double b_plan=picture_plan;
            const double b_floor=b_plan*0.55;
            const double b_bits=predict_bits(qscale);
            if (d.var>=b_plan*1.25 && b_bits<b_floor) {
                double spend_target=b_floor;
                if (vbv) spend_target=std::min(spend_target,std::max(32.0,vbv->max_picture_bits())*0.90);
                if (spend_target>b_bits) {
                    const double desired_qscale=qscale*(b_bits/spend_target);
                    const int current_q=scale_to_q(qscale,min_q);
                    const int desired_q=scale_to_q(desired_qscale,min_q);
                    const int bounded_q=std::max(reference_q,std::max(current_q-8,desired_q));
                    qscale=q_to_scale(bounded_q);
                }
            }
        }

        const QuantChoice quant=scale_to_quant(qscale,min_q,cfg.allow_halfqp);
        d.q=quant.q;
        d.halfqp=quant.halfqp;
        d.qscale=quant.qscale;
        d.block_rate_multiplier=block_rate_multiplier_for_q(
            k,picture_weight,static_cast<double>(d.q)+(d.halfqp?0.5:0.0));
        d.predicted_bits=predict_bits(d.qscale);
        d.target_bits=effective_target_per_frame();
        d.planned_bits=picture_plan;
        return d;
    }
    void observe(GopPictureKind k,const Decision& d,uint64_t bits,double aq_debt_credit_bits=0.0) {
        const size_t ti=type_index(k);
        const double model_bits=static_cast<double>(bits)/std::max(1e-9,d.block_rate_multiplier);
        pred[ti].update(d.qscale,d.var,model_bits);
        cplxr_sum += static_cast<double>(bits)*d.qscale/std::max(1e-9,d.rceq);
        cplxr_sum *= cbr_decay;
        wanted_bits_window += effective_target_per_frame();
        wanted_bits_window *= cbr_decay;
        actual_bits_sum += static_cast<double>(bits);
        const double this_plan=d.planned_bits;
        planned_bits_sum += this_plan;
        // Credit only the sampled AQ-specific syntax/residual premium, never
        // more than 12% of this picture's nominal share. This affects only the
        // intra-GOP debt pressure; predictor learning, long-term overflow and
        // authoritative VBV all continue to consume the true coded size.
        const double aq_credit=std::clamp(aq_debt_credit_bits,0.0,this_plan*0.40);
        gop_debt_planned_bits_sum += this_plan+aq_credit;
        if (k==GopPictureKind::I)
            post_i_credit_bits=std::max(0.0,this_plan-static_cast<double>(bits));
        ++frames_done;
        last_qscale[ti]=d.qscale;
        have_last[ti]=true;
        if (k!=GopPictureKind::B) {
            if (non_b_qscales>0) {
                previous_non_b_qscale=last_non_b_qscale;
                previous_non_b_kind=last_non_b_kind;
            }
            last_non_b_qscale=d.qscale;
            last_non_b_kind=k;
            ++non_b_qscales;
            if (k==GopPictureKind::P) post_i_credit_bits=0.0;
        }
    }
    void observe_skipped_p(uint64_t bits,double inherited_qscale) {
        // PTYPE=Skipped is a real timed P/reference picture, but its few syntax
        // bits say nothing about the cost of an ordinary inter-coded P frame.
        // Advance ABR/GOP accounting without poisoning the P size predictor or
        // the source-complexity rate-factor model.
        cplxr_sum *= cbr_decay;
        wanted_bits_window += effective_target_per_frame();
        wanted_bits_window *= cbr_decay;
        actual_bits_sum += static_cast<double>(bits);
        const double this_plan=planned_picture_bits(GopPictureKind::P);
        planned_bits_sum += this_plan;
        gop_debt_planned_bits_sum += this_plan;
        ++frames_done;
        const size_t ti=type_index(GopPictureKind::P);
        last_qscale[ti]=inherited_qscale;
        have_last[ti]=true;
        if (non_b_qscales>0) {
            previous_non_b_qscale=last_non_b_qscale;
            previous_non_b_kind=last_non_b_kind;
        }
        last_non_b_qscale=inherited_qscale;
        last_non_b_kind=GopPictureKind::P;
        ++non_b_qscales;
        post_i_credit_bits=0.0;
    }
};

static std::vector<uint8_t> detect_scene_flags(const std::vector<libvc1::Frame>& frames,
                                               const GopEncodeParams& p) {
    libvc1::SpeedProfileScope speed_scope(p.cfg.speed_profiler,libvc1::SpeedProfileOp::SceneDetection);
    const size_t n=frames.size();
    std::vector<uint8_t> flags(n,0);
    if (n<2 || !p.scene_cut || p.intra_only) return flags;
    const int workers=p.intra_gop_parallelism
        ? std::max(1,std::min<int>(p.intra_gop_workers,static_cast<int>(n-1))) : 1;
    auto scan_chunk=[&](size_t begin,size_t end) {
        libvc1::Vc1Encoder local_enc(p.cfg);
        for (size_t i=begin;i<end;++i)
            if (local_enc.scene_change_score(frames[i],frames[i-1]) >= p.cfg.scene_threshold)
                flags[i]=1;
    };
    if (workers<=1 || n<4) {
        scan_chunk(1,n);
    } else {
        std::vector<std::future<void>> jobs;
        jobs.reserve(static_cast<size_t>(workers-1));
        const size_t pairs=n-1;
        for (int w=1;w<workers;++w) {
            const size_t begin=1+(pairs*static_cast<size_t>(w))/static_cast<size_t>(workers);
            const size_t end=1+(pairs*static_cast<size_t>(w+1))/static_cast<size_t>(workers);
            jobs.emplace_back(std::async(std::launch::async,scan_chunk,begin,end));
        }
        scan_chunk(1,1+pairs/static_cast<size_t>(workers));
        for (auto& job:jobs) job.get();
    }
    return flags;
}

static std::vector<uint8_t> detect_motion_failure_flags(const std::vector<libvc1::Frame>& frames,
                                                        const GopEncodeParams& p) {
    const size_t n=frames.size();
    std::vector<uint8_t> flags(n,0);
    if (n<2 || p.intra_only) return flags;
    // The progressive no-match safety net is not valid on woven interlaced input:
    // combed temporal neighbors can look like a complete motion-search failure and
    // force nearly every slot to I. Field P/B pictures carry their own residuals,
    // so leave the fixed keyframe cadence in control until a field-domain safety
    // detector is implemented.
    if (p.cfg.interlaced()) return flags;
    // Deliberately short-circuit per pair. Normal temporal neighbors usually
    // find a usable block immediately; only a genuine no-match boundary pays
    // for searching every macroblock with the hybrid local/long-range search.
    libvc1::Vc1Encoder enc(p.cfg);
    for (size_t i=1;i<n;++i)
        if (!enc.has_good_motion_match(frames[i],frames[i-1])) flags[i]=1;
    return flags;
}

static bool frames_byte_identical(const libvc1::Frame& a,const libvc1::Frame& b) {
    return a.width==b.width && a.height==b.height && a.y==b.y && a.u==b.u && a.v==b.v;
}

static EncodedGop encode_gop_once(uint64_t index, uint64_t start_frame,
                             const std::vector<libvc1::Frame>& frames,
                             GopEncodeParams p,
                             libvc1::RateController* external_rate_control=nullptr,
                             int /*bounded_retry_attempt*/=0) {
    libvc1::SpeedProfileScope speed_scope(p.cfg.speed_profiler,libvc1::SpeedProfileOp::GopEncode);
    if (frames.empty()) throw std::runtime_error("internal empty GOP job");
    const size_t n=frames.size();
    EncodedGop out;
    out.index=index;
    out.start_frame=start_frame;
    out.previous_source=p.previous_source;
    out.pictures.reserve(n);

    libvc1::RateController* rate_control=external_rate_control;
    const uint64_t rc_bits_before=rate_control?rate_control->total_bits():0;
    const uint64_t rc_frames_before=rate_control?rate_control->frames():0;
    const uint64_t rc_pauses_before=rate_control?rate_control->transmission_pauses():0;
    const uint64_t rc_underflows_before=rate_control?rate_control->underflows():0;
    double local_hrd_min_after=std::numeric_limits<double>::infinity();
    double local_hrd_max_pre=0.0;
    if (rate_control && rc_frames_before==0) out.hrd_init_bits=rate_control->initial_fullness_bits_floor();
    // Determine scene boundaries before constructing B groups. When the
    // default reset-on-scene scheduler is active, the API layer has already
    // split GOP jobs at detected cuts and disables p.scene_cut for those jobs.
    // The fixed-grid compatibility mode retains the historical in-worker scan.
    std::vector<bool> is_i(n,false);
    is_i[0]=true;
    if (p.intra_only) {
        std::fill(is_i.begin(),is_i.end(),true);
    } else if (p.scene_cut) {
        const auto scene_flags=detect_scene_flags(frames,p);
        for (size_t i=1;i<n;++i) if (scene_flags[i]) is_i[i]=true;
    }

    std::vector<GopPictureKind> kind(n,GopPictureKind::P);
    std::vector<size_t> past_anchor(n,0), future_anchor(n,0);
    std::vector<uint8_t> skipped_picture(n,0);
    std::vector<uint8_t> duplicate_source(n,0);
    std::vector<size_t> coding_order;
    coding_order.reserve(n);

    // Advanced Profile has a normative whole-picture PTYPE=Skipped form. It
    // reconstructs as the previous reference picture and carries no motion,
    // transform, residual or quantizer syntax. Detect only exact source-plane
    // duplicates: near-identical frames must remain ordinary RDO decisions.
    if (!p.intra_only && p.cfg.syntax==libvc1::StreamSyntax::Advanced && p.cfg.skip_identical_frames) {
        for (size_t i=1;i<n;++i)
            if (!is_i[i] && frames_byte_identical(frames[i],frames[i-1])) duplicate_source[i]=1;
    }

    // Build independent display-order segments beginning at each I picture.
    // Duplicate runs impose mandatory adjacent reference anchors: the first
    // copy is encoded normally and each following exact duplicate may then be
    // represented by PTYPE=Skipped. Between mandatory anchors, retain the
    // normal (bframes+1) reference spacing and B-picture coding order.
    for (size_t seg=0; seg<n; ) {
        size_t end=seg+1;
        while (end<n && !is_i[end]) ++end;
        kind[seg]=GopPictureKind::I;
        coding_order.push_back(seg);
        if (!p.intra_only) {
            std::vector<uint8_t> mandatory(end-seg,0);
            mandatory[0]=1;
            for (size_t i=seg+1;i<end;++i) if (duplicate_source[i]) {
                mandatory[i-seg]=1;
                mandatory[i-1-seg]=1;
            }
            size_t prev=seg;
            while (prev+1<end) {
                const size_t max_next=std::min(prev+static_cast<size_t>(p.bframes+1),end-1);
                size_t next=max_next;
                for (size_t j=prev+1;j<=max_next;++j) {
                    if (mandatory[j-seg]) { next=j; break; }
                }
                kind[next]=GopPictureKind::P;
                past_anchor[next]=prev;
                if (duplicate_source[next] && prev+1==next) skipped_picture[next]=1;
                coding_order.push_back(next);
                for (size_t j=prev+1;j<next;++j) {
                    kind[j]=GopPictureKind::B;
                    past_anchor[j]=prev;
                    future_anchor[j]=next;
                    coding_order.push_back(j);
                }
                prev=next;
            }
        } else {
            for (size_t j=seg+1;j<end;++j) {
                kind[j]=GopPictureKind::I;
                coding_order.push_back(j);
            }
        }
        seg=end;
    }
    if (coding_order.size()!=n) throw std::runtime_error("internal GOP picture-plan mismatch");

    // Simple/Main Profile has implicit RND state: I/BI sets it to 1, each P
    // toggles it, and ordinary B pictures inherit it.  BI selection is now a
    // coding decision made after B analysis, so resolve this state incrementally
    // in coded order rather than precomputing under the assumption that every B
    // slot remains a temporal B picture.
    std::vector<bool> picture_rnd(n,false);
    bool main_rnd_state=false;

    uint64_t plan_i=0,plan_p=0,plan_b=0;
    for (auto k:kind) {
        if (k==GopPictureKind::I) ++plan_i;
        else if (k==GopPictureKind::P) ++plan_p;
        else ++plan_b;
    }

    AbrRateControl abrrc(p.target_rate_bits,p.hrd_buffer_bits,p.cfg.fps);
    abrrc.set_gop_plan(n,plan_i,plan_p,plan_b,p.modern_budget_scale,
                        p.rc_i_weight,p.rc_p_weight,p.rc_b_weight);

    std::vector<libvc1::Frame> reconstructed(n);
    std::vector<libvc1::Frame> padded_reconstructed(n);
    std::vector<std::vector<libvc1::Vc1Encoder::MotionVector>> anchor_motion(n);
    std::vector<std::vector<uint8_t>> anchor_4mv(n);
    std::vector<libvc1::Vc1Encoder::IntensityComp> anchor_intensity(n);
    // 0.1.72: keep the resolved quantizer law of reconstructed anchors for
    // dependent B analysis and as the seed law for the provisional ABR P pass.
    std::vector<int> anchor_q(n,p.cfg.pqindex);
    std::vector<uint8_t> anchor_halfqp(n,p.cfg.halfqp?1u:0u);
    std::vector<libvc1::QuantizerType> anchor_quantizer(n,
        p.cfg.quantizer_type==libvc1::QuantizerType::NonUniform
            ? libvc1::QuantizerType::NonUniform : libvc1::QuantizerType::Uniform);
    int previous_q=p.cfg.pqindex;
    int bounded_q_state=std::max(std::clamp(p.bounded_start_q,1,31),
                                 std::clamp(p.bounded_min_q,1,31));

    // B pictures between the same two anchors are mutually independent during
    // motion/prediction analysis.  Launch those analyses as soon as the future
    // P anchor has been reconstructed.  The coded-picture loop still consumes
    // results in exact VC-1 coding order, so bitstream/rate-control decisions
    // remain deterministic.
    std::vector<std::future<libvc1::Vc1Encoder::BAnalysis>> b_analysis_jobs(n);

    for (size_t coded=0; coded<coding_order.size(); ++coded) {
        const size_t local=coding_order[coded];
        const libvc1::Frame& f=frames[local];
        const libvc1::Frame* previous_source=local?&frames[local-1]:p.previous_source.get();
        const GopPictureKind pk=kind[local];
        const bool skip_pic=skipped_picture[local]!=0;
        if (p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main) {
            if (pk==GopPictureKind::I) main_rnd_state=true;
            else if (pk==GopPictureKind::P) main_rnd_state=!main_rnd_state;
            picture_rnd[local]=main_rnd_state;
        }
        const bool force_i=pk==GopPictureKind::I;
        const bool scene_i=force_i && !p.intra_only && (local!=0 || (local==0 && p.first_is_scene_i));
        const bool motion_failure_i=force_i && !p.intra_only && local==0 && p.first_is_motion_failure_i && !p.first_is_scene_i;

        libvc1::Vc1Encoder::PAnalysis panalysis;
        libvc1::Vc1Encoder::BAnalysis banalysis;
        std::optional<libvc1::Vc1Encoder::PAnalysis> shared_p_qpel_seed;
        libvc1::EncoderConfig analysis_cfg=p.cfg;
        analysis_cfg.rndctrl=picture_rnd[local];
        if (!p.cq_set && pk==GopPictureKind::P && !skip_pic) {
            // 0.1.72 ABR parity is a two-stage decision.  Motion-mode RDO must
            // be scored at the quantizer of the picture being coded, not at
            // the reconstructed reference anchor's quantizer.  Obtain a cheap
            // provisional Q from the historical Advanced Mixed-MV/Main qpel
            // analysis on a copy of the controller, then run the complete
            // progressive mode RDO at that provisional Q.  The authoritative
            // controller is still advanced only once below, using complexity
            // from the winning full-mode analysis.
            libvc1::EncoderConfig seed_cfg=p.cfg;
            seed_cfg.rndctrl=picture_rnd[local];
            const size_t pref=past_anchor[local];
            seed_cfg.pqindex=anchor_q[pref];
            seed_cfg.halfqp=anchor_halfqp[pref]!=0;
            seed_cfg.quantizer_type=anchor_quantizer[pref];
            libvc1::Vc1Encoder seed_enc(seed_cfg);
            // Build the expensive qpel/global/long-range search once and retain
            // it for the real progressive-mode analysis below.  The historical
            // provisional-Q complexity used Mixed-MV (Advanced) or qpel+intra
            // (Main); derive that result from the common qpel seed instead of
            // running a second full integer search.
            shared_p_qpel_seed.emplace(seed_enc.analyze_p_picture_core(
                f,reconstructed[pref],libvc1::Vc1Encoder::ProgressiveMvMode::OneMvQpel,false,nullptr));
            const auto seed_mode=p.cfg.syntax==libvc1::StreamSyntax::Advanced
                ? libvc1::Vc1Encoder::ProgressiveMvMode::MixedMv : libvc1::Vc1Encoder::ProgressiveMvMode::OneMvQpel;
            auto seed=seed_enc.analyze_p_picture_core(f,reconstructed[pref],seed_mode,true,&*shared_p_qpel_seed);
            // Full Mixed-MV used min(local,global).  A seeded follow-up has no
            // separate global result, but the qpel seed already carries the same
            // global floor and Mixed-MV cannot have a worse local winner.
            seed.mean_abs_residual=std::min(seed.mean_abs_residual,shared_p_qpel_seed->mean_abs_residual);
            seed.rate_complexity=std::min(seed.rate_complexity,shared_p_qpel_seed->rate_complexity);
            auto probe_rc=abrrc;
            const auto probe=probe_rc.choose(pk,seed.rate_complexity,p.cfg,rate_control,
                                              p.bounded_maximize,p.bounded_min_q);
            analysis_cfg.pqindex=probe.q;
            analysis_cfg.halfqp=probe.halfqp;
            if (analysis_cfg.quantizer_type==libvc1::QuantizerType::Auto) {
                libvc1::Vc1Encoder qsel(analysis_cfg);
                analysis_cfg.quantizer_type=qsel.choose_picture_quantizer_type(f);
                if (analysis_cfg.halfqp && analysis_cfg.quantizer_type==libvc1::QuantizerType::NonUniform)
                    analysis_cfg.quantizer_type=libvc1::QuantizerType::Uniform;
            }
        } else if (!p.cq_set && pk==GopPictureKind::B) {
            // B candidates can be pre-analysed as soon as the future anchor is
            // reconstructed.  Use that anchor's resolved coding law for now;
            // the qpel/halfpel comparison itself remains AQ-neutral.
            const size_t qref=future_anchor[local];
            analysis_cfg.pqindex=anchor_q[qref];
            analysis_cfg.halfqp=anchor_halfqp[qref]!=0;
            analysis_cfg.quantizer_type=anchor_quantizer[qref];
        }
        libvc1::Vc1Encoder picture_analysis_enc(analysis_cfg);
        if (pk==GopPictureKind::P && !skip_pic) {
            panalysis=picture_analysis_enc.analyze_p_picture(
                f,reconstructed[past_anchor[local]],shared_p_qpel_seed?&*shared_p_qpel_seed:nullptr);
            anchor_intensity[local]=panalysis.intensity;
        } else if (pk==GopPictureKind::B) {
            if (b_analysis_jobs[local].valid()) {
                banalysis=b_analysis_jobs[local].get();
            } else {
                const int den=static_cast<int>(future_anchor[local]-past_anchor[local]);
                const int num=static_cast<int>(local-past_anchor[local]);
                banalysis=picture_analysis_enc.analyze_b_picture(f,reconstructed[past_anchor[local]],
                                                                 reconstructed[future_anchor[local]],
                                                                 anchor_motion[future_anchor[local]],
                                                                 anchor_4mv[future_anchor[local]],
                                                                 anchor_intensity[future_anchor[local]],num,den);
            }
        }

        const size_t picture_mbs=static_cast<size_t>((p.cfg.width+15)/16)*
                                 static_cast<size_t>((p.cfg.height+15)/16);
        size_t picture_intra_mbs=0;
        if (pk==GopPictureKind::I) picture_intra_mbs=picture_mbs;
        else if (!skip_pic && pk==GopPictureKind::P) picture_intra_mbs=std::min(picture_mbs,panalysis.intra_macroblocks);
        else if (pk==GopPictureKind::B) picture_intra_mbs=std::min(picture_mbs,banalysis.intra_macroblocks);
        const double temporal_block_weight=pk==GopPictureKind::I?p.rc_i_weight:
            (pk==GopPictureKind::P?p.rc_p_weight:p.rc_b_weight);
        const double picture_block_weight=picture_mbs
            ? (p.rc_i_weight*static_cast<double>(picture_intra_mbs)+
               temporal_block_weight*static_cast<double>(picture_mbs-picture_intra_mbs))/
              static_cast<double>(picture_mbs)
            : temporal_block_weight;
        // Announce the analyzed block mix exactly once. Unknown future pictures
        // remain at their nominal temporal-class weight until their analysis is
        // available, so later plans repay any extra I-block share automatically.
        const double measured_weight=p.two_pass_picture_scale.empty()?1.0:
            p.two_pass_picture_scale.at(local);
        abrrc.announce_picture_block_weight(pk,picture_block_weight*measured_weight);

        const bool aq_predictable_texture_picture = p.cfg.adaptive_quality && !skip_pic &&
            pk!=GopPictureKind::I && sparse_texture_activity(f)>=90.0;
        const bool aq_contextual_integer_picture = p.cfg.adaptive_quality && !skip_pic && pk!=GopPictureKind::I &&
            picture_analysis_enc.strong_contextual_aq_picture(f);

        struct EncodedChoice {
            int q=9;
            std::vector<uint8_t> au;
            libvc1::Frame reconstructed;
            libvc1::Frame padded_reconstructed;
            libvc1::Vc1Encoder::IEncodeStats istats;
            libvc1::Vc1Encoder::PEncodeResult pstats;
            libvc1::Vc1Encoder::BEncodeResult bstats;
            bool halfqp=false;
            libvc1::QuantizerType quantizer_type=libvc1::QuantizerType::Uniform;
            double quant_step=0.0;
        };
        std::map<int,std::unique_ptr<EncodedChoice>> cache;
        const uint8_t entry_fullness=rate_control ? rate_control->fullness_code() : 127;
        auto evaluate=[&](int q,bool requested_halfqp)->EncodedChoice& {
            const bool resolved_halfqp=p.cq_set ? p.cfg.halfqp
                : (requested_halfqp && p.cfg.allow_halfqp && q<=8);
            const int cache_key=skip_pic ? -1 : q*2+(resolved_halfqp?1:0);
            auto found=cache.find(cache_key);
            if (found!=cache.end()) return *found->second;
            if (skip_pic) {
                const size_t pref=past_anchor[local];
                libvc1::EncoderConfig scfg=p.cfg;
                scfg.pqindex=anchor_q[pref];
                scfg.halfqp=anchor_halfqp[pref]!=0;
                scfg.quantizer_type=anchor_quantizer[pref];
                libvc1::Vc1Encoder senc(scfg);
                auto choice=std::make_unique<EncodedChoice>();
                choice->q=anchor_q[pref];
                choice->halfqp=anchor_halfqp[pref]!=0;
                choice->quantizer_type=anchor_quantizer[pref];
                choice->quant_step=static_cast<double>(2*choice->q+(choice->halfqp?1:0));
                choice->au=senc.encode_skipped_picture();
                // PTYPE=Skipped reconstructs exactly as the previous reference.
                // Do not clone two full frames into the trial cache; ownership is
                // transferred from the previous anchor when this sole choice is committed.
                auto [it,ok]=cache.emplace(cache_key,std::move(choice));
                (void)ok;
                return *it->second;
            }
            libvc1::EncoderConfig qcfg=p.cfg;
            qcfg.pqindex=q;
            qcfg.aq_predictable_texture=aq_predictable_texture_picture;
            qcfg.rc_block_weighting=!p.cq_set && qcfg.dquant;
            qcfg.rc_picture_block_weight=picture_block_weight;
            qcfg.rc_intra_block_weight=p.rc_i_weight;
            qcfg.rc_inter_block_weight=temporal_block_weight;
            qcfg.rc_prediction_residual_mean=pk==GopPictureKind::P?panalysis.mean_abs_residual:
                (pk==GopPictureKind::B?banalysis.mean_abs_residual:0.0);
            // 0.1.72: TTFRM/TTBLK, HALFQP, and uniform/nonuniform PQUANT all
            // use the same final coding law in CQP and ABR/VBV.
            qcfg.extended_transform_signaling=true;
            qcfg.halfqp=resolved_halfqp;
            qcfg.rndctrl=picture_rnd[local];
            if (qcfg.quantizer_type==libvc1::QuantizerType::Auto) {
                libvc1::Vc1Encoder selector(qcfg);
                qcfg.quantizer_type=aq_contextual_integer_picture ? libvc1::QuantizerType::Uniform
                    : selector.choose_picture_quantizer_type(f);
                // Keep AUTO HALFQP/non-uniform PQUANT independent for rate-model stability:
                // combining HALFQP and non-uniform PQUANT makes the one-pass
                // predictor substantially less stable before the tuning pass.
                // Forced nonuniform remains legal below PQUANT 8, but AUTO uses
                // uniform on a selected half step and may choose nonuniform on
                // the neighboring integer steps.
                if (qcfg.halfqp && qcfg.quantizer_type==libvc1::QuantizerType::NonUniform)
                    qcfg.quantizer_type=libvc1::QuantizerType::Uniform;
            }
            // SMPTE 421M permits non-uniform HALFQP only through PQUANT 7.
            if (qcfg.quantizer_type==libvc1::QuantizerType::NonUniform && qcfg.halfqp && q>=8)
                qcfg.halfqp=false;
            libvc1::Vc1Encoder qenc(qcfg);
            auto choice=std::make_unique<EncodedChoice>();
            choice->q=q;
            choice->halfqp=qcfg.halfqp;
            choice->quantizer_type=qcfg.quantizer_type;
            choice->quant_step=static_cast<double>(2*q+(qcfg.halfqp?1:0));
            if (pk==GopPictureKind::I) {
                const auto ep=qenc.entry_point(entry_fullness);
                const auto pic=qenc.encode_i_picture(f,&choice->istats,false,previous_source);
                // Blu-ray access points repeat SEQUENCE+ENTRYPOINT; generic Advanced keeps one SEQUENCE.
                const bool add_sequence=p.include_sequence && (p.cfg.bluray_compat || local==0);
                choice->au.reserve((add_sequence?p.sequence.size():0)+ep.size()+pic.size());
                if (add_sequence) choice->au.insert(choice->au.end(),p.sequence.begin(),p.sequence.end());
                choice->au.insert(choice->au.end(),ep.begin(),ep.end());
                choice->au.insert(choice->au.end(),pic.begin(),pic.end());
                auto padded=qenc.reconstruct_i_picture_padded(f);
                choice->reconstructed=qenc.crop_reconstructed_picture(padded);
                if (p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main)
                    choice->padded_reconstructed=std::move(padded);
            } else if (pk==GopPictureKind::P) {
                const libvc1::Frame* pref=p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main
                    ? &padded_reconstructed[past_anchor[local]] : nullptr;
                const size_t pref_index=past_anchor[local];
                const bool pref_field_picture=p.cfg.syntax==libvc1::StreamSyntax::Advanced && p.cfg.interlaced() &&
                    kind[pref_index]==GopPictureKind::P && !skipped_picture[pref_index];
                choice->pstats=qenc.encode_p_picture(f,reconstructed[pref_index],panalysis,pref,pref_field_picture,previous_source);
                choice->au=std::move(choice->pstats.data);
                choice->reconstructed=std::move(choice->pstats.reconstructed);
                if (p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main)
                    choice->padded_reconstructed=std::move(choice->pstats.padded_reconstructed);
                else
                    choice->pstats.padded_reconstructed=libvc1::Frame{};
            } else {
                if (!p.cfg.interlaced() && picture_mbs && banalysis.intra_macroblocks==picture_mbs) {
                    // When every B macroblock independently rejects temporal
                    // prediction, use the normative BI picture type. BI carries
                    // the I/BI intra syntax but remains a disposable B-slot in
                    // the GOP/rate-control model and is never used as an anchor.
                    libvc1::Vc1Encoder::IEncodeStats bi_stats;
                    choice->bstats.data=qenc.encode_i_picture(f,&bi_stats,true,previous_source);
                    choice->bstats.padded_reconstructed=qenc.reconstruct_i_picture_padded(f);
                    choice->bstats.reconstructed=qenc.crop_reconstructed_picture(choice->bstats.padded_reconstructed);
                    choice->bstats.intra_macroblocks=picture_mbs;
                    choice->bstats.acpred_macroblocks=bi_stats.acpred_macroblocks;
                    choice->bstats.dquant_macroblocks=bi_stats.dquant_macroblocks;
                    choice->bstats.mquant_sum=bi_stats.mquant_sum;
                    choice->bstats.mquant_samples=bi_stats.mquant_samples;
                    choice->bstats.mquant_min=bi_stats.mquant_min;
                    choice->bstats.mquant_max=bi_stats.mquant_max;
                    choice->bstats.explicit_macroblocks=picture_mbs;
                    choice->bstats.bi_picture=true;
                    choice->bstats.macroblock_debug=std::move(bi_stats.macroblock_debug);
                } else {
                    const int den=static_cast<int>(future_anchor[local]-past_anchor[local]);
                    const int num=static_cast<int>(local-past_anchor[local]);
                    const libvc1::Frame* ppast=p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main
                        ? &padded_reconstructed[past_anchor[local]] : nullptr;
                    const libvc1::Frame* pfuture=p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main
                        ? &padded_reconstructed[future_anchor[local]] : nullptr;
                    const size_t past_index=past_anchor[local], future_index=future_anchor[local];
                    const bool past_field_picture=p.cfg.syntax==libvc1::StreamSyntax::Advanced && p.cfg.interlaced() &&
                        kind[past_index]==GopPictureKind::P && !skipped_picture[past_index];
                    const bool future_field_picture=p.cfg.syntax==libvc1::StreamSyntax::Advanced && p.cfg.interlaced() &&
                        kind[future_index]==GopPictureKind::P && !skipped_picture[future_index];
                    choice->bstats=qenc.encode_b_picture(f,reconstructed[past_index],
                                                          reconstructed[future_index],banalysis,num,den,ppast,pfuture,
                                                          past_field_picture,future_field_picture,previous_source);
                }
                choice->au=std::move(choice->bstats.data);
                choice->reconstructed=std::move(choice->bstats.reconstructed);
                if (p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main)
                    choice->padded_reconstructed=std::move(choice->bstats.padded_reconstructed);
                else
                    choice->bstats.padded_reconstructed=libvc1::Frame{};
            }
            auto [it,ok]=cache.emplace(cache_key,std::move(choice));
            (void)ok;
            return *it->second;
        };

        int selected_q=p.cq_set?p.cq_value:previous_q;
        bool selected_halfqp=p.cq_set?p.cfg.halfqp:false;
        int predicted_q=0;
        bool predicted_halfqp=false;
        int rc_retries=0;
        double rc_complexity=0.0,rc_predicted_bits=0.0,rc_target_bits=0.0,rc_allowed_bits=0.0;
        double rc_planned_bits=0.0;
        double rc_aq_debt_credit_bits=0.0;
        uint64_t rc_first_actual_bits=0;
        if (p.bounded_cq && skip_pic) {
            const size_t pref=past_anchor[local];
            selected_q=anchor_q[pref];
            selected_halfqp=anchor_halfqp[pref]!=0;
            predicted_q=selected_q;
            predicted_halfqp=selected_halfqp;
            rc_complexity=0.0;
            rc_allowed_bits=rate_control?rate_control->max_picture_bits():0.0;
            rc_target_bits=abrrc.effective_target_per_frame();
            rc_planned_bits=abrrc.planned_picture_bits(GopPictureKind::P)*measured_weight;
            EncodedChoice& sc=evaluate(selected_q,selected_halfqp);
            rc_first_actual_bits=static_cast<uint64_t>(sc.au.size())*8ull;
            rc_predicted_bits=static_cast<double>(rc_first_actual_bits); // exact fixed syntax, not a learned P prediction
            if (rate_control && static_cast<double>(rc_first_actual_bits)>rc_allowed_bits) throw BoundedCqExceeded();
            abrrc.observe_skipped_p(rc_first_actual_bits,
                AbrRateControl::q_to_scale(static_cast<double>(selected_q)+(selected_halfqp?0.5:0.0)));
        } else if (p.bounded_cq) {
            // Analyze before entropy coding, then let the libvc1 ABR running
            // rate-factor/overflow/size-predictor model choose Q.
            if (pk==GopPictureKind::I) rc_complexity=fast_intra_activity(f,p.cfg);
            else if (pk==GopPictureKind::P) rc_complexity=panalysis.rate_complexity;
            else rc_complexity=banalysis.rate_complexity;

            // If inter-picture intra was selected, perform one exact dry-run at
            // the analysis quantizer and use its real entropy size as a rate-
            // complexity floor.  evaluate() is cached, so when final Q equals
            // the analysis Q this pass is reused rather than encoded twice.
            // This captures actual DC/AC prediction, CBP/MVDATA, DQUANT, chroma,
            // and BI promotion instead of trying to infer their cost from SAD.
            const size_t rate_intra_mbs=pk==GopPictureKind::P?panalysis.intra_macroblocks:
                (pk==GopPictureKind::B?banalysis.intra_macroblocks:0u);
            if (rate_intra_mbs && !p.cfg.interlaced()) {
                const uint64_t reference_bits=static_cast<uint64_t>(
                    evaluate(analysis_cfg.pqindex,analysis_cfg.halfqp).au.size())*8ull;
                rc_complexity=abrrc.reference_bits_adjusted_complexity(
                    pk,rc_complexity,reference_bits,analysis_cfg.pqindex,analysis_cfg.halfqp,p.cfg,
                    picture_block_weight);
            }

            rc_allowed_bits=rate_control?rate_control->max_picture_bits():0.0;

            // 0.1.72 ABR coding-law integration. Obtain a provisional legal
            // integer/half-step Q from an untouched controller copy, resolve
            // AUTO uniform/nonuniform PQUANT at that Q, and model its sampled
            // coefficient-rate change together with the already-integrated
            // TTFRM/TTBLK transform-law delta. Motion complexity already comes
            // from the same full progressive mode RDO used by the final encode.
            libvc1::EncoderConfig rc_cfg=p.cfg;
            if (aq_contextual_integer_picture) rc_cfg.allow_halfqp=false;
            auto provisional_rc=abrrc;
            const auto provisional=provisional_rc.choose(pk,rc_complexity,rc_cfg,rate_control,
                                                          p.bounded_maximize,p.bounded_min_q,1.0,
                                                          picture_block_weight,measured_weight,p.two_pass_mode==2);
            libvc1::EncoderConfig lawcfg=rc_cfg;
            lawcfg.pqindex=provisional.q;
            lawcfg.halfqp=provisional.halfqp;
            lawcfg.extended_transform_signaling=true;
            libvc1::Vc1Encoder qestimator(lawcfg);
            auto qrate=qestimator.estimate_picture_quantizer_rate(f);
            if (aq_contextual_integer_picture) {
                qrate.selected=libvc1::QuantizerType::Uniform;
                qrate.rate_scale=1.0;
                lawcfg.halfqp=false;
            }
            if (lawcfg.halfqp && qrate.selected==libvc1::QuantizerType::NonUniform) {
                qrate.selected=libvc1::QuantizerType::Uniform;
                qrate.rate_scale=1.0;
            }
            lawcfg.quantizer_type=qrate.selected;
            double transform_scale=1.0;
            double aq_rate_scale=1.0;
            if (pk!=GopPictureKind::I && p.cfg.variable_transforms) {
                libvc1::Vc1Encoder testimator(lawcfg);
                libvc1::Vc1Encoder::TransformRateEstimate tx{};
                uint64_t motion_bits=0;
                if (pk==GopPictureKind::P) {
                    tx=testimator.estimate_p_transform_rate(f,reconstructed[past_anchor[local]],panalysis);
                    motion_bits=panalysis.motion_bits;
                } else {
                    tx=testimator.estimate_b_transform_rate(f,reconstructed[past_anchor[local]],
                                                           reconstructed[future_anchor[local]],banalysis);
                    motion_bits=banalysis.motion_bits;
                }
                if (tx.sampled_inter_macroblocks && tx.legacy_transform_bits>0.0) {
                    const double provisional_whole=std::max(64.0,provisional.predicted_bits);
                    const double fixed_other=std::max(64.0,provisional_whole-
                        static_cast<double>(motion_bits)-tx.legacy_transform_bits);
                    const double legacy_whole=static_cast<double>(motion_bits)+fixed_other+tx.legacy_transform_bits;
                    const double extended_whole=static_cast<double>(motion_bits)+fixed_other+tx.extended_transform_bits;
                    transform_scale=std::clamp(extended_whole/std::max(64.0,legacy_whole),0.90,1.10);
                    if (p.cfg.adaptive_quality && tx.aq_extended_transform_bits>0.0)
                        aq_rate_scale=std::clamp(tx.aq_extended_transform_bits/
                            std::max(1.0,tx.extended_transform_bits),0.90,1.12);
                }
            }
            // AQ's sampled residual premium is intentional perceptual spend, not
            // a different codec law that should immediately coarsen the same
            // picture.  Keep quantizer-type/transform-law scaling in the Q
            // predictor, but account the AQ premium only through the bounded
            // short-term debt credit below. Long-term ABR and VBV still see the
            // real coded bits. This prevents the controller from cancelling AQ
            // before the protected transform decisions are even encoded.
            const double coding_law_scale=std::clamp(qrate.rate_scale*transform_scale,0.85,1.15);
            auto decision=[&]{
                libvc1::SpeedProfileScope speed_scope(p.cfg.speed_profiler,libvc1::SpeedProfileOp::RateControl);
                return abrrc.choose(pk,rc_complexity,rc_cfg,rate_control,p.bounded_maximize,
                                     p.bounded_min_q,coding_law_scale,picture_block_weight,measured_weight,p.two_pass_mode==2);
            }();
            predicted_q=decision.q;
            predicted_halfqp=decision.halfqp;
            selected_q=predicted_q;
            selected_halfqp=predicted_halfqp;
            rc_predicted_bits=decision.predicted_bits;
            rc_planned_bits=decision.planned_bits;
            // Keep the CSV target field meaningful: ABR allocates
            // quality through its rate factor rather than fixed I/P/B shares.
            rc_target_bits=decision.target_bits;
            rc_first_actual_bits=static_cast<uint64_t>(evaluate(predicted_q,predicted_halfqp).au.size())*8ull;
            double actual=static_cast<double>(rc_first_actual_bits);

            // GOP-opening I pictures have no earlier same-type observation
            // inside an independently threaded GOP.  If the first entropy
            // result still exceeds its intentional type-weighted share by
            // a very large margin, correct it once before that excess turns
            // into GOP debt and starves the first P/B group.  Since 0.2.12's
            // default I-frame share is already intentionally larger, do not
            // stack the old extra 65%/35% overspend cushions on top of that
            // enlarged plan.  Trigger at 35% over the configured I share and
            // repair toward 20% over it.  This preserves user-controlled I
            // weighting while bounding pathological first-I predictor misses.
            // The retry still reuses the already-computed intra analysis and
            // remains limited to at most one extra I encode.
            if (p.two_pass_mode!=2 && pk==GopPictureKind::I && selected_q<31 &&
                actual>decision.planned_bits*1.35) {
                double repair_target=decision.planned_bits*1.20;
                if (rate_control) repair_target=std::min(repair_target,rc_allowed_bits*0.95);
                if (repair_target>0.0 && repair_target<actual) {
                    const double qs0=AbrRateControl::q_to_scale(static_cast<double>(selected_q)+(selected_halfqp?0.5:0.0));
                    const double qs=qs0*(actual/repair_target);
                    int q2=AbrRateControl::scale_to_q(qs,p.bounded_min_q);
                    q2=std::clamp(std::max(selected_q+1,q2),selected_q+1,31);
                    if (q2!=selected_q) {
                        selected_q=q2;
                        selected_halfqp=false;
                        ++rc_retries;
                        actual=static_cast<double>(evaluate(q2,false).au.size())*8.0;
                    }
                }
            }

            // Rare VC-1 skip/residual cliff safety net.  After the
            // predictor-side B allocation fixes above, a difficult picture
            // can still occasionally quantize almost all correction away
            // and land below 20% of its meaningful frame-budget reference.
            // In that extreme case only, repeat transform/quantization/entropy coding at a
            // bounded finer Q. Motion analysis and mode decisions are reused.
            if (p.two_pass_mode!=2 && pk==GopPictureKind::B && predicted_q>p.bounded_min_q &&
                decision.var>=decision.planned_bits*1.50) {
                // Compare the actual first encode against both the local
                // type-weighted plan and the nominal/current frame target.
                // Syntax-efficiency gains elsewhere in the GOP (for
                // example intra AC prediction) can shrink the local plan
                // just enough to hide a real VC-1 skip/residual cliff.
                // Difficult B pictures should still be protected from
                // collapsing below 20% of the ordinary frame budget.
                const double cliff_reference=std::max(decision.planned_bits,decision.target_bits);
                if (actual<cliff_reference*0.20) {
                    double repair_target=cliff_reference*0.45;
                    if (rate_control) repair_target=std::min(repair_target,rc_allowed_bits*0.90);
                    if (repair_target>actual) {
                        const double ratio=std::clamp(actual/repair_target,0.05,1.0);
                        const double raw_q=static_cast<double>(predicted_q)*
                            std::pow(ratio,1.0/AbrRateControl::kQExponent);
                        int q2=std::clamp(static_cast<int>(std::llround(raw_q)),
                                          p.bounded_min_q,predicted_q-1);
                        q2=std::max(q2,predicted_q-8);
                        if (decision.b_reference_q>0) q2=std::max(q2,decision.b_reference_q);
                        if (q2<predicted_q) {
                            const double repaired=static_cast<double>(evaluate(q2,false).au.size())*8.0;
                            if (!rate_control || repaired<=rc_allowed_bits*0.97) {
                                selected_q=q2;
                                selected_halfqp=false;
                                actual=repaired;
                                ++rc_retries;
                            }
                        }
                    }
                }
            }


            // Ordinary pictures avoid whole-frame re-encoding; without row-level VBV feedback the encoder cannot
            // react before the frame finishes. VC-1 currently lacks that
            // row-level DQUANT path, so only a *real* VBV violation gets one
            // direct corrective encode. Ordinary ABR over/undershoot is
            // learned by the predictor and overflow state for future frames.
            if (rate_control && actual>rc_allowed_bits && selected_q<31) {
                const int before_vbv_q=selected_q;
                const double qs0=AbrRateControl::q_to_scale(static_cast<double>(before_vbv_q)+(selected_halfqp?0.5:0.0));
                double qs=qs0*std::max(1.02,actual/std::max(32.0,rc_allowed_bits*0.97));
                int q2=std::clamp(AbrRateControl::scale_to_q(qs,p.bounded_min_q),before_vbv_q+1,31);
                if (q2!=before_vbv_q) {
                    selected_q=q2;
                    selected_halfqp=false;
                    ++rc_retries;
                    actual=static_cast<double>(evaluate(q2,false).au.size())*8.0;
                }
                if (actual>rc_allowed_bits && selected_q<31) {
                    selected_q=31;
                    selected_halfqp=false;
                    ++rc_retries;
                    actual=static_cast<double>(evaluate(31,false).au.size())*8.0;
                }
                if (actual>rc_allowed_bits) throw BoundedCqExceeded();
            }
            auto observed=decision;
            observed.qscale=AbrRateControl::q_to_scale(static_cast<double>(selected_q)+(selected_halfqp?0.5:0.0));
            const uint64_t observed_bits=static_cast<uint64_t>(evaluate(selected_q,selected_halfqp).au.size())*8ull;
            if (p.cfg.adaptive_quality && p.cfg.aq_strength>0.0 && aq_rate_scale>1.0) {
                // AQ changes the distribution of residual bits inside a picture.
                // Estimate that intentional premium from the sampled AQ/non-AQ
                // transform ratio and the *actual* coded size rather than from
                // the much smaller nominal B/P share.  This credit affects only
                // short-term intra-GOP debt.  Predictor learning, long-term ABR
                // overflow and authoritative VBV still consume observed_bits.
                rc_aq_debt_credit_bits=static_cast<double>(observed_bits)*(1.0-1.0/aq_rate_scale);
            }
            {
                libvc1::SpeedProfileScope speed_scope(p.cfg.speed_profiler,libvc1::SpeedProfileOp::RateControl);
                abrrc.observe(pk,observed,observed_bits,rc_aq_debt_credit_bits);
            }

        }

        EncodedChoice& choice=evaluate(selected_q,selected_halfqp);
        if (p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main && pk==GopPictureKind::B && choice.bstats.bi_picture)
            main_rnd_state=true;
        if (skip_pic) {
            const size_t pref=past_anchor[local];
            // Skipped pictures are mandatory adjacent anchors, so the old
            // reference has no remaining B dependants. Transfer its backing
            // storage instead of copying a full Y/U/V frame.
            reconstructed[local]=std::move(reconstructed[pref]);
            if (p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main)
                padded_reconstructed[local]=std::move(padded_reconstructed[pref]);
        } else {
            reconstructed[local]=std::move(choice.reconstructed);
            if (p.cfg.syntax==libvc1::StreamSyntax::Wmv9Main)
                padded_reconstructed[local]=std::move(choice.padded_reconstructed);
        }
        if (p.debug_macroblock_stats && skip_pic) {
            const int mbw=(p.cfg.width+15)/16,mbh=(p.cfg.height+15)/16;
            choice.pstats.macroblock_debug.assign(static_cast<size_t>(mbw)*mbh,{});
            for(auto& d:choice.pstats.macroblock_debug){d.i_mode=VC1_MB_DEBUG_P_SKIPPED;d.i_field_index=d.i_field_parity=-1;d.b_skipped=1;d.i_picture_q=choice.q;d.i_mquant=choice.q;d.i_forward_reference_display_order=d.i_backward_reference_display_order=-1;}
            libvc1::Vc1Encoder dbgenc(p.cfg);
            dbgenc.fill_macroblock_quality(choice.pstats.macroblock_debug,f,reconstructed[local],previous_source,&reconstructed[local],nullptr);
        }
        if (pk!=GopPictureKind::B) {
            anchor_q[local]=choice.q;
            anchor_halfqp[local]=choice.halfqp?1u:0u;
            anchor_quantizer[local]=choice.quantizer_type;
        }
        if (pk==GopPictureKind::P) {
            if (skip_pic) {
                const size_t mbs=static_cast<size_t>((p.cfg.width+15)/16)*static_cast<size_t>((p.cfg.height+15)/16);
                anchor_motion[local].assign(mbs,{});
                anchor_4mv[local].assign(mbs,0);
                anchor_intensity[local]={};
            } else {
                anchor_motion[local]=panalysis.mvs;
                anchor_4mv[local]=panalysis.use_4mv;
            }
            if (p.intra_gop_parallelism && p.intra_gop_workers>1 && p.cfg.syntax==libvc1::StreamSyntax::Advanced) {
                int launched=0;
                for (size_t j=0;j<n && launched<p.intra_gop_workers-1;++j) {
                    if (kind[j]!=GopPictureKind::B || future_anchor[j]!=local || b_analysis_jobs[j].valid()) continue;
                    const size_t past=past_anchor[j],future=future_anchor[j];
                    const int den=static_cast<int>(future-past);
                    const int num=static_cast<int>(j-past);
                    const libvc1::Frame* fj=&frames[j];
                    const libvc1::Frame* pr=&reconstructed[past];
                    const libvc1::Frame* fr=&reconstructed[future];
                    const auto* fm=&anchor_motion[future];
                    const auto* f4=&anchor_4mv[future];
                    const auto ic=anchor_intensity[future];
                    auto cfg_copy=p.cfg;
                    if (!p.cq_set) {
                        cfg_copy.pqindex=anchor_q[future];
                        cfg_copy.halfqp=anchor_halfqp[future]!=0;
                        cfg_copy.quantizer_type=anchor_quantizer[future];
                    }
                    b_analysis_jobs[j]=std::async(std::launch::async,[fj,pr,fr,fm,f4,ic,num,den,cfg_copy]() {
                        libvc1::Vc1Encoder enc(cfg_copy);
                        return enc.analyze_b_picture(*fj,*pr,*fr,*fm,*f4,ic,num,den);
                    });
                    ++launched;
                }
            }
        }
        double debug_mean_mv=0.0,debug_max_mv=0.0;
        if (p.debug_stats && pk!=GopPictureKind::I) {
            long double sum=0.0L; uint64_t count=0;
            auto add_mv=[&](const libvc1::Vc1Encoder::MotionVector& mv) {
                const double mag=std::hypot(static_cast<double>(mv.xq),static_cast<double>(mv.yq))/4.0;
                sum+=mag; debug_max_mv=std::max(debug_max_mv,mag); ++count;
            };
            if (pk==GopPictureKind::P) for (const auto& mv:panalysis.mvs) add_mv(mv);
            else {
                for (const auto& mv:banalysis.forward_mvs) add_mv(mv);
                for (const auto& mv:banalysis.backward_mvs) add_mv(mv);
            }
            if (count) debug_mean_mv=static_cast<double>(sum/static_cast<long double>(count));
        }

        EncodedGopPicture picture;
        picture.au=std::move(choice.au);
        picture.display_index=static_cast<uint64_t>(local);
        picture.kind=pk;
        picture.key=force_i;
        picture.skipped_picture=skip_pic;
        picture.q=selected_q;
        picture.halfqp=choice.halfqp;
        picture.quantizer_type=choice.quantizer_type;
        picture.quant_step=choice.quant_step;
        if (p.debug_macroblock_stats) {
            if (pk==GopPictureKind::I) picture.macroblock_debug=choice.istats.macroblock_debug;
            else if (pk==GopPictureKind::P) picture.macroblock_debug=choice.pstats.macroblock_debug;
            else picture.macroblock_debug=choice.bstats.macroblock_debug;
            const int64_t forward_ref = pk==GopPictureKind::I ? -1 :
                static_cast<int64_t>(start_frame+past_anchor[local]);
            const int64_t backward_ref = pk==GopPictureKind::B ?
                static_cast<int64_t>(start_frame+future_anchor[local]) : -1;
            for (auto& d:picture.macroblock_debug) {
                d.i_forward_reference_display_order=-1;
                d.i_backward_reference_display_order=-1;
                if (d.b_intra || d.i_mode==VC1_MB_DEBUG_I || d.i_mode==VC1_MB_DEBUG_BI_INTRA) continue;
                const int64_t current_display=static_cast<int64_t>(start_frame+local);
                if (d.i_mode==VC1_MB_DEBUG_FIELD_P_FORWARD || d.i_mode==VC1_MB_DEBUG_FIELD_B_FORWARD) {
                    // In the second coded field an opposite-polarity forward reference is
                    // the already reconstructed first field of this same picture.
                    d.i_forward_reference_display_order=(d.i_field_index==1 && d.b_opposite_field_reference)
                        ? current_display : forward_ref;
                } else if (pk==GopPictureKind::P || d.i_mode==VC1_MB_DEBUG_B_FORWARD ||
                           d.i_mode==VC1_MB_DEBUG_B_INTERPOLATED || d.i_mode==VC1_MB_DEBUG_B_DIRECT) {
                    d.i_forward_reference_display_order=forward_ref;
                }
                if (d.i_mode==VC1_MB_DEBUG_FIELD_B_BACKWARD ||
                    (pk==GopPictureKind::B && (d.i_mode==VC1_MB_DEBUG_B_BACKWARD ||
                     d.i_mode==VC1_MB_DEBUG_B_INTERPOLATED || d.i_mode==VC1_MB_DEBUG_B_DIRECT)))
                    d.i_backward_reference_display_order=backward_ref;
            }
            uint64_t local_debug_bits=0;
            for(const auto& d:picture.macroblock_debug)local_debug_bits+=d.i_local_bits;
            const uint64_t frame_debug_bits=static_cast<uint64_t>(picture.au.size())*8ull;
            const double shared_per_mb=picture.macroblock_debug.empty()?0.0:
                static_cast<double>(frame_debug_bits>local_debug_bits?frame_debug_bits-local_debug_bits:0)/picture.macroblock_debug.size();
            for(auto& d:picture.macroblock_debug)d.f_amortized_frame_bits=static_cast<double>(d.i_local_bits)+shared_per_mb;
        }
        if (p.bounded_cq) {
            picture.rc_complexity=rc_complexity;
            picture.rc_predicted_bits=rc_predicted_bits;
            picture.rc_target_bits=rc_target_bits;
            picture.rc_allowed_bits=rc_allowed_bits;
            picture.rc_first_actual_bits=rc_first_actual_bits;
            picture.rc_predicted_q=predicted_q;
            picture.rc_retries=rc_retries;
            picture.rc_prediction_error_percent=rc_predicted_bits>0.0
                ? (static_cast<double>(rc_first_actual_bits)-rc_predicted_bits)*100.0/rc_predicted_bits : 0.0;
        }
        if (p.two_pass_mode==1) {
            picture.pass1_mse_y=vc1_twopass::plane_mse(f.y,reconstructed[local].y);
            picture.pass1_mse_uv=vc1_twopass::chroma_mse(f.u,f.v,reconstructed[local].u,reconstructed[local].v);
            picture.debug_intra_macroblocks=picture_intra_mbs;
            if (pk==GopPictureKind::P) picture.debug_moved_macroblocks=panalysis.moved_macroblocks;
            if (pk==GopPictureKind::B) picture.debug_moved_macroblocks=banalysis.moved_macroblocks;
        }
        if (p.two_pass_mode!=0) {
            picture.two_pass_budget_scale=p.modern_budget_scale;
            picture.two_pass_i_weight=p.rc_i_weight;
            picture.two_pass_p_weight=p.rc_p_weight;
            picture.two_pass_b_weight=p.rc_b_weight;
        }
        if (p.debug_stats) {
            picture.debug_scene_i=scene_i;
            picture.debug_motion_failure_i=motion_failure_i;
            picture.debug_qscale=AbrRateControl::q_to_scale(static_cast<double>(selected_q)+(choice.halfqp?0.5:0.0));
            picture.debug_rc_planned_bits=rc_planned_bits;
            picture.debug_gop_budget_scale=p.modern_budget_scale;
            picture.debug_gop_difficulty=p.modern_difficulty;
            picture.debug_motion_residual=pk==GopPictureKind::P?panalysis.mean_abs_residual:
                (pk==GopPictureKind::B?banalysis.mean_abs_residual:0.0);
            picture.debug_mean_mv_pixels=debug_mean_mv;
            picture.debug_max_mv_pixels=debug_max_mv;
            picture.debug_encode_trials=static_cast<int>(cache.size());
            if (pk==GopPictureKind::I) {
                picture.debug_acpred_macroblocks=choice.istats.acpred_macroblocks;
                picture.debug_dquant_macroblocks=choice.istats.dquant_macroblocks;
                if (choice.istats.mquant_samples) {
                    picture.debug_mquant_min=choice.istats.mquant_min;
                    picture.debug_mquant_max=choice.istats.mquant_max;
                    picture.debug_mquant_mean=static_cast<double>(choice.istats.mquant_sum)/choice.istats.mquant_samples;
                }
            } else if (pk==GopPictureKind::P) {
                picture.debug_intensity_comp=!p.cfg.interlaced() && panalysis.intensity.enabled;
                picture.debug_moved_macroblocks=p.cfg.interlaced()?0:panalysis.moved_macroblocks;
                picture.debug_fractional_chroma_macroblocks=p.cfg.interlaced()?0:panalysis.fractional_chroma_macroblocks;
                picture.debug_skipped_macroblocks=choice.pstats.skipped_macroblocks;
                picture.debug_explicit_macroblocks=choice.pstats.explicit_macroblocks;
                picture.debug_coded_macroblocks=choice.pstats.coded_macroblocks;
                picture.debug_coded_blocks=choice.pstats.coded_blocks;
                picture.debug_four_mv_macroblocks=choice.pstats.four_mv_macroblocks;
                picture.debug_intra_macroblocks=choice.pstats.intra_macroblocks;
                picture.debug_acpred_macroblocks=choice.pstats.acpred_macroblocks;
                picture.debug_dquant_macroblocks=choice.pstats.dquant_macroblocks;
                if (choice.pstats.mquant_samples) {
                    picture.debug_mquant_min=choice.pstats.mquant_min;
                    picture.debug_mquant_max=choice.pstats.mquant_max;
                    picture.debug_mquant_mean=static_cast<double>(choice.pstats.mquant_sum)/choice.pstats.mquant_samples;
                }
                for (size_t t=0;t<4;++t) picture.debug_transform_parts[t]=choice.pstats.transform_parts[t];
                picture.debug_ttmbf=choice.pstats.frame_level_transform;
                picture.debug_ttfrm=choice.pstats.frame_level_transform ? static_cast<int>(choice.pstats.frame_transform_type) : 0;
                picture.debug_ttfrm_exact_checked=choice.pstats.frame_transform_exact_checked;
            } else {
                picture.debug_moved_macroblocks=p.cfg.interlaced()?0:banalysis.moved_macroblocks;
                picture.debug_fractional_chroma_macroblocks=p.cfg.interlaced()?0:banalysis.fractional_chroma_macroblocks;
                picture.debug_skipped_macroblocks=choice.bstats.skipped_macroblocks;
                picture.debug_explicit_macroblocks=choice.bstats.explicit_macroblocks;
                picture.debug_coded_macroblocks=choice.bstats.coded_macroblocks;
                picture.debug_coded_blocks=choice.bstats.coded_blocks;
                picture.debug_intra_macroblocks=choice.bstats.intra_macroblocks;
                picture.debug_acpred_macroblocks=choice.bstats.acpred_macroblocks;
                picture.debug_dquant_macroblocks=choice.bstats.dquant_macroblocks;
                if (choice.bstats.mquant_samples) {
                    picture.debug_mquant_min=choice.bstats.mquant_min;
                    picture.debug_mquant_max=choice.bstats.mquant_max;
                    picture.debug_mquant_mean=static_cast<double>(choice.bstats.mquant_sum)/choice.bstats.mquant_samples;
                }
                for (size_t t=0;t<4;++t) picture.debug_transform_parts[t]=choice.bstats.transform_parts[t];
                picture.debug_ttmbf=choice.bstats.frame_level_transform;
                picture.debug_ttfrm=choice.bstats.frame_level_transform ? static_cast<int>(choice.bstats.frame_transform_type) : 0;
                picture.debug_ttfrm_exact_checked=choice.bstats.frame_transform_exact_checked;
                picture.debug_b_forward=p.cfg.interlaced()?choice.bstats.forward_macroblocks:banalysis.forward_macroblocks;
                picture.debug_b_backward=p.cfg.interlaced()?choice.bstats.backward_macroblocks:banalysis.backward_macroblocks;
                picture.debug_b_interpolated=p.cfg.interlaced()?choice.bstats.interpolated_macroblocks:banalysis.interpolated_macroblocks;
                picture.debug_b_direct=p.cfg.interlaced()?choice.bstats.direct_macroblocks:banalysis.direct_macroblocks;
            }
        }
        if (p.keep_reconstruction) {
            if (pk==GopPictureKind::B) picture.reconstructed=std::move(reconstructed[local]);
            else picture.reconstructed=reconstructed[local];
        }
        if (pk==GopPictureKind::P) picture.transform_map=choice.pstats.transform_map;
        else if (pk==GopPictureKind::B) picture.transform_map=choice.bstats.transform_map;

        if (pk==GopPictureKind::I) {
            ++out.i_frames;
            out.dquant_macroblocks += choice.istats.dquant_macroblocks;
            if (scene_i) ++out.scene_i_frames;
            if (motion_failure_i) ++out.motion_failure_i_frames;
        } else if (pk==GopPictureKind::P) {
            ++out.p_frames;
            if (skip_pic) ++out.skipped_pictures;
            if (!p.cfg.interlaced() && !skip_pic && panalysis.intensity.enabled) ++out.ic_p_frames;
            if (!p.cfg.interlaced()) {
                out.moved_macroblocks += panalysis.moved_macroblocks;
                out.fractional_chroma_macroblocks += panalysis.fractional_chroma_macroblocks;
            }
            out.skipped_macroblocks += choice.pstats.skipped_macroblocks;
            out.explicit_macroblocks += choice.pstats.explicit_macroblocks;
            out.coded_macroblocks += choice.pstats.coded_macroblocks;
            out.coded_blocks += choice.pstats.coded_blocks;
            out.four_mv_macroblocks += choice.pstats.four_mv_macroblocks;
            out.intra_macroblocks += choice.pstats.intra_macroblocks;
            out.dquant_macroblocks += choice.pstats.dquant_macroblocks;
            for (size_t t=0;t<4;++t) out.transform_parts[t]+=choice.pstats.transform_parts[t];
        } else {
            ++out.b_frames;
            out.b_forward += p.cfg.interlaced()?choice.bstats.forward_macroblocks:banalysis.forward_macroblocks;
            out.b_backward += p.cfg.interlaced()?choice.bstats.backward_macroblocks:banalysis.backward_macroblocks;
            out.b_interpolated += p.cfg.interlaced()?choice.bstats.interpolated_macroblocks:banalysis.interpolated_macroblocks;
            out.b_direct += p.cfg.interlaced()?choice.bstats.direct_macroblocks:banalysis.direct_macroblocks;
            if (!p.cfg.interlaced()) {
                out.moved_macroblocks += banalysis.moved_macroblocks;
                out.fractional_chroma_macroblocks += banalysis.fractional_chroma_macroblocks;
            }
            out.skipped_macroblocks += choice.bstats.skipped_macroblocks;
            out.explicit_macroblocks += choice.bstats.explicit_macroblocks;
            out.coded_macroblocks += choice.bstats.coded_macroblocks;
            out.coded_blocks += choice.bstats.coded_blocks;
            out.intra_macroblocks += choice.bstats.intra_macroblocks;
            out.dquant_macroblocks += choice.bstats.dquant_macroblocks;
            for (size_t t=0;t<4;++t) out.transform_parts[t]+=choice.bstats.transform_parts[t];
        }

        const uint64_t picture_bits=static_cast<uint64_t>(picture.au.size())*8ull;
        if (rate_control) {
            const double before=rate_control->max_picture_bits();
            local_hrd_max_pre=std::max(local_hrd_max_pre,before);
            if (p.bounded_cq) {
                picture.rc_allowed_bits=before;
                picture.rc_vbv_before_bits=before;
            }
            rate_control->commit(picture_bits);
            local_hrd_min_after=std::min(local_hrd_min_after,rate_control->fullness());
            if (p.bounded_cq) picture.rc_vbv_after_bits=rate_control->fullness();
        }
        previous_q=selected_q;
        if (p.bounded_cq) bounded_q_state=selected_q;
        out.q_sum+=static_cast<uint64_t>(selected_q);
        if (choice.halfqp) ++out.halfqp_frames;
        if (choice.quantizer_type==libvc1::QuantizerType::NonUniform) ++out.nonuniform_quantizer_frames; else ++out.uniform_quantizer_frames;
        out.q_min_used=std::min(out.q_min_used,selected_q);
        out.q_max_used=std::max(out.q_max_used,selected_q);
        out.pictures.push_back(std::move(picture));

        // B pictures are never references. Release their working reconstruction
        // immediately (or it was just moved to the optional output copy). Once
        // the last B between two anchors is coded, the past anchor is dead too.
        // This keeps each worker near a two-reference working set instead of
        // retaining reconstructed copies for the entire GOP.
        if (pk==GopPictureKind::B) {
            reconstructed[local]=libvc1::Frame{};
            padded_reconstructed[local]=libvc1::Frame{};
            const size_t future=future_anchor[local];
            if (local+1==future) {
                const size_t past=past_anchor[local];
                reconstructed[past]=libvc1::Frame{};
                padded_reconstructed[past]=libvc1::Frame{};
            }
        } else if (pk==GopPictureKind::P && local>0 && past_anchor[local]+1==local) {
            // Adjacent anchors have no intervening B pictures. The previous
            // anchor can be released as soon as the new one is committed.
            const size_t past=past_anchor[local];
            reconstructed[past]=libvc1::Frame{};
            padded_reconstructed[past]=libvc1::Frame{};
        }
        if (p.cfg.syntax==libvc1::StreamSyntax::Advanced)
            padded_reconstructed[local]=libvc1::Frame{};
    }

    if (p.bounded_cq) out.bounded_end_q=bounded_q_state;
    if (rate_control) {
        out.rate_total_bits=rate_control->total_bits()-rc_bits_before;
        out.rate_frames=rate_control->frames()-rc_frames_before;
        out.hrd_pauses=rate_control->transmission_pauses()-rc_pauses_before;
        out.hrd_underflows=rate_control->underflows()-rc_underflows_before;
        out.hrd_min_after_bits=std::isfinite(local_hrd_min_after)?local_hrd_min_after:0.0;
        out.hrd_max_pre_bits=local_hrd_max_pre;
    }
    return out;
}

static EncodedGop encode_gop(uint64_t index, uint64_t start_frame,
                             std::vector<libvc1::Frame> frames,
                             GopEncodeParams p) {
    if (p.two_pass_mode==2 && !p.cfg.bluray_compat)
        return encode_gop_once(index,start_frame,frames,std::move(p));
    if (p.bounded_cq) {
        // Fast speculative libvc1 ABR pass. Authoritative global VBV validation
        // happens later in strict output order; only a true reservoir violation
        // triggers recovery.
        GopEncodeParams probe=p;
        probe.cq_set=false;
        auto out=encode_gop_once(index,start_frame,frames,std::move(probe));
        out.retry_source=std::move(frames);
        out.bounded_probe=true;
        out.bounded_probe_q=0;
        return out;
    }
    return encode_gop_once(index,start_frame,frames,std::move(p));
}

struct InputMeta {
    int64_t pts=0;
    void* opaque=nullptr;
};

struct ReadyPicture {
    EncodedGopPicture pic;
    uint64_t global_display=0;
    uint64_t coded_order=0;
    uint64_t gop_index=0;
    uint64_t gop_frames=0;
    int64_t pts=0;
    void* opaque=nullptr;
};

static vc1_picture_type_e api_type(GopPictureKind k) {
    switch (k) {
        case GopPictureKind::I: return VC1_TYPE_I;
        case GopPictureKind::B: return VC1_TYPE_B;
        default: return VC1_TYPE_P;
    }
}

static libvc1::AcMode internal_ac_mode(vc1_ac_mode_e m) {
    switch (m) {
        case VC1_AC_VLC: return libvc1::AcMode::Vlc;
        case VC1_AC_ESC3: return libvc1::AcMode::Esc3;
        default: return libvc1::AcMode::Auto;
    }
}


static vc1_simd_e api_simd(libvc1::SimdTier t) {
    switch (t) {
        case libvc1::SimdTier::Mixed: return VC1_SIMD_MIXED;
        case libvc1::SimdTier::X86V1: return VC1_SIMD_X86_64_V1;
        case libvc1::SimdTier::X86V2: return VC1_SIMD_X86_64_V2;
        case libvc1::SimdTier::X86V3: return VC1_SIMD_X86_64_V3;
        case libvc1::SimdTier::X86V4: return VC1_SIMD_X86_64_V4;
        case libvc1::SimdTier::Prescott: return VC1_SIMD_PRESCOTT;
        case libvc1::SimdTier::Conroe: return VC1_SIMD_CONROE;
        case libvc1::SimdTier::Penryn: return VC1_SIMD_PENRYN;
        case libvc1::SimdTier::SandyBridge: return VC1_SIMD_SANDYBRIDGE;
        case libvc1::SimdTier::K10: return VC1_SIMD_K10;
        case libvc1::SimdTier::Bulldozer: return VC1_SIMD_BULLDOZER;
        case libvc1::SimdTier::Piledriver: return VC1_SIMD_PILEDRIVER;
        case libvc1::SimdTier::Avx2Partial: return VC1_SIMD_AVX2_PARTIAL;
        default: return VC1_SIMD_NONE;
    }
}

static libvc1::SimdTier internal_simd(vc1_simd_e t) {
    switch (t) {
        case VC1_SIMD_X86_64_V1: return libvc1::SimdTier::X86V1;
        case VC1_SIMD_X86_64_V2: return libvc1::SimdTier::X86V2;
        case VC1_SIMD_X86_64_V3: return libvc1::SimdTier::X86V3;
        case VC1_SIMD_X86_64_V4: return libvc1::SimdTier::X86V4;
        case VC1_SIMD_PRESCOTT: return libvc1::SimdTier::Prescott;
        case VC1_SIMD_CONROE: return libvc1::SimdTier::Conroe;
        case VC1_SIMD_PENRYN: return libvc1::SimdTier::Penryn;
        case VC1_SIMD_SANDYBRIDGE: return libvc1::SimdTier::SandyBridge;
        case VC1_SIMD_K10: return libvc1::SimdTier::K10;
        case VC1_SIMD_BULLDOZER: return libvc1::SimdTier::Bulldozer;
        case VC1_SIMD_PILEDRIVER: return libvc1::SimdTier::Piledriver;
        case VC1_SIMD_AVX2_PARTIAL: return libvc1::SimdTier::Avx2Partial;
        default: return libvc1::SimdTier::None;
    }
}

static int coded_dimension_for(const vc1_param_t&,int value) {
    // Keep the internal 4:2:0 raster even for every profile. Advanced Profile
    // signals an odd presentation crop with DISPLAY_EXT; Main Profile carries
    // the visible dimensions out-of-band in its container/transport metadata.
    return (value+1)&~1;
}

static uint64_t coded_macroblocks(int width,int height) {
    return static_cast<uint64_t>((width+15)/16) * static_cast<uint64_t>((height+15)/16);
}

static bool macroblock_rate_exceeds(uint64_t mbs,const vc1_param_t& p,uint64_t cap_per_second) {
    return mbs*static_cast<uint64_t>(p.i_fps_num) >
           cap_per_second*static_cast<uint64_t>(p.i_fps_den);
}

static int validate_dimensions_and_level(const vc1_param_t& p,int coded_width,int coded_height) {
    if (p.i_width<=0 || p.i_height<=0 || p.i_width>8192 || p.i_height>8192)
        throw std::runtime_error("width/height must be positive values no greater than 8192");
    if (coded_width<2 || coded_height<2 || coded_width>8192 || coded_height>8192 ||
        (coded_width&1) || (coded_height&1))
        throw std::runtime_error("VC-1 coded dimensions must be even values in 2..8192");

    const uint64_t mbs=coded_macroblocks(coded_width,coded_height);
    if (p.i_profile==VC1_PROFILE_MAIN) {
        // MP@HL is the largest Main-Profile level: 8192 MB/frame, 245760 MB/s,
        // Rmax=20 Mbit/s, and VBVmax=40,009,728 bits.  Main-profile level is
        // carried by the transport rather than STRUCT_C, but the elementary
        // bitstream must still remain within a defined Main level.
        if (mbs>8192 || macroblock_rate_exceeds(mbs,p,245760))
            throw std::runtime_error("resolution/frame rate exceeds VC-1 Main Profile High level limits");
        if (p.i_rc_method==VC1_RC_ABR) {
            if (p.i_two_pass!=2 && resolved_peak_bitrate(p)>20000000ull)
                throw std::runtime_error("bitrate exceeds VC-1 Main Profile High Rmax (20 Mbit/s)");
            if ((p.i_two_pass!=2 || p.b_bluray_compat) && p.i_vbv_buffer_size>40009728ull)
                throw std::runtime_error("VBV buffer exceeds VC-1 Main Profile High VBVmax");
        }
        return 0;
    }

    // AP@L4 is the largest Advanced-Profile level. The coded-size syntax has
    // 12-bit half-resolution fields (2..8192), while level conformance further
    // caps a frame at 16384 macroblocks and 491520 macroblocks/s.
    if (mbs>16384 || macroblock_rate_exceeds(mbs,p,491520))
        throw std::runtime_error("resolution/frame rate exceeds VC-1 Advanced Profile Level 4 limits");
    if (p.i_rc_method==VC1_RC_ABR) {
        if (p.i_two_pass!=2 && resolved_peak_bitrate(p)>135000000ull)
            throw std::runtime_error("bitrate exceeds VC-1 Advanced Profile Level 4 Rmax (135 Mbit/s)");
        if ((p.i_two_pass!=2 || p.b_bluray_compat) && p.i_vbv_buffer_size>270336000ull)
            throw std::runtime_error("VBV buffer exceeds VC-1 Advanced Profile Level 4 Bmax");
    }

    // Preserve AP@L3 when possible. Non-Blu-ray pass 2 may deliberately exceed
    // even L4's nominal rate; that output is not level-rate conformant.
    const bool needs_l4 = mbs>8192 || macroblock_rate_exceeds(mbs,p,245760) ||
        (p.i_rc_method==VC1_RC_ABR &&
         (resolved_peak_bitrate(p)>45000000ull ||
          ((p.i_two_pass!=2 || p.b_bluray_compat) && p.i_vbv_buffer_size>90112000ull)));
    return needs_l4 ? 4 : 3;
}

static uint64_t resolved_keyint(const vc1_param_t& p) {
    if (p.i_keyint_max>0) return static_cast<uint64_t>(p.i_keyint_max);
    // The historical one-second default is a Blu-ray application rule, not a
    // VC-1 syntax requirement. Generic VC-1 uses a modest 120-frame default;
    // callers remain free to request any positive interval explicitly.
    return p.b_bluray_compat ? bluray_keyint_limit(p) : 120u;
}

static libvc1::Frame copy_input_picture(const vc1_param_t& p,const vc1_picture_t& pic) {
    if (pic.img.i_plane != 3) throw std::runtime_error("input picture must contain three YUV420 planes");
    if (!pic.img.plane[0] || !pic.img.plane[1] || !pic.img.plane[2])
        throw std::runtime_error("input picture contains a null plane");

    const int coded_w=coded_dimension_for(p,p.i_width);
    const int coded_h=coded_dimension_for(p,p.i_height);
    const int cw=(p.i_width+1)/2, ch=(p.i_height+1)/2;
    if (pic.img.i_stride[0] < p.i_width || pic.img.i_stride[1] < cw || pic.img.i_stride[2] < cw)
        throw std::runtime_error("input picture stride is smaller than its plane width");

    libvc1::Frame f;
    f.width=coded_w; f.height=coded_h;
    f.y.resize(static_cast<size_t>(coded_w)*coded_h);
    f.u.resize(static_cast<size_t>(coded_w/2)*(coded_h/2));
    f.v.resize(f.u.size());

    // Copy the caller-visible luma raster and edge-extend the at-most-one hidden
    // row/column needed by libvc1's even internal 4:2:0 coding raster.
    for (int y=0;y<p.i_height;++y) {
        uint8_t* dst=f.y.data()+static_cast<size_t>(y)*coded_w;
        std::memcpy(dst,pic.img.plane[0]+static_cast<size_t>(y)*pic.img.i_stride[0],p.i_width);
        for (int x=p.i_width;x<coded_w;++x) dst[x]=dst[p.i_width-1];
    }
    for (int y=p.i_height;y<coded_h;++y)
        std::memcpy(f.y.data()+static_cast<size_t>(y)*coded_w,
                    f.y.data()+static_cast<size_t>(p.i_height-1)*coded_w,coded_w);

    // For 4:2:0, ceil(display/2) equals coded/2 after even luma padding, so no
    // synthetic chroma sample is required for an odd display edge.
    for (int y=0;y<ch;++y) {
        std::memcpy(f.u.data()+static_cast<size_t>(y)*(coded_w/2),
                    pic.img.plane[1]+static_cast<size_t>(y)*pic.img.i_stride[1],cw);
        std::memcpy(f.v.data()+static_cast<size_t>(y)*(coded_w/2),
                    pic.img.plane[2]+static_cast<size_t>(y)*pic.img.i_stride[2],cw);
    }
    return f;
}

} // anonymous namespace

struct PlannedGop {
    uint64_t index=0;
    uint64_t start=0;
    std::vector<libvc1::Frame> frames;
    GopEncodeParams params;
    double difficulty=1.0;
};

static size_t find_advanced_start_code(const std::vector<uint8_t>& au,uint8_t code,size_t from=0) {
    for (size_t i=from;i+4<=au.size();++i)
        if (au[i]==0x00 && au[i+1]==0x00 && au[i+2]==0x01 && au[i+3]==code) return i;
    return std::string::npos;
}

static void refresh_entry_point_fullness(EncodedGopPicture& picture,
                                         const libvc1::EncoderConfig& cfg,
                                         uint8_t fullness) {
    if (cfg.syntax!=libvc1::StreamSyntax::Advanced || picture.kind!=GopPictureKind::I) return;
    const size_t ep=find_advanced_start_code(picture.au,0x0e);
    const size_t frame=find_advanced_start_code(picture.au,0x0d,ep==std::string::npos?0:ep+4);
    if (ep==std::string::npos || frame==std::string::npos || ep>=frame)
        throw std::runtime_error("predictor probe lacks an ordered VC-1 entry-point/frame header");
    libvc1::Vc1Encoder e(cfg);
    const auto replacement=e.entry_point(fullness);
    picture.au.erase(picture.au.begin()+static_cast<std::ptrdiff_t>(ep),
                     picture.au.begin()+static_cast<std::ptrdiff_t>(frame));
    picture.au.insert(picture.au.begin()+static_cast<std::ptrdiff_t>(ep),replacement.begin(),replacement.end());
}

struct vc1_t {
    vc1_param_t param{};
    libvc1::EncoderConfig cfg{};
    GopEncodeParams base{};
    std::vector<uint8_t> sequence;
    vc1_au_t header_au{};
    std::string last_error;

    uint64_t keyint=1;
    uint64_t frames_received=0;
    uint64_t pictures_output=0;
    uint64_t gops_submitted=0;
    uint64_t gops_written=0;
    uint64_t next_coded_order=0;
    uint64_t current_gop_start=0;
    bool current_gop_starts_scene_i=false;
    bool current_gop_starts_motion_failure_i=false;
    uint64_t scene_cut_min_frames=0;
    uint64_t last_scene_cut_frame=0;
    bool have_last_scene_cut=false;
    bool input_finished=false;
    std::vector<libvc1::Frame> current_gop;
    std::shared_ptr<const libvc1::Frame> previous_source_for_next_gop;
    std::vector<InputMeta> metadata;
    std::deque<PlannedGop> planning;
    std::deque<PlannedGop> queued;
    struct PendingGopFuture {
        uint64_t index=0;
        std::future<EncodedGop> future;
    };
    std::deque<PendingGopFuture> pending;
    // Finished GOP jobs can complete out of order.  Keep them here until the
    // preceding GOPs have committed their serialized VBV state.  Crucially, a
    // finished future no longer occupies one of the worker slots.
    std::map<uint64_t,EncodedGop> completed;

    // Predictor/VBV correction must not run synchronously on the API thread.
    // Difficult scenes can require several full-GOP attempts; doing those in
    // process_completed_probe() prevented the frontend from feeding more input
    // and let the ordinary GOP worker pool drain to zero.  Keep one correction
    // job in flight while later speculative GOPs continue encoding.
    struct BoundedRetryResult {
        EncodedGop encoded;
        libvc1::RateController final_state;
        int final_q=2;
        BoundedRetryResult(EncodedGop&& e,libvc1::RateController&& s,int q)
            : encoded(std::move(e)),final_state(std::move(s)),final_q(q) {}
    };
    struct BoundedHistoryRecoveryResult {
        size_t depth=0;
        std::vector<EncodedGop> outputs; // replacement private GOPs, then current GOP
        std::vector<libvc1::RateController> states_before;
        std::vector<int> qs_before;
        libvc1::RateController final_state;
        int final_q=2;
        BoundedHistoryRecoveryResult(size_t d,std::vector<EncodedGop>&& o,
                                     std::vector<libvc1::RateController>&& st,
                                     std::vector<int>&& qs,
                                     libvc1::RateController&& fs,int fq)
            : depth(d),outputs(std::move(o)),states_before(std::move(st)),
              qs_before(std::move(qs)),final_state(std::move(fs)),final_q(fq) {}
    };
    struct PendingBoundedRetry {
        enum class Phase { GopRetry, PrivateHistoryRecovery };
        uint64_t index=0;
        EncodedGop probe;
        libvc1::RateController state_before;
        int q_before=2;
        Phase phase=Phase::GopRetry;
        std::future<std::optional<BoundedRetryResult>> retry_future;
        std::future<std::optional<BoundedHistoryRecoveryResult>> history_future;
        PendingBoundedRetry(EncodedGop&& p,const libvc1::RateController& state,int q)
            : index(p.index),probe(std::move(p)),state_before(state),q_before(q) {}
    };
    std::unique_ptr<PendingBoundedRetry> bounded_retry_pending;

    std::deque<ReadyPicture> ready;
    ReadyPicture current_output;
    vc1_au_t current_au{};

    vc1_stats_t stats{};
    libvc1::SimdBenchmarkResult simd_benchmark{};
    // New default rate controller is global across GOPs. Workers still run CQ
    // probes independently; only output-order acceptance/retry touches this state.
    std::unique_ptr<libvc1::RateController> bounded_rate_control;
    int bounded_current_q=2;
    vc1_twopass::Plan two_pass;
    std::unique_ptr<vc1_twopass::BudgetLedger> two_pass_budget;
    std::string stats_path_owned;
    std::ofstream pass1_stats;
    std::vector<uint64_t> source_fingerprints;
    uint64_t pass1_total_bits=0;
    bool pass1_complete=false;

    ~vc1_t() {
        // Recovery jobs may read immutable configuration plus the still-private
        // rollback window. Join before member destruction if the caller aborts
        // an encode while a difficult GOP is being repaired.
        if (bounded_retry_pending) {
            if (bounded_retry_pending->phase==PendingBoundedRetry::Phase::GopRetry &&
                bounded_retry_pending->retry_future.valid())
                bounded_retry_pending->retry_future.wait();
            if (bounded_retry_pending->phase==PendingBoundedRetry::Phase::PrivateHistoryRecovery &&
                bounded_retry_pending->history_future.valid())
                bounded_retry_pending->history_future.wait();
        }
    }

    size_t gop_worker_limit() const {
        // 0.1.63: outer GOP workers are the steady-state throughput path.
        if (!two_pass_budget) return static_cast<size_t>(std::max(1,param.i_threads));
        {
            // A short two-pass clip cannot use every GOP as a speculative
            // worker: it would finish before any measured size can feed back.
            // Long films still use all the requested worker threads.
            return std::min(static_cast<size_t>(std::max(1,param.i_threads)),std::max<size_t>(1,
                static_cast<size_t>(std::sqrt(static_cast<double>(two_pass.gops.size())))));
        }
    }

    void pump_workers() {
        // During an authoritative recovery the current GOP owns the encoder's
        // compute budget. Do not launch later GOPs: they would only accumulate
        // behind the ordered output point and consume RAM. Already-running jobs
        // are allowed to finish, but no replacements are started until recovery
        // completes.
        if (bounded_retry_pending) return;
        // pending contains only jobs that have not yet been harvested. Ready
        // futures are removed by harvest_workers(), so completed-but-buffered
        // GOPs do not artificially consume a worker slot.
        while (!queued.empty() && pending.size()<gop_worker_limit()) {
            PlannedGop job=std::move(queued.front()); queued.pop_front();
            const uint64_t index=job.index;
            if (two_pass_budget) {
                // Apply the correction when the worker is actually launched,
                // not when piped source input first forms this GOP.
                job.params.modern_budget_scale*=two_pass_budget->launch(index);
            }
            PendingGopFuture pf;
            pf.index=index;
            pf.future=std::async(std::launch::async,
                [job=std::move(job)]() mutable {
                    return encode_gop(job.index,job.start,std::move(job.frames),std::move(job.params));
                });
            pending.push_back(std::move(pf));
        }
    }

    void plan_window(bool /*flush*/) {
        // No future-GOP buffering: dispatch the current GOP immediately. A two-pass run
        // obtains whole-film information from disk, not from raw frame buffers.
        while (!planning.empty()) {
            queued.push_back(std::move(planning.front()));
            planning.pop_front();
        }
        pump_workers();
    }

    void apply_two_pass_plan(GopEncodeParams& params,uint64_t index,uint64_t start,size_t frames) const {
        if (param.i_two_pass!=2) return;
        if (index>=two_pass.gops.size())
            throw std::runtime_error("pass 2 has more GOPs than its statistics");
        const auto& plan=two_pass.gops[static_cast<size_t>(index)];
        if (plan.start!=start || plan.frames!=frames)
            throw std::runtime_error("pass 2 GOP layout differs from pass 1 (scene cut/keyframe mismatch)");
        params.modern_budget_scale=plan.scale;
        params.modern_difficulty=plan.difficulty;
        params.two_pass_picture_scale=plan.frame_scale;
        if (param.b_two_pass_dynamic_weights) {
            params.rc_i_weight*=plan.dynamic[0];
            params.rc_p_weight*=plan.dynamic[1];
            params.rc_b_weight*=plan.dynamic[2];
        }
    }

    void submit_prefix(size_t count,bool scene_scan_complete) {
        if (!count || count>current_gop.size()) return;
        PlannedGop job;
        job.index=gops_submitted++;
        job.start=current_gop_start;
        job.params=base;
        job.params.previous_source=previous_source_for_next_gop;
        // In reset-on-scene mode this prefix was already scanned before GOP
        // dispatch. Avoid repeating the expensive scene pass in the worker.
        if (scene_scan_complete) job.params.scene_cut=false;
        job.params.first_is_scene_i=current_gop_starts_scene_i;
        job.params.first_is_motion_failure_i=current_gop_starts_motion_failure_i;
        if (base.bounded_cq) job.params.bounded_start_q=bounded_current_q;
        job.params.include_sequence=(cfg.syntax==libvc1::StreamSyntax::Advanced &&
                                     param.b_emit_sequence_header && (job.index==0 || cfg.bluray_compat));
        if (job.params.include_sequence) job.params.sequence=sequence;
        if (count==current_gop.size()) {
            job.frames=std::move(current_gop);
            current_gop.clear();
        } else {
            job.frames.reserve(count);
            for (size_t i=0;i<count;++i) job.frames.push_back(std::move(current_gop[i]));
            std::vector<libvc1::Frame> remain;
            remain.reserve(current_gop.size()-count);
            for (size_t i=count;i<current_gop.size();++i) remain.push_back(std::move(current_gop[i]));
            current_gop=std::move(remain);
        }
        if (job.params.debug_macroblock_stats && !job.frames.empty())
            previous_source_for_next_gop=std::make_shared<libvc1::Frame>(job.frames.back());
        current_gop_start+=count;
        current_gop_starts_scene_i=false;
        current_gop_starts_motion_failure_i=false;
        apply_two_pass_plan(job.params,job.index,job.start,job.frames.size());
        planning.push_back(std::move(job));
        plan_window(false);
    }

    void submit_current() {
        if (current_gop.empty()) return;
        submit_prefix(current_gop.size(),false);
        current_gop_start=frames_received;
    }

    // Default 0.1.50 scheduling: keyint is a maximum distance from the most
    // recent I picture. Scan a full input buffer before dispatch; if a scene
    // cut occurs inside it, submit only the prefix before the cut and keep the
    // cut picture as frame zero of the next GOP. This resets the keyframe
    // interval without duplicating scene analysis in the GOP worker.
    void process_dynamic_boundaries(bool flush_all) {
        // --fixed-gop-grid is the strict-cadence escape hatch: it suppresses
        // both threshold scene cuts and the motion-failure adaptive-I safety net.
        if (param.b_fixed_gop_grid || base.intra_only) {
            if (flush_all) submit_current();
            else while (current_gop.size()>=keyint) submit_prefix(static_cast<size_t>(keyint),false);
            return;
        }
        while (!current_gop.empty() && (flush_all || current_gop.size()>=keyint)) {
            GopEncodeParams scan_params=base;
            if (scan_params.intra_gop_parallelism)
                scan_params.intra_gop_workers=std::max(scan_params.intra_gop_workers,
                    scan_params.intra_gop_workers*static_cast<int>(gop_worker_limit()));
            const auto scene_flags=base.scene_cut ? detect_scene_flags(current_gop,scan_params)
                                                  : std::vector<uint8_t>(current_gop.size(),0);
            const auto motion_flags=detect_motion_failure_flags(current_gop,scan_params);
            size_t cut=current_gop.size();
            bool cut_scene=false,cut_motion=false;
            for (size_t i=1;i<current_gop.size();++i) {
                if (scene_flags[i]) {
                    const uint64_t absolute_frame=current_gop_start+static_cast<uint64_t>(i);
                    const bool scene_allowed=!have_last_scene_cut || scene_cut_min_frames==0 ||
                        absolute_frame-last_scene_cut_frame>=scene_cut_min_frames;
                    if (scene_allowed) {
                        cut=i;
                        cut_scene=true;
                        cut_motion=false;
                        last_scene_cut_frame=absolute_frame;
                        have_last_scene_cut=true;
                        break;
                    }
                    // This boundary was positively identified as a scene cut but
                    // is inside the user-requested cooldown. Do not let the
                    // motion-failure safety net immediately reinsert the same I.
                    continue;
                }
                if (motion_flags[i]) {
                    cut=i;
                    cut_scene=false;
                    cut_motion=true;
                    break;
                }
            }
            if (cut<current_gop.size()) {
                submit_prefix(cut,base.scene_cut);
                current_gop_starts_scene_i=cut_scene;
                current_gop_starts_motion_failure_i=cut_motion;
                if (!flush_all && current_gop.size()<keyint) break;
                continue;
            }
            if (flush_all) {
                submit_prefix(current_gop.size(),base.scene_cut);
                break;
            }
            submit_prefix(static_cast<size_t>(keyint),base.scene_cut);
        }
        if (current_gop.empty()) current_gop_start=frames_received;
    }

    bool accept_bounded_probe(EncodedGop& g) {
        if (!g.bounded_probe || !bounded_rate_control) return true;
        // Validate the already entropy-coded pictures in strict output order.
        // The libvc1 ABR rate-factor learner is deliberately GOP-local so
        // speculative worker decisions are independent of completion timing;
        // the actual VBV reservoir is the global serialized state.
        libvc1::RateController trial=*bounded_rate_control;
        const uint64_t bits_before=trial.total_bits();
        const uint64_t frames_before=trial.frames();
        const uint64_t pauses_before=trial.transmission_pauses();
        const uint64_t underflows_before=trial.underflows();
        double local_min_after=std::numeric_limits<double>::infinity();
        double local_max_pre=0.0;
        for (auto& picture:g.pictures) {
            const double before=trial.max_picture_bits();
            if (picture.kind==GopPictureKind::I)
                refresh_entry_point_fullness(picture,cfg,trial.fullness_code());
            const uint64_t picture_bits=static_cast<uint64_t>(picture.au.size())*8ull;
            if (static_cast<double>(picture_bits)>before) return false;
            picture.rc_allowed_bits=before;
            picture.rc_vbv_before_bits=before;
            local_max_pre=std::max(local_max_pre,before);
            trial.commit(picture_bits);
            picture.rc_vbv_after_bits=trial.fullness();
            local_min_after=std::min(local_min_after,trial.fullness());
        }
        g.rate_total_bits=trial.total_bits()-bits_before;
        g.rate_frames=trial.frames()-frames_before;
        g.hrd_init_bits=frames_before==0?trial.initial_fullness_bits_floor():0;
        g.hrd_pauses=trial.transmission_pauses()-pauses_before;
        g.hrd_underflows=trial.underflows()-underflows_before;
        g.hrd_min_after_bits=std::isfinite(local_min_after)?local_min_after:0.0;
        g.hrd_max_pre_bits=local_max_pre;
        g.bounded_probe=false;
        *bounded_rate_control=trial;
        if (!g.pictures.empty()) bounded_current_q=g.pictures.back().q;
        g.bounded_end_q=bounded_current_q;
        return true;
    }

    static void carry_probe_diagnostics(const EncodedGop& probe,EncodedGop& out,int extra_retry=1) {
        out.scene_i_frames=probe.scene_i_frames;
        for (auto& dst:out.pictures) {
            auto it=std::find_if(probe.pictures.begin(),probe.pictures.end(),[&](const EncodedGopPicture& src) {
                return src.display_index==dst.display_index;
            });
            if (it==probe.pictures.end()) continue;
            dst.rc_complexity=it->rc_complexity;
            dst.rc_predicted_bits=it->rc_predicted_bits;
            dst.rc_target_bits=it->rc_target_bits;
            dst.rc_first_actual_bits=it->rc_first_actual_bits;
            dst.rc_predicted_q=it->rc_predicted_q;
            dst.rc_prediction_error_percent=it->rc_prediction_error_percent;
            dst.rc_retries=std::max(dst.rc_retries,it->rc_retries+extra_retry);
        }
    }

    void refresh_bounded_headers(EncodedGop& g,const libvc1::RateController& state_before) const {
        libvc1::RateController patch=state_before;
        for (auto& picture:g.pictures) {
            if (picture.kind==GopPictureKind::I)
                refresh_entry_point_fullness(picture,cfg,patch.fullness_code());
            patch.commit(static_cast<uint64_t>(picture.au.size())*8ull);
        }
    }

    std::optional<BoundedRetryResult> compute_bounded_retry(const EncodedGop& probe,
                                                               libvc1::RateController start_state,
                                                               int start_q) const {
        if (probe.retry_source.empty())
            throw std::runtime_error("libvc1 ABR rate-control retry lost its source GOP");

        // Recovery is confined to this GOP.  Candidate Q floors are independent
        // once the exact GOP-boundary VBV snapshot is known, so evaluate a
        // coarse set in parallel and then refine only the winning interval.
        // This uses the available CPU on the difficult scene itself instead of
        // encoding later scenes that cannot yet be written.
        auto attempt = [&](int minimum_q) -> std::optional<std::pair<EncodedGop,libvc1::RateController>> {
            libvc1::RateController trial=start_state;
            GopEncodeParams rp=base;
            apply_two_pass_plan(rp,probe.index,probe.start_frame,probe.retry_source.size());
            rp.previous_source=probe.previous_source;
            rp.cq_set=false;
            rp.bounded_cq=true;
            rp.bounded_start_q=std::max(start_q,minimum_q);
            rp.bounded_min_q=minimum_q;
            // Candidate GOPs themselves are the parallel work units.  Disable
            // nested B-analysis tasks here to avoid multiplying the requested
            // thread count during recovery.
            rp.intra_gop_parallelism=false;
            rp.intra_gop_workers=1;
            rp.include_sequence=(cfg.syntax==libvc1::StreamSyntax::Advanced &&
                                 param.b_emit_sequence_header && (probe.index==0 || cfg.bluray_compat));
            if (rp.include_sequence) rp.sequence=sequence;
            try {
                auto out=encode_gop_once(probe.index,probe.start_frame,probe.retry_source,std::move(rp),&trial,0);
                refresh_bounded_headers(out,start_state);
                return std::make_optional(std::make_pair(std::move(out),std::move(trial)));
            } catch (const BoundedCqExceeded&) {
                return std::nullopt;
            } catch (const std::runtime_error& ex) {
                const std::string_view what=ex.what();
                if (what.find("rate control cannot satisfy the selected HRD rate/buffer")!=std::string_view::npos)
                    return std::nullopt;
                throw;
            }
        };

        const int workers=std::max(1,param.i_threads);
        const int minimum_floor=std::clamp(base.bounded_min_q,1,31);
        if (workers==1) {
            // In the single-thread case an exact-state retry remains the cheapest
            // common recovery. With multiple workers this candidate is included
            // in the parallel floor wave below; running it synchronously first
            // used to serialize a complete difficult GOP and then repeat the
            // same failed floor in the coarse wave.
            if (auto direct=attempt(minimum_floor)) {
                auto out=std::move(direct->first);
                carry_probe_diagnostics(probe,out,1);
                const int q=std::clamp(out.bounded_end_q?out.bounded_end_q:start_q,1,31);
                return BoundedRetryResult(std::move(out),std::move(direct->second),q);
            }
            // Prove the coarsest candidate first, then binary-search downward.
            // This keeps explicit single-thread recovery bounded to O(log Q)
            // full-GOP attempts instead of up to 31 serial re-encodes.
            auto best=attempt(31);
            if (!best) return std::nullopt;
            int best_floor=31;
            int lo=1,hi=30;
            while (lo<=hi) {
                const int mid=lo+(hi-lo)/2;
                if (auto candidate=attempt(mid)) {
                    best_floor=mid;
                    best=std::move(candidate);
                    hi=mid-1;
                } else lo=mid+1;
            }
            auto out=std::move(best->first);
            carry_probe_diagnostics(probe,out,1);
            const int q=std::clamp(out.bounded_end_q?out.bounded_end_q:best_floor,1,31);
            return BoundedRetryResult(std::move(out),std::move(best->second),q);
        }

        auto run_wave = [&](const std::vector<int>& floors) {
            using Candidate=std::optional<std::pair<EncodedGop,libvc1::RateController>>;
            std::vector<std::future<Candidate>> futures;
            futures.reserve(floors.size());
            for (int floor:floors)
                futures.push_back(std::async(std::launch::async,[&,floor](){ return attempt(floor); }));
            std::vector<Candidate> results;
            results.reserve(futures.size());
            for (auto& f:futures) results.push_back(f.get());
            return results;
        };

        // Recovery candidates are independent full-GOP jobs.  The historical
        // eight-job ceiling left 24 of 32 hardware threads idle on large CPUs,
        // producing the characteristic ~25-35% utilization trough.  Spread the
        // legal Q-floor interval across all available workers (up to the 31
        // distinct VC-1 picture quantizers). On a 32-thread machine this becomes
        // one 31-way wave and normally eliminates the second refinement wave.
        const int floor_span=32-minimum_floor;
        const int coarse_count=std::min(floor_span,workers);
        std::vector<int> coarse_floors;
        coarse_floors.reserve(static_cast<size_t>(coarse_count));
        if (coarse_count==1) coarse_floors.push_back(minimum_floor);
        else {
            for (int i=0;i<coarse_count;++i) {
                const int q=minimum_floor+((31-minimum_floor)*i)/(coarse_count-1);
                if (coarse_floors.empty() || coarse_floors.back()!=q) coarse_floors.push_back(q);
            }
        }
        auto coarse=run_wave(coarse_floors);
        int pass_pos=-1;
        for (size_t i=0;i<coarse.size();++i) if (coarse[i]) { pass_pos=static_cast<int>(i); break; }
        if (pass_pos<0) return std::nullopt;

        int best_floor=coarse_floors[static_cast<size_t>(pass_pos)];
        std::optional<std::pair<EncodedGop,libvc1::RateController>> best=
            std::move(coarse[static_cast<size_t>(pass_pos)]);

        // If the coarse winner was not Q1, refine the gap in one second parallel
        // wave and choose the finest legal floor.  At most a handful of values
        // lie between adjacent coarse candidates.
        const int lower=(pass_pos==0)?minimum_floor:coarse_floors[static_cast<size_t>(pass_pos-1)]+1;
        if (best_floor>lower) {
            std::vector<int> refine_floors;
            for (int q=lower;q<best_floor;++q) refine_floors.push_back(q);
            auto refined=run_wave(refine_floors);
            for (size_t i=0;i<refined.size();++i) {
                if (refined[i]) {
                    best_floor=refine_floors[i];
                    best=std::move(refined[i]);
                    break;
                }
            }
        }

        if (!best) return std::nullopt;
        auto out=std::move(best->first);
        carry_probe_diagnostics(probe,out,1);
        const int q=std::clamp(out.bounded_end_q?out.bounded_end_q:best_floor,1,31);
        return BoundedRetryResult(std::move(out),std::move(best->second),q);
    }

    void start_bounded_retry(EncodedGop probe) {
        if (!bounded_rate_control || !probe.bounded_probe || probe.retry_source.empty())
            throw std::runtime_error("internal libvc1 ABR rate-control retry state mismatch");
        if (bounded_retry_pending)
            throw std::runtime_error("internal overlapping rate-control retries");

        auto pending_retry=std::make_unique<PendingBoundedRetry>(
            std::move(probe),*bounded_rate_control,bounded_current_q);
        const EncodedGop* probe_ptr=&pending_retry->probe;
        libvc1::RateController start_state=pending_retry->state_before;
        const int start_q=pending_retry->q_before;
        pending_retry->retry_future=std::async(std::launch::async,
            [this,probe_ptr,start_state=std::move(start_state),start_q]() mutable {
                return compute_bounded_retry(*probe_ptr,std::move(start_state),start_q);
            });
        bounded_retry_pending=std::move(pending_retry);
        // Do not encode past a difficult GOP.  Instead expose the already-final
        // output reservoir while all newly available CPU is spent recovering
        // this GOP.  This bounds memory and keeps third-party callers insulated
        // from out-of-order/hole semantics.
        release_bounded_recovery_reservoir();
    }

    struct HeldBoundedGop {
        EncodedGop encoded;
        std::vector<libvc1::Frame> source;
        libvc1::RateController state_before;
        int q_before=2;
    };
    // Only the last two finalized GOPs remain reversible, and they have never
    // been exposed through the API. Older held GOPs form a compressed output
    // reservoir and carry no raw source. This bounds rollback RAM to two GOPs.
    std::deque<HeldBoundedGop> bounded_history;
    static constexpr size_t kBoundedRecoveryHistoryGops=2;
    static constexpr size_t kBoundedOutputReservoirGops=4;
    static constexpr size_t kRecoveryBufferedInputGops=3;
    uint64_t gops_drained=0;

    std::optional<BoundedHistoryRecoveryResult> try_private_history_candidate(
            size_t depth,const EncodedGop& current_probe,int minimum_q) const {
        if (depth==0 || depth>kBoundedRecoveryHistoryGops || depth>bounded_history.size())
            return std::nullopt;
        const size_t begin=bounded_history.size()-depth;
        for (size_t i=begin;i<bounded_history.size();++i)
            if (bounded_history[i].source.empty()) return std::nullopt;

        libvc1::RateController trial=bounded_history[begin].state_before;
        int q=bounded_history[begin].q_before;
        std::vector<EncodedGop> outputs;
        std::vector<libvc1::RateController> states;
        std::vector<int> qs;
        outputs.reserve(depth+1); states.reserve(depth+1); qs.reserve(depth+1);

        auto one=[&](uint64_t index,uint64_t start_frame,const std::vector<libvc1::Frame>& source,
                       const std::shared_ptr<const libvc1::Frame>& previous_source)->bool {
            states.push_back(trial); qs.push_back(q);
            GopEncodeParams rp=base;
            apply_two_pass_plan(rp,index,start_frame,source.size());
            rp.previous_source=previous_source;
            rp.cq_set=false; rp.bounded_cq=true;
            rp.bounded_start_q=std::max(q,minimum_q);
            rp.bounded_min_q=minimum_q;
            // The history candidates are already parallel recovery work units.
            // Avoid nested oversubscription inside each candidate.
            rp.intra_gop_parallelism=false; rp.intra_gop_workers=1;
            rp.include_sequence=(cfg.syntax==libvc1::StreamSyntax::Advanced &&
                                 param.b_emit_sequence_header && (index==0 || cfg.bluray_compat));
            if (rp.include_sequence) rp.sequence=sequence;
            try {
                auto out=encode_gop_once(index,start_frame,source,std::move(rp),&trial,0);
                refresh_bounded_headers(out,states.back());
                q=std::clamp(out.bounded_end_q?out.bounded_end_q:q,1,31);
                outputs.push_back(std::move(out));
                return true;
            } catch (const BoundedCqExceeded&) {
                states.pop_back(); qs.pop_back(); return false;
            } catch (const std::runtime_error& ex) {
                if (std::string_view(ex.what()).find("rate control cannot satisfy the selected HRD rate/buffer")!=std::string_view::npos) {
                    states.pop_back(); qs.pop_back(); return false;
                }
                throw;
            }
        };

        for (size_t i=begin;i<bounded_history.size();++i)
            if (!one(bounded_history[i].encoded.index,bounded_history[i].encoded.start_frame,bounded_history[i].source,
                     bounded_history[i].encoded.previous_source))
                return std::nullopt;
        if (!one(current_probe.index,current_probe.start_frame,current_probe.retry_source,current_probe.previous_source)) return std::nullopt;
        return BoundedHistoryRecoveryResult(depth,std::move(outputs),std::move(states),std::move(qs),
                                            std::move(trial),q);
    }

    std::optional<BoundedHistoryRecoveryResult> compute_private_history_recovery(
            const EncodedGop& current_probe) const {
        const size_t max_depth=std::min(kBoundedRecoveryHistoryGops,bounded_history.size());
        if (!max_depth) return std::nullopt;
        const int workers=std::max(1,param.i_threads);

        // For one worker, first give the ordinary controller a cheap Q1 chance
        // from each private boundary. Multi-threaded recovery folds Q1 into the
        // parallel wave so it never begins with a complete serial suffix encode.
        if (workers==1) for (size_t depth=1;depth<=max_depth;++depth)
            if (auto r=try_private_history_candidate(depth,current_probe,1)) return r;

        auto search_depth=[&](size_t depth)->std::optional<BoundedHistoryRecoveryResult> {
            if (workers==1) {
                auto best=try_private_history_candidate(depth,current_probe,31);
                if (!best) return std::nullopt;
                int lo=2,hi=30;
                while (lo<=hi) {
                    const int mid=lo+(hi-lo)/2;
                    if (auto r=try_private_history_candidate(depth,current_probe,mid)) {
                        best=std::move(r); hi=mid-1;
                    } else lo=mid+1;
                }
                return best;
            }

            const int count=std::min(31,workers);
            std::vector<int> floors; floors.reserve(static_cast<size_t>(count));
            if (count==1) floors.push_back(1);
            else for (int i=0;i<count;++i) {
                const int q=1+(30*i)/(count-1);
                if (floors.empty() || floors.back()!=q) floors.push_back(q);
            }
            std::vector<std::future<std::optional<BoundedHistoryRecoveryResult>>> fs;
            fs.reserve(floors.size());
            for (int q:floors)
                fs.push_back(std::async(std::launch::async,[&,depth,q](){
                    return try_private_history_candidate(depth,current_probe,q);
                }));
            std::vector<std::optional<BoundedHistoryRecoveryResult>> results;
            results.reserve(fs.size());
            for (auto& f:fs) results.push_back(f.get());
            int pass=-1;
            for (size_t i=0;i<results.size();++i) if (results[i]) { pass=static_cast<int>(i); break; }
            if (pass<0) return std::nullopt;
            auto best=std::move(results[static_cast<size_t>(pass)]);
            const int best_floor=floors[static_cast<size_t>(pass)];
            const int lower=(pass==0)?1:floors[static_cast<size_t>(pass-1)]+1;
            if (best_floor>lower) {
                std::vector<std::future<std::optional<BoundedHistoryRecoveryResult>>> refine;
                for (int q=lower;q<best_floor;++q)
                    refine.push_back(std::async(std::launch::async,[&,depth,q](){
                        return try_private_history_candidate(depth,current_probe,q);
                    }));
                for (auto& f:refine) {
                    auto r=f.get();
                    if (r) { best=std::move(r); break; }
                }
            }
            return best;
        };

        // Prefer the shallowest rollback that can make the suffix legal.
        for (size_t depth=1;depth<=max_depth;++depth)
            if (auto r=search_depth(depth)) return r;
        return std::nullopt;
    }

    void aggregate(const EncodedGop& g) {
        stats.i_i_frames+=g.i_frames; stats.i_p_frames+=g.p_frames; stats.i_skipped_pictures+=g.skipped_pictures; stats.i_b_frames+=g.b_frames;
        stats.i_scene_i_frames+=g.scene_i_frames; stats.i_motion_failure_i_frames+=g.motion_failure_i_frames; stats.i_ic_p_frames+=g.ic_p_frames;
        stats.i_b_forward+=g.b_forward; stats.i_b_backward+=g.b_backward;
        stats.i_b_interpolated+=g.b_interpolated; stats.i_b_direct+=g.b_direct;
        stats.i_moved_macroblocks+=g.moved_macroblocks;
        stats.i_fractional_chroma_macroblocks+=g.fractional_chroma_macroblocks;
        stats.i_skipped_macroblocks+=g.skipped_macroblocks;
        stats.i_explicit_macroblocks+=g.explicit_macroblocks;
        stats.i_coded_macroblocks+=g.coded_macroblocks; stats.i_coded_blocks+=g.coded_blocks;
        stats.i_four_mv_macroblocks+=g.four_mv_macroblocks; stats.i_intra_macroblocks+=g.intra_macroblocks;
        stats.i_dquant_macroblocks+=g.dquant_macroblocks;
        stats.i_halfqp_frames+=g.halfqp_frames; stats.i_uniform_quantizer_frames+=g.uniform_quantizer_frames; stats.i_nonuniform_quantizer_frames+=g.nonuniform_quantizer_frames;
        for (size_t i=0;i<4;++i) stats.i_transform_parts[i]+=g.transform_parts[i];
        stats.i_q_sum+=g.q_sum;
        if (g.i_frames+g.p_frames+g.b_frames) {
            stats.i_q_min=std::min(stats.i_q_min,g.q_min_used);
            stats.i_q_max=std::max(stats.i_q_max,g.q_max_used);
        }
        stats.i_rate_total_bits+=g.rate_total_bits; stats.i_rate_frames+=g.rate_frames;
        stats.i_hrd_pauses+=g.hrd_pauses; stats.i_hrd_underflows+=g.hrd_underflows;
        if (gops_written==0) stats.i_hrd_init_bits=g.hrd_init_bits;
        if (g.rate_frames) {
            if (stats.i_gops==0 || g.hrd_min_after_bits<stats.f_hrd_min_after_bits)
                stats.f_hrd_min_after_bits=g.hrd_min_after_bits;
            stats.f_hrd_max_pre_bits=std::max(stats.f_hrd_max_pre_bits,g.hrd_max_pre_bits);
        }
        stats.i_frames+=g.i_frames+g.p_frames+g.b_frames;
        ++stats.i_gops;
    }

    void finalize_gop(EncodedGop&& g) {
        if (g.index!=gops_written) throw std::runtime_error("internal finalized GOP output-order mismatch");
        if (two_pass_budget) {
            double actual=0.0;
            for (const auto& pic:g.pictures) actual+=static_cast<double>(pic.au.size())*8.0;
            two_pass_budget->commit(g.index,actual);
            stats.i_rate_total_bits+=static_cast<uint64_t>(actual);
            stats.i_rate_frames+=g.pictures.size();
        }
        aggregate(g);
        for (auto& pic:g.pictures) {
            const uint64_t display=g.start_frame+pic.display_index;
            if (display>=metadata.size()) throw std::runtime_error("internal picture metadata mismatch");
            ReadyPicture r;
            r.global_display=display;
            r.coded_order=next_coded_order++;
            r.gop_index=g.index;
            r.gop_frames=static_cast<uint64_t>(g.pictures.size());
            r.pts=metadata[static_cast<size_t>(display)].pts;
            r.opaque=metadata[static_cast<size_t>(display)].opaque;
            r.pic=std::move(pic);
            ready.push_back(std::move(r));
        }
        ++gops_written;
    }

    void release_bounded_front() {
        if (bounded_history.empty()) return;
        auto held=std::move(bounded_history.front());
        bounded_history.pop_front();
        finalize_gop(std::move(held.encoded));
    }

    void release_bounded_recovery_reservoir() {
        // Keep only the two private rollback GOPs. Everything older is final and
        // may be returned while recovery works on the blocked suffix.
        while (bounded_history.size()>kBoundedRecoveryHistoryGops) release_bounded_front();
    }

    void hold_bounded_gop(EncodedGop g,EncodedGop probe,
                          libvc1::RateController state_before,int q_before) {
        if (param.i_two_pass!=0) {
            // Disk stats replace private source history; publish each accepted
            // GOP immediately rather than holding 2+ raw GOPs for rollback.
            g.retry_source.clear();probe.retry_source.clear();g.bounded_probe=false;
            ++gops_drained;
            finalize_gop(std::move(g));
            return;
        }
        std::vector<libvc1::Frame> source;
        if (!g.retry_source.empty()) source=std::move(g.retry_source);
        else source=std::move(probe.retry_source);
        g.retry_source.clear(); g.bounded_probe=false;
        bounded_history.push_back(HeldBoundedGop{
            std::move(g),std::move(source),std::move(state_before),q_before});
        ++gops_drained;

        // Raw rollback data is needed only for the two newest private GOPs.
        if (bounded_history.size()>kBoundedRecoveryHistoryGops) {
            const size_t cutoff=bounded_history.size()-kBoundedRecoveryHistoryGops;
            for (size_t i=0;i<cutoff;++i) {
                bounded_history[i].source.clear();
                bounded_history[i].source.shrink_to_fit();
            }
        }
        if (bounded_history.size()>kBoundedOutputReservoirGops) release_bounded_front();
    }

    bool harvest_bounded_retry(bool block) {
        if (!bounded_retry_pending) return false;

        if (bounded_retry_pending->phase==PendingBoundedRetry::Phase::GopRetry) {
            if (!block && bounded_retry_pending->retry_future.wait_for(std::chrono::seconds(0))!=std::future_status::ready)
                return false;
            auto result=bounded_retry_pending->retry_future.get();
            if (result) {
                EncodedGop probe=std::move(bounded_retry_pending->probe);
                libvc1::RateController state_before=bounded_retry_pending->state_before;
                const int q_before=bounded_retry_pending->q_before;
                EncodedGop g=std::move(result->encoded);
                *bounded_rate_control=std::move(result->final_state);
                bounded_current_q=result->final_q;
                bounded_retry_pending.reset();
                hold_bounded_gop(std::move(g),std::move(probe),std::move(state_before),q_before);
                pump_workers();
                return true;
            }

            if (param.i_two_pass!=0)
                throw std::runtime_error("two-pass VBV/HRD limit cannot be met by this GOP even at Q31; increase bitrate or buffer size");
            // Current-GOP-only recovery is mathematically impossible from this
            // boundary. Rewind only GOPs that are still private to libvc1; no AU
            // previously returned to the caller can ever change.
            bounded_retry_pending->phase=PendingBoundedRetry::Phase::PrivateHistoryRecovery;
            const EncodedGop* probe_ptr=&bounded_retry_pending->probe;
            bounded_retry_pending->history_future=std::async(std::launch::async,[this,probe_ptr](){
                return compute_private_history_recovery(*probe_ptr);
            });
            if (!block) return false;
        }

        if (!block && bounded_retry_pending->history_future.wait_for(std::chrono::seconds(0))!=std::future_status::ready)
            return false;
        auto recovery=bounded_retry_pending->history_future.get();
        if (!recovery)
            throw std::runtime_error("libvc1 ABR rate control cannot satisfy the selected bitrate/VBV ceiling even after the private two-GOP rollback window at Q31");
        if (recovery->depth==0 || recovery->outputs.size()!=recovery->depth+1 ||
            recovery->states_before.size()!=recovery->outputs.size() ||
            recovery->qs_before.size()!=recovery->outputs.size())
            throw std::runtime_error("internal private rollback recovery result mismatch");

        const size_t begin=bounded_history.size()-recovery->depth;
        for (size_t j=0;j<recovery->depth;++j) {
            auto& held=bounded_history[begin+j];
            held.encoded=std::move(recovery->outputs[j]);
            held.state_before=std::move(recovery->states_before[j]);
            held.q_before=recovery->qs_before[j];
        }
        EncodedGop g=std::move(recovery->outputs[recovery->depth]);
        libvc1::RateController state_before=std::move(recovery->states_before[recovery->depth]);
        const int q_before=recovery->qs_before[recovery->depth];
        *bounded_rate_control=std::move(recovery->final_state);
        bounded_current_q=recovery->final_q;
        EncodedGop probe=std::move(bounded_retry_pending->probe);
        bounded_retry_pending.reset();
        hold_bounded_gop(std::move(g),std::move(probe),std::move(state_before),q_before);
        pump_workers();
        return true;
    }

    void process_completed_probe(EncodedGop probe) {
        if (probe.index!=gops_drained) throw std::runtime_error("internal GOP output-order mismatch");

        if (base.bounded_cq && !(param.i_two_pass==2 && !param.b_bluray_compat)) {
            if (!bounded_rate_control || !probe.bounded_probe)
                throw std::runtime_error("internal libvc1 ABR rate-control GOP state mismatch");
            libvc1::RateController state_before=*bounded_rate_control;
            const int q_before=bounded_current_q;
            if (accept_bounded_probe(probe)) {
                hold_bounded_gop(std::move(probe),EncodedGop{},std::move(state_before),q_before);
            } else {
                // Do not perform a full GOP retry on the API thread. Difficult
                // sections use a current-GOP recovery worker; that worker fans
                // out Q-floor candidates while later GOP launching is paused.
                start_bounded_retry(std::move(probe));
            }
        } else {
            if (probe.index!=gops_written) throw std::runtime_error("internal GOP output-order mismatch");
            finalize_gop(std::move(probe));
            ++gops_drained;
        }
    }

    void harvest_workers() {
        // Complete an authoritative correction first if it is ready. Recovery
        // owns the launch barrier, so pump_workers() remains paused until this
        // advances gops_drained.
        harvest_bounded_retry(false);

        // Harvest every finished async job, not just the oldest one.  The old
        // scheduler left completed futures in `pending` while `ready` contained
        // output pictures; because pending was also the thread-count limiter,
        // workers eventually all went idle and encoding collapsed toward one
        // GOP/core at a time.  Completion and ordered VBV commit are separate
        // concerns: collect out of order, commit strictly in GOP order.
        for (auto it=pending.begin();it!=pending.end();) {
            if (it->future.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
                EncodedGop g=it->future.get();
                const uint64_t index=g.index;
                if (index!=it->index) throw std::runtime_error("internal worker GOP index mismatch");
                if (!completed.emplace(index,std::move(g)).second)
                    throw std::runtime_error("internal duplicate completed GOP");
                it=pending.erase(it);
            } else ++it;
        }
        // Refill newly freed worker slots during normal operation. During
        // recovery pump_workers() intentionally becomes a no-op.
        pump_workers();

        for (;;) {
            if (bounded_retry_pending) break;
            auto it=completed.find(gops_drained);
            if (it==completed.end()) break;
            EncodedGop g=std::move(it->second);
            completed.erase(it);
            process_completed_probe(std::move(g));
            // A commit may have released output/history but does not consume a
            // worker.  Keep the pool full if more input GOPs are queued.
            pump_workers();
        }
    }

    void drain_one() {
        harvest_workers();
        if (bounded_retry_pending) {
            // Explicit drain/flush waits for the ordered correction. Live input
            // may buffer only a small bounded number of future raw GOPs before
            // applying the same backpressure.
            harvest_bounded_retry(true);
            harvest_workers();
            return;
        }
        // If the next GOP is already completed, harvest_workers() processed it.
        // Otherwise block only on the exact GOP required by serialized output
        // order; other workers continue running while this wait occurs.
        auto it=std::find_if(pending.begin(),pending.end(),[&](const PendingGopFuture& pf) {
            return pf.index==gops_drained;
        });
        if (it==pending.end()) return;
        EncodedGop g=it->future.get();
        pending.erase(it);
        if (!completed.emplace(g.index,std::move(g)).second)
            throw std::runtime_error("internal duplicate completed GOP");
        pump_workers();
        harvest_workers();
    }

    int emit_one(vc1_au_t** pp_au,int* pi_au,vc1_picture_t* pic_out) {
        if (ready.empty()) {
            if (param.i_two_pass==1 && !pass1_complete && input_finished && pictures_output==frames_received && frames_received>0) {
                pass1_stats<<"END "<<pictures_output<<' '<<pass1_total_bits<<'\n';
                pass1_stats.flush();
                if (!pass1_stats) throw std::runtime_error("cannot finalize two-pass statistics");
                pass1_stats.close();pass1_complete=true;
            }
            *pp_au=nullptr; *pi_au=0; return 0;
        }
        current_output=std::move(ready.front()); ready.pop_front();
        current_au={};
        current_au.p_payload=current_output.pic.au.data();
        current_au.i_payload=current_output.pic.au.size();
        current_au.i_type=api_type(current_output.pic.kind);
        current_au.i_qp=current_output.pic.q;
        current_au.b_halfqp=current_output.pic.halfqp?1:0;
        current_au.i_quantizer_type=current_output.pic.quantizer_type==libvc1::QuantizerType::NonUniform?VC1_QUANTIZER_NONUNIFORM:VC1_QUANTIZER_UNIFORM;
        current_au.f_quant_step=current_output.pic.quant_step;
        current_au.b_keyframe=current_output.pic.key?1:0;
        current_au.b_skipped_picture=current_output.pic.skipped_picture?1:0;
        current_au.i_pts=current_output.pts;
        current_au.i_dts=static_cast<int64_t>(current_output.coded_order);
        current_au.i_display_order=current_output.global_display;
        current_au.i_coded_order=current_output.coded_order;
        current_au.f_rc_complexity=current_output.pic.rc_complexity;
        current_au.f_two_pass_gop_scale=current_output.pic.two_pass_budget_scale;
        current_au.f_two_pass_i_weight=current_output.pic.two_pass_i_weight;
        current_au.f_two_pass_p_weight=current_output.pic.two_pass_p_weight;
        current_au.f_two_pass_b_weight=current_output.pic.two_pass_b_weight;
        current_au.f_rc_predicted_bits=current_output.pic.rc_predicted_bits;
        current_au.f_rc_target_bits=current_output.pic.rc_target_bits;
        current_au.f_rc_allowed_bits=current_output.pic.rc_allowed_bits;
        current_au.f_rc_prediction_error_percent=current_output.pic.rc_prediction_error_percent;
        current_au.f_rc_vbv_before_bits=current_output.pic.rc_vbv_before_bits;
        current_au.f_rc_vbv_after_bits=current_output.pic.rc_vbv_after_bits;
        current_au.i_rc_first_actual_bits=current_output.pic.rc_first_actual_bits;
        current_au.i_rc_predicted_q=current_output.pic.rc_predicted_q;
        current_au.i_rc_retries=current_output.pic.rc_retries;
        current_au.b_rc_reencoded=current_output.pic.rc_retries>0?1:0;
        if (param.b_debug_macroblock_stats && !current_output.pic.macroblock_debug.empty()) {
            current_au.p_debug_macroblocks=current_output.pic.macroblock_debug.data();
            current_au.i_debug_macroblocks=current_output.pic.macroblock_debug.size();
        }
        if (param.b_debug_stats || param.i_two_pass!=0) {
            current_au.i_debug_gop_index=current_output.gop_index;
            current_au.i_debug_frame_in_gop=current_output.pic.display_index;
            current_au.i_debug_gop_frames=current_output.gop_frames;
        }
        if (param.b_debug_stats) {
            current_au.b_debug_scene_i=current_output.pic.debug_scene_i?1:0;
            current_au.b_debug_motion_failure_i=current_output.pic.debug_motion_failure_i?1:0;
            current_au.f_debug_qscale=current_output.pic.debug_qscale;
            current_au.f_debug_rc_planned_bits=current_output.pic.debug_rc_planned_bits;
            current_au.f_debug_gop_budget_scale=current_output.pic.debug_gop_budget_scale;
            current_au.f_debug_gop_difficulty=current_output.pic.debug_gop_difficulty;
            current_au.f_debug_motion_residual=current_output.pic.debug_motion_residual;
            current_au.f_debug_mean_mv_pixels=current_output.pic.debug_mean_mv_pixels;
            current_au.f_debug_max_mv_pixels=current_output.pic.debug_max_mv_pixels;
            current_au.i_debug_moved_macroblocks=current_output.pic.debug_moved_macroblocks;
            current_au.i_debug_fractional_chroma_macroblocks=current_output.pic.debug_fractional_chroma_macroblocks;
            current_au.i_debug_skipped_macroblocks=current_output.pic.debug_skipped_macroblocks;
            current_au.i_debug_explicit_macroblocks=current_output.pic.debug_explicit_macroblocks;
            current_au.i_debug_coded_macroblocks=current_output.pic.debug_coded_macroblocks;
            current_au.i_debug_coded_blocks=current_output.pic.debug_coded_blocks;
            current_au.i_debug_four_mv_macroblocks=current_output.pic.debug_four_mv_macroblocks;
            current_au.i_debug_intra_macroblocks=current_output.pic.debug_intra_macroblocks;
            current_au.i_debug_dquant_macroblocks=current_output.pic.debug_dquant_macroblocks;
            current_au.i_debug_mquant_min=current_output.pic.debug_mquant_min;
            current_au.i_debug_mquant_max=current_output.pic.debug_mquant_max;
            current_au.f_debug_mquant_mean=current_output.pic.debug_mquant_mean;
            for (size_t i=0;i<4;++i) current_au.i_debug_transform_parts[i]=current_output.pic.debug_transform_parts[i];
            current_au.b_debug_ttmbf=current_output.pic.debug_ttmbf?1:0;
            current_au.i_debug_ttfrm=current_output.pic.debug_ttfrm;
            current_au.b_debug_ttfrm_exact_checked=current_output.pic.debug_ttfrm_exact_checked?1:0;
            current_au.i_debug_acpred_macroblocks=current_output.pic.debug_acpred_macroblocks;
            current_au.i_debug_b_forward=current_output.pic.debug_b_forward;
            current_au.i_debug_b_backward=current_output.pic.debug_b_backward;
            current_au.i_debug_b_interpolated=current_output.pic.debug_b_interpolated;
            current_au.i_debug_b_direct=current_output.pic.debug_b_direct;
            current_au.b_debug_intensity_comp=current_output.pic.debug_intensity_comp?1:0;
            current_au.i_debug_encode_trials=current_output.pic.debug_encode_trials;
        }
        *pp_au=&current_au; *pi_au=1;
        if (pic_out) {
            vc1_picture_init(pic_out);
            pic_out->i_type=current_au.i_type;
            pic_out->i_qpplus1=current_output.pic.q+1;
            pic_out->b_keyframe=current_au.b_keyframe;
            pic_out->b_skipped_picture=current_au.b_skipped_picture;
            pic_out->i_pts=current_au.i_pts;
            pic_out->i_dts=current_au.i_dts;
            pic_out->i_display_order=current_au.i_display_order;
            pic_out->i_coded_order=current_au.i_coded_order;
            pic_out->opaque=current_output.opaque;
            if ((param.b_recon || param.b_debug_stats || param.b_debug_macroblock_stats) && !current_output.pic.reconstructed.y.empty()) {
                pic_out->img.i_plane=3;
                pic_out->img.plane[0]=current_output.pic.reconstructed.y.data();
                pic_out->img.plane[1]=current_output.pic.reconstructed.u.data();
                pic_out->img.plane[2]=current_output.pic.reconstructed.v.data();
                pic_out->img.i_stride[0]=cfg.width;
                pic_out->img.i_stride[1]=cfg.width/2;
                pic_out->img.i_stride[2]=cfg.width/2;
            }
            if (param.b_transform_info) {
                pic_out->prop.transform_map=current_output.pic.transform_map.data();
                pic_out->prop.transform_map_size=current_output.pic.transform_map.size();
            }
        }
        if (param.i_two_pass==1) {
            if (!pass1_stats || current_au.i_display_order>=source_fingerprints.size())
                throw std::runtime_error("two-pass statistics stream or frame index invalid");
            const uint64_t bits=static_cast<uint64_t>(current_au.i_payload)*8ull;
            if (bits>std::numeric_limits<uint64_t>::max()-pass1_total_bits)
                throw std::runtime_error("two-pass statistics bit count overflow");
            pass1_total_bits+=bits;
            pass1_stats<<"F "<<current_au.i_coded_order<<' '<<current_au.i_display_order<<' '
                <<current_output.gop_index<<' '
                <<(current_au.i_type==VC1_TYPE_I?'I':(current_au.i_type==VC1_TYPE_B?'B':'P'))<<' '
                <<bits<<' '<<current_au.i_qp<<' '<<std::setprecision(17)
                <<current_au.f_rc_complexity<<' '<<current_output.pic.debug_intra_macroblocks<<' '
                <<current_output.pic.debug_moved_macroblocks<<' '
                <<source_fingerprints.at(static_cast<size_t>(current_au.i_display_order))<<' '
                <<current_output.pic.pass1_mse_y<<' '<<current_output.pic.pass1_mse_uv<<'\n';
            if (!pass1_stats) throw std::runtime_error("write failed on two-pass statistics");
        }
        ++pictures_output;
        if (param.i_two_pass==1 && !pass1_complete && input_finished && pictures_output==frames_received) {
            pass1_stats<<"END "<<pictures_output<<' '<<pass1_total_bits<<'\n';
            pass1_stats.flush();
            if (!pass1_stats) throw std::runtime_error("cannot finalize two-pass statistics");
            pass1_stats.close();pass1_complete=true;
        }
        return static_cast<int>(current_au.i_payload);
    }
};
static thread_local std::string g_open_error;
extern "C" {
vc1_t *vc1_encoder_open(const vc1_param_t *input) {
    try {
        if (!input) throw std::runtime_error("null encoder parameters");
        if (input->i_struct_size < static_cast<int>(sizeof(vc1_param_t)) || input->i_api_version!=LIBVC1_API_VERSION)
            throw std::runtime_error("libvc1 parameter ABI mismatch");
        auto e=std::make_unique<vc1_t>(); e->param=*input;
        if (input->psz_two_pass_stats_file) {
            e->stats_path_owned=input->psz_two_pass_stats_file;
            e->param.psz_two_pass_stats_file=e->stats_path_owned.c_str();
        }
        auto& p=e->param;
        if (p.i_fps_num<=0 || p.i_fps_den<=0) throw std::runtime_error("invalid frame rate");
        if (p.i_profile!=VC1_PROFILE_ADVANCED && p.i_profile!=VC1_PROFILE_MAIN) throw std::runtime_error("unsupported VC-1 profile");
        const int coded_width=coded_dimension_for(p,p.i_width);
        const int coded_height=coded_dimension_for(p,p.i_height);
        const int advanced_level=validate_dimensions_and_level(p,coded_width,coded_height);
        if (p.i_scan_mode<VC1_SCAN_PROGRESSIVE || p.i_scan_mode>VC1_SCAN_INTERLACED_BFF)
            throw std::runtime_error("invalid scan mode");
        if (p.i_scan_mode!=VC1_SCAN_PROGRESSIVE && p.i_profile!=VC1_PROFILE_ADVANCED)
            throw std::runtime_error("interlaced scan modes require VC-1 Advanced Profile");
        if (p.b_bluray_compat!=0 && p.b_bluray_compat!=1)
            throw std::runtime_error("Blu-ray compatibility flag must be 0 or 1");
        validate_bluray_compat(p,advanced_level);
        if (p.i_threads<1 || p.i_threads>4096) throw std::runtime_error("threads must be 1..4096");
        if (p.b_intra_gop_parallelism!=0 && p.b_intra_gop_parallelism!=1) throw std::runtime_error("intra-GOP parallelism flag must be 0 or 1");
        if (p.b_fixed_gop_grid!=0 && p.b_fixed_gop_grid!=1) throw std::runtime_error("fixed GOP grid flag must be 0 or 1");
        libvc1::validate_compute_params(p); if (p.b_debug_stats!=0 && p.b_debug_stats!=1) throw std::runtime_error("debug statistics flag must be 0 or 1");
        if (p.b_debug_macroblock_stats!=0 && p.b_debug_macroblock_stats!=1) throw std::runtime_error("macroblock debug statistics flag must be 0 or 1");
        if (p.b_speed_profile!=0 && p.b_speed_profile!=1) throw std::runtime_error("speed profiling flag must be 0 or 1");
        if (p.b_debug_disable_p_intra!=0 && p.b_debug_disable_p_intra!=1) throw std::runtime_error("debug disable-P-intra flag must be 0 or 1");
        if (p.b_debug_disable_b_intra!=0 && p.b_debug_disable_b_intra!=1) throw std::runtime_error("debug disable-B-intra flag must be 0 or 1");
        if (p.b_dquant!=0 && p.b_dquant!=1) throw std::runtime_error("DQUANT flag must be 0 or 1");
        if (p.b_overlap!=0 && p.b_overlap!=1) throw std::runtime_error("OVERLAP flag must be 0 or 1");
        if (p.b_skip_identical_frames!=0 && p.b_skip_identical_frames!=1) throw std::runtime_error("identical-frame skip flag must be 0 or 1");
        if (p.i_bframes<0 || p.i_bframes>2) throw std::runtime_error("B-frame count must be 0..2");
        if (p.i_keyint_max<0) throw std::runtime_error("keyframe interval must be zero (auto) or positive");
        if (p.i_motion_search_range<0 || p.i_motion_search_range>1024) throw std::runtime_error("motion search range must be 0..1024");
        if (p.i_motion_local_search_range<0 || p.i_motion_local_search_range>1024) throw std::runtime_error("local motion search range must be 0..1024");
        if (!std::isfinite(p.f_distant_match_max_mae) || p.f_distant_match_max_mae<6.0 || p.f_distant_match_max_mae>255.0)
            throw std::runtime_error("distant-match maximum error must be finite and 6..255 luma MAE");
        if (p.i_me_quality<VC1_ME_SAD || p.i_me_quality>VC1_ME_RD) throw std::runtime_error("motion-estimation quality must be SAD, rate, SATD, or RD");
        // WMV3 / VC-1 Main Profile has an asymmetric maximum motion-vector
        // range: [-1024,1023.75] horizontally but only [-256,255.75]
        // vertically at Main@High. libvc1 exposes one symmetric integer-pixel
        // search radius, so cap Main at 255: a radius of 256 would conventionally
        // include +256, which is outside the legal positive vertical endpoint.
        // Keep Advanced Profile's 1024-pixel search request unchanged.
        if (p.i_profile==VC1_PROFILE_MAIN) {
            p.i_motion_search_range=std::min(p.i_motion_search_range,255);
            p.i_motion_local_search_range=std::min(p.i_motion_local_search_range,255);
        }
        if (p.i_long_range_search_mode<VC1_LONG_RANGE_COMPARE || p.i_long_range_search_mode>VC1_LONG_RANGE_LEGACY_DISTANT_FIRST)
            throw std::runtime_error("invalid long-range motion-search mode");
        if (!(p.f_scene_threshold>=0.0 && p.f_scene_threshold<=255.0)) throw std::runtime_error("scene threshold must be 0..255");
        if (!std::isfinite(p.f_scene_cut_min_interval) || p.f_scene_cut_min_interval<0.0 || p.f_scene_cut_min_interval>3600.0)
            throw std::runtime_error("scene cut interval must be finite and 0..3600 seconds");
        if (p.i_trellis<0 || p.i_trellis>2) throw std::runtime_error("trellis must be 0..2");
        if (!(p.f_aq_strength>=0.0 && p.f_aq_strength<=3.0) || !std::isfinite(p.f_aq_strength)) throw std::runtime_error("AQ strength must be finite and 0..3");
        if (p.i_qp_constant<1 || p.i_qp_constant>31) throw std::runtime_error("constant QP must be 1..31");
        if (p.b_qp_half!=0 && p.b_qp_half!=1) throw std::runtime_error("CQP HALFQP flag must be 0 or 1");
        if (p.b_halfqp!=0 && p.b_halfqp!=1) throw std::runtime_error("HALFQP enable flag must be 0 or 1");
        if (p.b_qp_half && (p.i_rc_method!=VC1_RC_CQP || p.i_qp_constant>8)) throw std::runtime_error("HALFQP CQP is legal only at PQINDEX 1..8");
        if (p.i_quantizer_type<VC1_QUANTIZER_AUTO || p.i_quantizer_type>VC1_QUANTIZER_NONUNIFORM) throw std::runtime_error("invalid quantizer type");
        if (p.b_rc_maximize!=0 && p.b_rc_maximize!=1) throw std::runtime_error("rate-control maximize flag must be 0 or 1");
        if (p.i_rc_method!=VC1_RC_ABR && p.i_rc_method!=VC1_RC_CQP) throw std::runtime_error("unsupported rate-control method");
        if (p.i_two_pass<0 || p.i_two_pass>2 || (p.i_two_pass &&
            (!p.psz_two_pass_stats_file || !*p.psz_two_pass_stats_file)))
            throw std::runtime_error("two-pass mode must be 0, 1, or 2 and requires a statistics filename");
        if (p.i_two_pass && p.i_rc_method!=VC1_RC_ABR)
            throw std::runtime_error("two-pass rate control requires ABR; --cq is not supported");
        if (p.i_peak_bitrate && p.i_rc_method!=VC1_RC_ABR)
            throw std::runtime_error("--max-bitrate requires bitrate-controlled mode, not --cq");
        if (p.i_two_pass==2 && !p.b_bluray_compat && p.i_peak_bitrate)
            throw std::runtime_error("non-Blu-ray second pass has unrestricted peak; --max-bitrate requires --bluray-compat");
        if (p.i_rc_method==VC1_RC_ABR && p.i_two_pass!=2 && p.i_peak_bitrate &&
            p.i_bitrate>p.i_peak_bitrate)
            throw std::runtime_error("average bitrate exceeds explicitly requested maximum bitrate");
        if ((p.b_two_pass_dynamic_weights!=0 && p.b_two_pass_dynamic_weights!=1) ||
            !std::isfinite(p.f_two_pass_dynamic_strength) ||
            p.f_two_pass_dynamic_strength<0.0 || p.f_two_pass_dynamic_strength>2.0)
            throw std::runtime_error("two-pass dynamic weights must be enabled/disabled and strength 0..2");
        auto valid_rc_weight=[](double v){ return std::isfinite(v) && v>=0.05 && v<=20.0; };
        if (!valid_rc_weight(p.f_rc_i_weight) || !valid_rc_weight(p.f_rc_p_weight) || !valid_rc_weight(p.f_rc_b_weight))
            throw std::runtime_error("I/P/B rate-control weights must be finite and in the range 0.05..20");
        if (!std::isfinite(p.f_rc_residual_threshold) || p.f_rc_residual_threshold<0.0 || p.f_rc_residual_threshold>255.0)
            throw std::runtime_error("residual-priority threshold must be finite and 0..255 luma MAE");
        if (!std::isfinite(p.f_rc_residual_width) || p.f_rc_residual_width<=0.0 || p.f_rc_residual_width>255.0)
            throw std::runtime_error("residual-priority width must be finite and >0..255 luma MAE");
        if (!std::isfinite(p.f_rc_residual_max_q_boost) || p.f_rc_residual_max_q_boost<0.0 || p.f_rc_residual_max_q_boost>12.0)
            throw std::runtime_error("residual-priority strength must be finite and 0..12 Q-index steps");
        if (!std::isfinite(p.f_inter_intra_threshold) || p.f_inter_intra_threshold<0.05 || p.f_inter_intra_threshold>2.0)
            throw std::runtime_error("inter/intra threshold must be finite and 0.05..2.0");
        if (p.i_rc_method==VC1_RC_ABR &&
            ((p.i_two_pass==2 && !p.b_bluray_compat) ? !p.i_bitrate :
             (p.i_bitrate<64000 || p.i_vbv_buffer_size<65536)))
            throw std::runtime_error("invalid bitrate/VBV buffer");
        // Advanced Profile bitrate/VBV limits are validated against the selected
        // SMPTE 421M level in validate_dimensions_and_level().  Do not retain
        // the older Blu-ray-only 40 Mbit/s / 30 Mbit guards here: AP@L4 permits
        // larger values and is required for the larger coded pictures supported
        // by this release.
        if (p.i_ac_mode<VC1_AC_AUTO || p.i_ac_mode>VC1_AC_ESC3 || p.i_ac_y_table<0 || p.i_ac_y_table>2 || p.i_ac_c_table<0 || p.i_ac_c_table>2)
            throw std::runtime_error("invalid AC entropy configuration");
        if (p.f_aq_strength==0.0) p.b_adaptive_quality=0;
        auto simd_request_valid=[](vc1_simd_e m) {
            return m==VC1_SIMD_AUTO || m==VC1_SIMD_NONE || m==VC1_SIMD_X86_64_V1 || m==VC1_SIMD_X86_64_V2 ||
                   m==VC1_SIMD_X86_64_V3 || m==VC1_SIMD_X86_64_V4 ||
                   m==VC1_SIMD_PRESCOTT || m==VC1_SIMD_CONROE ||
                   m==VC1_SIMD_PENRYN || m==VC1_SIMD_SANDYBRIDGE || m==VC1_SIMD_K10 ||
                   m==VC1_SIMD_BULLDOZER || m==VC1_SIMD_PILEDRIVER || m==VC1_SIMD_AVX2_PARTIAL;
        };
        for (int i=0;i<VC1_SIMD_PRIMITIVE_COUNT;++i) {
            const vc1_simd_e m=p.i_simd_primitive[i];
            if (!simd_request_valid(m))
                throw std::runtime_error(std::string("invalid SIMD override for primitive ")+vc1_simd_primitive_name(static_cast<vc1_simd_primitive_e>(i)));
        }
        if (!simd_request_valid(p.i_simd)) throw std::runtime_error("invalid SIMD mode");
        if (p.b_simd_benchmark_all!=0 && p.b_simd_benchmark_all!=1) throw std::runtime_error("SIMD benchmark-all flag must be 0 or 1");
        libvc1::SimdTier selected=libvc1::SimdTier::None;
        libvc1::SimdDispatch dispatch{};
        if (p.i_simd==VC1_SIMD_AUTO) {
            e->simd_benchmark=libvc1::auto_select_simd(p.b_simd_fma!=0,p.b_simd_benchmark_all!=0);
            selected=e->simd_benchmark.tier;
            dispatch=e->simd_benchmark.dispatch;
        } else if (p.i_simd==VC1_SIMD_NONE || p.i_simd==VC1_SIMD_SCALAR) {
            selected=libvc1::SimdTier::None;
            dispatch.tier.fill(selected);
        } else {
            const auto requested=internal_simd(p.i_simd);
            if (!libvc1::simd_target_implemented(requested))
                throw std::runtime_error(std::string("SIMD target ")+libvc1::simd_target_name(requested)+" is reserved for a future kernel implementation");
            if (!libvc1::simd_target_available(requested))
                throw std::runtime_error(std::string("SIMD target ")+libvc1::simd_target_name(requested)+" requested but unavailable on this CPU/build");
            selected=requested;
            dispatch.tier.fill(selected);
        }
        auto primitive_fma_capable=[](size_t i){
            const auto primitive=static_cast<libvc1::SimdPrimitive>(i);
            return primitive==libvc1::SimdPrimitive::ForwardIntra8 || primitive==libvc1::SimdPrimitive::ForwardResidual8 || primitive==libvc1::SimdPrimitive::ForwardResidualRect;
        };
        if (p.i_simd!=VC1_SIMD_AUTO && p.b_simd_fma && libvc1::simd_target_fma_capable(selected)) {
            for (size_t i=0;i<libvc1::kSimdPrimitiveCount;++i)
                dispatch.fma[i]=primitive_fma_capable(i);
        }
        for (size_t i=0;i<libvc1::kSimdPrimitiveCount;++i) {
            const vc1_simd_e ov=p.i_simd_primitive[i];
            if (ov==VC1_SIMD_AUTO) continue;
            if (ov==VC1_SIMD_NONE || ov==VC1_SIMD_SCALAR) {
                dispatch.tier[i]=libvc1::SimdTier::None;
                dispatch.fma[i]=false;
                continue;
            }
            const auto target=internal_simd(ov);
            if (!libvc1::simd_target_implemented(target))
                throw std::runtime_error(std::string("SIMD target ")+libvc1::simd_target_name(target)+" is reserved for a future kernel implementation for primitive "+vc1_simd_primitive_name(static_cast<vc1_simd_primitive_e>(i)));
            if (!libvc1::simd_target_available(target))
                throw std::runtime_error(std::string("SIMD target ")+libvc1::simd_target_name(target)+" override requested but unavailable for primitive "+vc1_simd_primitive_name(static_cast<vc1_simd_primitive_e>(i)));
            dispatch.tier[i]=target;
            dispatch.fma[i]=p.b_simd_fma && libvc1::simd_target_fma_capable(target) && primitive_fma_capable(i);
        }
        selected=dispatch.tier[0];
        for (const auto t:dispatch.tier) if (t!=selected) { selected=libvc1::SimdTier::Mixed; break; }

        if (p.i_two_pass==2) {
            e->two_pass=vc1_twopass::read_plan(p.psz_two_pass_stats_file,p,
                  p.b_two_pass_dynamic_weights?p.f_two_pass_dynamic_strength:0.0);
            e->two_pass_budget=std::make_unique<vc1_twopass::BudgetLedger>(
                e->two_pass,p.i_bitrate,p.i_fps_num,p.i_fps_den);
        }
        if (p.i_two_pass==1) {
            e->pass1_stats.open(p.psz_two_pass_stats_file,std::ios::binary|std::ios::trunc);
            if (!e->pass1_stats) throw std::runtime_error("cannot create two-pass statistics file");
            e->pass1_stats<<"LIBVC1_TWO_PASS 2 "<<vc1_twopass::signature(p)<<'\n';
            if (!e->pass1_stats) throw std::runtime_error("cannot write two-pass statistics header");
        }
        e->cfg.width=coded_width; e->cfg.height=coded_height;
        e->cfg.display_width=p.i_width; e->cfg.display_height=p.i_height;
        e->cfg.advanced_level=advanced_level; e->cfg.bluray_compat=p.b_bluray_compat!=0; e->cfg.fps={p.i_fps_num,p.i_fps_den};
        const bool bounded_cq=p.i_rc_method==VC1_RC_ABR;
        e->cfg.pqindex=p.i_qp_constant;
        e->cfg.halfqp=(p.i_rc_method==VC1_RC_CQP && p.b_qp_half!=0);
        e->cfg.allow_halfqp=p.b_halfqp!=0;
        e->cfg.quantizer_type=p.i_quantizer_type==VC1_QUANTIZER_NONUNIFORM?libvc1::QuantizerType::NonUniform:(p.i_quantizer_type==VC1_QUANTIZER_UNIFORM?libvc1::QuantizerType::Uniform:libvc1::QuantizerType::Auto);
        e->cfg.ac_coding=p.b_ac_coding!=0; e->cfg.ac_mode=internal_ac_mode(p.i_ac_mode);
        e->cfg.ac_y_table_index=p.i_ac_y_table; e->cfg.ac_c_table_index=p.i_ac_c_table;
        e->cfg.motion_search_range=p.i_motion_search_range; e->cfg.motion_local_search_range=std::min(p.i_motion_local_search_range,p.i_motion_search_range); e->cfg.me_quality=p.i_me_quality; e->cfg.long_range_search_mode=p.i_long_range_search_mode; e->cfg.distant_match_max_mae=p.f_distant_match_max_mae; e->cfg.scene_threshold=p.f_scene_threshold;
        e->cfg.fade_compensation=p.b_fade_compensation!=0; e->cfg.variable_transforms=p.b_variable_transforms!=0;
        e->cfg.dquant=(p.b_dquant!=0);
        e->cfg.loop_filter=p.b_loop_filter!=0; e->cfg.overlap=p.b_overlap!=0; e->cfg.trellis=p.i_trellis;
        e->cfg.adaptive_quality=p.b_adaptive_quality!=0; e->cfg.aq_strength=p.f_aq_strength;
        e->cfg.rc_residual_threshold=p.f_rc_residual_threshold; e->cfg.rc_residual_width=p.f_rc_residual_width;
        e->cfg.rc_residual_max_q_boost=p.f_rc_residual_max_q_boost; e->cfg.inter_intra_threshold=p.f_inter_intra_threshold;
        e->cfg.track_transform_map=p.b_transform_info!=0; e->cfg.simd_dispatch=dispatch;
        e->cfg.syntax=p.i_profile==VC1_PROFILE_MAIN?libvc1::StreamSyntax::Wmv9Main:libvc1::StreamSyntax::Advanced;
        e->cfg.scan_mode=p.i_scan_mode; libvc1::configure_compute_backend(e->cfg,e->stats,p);
        e->cfg.max_b_frames=p.b_intra_only?0:p.i_bframes;
        e->cfg.skip_identical_frames=(p.b_skip_identical_frames!=0 && e->cfg.syntax==libvc1::StreamSyntax::Advanced);
        e->cfg.debug_macroblock_stats=(p.b_debug_macroblock_stats!=0);
        if (p.b_speed_profile) e->cfg.speed_profiler=std::make_shared<libvc1::SpeedProfiler>();
        e->cfg.debug_disable_p_intra=(p.b_debug_disable_p_intra!=0);
        e->cfg.debug_disable_b_intra=(p.b_debug_disable_b_intra!=0);

        libvc1::HrdValue rate{},buffer{};
        if (p.i_rc_method==VC1_RC_ABR) {
            if (p.i_two_pass==2 && !p.b_bluray_compat)
                rate.represented=p.i_bitrate; // No HRD syntax, hence no HRD representability ceiling.
            else
                rate=libvc1::hrd_floor(resolved_peak_bitrate(p),6);
            if (p.i_two_pass!=2 || p.b_bluray_compat)
                buffer=libvc1::hrd_floor(p.i_vbv_buffer_size,4);
            // A non-Blu-ray offline pass has an average-size objective, not
            // a hypothetical HRD peak/buffer. Do not signal HRD constraints
            // that its output deliberately does not satisfy.
            e->cfg.hrd_enabled=p.i_profile==VC1_PROFILE_ADVANCED &&
                (p.i_two_pass!=2 || p.b_bluray_compat);
            e->cfg.hrd_bit_rate_exponent=rate.exponent; e->cfg.hrd_buffer_size_exponent=buffer.exponent;
            e->cfg.hrd_rate=rate.mantissa; e->cfg.hrd_buffer=buffer.mantissa;
        }
        libvc1::Vc1Encoder header_encoder(e->cfg); e->sequence=header_encoder.sequence_header();
        e->header_au.p_payload=e->sequence.data(); e->header_au.i_payload=e->sequence.size();
        e->header_au.i_type=VC1_TYPE_I; e->header_au.b_keyframe=1;
        e->keyint=resolved_keyint(p);
        if (p.f_scene_cut_min_interval<=0.0) e->scene_cut_min_frames=0;
        else {
            const long double frames=static_cast<long double>(p.f_scene_cut_min_interval)*
                static_cast<long double>(p.i_fps_num)/static_cast<long double>(p.i_fps_den);
            e->scene_cut_min_frames=static_cast<uint64_t>(std::max<long double>(1.0L,std::ceil(frames-1e-12L)));
        }
        e->base.cfg=e->cfg; e->base.two_pass_mode=p.i_two_pass;
        e->base.cq_set=p.i_rc_method==VC1_RC_CQP;
        e->base.bounded_cq=bounded_cq;
        e->base.bounded_maximize=bounded_cq && p.b_rc_maximize;
        e->base.cq_value=p.i_qp_constant; e->base.bounded_preferred_q=p.i_qp_constant;
        e->base.scene_cut=p.b_scene_cut!=0; e->base.intra_only=p.b_intra_only!=0; e->base.debug_stats=p.b_debug_stats!=0; e->base.debug_macroblock_stats=p.b_debug_macroblock_stats!=0;
        e->base.bframes=p.b_intra_only?0:p.i_bframes;
        e->base.intra_gop_parallelism=p.b_intra_gop_parallelism!=0;
        e->base.intra_gop_workers=e->base.intra_gop_parallelism?std::max(1,std::min(p.i_threads,1+std::max(0,e->base.bframes))):1;
        e->base.hrd_rate_bits=rate.represented;
        e->base.target_rate_bits=(p.i_two_pass!=0 || p.i_peak_bitrate)?p.i_bitrate:rate.represented;
        e->base.hrd_buffer_bits=buffer.represented;
        e->base.rc_i_weight=p.f_rc_i_weight; e->base.rc_p_weight=p.f_rc_p_weight; e->base.rc_b_weight=p.f_rc_b_weight;
        if (bounded_cq && (p.i_two_pass!=2 || p.b_bluray_compat)) {
            e->bounded_rate_control=std::make_unique<libvc1::RateController>(rate.represented,buffer.represented,e->cfg.fps);
            e->bounded_current_q=std::clamp(p.i_qp_constant,1,31);
            e->base.bounded_start_q=e->bounded_current_q;
        }
        e->base.keep_reconstruction=p.b_recon || p.b_transform_info || p.b_debug_stats || p.b_debug_macroblock_stats;
        e->current_gop.reserve(static_cast<size_t>(e->keyint)); e->metadata.reserve(static_cast<size_t>(e->keyint)*2);
        e->stats.i_q_min=32; e->stats.i_q_max=0;
        e->stats.i_hrd_rate_bits=p.i_two_pass==2 && !p.b_bluray_compat?0:rate.represented;
        e->stats.i_hrd_buffer_bits=buffer.represented;
        e->stats.i_simd_selected=api_simd(selected);
        e->stats.f_simd_scalar_units_per_second=e->simd_benchmark.scalar_units_per_second;
        e->stats.f_simd_v1_units_per_second=e->simd_benchmark.v1_units_per_second;
        e->stats.f_simd_v2_units_per_second=e->simd_benchmark.v2_units_per_second;
        e->stats.f_simd_v3_units_per_second=e->simd_benchmark.v3_units_per_second;
        e->stats.f_simd_v4_units_per_second=e->simd_benchmark.v4_units_per_second;
        for (size_t i=0;i<libvc1::kSimdPrimitiveCount;++i) {
            e->stats.i_simd_primitive_selected[i]=api_simd(dispatch.tier[i]);
            e->stats.b_simd_primitive_fma_selected[i]=dispatch.fma[i]?1:0;
            for (int t=0;t<VC1_SIMD_BENCH_TIER_COUNT;++t)
                e->stats.f_simd_primitive_units_per_second[i][t]=e->simd_benchmark.primitive_units_per_second[i][t];
        }
        return e.release();
    } catch (const std::exception& ex) { g_open_error=ex.what(); return nullptr; }
}

int vc1_encoder_headers(vc1_t *e,vc1_au_t **pp_au,int *pi_au) {
    if (!e || !pp_au || !pi_au) return -1;
    *pp_au=&e->header_au; *pi_au=1;
    return static_cast<int>(e->header_au.i_payload);
}

int vc1_encoder_encode(vc1_t *e,vc1_au_t **pp_au,int *pi_au,vc1_picture_t *pic_out,const vc1_picture_t *pic_in) {
    if (!e || !pp_au || !pi_au) return -1;
    try {
        e->last_error.clear();
        if (pic_in) {
            // While recovering a difficult GOP, accept at most a small bounded
            // amount of later raw input but do not encode it.  The output
            // reservoir and this input allowance are the same size, so a normal
            // caller can keep receiving finalized AUs while recovery runs
            // without turning later source frames into an unbounded RAM queue.
            if (e->bounded_retry_pending && e->queued.size()>=vc1_t::kRecoveryBufferedInputGops) {
                e->harvest_bounded_retry(true);
                e->harvest_workers();
            }
            if (pic_in->i_type==VC1_TYPE_P || pic_in->i_type==VC1_TYPE_B) throw std::runtime_error("only AUTO or forced-I input picture types are currently accepted");
            if (pic_in->i_type==VC1_TYPE_I && !e->current_gop.empty()) e->process_dynamic_boundaries(true);
            if (e->current_gop.empty()) { e->current_gop_start=e->frames_received; e->current_gop_starts_scene_i=false; e->current_gop_starts_motion_failure_i=false; }
            auto source=copy_input_picture(e->param,*pic_in);
            if (e->param.i_two_pass!=0) {
                const uint64_t fingerprint=vc1_twopass::source_hash(source.y,source.u,source.v);
                if (e->param.i_two_pass==1) e->source_fingerprints.push_back(fingerprint);
                else if (e->frames_received>=e->two_pass.by_display.size() ||
                         fingerprint!=e->two_pass.by_display[static_cast<size_t>(e->frames_received)].hash)
                    throw std::runtime_error("pass 2 source frame differs from pass 1 or contains extra frames");
            }
            e->current_gop.push_back(std::move(source));
            e->metadata.push_back({pic_in->i_pts,pic_in->opaque}); ++e->frames_received;
            if (e->current_gop.size()>=e->keyint) e->process_dynamic_boundaries(false);
            // Keep harvesting completed GOPs even while output is buffered.
            // Otherwise finished futures occupy all worker slots and CPU usage
            // collapses while the caller drains `ready` one picture at a time.
            e->harvest_workers();
            if (e->ready.empty() && e->pending.size()>=e->gop_worker_limit()) e->drain_one();
        } else {
            e->input_finished=true;
            if (e->param.i_two_pass==2 && e->frames_received!=e->two_pass.by_display.size())
                throw std::runtime_error("pass 2 frame count differs from pass 1");
            e->process_dynamic_boundaries(true);
            e->plan_window(true);
            e->pump_workers();
            // EOF does not mean the GOP workers are finished.  Keep harvesting
            // completed futures even while older pictures are buffered in
            // `ready`; otherwise the flush loop can spend dozens of calls
            // emitting buffered AUs while completed futures occupy every worker
            // slot and no queued GOP can start.  That reproduces the same
            // one-core collapse fixed for the input path in 0.1.35.
            e->harvest_workers();
            while (e->ready.empty()) {
                e->harvest_workers();
                if (!e->ready.empty()) break;
                if (e->bounded_retry_pending) {
                    e->drain_one();
                    continue;
                }
                if (!e->pending.empty()) {
                    e->drain_one();
                    continue;
                }
                if (!e->bounded_history.empty()) {
                    e->release_bounded_front();
                    continue;
                }
                break;
            }
        }
        return e->emit_one(pp_au,pi_au,pic_out);
    } catch (const std::exception& ex) { e->last_error=ex.what(); *pp_au=nullptr; *pi_au=0; return -1; }
}

int vc1_encoder_delayed_frames(vc1_t *e) {
    if (!e) return 0;
    const uint64_t n=e->frames_received-e->pictures_output;
    return n>static_cast<uint64_t>(INT32_MAX)?INT32_MAX:static_cast<int>(n);
}

int vc1_encoder_parameters(vc1_t *e,vc1_param_t *p) {
    if (!e || !p) return -1;
    *p=e->param; p->i_simd=e->stats.i_simd_selected; p->i_compute_backend=e->stats.i_compute_selected; p->i_compute_device=e->stats.i_compute_device;
    return 0;
}

int vc1_encoder_stats(vc1_t *e,vc1_stats_t *s) {
    if (!e || !s) return -1;
    *s=e->stats;
    if (e->cfg.speed_profiler) {
        for (int i=0;i<VC1_SPEED_PROFILE_OPERATION_COUNT;++i) {
            s->i_speed_profile_calls[i]=e->cfg.speed_profiler->calls[static_cast<size_t>(i)].load(std::memory_order_relaxed);
            s->i_speed_profile_nanoseconds[i]=e->cfg.speed_profiler->nanoseconds[static_cast<size_t>(i)].load(std::memory_order_relaxed);
        }
    } libvc1::update_compute_stats(e->cfg,*s);
    return 0;
}
const char *vc1_encoder_last_error(vc1_t *e) { return e?e->last_error.c_str():g_open_error.c_str(); }
void vc1_encoder_close(vc1_t *e) { delete e; }

} // extern C
