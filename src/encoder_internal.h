#pragma once
#include <libvc1.h>

#include <algorithm>
#include <atomic>
#include <array>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <future>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "vc1_entropy_tables.h"
#include "vc1_groupa_tables.h"
#include "simd.h"
#include "vulkan_compute.h"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace libvc1 {

struct Rational { int64_t num = 0, den = 1; };

class BitWriter {
public:
    void bit(bool v) {
        ++total_bits_;
        cur_ = static_cast<uint8_t>((cur_ << 1) | (v ? 1 : 0));
        if (++bits_ == 8) { data_.push_back(cur_); cur_ = 0; bits_ = 0; }
    }
    void bits(uint64_t v, int n) {
        if (n < 0 || n > 64) throw std::runtime_error("invalid bit count");
        for (int i = n - 1; i >= 0; --i) bit((v >> i) & 1u);
    }
    void vlc(uint32_t code, int n) { bits(code, n); }
    size_t bit_count() const { return total_bits_; }
    void append(const BitWriter& other);
    std::vector<uint8_t> finish_rbdu();
    std::vector<uint8_t> finish_raw();
private:
    std::vector<uint8_t> data_;
    uint8_t cur_ = 0;
    int bits_ = 0;
    size_t total_bits_ = 0;
};

std::vector<uint8_t> escape_ebdu(const std::vector<uint8_t>& in);
std::vector<uint8_t> bdu(uint8_t suffix,std::vector<uint8_t> rbdu);

struct Frame {
    int width = 0, height = 0;
    std::vector<uint8_t> y, u, v;
};


enum class AcMode { Auto, Vlc, Esc3 };
enum class QuantizerType { Auto, Uniform, NonUniform };
enum class StreamSyntax { Advanced, Wmv9Main };
enum class SimdTier { None=0, Mixed=1, X86V1=2, X86V2=3, X86V3=4, X86V4=5,
                      Prescott=10, Conroe=11, Penryn=12, SandyBridge=13,
                      K10=14, Bulldozer=15, Piledriver=16, Avx2Partial=17 };
// Source compatibility for older internal code while the user-facing name is "none".
inline constexpr SimdTier kSimdScalar = SimdTier::None;

enum class SimdPrimitive : uint8_t {
    FrameSad=0, BlockSad, ForwardIntra8, ForwardResidual8, Inverse8x8,
    PutBlock8, AddBlockRect, LumaQpelSad, LumaBilinearSad, LumaMc,
    LumaMcAvg, ChromaMc, ChromaMcAvg, IntensityLuma, IntensityChroma,
    ForwardResidualRect, Quantize, PerceptualStats, SumU8, Satd4x4, Count
};
inline constexpr size_t kSimdPrimitiveCount=static_cast<size_t>(SimdPrimitive::Count);

struct SimdDispatch {
    std::array<SimdTier,kSimdPrimitiveCount> tier{};
    std::array<bool,kSimdPrimitiveCount> fma{};
    SimdDispatch() { tier.fill(SimdTier::None); fma.fill(false); }
    SimdTier get(SimdPrimitive p) const { return tier[static_cast<size_t>(p)]; }
    bool use_fma(SimdPrimitive p) const { return fma[static_cast<size_t>(p)]; }
};


bool cpu_has_x86_64_v1();
bool cpu_has_x86_64_v2();
bool cpu_has_x86_64_v3();
bool cpu_has_x86_64_v4();
bool cpu_has_prescott();
bool cpu_has_conroe();
bool cpu_has_penryn();
bool cpu_has_sandybridge();
bool cpu_has_k10();
bool cpu_has_bulldozer();
bool cpu_has_piledriver();
bool cpu_has_avx2_partial();
const char* simd_target_name(SimdTier tier);
bool simd_target_implemented(SimdTier tier);
bool simd_target_available(SimdTier tier);
bool simd_target_auto_eligible(SimdTier tier,bool benchmark_all);
bool simd_target_fma_capable(SimdTier tier);
int simd_target_benchmark_slot(SimdTier tier);

struct SimdBenchmarkResult {
    SimdTier tier=SimdTier::None;
    SimdDispatch dispatch{};
    /* Per-primitive throughput uses the public VC1_SIMD_BENCH_* target slots.
       Slots are stable for generic v1/v2/v3/v4 plus the intermediate feature-band
       targets, so benchmark reports remain ABI-addressable by target. */
    std::array<std::array<double,VC1_SIMD_BENCH_TARGET_COUNT>,kSimdPrimitiveCount> primitive_units_per_second{};
    double scalar_units_per_second=0.0;
    double v1_units_per_second=0.0;
    double v2_units_per_second=0.0;
    double v3_units_per_second=0.0;
    double v4_units_per_second=0.0;
};

const char* simd_primitive_name(SimdPrimitive primitive);

/* Strict throughput winner in target-registry order; exact ties retain the earlier target. */
SimdTier choose_fastest_simd_target(const std::array<double,VC1_SIMD_BENCH_TARGET_COUNT>& rates);
SimdBenchmarkResult auto_select_simd(bool use_fma,bool benchmark_all=false);

struct HrdValue {
    uint8_t exponent=0;
    uint16_t mantissa=0;
    uint64_t represented=0;
};

HrdValue hrd_floor(uint64_t requested,int exponent_bias);

class RateController {
public:
    RateController(uint64_t rate_bits,uint64_t buffer_bits,Rational fps);
    double fullness() const;
    double frame_budget() const;
    double max_picture_bits() const;
    uint64_t rate_bits() const;
    uint64_t buffer_bits() const;
    uint64_t initial_fullness_bits_floor() const;
    uint8_t fullness_code() const;
    void commit(uint64_t picture_bits);
    uint64_t transmission_pauses() const;
    uint64_t underflows() const;
    uint64_t total_bits() const;
    uint64_t frames() const;
    double min_after_bits() const;
    double max_pre_bits() const;
    double average_bitrate() const;
private:
    uint64_t pre_removal_units() const;
    uint64_t rate_bits_=0,buffer_bits_=0;
    Rational fps_{};
    uint64_t buffer_units_=0,refill_units_=0,midpoint_units_=0;
    uint64_t initial_fullness_units_=0,after_units_=0;
    uint64_t transmission_pauses_=0;
    uint64_t underflows_=0;
    uint64_t min_after_units_=std::numeric_limits<uint64_t>::max();
    uint64_t max_pre_units_=0;
    uint64_t total_bits_=0,frames_=0;
    bool first_picture_=true;
};

enum class SpeedProfileOp : uint8_t {
    GopEncode=VC1_SPEED_GOP_ENCODE, SceneDetection=VC1_SPEED_SCENE_DETECTION,
    RateControl=VC1_SPEED_RATE_CONTROL, IPicture=VC1_SPEED_I_PICTURE,
    PPicture=VC1_SPEED_P_PICTURE, BPicture=VC1_SPEED_B_PICTURE,
    PAnalysis=VC1_SPEED_P_ANALYSIS, BAnalysis=VC1_SPEED_B_ANALYSIS,
    MotionLocalSearch=VC1_SPEED_MOTION_LOCAL_SEARCH, MotionLongRange=VC1_SPEED_MOTION_LONG_RANGE,
    MotionSubpel=VC1_SPEED_MOTION_SUBPEL,
    IntensityComp=VC1_SPEED_INTENSITY_COMP, MotionComp=VC1_SPEED_MOTION_COMP,
    TransformRdo=VC1_SPEED_TRANSFORM_RDO, AdaptiveQuantization=VC1_SPEED_ADAPTIVE_QUANTIZATION,
    DQuant=VC1_SPEED_DQUANT, Trellis=VC1_SPEED_TRELLIS, Entropy=VC1_SPEED_ENTROPY,
    Overlap=VC1_SPEED_OVERLAP, LoopFilter=VC1_SPEED_LOOP_FILTER,
    MacroblockDebug=VC1_SPEED_MACROBLOCK_DEBUG
};

struct SpeedProfiler {
    std::array<std::atomic<uint64_t>,VC1_SPEED_PROFILE_OPERATION_COUNT> calls{};
    std::array<std::atomic<uint64_t>,VC1_SPEED_PROFILE_OPERATION_COUNT> nanoseconds{};
    void add(SpeedProfileOp op,uint64_t ns) {
        const size_t i=static_cast<size_t>(op);
        calls[i].fetch_add(1,std::memory_order_relaxed);
        nanoseconds[i].fetch_add(ns,std::memory_order_relaxed);
    }
};

class SpeedProfileScope {
public:
    SpeedProfileScope(const std::shared_ptr<SpeedProfiler>& profiler,SpeedProfileOp op)
        : profiler_(profiler.get()),op_(op) { if(profiler_) start_=std::chrono::steady_clock::now(); }
    ~SpeedProfileScope() {
        if(!profiler_) return;
        const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start_).count();
        profiler_->add(op_,static_cast<uint64_t>(std::max<int64_t>(0,ns)));
    }
    SpeedProfileScope(const SpeedProfileScope&)=delete;
    SpeedProfileScope& operator=(const SpeedProfileScope&)=delete;
private:
    SpeedProfiler* profiler_=nullptr;
    SpeedProfileOp op_;
    std::chrono::steady_clock::time_point start_{};
};

struct EncoderConfig {
    // libvc1 keeps an even internal 4:2:0 coding raster. Advanced Profile may
    // expose a smaller odd presentation raster through DISPLAY_EXT; Simple/Main
    // dimensions are carried out-of-band by their container/transport.
    int width = 0;
    int height = 0;
    int display_width = 0;
    int display_height = 0;
    int advanced_level = 3; // Preserve AP@L3 unless AP@L4 is actually required.
    bool bluray_compat = false;
    int visible_width() const { return display_width>0 ? display_width : width; }
    int visible_height() const { return display_height>0 ? display_height : height; }
    int visible_chroma_width() const { return (visible_width()+1)/2; }
    int visible_chroma_height() const { return (visible_height()+1)/2; }
    Rational fps{24,1};
    int pqindex = 9; // Picture PQINDEX, 1..31.
    bool halfqp = false; // Legal only for PQINDEX <= 8 and picture-PQUANT blocks.
    bool allow_halfqp = true; // ABR policy; picture config stores the resolved HALFQP bit.
    QuantizerType quantizer_type = QuantizerType::Auto; // Resolved to uniform/nonuniform before picture coding.
    bool ac_coding = true;
    AcMode ac_mode = AcMode::Auto;
    int ac_y_table_index = 0; // Used by AcMode::Vlc; VC-1 decode012 index 0..2.
    int ac_c_table_index = 0;
    int motion_search_range = 1024;
    int motion_local_search_range = 32;
    vc1_me_quality_e me_quality = VC1_ME_SATD;
    vc1_long_range_search_mode_e long_range_search_mode = VC1_LONG_RANGE_COMPARE;
    double distant_match_max_mae = 255.0;
    double scene_threshold = 28.0;
    bool fade_compensation = true;
    double fade_min_gain = 0.04; // minimum luma prediction-MAD improvement to signal intensity compensation
    bool variable_transforms = true;
    bool extended_transform_signaling = true; // TTFRM/TTBLK enabled for CQP and modeled ABR/VBV.
    bool dquant = true; // VC-1 frame-selectable macroblock quantizer signaling/RDO (Main/Advanced).
    bool loop_filter = true;
    bool overlap = true; // VC-1 overlap transform smoothing; CONDOVER at Advanced PQ<=8.
    int trellis = 1; // 0=scalar, 1=fast bounded trellis, 2=wider high-quality trellis.
    bool adaptive_quality = true; // perceptual block RDO and bounded ABR adaptation.
    double aq_strength = 1.0;
    bool aq_predictable_texture = false; // per-picture internal ABR/AQ hint; not public ABI.
    // Internal ABR macroblock-class allocation. The picture controller supplies
    // the analyzed average block weight for this picture; DQUANT then moves
    // intra/inter macroblocks around the selected picture PQUANT so the same
    // I/P/B relative weights apply inside mixed P/B pictures.
    bool rc_block_weighting = false;
    double rc_picture_block_weight = 1.0;
    double rc_intra_block_weight = 1.0;
    double rc_inter_block_weight = 1.0;
    // Smooth residual-priority allocation for temporal P/B macroblocks.
    double rc_residual_threshold = 8.0;
    double rc_residual_width = 24.0;
    double rc_residual_max_q_boost = 4.0;
    // Mean luma post-prediction absolute error for the current P/B picture.
    // Retained for diagnostics/context while the user-facing protection ramp
    // is expressed directly in absolute luma MAE units.
    double rc_prediction_residual_mean = 0.0;
    // Spatial-vs-temporal proxy ratio used by P/B intra macroblock selection.
    double inter_intra_threshold = 0.80;
    int vulkan_min_batch = 8;
    int vulkan_min_batch_sad = 8;
    int vulkan_min_batch_satd = 8;
    std::shared_ptr<VulkanComputeContext> vulkan_compute;
    bool track_transform_map = false;
    bool debug_macroblock_stats = false;
    std::shared_ptr<SpeedProfiler> speed_profiler;
    SimdDispatch simd_dispatch{};
    StreamSyntax syntax = StreamSyntax::Advanced;
    vc1_scan_mode_e scan_mode = VC1_SCAN_PROGRESSIVE;
    bool interlaced() const { return scan_mode != VC1_SCAN_PROGRESSIVE; }
    bool top_field_first() const { return scan_mode != VC1_SCAN_INTERLACED_BFF; }
    int max_b_frames = 2;
    bool skip_identical_frames = true; // Advanced PTYPE=Skipped for byte-identical source repeats.
    bool debug_disable_p_intra = false; // Diagnostic only: suppress P-picture whole-MB intra selection.
    bool debug_disable_b_intra = false; // Diagnostic only: suppress B-picture intra selection/BI promotion.
    bool rndctrl = false; // decoder state for Simple/Main Profile; ignored by Advanced Profile.
    SimdTier simd_for(SimdPrimitive p) const { return simd_dispatch.get(p); }
    bool simd_fma_for(SimdPrimitive p) const { return simd_dispatch.use_fma(p); }

    // Advanced-profile HRD signaling. One leaky bucket is enough because the
    // encoder knows the selected peak rate. Values are the already-encoded
    // ST 421 mantissa/exponent fields, so the exact represented values can
    // also drive the rate controller.
    bool hrd_enabled = false;
    uint8_t hrd_bit_rate_exponent = 0;
    uint8_t hrd_buffer_size_exponent = 0;
    uint16_t hrd_rate = 0;
    uint16_t hrd_buffer = 0;
};

struct ComputeBenchmarkResult {
    bool ran = false;
    int sad_min_batch = 0;   // 0 means keep SAD on CPU in AUTO mode.
    int satd_min_batch = 0;  // 0 means keep SATD on CPU in AUTO mode.
    double cpu_sad_candidates_per_second = 0.0;
    double vulkan_sad_candidates_per_second = 0.0;
    double cpu_satd_candidates_per_second = 0.0;
    double vulkan_satd_candidates_per_second = 0.0;
};

ComputeBenchmarkResult benchmark_vulkan_motion_compute(const EncoderConfig& cfg,
                                                        VulkanComputeContext& vk,
                                                        int user_min_batch);
void validate_compute_params(const vc1_param_t& p);
void configure_compute_backend(EncoderConfig& cfg,vc1_stats_t& stats,const vc1_param_t& p);
void update_compute_stats(const EncoderConfig& cfg,vc1_stats_t& stats);

class Vc1Encoder {
public:
    struct MotionVector { int xq=0, yq=0; }; // quarter-pixel units; decoder predictor state always uses qpel units.
    enum class ProgressiveMvMode : uint8_t { OneMvQpel=0, OneMvHpel=1, OneMvHpelBilinear=2, MixedMv=3 };
    static bool mv_mode_halfpel(ProgressiveMvMode m);
    static bool mv_mode_bilinear(ProgressiveMvMode m);
    static bool mv_mode_mixed(ProgressiveMvMode m);
    struct IntensityComp {
        bool enabled=false;
        uint8_t lumscale=32;
        uint8_t lumshift=0;
    };
    struct PAnalysis {
        std::vector<MotionVector> mvs; // decoder block-0/collocated MV for each MB (1-MV value when not mixed)
        std::vector<std::array<MotionVector,4>> block_mvs; // decoder-equivalent four-luma-block MV field
        std::vector<uint8_t> use_4mv; // progressive Mixed-MV / MVTYPEMB decision
        std::vector<uint8_t> use_intra; // whole-MB P-intra decision; mutually exclusive with 4-MV
        std::vector<uint64_t> screen_sads; // per-MB best SAD discovered before rate/SATD/RD finalist selection
        std::vector<double> inter_intra_cost_ratio; // (spatial proxy + guard) / temporal proxy; -1 when unavailable
        std::vector<double> distant_local_mae;
        std::vector<double> distant_candidate_mae;
        std::vector<uint8_t> distant_match_decision; // vc1_distant_match_decision_e
        double mean_abs_residual=0.0;
        // Temporal prediction complexity kept separate from the cheap intra-mode
        // decision proxy. ABR replaces this with an exact reference-Q dry-run
        // complexity whenever P/B-intra is actually selected.
        double rate_complexity=0.0;
        size_t moved_macroblocks=0;
        size_t fractional_chroma_macroblocks=0;
        size_t four_mv_macroblocks=0;
        size_t intra_macroblocks=0;
        IntensityComp intensity;
        ProgressiveMvMode mv_mode=ProgressiveMvMode::OneMvQpel;
        uint64_t motion_sad=0;
        uint64_t motion_bits=0;
        std::shared_ptr<Frame> compensated_reference;
    };
    enum class TransformType : uint8_t { T8x8=1, T8x4=2, T4x8=3, T4x4=4 };
    enum class TransformSignalLevel : uint8_t { Macroblock=0, Block=1, Frame=2 };
    struct TransformPicturePlan {
        bool frame_level=false;
        TransformType frame_type=TransformType::T8x8;
        // Set for B pictures when the sampled TTFRM candidate was checked
        // against exact whole-picture local/frame-level RDO.
        bool exact_checked=false;
    };
    struct PEncodeResult {
        std::vector<uint8_t> data;
        std::vector<vc1_mb_debug_t> macroblock_debug;
        Frame reconstructed;
        Frame padded_reconstructed; // complete coded macroblock raster for reference/deblock correctness
        std::vector<uint8_t> transform_map; // packed type + skip mask for four luma 8x8 parents/MB
        size_t skipped_macroblocks=0;
        size_t explicit_macroblocks=0;
        size_t coded_macroblocks=0;
        size_t coded_blocks=0;
        size_t four_mv_macroblocks=0;
        size_t intra_macroblocks=0;
        size_t acpred_macroblocks=0;
        size_t dquant_macroblocks=0;
        uint64_t mquant_sum=0;
        size_t mquant_samples=0;
        int mquant_min=32;
        int mquant_max=0;
        std::array<size_t,4> transform_parts{};
        bool frame_level_transform=false;
        TransformType frame_transform_type=TransformType::T8x8;
        bool frame_transform_exact_checked=false;
        // Experimental interlace-field diagnostics used by focused regressions.
        size_t field_same_macroblocks=0;
        size_t field_opposite_macroblocks=0;
        size_t field_moved_macroblocks=0;
    };
    enum class BMbMode : uint8_t { Forward, Backward, Interpolated, Direct, Intra };
    struct BAnalysis {
        std::vector<BMbMode> modes;
        // Decoder state contains both temporal MV fields for every B macroblock.
        // For a single-direction MB the unused field contains its direct-scaled
        // collocated vector, exactly as ff_vc1_pred_b_mv initializes it.
        std::vector<MotionVector> forward_mvs;
        std::vector<MotionVector> backward_mvs;
        std::vector<double> inter_intra_cost_ratio; // (spatial proxy + guard) / temporal proxy; -1 when unavailable
        std::vector<double> forward_distant_local_mae, forward_distant_candidate_mae;
        std::vector<uint8_t> forward_distant_match_decision;
        std::vector<double> backward_distant_local_mae, backward_distant_candidate_mae;
        std::vector<uint8_t> backward_distant_match_decision;
        size_t forward_macroblocks=0;
        size_t backward_macroblocks=0;
        size_t interpolated_macroblocks=0;
        size_t direct_macroblocks=0;
        size_t intra_macroblocks=0;
        size_t moved_macroblocks=0;
        size_t fractional_chroma_macroblocks=0;
        double mean_abs_residual=0.0;
        double rate_complexity=0.0;
        IntensityComp forward_intensity;
        ProgressiveMvMode mv_mode=ProgressiveMvMode::OneMvQpel;
        uint64_t motion_sad=0;
        uint64_t motion_bits=0;
        std::shared_ptr<Frame> compensated_past;
    };
    struct TransformRateEstimate {
        double legacy_transform_bits=0.0;
        double extended_transform_bits=0.0;
        double aq_extended_transform_bits=0.0;
        size_t sampled_inter_macroblocks=0;
        size_t total_inter_macroblocks=0;
        bool frame_level=false;
        TransformType frame_type=TransformType::T8x8;
    };
    struct QuantizerRateEstimate {
        QuantizerType selected=QuantizerType::Uniform;
        double uniform_bits=0.0;
        double nonuniform_bits=0.0;
        double rate_scale=1.0;
        size_t sampled_blocks=0;
    };
    struct IEncodeStats {
        std::vector<vc1_mb_debug_t> macroblock_debug;
        size_t acpred_macroblocks=0;
        size_t dquant_macroblocks=0;
        uint64_t mquant_sum=0;
        size_t mquant_samples=0;
        int mquant_min=32;
        int mquant_max=0;
    };

    struct BEncodeResult {
        std::vector<uint8_t> data;
        std::vector<vc1_mb_debug_t> macroblock_debug;
        Frame reconstructed;
        Frame padded_reconstructed;
        std::vector<uint8_t> transform_map;
        size_t skipped_macroblocks=0;
        size_t explicit_macroblocks=0;
        size_t coded_macroblocks=0;
        size_t coded_blocks=0;
        size_t dquant_macroblocks=0;
        uint64_t mquant_sum=0;
        size_t mquant_samples=0;
        int mquant_min=32;
        int mquant_max=0;
        std::array<size_t,4> transform_parts{};
        bool frame_level_transform=false;
        TransformType frame_transform_type=TransformType::T8x8;
        bool frame_transform_exact_checked=false;
        size_t forward_macroblocks=0;
        size_t backward_macroblocks=0;
        size_t interpolated_macroblocks=0;
        size_t direct_macroblocks=0;
        size_t intra_macroblocks=0;
        size_t acpred_macroblocks=0;
        bool bi_picture=false;
        // Experimental interlace-field diagnostics used by focused regressions.
        size_t field_forward_same_macroblocks=0;
        size_t field_forward_opposite_macroblocks=0;
        size_t field_backward_same_macroblocks=0;
        size_t field_backward_opposite_macroblocks=0;
        size_t field_moved_macroblocks=0;
    };

    explicit Vc1Encoder(EncoderConfig c);

    std::vector<uint8_t> sequence_header() const;

    std::vector<uint8_t> entry_point(uint8_t hrd_fullness=127) const;

    std::vector<uint8_t> encode_skipped_picture() const;

    std::vector<uint8_t> encode_i_picture(const Frame& f, IEncodeStats* stats=nullptr, bool bi_picture=false,
                                          const Frame* previous_source=nullptr) const;

    // Reconstruct an I picture exactly as the decoder does.  P-picture motion
    // estimation must reference this quantized/reconstructed picture rather
    // than the source input; otherwise encoder and decoder references diverge
    // after the first lossy frame and prediction drift accumulates visibly.
    Frame reconstruct_i_picture_padded(const Frame& f) const;

    Frame reconstruct_i_picture(const Frame& f) const;

    Frame crop_reconstructed_picture(const Frame& padded) const;

    void fill_macroblock_quality(std::vector<vc1_mb_debug_t>& stats,const Frame& source,
                                 const Frame& reconstructed,const Frame* previous_source,
                                 const Frame* prediction=nullptr,
                                 const std::vector<uint8_t>* prediction_valid=nullptr) const;

    // Analyse a P candidate against the immediately preceding reference picture.
    // Group B retains the robust integer search as a seed, then performs decoder-
    // equivalent half/quarter-pixel refinement and progressive Mixed-MV decisions.
    PAnalysis analyze_p_picture_core(const Frame& f, const Frame& ref, ProgressiveMvMode mode=ProgressiveMvMode::OneMvQpel, bool allow_intra=true, const PAnalysis* shared_seed=nullptr) const;

    static uint8_t intensity_map_y(uint8_t v, const IntensityComp& ic);

    static uint8_t intensity_map_uv(uint8_t v, const IntensityComp& ic);

    Frame intensity_compensated_reference(const Frame& ref, const IntensityComp& ic) const;

    IntensityComp estimate_intensity_comp(const Frame& f, const Frame& ref,
                                          const std::vector<MotionVector>& mvs) const;

    uint64_t estimate_p_motion_bits(const PAnalysis& a,bool intensity) const;

    long double sampled_p_mode_rd(const Frame& f,const Frame& ref,const PAnalysis& a,uint64_t motion_bits,Frame& pred,
                                  const Vc1Encoder& neutral_eval) const;

    TransformPicturePlan choose_p_transform_plan(const Frame& f,const Frame& ref,const PAnalysis& a,
                                                   int coding_set,bool use_vlc,Frame* pred_scratch=nullptr) const;

    PAnalysis analyze_p_picture(const Frame& f, const Frame& ref, const PAnalysis* qpel_seed=nullptr) const;

    PEncodeResult encode_p_picture(const Frame& f, const Frame& ref, const PAnalysis& a,
                                   const Frame* padded_ref=nullptr, bool ref_field_picture=false,
                                   const Frame* previous_source=nullptr) const;

    // Advanced-Profile interlace-field path. P field pictures signal NUMREF=1
    // and therefore expose both legal field references. P/B field macroblocks
    // carry real 8x8 inter residuals; field motion vectors remain zero in this
    // first source-correct field implementation.
    PEncodeResult encode_p_interlaced_fields(const Frame& f, const Frame& ref,
                                               bool ref_field_picture=false,
                                               const Frame* previous_source=nullptr) const;

    // Scene-cut score used before B-picture reordering is planned.  A genuine
    // fade is not a scene cut: when enabled, test the best legal intensity
    // compensation around the global-motion match before classifying it.
    double scene_change_score(const Frame& f, const Frame& ref) const;

    bool has_good_motion_match(const Frame& f, const Frame& ref) const;

    // Advanced-profile B pictures keep independent forward/backward motion
    // fields. Group B reuses the same qpel-refined searches for forward/backward
    // candidates, then performs syntax-aware per-MB direct/F/B/bi decisions.
    BAnalysis analyze_b_picture_mode(const Frame& f, const Frame& past, const Frame& future,
                                const std::vector<MotionVector>& future_anchor_mvs,
                                const std::vector<uint8_t>& future_anchor_4mv,
                                const IntensityComp& forward_ic,
                                int fraction_num, int fraction_den, ProgressiveMvMode mode) const;


    long double sampled_b_mode_rd(const Frame& f,const Frame& past,const Frame& future,const BAnalysis& a,Frame& pred,
                                  const Vc1Encoder& neutral_eval) const;


    TransformPicturePlan choose_b_transform_plan(const Frame& f,const Frame& past,const Frame& future,
                                                   const BAnalysis& a,int coding_set,bool use_vlc,Frame* pred_scratch=nullptr) const;

    BAnalysis analyze_b_picture(const Frame& f, const Frame& past, const Frame& future,
                                const std::vector<MotionVector>& future_anchor_mvs,
                                const std::vector<uint8_t>& future_anchor_4mv,
                                const IntensityComp& forward_ic,
                                int fraction_num, int fraction_den) const;

    BEncodeResult encode_b_picture(const Frame& f, const Frame& past, const Frame& future,
                                   const BAnalysis& a, int fraction_num, int fraction_den,
                                   const Frame* padded_past=nullptr, const Frame* padded_future=nullptr,
                                   bool past_field_picture=false, bool future_field_picture=false,
                                   const Frame* previous_source=nullptr) const;

    BEncodeResult encode_b_interlaced_fields(const Frame& f, const Frame& past, const Frame& future,
                                              int fraction_num, int fraction_den,
                                              bool past_field_picture=false, bool future_field_picture=false,
                                              const Frame* previous_source=nullptr) const;

    TransformRateEstimate estimate_p_transform_rate(const Frame& f,const Frame& ref,const PAnalysis& a) const;

    TransformRateEstimate estimate_b_transform_rate(const Frame& f,const Frame& past,const Frame& future,
                                                     const BAnalysis& a) const;

    QuantizerRateEstimate estimate_picture_quantizer_rate(const Frame& f) const;

    QuantizerType choose_picture_quantizer_type(const Frame& f) const;

    bool strong_contextual_aq_picture(const Frame& f) const;

public: // internal helper surface for split implementation units
    struct PredictorInfo {
        MotionVector pre{}, a{}, c{};
        bool a_valid=false, c_valid=false, hybrid=false;
    };

    struct BitplaneChoice {
        BitWriter syntax;
        bool raw=true;
        int imode=0;
        size_t total_bits=0;
    };

    enum class OverlapMode : uint8_t { None=0, All=1, Select=2 };
    struct OverlapPlan {
        OverlapMode mode=OverlapMode::None;
        std::vector<uint8_t> flags;
        BitplaneChoice bitplane;
    };
    struct SignedFrame {
        int width=0,height=0;
        std::vector<int> y,u,v;
    };

    OverlapPlan choose_i_overlap_plan(const Frame& f) const;
    SignedFrame reconstruct_i_signed_padded(const Frame& f) const;
    void apply_i_overlap(SignedFrame& f,const OverlapPlan& plan) const;
    void apply_p_overlap(SignedFrame& f,const std::vector<uint8_t>& intra,int mbw,int mbh) const;
    int intra_recon_bias() const;

    static void encode_rowskip_bits(BitWriter& b, const std::vector<uint8_t>& p, int w, int h);
    static void encode_colskip_bits(BitWriter& b, const std::vector<uint8_t>& p, int w, int h);
    static void encode_norm2_bits(BitWriter& b, const std::vector<uint8_t>& p);
    static void encode_norm6_bits(BitWriter& b, const std::vector<uint8_t>& p, int w, int h);
    static BitplaneChoice choose_bitplane(const std::vector<uint8_t>& plane, int w, int h);

    struct MvDataSyntax {
        int dx=0,dy=0;
        bool more=false;
        bool intra=false;
        bool halfpel=false;
    };
    static MvDataSyntax mvdata_for_mode(MotionVector desired,MotionVector pred,
                                        ProgressiveMvMode mode,bool more=false,bool intra=false);
    static int p_mv_mode_index(ProgressiveMvMode mode,int pq,bool secondary=false);
    static int unary_mode_bits(int index,int maximum);
    static void write_unary_mode(BitWriter& b,int index,int maximum);
    static int mv_cat(int v,bool halfpel=false);
    static int mv_symbol(const MvDataSyntax& m);
    static int mv_component_bits(int cat,bool halfpel);
    int mv_suffix_bits(const MvDataSyntax& m) const;
    static void write_mv_component(BitWriter& b,int v,int cat,bool halfpel);
    void write_mvdata_table(BitWriter& b,int table,const MvDataSyntax& m) const;
    int choose_mv_table(const std::vector<MvDataSyntax>& values) const;
    static int choose_cbp_table(const std::vector<uint8_t>& cbps);
    static void write_cbp_table(BitWriter& b,int table,uint8_t cbp);
    static int choose_inter_ac_index(double residual,int pqindex);

    static bool same_mv(MotionVector a, MotionVector b);
    struct MotionSearchBounds { int xmin=0,xmax=0,ymin=0,ymax=0; };
    MotionSearchBounds motion_search_bounds_px(int requested) const;
    MotionSearchBounds motion_search_bounds_qpel(int requested) const;
    int motion_mvrange() const;
    bool extended_mv_enabled() const;
    static int mvrange_x_qpel(int mvrange);
    static int mvrange_y_qpel(int mvrange);
    static int mvrange_kx(int mvrange);
    static int mvrange_ky(int mvrange);
    void write_mvrange(BitWriter& b) const;
    static int median3(int a,int b,int c);
    static int round_qpel_to_pixel(int q);
    static int modular_component_distance(int a,int b,int range);
    int modular_mv_distance(MotionVector a,MotionVector b) const;

    PredictorInfo predictor_info(const std::vector<MotionVector>& mv,int mbw,int mbh,int mx,int my) const;

    PredictorInfo predictor_info_mixed_1mv(const std::vector<std::array<MotionVector,4>>& field,
                                           const std::vector<uint8_t>& mb_intra,
                                           int mbw,int mbh,int mx,int my) const;

    PredictorInfo predictor_info_4mv(const std::vector<std::array<MotionVector,4>>& field,
                                     const std::vector<uint8_t>& mb_intra,
                                     int mbw,int mbh,int mx,int my,int n) const;

    MotionVector predictor_b_main(const std::vector<MotionVector>& mv,int mbw,int mx,int my) const;

    struct GlobalMotionResult { int dx=0, dy=0; uint64_t full_sad=0; };

    uint64_t frame_sad(const Frame& cur,const Frame& ref,int dx,int dy,int stride) const;

    struct IntegerMotionResult { int dx=0,dy=0; uint64_t full_sad=0; };

    struct LongRangeSignature {
        std::array<uint8_t,4> ydc{}; // four luma 8x8 DC/mean samples
        uint8_t udc=128;
        uint8_t vdc=128;
    };
    struct LongRangeSignatureCell {
        int x=0,y=0;
        LongRangeSignature sig;
    };
    struct LongRangeIndex {
        std::vector<LongRangeSignatureCell> cells;
    };

    static uint8_t mean_region(const std::vector<uint8_t>& plane,int pw,int ph,
                               int x0,int y0,int bw,int bh);

    LongRangeSignature long_range_signature(const Frame& f,int x0,int y0) const;

    static unsigned long_range_signature_distance(const LongRangeSignature& a,
                                                   const LongRangeSignature& b);

    LongRangeIndex build_long_range_index(const Frame& ref) const;

    uint64_t active_mb_pixels(int mx,int my) const;

    bool motion_match_good(uint64_t sad,int mx,int my) const;

    int distant_match_decision(uint64_t candidate_sad,uint64_t local_sad,int mx,int my) const;

    IntegerMotionResult propagated_long_range_motion(const Frame& cur,const Frame& ref,
                                                      int mx,int my,int mbw,
                                                      const std::vector<MotionVector>& field,
                                                      const std::vector<uint8_t>& propagated,
                                                      int requested_range) const;

    IntegerMotionResult extended_content_motion(const Frame& cur,const Frame& ref,
                                                 int mx,int my,int requested_range,
                                                 const LongRangeIndex& idx,
                                                 const IntegerMotionResult& local,
                                                 uint64_t* distant_candidate_sad=nullptr,
                                                 int* distant_decision=nullptr) const;

    IntegerMotionResult integer_motion_umh(const Frame& cur,const Frame& ref,int mx,int my,
                                           const PredictorInfo& pi,const GlobalMotionResult& global,
                                           int range) const;

    GlobalMotionResult global_motion_search_extended(const Frame& cur,const Frame& ref,int range) const;

    GlobalMotionResult global_motion_search(const Frame& cur,const Frame& ref,int range) const;

    uint64_t mb_sad(const Frame& cur,const Frame& ref,int mx,int my,int dx,int dy,int stride) const;

    uint64_t block_sad_qpel(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,MotionVector mv) const;

    uint64_t block_sad_motion(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                              MotionVector mv,ProgressiveMvMode mode) const;
    uint64_t block_satd_motion(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                               MotionVector mv,ProgressiveMvMode mode) const;
    bool vulkan_motion_costs(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                             const std::vector<MotionVector>& candidates,int sample_stride,
                             ProgressiveMvMode mode,bool satd,std::vector<uint64_t>& costs) const;
    void predict_luma_motion_block(uint8_t* dst,int dst_stride,const Frame& ref,
                                   int x0,int y0,int bw,int bh,MotionVector mv,
                                   ProgressiveMvMode mode) const;
    uint64_t motion_vector_rate_bits(MotionVector desired,MotionVector pred,ProgressiveMvMode mode,bool more) const;
    MotionVector refine_motion_block_staged(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                                            MotionVector seed,int range,ProgressiveMvMode mode,const PredictorInfo& pi,
                                            bool more,Frame* rd_pred,uint64_t* best_sad=nullptr,
                                            uint64_t known_seed_sad=std::numeric_limits<uint64_t>::max()) const;
    long double motion_mb_rd_cost(const Frame& cur,const Frame& ref,int mx,int my,MotionVector mv,
                                  ProgressiveMvMode mode,uint64_t mv_bits,Frame& pred) const;

    MotionVector refine_qpel_block(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                                   MotionVector seed,int range,uint64_t* best_sad=nullptr,
                                   uint64_t known_seed_sad=std::numeric_limits<uint64_t>::max()) const;

    MotionVector refine_motion_block(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                                     MotionVector seed,int range,ProgressiveMvMode mode,uint64_t* best_sad=nullptr,
                                     uint64_t known_seed_sad=std::numeric_limits<uint64_t>::max()) const;

    struct VlcCode { uint32_t code; int bits; };
    struct BlockCoding {
        const std::vector<uint8_t>* plane = nullptr;
        int width=0, height=0, bx=0, by=0;
        std::array<int,64> qac{};
        int dc_target=0;
        int dc_diff=0;
        bool dc_pred_left=false;
        bool coded=false;
    };

    // ff_msmp4_mb_i_table: VC-1 uses this VLC for the six-bit intra CBP.
    static constexpr std::array<VlcCode,64> kMbIntraVlc = {{
        {0x1,1},{0x17,6},{0x9,5},{0x5,5},{0x6,5},{0x47,9},{0x20,7},{0x10,7},
        {0x2,5},{0x7c,9},{0x3a,7},{0x1d,7},{0x2,6},{0xec,9},{0x77,8},{0x0,8},
        {0x3,5},{0xb7,9},{0x2c,7},{0x13,7},{0x1,6},{0x168,10},{0x46,8},{0x3f,8},
        {0x1e,6},{0x712,13},{0xb5,9},{0x42,8},{0x22,7},{0x1c5,11},{0x11e,10},{0x87,9},
        {0x6,4},{0x3,9},{0x1e,7},{0x1c,6},{0x12,7},{0x388,12},{0x44,9},{0x70,9},
        {0x1f,6},{0x23e,11},{0x39,8},{0x8e,9},{0x1,7},{0x1c6,11},{0xb6,9},{0x45,9},
        {0x14,6},{0x23f,11},{0x7d,9},{0x18,9},{0x7,7},{0x1c7,11},{0x86,9},{0x19,9},
        {0x15,6},{0x1db,10},{0x2,9},{0x46,9},{0xd,8},{0x713,13},{0x1da,10},{0x169,10}
    }};

    // VC-1 progressive P-picture CBPCY VLC table 0 (Table 77 in ST 421).
    // Index is the six-bit CBP in Y0,Y1,Y2,Y3,U,V order.
    static constexpr std::array<VlcCode,64> kPcbpVlc = {{
        {0,13},{6,13},{15,7},{13,13},{13,7},{11,13},{3,13},{13,12},
        {5,6},{8,13},{49,7},{10,12},{12,6},{114,8},{102,8},{119,8},
        {1,5},{54,7},{96,8},{8,12},{10,6},{111,8},{5,13},{15,12},
        {12,7},{10,13},{2,13},{12,12},{13,6},{115,8},{53,7},{63,7},
        {1,6},{7,13},{1,8},{7,12},{14,7},{12,13},{4,13},{14,12},
        {1,7},{9,13},{97,8},{11,12},{7,5},{58,7},{52,7},{62,7},
        {4,6},{103,8},{1,13},{9,12},{11,6},{56,7},{101,8},{118,8},
        {4,5},{110,8},{100,8},{30,6},{2,3},{5,3},{4,3},{3,2}
    }};

    // Advanced Profile transposes the WMV1 intra scan before use. This is
    // transpose(ff_wmv1_scantable[1]), which is zz_8x8[1] in FFmpeg's decoder.
    static constexpr std::array<uint8_t,64> kIntraScan = {{
         0, 1, 8,16, 9, 2, 3,10,17,24,32,25,18,11, 4, 5,
        12, 6,19,26,33,40,48,41,34,27,20,13, 7,14,15,21,
        28,35,42,49,56,57,50,43,36,29,22,23,30,31,37,44,
        51,58,59,52,45,38,39,46,47,53,60,61,54,55,62,63
    }};

    // Progressive intra scans used when AC prediction follows the top or left
    // DC predictor. These are VC-1 zz_8x8[2] and zz_8x8[3] respectively
    // (the transposed WMV1 directional scans used by the decoder).
    static constexpr std::array<uint8_t,64> kIntraTopPredScan = {{
         0, 8, 1,16,24, 9, 2, 3,10,17,32,40,25,18,11, 4,
         5, 6,12,19,26,33,48,56,41,34,27,20,13, 7,14,15,
        21,28,35,42,49,57,50,43,36,29,22,23,30,37,44,51,
        58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63
    }};
    static constexpr std::array<uint8_t,64> kIntraLeftPredScan = {{
         0, 1, 2, 8, 3, 4, 5, 9,16,24,17,10,11, 6, 7,13,
        12,19,18,25,32,40,33,26,27,20,14,15,22,21,28,35,
        34,41,48,56,49,42,43,36,29,30,23,31,38,37,44,51,
        50,57,58,59,52,45,39,46,53,60,61,54,47,55,62,63
    }};

    // transpose(ff_wmv1_scantable[0]) = zz_8x8[0], used by progressive
    // inter 8x8 blocks.
    static constexpr std::array<uint8_t,64> kInterScan = {{
         0, 1, 8,16, 9, 2, 3,10,17,24,32,25,18,11, 4, 5,
         6, 7,13,12,19,26,33,40,48,41,34,27,20,14,15,23,
        22,21,28,35,42,49,56,57,50,43,36,29,30,31,39,38,
        37,44,51,58,59,52,45,46,47,55,54,53,60,61,62,63
    }};

    // Progressive variable-transform scans (VC-1 8x4, 4x8, and 4x4).
    // Entries use an 8-sample coefficient stride, matching decoder block storage.
    static constexpr std::array<uint8_t,32> kInterScan8x4 = {{
         0, 8, 1,16, 2, 9,10, 3,24,17, 4,11,18,12, 5,19,
        25,13,20,26,27, 6,21,28,14,22,29, 7,30,15,23,31
    }};
    static constexpr std::array<uint8_t,32> kInterScan4x8 = {{
         0, 1, 8, 2, 9,16,17,24,10,32,25,18,40, 3,33,26,
        48,11,56,41,34,49,57,42,19,50,27,58,35,43,51,59
    }};
    static constexpr std::array<uint8_t,16> kInterScan4x4 = {{
         0, 8,16, 1, 9,24,17, 2,10,18,25, 3,11,26,19,27
    }};

    // Simple/Main Profile rectangular scans (ff_wmv2_scantableA/B). 8x8 and
    // 4x4 use the same progressive scans as the current Advanced path.
    static constexpr std::array<uint8_t,32> kMainInterScan8x4 = {{
         0, 1, 2, 8, 3, 9,10,16, 4,11,17,24,18,12, 5,19,
        25,13,20,26,27, 6,21,28,14,22,29, 7,30,15,23,31
    }};
    static constexpr std::array<uint8_t,32> kMainInterScan4x8 = {{
         0, 8, 1,16, 9,24,17, 2,32,10,25,40,18,48,33,26,
        56,41,34, 3,49,57,11,42,19,50,27,58,35,43,51,59
    }};


    Frame empty_frame() const;

    Frame empty_padded_frame(int mbw,int mbh) const;

    Frame crop_padded_frame(const Frame& padded) const;


    // VC-1 section 8.6 in-loop deblocking.  The primitive filters the two
    // samples immediately adjacent to an edge.  A four-line group is enabled
    // by testing its third line first, exactly as required by the VC-1 filter
    // decision rule; when that test succeeds the remaining three lines are
    // evaluated independently.
    static bool loop_filter_line(std::vector<uint8_t>& p,int w,int h,int x,int y,
                                 int cross_dx,int cross_dy,int pq);

    static void loop_filter_edge(std::vector<uint8_t>& p,int w,int h,int x,int y,
                                 int along_dx,int along_dy,int cross_dx,int cross_dy,
                                 int len,int pq,SimdTier tier);

    static void loop_filter_hborder(std::vector<uint8_t>& p,int w,int h,int x,int y,int len,int pq,SimdTier tier);
    static void loop_filter_vborder(std::vector<uint8_t>& p,int w,int h,int x,int y,int len,int pq,SimdTier tier);

    void apply_i_like_loop_filter_plane(std::vector<uint8_t>& p,int w,int h) const;

    void apply_i_like_loop_filter(Frame& f) const;

    static uint8_t loop_filter_pattern(uint8_t skip_mask,TransformType t);

    static uint8_t loop_filter_tt(TransformType t);

    void apply_p_loop_filter_plane_main(std::vector<uint8_t>& p,int w,int h,int plane_k,
                                        const std::vector<std::array<MotionVector,4>>& block_mvs,
                                        const std::vector<std::array<uint8_t,6>>& cbp,
                                        const std::vector<std::array<uint8_t,6>>& tt,
                                        const std::vector<uint8_t>& intra,
                                        int mbw,int mbh) const;

    void apply_p_loop_filter_plane(std::vector<uint8_t>& p,int w,int h,int plane_k,
                                   const std::vector<std::array<MotionVector,4>>& block_mvs,
                                   const std::vector<std::array<uint8_t,6>>& cbp,
                                   const std::vector<std::array<uint8_t,6>>& tt,
                                   const std::vector<uint8_t>& intra,
                                   int mbw,int mbh) const;

    void apply_p_loop_filter(Frame& f,const std::vector<std::array<MotionVector,4>>& block_mvs,
                             const std::vector<std::array<uint8_t,6>>& cbp,
                             const std::vector<std::array<uint8_t,6>>& tt,
                             const std::vector<uint8_t>& intra,
                             int mbw,int mbh) const;

    static uint8_t sample_clamped(const std::vector<uint8_t>& p,int w,int h,int x,int y);

    static int floor_div4(int v);

    static int mspel_filter4(int a,int b,int c,int d,int mode);

    static uint8_t luma_mc_sample(const std::vector<uint8_t>& p,int w,int h,
                                  int x,int y,int mvq_x,int mvq_y,bool rnd=false);

    static uint8_t luma_mc_sample_mode(const std::vector<uint8_t>& p,int w,int h,
                                       int x,int y,int mvq_x,int mvq_y,
                                       ProgressiveMvMode mode,bool rnd=false);

    static uint8_t chroma_mc_sample(const std::vector<uint8_t>& p,int w,int h,
                                    int x,int y,int mvq_x,int mvq_y,
                                    bool b_interp_future=false,bool rnd=false);

    void motion_compensate_mb(Frame& dst,const Frame& ref,int mx,int my,MotionVector mv, ProgressiveMvMode mode=ProgressiveMvMode::OneMvQpel) const;

    static int median4(int a,int b,int c,int d);

    static MotionVector chroma_mv_4mv(const std::array<MotionVector,4>& mv);

    void motion_compensate_4mv(Frame& dst,const Frame& ref,int mx,int my,
                               const std::array<MotionVector,4>& mv) const;

    void motion_compensate_4mv_padded(Frame& dst,const Frame& ref,int mx,int my,
                                      const std::array<MotionVector,4>& mv) const;

    void motion_compensate_mb_padded(Frame& dst,const Frame& ref,int mx,int my,MotionVector mv, ProgressiveMvMode mode=ProgressiveMvMode::OneMvQpel) const;

    void motion_compensate_bi_mb(Frame& dst,const Frame& past,const Frame& future,
                                 int mx,int my,MotionVector fmv,MotionVector bmv,
                                 ProgressiveMvMode mode=ProgressiveMvMode::OneMvQpel) const;

    void motion_compensate_bi_mb_padded(Frame& dst,const Frame& past,const Frame& future,
                                        int mx,int my,MotionVector fmv,MotionVector bmv,
                                        ProgressiveMvMode mode=ProgressiveMvMode::OneMvQpel) const;

    void inverse_transform_8x8(std::array<int,64>& block) const;

    void put_block(std::vector<uint8_t>& dst,int w,int h,int x0,int y0,
                   const std::array<int,64>& block,int bias) const;

    static void add_block(std::vector<uint8_t>& dst,int w,int h,int x0,int y0,
                          const std::array<int,64>& block);

    std::array<double,64> forward_transform(const std::vector<uint8_t>& p,
                                             int w, int h, int x0, int y0,
                                             int pad_sample=128) const;

    std::array<double,64> forward_transform_residual(const std::vector<uint8_t>& src,
                                                      const std::vector<uint8_t>& pred,
                                                      int w,int h,int x0,int y0) const;

    struct TransformParentDecision {
        TransformType type=TransformType::T8x8;
        // All partitions of an 8x8 parent cover exactly 64 spatial coefficient
        // positions.  Older releases reserved four independent 64-element
        // arrays here (one per possible 4x4 partition), making every transform
        // decision about 4x larger than necessary.  Store the selected
        // partition coefficients in one spatial 8x8 grid instead. VC-1 Escape-3
        // magnitudes are at most 2047 here, so signed 16-bit storage is exact.
        std::array<int16_t,64> qcoeff{};
        uint8_t skip_mask=0;
        double distortion=0.0;
        size_t estimated_bits=0; // coefficient/subblock payload estimate, excluding CBPCY/TTMB
    };
    struct MbTransformDecision {
        TransformType type=TransformType::T8x8; // common type for macroblock/frame signaling; first coded type in block mode
        TransformSignalLevel signal_level=TransformSignalLevel::Macroblock;
        std::array<TransformParentDecision,6> parents{};
        uint8_t cbp=0;
        double distortion=0.0;
        size_t estimated_bits=0;
    };

    static int transform_part_count(TransformType t);

    static void transform_part_geometry(TransformType t,int part,int& ox,int& oy,int& bw,int& bh);

    std::array<double,64> forward_transform_residual_part(const std::vector<uint8_t>& src,
                                                           const std::vector<uint8_t>& pred,
                                                           int w,int h,int x0,int y0,
                                                           int bw,int bh) const;

    bool uniform_quantizer() const;

    int picture_double_quant(int mquant=-1,bool dquant_derived=false) const;

    int dequant_level(int level,int mquant=-1,bool dquant_derived=false) const;

    int scalar_quantize_level(double coeff,int mquant=-1,bool dquant_derived=false) const;

    std::array<int,64> quantize_inter_part(const std::vector<uint8_t>& src,
                                           const std::vector<uint8_t>& pred,
                                           int w,int h,int x0,int y0,int bw,int bh,
                                           bool allow_trellis=true,int mquant=-1,
                                           const std::vector<uint8_t>* luma_context=nullptr) const;

    std::array<int,64> inverse_partition(TransformType t,const std::array<int,64>& q,int mquant,bool dquant_derived=false) const;

    void add_partition(std::vector<uint8_t>& dst,int w,int h,int x0,int y0,
                       TransformType t,int part,const std::array<int,64>& q,int mquant,bool dquant_derived=false) const;

    static constexpr uint16_t kTtmbCodes[3][16] = {
        {0x0003,0x002E,0x005F,0x0000,0x0016,0x0015,0x0001,0x0004,0x0014,0x02F1,0x0179,0x017B,0x0BC0,0x0BC1,0x05E1,0x017A},
        {0x0006,0x0006,0x0003,0x0007,0x000F,0x000E,0x0000,0x0002,0x0002,0x0014,0x0011,0x000B,0x0009,0x0021,0x0015,0x0020},
        {0x0006,0x0000,0x000E,0x0005,0x0002,0x0003,0x0003,0x000F,0x0002,0x0081,0x0021,0x0009,0x0101,0x0041,0x0011,0x0100}
    };
    static constexpr uint8_t kTtmbBits[3][16] = {
        {2,6,7,2,5,5,2,3,5,10,9,9,12,12,11,9},
        {3,4,4,4,4,4,3,3,2,7,7,6,6,8,7,8},
        {3,3,4,5,3,3,4,4,2,10,8,6,11,9,7,11}
    };
    static constexpr uint8_t kTtblkCodes[3][8] = {
        {0,1,3,5,16,17,18,19},
        {3,0,1,2,3,5,8,9},
        {1,0,1,4,6,7,10,11}
    };
    static constexpr uint8_t kTtblkBits[3][8] = {
        {2,2,2,3,5,5,5,5},
        {2,3,3,3,3,3,4,4},
        {2,3,3,3,3,3,4,4}
    };
    // TTMB block-level and TTBLK use the same eight semantic symbols, but
    // their VLC symbol ordering changes with PQUANT. Variant numbering here is
    // encoder-private: 8x8, 8x4-both/top/bottom, 4x8-both/left/right, 4x4.
    static constexpr uint8_t kTtSymbolVariant[3][8] = {
        {1,4,0,7,2,3,6,5},
        {0,6,5,7,1,4,3,2},
        {0,4,7,3,6,5,1,2}
    };
    static constexpr uint16_t kSubblkCodes[3][15] = {
        {14,12,7,11,9,26,2,10,27,8,0,6,1,15,1},
        {14,0,8,15,10,4,23,13,5,9,25,3,24,22,1},
        {5,6,2,2,8,0,28,3,1,3,29,1,19,18,15}
    };
    static constexpr uint8_t kSubblkBits[3][15] = {
        {5,5,5,5,5,6,4,5,6,5,4,5,4,5,1},
        {4,3,4,4,4,5,5,4,5,4,5,4,5,5,2},
        {3,3,4,3,4,5,5,3,5,4,5,4,5,5,4}
    };

    int tt_table_index() const;

    void write_subblkpat(BitWriter& b,uint8_t skipmask) const;

    static uint8_t tt_variant(TransformType t,uint8_t skipmask);

    int tt_symbol_for_variant(uint8_t variant) const;

    void write_ttmb_macro(BitWriter& b,TransformType t,uint8_t first_skipmask) const;

    static int ttmb_block_value(TransformType t,uint8_t skipmask);

    void write_ttmb_block(BitWriter& b,TransformType t,uint8_t first_skipmask) const;

    void write_ttblk(BitWriter& b,TransformType t,uint8_t skipmask) const;

    static void write_rect_subblkpat(BitWriter& b,uint8_t skipmask);

    void write_inter_scan(BitWriter& b,const std::array<int,64>& q,TransformType t,
                          int coding_set,bool use_vlc,bool& esc3_lengths_written,
                          bool dquantfrm=false) const;

    void write_transform_payload(BitWriter& b,const MbTransformDecision& d,int coding_set,
                                 bool use_vlc,bool& esc3_lengths_written,bool implicit_8x8=false,
                                 bool dquantfrm=false) const;

    void refresh_transform_estimate(MbTransformDecision& d,int coding_set,bool use_vlc,
                                    bool dquantfrm=false) const;

    size_t estimate_parent_payload_bits(const TransformParentDecision& p,int coding_set,bool use_vlc,
                                        bool dquantfrm=false) const;

    size_t estimate_ttblk_bits(const TransformParentDecision& p) const;

    TransformParentDecision evaluate_transform_parent(const Frame& src,const Frame& pred,int mx,int my,int parent_index,
                                                        TransformType type,int coding_set,bool use_vlc,
                                                        bool allow_trellis=true,int mquant=-1,bool dquantfrm=false) const;

    MbTransformDecision evaluate_transform_type(const Frame& src,const Frame& pred,int mx,int my,
                                                TransformType type,int coding_set,bool use_vlc,
                                                bool allow_trellis=true,int mquant=-1,bool dquantfrm=false) const;

    MbTransformDecision choose_transform_mb(const Frame& src,const Frame& pred,int mx,int my,
                                            int coding_set,bool use_vlc,int mquant=-1,bool dquantfrm=false,
                                            const TransformPicturePlan* picture_plan=nullptr) const;

    struct MbQuantDecision {
        MbTransformDecision tx;
        int mquant=0;
    };

    enum class DQuantProfile : uint8_t { FourEdges=0, DoubleEdges=1, SingleEdge=2, AllMbs=3 };
    struct DQuantPlan {
        bool enabled=false;
        DQuantProfile profile=DQuantProfile::AllMbs;
        int edge_selector=0;
        bool bilevel=false;
        int altpq=0;
        std::vector<int> mquant;
    };

    int rate_weighted_mquant(double block_weight) const;
    struct PerceptualMbPriority {
        double dark_detail=0.0;
        double color_luma=0.0;
        double color_chroma=0.0;
        double requested_q_boost=0.0;
    };
    double residual_priority_position(double local_mae) const;
    double residual_priority_requested_q_boost(double local_mae,double picture_mae) const;
    int residual_priority_mquant(int rate_base,double local_mae,double picture_mae) const;
    PerceptualMbPriority perceptual_mb_priority(const Frame& src,int mx,int my,bool intra) const;
    int choose_intra_mquant(const Frame& src,int mx,int my) const;
    static bool dquant_edge_member(DQuantProfile profile,int selector,int mx,int my,int mbw,int mbh);
    DQuantPlan make_dquant_plan(const std::vector<int>& desired,int mbw,int mbh) const;
    static void write_altpq(BitWriter& b,int pquant,int altpq);
    void write_dquant_header(BitWriter& b,const DQuantPlan& plan) const;
    void write_dquant_mb(BitWriter& b,const DQuantPlan& plan,size_t pos) const;
    static bool dquant_mb_derived(const DQuantPlan& plan,size_t pos,int mbw);

    MbQuantDecision choose_dquant_transform_mb(const Frame& src,const Frame& pred,int mx,int my,
                                               int coding_set,bool use_vlc,
                                               const TransformPicturePlan* picture_plan=nullptr) const;

    template<class R>
    static void record_mquant(R& out,int pquant,int mquant) {
        ++out.mquant_samples;
        out.mquant_sum+=static_cast<uint64_t>(mquant);
        out.mquant_min=std::min(out.mquant_min,mquant);
        out.mquant_max=std::max(out.mquant_max,mquant);
        if (mquant!=pquant) ++out.dquant_macroblocks;
    }

    static void write_mqdiff(BitWriter& b,int pquant,int mquant);

    static uint8_t pack_transform_map(TransformType type,uint8_t skipmask);

    static size_t count_coded_parts(const TransformParentDecision& p,TransformType type);

    void apply_transform_parent(std::vector<uint8_t>& dst,int w,int h,int x0,int y0,
                                const TransformParentDecision& p,TransformType type,int mquant,bool dquant_derived=false) const;

    int trellis_rate_bits_for_set(int run,int level,bool last,int coding_set) const;

    int trellis_rate_bits(int run,int level,bool last,bool intra_family) const;

    static double trellis_coeff_weight(TransformType type,int index);

    std::array<int,64> trellis_quantize(const std::array<double,64>& coeff,
                                        const uint8_t* scan,int n,int first_scan,
                                        TransformType type,bool intra_family,double lambda_scale=1.0,int mquant=-1) const;

    double perceptual_lambda_scale(const std::vector<uint8_t>& src,
                                   int w,int h,int x0,int y0,int bw,int bh,
                                   const std::vector<uint8_t>* pred=nullptr,
                                   double* adjacency_out=nullptr,
                                   const std::vector<uint8_t>* luma_context=nullptr) const;

    std::array<int,64> quantize_ac(const std::vector<uint8_t>& p,
                                   int w, int h, int x0, int y0,
                                   bool chroma=false, const uint8_t* scan=kIntraScan.data(),
                                   int mquant=-1, bool dquant_derived=false,
                                   const std::vector<uint8_t>* luma_context=nullptr) const;

    std::array<int,64> quantize_inter(const std::vector<uint8_t>& src,
                                      const std::vector<uint8_t>& pred,
                                      int w,int h,int x0,int y0,
                                      int mquant=-1,bool dquant_derived=false) const;

    static bool has_ac(const std::array<int,64>& q);

    static bool has_any(const std::array<int,64>& q);

    static int predict_coded(const std::vector<uint8_t>& g, int gw, int x, int y);

    using AcVlcEntry = entropy::AcVlcEntry;
    using AcTableView = std::pair<const AcVlcEntry*, size_t>;

    static size_t block_index(int mbw, int mx, int my, int k);

    static void write_decode012(BitWriter& b, int index);

    static int luma_coding_set(int table_index, int pqindex);

    static int chroma_coding_set(int table_index, int pqindex);

    static AcTableView ac_table(int coding_set);

    static entropy::VlcCode ac_escape(int coding_set);

    struct AcLookup {
        // Direct VLC levels in coding sets 0..7 never exceed 63. Store table
        // indices rather than pointers to keep the eight-set cache compact and
        // friendly to Windows' smaller default thread stack.
        std::array<int16_t, 2*64*64> direct_index{};
        std::array<uint8_t, 2*64> max_level{};
        std::array<int8_t, 2*64> max_run{};
        AcLookup() { direct_index.fill(-1); max_run.fill(-1); }
    };

    static const AcLookup& ac_lookup(int coding_set);

    static const AcVlcEntry* find_ac_vlc(int coding_set, int run, int level, bool last);

    static int max_direct_level(int coding_set, int run, bool last);

    static int max_direct_run(int coding_set, int level, bool last);

    static void write_ac_sign(BitWriter& b, int level);

    int esc3_level_bits() const;

    int max_quantized_level() const;

    void write_ac_esc3(BitWriter& b, int run, int level, bool last,
                       int coding_set, bool& esc3_lengths_written,
                       bool dquantfrm=false) const;

    void write_ac_coeff(BitWriter& b, int run, int level, bool last,
                        int coding_set, bool use_vlc, bool& esc3_lengths_written,
                        bool dquantfrm=false) const;

    uint64_t estimate_intra_ac_bits(const std::array<int,64>& q, int coding_set,
                                    bool use_vlc, const uint8_t* scan) const;

    void write_ac_block(BitWriter& b, const std::array<int,64>& q,
                        bool /*chroma*/, int coding_set, bool use_vlc,
                        bool& esc3_lengths_written,
                        const uint8_t* scan=kIntraScan.data(), bool dquantfrm=false) const;

    void write_inter_block(BitWriter& b,const std::array<int,64>& q,
                           int coding_set,bool use_vlc,bool& esc3_lengths_written,
                           const uint8_t* scan=nullptr,bool dquantfrm=false) const;

    static int dc_scale(int pq);
    static int arshift(int v, int n);
    int reconstruct_dc_sample_with_bias(int qdc,int mquant,int bias) const;
    int reconstruct_dc_sample(int qdc,int mquant=-1) const;
    int choose_dc_for_mean_with_bias(int mean,int mquant,int bias) const;
    int choose_dc_for_mean(int mean,int mquant=-1) const;
    int choose_inter_intra_dc_for_mean(int mean,int mquant=-1) const;
    static int clamp_dc_target_for_diff(int target,int predictor,int mquant);
    static int dqscale_value(int index);
    static int scale_dc_predictor(int value,int from_q,int to_q);
    int scale_ac_predictor(int value,int from_qstate,int to_qstate) const;
    static int block_mean(const std::vector<uint8_t>& p, int w, int h, int x0, int y0);
    static int padded_block_mean(const std::vector<uint8_t>& p, int w, int h, int x0, int y0, int pad_sample);
    static int visible_block_mean(const std::vector<uint8_t>& p, int w, int h, int x0, int y0);
    int main_dc_pred_base() const;
    static int predict_dc_main(const std::vector<int>& g, int gw, int x, int y, int border,
                               bool* pred_left=nullptr);
    static int predict_dc(const std::vector<int>& g, int gw, int x, int y,
                          bool a_avail, bool c_avail, bool* pred_left=nullptr);
    static int predict_dc_dquant(const std::vector<int>& g,const std::vector<int>& qg,int gw,int x,int y,
                                 int current_q,bool a_avail,bool c_avail,bool* pred_left=nullptr);
    struct DcCodePlan { int symbol=0; unsigned suffix=0; int suffix_bits=0; bool escape=false; };
    static DcCodePlan dc_code_plan(int diff,int pqindex);
    static uint64_t dc_diff_bits(int table,int diff,bool chroma,int pqindex);
    static void write_dc_diff(BitWriter& b,int diff,bool chroma,int pqindex,int table);
private:
    EncoderConfig c_;
    int scale_=10;
    mutable const uint8_t* aq_luma_mean_ptr_=nullptr;
    mutable size_t aq_luma_mean_size_=0;
    mutable double aq_luma_mean_=0.0;
};

} // namespace libvc1
