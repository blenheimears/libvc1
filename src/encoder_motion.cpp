#include "encoder_internal.h"
#include <cstring>
#include <chrono>
#include <functional>


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

bool Vc1Encoder::mv_mode_halfpel(ProgressiveMvMode m) { return m==ProgressiveMvMode::OneMvHpel || m==ProgressiveMvMode::OneMvHpelBilinear; }

bool Vc1Encoder::mv_mode_bilinear(ProgressiveMvMode m) { return m==ProgressiveMvMode::OneMvHpelBilinear; }

bool Vc1Encoder::mv_mode_mixed(ProgressiveMvMode m) { return m==ProgressiveMvMode::MixedMv; }

PAnalysis Vc1Encoder::analyze_p_picture_core(const Frame& f, const Frame& ref, ProgressiveMvMode mode, bool allow_intra, const PAnalysis* shared_seed) const {
        if (f.width != c_.width || f.height != c_.height ||
            ref.width != c_.width || ref.height != c_.height)
            throw std::runtime_error("P-picture reference dimensions do not match encoder configuration");
        const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
        PAnalysis out;
        out.mv_mode=mode;
        const bool allow_4mv=mv_mode_mixed(mode) && c_.syntax==StreamSyntax::Advanced;
        const ProgressiveMvMode one_mode=allow_4mv?ProgressiveMvMode::OneMvQpel:mode;
        const size_t mbs=static_cast<size_t>(mbw)*mbh;
        out.mvs.resize(mbs);
        out.block_mvs.resize(mbs);
        out.use_4mv.assign(mbs,0);
        out.use_intra.assign(mbs,0);
        out.screen_sads.assign(mbs,0);
        out.inter_intra_cost_ratio.assign(mbs,-1.0);
        out.distant_local_mae.assign(mbs,-1.0);
        out.distant_candidate_mae.assign(mbs,-1.0);
        out.distant_match_decision.assign(mbs,VC1_DISTANT_MATCH_NONE);
        uint64_t total_sad=0, rate_sad_total=0,total_pixels=0;

        // A zero search range is a real no-search mode.  Older releases still
        // ran global search plus predictor/coarse/fine/refinement bookkeeping,
        // repeatedly measuring the exact same zero-motion candidate.  This is
        // both the natural behavior of --search-range 0 and the foundation of
        // the --fastest preset.
        if (c_.motion_search_range==0) {
            for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
                const uint64_t zero_sad=mb_sad(f,ref,mx,my,0,0,1);
                total_sad+=zero_sad;
                out.screen_sads[static_cast<size_t>(my)*mbw+mx]=zero_sad;
                const int x0=mx*16,y0=my*16;
                total_pixels+=static_cast<uint64_t>(std::max(0,std::min(16,c_.width-x0))) *
                              static_cast<uint64_t>(std::max(0,std::min(16,c_.height-y0)));
            }
            out.mean_abs_residual=total_pixels
                ? static_cast<double>(total_sad)/static_cast<double>(total_pixels) : 0.0;
            out.rate_complexity=out.mean_abs_residual;
            return out;
        }
        const int configured_range=c_.motion_search_range;
        const int local_range=std::min(configured_range,std::max(0,c_.motion_local_search_range));
        const GlobalMotionResult global=shared_seed ? GlobalMotionResult{} : global_motion_search(f,ref,local_range);
        if (shared_seed && shared_seed->mvs.size()!=mbs)
            throw std::runtime_error("shared P motion seed has the wrong macroblock count");
        if (shared_seed) {
            out.distant_local_mae=shared_seed->distant_local_mae;
            out.distant_candidate_mae=shared_seed->distant_candidate_mae;
            out.distant_match_decision=shared_seed->distant_match_decision;
        }
        // Long-range matches are propagated in raster order.  The expensive
        // content index is built lazily only after a block actually fails the
        // local/neighbor short-circuit.
        std::vector<uint8_t> long_range_propagated(mbs,0);
        std::unique_ptr<LongRangeIndex> long_range_index;
        std::unique_ptr<Frame> me_rd_pred;
        if (c_.me_quality==VC1_ME_RD) me_rd_pred=std::make_unique<Frame>(empty_frame());

        for (int my=0; my<mbh; ++my) {
            for (int mx=0; mx<mbw; ++mx) {
                const size_t pos=static_cast<size_t>(my)*mbw+mx;
                const int range=configured_range;
                const int x0=mx*16, y0=my*16;
                const auto rate_pi=predictor_info_mixed_1mv(out.block_mvs,out.use_intra,mbw,mbh,mx,my);
                uint64_t one_sad=0;
                MotionVector one{};
                if (shared_seed) {
                    MotionVector seed=shared_seed->mvs[pos];
                    const auto qb=motion_search_bounds_qpel(range);
                    seed.xq=std::clamp(seed.xq,qb.xmin,qb.xmax);
                    seed.yq=std::clamp(seed.yq,qb.ymin,qb.ymax);
                    // Preserve the historical second mode-specific refinement:
                    // refine_motion_block is intentionally local rather than an
                    // exhaustive qpel search, so a second seeded pass can advance
                    // the winner by another sub-pixel step on sloped SAD surfaces.
                    one=refine_motion_block_staged(f,ref,x0,y0,16,16,seed,range,one_mode,rate_pi,true,
                                                   me_rd_pred.get(),&one_sad);
                } else {
                    const auto pi=predictor_info(out.mvs,mbw,mbh,mx,my);
                    IntegerMotionResult im{};
                    bool selected_distant=false;
                    auto better_integer=[](const IntegerMotionResult& a,const IntegerMotionResult& b) {
                        if (a.full_sad!=b.full_sad) return a.full_sad<b.full_sad;
                        const int am=std::abs(a.dx)+std::abs(a.dy),bm=std::abs(b.dx)+std::abs(b.dy);
                        if (am!=bm) return am<bm;
                        if (a.dy!=b.dy) return a.dy<b.dy;
                        return a.dx<b.dx;
                    };
                    if (c_.long_range_search_mode==VC1_LONG_RANGE_LEGACY_DISTANT_FIRST) {
                        // Historical 0.1.74-0.1.76 behavior: a good propagated
                        // distant vector can bypass local UMH entirely.
                        bool have_short_circuit=false;
                        if (range>local_range) {
                            const auto propagated=propagated_long_range_motion(
                                f,ref,mx,my,mbw,out.mvs,long_range_propagated,range);
                            if (motion_match_good(propagated.full_sad,mx,my)) {
                                im=propagated;
                                have_short_circuit=true;
                                selected_distant=true;
                            }
                        }
                        if (!have_short_circuit)
                            im=integer_motion_umh(f,ref,mx,my,pi,global,local_range);
                    } else {
                        // Normal policy always measures local UMH first. In the
                        // default compare mode, a propagated distant candidate
                        // is also measured even when local prediction is good,
                        // and the lower full-SAD candidate wins. local-good
                        // deliberately skips all distant work after a strong
                        // local result.
                        im=integer_motion_umh(f,ref,mx,my,pi,global,local_range);
                        const uint64_t local_sad=im.full_sad;
                        const uint64_t pixels=active_mb_pixels(mx,my);
                        if (pixels) out.distant_local_mae[pos]=static_cast<double>(local_sad)/static_cast<double>(pixels);
                        auto record_distant=[&](uint64_t sad,int decision) {
                            if (!pixels || sad>=std::numeric_limits<uint64_t>::max()/8) return;
                            const double mae=static_cast<double>(sad)/static_cast<double>(pixels);
                            if (out.distant_candidate_mae[pos]<0.0 || mae<out.distant_candidate_mae[pos]) {
                                out.distant_candidate_mae[pos]=mae;
                                out.distant_match_decision[pos]=static_cast<uint8_t>(decision);
                            }
                        };
                        const bool skip_distant=
                            c_.long_range_search_mode==VC1_LONG_RANGE_LOCAL_GOOD_SKIP &&
                            motion_match_good(im.full_sad,mx,my);
                        if (range>local_range && !skip_distant) {
                            const auto propagated=propagated_long_range_motion(
                                f,ref,mx,my,mbw,out.mvs,long_range_propagated,range);
                            const int decision=distant_match_decision(propagated.full_sad,local_sad,mx,my);
                            record_distant(propagated.full_sad,decision);
                            if (better_integer(propagated,im) && decision>=VC1_DISTANT_MATCH_ABSOLUTE_GOOD) {
                                im=propagated;
                                selected_distant=true;
                            }
                        }
                    }
                    // Keep the content-signature lookup lazy. compare/local-good
                    // differ only in whether a good local result may suppress
                    // propagated-distant evaluation; neither makes every block
                    // pay for the frame-wide signature index.
                    if (range>local_range && !motion_match_good(im.full_sad,mx,my)) {
                        if (!long_range_index)
                            long_range_index=std::make_unique<LongRangeIndex>(build_long_range_index(ref));
                        uint64_t ext_candidate_sad=std::numeric_limits<uint64_t>::max();
                        int ext_decision=VC1_DISTANT_MATCH_NONE;
                        const auto ext=extended_content_motion(f,ref,mx,my,range,*long_range_index,im,
                                                               &ext_candidate_sad,&ext_decision);
                        const uint64_t pixels=active_mb_pixels(mx,my);
                        if (c_.long_range_search_mode!=VC1_LONG_RANGE_LEGACY_DISTANT_FIRST && pixels &&
                            ext_candidate_sad<std::numeric_limits<uint64_t>::max()/8) {
                            const double mae=static_cast<double>(ext_candidate_sad)/static_cast<double>(pixels);
                            if (out.distant_candidate_mae[pos]<0.0 || mae<out.distant_candidate_mae[pos]) {
                                out.distant_candidate_mae[pos]=mae;
                                out.distant_match_decision[pos]=static_cast<uint8_t>(ext_decision);
                            }
                        }
                        if (ext.dx!=im.dx || ext.dy!=im.dy) {
                            im=ext;
                            selected_distant=true;
                        }
                    }
                    if (selected_distant) long_range_propagated[pos]=1;
                    const uint64_t known_one_seed_sad=c_.syntax==StreamSyntax::Wmv9Main
                        ? std::numeric_limits<uint64_t>::max() : im.full_sad;
                    one=refine_motion_block_staged(f,ref,x0,y0,16,16,
                                                   MotionVector{im.dx*4,im.dy*4},range,one_mode,rate_pi,true,
                                                   me_rd_pred.get(),&one_sad,known_one_seed_sad);
                }
                const uint64_t one_screen_sad=one_sad;
                out.mvs[pos]=one;
                out.block_mvs[pos].fill(one);

                // Group B: progressive Mixed-MV.  Search each 8x8 luma block
                // only when the caller can actually select 4-MV.  B-picture
                // forward/backward analysis calls this routine with allow_4mv=false
                // and consumes only the 1-MV field; older code nevertheless ran
                // all four qpel sub-block searches and then discarded them.
                const double lambda=std::max(1.0,0.35*static_cast<double>(c_.pqindex));
                uint64_t chosen_sad=one_sad;
                uint64_t temporal_screen_sad=one_screen_sad;
                if (allow_4mv) {
                    std::array<MotionVector,4> four=out.block_mvs[pos];
                    uint64_t four_sad=0, four_screen_sad=0;
                    uint64_t extra_mv_bits=1; // MVTYPEMB contribution (bitplane amortized conservatively).
                    for (int k=0;k<4;++k) {
                        const auto pi4=predictor_info_4mv(out.block_mvs,out.use_intra,mbw,mbh,mx,my,k);
                        MotionVector pred=pi4.pre;
                        const int bx=x0+(k&1)*8, by=y0+((k>>1)&1)*8;
                        MotionVector best_seed=one;
                        uint64_t seed_cost=block_sad_qpel(f,ref,bx,by,8,8,best_seed);
                        const std::array<MotionVector,10> seeds={{
                            one,pred,
                            MotionVector{one.xq-8,one.yq},MotionVector{one.xq+8,one.yq},
                            MotionVector{one.xq,one.yq-8},MotionVector{one.xq,one.yq+8},
                            MotionVector{one.xq-4,one.yq},MotionVector{one.xq+4,one.yq},
                            MotionVector{one.xq,one.yq-4},MotionVector{one.xq,one.yq+4}
                        }};
                        const auto qb=motion_search_bounds_qpel(range);
                        for (auto seed:seeds) {
                            seed.xq=std::clamp(seed.xq,qb.xmin,qb.xmax); seed.yq=std::clamp(seed.yq,qb.ymin,qb.ymax);
                            if (same_mv(seed,best_seed)) continue; // already measured.
                            const uint64_t c=block_sad_qpel(f,ref,bx,by,8,8,seed);
                            const int cm=std::abs(seed.xq)+std::abs(seed.yq),bm=std::abs(best_seed.xq)+std::abs(best_seed.yq);
                            if (c<seed_cost || (c==seed_cost && cm<bm)) { seed_cost=c; best_seed=seed; }
                        }
                        uint64_t bsad=0;
                        MotionVector mv=refine_motion_block_staged(f,ref,bx,by,8,8,best_seed,range,
                                                               ProgressiveMvMode::MixedMv,pi4,true,nullptr,
                                                               &bsad,seed_cost);
                        four_screen_sad+=bsad;
                        // All four luma vectors contribute to one shared chroma MV.
                        // Edge macroblocks retain the motion chosen from their visible
                        // samples; reference padding/clamping handles samples outside
                        // the displayed raster without forcing an axis to zero.
                        four[static_cast<size_t>(k)]=mv;
                        out.block_mvs[pos][static_cast<size_t>(k)]=mv; // feeds n=2/3 predictors.
                        four_sad+=bsad;
                        MotionVector actual_pred=pi4.pre;
                        if (pi4.hybrid) {
                            const int da=modular_mv_distance(mv,pi4.a),dc=modular_mv_distance(mv,pi4.c);
                            actual_pred=(da<=dc)?pi4.a:pi4.c;
                            ++extra_mv_bits;
                        }
                        if (!same_mv(mv,actual_pred)) {
                            const MvDataSyntax est{mv.xq-actual_pred.xq,mv.yq-actual_pred.yq,false};
                            const int sym=mv_symbol(est);
                            extra_mv_bits+=groupa::kMvDiffBits[0][static_cast<size_t>(sym)]+mv_suffix_bits(est);
                        }
                    }
                    bool choose4=false;
                    if (c_.me_quality==VC1_ME_RD && me_rd_pred) {
                        // Mixed-MV's four subblock vectors are shortlisted with
                        // SATD+rate individually.  Make the final 1-MV-vs-4-MV
                        // choice with the same codec-aware whole-MB transform
                        // RDO used by the normal finalist stage.  This captures
                        // transform interactions and the decoder's 4-MV chroma
                        // prediction instead of falling back to a luma-SAD proxy.
                        MotionVector one_pred=rate_pi.pre;
                        uint64_t one_bits=0;
                        if (rate_pi.hybrid) {
                            one_pred=modular_mv_distance(one,rate_pi.a)<=modular_mv_distance(one,rate_pi.c)
                                ? rate_pi.a : rate_pi.c;
                            ++one_bits;
                        }
                        one_bits+=motion_vector_rate_bits(one,one_pred,ProgressiveMvMode::MixedMv,true);

                        uint64_t four_bits=1; // MVTYPEMB contribution; bitplane amortized conservatively.
                        for (int k=0;k<4;++k) {
                            const auto pi4=predictor_info_4mv(out.block_mvs,out.use_intra,mbw,mbh,mx,my,k);
                            MotionVector pred=pi4.pre;
                            const MotionVector d=four[static_cast<size_t>(k)];
                            if (pi4.hybrid) {
                                pred=modular_mv_distance(d,pi4.a)<=modular_mv_distance(d,pi4.c)?pi4.a:pi4.c;
                                ++four_bits;
                            }
                            four_bits+=motion_vector_rate_bits(d,pred,ProgressiveMvMode::MixedMv,true);
                        }
                        const int decision_index=(c_.pqindex<=8)?1:0;
                        const int coding_set=chroma_coding_set(decision_index,c_.pqindex);
                        const bool use_vlc=c_.ac_mode!=AcMode::Esc3;
                        const double q=static_cast<double>(picture_double_quant(c_.pqindex,false));
                        const long double base_lambda=0.75L*static_cast<long double>(q)*static_cast<long double>(q);

                        motion_compensate_mb(*me_rd_pred,ref,mx,my,one,ProgressiveMvMode::MixedMv);
                        const MbTransformDecision one_tx=choose_transform_mb(f,*me_rd_pred,mx,my,coding_set,use_vlc,c_.pqindex,false);
                        const double one_lscale=perceptual_lambda_scale(f.y,c_.width,c_.height,
                                                                       mx*16,my*16,16,16,&me_rd_pred->y);
                        const long double one_rd=static_cast<long double>(one_tx.distortion)+
                            base_lambda*static_cast<long double>(one_lscale)*
                            static_cast<long double>(one_tx.estimated_bits+one_bits);

                        motion_compensate_4mv(*me_rd_pred,ref,mx,my,four);
                        const MbTransformDecision four_tx=choose_transform_mb(f,*me_rd_pred,mx,my,coding_set,use_vlc,c_.pqindex,false);
                        const double four_lscale=perceptual_lambda_scale(f.y,c_.width,c_.height,
                                                                        mx*16,my*16,16,16,&me_rd_pred->y);
                        const long double four_rd=static_cast<long double>(four_tx.distortion)+
                            base_lambda*static_cast<long double>(four_lscale)*
                            static_cast<long double>(four_tx.estimated_bits+four_bits);
                        choose4=four_rd<one_rd;
                    } else {
                        choose4=four_sad + static_cast<uint64_t>(std::llround(lambda*extra_mv_bits)) + 8 <
                                static_cast<uint64_t>(std::llround(static_cast<double>(one_sad)*0.985));
                    }
                    if (choose4) {
                        out.block_mvs[pos]=four;
                        out.use_4mv[pos]=1;
                        out.mvs[pos]=four[0]; // B direct uses the collocated block-0 decoder MV.
                        ++out.four_mv_macroblocks;
                        chosen_sad=four_sad;
                        temporal_screen_sad=four_screen_sad;
                    } else {
                        out.block_mvs[pos].fill(one);
                        out.mvs[pos]=one;
                        temporal_screen_sad=one_screen_sad;
                    }
                }

                // Group B whole-macroblock P-intra decision.  Compare the best
                // motion-compensated SAD with a cheap four-8x8 DC/activity proxy.
                // The strong hysteresis keeps intra for genuinely new/local-cut
                // material rather than stealing ordinary textured inter blocks.
                const uint64_t temporal_sad=chosen_sad;
                out.screen_sads[pos]=temporal_screen_sad;
                uint64_t intra_sad=0;
                if (allow_intra && !c_.debug_disable_p_intra) {
                    for (int k=0;k<4;++k) {
                        const int bx=x0+(k&1)*8, by=y0+((k>>1)&1)*8;
                        const int mean=visible_block_mean(f.y,c_.width,c_.height,bx,by);
                        for (int yy=0;yy<8 && by+yy<c_.height;++yy)
                            for (int xx=0;xx<8 && bx+xx<c_.width;++xx)
                                intra_sad+=static_cast<uint64_t>(std::abs(
                                    static_cast<int>(f.y[static_cast<size_t>(by+yy)*c_.width+bx+xx])-mean));
                    }
                    const uint64_t intra_rate=static_cast<uint64_t>(std::llround(lambda*72.0));
                    // Edge-safe motion is a normative padding constraint, not
                    // evidence that the visible content became spatial.  Keep
                    // the P-intra escape decision tied to the unconstrained
                    // screening SAD so padding hygiene cannot spuriously turn
                    // edge macroblocks into intra blocks.
                    const double ratio=temporal_screen_sad>0
                        ? static_cast<double>(intra_sad+intra_rate)/static_cast<double>(temporal_screen_sad)
                        : std::numeric_limits<double>::infinity();
                    out.inter_intra_cost_ratio[pos]=ratio;
                    if (intra_sad+intra_rate < static_cast<uint64_t>(std::llround(temporal_screen_sad*c_.inter_intra_threshold))) {
                        out.use_intra[pos]=1;
                        out.use_4mv[pos]=0;
                        out.mvs[pos]=MotionVector{};
                        out.block_mvs[pos].fill(MotionVector{});
                        ++out.intra_macroblocks;
                        chosen_sad=intra_sad;
                    }
                }
                bool moved=false,frac=false;
                for (const auto mv:out.block_mvs[pos]) {
                    moved |= mv.xq!=0 || mv.yq!=0;
                    frac |= (mv.xq%8)!=0 || (mv.yq%8)!=0;
                }
                if (moved) ++out.moved_macroblocks;
                if (frac) ++out.fractional_chroma_macroblocks;
                total_sad += chosen_sad;
                rate_sad_total += temporal_sad;

                total_pixels += static_cast<uint64_t>(std::max(0,std::min(16,c_.width-x0))) *
                                static_cast<uint64_t>(std::max(0,std::min(16,c_.height-y0)));
            }
        }
        const double local_mad=total_pixels ? static_cast<double>(total_sad)/static_cast<double>(total_pixels) : 0.0;
        const double global_mad=shared_seed ? local_mad : ((c_.width>0 && c_.height>0)
            ? static_cast<double>(global.full_sad)/static_cast<double>(static_cast<uint64_t>(c_.width)*c_.height)
            : 0.0);
        // The scene detector intentionally runs after motion search.  Local MB
        // motion handles object motion; a robust frame-translation candidate
        // prevents a camera pan from looking like a cut when repetitive detail
        // fools one of the local coarse searches.
        out.mean_abs_residual=std::min(local_mad,global_mad);
        const double rate_local_mad=total_pixels ? static_cast<double>(rate_sad_total)/static_cast<double>(total_pixels) : 0.0;
        out.rate_complexity=std::min(rate_local_mad,global_mad);
        out.motion_sad=total_sad;
        return out;
    }

uint64_t Vc1Encoder::estimate_p_motion_bits(const PAnalysis& a,bool intensity) const {
        const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
        const size_t mbs=static_cast<size_t>(mbw)*mbh;
        std::vector<MvDataSyntax> vals;
        vals.reserve(mbs*4);
        uint64_t bits=2; // MVTAB selector itself
        const bool mixed=mv_mode_mixed(a.mv_mode);
        if (intensity) {
            bits+=unary_mode_bits(3,4)+unary_mode_bits(p_mv_mode_index(a.mv_mode,c_.pqindex,true),3)+12;
        } else bits+=unary_mode_bits(p_mv_mode_index(a.mv_mode,c_.pqindex,false),4);
        if (mixed) bits+=choose_bitplane(a.use_4mv,mbw,mbh).total_bits;
        for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
            const size_t pos=static_cast<size_t>(my)*mbw+mx;
            if (a.use_intra[pos]) { vals.push_back(MvDataSyntax{0,0,true,true}); continue; }
            if (mixed && a.use_4mv[pos]) {
                for (int k=0;k<4;++k) {
                    const auto pi=predictor_info_4mv(a.block_mvs,a.use_intra,mbw,mbh,mx,my,k);
                    MotionVector pred=pi.pre;
                    const MotionVector d=a.block_mvs[pos][static_cast<size_t>(k)];
                    if (pi.hybrid) {
                        pred=modular_mv_distance(d,pi.a)<=modular_mv_distance(d,pi.c)?pi.a:pi.c;
                        ++bits;
                    }
                    vals.push_back(mvdata_for_mode(d,pred,ProgressiveMvMode::MixedMv,true));
                }
            } else {
                const auto pi=predictor_info_mixed_1mv(a.block_mvs,a.use_intra,mbw,mbh,mx,my);
                MotionVector pred=pi.pre;
                const MotionVector d=a.mvs[pos];
                if (pi.hybrid) {
                    pred=modular_mv_distance(d,pi.a)<=modular_mv_distance(d,pi.c)?pi.a:pi.c;
                    ++bits;
                }
                vals.push_back(mvdata_for_mode(d,pred,a.mv_mode,true));
            }
        }
        const int table=choose_mv_table(vals);
        for (const auto& v:vals) {
            const int sym=mv_symbol(v);
            bits+=groupa::kMvDiffBits[static_cast<size_t>(table)][static_cast<size_t>(sym)]+mv_suffix_bits(v);
        }
        return bits;
    }

long double Vc1Encoder::sampled_p_mode_rd(const Frame& f,const Frame& ref,const PAnalysis& a,uint64_t motion_bits,Frame& pred,
                                           const Vc1Encoder& neutral_eval) const {
        const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
        const int total=mbw*mbh;
        if (total<=0) return 0.0L;
        const Frame& prediction_ref=(a.intensity.enabled && a.compensated_reference) ? *a.compensated_reference : ref;
        const int decision_index=(c_.pqindex<=8)?1:0;
        const int coding_set=chroma_coding_set(decision_index,c_.pqindex);
        const bool use_vlc=c_.ac_mode!=AcMode::Esc3;
        const double q=static_cast<double>(picture_double_quant(c_.pqindex,false));
        const double lambda=0.75*q*q;
        // Motion-mode selection itself is AQ-neutral. AQ must refine residual
        // allocation *after* the prediction law is chosen; otherwise AQ/no-AQ
        // can select different picture-level interpolation modes and the mode
        // change can overwhelm the perceptual protection it is meant to add.
        // Using a neutral transform evaluator keeps the prediction decision
        // stable while the final encode still applies full AQ/DQUANT RDO.
        const int wanted=std::min(32,total);
        const int stride=std::max(1,total/wanted);
        long double rd=0.0L;
        int sampled=0;
        for (int linear=0;linear<total && sampled<wanted;linear+=stride) {
            const int my=linear/mbw,mx=linear%mbw;
            const size_t pos=static_cast<size_t>(linear);
            if (a.use_4mv[pos]) motion_compensate_4mv(pred,prediction_ref,mx,my,a.block_mvs[pos]);
            else motion_compensate_mb(pred,prediction_ref,mx,my,a.mvs[pos],a.mv_mode);
            const MbTransformDecision tx=neutral_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false);
            rd+=static_cast<long double>(tx.distortion)+lambda*static_cast<long double>(tx.estimated_bits);
            ++sampled;
        }
        if (!sampled) return 0.0L;
        const long double residual_per_mb=rd/static_cast<long double>(sampled);
        const long double motion_per_mb=static_cast<long double>(motion_bits)/static_cast<long double>(total);
        return residual_per_mb+lambda*motion_per_mb;
    }

PAnalysis Vc1Encoder::analyze_p_picture(const Frame& f, const Frame& ref, const PAnalysis* external_qpel_seed) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::PAnalysis);
        const std::array<ProgressiveMvMode,4> modes={{ProgressiveMvMode::OneMvQpel,
            ProgressiveMvMode::OneMvHpel,ProgressiveMvMode::OneMvHpelBilinear,ProgressiveMvMode::MixedMv}};
        // Main-profile B prediction has a separate 32-unit pullback/modulo
        // predictor. With two B pictures (1/3 and 2/3 temporal positions), a
        // half-pel P anchor can expose a remaining decoder-model corner case.
        // Keep Main anchors qpel when a two-B group is configured; Main P-only
        // and one-B encodes may still RDO the three legal 1-MV modes. Advanced
        // Profile has decoder-exact support for the full set.
        const int n=c_.syntax==StreamSyntax::Advanced ? 4
            : (c_.max_b_frames>=2 ? 1 : 3);

        // 0.1.72: ABR/VBV uses the same progressive motion-mode RDO as CQP.
        // The qpel full-search seed may now be supplied by the provisional ABR
        // pass so that search is not launched a second time. Keep the historical
        // candidate construction and tie-breaking exactly the same after that.
        PAnalysis owned_qseed;
        if (!external_qpel_seed) {
            owned_qseed=analyze_p_picture_core(f,ref,ProgressiveMvMode::OneMvQpel,false,nullptr);
            external_qpel_seed=&owned_qseed;
        }
        const size_t mbs=static_cast<size_t>((c_.width+15)/16)*static_cast<size_t>((c_.height+15)/16);
        if (external_qpel_seed->mv_mode!=ProgressiveMvMode::OneMvQpel || external_qpel_seed->mvs.size()!=mbs)
            throw std::runtime_error("invalid shared qpel P-analysis seed");
        const PAnalysis& qseed=*external_qpel_seed;

        // Reuse one AQ-neutral transform evaluator and one full-frame prediction
        // scratch across every candidate (including the weighted fade pass).
        // The evaluator configuration is mode-independent and constructing it
        // separately for each sampled score was pure repeated setup.
        EncoderConfig neutral_cfg=c_;
        neutral_cfg.adaptive_quality=false;
        neutral_cfg.aq_strength=0.0;
        const Vc1Encoder neutral_eval(neutral_cfg);
        Frame pred_scratch=empty_frame();
        PAnalysis best_seed;
        ProgressiveMvMode best_mode=ProgressiveMvMode::OneMvQpel;
        long double best_score=std::numeric_limits<long double>::infinity();
        for (int i=0;i<n;++i) {
            PAnalysis cand=(i==0) ? qseed
                : analyze_p_picture_core(f,ref,modes[static_cast<size_t>(i)],false,&qseed);
            cand.motion_bits=estimate_p_motion_bits(cand,false);
            const long double sc=sampled_p_mode_rd(f,ref,cand,cand.motion_bits,pred_scratch,neutral_eval);
            if (sc<best_score) { best_score=sc; best_mode=cand.mv_mode; best_seed=std::move(cand); }
        }
        PAnalysis best=analyze_p_picture_core(f,ref,best_mode,true,&best_seed);
        best.motion_bits=estimate_p_motion_bits(best,false);
        if (!c_.fade_compensation) return best;

        const IntensityComp ic=estimate_intensity_comp(f,ref,best.mvs);
        if (!ic.enabled) return best;
        auto compensated=std::make_shared<Frame>(intensity_compensated_reference(ref,ic));
        PAnalysis wqseed=analyze_p_picture_core(f,*compensated,ProgressiveMvMode::OneMvQpel,false,nullptr);
        PAnalysis weighted_seed;
        ProgressiveMvMode weighted_mode=ProgressiveMvMode::OneMvQpel;
        long double weighted_score=std::numeric_limits<long double>::infinity();
        for (int i=0;i<n;++i) {
            PAnalysis cand=(i==0) ? wqseed
                : analyze_p_picture_core(f,*compensated,modes[static_cast<size_t>(i)],false,&wqseed);
            cand.intensity=ic;
            cand.compensated_reference=compensated;
            cand.motion_bits=estimate_p_motion_bits(cand,true);
            const long double sc=sampled_p_mode_rd(f,ref,cand,cand.motion_bits,pred_scratch,neutral_eval);
            if (sc<weighted_score) { weighted_score=sc; weighted_mode=cand.mv_mode; weighted_seed=std::move(cand); }
        }
        const double need=std::max(0.25,best.mean_abs_residual*c_.fade_min_gain);
        if (weighted_seed.mean_abs_residual+need>=best.mean_abs_residual || weighted_score>=best_score) return best;
        PAnalysis weighted=analyze_p_picture_core(f,*compensated,weighted_mode,true,&weighted_seed);
        weighted.intensity=ic;
        weighted.compensated_reference=std::move(compensated);
        weighted.motion_bits=estimate_p_motion_bits(weighted,true);
        return weighted;
    }

double Vc1Encoder::scene_change_score(const Frame& f, const Frame& ref) const {
        if (f.width != c_.width || f.height != c_.height ||
            ref.width != c_.width || ref.height != c_.height)
            throw std::runtime_error("scene-analysis dimensions do not match encoder configuration");
        const auto g=global_motion_search(f,ref,c_.motion_search_range);
        const uint64_t pixels=static_cast<uint64_t>(c_.width)*c_.height;
        double best=pixels?static_cast<double>(g.full_sad)/static_cast<double>(pixels):0.0;
        if (c_.fade_compensation) {
            const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
            std::vector<MotionVector> uniform(static_cast<size_t>(mbw)*mbh,MotionVector{g.dx*4,g.dy*4});
            const IntensityComp ic=estimate_intensity_comp(f,ref,uniform);
            if (ic.enabled) {
                const Frame weighted=intensity_compensated_reference(ref,ic);
                const auto wg=global_motion_search(f,weighted,c_.motion_search_range);
                const double wm=pixels?static_cast<double>(wg.full_sad)/static_cast<double>(pixels):0.0;
                best=std::min(best,wm);
            }
        }

        // A single global translation cannot model a camera zoom.  If the
        // global/fade-compensated score still looks like a cut, make one
        // candidate-only local-motion check before inserting an I picture.
        // This reuses the normal P-picture motion model and therefore rescues
        // coherent zooms/other locally explainable motion, while ordinary
        // non-candidate frames pay no extra motion-search cost.  Require both
        // a substantial relative improvement and comfortable margin below the
        // scene threshold so unrelated cuts are not hidden by chance block
        // matches.  This check is independent of fade compensation.
        if (best>=c_.scene_threshold) {
            const PAnalysis local=analyze_p_picture_core(f,ref,ProgressiveMvMode::OneMvQpel,false,nullptr);
            const double lm=local.mean_abs_residual;

            // Low block-matching residual alone is not enough: unrelated cuts
            // can contain repetitive texture that finds accidental local
            // matches.  Fit the interior MV field to translation + isotropic
            // scale (a camera zoom about any center).  Require most vectors to
            // lie close to that coherent field before using local motion to
            // veto a scene cut.
            const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
            long double mx=0.0L,my=0.0L,mdx=0.0L,mdy=0.0L;
            size_t samples=0;
            for (int by=1;by<mbh-1;++by) for (int bx=1;bx<mbw-1;++bx) {
                const auto mv=local.mvs[static_cast<size_t>(by)*mbw+bx];
                mx+=bx*16.0L+8.0L; my+=by*16.0L+8.0L;
                mdx+=mv.xq/4.0L; mdy+=mv.yq/4.0L; ++samples;
            }
            bool coherent=false;
            if (samples>=12) {
                mx/=samples; my/=samples; mdx/=samples; mdy/=samples;
                long double num=0.0L,den=0.0L;
                for (int by=1;by<mbh-1;++by) for (int bx=1;bx<mbw-1;++bx) {
                    const auto mv=local.mvs[static_cast<size_t>(by)*mbw+bx];
                    const long double x=bx*16.0L+8.0L-mx,y=by*16.0L+8.0L-my;
                    const long double dx=mv.xq/4.0L-mdx,dy=mv.yq/4.0L-mdy;
                    num+=x*dx+y*dy; den+=x*x+y*y;
                }
                const long double z=den>0.0L?num/den:0.0L;
                size_t inliers=0;
                for (int by=1;by<mbh-1;++by) for (int bx=1;bx<mbw-1;++bx) {
                    const auto mv=local.mvs[static_cast<size_t>(by)*mbw+bx];
                    const long double x=bx*16.0L+8.0L-mx,y=by*16.0L+8.0L-my;
                    const long double ex=mv.xq/4.0L-(mdx+z*x);
                    const long double ey=mv.yq/4.0L-(mdy+z*y);
                    if (ex*ex+ey*ey<=6.25L) ++inliers; // <=2.5 px from zoom field.
                }
                // Translation is already handled by global_motion_search().
                // This rescue is specifically for scale change, so require a
                // measurable but still plausible per-frame zoom slope as well
                // as a coherent field.  That prevents repetitive local motion
                // or accidental block matches across a real cut from bypassing
                // the scene detector.
                const long double az=std::abs(z);
                coherent=inliers*5>=samples*3 && az>=0.005L && az<=0.08L;
            }
            if (coherent && lm<best*0.72 && lm<c_.scene_threshold*0.86) best=lm;
        }
        return best;
    }

bool Vc1Encoder::has_good_motion_match(const Frame& f,const Frame& ref) const {
        if (f.width!=c_.width || f.height!=c_.height || ref.width!=c_.width || ref.height!=c_.height)
            throw std::runtime_error("motion-failure analysis dimensions do not match encoder configuration");
        const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
        const size_t mbs=static_cast<size_t>(mbw)*mbh;
        const int range=std::max(0,c_.motion_search_range);
        const int local_range=std::min(range,std::max(0,c_.motion_local_search_range));
        const Frame* prediction_ref=&ref;
        std::shared_ptr<Frame> weighted;
        GlobalMotionResult global=global_motion_search(f,ref,local_range);
        if (c_.fade_compensation) {
            std::vector<MotionVector> uniform(mbs,MotionVector{global.dx*4,global.dy*4});
            const IntensityComp ic=estimate_intensity_comp(f,ref,uniform);
            if (ic.enabled) {
                weighted=std::make_shared<Frame>(intensity_compensated_reference(ref,ic));
                prediction_ref=weighted.get();
                global=global_motion_search(f,*prediction_ref,local_range);
            }
        }
        std::vector<MotionVector> field(mbs,MotionVector{});
        std::vector<uint8_t> propagated(mbs,0);
        std::unique_ptr<LongRangeIndex> long_index;
        for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
            const size_t pos=static_cast<size_t>(my)*mbw+mx;
            const auto pi=predictor_info(field,mbw,mbh,mx,my);
            IntegerMotionResult im{};
            bool selected_distant=false;
            auto better_integer=[](const IntegerMotionResult& a,const IntegerMotionResult& b) {
                if (a.full_sad!=b.full_sad) return a.full_sad<b.full_sad;
                const int am=std::abs(a.dx)+std::abs(a.dy),bm=std::abs(b.dx)+std::abs(b.dy);
                if (am!=bm) return am<bm;
                if (a.dy!=b.dy) return a.dy<b.dy;
                return a.dx<b.dx;
            };
            if (c_.long_range_search_mode==VC1_LONG_RANGE_LEGACY_DISTANT_FIRST) {
                bool done=false;
                if (range>local_range) {
                    const auto pr=propagated_long_range_motion(f,*prediction_ref,mx,my,mbw,field,propagated,range);
                    if (motion_match_good(pr.full_sad,mx,my)) { im=pr; done=true; selected_distant=true; }
                }
                if (!done)
                    im=integer_motion_umh(f,*prediction_ref,mx,my,pi,global,local_range);
            } else {
                im=integer_motion_umh(f,*prediction_ref,mx,my,pi,global,local_range);
                const bool skip_distant=
                    c_.long_range_search_mode==VC1_LONG_RANGE_LOCAL_GOOD_SKIP &&
                    motion_match_good(im.full_sad,mx,my);
                if (range>local_range && !skip_distant) {
                    const auto pr=propagated_long_range_motion(f,*prediction_ref,mx,my,mbw,field,propagated,range);
                    if (better_integer(pr,im) && distant_match_decision(pr.full_sad,im.full_sad,mx,my)>=2) { im=pr; selected_distant=true; }
                }
            }
            if (range>local_range && !motion_match_good(im.full_sad,mx,my)) {
                if (!long_index) long_index=std::make_unique<LongRangeIndex>(build_long_range_index(*prediction_ref));
                const auto ext=extended_content_motion(f,*prediction_ref,mx,my,range,*long_index,im);
                if (ext.dx!=im.dx || ext.dy!=im.dy) { im=ext; selected_distant=true; }
            }
            if (selected_distant) propagated[pos]=1;
            uint64_t refined_sad=im.full_sad;
            const int x0=mx*16,y0=my*16;
            const MotionVector mv=refine_motion_block(f,*prediction_ref,x0,y0,16,16,
                MotionVector{im.dx*4,im.dy*4},range,ProgressiveMvMode::OneMvQpel,
                &refined_sad,im.full_sad);
            field[pos]=mv;
            const uint64_t pixels=active_mb_pixels(mx,my);
            // This is an emergency-I safety criterion, deliberately looser
            // than the <=6-level long-range-search early-out. One reasonably
            // useful block is enough to keep the legal inter path available.
            const uint64_t zero_sad=mb_sad(f,*prediction_ref,mx,my,0,0,1);
            if (pixels && (refined_sad<=pixels*12u ||
                           (zero_sad>0 && refined_sad*10u<=zero_sad*9u))) return true;
        }
        return false;
    }

BAnalysis Vc1Encoder::analyze_b_picture_mode(const Frame& f, const Frame& past, const Frame& future,
                                const std::vector<MotionVector>& future_anchor_mvs,
                                const std::vector<uint8_t>& future_anchor_4mv,
                                const IntensityComp& forward_ic,
                                int fraction_num, int fraction_den, ProgressiveMvMode mode) const {
        const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
        const size_t mbs=static_cast<size_t>(mbw)*mbh;
        if (future_anchor_mvs.size()!=mbs || future_anchor_4mv.size()!=mbs)
            throw std::runtime_error("B analysis needs the future anchor's collocated P MV/type fields");
        if (!((fraction_den==2 && fraction_num==1) ||
              (fraction_den==3 && (fraction_num==1 || fraction_num==2))))
            throw std::runtime_error("unsupported B-picture temporal fraction");

        // The deliberately minimal configuration used by --fastest does not
        // spend CPU choosing among B prediction modes.  Preserve B pictures and
        // legal syntax, but make every macroblock forward-predicted with zero
        // forward/backward vector state.  Residual coding still corrects the
        // prediction, so this remains a real lossy video mode rather than a
        // stream of empty predictive pictures.
        const bool minimal_zero_motion = c_.motion_search_range==0 && !c_.fade_compensation &&
            !c_.loop_filter && !c_.variable_transforms && c_.trellis==0 && !c_.adaptive_quality;
        if (minimal_zero_motion) {
            BAnalysis out;
            out.mv_mode=mode;
            out.modes.assign(mbs,BMbMode::Forward);
            out.forward_mvs.assign(mbs,MotionVector{});
            out.backward_mvs.assign(mbs,MotionVector{});
            out.inter_intra_cost_ratio.assign(mbs,-1.0);
            out.forward_macroblocks=mbs;
            return out;
        }

        std::shared_ptr<Frame> weighted_past;
        const Frame* past_prediction=&past;
        if (forward_ic.enabled) {
            weighted_past=std::make_shared<Frame>(intensity_compensated_reference(past,forward_ic));
            past_prediction=weighted_past.get();
        }
        // B pictures cannot signal intensity compensation themselves. The forward
        // reference inherits the LUT signaled by the future P anchor; backward
        // prediction from that future anchor is unweighted.
        const PAnalysis fwd=analyze_p_picture_core(f,*past_prediction,mode,false,nullptr);
        const PAnalysis bwd=analyze_p_picture_core(f,future,mode,false,nullptr);
        BAnalysis out;
        out.mv_mode=mode;
        out.forward_intensity=forward_ic;
        out.compensated_past=std::move(weighted_past);
        out.modes.resize(mbs,BMbMode::Forward);
        out.forward_mvs.resize(mbs);
        out.backward_mvs.resize(mbs);
        out.inter_intra_cost_ratio.assign(mbs,-1.0);
        out.forward_distant_local_mae=fwd.distant_local_mae;
        out.forward_distant_candidate_mae=fwd.distant_candidate_mae;
        out.forward_distant_match_decision=fwd.distant_match_decision;
        out.backward_distant_local_mae=bwd.distant_local_mae;
        out.backward_distant_candidate_mae=bwd.distant_candidate_mae;
        out.backward_distant_match_decision=bwd.distant_match_decision;

        const int bfrac = fraction_den==2 ? 128 : (fraction_num==1 ? 85 : 170);
        const bool below_half=(2*fraction_num < fraction_den);
        auto scale_direct=[&](MotionVector collocated,bool inverse,int mx,int my) {
            const int n=inverse ? bfrac-256 : bfrac;
            MotionVector mv;
            if (mv_mode_halfpel(mode)) {
                mv.xq=2*arshift(collocated.xq*n+255,9);
                mv.yq=2*arshift(collocated.yq*n+255,9);
            } else {
                mv.xq=arshift(collocated.xq*n+128,8);
                mv.yq=arshift(collocated.yq*n+128,8);
            }
            const int minx=-60-mx*64, maxx=mbw*64-4-mx*64;
            const int miny=-60-my*64, maxy=mbh*64-4-my*64;
            mv.xq=std::clamp(mv.xq,minx,maxx);
            mv.yq=std::clamp(mv.yq,miny,maxy);
            return mv;
        };

        uint64_t selected_sad_total=0;
        uint64_t rate_sad_total=0;
        uint64_t selected_rate_total=1; // progressive B MVMODE bit
        uint64_t selected_pixels_total=0;

        for (int my=0;my<mbh;++my) {
            for (int mx=0;mx<mbw;++mx) {
                const size_t pos=static_cast<size_t>(my)*mbw+mx;
                const MotionVector df=scale_direct(future_anchor_mvs[pos],false,mx,my);
                const MotionVector db=scale_direct(future_anchor_mvs[pos],true,mx,my);
                const MotionVector fm=fwd.mvs[pos], bm=bwd.mvs[pos];

                // Compute the four B-mode distortion candidates in one raster
                // pass.  The old code sampled fm once for Forward and again for
                // Interpolated, and bm once for Backward and again for
                // Interpolated.  Reusing those identical decoder-equivalent
                // samples reduces six luma interpolation calls/pixel to four.
                const int x0=mx*16,y0=my*16;
                uint64_t cf=0,cb_only=0,ci=0,cd=0;
                for (int y=0;y<16 && y0+y<c_.height;++y) for (int x=0;x<16 && x0+x<c_.width;++x) {
                    const int px=x0+x,py=y0+y;
                    const int src=f.y[static_cast<size_t>(py)*c_.width+px];
                    const int pf=luma_mc_sample_mode(past_prediction->y,c_.width,c_.height,px,py,fm.xq,fm.yq,mode,c_.rndctrl);
                    const int pb=luma_mc_sample_mode(future.y,c_.width,c_.height,px,py,bm.xq,bm.yq,mode,c_.rndctrl);
                    const int pdf=luma_mc_sample_mode(past_prediction->y,c_.width,c_.height,px,py,df.xq,df.yq,mode,c_.rndctrl);
                    const int pdb=luma_mc_sample_mode(future.y,c_.width,c_.height,px,py,db.xq,db.yq,mode,c_.rndctrl);
                    cf+=static_cast<uint64_t>(std::abs(src-pf));
                    cb_only+=static_cast<uint64_t>(std::abs(src-pb));
                    ci+=static_cast<uint64_t>(std::abs(src-((pf+pb+1)>>1)));
                    cd+=static_cast<uint64_t>(std::abs(src-((pdf+pdb+1)>>1)));
                }

                // Group B mode RDO: compare prediction error with an estimate
                // of the MVDATA + BMVTYPE syntax consumed by each candidate.
                // Preceding macroblocks already contain their final decoder MV
                // state, so this is still a single analysis pass.
                const MotionVector fpred=c_.syntax==StreamSyntax::Wmv9Main
                    ? predictor_b_main(out.forward_mvs,mbw,mx,my)
                    : predictor_info(out.forward_mvs,mbw,mbh,mx,my).pre;
                const MotionVector bpred=c_.syntax==StreamSyntax::Wmv9Main
                    ? predictor_b_main(out.backward_mvs,mbw,mx,my)
                    : predictor_info(out.backward_mvs,mbw,mbh,mx,my).pre;
                auto mvbits=[&](MotionVector d,MotionVector p,bool more) {
                    const MvDataSyntax mv=mvdata_for_mode(d,p,mode,more);
                    const int sym=mv_symbol(mv);
                    return static_cast<uint64_t>(groupa::kMvDiffBits[0][static_cast<size_t>(sym)] + mv_suffix_bits(mv));
                };
                const uint64_t fbits=mvbits(fm,fpred,false);
                const uint64_t bbits=mvbits(bm,bpred,false);
                const uint64_t ibits=mvbits(bm,bpred,true)+mvbits(fm,fpred,false)+2;
                const uint64_t ftype=(false==below_half)?2u:1u;
                const uint64_t btype=(true==below_half)?2u:1u;
                const double lambda=std::max(1.0,0.45*static_cast<double>(c_.pqindex));
                struct Candidate { BMbMode mode; uint64_t cost; uint64_t rate_bits; };
                // Progressive VC-1 derives the otherwise-unused temporal field of
                // direct/single-direction B macroblocks implicitly from the collocated
                // future-anchor vector.  When that P macroblock is Mixed-MV, decoder
                // implementations do not expose a portable single collocated field for
                // encoder-side reconstruction.  Make both B directions explicit for
                // exactly those macroblocks; ordinary 1-MV collocations retain all four
                // direct/forward/backward/interpolated choices.
                const uint64_t implicit_penalty=future_anchor_4mv[pos]?1000000000ull:0ull;
                const std::array<Candidate,4> cand={{
                    {BMbMode::Direct,cd+implicit_penalty,0},
                    {BMbMode::Forward,cf+implicit_penalty,fbits+ftype},
                    {BMbMode::Backward,cb_only+implicit_penalty,bbits+btype},
                    {BMbMode::Interpolated,ci,ibits}
                }};
                const Candidate* best=&cand[0];
                auto score=[&](const Candidate& c) { return static_cast<long double>(c.cost)+lambda*c.rate_bits; };
                for (const auto& c:cand) if (score(c)<score(*best)) best=&c;

                // Full progressive B-macroblock intra decision.  Use the same
                // deliberately strong 20% hysteresis as P-intra: the spatial
                // four-8x8 mean/activity proxy must decisively beat the best
                // temporal prediction after an intra syntax allowance.  This
                // makes B-intra a local-cut/new-object escape rather than a
                // substitute for ordinary motion compensation.
                uint64_t intra_sad=0;
                for (int k=0;k<4;++k) {
                    const int bx=x0+(k&1)*8,by=y0+((k>>1)&1)*8;
                    const int mean=visible_block_mean(f.y,c_.width,c_.height,bx,by);
                    for (int yy=0;yy<8 && by+yy<c_.height;++yy)
                        for (int xx=0;xx<8 && bx+xx<c_.width;++xx)
                            intra_sad+=static_cast<uint64_t>(std::abs(
                                static_cast<int>(f.y[static_cast<size_t>(by+yy)*c_.width+bx+xx])-mean));
                }
                const uint64_t intra_guard=static_cast<uint64_t>(std::llround(lambda*72.0));
                // Forward/backward analysis may intentionally pick a SATD/RD
                // finalist whose raw SAD is slightly above the SAD screening
                // winner.  The B-intra proxy is itself SAD-based, so gate it
                // against the best temporal SAD discovered in either reference
                // as well as the selected B-mode cost.  Otherwise enabling RD
                // alone can spuriously turn edge macroblocks into intra blocks.
                const uint64_t temporal_gate=std::min({best->cost,
                    fwd.screen_sads[pos],bwd.screen_sads[pos]});
                const double intra_ratio=temporal_gate>0
                    ? static_cast<double>(intra_sad+intra_guard)/static_cast<double>(temporal_gate)
                    : std::numeric_limits<double>::infinity();
                out.inter_intra_cost_ratio[pos]=intra_ratio;
                const bool choose_intra=!c_.debug_disable_b_intra && intra_sad+intra_guard <
                    static_cast<uint64_t>(std::llround(static_cast<double>(temporal_gate)*c_.inter_intra_threshold));
                const uint64_t intra_marker_bits=static_cast<uint64_t>(std::max(
                    groupa::kMvDiffBits[0][35],groupa::kMvDiffBits[0][72]))+1u; // MVDATA + ACPRED

                const uint64_t chosen_sad=choose_intra?intra_sad:best->cost;
                const uint64_t chosen_rate=choose_intra?intra_marker_bits:best->rate_bits;
                selected_sad_total += chosen_sad;
                rate_sad_total += best->cost;
                selected_rate_total += chosen_rate;
                selected_pixels_total += static_cast<uint64_t>(std::max(0,std::min(16,c_.width-mx*16))) *
                                         static_cast<uint64_t>(std::max(0,std::min(16,c_.height-my*16)));
                if (choose_intra) {
                    out.modes[pos]=BMbMode::Intra;
                    // ff_vc1_pred_b_mv() zeros both decoder temporal fields for
                    // an intra B macroblock, so keep the encoder predictor state
                    // identical for the following macroblocks.
                    out.forward_mvs[pos]=MotionVector{};
                    out.backward_mvs[pos]=MotionVector{};
                    ++out.intra_macroblocks;
                } else {
                    out.modes[pos]=best->mode;
                    if (best->mode==BMbMode::Direct) {
                        out.forward_mvs[pos]=df; out.backward_mvs[pos]=db; ++out.direct_macroblocks;
                    } else if (best->mode==BMbMode::Forward) {
                        out.forward_mvs[pos]=fm; out.backward_mvs[pos]=db; ++out.forward_macroblocks;
                    } else if (best->mode==BMbMode::Backward) {
                        out.forward_mvs[pos]=df; out.backward_mvs[pos]=bm; ++out.backward_macroblocks;
                    } else {
                        out.forward_mvs[pos]=fm; out.backward_mvs[pos]=bm; ++out.interpolated_macroblocks;
                    }
                }
                if (out.forward_mvs[pos].xq || out.forward_mvs[pos].yq ||
                    out.backward_mvs[pos].xq || out.backward_mvs[pos].yq) ++out.moved_macroblocks;
                auto frac_chroma=[](MotionVector mv) { return (mv.xq&3) || (mv.yq&3) || ((mv.xq/4)&1) || ((mv.yq/4)&1); };
                if (frac_chroma(out.forward_mvs[pos]) || frac_chroma(out.backward_mvs[pos]))
                    ++out.fractional_chroma_macroblocks;
            }
        }
        out.mean_abs_residual = selected_pixels_total
            ? static_cast<double>(selected_sad_total)/static_cast<double>(selected_pixels_total) : 0.0;
        out.rate_complexity = selected_pixels_total
            ? static_cast<double>(rate_sad_total)/static_cast<double>(selected_pixels_total) : 0.0;
        out.motion_sad=selected_sad_total;
        out.motion_bits=selected_rate_total;
        return out;
    }

long double Vc1Encoder::sampled_b_mode_rd(const Frame& f,const Frame& past,const Frame& future,const BAnalysis& a,Frame& pred,
                                           const Vc1Encoder& neutral_eval) const {
        const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
        const int total=mbw*mbh;
        if (total<=0) return 0.0L;
        const Frame& forward_ref=(a.forward_intensity.enabled && a.compensated_past) ? *a.compensated_past : past;
        const int decision_index=(c_.pqindex<=8)?1:0;
        const int coding_set=chroma_coding_set(decision_index,c_.pqindex);
        const bool use_vlc=c_.ac_mode!=AcMode::Esc3;
        const double q=static_cast<double>(picture_double_quant(c_.pqindex,false));
        const double lambda=0.75*q*q;
        // Motion-mode selection itself is AQ-neutral. AQ must refine residual
        // allocation *after* the prediction law is chosen; otherwise AQ/no-AQ
        // can select different picture-level interpolation modes and the mode
        // change can overwhelm the perceptual protection it is meant to add.
        // Using a neutral transform evaluator keeps the prediction decision
        // stable while the final encode still applies full AQ/DQUANT RDO.
        const int wanted=std::min(32,total);
        const int stride=std::max(1,total/wanted);
        long double rd=0.0L;
        int sampled=0;
        for (int linear=0;linear<total && sampled<wanted;linear+=stride) {
            const int my=linear/mbw,mx=linear%mbw;
            const size_t pos=static_cast<size_t>(linear);
            if (a.modes[pos]==BMbMode::Intra) {
                // Give picture-level qpel/halfpel selection a spatial predictor
                // for B-intra samples.  The actual intra encoder uses DC+AC
                // coding, but a block-mean predictor supplies a stable neutral
                // residual proxy while the extra 72-bit term models intra syntax.
                for (int k=0;k<6;++k) {
                    const std::vector<uint8_t>* src=nullptr; std::vector<uint8_t>* dst=nullptr;
                    int sw=0,sh=0,dw=0,dh=0,bx=0,by=0;
                    if (k<4) { src=&f.y; dst=&pred.y; sw=c_.width; sh=c_.height; dw=pred.width; dh=pred.height; bx=mx*2+(k&1); by=my*2+((k>>1)&1); }
                    else { src=(k==4)?&f.u:&f.v; dst=(k==4)?&pred.u:&pred.v; sw=c_.width/2; sh=c_.height/2; dw=pred.width/2; dh=pred.height/2; bx=mx; by=my; }
                    const int mean=visible_block_mean(*src,sw,sh,bx*8,by*8);
                    for (int yy=0;yy<8 && by*8+yy<dh;++yy)
                        for (int xx=0;xx<8 && bx*8+xx<dw;++xx)
                            (*dst)[static_cast<size_t>(by*8+yy)*dw+bx*8+xx]=static_cast<uint8_t>(mean);
                }
            } else if (a.modes[pos]==BMbMode::Forward)
                motion_compensate_mb(pred,forward_ref,mx,my,a.forward_mvs[pos],a.mv_mode);
            else if (a.modes[pos]==BMbMode::Backward)
                motion_compensate_mb(pred,future,mx,my,a.backward_mvs[pos],a.mv_mode);
            else
                motion_compensate_bi_mb(pred,forward_ref,future,mx,my,a.forward_mvs[pos],a.backward_mvs[pos],a.mv_mode);
            const MbTransformDecision tx=neutral_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false);
            rd+=static_cast<long double>(tx.distortion)+lambda*static_cast<long double>(tx.estimated_bits);
            if (a.modes[pos]==BMbMode::Intra) rd+=lambda*72.0L;
            ++sampled;
        }
        if (!sampled) return 0.0L;
        const long double residual_per_mb=rd/static_cast<long double>(sampled);
        const long double motion_per_mb=static_cast<long double>(a.motion_bits)/static_cast<long double>(total);
        return residual_per_mb+lambda*motion_per_mb;
    }

BAnalysis Vc1Encoder::analyze_b_picture(const Frame& f, const Frame& past, const Frame& future,
                                const std::vector<MotionVector>& future_anchor_mvs,
                                const std::vector<uint8_t>& future_anchor_4mv,
                                const IntensityComp& forward_ic,
                                int fraction_num, int fraction_den) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::BAnalysis);
        BAnalysis q=analyze_b_picture_mode(f,past,future,future_anchor_mvs,future_anchor_4mv,
                                           forward_ic,fraction_num,fraction_den,ProgressiveMvMode::OneMvQpel);
        // 0.1.72: ABR/VBV and CQP share this qpel-vs-halfpel B-picture RDO.
        // Main-profile B half-pel predictor state has profile-specific pullback
        // and modulo behavior that is not yet independently decoder-equivalent
        // for two-B (1/3,2/3) groups. Keep WMV9/Main on its proven qpel B path
        // in this release; Advanced Profile performs the full qpel-vs-halfpel
        // residual-aware comparison below. P-picture half-pel modes remain
        // available in both profiles.
        if (c_.syntax==StreamSyntax::Wmv9Main) return q;
        // --fastest zero-motion case is intentionally one pass.
        if (c_.motion_search_range==0 && !c_.fade_compensation && !c_.loop_filter &&
            !c_.variable_transforms && c_.trellis==0 && !c_.adaptive_quality) return q;
        BAnalysis h=analyze_b_picture_mode(f,past,future,future_anchor_mvs,future_anchor_4mv,
                                           forward_ic,fraction_num,fraction_den,ProgressiveMvMode::OneMvHpelBilinear);
        EncoderConfig neutral_cfg=c_;
        neutral_cfg.adaptive_quality=false;
        neutral_cfg.aq_strength=0.0;
        const Vc1Encoder neutral_eval(neutral_cfg);
        Frame pred_scratch=empty_frame();
        const long double qs=sampled_b_mode_rd(f,past,future,q,pred_scratch,neutral_eval);
        const long double hs=sampled_b_mode_rd(f,past,future,h,pred_scratch,neutral_eval);
        return hs<qs ? std::move(h) : std::move(q);
    }

MvDataSyntax Vc1Encoder::mvdata_for_mode(MotionVector desired,MotionVector pred,
                                        ProgressiveMvMode mode,bool more,bool intra) {
        const bool halfpel=mv_mode_halfpel(mode);
        if (intra) return MvDataSyntax{0,0,more,true,halfpel};
        int dx=desired.xq-pred.xq,dy=desired.yq-pred.yq;
        if (halfpel) {
            // Progressive half-pel modes code the differential in half-pixel
            // units while predictor state remains in the decoder's qpel units.
            // A half-pel picture's MV state is even by construction, so an odd
            // differential here means the candidate is not representable.
            if ((dx&1) || (dy&1))
                throw std::runtime_error("half-pel MV is not representable from its predictor");
            dx/=2; dy/=2;
        }
        return MvDataSyntax{dx,dy,more,false,halfpel};
    }

int Vc1Encoder::p_mv_mode_index(ProgressiveMvMode mode,int pq,bool secondary) {
        if (secondary) {
            if (pq<=12) {
                if (mode==ProgressiveMvMode::OneMvQpel) return 0;
                if (mode==ProgressiveMvMode::MixedMv) return 1;
                if (mode==ProgressiveMvMode::OneMvHpel) return 2;
                return 3;
            }
            if (mode==ProgressiveMvMode::OneMvHpelBilinear) return 0;
            if (mode==ProgressiveMvMode::OneMvQpel) return 1;
            if (mode==ProgressiveMvMode::OneMvHpel) return 2;
            return 3;
        }
        if (pq<=12) {
            if (mode==ProgressiveMvMode::OneMvQpel) return 0;
            if (mode==ProgressiveMvMode::MixedMv) return 1;
            if (mode==ProgressiveMvMode::OneMvHpel) return 2;
            if (mode==ProgressiveMvMode::OneMvHpelBilinear) return 4;
        } else {
            if (mode==ProgressiveMvMode::OneMvHpelBilinear) return 0;
            if (mode==ProgressiveMvMode::OneMvQpel) return 1;
            if (mode==ProgressiveMvMode::OneMvHpel) return 2;
            if (mode==ProgressiveMvMode::MixedMv) return 4;
        }
        return 3;
    }

int Vc1Encoder::unary_mode_bits(int index,int maximum) { return index<maximum?index+1:maximum; }

void Vc1Encoder::write_unary_mode(BitWriter& b,int index,int maximum) {
        for (int i=0;i<index && i<maximum;++i) b.bit(false);
        if (index<maximum) b.bit(true);
    }

int Vc1Encoder::mv_cat(int v,bool halfpel) {
        const int a=std::abs(v);
        if (!a) return 0;
        if (a<=2) return 1;
        if (a<=6) return 2;
        if (a<=14) return 3;
        if (a<=30) return 4;
        // GET_MVDATA shortens category 5 by one suffix bit when
        // quarter_sample==0.  The largest ordinary half-pel differential is
        // therefore 94 rather than the qpel path's 158; larger values use ESC.
        if (a<=(halfpel?94:158)) return 5;
        return -1;
    }

int Vc1Encoder::mv_symbol(const MvDataSyntax& m) {
        if (m.intra) return m.more ? 72 : 35;
        const int xc=mv_cat(m.dx,m.halfpel), yc=mv_cat(m.dy,m.halfpel);
        if (xc<0 || yc<0) return m.more ? 71 : 34; // long-vector escape
        const int idx=yc*6+xc;
        if (!m.more && idx==0) return 34; // this should normally be skipped; long form is legal.
        return m.more ? idx+36 : idx-1;
    }

int Vc1Encoder::mv_component_bits(int cat,bool halfpel) {
        static constexpr int sz[6]={0,2,3,4,5,8};
        return sz[cat] - ((halfpel && cat==5)?1:0);
    }

int Vc1Encoder::mv_suffix_bits(const MvDataSyntax& m) const {
        if (m.intra) return 0;
        const int sym=mv_symbol(m);
        if (sym==34 || sym==71) {
            const int r=motion_mvrange();
            const int hp=m.halfpel?1:0;
            return (mvrange_kx(r)-hp)+(mvrange_ky(r)-hp);
        }
        const int xc=mv_cat(m.dx,m.halfpel),yc=mv_cat(m.dy,m.halfpel);
        // mv_symbol() maps any out-of-range component to the long-vector
        // escape above. Keep this defensive check local as well so neither a
        // future symbol-table change nor compiler range analysis can turn a
        // negative category into an array subscript.
        if (xc<0 || yc<0) {
            const int r=motion_mvrange();
            const int hp=m.halfpel?1:0;
            return (mvrange_kx(r)-hp)+(mvrange_ky(r)-hp);
        }
        return mv_component_bits(xc,m.halfpel)+mv_component_bits(yc,m.halfpel);
    }

void Vc1Encoder::write_mv_component(BitWriter& b,int v,int cat,bool halfpel) {
        static constexpr int base[6]={0,1,3,7,15,31};
        if (!cat) return;
        const unsigned suffix=static_cast<unsigned>((std::abs(v)-base[cat])*2 + (v<0));
        b.bits(suffix,mv_component_bits(cat,halfpel));
    }

void Vc1Encoder::write_mvdata_table(BitWriter& b,int table,const MvDataSyntax& m) const {
        const int sym=mv_symbol(m);
        b.vlc(groupa::kMvDiffCodes[static_cast<size_t>(table)][static_cast<size_t>(sym)],
              groupa::kMvDiffBits[static_cast<size_t>(table)][static_cast<size_t>(sym)]);
        if (m.intra) return;
        if (sym==34 || sym==71) {
            const int r=motion_mvrange();
            const int hp=m.halfpel?1:0;
            const int xb=mvrange_kx(r)-hp, yb=mvrange_ky(r)-hp;
            b.bits(static_cast<uint64_t>(m.dx & ((1u<<xb)-1u)),xb);
            b.bits(static_cast<uint64_t>(m.dy & ((1u<<yb)-1u)),yb);
        } else {
            write_mv_component(b,m.dx,mv_cat(m.dx,m.halfpel),m.halfpel);
            write_mv_component(b,m.dy,mv_cat(m.dy,m.halfpel),m.halfpel);
        }
    }

int Vc1Encoder::choose_mv_table(const std::vector<MvDataSyntax>& values) const {
        uint64_t best=std::numeric_limits<uint64_t>::max(); int best_t=0;
        for (int t=0;t<4;++t) {
            uint64_t bits=0;
            for (const auto& m:values) {
                const int sym=mv_symbol(m);
                bits += groupa::kMvDiffBits[static_cast<size_t>(t)][static_cast<size_t>(sym)] + mv_suffix_bits(m);
            }
            if (bits<best) { best=bits; best_t=t; }
        }
        return best_t;
    }

bool Vc1Encoder::same_mv(MotionVector a, MotionVector b) { return a.xq==b.xq && a.yq==b.yq; }

MotionSearchBounds Vc1Encoder::motion_search_bounds_px(int requested) const {
        // ST 421 MVRANGE=3 reaches [-1024,1023.75] horizontally and
        // [-256,255.75] vertically. --search-range remains one convenient
        // radius: values above the vertical syntax limit continue extending X
        // while Y is clamped to the largest representable range.
        const int r=std::clamp(requested,0,1024);
        return {-std::min(r,1024),std::min(r,1023),
                -std::min(r,256),std::min(r,255)};
    }

MotionSearchBounds Vc1Encoder::motion_search_bounds_qpel(int requested) const {
        const int r=std::clamp(requested,0,1024);
        const int xmin=-std::min(r*4,4096);
        const int xmax=(r>=1024)?4095:std::min(r*4,4095);
        const int ymin=-std::min(r*4,1024);
        const int ymax=(r>=256)?1023:std::min(r*4,1023);
        return {xmin,xmax,ymin,ymax};
    }

int Vc1Encoder::motion_mvrange() const {
        const int r=c_.motion_search_range;
        if (r<=31) return 0;
        if (r<=63) return 1;
        if (r<=127) return 2;
        return 3;
    }

bool Vc1Encoder::extended_mv_enabled() const { return c_.motion_search_range>31; }

int Vc1Encoder::mvrange_x_qpel(int mvrange) {
        static constexpr int v[4]={256,512,2048,4096};
        return v[std::clamp(mvrange,0,3)];
    }

int Vc1Encoder::mvrange_y_qpel(int mvrange) {
        static constexpr int v[4]={128,256,512,1024};
        return v[std::clamp(mvrange,0,3)];
    }

int Vc1Encoder::mvrange_kx(int mvrange) {
        static constexpr int v[4]={9,10,12,13};
        return v[std::clamp(mvrange,0,3)];
    }

int Vc1Encoder::mvrange_ky(int mvrange) {
        static constexpr int v[4]={8,9,10,11};
        return v[std::clamp(mvrange,0,3)];
    }

void Vc1Encoder::write_mvrange(BitWriter& b) const {
        const int r=motion_mvrange();
        for (int i=0;i<r;++i) b.bit(true);
        if (r<3) b.bit(false);
    }

int Vc1Encoder::median3(int a,int b,int c) { return a+b+c-std::min({a,b,c})-std::max({a,b,c}); }

int Vc1Encoder::round_qpel_to_pixel(int q) {
        return q>=0 ? (q+2)/4 : -((-q+2)/4);
    }

int Vc1Encoder::modular_component_distance(int a,int b,int range) {
        int d=a-b;
        const int span=range*2;
        d=((d+range)%span+span)%span-range;
        return std::abs(d);
    }

int Vc1Encoder::modular_mv_distance(MotionVector a,MotionVector b) const {
        const int r=motion_mvrange();
        return modular_component_distance(a.xq,b.xq,mvrange_x_qpel(r))+
               modular_component_distance(a.yq,b.yq,mvrange_y_qpel(r));
    }

PredictorInfo Vc1Encoder::predictor_info(const std::vector<MotionVector>& mv,int mbw,int mbh,int mx,int my) const {
        PredictorInfo out;
        MotionVector A{},B{},C{};
        const bool a_valid=my>0;
        const bool c_valid=mx>0;
        bool b_valid=my>0 && mbw>1;
        if (a_valid) A=mv[static_cast<size_t>(my-1)*mbw+mx];
        if (c_valid) C=mv[static_cast<size_t>(my)*mbw+mx-1];
        if (b_valid) {
            const int bx=(mx==mbw-1)?mx-1:mx+1;
            B=mv[static_cast<size_t>(my-1)*mbw+bx];
        }
        MotionVector pre{};
        if (a_valid) pre=A;
        else if (c_valid) pre=C;
        else if (b_valid) pre=B;
        const int valid=(a_valid?1:0)+(b_valid?1:0)+(c_valid?1:0);
        if (valid>1) {
            pre.xq=median3(A.xq,B.xq,C.xq);
            pre.yq=median3(A.yq,B.yq,C.yq);
        }

        // Advanced-profile 1-MV predictor pullback (quarter-pixel units).
        const int qx=mx*64, qy=my*64;
        const int X=mbw*64-4, Y=mbh*64-4;
        if (qx+pre.xq < -60) pre.xq=-60-qx;
        if (qx+pre.xq > X) pre.xq=X-qx;
        if (qy+pre.yq < -60) pre.yq=-60-qy;
        if (qy+pre.yq > Y) pre.yq=Y-qy;
        bool hybrid=false;
        if (a_valid && c_valid) {
            int sum=std::abs(pre.xq-A.xq)+std::abs(pre.yq-A.yq);
            if (sum>32) hybrid=true;
            else {
                sum=std::abs(pre.xq-C.xq)+std::abs(pre.yq-C.yq);
                if (sum>32) hybrid=true;
            }
        }
        out.pre=pre; out.a=A; out.c=C; out.a_valid=a_valid; out.c_valid=c_valid; out.hybrid=hybrid;
        return out;
    }

PredictorInfo Vc1Encoder::predictor_info_mixed_1mv(const std::vector<std::array<MotionVector,4>>& field,
                                           const std::vector<uint8_t>& mb_intra,
                                           int mbw,int mbh,int mx,int my) const {
        PredictorInfo out;
        const int bx=mx*2, by=my*2;
        auto get=[&](int x,int y)->MotionVector {
            if (x<0 || y<0 || x>=mbw*2 || y>=mbh*2) return {};
            const int mmx=x>>1,mmy=y>>1,k=((y&1)<<1)|(x&1);
            return field[static_cast<size_t>(mmy)*mbw+mmx][static_cast<size_t>(k)];
        };
        auto intra_at=[&](int x,int y)->bool {
            if (x<0 || y<0 || x>=mbw*2 || y>=mbh*2) return false;
            return mb_intra[static_cast<size_t>(y>>1)*mbw+(x>>1)]!=0;
        };
        const bool a_valid=my>0;
        const bool c_valid=mx>0;
        const bool b_valid=my>0 && mbw>1;
        MotionVector A=a_valid?get(bx,by-1):MotionVector{};
        MotionVector C=c_valid?get(bx-1,by):MotionVector{};
        MotionVector B{};
        int bbx=bx;
        if (b_valid) {
            bbx += (mx==mbw-1)?-1:2;
            B=get(bbx,by-1);
        }
        MotionVector pre{};
        if (a_valid) pre=A; else if (c_valid) pre=C; else if (b_valid) pre=B;
        const int valid=(a_valid?1:0)+(b_valid?1:0)+(c_valid?1:0);
        if (valid>1) {
            pre.xq=median3(A.xq,B.xq,C.xq);
            pre.yq=median3(A.yq,B.yq,C.yq);
        }
        const int qx=mx*64,qy=my*64,X=mbw*64-4,Y=mbh*64-4;
        if (qx+pre.xq < -60) pre.xq=-60-qx;
        if (qx+pre.xq > X) pre.xq=X-qx;
        if (qy+pre.yq < -60) pre.yq=-60-qy;
        if (qy+pre.yq > Y) pre.yq=Y-qy;
        bool hybrid=false;
        if (a_valid && c_valid) {
            const int suma=intra_at(bx,by-1)
                ? std::abs(pre.xq)+std::abs(pre.yq)
                : std::abs(pre.xq-A.xq)+std::abs(pre.yq-A.yq);
            if (suma>32) hybrid=true;
            else {
                const int sumc=intra_at(bx-1,by)
                    ? std::abs(pre.xq)+std::abs(pre.yq)
                    : std::abs(pre.xq-C.xq)+std::abs(pre.yq-C.yq);
                if (sumc>32) hybrid=true;
            }
        }
        out.pre=pre;out.a=A;out.c=C;out.a_valid=a_valid;out.c_valid=c_valid;out.hybrid=hybrid;
        return out;
    }

PredictorInfo Vc1Encoder::predictor_info_4mv(const std::vector<std::array<MotionVector,4>>& field,
                                     const std::vector<uint8_t>& mb_intra,
                                     int mbw,int mbh,int mx,int my,int n) const {
        PredictorInfo out;
        const int bx=mx*2+(n&1), by=my*2+((n>>1)&1);
        auto get=[&](int x,int y)->MotionVector {
            if (x<0 || y<0 || x>=mbw*2 || y>=mbh*2) return {};
            const int mmx=x>>1,mmy=y>>1,k=((y&1)<<1)|(x&1);
            return field[static_cast<size_t>(mmy)*mbw+mmx][static_cast<size_t>(k)];
        };
        const bool a_valid=my>0 || n==2 || n==3;
        const bool c_valid=mx>0 || n==1 || n==3;
        const bool b_valid=a_valid;
        MotionVector A=a_valid?get(bx,by-1):MotionVector{};
        MotionVector C=c_valid?get(bx-1,by):MotionVector{};
        MotionVector B{};
        if (b_valid) {
            int bxx=bx;
            if (n==0) bxx += mx ? -1 : 1;       // RES_RTM_FLAG is permanently 1.
            else if (n==1) bxx += (mx==mbw-1) ? -1 : 1;
            else if (n==2) bxx += 1;
            else bxx -= 1;
            B=get(bxx,by-1);
        }
        MotionVector pre{};
        if (a_valid) pre=A; else if (c_valid) pre=C; else if (b_valid) pre=B;
        const int valid=(a_valid?1:0)+(b_valid?1:0)+(c_valid?1:0);
        if (valid>1) {
            pre.xq=median3(A.xq,B.xq,C.xq);
            pre.yq=median3(A.yq,B.yq,C.yq);
        }
        // Progressive 4-MV pullback uses -28 instead of the 1-MV -60 margin.
        const int qx=mx*64+((n==1||n==3)?32:0);
        const int qy=my*64+((n==2||n==3)?32:0);
        const int X=mbw*64-4,Y=mbh*64-4;
        if (qx+pre.xq < -28) pre.xq=-28-qx;
        if (qx+pre.xq > X) pre.xq=X-qx;
        if (qy+pre.yq < -28) pre.yq=-28-qy;
        if (qy+pre.yq > Y) pre.yq=Y-qy;
        auto intra_at=[&](int x,int y)->bool {
            if (x<0 || y<0 || x>=mbw*2 || y>=mbh*2) return false;
            return mb_intra[static_cast<size_t>(y>>1)*mbw+(x>>1)]!=0;
        };
        bool hybrid=false;
        if (a_valid && c_valid) {
            const int suma=intra_at(bx,by-1)
                ? std::abs(pre.xq)+std::abs(pre.yq)
                : std::abs(pre.xq-A.xq)+std::abs(pre.yq-A.yq);
            if (suma>32) hybrid=true;
            else {
                const int sumc=intra_at(bx-1,by)
                    ? std::abs(pre.xq)+std::abs(pre.yq)
                    : std::abs(pre.xq-C.xq)+std::abs(pre.yq-C.yq);
                if (sumc>32) hybrid=true;
            }
        }
        out.pre=pre; out.a=A; out.c=C; out.a_valid=a_valid; out.c_valid=c_valid; out.hybrid=hybrid;
        return out;
    }

MotionVector Vc1Encoder::predictor_b_main(const std::vector<MotionVector>& mv,int mbw,int mx,int my) const {
        // Simple/Main B prediction uses the same A/B/C median topology as 1-MV
        // P prediction, but ff_vc1_pred_b_mv operates on a 32-unit MB grid
        // (sh=5) and does not signal/use HYBRIDPRED.
        MotionVector A{},B{},C{};
        const bool a_valid=my>0;
        const bool c_valid=mx>0;
        const bool b_valid=my>0 && mbw>1;
        if (a_valid) A=mv[static_cast<size_t>(my-1)*mbw+mx];
        if (c_valid) C=mv[static_cast<size_t>(my)*mbw+mx-1];
        if (b_valid) {
            const int bx=(mx==mbw-1)?mx-1:mx+1;
            B=mv[static_cast<size_t>(my-1)*mbw+bx];
        }
        MotionVector pre{};
        if (a_valid) pre=A; else if (c_valid) pre=C; else if (b_valid) pre=B;
        const int valid=(a_valid?1:0)+(b_valid?1:0)+(c_valid?1:0);
        if (valid>1) {
            pre.xq=median3(A.xq,B.xq,C.xq);
            pre.yq=median3(A.yq,B.yq,C.yq);
        }
        const int qx=mx*32, qy=my*32;
        const int X=mbw*32-4, Y=((c_.height+15)/16)*32-4;
        constexpr int MV=-28;
        if (qx+pre.xq < MV) pre.xq=MV-qx;
        if (qx+pre.xq > X) pre.xq=X-qx;
        if (qy+pre.yq < MV) pre.yq=MV-qy;
        if (qy+pre.yq > Y) pre.yq=Y-qy;
        return pre;
    }

uint64_t Vc1Encoder::frame_sad(const Frame& cur,const Frame& ref,int dx,int dy,int stride) const {
#if defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::X86V4)
            return simd::frame_sad_x86_64_v4(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::X86V1)
            return simd::frame_sad_x86_64_v1(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::Prescott)
            return simd::frame_sad_prescott(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_K10)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::K10)
            return simd::frame_sad_k10(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_CONROE)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::Conroe)
            return simd::frame_sad_conroe(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::X86V2)
            return simd::frame_sad_x86_64_v2(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::Penryn)
            return simd::frame_sad_penryn(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::SandyBridge)
            return simd::frame_sad_sandybridge(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::Bulldozer)
            return simd::frame_sad_bulldozer(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::Piledriver)
            return simd::frame_sad_piledriver(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::Avx2Partial)
            return simd::frame_sad_avx2_partial(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        if (c_.simd_for(SimdPrimitive::FrameSad)==SimdTier::X86V3)
            return simd::frame_sad_x86_64_v3(cur.y.data(),cur.width,ref.y.data(),ref.width,
                                             cur.width,cur.height,dx,dy,stride);
#endif
        uint64_t sad=0;
        for (int y=0; y<cur.height; y+=stride) {
            const int sy=std::clamp(y+dy,0,ref.height-1);
            for (int x=0; x<cur.width; x+=stride) {
                const int sx=std::clamp(x+dx,0,ref.width-1);
                sad += static_cast<uint64_t>(std::abs(static_cast<int>(cur.y[static_cast<size_t>(y)*cur.width+x])-
                                                     static_cast<int>(ref.y[static_cast<size_t>(sy)*ref.width+sx])));
            }
        }
        if (stride>1) sad*=static_cast<uint64_t>(stride*stride);
        return sad;
    }

uint8_t Vc1Encoder::mean_region(const std::vector<uint8_t>& plane,int pw,int ph,
                               int x0,int y0,int bw,int bh) {
        uint64_t sum=0; int n=0;
        for (int y=0;y<bh && y0+y<ph;++y) {
            if (y0+y<0) continue;
            for (int x=0;x<bw && x0+x<pw;++x) {
                if (x0+x<0) continue;
                sum+=plane[static_cast<size_t>(y0+y)*pw+x0+x]; ++n;
            }
        }
        return n?static_cast<uint8_t>((sum+static_cast<uint64_t>(n/2))/static_cast<uint64_t>(n)):128;
    }

LongRangeSignature Vc1Encoder::long_range_signature(const Frame& f,int x0,int y0) const {
        LongRangeSignature s;
        s.ydc[0]=mean_region(f.y,f.width,f.height,x0,y0,8,8);
        s.ydc[1]=mean_region(f.y,f.width,f.height,x0+8,y0,8,8);
        s.ydc[2]=mean_region(f.y,f.width,f.height,x0,y0+8,8,8);
        s.ydc[3]=mean_region(f.y,f.width,f.height,x0+8,y0+8,8,8);
        const int cw=f.width/2,ch=f.height/2;
        s.udc=mean_region(f.u,cw,ch,x0/2,y0/2,8,8);
        s.vdc=mean_region(f.v,cw,ch,x0/2,y0/2,8,8);
        return s;
    }

unsigned Vc1Encoder::long_range_signature_distance(const LongRangeSignature& a,
                                                   const LongRangeSignature& b) {
        unsigned d=0;
        for (size_t i=0;i<4;++i)
            d+=static_cast<unsigned>(std::abs(static_cast<int>(a.ydc[i])-static_cast<int>(b.ydc[i])))*2u;
        // Chroma DC is cheap and prevents an equally bright but differently
        // colored object from becoming a strong distant candidate.
        d+=static_cast<unsigned>(std::abs(static_cast<int>(a.udc)-static_cast<int>(b.udc)))*3u;
        d+=static_cast<unsigned>(std::abs(static_cast<int>(a.vdc)-static_cast<int>(b.vdc)))*3u;
        return d;
    }

LongRangeIndex Vc1Encoder::build_long_range_index(const Frame& ref) const {
        LongRangeIndex idx;
        // Eight-pixel spacing is intentionally denser than the coded MB raster.
        // The signature scan is still very cheap compared with full SAD, and
        // the later +/-8 local refinement recovers arbitrary integer motion.
        const int nx=(ref.width+7)/8,ny=(ref.height+7)/8;
        idx.cells.reserve(static_cast<size_t>(nx)*ny);
        for (int y=0;y<ref.height;y+=8) for (int x=0;x<ref.width;x+=8)
            idx.cells.push_back({x,y,long_range_signature(ref,x,y)});
        return idx;
    }

uint64_t Vc1Encoder::active_mb_pixels(int mx,int my) const {
        const int x0=mx*16,y0=my*16;
        return static_cast<uint64_t>(std::max(0,std::min(16,c_.width-x0))) *
               static_cast<uint64_t>(std::max(0,std::min(16,c_.height-y0)));
    }

bool Vc1Encoder::motion_match_good(uint64_t sad,int mx,int my) const {
        const uint64_t pixels=active_mb_pixels(mx,my);
        if (!pixels || sad>=std::numeric_limits<uint64_t>::max()/8) return false;
        // Six luma levels of average absolute error is a deliberately strict
        // early-out.  A block this close is already a strong prediction, so a
        // frame-wide distant lookup is very unlikely to repay its cost.
        return sad<=pixels*6u;
    }

int Vc1Encoder::distant_match_decision(uint64_t candidate_sad,uint64_t local_sad,int mx,int my) const {
        const uint64_t pixels=active_mb_pixels(mx,my);
        if (!pixels || candidate_sad>=std::numeric_limits<uint64_t>::max()/8) return 0;
        // A genuinely strong distant match is always useful, including across
        // very large translations. Keep the historical <=6-MAE fast path.
        if (candidate_sad<=pixels*6u) return 2;
        // Relative improvement alone is unsafe at a hard cut: 40 -> 29 MAE is
        // 27.5% better yet still a terrible predictor. Require the candidate
        // to be objectively plausible before the >=25% relative-improvement
        // rule may admit it.
        const long double max_sad=static_cast<long double>(pixels)*c_.distant_match_max_mae;
        if (static_cast<long double>(candidate_sad)>max_sad) return 1;
        if (local_sad>=std::numeric_limits<uint64_t>::max()/8) return 3;
        return candidate_sad*4u<=local_sad*3u ? 3 : 1;
    }

IntegerMotionResult Vc1Encoder::propagated_long_range_motion(const Frame& cur,const Frame& ref,
                                                      int mx,int my,int mbw,
                                                      const std::vector<MotionVector>& field,
                                                      const std::vector<uint8_t>& propagated,
                                                      int requested_range) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionLongRange);
        IntegerMotionResult best{0,0,std::numeric_limits<uint64_t>::max()};
        const auto b=motion_search_bounds_px(requested_range);
        auto test_mv=[&](MotionVector qmv) {
            const int cx=round_qpel_to_pixel(qmv.xq),cy=round_qpel_to_pixel(qmv.yq);
            static constexpr std::array<std::array<int,2>,13> offsets{{
                {{0,0}},{{-1,0}},{{1,0}},{{0,-1}},{{0,1}},
                {{-2,0}},{{2,0}},{{0,-2}},{{0,2}},
                {{-1,-1}},{{1,-1}},{{-1,1}},{{1,1}}
            }};
            for (const auto& o:offsets) {
                const int dx=std::clamp(cx+o[0],b.xmin,b.xmax);
                const int dy=std::clamp(cy+o[1],b.ymin,b.ymax);
                const uint64_t sad=mb_sad(cur,ref,mx,my,dx,dy,1);
                if (sad<best.full_sad ||
                    (sad==best.full_sad && std::abs(dx)+std::abs(dy)<std::abs(best.dx)+std::abs(best.dy)))
                    best={dx,dy,sad};
            }
        };
        // Prefer the left neighbor because raster-order objects normally
        // continue horizontally; then top, top-left, and top-right.
        const std::array<std::pair<int,int>,4> n{{{-1,0},{0,-1},{-1,-1},{1,-1}}};
        for (const auto& [ox,oy]:n) {
            const int nx=mx+ox,ny=my+oy;
            if (nx<0||ny<0||nx>=mbw) continue;
            const size_t pos=static_cast<size_t>(ny)*mbw+nx;
            if (pos>=propagated.size() || !propagated[pos]) continue;
            test_mv(field[pos]);
            if (motion_match_good(best.full_sad,mx,my)) break;
        }
        return best;
    }

IntegerMotionResult Vc1Encoder::extended_content_motion(const Frame& cur,const Frame& ref,
                                                 int mx,int my,int requested_range,
                                                 const LongRangeIndex& idx,
                                                 const IntegerMotionResult& local,
                                                 uint64_t* distant_candidate_sad,
                                                 int* distant_decision) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionLongRange);
        struct SigCandidate { unsigned score; int dx; int dy; };
        std::vector<SigCandidate> top;
        top.reserve(16);
        const int x0=mx*16,y0=my*16;
        auto score_integer_batch=[&](const std::vector<std::pair<int,int>>& points,int sample_stride,
                                     std::vector<uint64_t>& costs) {
            std::vector<MotionVector> mvs; mvs.reserve(points.size());
            for (const auto& [dx,dy]:points) mvs.push_back({dx*4,dy*4});
            return vulkan_motion_costs(cur,ref,x0,y0,16,16,mvs,sample_stride,
                                       ProgressiveMvMode::OneMvQpel,false,costs) && costs.size()==points.size();
        };
        const auto want=long_range_signature(cur,x0,y0);
        const auto bounds=motion_search_bounds_px(requested_range);
        auto sig_less=[](const SigCandidate& a,const SigCandidate& b) {
            if (a.score!=b.score) return a.score<b.score;
            const int am=std::abs(a.dx)+std::abs(a.dy),bm=std::abs(b.dx)+std::abs(b.dy);
            if (am!=bm) return am<bm;
            if (a.dy!=b.dy) return a.dy<b.dy;
            return a.dx<b.dx;
        };
        for (const auto& cell:idx.cells) {
            const int dx=cell.x-x0,dy=cell.y-y0;
            if (dx<bounds.xmin||dx>bounds.xmax||dy<bounds.ymin||dy>bounds.ymax) continue;
            if (std::abs(dx)<=32 && std::abs(dy)<=32) continue; // local UMH already covered it.
            const SigCandidate c{long_range_signature_distance(want,cell.sig),dx,dy};
            // Once the exact top-16 is full, almost every frame-index cell is
            // worse than the current tail. Avoid push+sort+resize for those
            // rejected cells; insertion preserves the same total ordering.
            if (top.size()==16 && !sig_less(c,top.back())) continue;
            const auto it=std::lower_bound(top.begin(),top.end(),c,sig_less);
            top.insert(it,c);
            if (top.size()>16) top.pop_back();
        }
        IntegerMotionResult best{0,0,std::numeric_limits<uint64_t>::max()};
        for (const auto& c:top) {
            int sx=c.dx,sy=c.dy;
            uint64_t sampled=std::numeric_limits<uint64_t>::max();
            // The DC lookup is on an 8-pixel lattice. Refine each promising
            // destination over the surrounding 16x16 area before paying for
            // full-resolution SAD.
            std::vector<std::pair<int,int>> sampled_points;
            sampled_points.reserve(25);
            for (int oy=-8;oy<=8;oy+=4) for (int ox=-8;ox<=8;ox+=4)
                sampled_points.emplace_back(std::clamp(c.dx+ox,bounds.xmin,bounds.xmax),
                                            std::clamp(c.dy+oy,bounds.ymin,bounds.ymax));
            std::vector<uint64_t> sampled_costs;
            const bool gpu_sampled=score_integer_batch(sampled_points,2,sampled_costs);
            for (size_t i=0;i<sampled_points.size();++i) {
                const auto [dx,dy]=sampled_points[i];
                const uint64_t sad=gpu_sampled?sampled_costs[i]:mb_sad(cur,ref,mx,my,dx,dy,2);
                if (sad<sampled) { sampled=sad;sx=dx;sy=dy; }
            }
            IntegerMotionResult full{sx,sy,std::numeric_limits<uint64_t>::max()};
            std::vector<std::pair<int,int>> full_points;
            full_points.reserve(25);
            for (int oy=-2;oy<=2;++oy) for (int ox=-2;ox<=2;++ox)
                full_points.emplace_back(std::clamp(sx+ox,bounds.xmin,bounds.xmax),
                                         std::clamp(sy+oy,bounds.ymin,bounds.ymax));
            std::vector<uint64_t> full_costs;
            const bool gpu_full=score_integer_batch(full_points,1,full_costs);
            for (size_t i=0;i<full_points.size();++i) {
                const auto [dx,dy]=full_points[i];
                const uint64_t sad=gpu_full?full_costs[i]:mb_sad(cur,ref,mx,my,dx,dy,1);
                if (sad<full.full_sad ||
                    (sad==full.full_sad && std::abs(dx)+std::abs(dy)<std::abs(full.dx)+std::abs(full.dy)))
                    full={dx,dy,sad};
            }
            if (full.full_sad<best.full_sad) best=full;
            if (motion_match_good(best.full_sad,mx,my)) break;
        }
        if (distant_candidate_sad) *distant_candidate_sad=best.full_sad;
        const int decision=best.full_sad<std::numeric_limits<uint64_t>::max()/8
            ? distant_match_decision(best.full_sad,local.full_sad,mx,my) : VC1_DISTANT_MATCH_NONE;
        if (distant_decision) *distant_decision=decision;
        if (best.full_sad>=local.full_sad || decision<VC1_DISTANT_MATCH_ABSOLUTE_GOOD) return local;
        return best;
    }

IntegerMotionResult Vc1Encoder::integer_motion_umh(const Frame& cur,const Frame& ref,int mx,int my,
                                           const PredictorInfo& pi,const GlobalMotionResult& global,
                                           int range) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionLocalSearch);
        struct Candidate { uint64_t cost; int dx; int dy; };
        auto better=[](uint64_t sad,int dx,int dy,uint64_t best,int bx,int by) {
            if (sad!=best) return sad<best;
            const int m=std::abs(dx)+std::abs(dy), bm=std::abs(bx)+std::abs(by);
            if (m!=bm) return m<bm;
            if (dy!=by) return dy<by;
            return dx<bx;
        };
        auto clamp_coord=[&](int v){ return std::clamp(v,-range,range); };
        auto score_integer_batch=[&](const std::vector<std::pair<int,int>>& points,int sample_stride,
                                     std::vector<uint64_t>& costs) {
            std::vector<MotionVector> mvs;
            mvs.reserve(points.size());
            for (const auto& [dx,dy]:points) mvs.push_back({dx*4,dy*4});
            return vulkan_motion_costs(cur,ref,mx*16,my*16,16,16,mvs,sample_stride,
                                       ProgressiveMvMode::OneMvQpel,false,costs) && costs.size()==points.size();
        };
        std::array<Candidate,8> top{};
        size_t top_count=0;
        Candidate best{std::numeric_limits<uint64_t>::max(),0,0};
        // UMH patterns overlap heavily (predictor seeds, 5x5, lattice, cross,
        // hexagons and iterative refinement).  For the normal <=64-pixel local
        // radius, a 2 KiB stack bitset prevents scoring the same integer vector
        // twice while preserving first-visit ordering and every tie decision.
        static constexpr int kSeenRadius=64;
        static constexpr int kSeenSide=2*kSeenRadius+1;
        std::array<uint64_t,(kSeenSide*kSeenSide+63)/64> seen_sampled{};
        auto already_sampled=[&](int dx,int dy) {
            if (range>kSeenRadius) return false;
            const size_t bit=static_cast<size_t>(dy+kSeenRadius)*kSeenSide+static_cast<size_t>(dx+kSeenRadius);
            uint64_t& word=seen_sampled[bit>>6];
            const uint64_t mask=uint64_t{1}<<(bit&63);
            if (word&mask) return true;
            word|=mask;
            return false;
        };
        auto consider=[&](int dx,int dy) {
            dx=clamp_coord(dx); dy=clamp_coord(dy);
            if (already_sampled(dx,dy)) return;
            const uint64_t cost=mb_sad(cur,ref,mx,my,dx,dy,2);
            if (better(cost,dx,dy,best.cost,best.dx,best.dy)) best={cost,dx,dy};
            for (size_t i=0;i<top_count;++i) {
                auto& c=top[i];
                if (c.dx==dx && c.dy==dy) { if (cost<c.cost) c.cost=cost; return; }
            }
            const Candidate cand{cost,dx,dy};
            size_t at=0;
            while (at<top_count && !better(cand.cost,cand.dx,cand.dy,top[at].cost,top[at].dx,top[at].dy)) ++at;
            if (at<top.size()) {
                const size_t new_count=std::min(top_count+1,top.size());
                for (size_t j=new_count-1;j>at;--j) top[j]=top[j-1];
                top[at]=cand;
                top_count=new_count;
            }
        };

        // Predictor-driven UMH start: median spatial predictor, zero, frame
        // translation, and the two directly addressable spatial candidates.
        // These seeds are deliberately evaluated before any geometric pattern.
        const int pdx=clamp_coord(round_qpel_to_pixel(pi.pre.xq));
        const int pdy=clamp_coord(round_qpel_to_pixel(pi.pre.yq));
        consider(pdx,pdy);
        consider(0,0);
        consider(global.dx,global.dy);
        if (pi.a_valid) consider(round_qpel_to_pixel(pi.a.xq),round_qpel_to_pixel(pi.a.yq));
        if (pi.c_valid) consider(round_qpel_to_pixel(pi.c.xq),round_qpel_to_pixel(pi.c.yq));

        // Local 5x5 refinement catches the overwhelmingly common case before
        // the wider search.  All broad UMH stages use the 2x2 sampled SAD; only
        // the finalists pay full-resolution SAD.
        {
            const int cx=best.dx,cy=best.dy;
            for (int oy=-2;oy<=2;++oy) for (int ox=-2;ox<=2;++ox)
                consider(cx+ox,cy+oy);
        }

        // A very sparse whole-local-window lattice gives UMH a few independent
        // basins on repetitive texture without restoring the old dense coarse
        // exhaustive pass. At the normal +/-16 range this is only 25 sampled
        // candidates; at +/-32 it is 81.
        if (range>=8) {
            const int lattice=8;
            for (int dy=-range;dy<=range;dy+=lattice)
                for (int dx=-range;dx<=range;dx+=lattice) consider(dx,dy);
            for (int d=-range;d<=range;d+=lattice) { consider(range,d); consider(d,range); }
        }

        // Uneven cross: spend more horizontal reach, where camera/object motion
        // is commonly strongest, while the following multi-hex rings still
        // cover the complete configured vertical radius.
        const int cross_step=range>=96?8:(range>=32?4:2);
        const int cx=best.dx,cy=best.dy;
        const int vlimit=std::min(range,std::max(8,range/2));
        for (int d=cross_step;d<=range;d+=cross_step) {
            consider(cx-d,cy); consider(cx+d,cy);
        }
        for (int d=cross_step;d<=vlimit;d+=cross_step) {
            consider(cx,cy-d); consider(cx,cy+d);
        }

        // Multi-hexagon rings.  The six points are a true integer hexagonal
        // stencil; repeated rings make the search uneven/multi-hex rather than
        // a local diamond walk and can jump across repetitive-texture minima.
        const int hex_step=range>=96?8:(range>=24?4:2);
        const int hx=best.dx,hy=best.dy;
        for (int r=hex_step;r<=range;r+=hex_step) {
            const int h=(r+1)/2;
            consider(hx-r,hy); consider(hx+r,hy);
            consider(hx-h,hy-r); consider(hx+h,hy-r);
            consider(hx-h,hy+r); consider(hx+h,hy+r);
        }

        // Iterative small-hex refinement follows the best basin, then a diamond
        // closes the integer search.  A hard iteration bound prevents a malformed
        // edge case from walking forever while still allowing long convergence.
        for (int iter=0;iter<32;++iter) {
            const int ox=best.dx,oy=best.dy;
            consider(ox-2,oy); consider(ox+2,oy);
            consider(ox-1,oy-2); consider(ox+1,oy-2);
            consider(ox-1,oy+2); consider(ox+1,oy+2);
            if (best.dx==ox && best.dy==oy) break;
        }
        {
            const int ox=best.dx,oy=best.dy;
            consider(ox-1,oy); consider(ox+1,oy);
            consider(ox,oy-1); consider(ox,oy+1);
        }

        // Re-rank a few sampled-SAD finalists with the true 16x16 SAD.  This
        // keeps UMH fast while avoiding phase aliases from the 2x2 sample.
        Candidate full{std::numeric_limits<uint64_t>::max(),0,0};
        std::array<uint64_t,(kSeenSide*kSeenSide+63)/64> seen_full{};
        auto already_full=[&](int dx,int dy) {
            if (range>kSeenRadius) return false;
            const size_t bit=static_cast<size_t>(dy+kSeenRadius)*kSeenSide+static_cast<size_t>(dx+kSeenRadius);
            uint64_t& word=seen_full[bit>>6];
            const uint64_t mask=uint64_t{1}<<(bit&63);
            if (word&mask) return true;
            word|=mask;
            return false;
        };
        const size_t finalists=std::min<size_t>(4,top_count);
        std::vector<std::pair<int,int>> full_points;
        full_points.reserve(finalists*9);
        for (size_t i=0;i<finalists;++i) {
            const auto c=top[i];
            for (int oy=-1;oy<=1;++oy) for (int ox=-1;ox<=1;++ox) {
                const int dx=clamp_coord(c.dx+ox),dy=clamp_coord(c.dy+oy);
                if (already_full(dx,dy)) continue;
                full_points.emplace_back(dx,dy);
            }
        }
        std::vector<uint64_t> full_costs;
        const bool gpu_full=score_integer_batch(full_points,1,full_costs);
        for (size_t i=0;i<full_points.size();++i) {
            const auto [dx,dy]=full_points[i];
            const uint64_t sad=gpu_full?full_costs[i]:mb_sad(cur,ref,mx,my,dx,dy,1);
            if (better(sad,dx,dy,full.cost,full.dx,full.dy)) full={sad,dx,dy};
        }
        if (full.cost==std::numeric_limits<uint64_t>::max()) {
            (void)already_full(best.dx,best.dy);
            full={mb_sad(cur,ref,mx,my,best.dx,best.dy,1),best.dx,best.dy};
        }

        // UMH is the normal path.  On a genuinely poor local match, retain the
        // old wide-window robustness as a bounded rescue rather than paying for
        // it on every macroblock.  This is particularly useful on repetitive
        // fine texture where a geometric pattern can land in the wrong basin.
        if (range>0 && !motion_match_good(full.cost,mx,my)) {
            Candidate coarse{std::numeric_limits<uint64_t>::max(),0,0};
            const int step=range>=8?4:(range>=3?2:1);
            std::vector<std::pair<int,int>> coarse_points;
            coarse_points.reserve(static_cast<size_t>((2*range)/step+2)*static_cast<size_t>((2*range)/step+2));
            for (int dy=-range;dy<=range;dy+=step)
                for (int dx=-range;dx<=range;dx+=step) coarse_points.emplace_back(dx,dy);
            for (int d=-range;d<=range;d+=step) {
                coarse_points.emplace_back(range,d); coarse_points.emplace_back(d,range);
            }
            std::vector<uint64_t> coarse_costs;
            const bool gpu_coarse=score_integer_batch(coarse_points,4,coarse_costs);
            for (size_t i=0;i<coarse_points.size();++i) {
                const auto [dx,dy]=coarse_points[i];
                const uint64_t sad=gpu_coarse?coarse_costs[i]:mb_sad(cur,ref,mx,my,dx,dy,4);
                if (better(sad,dx,dy,coarse.cost,coarse.dx,coarse.dy)) coarse={sad,dx,dy};
            }
            Candidate rescue{std::numeric_limits<uint64_t>::max(),coarse.dx,coarse.dy};
            const int rr=std::max(1,step-1);
            std::vector<std::pair<int,int>> rescue_points;
            for (int dy=std::max(-range,coarse.dy-rr);dy<=std::min(range,coarse.dy+rr);++dy)
                for (int dx=std::max(-range,coarse.dx-rr);dx<=std::min(range,coarse.dx+rr);++dx)
                    if (!already_full(dx,dy)) rescue_points.emplace_back(dx,dy);
            std::vector<uint64_t> rescue_costs;
            const bool gpu_rescue=score_integer_batch(rescue_points,1,rescue_costs);
            for (size_t i=0;i<rescue_points.size();++i) {
                const auto [dx,dy]=rescue_points[i];
                const uint64_t sad=gpu_rescue?rescue_costs[i]:mb_sad(cur,ref,mx,my,dx,dy,1);
                if (better(sad,dx,dy,rescue.cost,rescue.dx,rescue.dy)) rescue={sad,dx,dy};
            }
            if (better(rescue.cost,rescue.dx,rescue.dy,full.cost,full.dx,full.dy)) full=rescue;

            // 0.1.72 robustness pass: if the ordinary UMH plus the historical
            // sparse rescue still leaves a very poor prediction, densify the
            // whole local window on a 2-pixel lattice and fully score the small
            // neighborhood around its winner. This is deliberately gated to
            // high-error blocks so ordinary motion keeps the fast UMH path, but
            // repetitive texture / hard transitions do not depend on which
            // phase happened to win the 4-pixel rescue lattice.
            const uint64_t pixels=active_mb_pixels(mx,my);
            if (pixels && full.cost>pixels*24u && range<=32) {
                Candidate dense{std::numeric_limits<uint64_t>::max(),0,0};
                std::vector<std::pair<int,int>> dense_points;
                for (int dy=-range;dy<=range;dy+=2)
                    for (int dx=-range;dx<=range;dx+=2) dense_points.emplace_back(dx,dy);
                std::vector<uint64_t> dense_costs;
                const bool gpu_dense=score_integer_batch(dense_points,2,dense_costs);
                for (size_t i=0;i<dense_points.size();++i) {
                    const auto [dx,dy]=dense_points[i];
                    const uint64_t sad=gpu_dense?dense_costs[i]:mb_sad(cur,ref,mx,my,dx,dy,2);
                    if (better(sad,dx,dy,dense.cost,dense.dx,dense.dy)) dense={sad,dx,dy};
                }
                Candidate dense_full{std::numeric_limits<uint64_t>::max(),dense.dx,dense.dy};
                std::vector<std::pair<int,int>> dense_full_points;
                for (int dy=std::max(-range,dense.dy-2);dy<=std::min(range,dense.dy+2);++dy)
                    for (int dx=std::max(-range,dense.dx-2);dx<=std::min(range,dense.dx+2);++dx)
                        if (!already_full(dx,dy)) dense_full_points.emplace_back(dx,dy);
                std::vector<uint64_t> dense_full_costs;
                const bool gpu_dense_full=score_integer_batch(dense_full_points,1,dense_full_costs);
                for (size_t i=0;i<dense_full_points.size();++i) {
                    const auto [dx,dy]=dense_full_points[i];
                    const uint64_t sad=gpu_dense_full?dense_full_costs[i]:mb_sad(cur,ref,mx,my,dx,dy,1);
                    if (better(sad,dx,dy,dense_full.cost,dense_full.dx,dense_full.dy))
                        dense_full={sad,dx,dy};
                }
                if (better(dense_full.cost,dense_full.dx,dense_full.dy,full.cost,full.dx,full.dy))
                    full=dense_full;
            }
        }
        return {full.dx,full.dy,full.cost};
    }

GlobalMotionResult Vc1Encoder::global_motion_search_extended(const Frame& cur,const Frame& ref,int range) const {
        struct Candidate { uint64_t cost; int dx; int dy; };
        auto better=[](const Candidate& a,const Candidate& b) {
            if (a.cost!=b.cost) return a.cost<b.cost;
            const int am=std::abs(a.dx)+std::abs(a.dy),bm=std::abs(b.dx)+std::abs(b.dy);
            if (am!=bm) return am<bm;
            if (a.dy!=b.dy) return a.dy<b.dy;
            return a.dx<b.dx;
        };
        std::vector<Candidate> top;
        top.reserve(8);
        auto consider=[&](int dx,int dy,int stride) {
            dx=std::clamp(dx,-range,range);dy=std::clamp(dy,-range,range);
            const Candidate c{frame_sad(cur,ref,dx,dy,stride),dx,dy};
            for (auto& e:top) if (e.dx==dx&&e.dy==dy) { if (c.cost<e.cost)e=c; return; }
            top.push_back(c);std::sort(top.begin(),top.end(),better);if(top.size()>8)top.resize(8);
        };

        // Large configured ranges would make the historical O(range^2) global
        // sweep dominate runtime.  Use a coarse uneven-cross/multi-hex search
        // on a 16x16 sample instead, then successively densify only the winners.
        consider(0,0,16);
        const int step=range>=160?16:8;
        for (int d=step;d<=range;d+=step) {
            consider(-d,0,16);consider(d,0,16);consider(0,-d,16);consider(0,d,16);
        }
        for (int r=step;r<=range;r+=step) {
            const int h=(r+1)/2;
            consider(-r,-h,16);consider(-r,h,16);consider(r,-h,16);consider(r,h,16);
            consider(-h,-r,16);consider(h,-r,16);consider(-h,r,16);consider(h,r,16);
        }
        const auto coarse=top;
        for (size_t i=0;i<std::min<size_t>(4,coarse.size());++i) {
            const auto c=coarse[i];
            for (int oy=-step;oy<=step;oy+=4) for (int ox=-step;ox<=step;ox+=4)
                consider(c.dx+ox,c.dy+oy,8);
        }
        if (top.empty()) consider(0,0,8);
        const Candidate mid=top.front();
        for (int oy=-4;oy<=4;++oy) for (int ox=-4;ox<=4;++ox)
            consider(mid.dx+ox,mid.dy+oy,4);
        const Candidate fine=top.front();
        Candidate full{std::numeric_limits<uint64_t>::max(),0,0};
        for (int oy=-1;oy<=1;++oy) for (int ox=-1;ox<=1;++ox) {
            const int dx=std::clamp(fine.dx+ox,-range,range),dy=std::clamp(fine.dy+oy,-range,range);
            Candidate c{frame_sad(cur,ref,dx,dy,1),dx,dy};
            if (better(c,full)) full=c;
        }
        return {full.dx,full.dy,full.cost};
    }

GlobalMotionResult Vc1Encoder::global_motion_search(const Frame& cur,const Frame& ref,int range) const {
        // Global translation is part of the ordinary/local motion stage. Large
        // configured ranges are handled by the lazy content-match fallback, so
        // a huge --search-range never turns scene/P analysis into a huge frame
        // displacement sweep.
        range=std::min(range,32);
        if (range<=0) {
            const uint64_t sad=frame_sad(cur,ref,0,0,1);
            return {0,0,sad};
        }
        if (range>31) return global_motion_search_extended(cur,ref,range);
        struct Candidate { uint64_t cost; int dx; int dy; };
        const int stride=4;
        const int coarse_step=range>=4 ? 4 : (range>=2 ? 2 : 1);
        std::vector<Candidate> top;
        top.reserve(8);
        auto consider=[&](uint64_t cost,int dx,int dy) {
            Candidate c{cost,dx,dy};
            top.push_back(c);
            std::sort(top.begin(),top.end(),[](const Candidate& a,const Candidate& b) {
                if (a.cost != b.cost) return a.cost < b.cost;
                const int am=std::abs(a.dx)+std::abs(a.dy), bm=std::abs(b.dx)+std::abs(b.dy);
                if (am != bm) return am < bm;
                if (a.dy != b.dy) return a.dy < b.dy;
                return a.dx < b.dx;
            });
            if (top.size()>8) top.resize(8);
        };

        // Search a sparse grid over the whole configured window first.  The
        // old path evaluated every integer displacement with a 4x4 sampled
        // frame SAD, then immediately evaluated the winners again more densely.
        // A 4-pixel coarse grid plus local refinement keeps the same wide-pan
        // coverage while removing most of those redundant full-frame passes.
        for (int dy=-range; dy<=range; dy+=coarse_step) {
            for (int dx=-range; dx<=range; dx+=coarse_step)
                consider(frame_sad(cur,ref,dx,dy,stride),dx,dy);
        }
        // Always include zero motion and the positive window edges, which may
        // not lie on the coarse grid when range is not divisible by the step.
        consider(frame_sad(cur,ref,0,0,stride),0,0);
        for (int d=-range;d<=range;d+=coarse_step) {
            consider(frame_sad(cur,ref,range,d,stride),range,d);
            consider(frame_sad(cur,ref,d,range,stride),d,range);
        }
        if (top.empty()) top.push_back({frame_sad(cur,ref,0,0,1),0,0});

        // Refine around the best coarse candidates using the 2x2 sample.  A
        // a small seen-table avoids evaluating overlapping neighborhoods
        // multiple times when zero/global candidates cluster together.
        Candidate best{std::numeric_limits<uint64_t>::max(),0,0};
        std::array<uint8_t,63*63> seen{};
        for (const auto& c : top) {
            for (int dy=std::max(-range,c.dy-coarse_step+1);dy<=std::min(range,c.dy+coarse_step-1);++dy) {
                for (int dx=std::max(-range,c.dx-coarse_step+1);dx<=std::min(range,c.dx+coarse_step-1);++dx) {
                    const size_t key=static_cast<size_t>(dy+31)*63+static_cast<size_t>(dx+31);
                    if (seen[key]) continue;
                    seen[key]=1;
                    const uint64_t cost=frame_sad(cur,ref,dx,dy,2);
                    const int cm=std::abs(dx)+std::abs(dy), bm=std::abs(best.dx)+std::abs(best.dy);
                    if (cost < best.cost || (cost==best.cost && cm<bm)) best={cost,dx,dy};
                }
            }
        }
        return {best.dx,best.dy,frame_sad(cur,ref,best.dx,best.dy,1)};
    }

uint64_t Vc1Encoder::mb_sad(const Frame& cur,const Frame& ref,int mx,int my,int dx,int dy,int stride) const {
        uint64_t sad=0;
        const int x0=mx*16,y0=my*16;
        if (c_.syntax == StreamSyntax::Wmv9Main) {
            // WMV3/Main reconstructs complete coded macroblocks beyond the visible
            // dimensions.  This encoder deliberately exposes/stores cropped
            // references, so do not select a vector whose visible prediction
            // would depend on those hidden coded padding samples.  Zero motion
            // is always legal, so this cannot leave an MB without a candidate.
            const int vw=std::max(0,std::min(16,cur.width-x0));
            const int vh=std::max(0,std::min(16,cur.height-y0));
            if (x0+dx < 0 || y0+dy < 0 ||
                x0+vw-1+dx >= ref.width || y0+vh-1+dy >= ref.height)
                return std::numeric_limits<uint64_t>::max()/4;
        }
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (x0+16<=cur.width && y0+16<=cur.height &&
            x0+dx>=0 && y0+dy>=0 && x0+dx+16<=ref.width && y0+dy+16<=ref.height) {
            const uint8_t* a=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            const uint8_t* b=ref.y.data()+static_cast<size_t>(y0+dy)*ref.width+x0+dx;
#if defined(LIBVC1_HAVE_X86_64_V4)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::X86V4)
                return simd::block_sad_x86_64_v4(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::X86V1)
                return simd::block_sad_x86_64_v1(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::Prescott)
                return simd::block_sad_prescott(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_K10)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::K10)
                return simd::block_sad_k10(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_CONROE)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::Conroe)
                return simd::block_sad_conroe(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::X86V2)
                return simd::block_sad_x86_64_v2(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::Penryn)
                return simd::block_sad_penryn(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::SandyBridge)
                return simd::block_sad_sandybridge(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::Bulldozer)
                return simd::block_sad_bulldozer(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::Piledriver)
                return simd::block_sad_piledriver(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::Avx2Partial)
                return simd::block_sad_avx2_partial(a,cur.width,b,ref.width,16,16,stride);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
            if (c_.simd_for(SimdPrimitive::BlockSad)==SimdTier::X86V3)
                return simd::block_sad_x86_64_v3(a,cur.width,b,ref.width,16,16,stride);
#endif
        }
#endif
        for (int y=0;y<16 && y0+y<cur.height;y+=stride) {
            const int sy=std::clamp(y0+y+dy,0,ref.height-1);
            for (int x=0;x<16 && x0+x<cur.width;x+=stride) {
                const int sx=std::clamp(x0+x+dx,0,ref.width-1);
                sad+=static_cast<uint64_t>(std::abs(static_cast<int>(cur.y[static_cast<size_t>(y0+y)*cur.width+x0+x])-
                                                   static_cast<int>(ref.y[static_cast<size_t>(sy)*ref.width+sx])));
            }
        }
        if (stride>1) sad*=static_cast<uint64_t>(stride*stride);
        return sad;
    }

uint64_t Vc1Encoder::block_sad_qpel(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,MotionVector mv) const {
        const uint64_t bad=std::numeric_limits<uint64_t>::max()/4;
        const int ix=floor_div4(mv.xq), iy=floor_div4(mv.yq);
        const int hm=mv.xq-ix*4, vm=mv.yq-iy*4;
        const int sx0=x0+ix, sy0=y0+iy;
        const int aw=std::max(0,std::min(bw,cur.width-x0));
        const int ah=std::max(0,std::min(bh,cur.height-y0));
        const int tap_left=hm?1:0, tap_right=hm?2:0;
        const int tap_top=vm?1:0, tap_bottom=vm?2:0;
        const bool interior=sx0-tap_left>=0 && sy0-tap_top>=0 &&
                            sx0+aw-1+tap_right<ref.width && sy0+ah-1+tap_bottom<ref.height;
        if (c_.syntax==StreamSyntax::Wmv9Main) {
            // Preserve the historical Main-profile legality check exactly: it
            // uses the requested block extent (not the cropped active edge) so
            // candidates that would depend on hidden padded samples stay out.
            const int left=sx0-tap_left, top=sy0-tap_top;
            const int right=sx0+bw-1+tap_right, bottom=sy0+bh-1+tap_bottom;
            if (left<0 || top<0 || right>=ref.width || bottom>=ref.height) return bad;
        }
#if defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::X86V4 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_x86_64_v4(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            alignas(64) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<ah+2;++py) for (int px=-1;px<aw+2;++px)
                patch[static_cast<size_t>(py+1)*ps+px+1]=sample_clamped(ref.y,ref.width,ref.height,sx0+px,sy0+py);
            return simd::block_sad_luma_qpel_x86_64_v4(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::X86V1 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_x86_64_v1(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_x86_64_v1(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::Prescott && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_prescott(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_prescott(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_K10)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::K10 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_k10(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_k10(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_CONROE)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::Conroe && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_conroe(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_conroe(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::X86V2 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_x86_64_v2(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_x86_64_v2(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::Penryn && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_penryn(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_penryn(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::SandyBridge && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_sandybridge(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_sandybridge(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::Bulldozer && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_bulldozer(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_bulldozer(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::Piledriver && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_piledriver(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_piledriver(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::Avx2Partial && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_avx2_partial(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_avx2_partial(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        if (c_.simd_for(SimdPrimitive::LumaQpelSad)==SimdTier::X86V3 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_qpel_x86_64_v3(cp,cur.width,rp,ref.width,aw,ah,hm,vm,c_.rndctrl);
            }
            // Advanced-profile edge emulation is much cheaper as one tiny
            // clamped patch followed by the normal AVX2 filter than by doing
            // four coordinate clamps for every interpolation tap of every
            // candidate pixel. 24 is enough for a 16x16 block plus -1/+2 taps.
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=-1;py<=ah+1;++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=-1;px<=aw+1;++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py+1)*ps+px+1]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_qpel_x86_64_v3(cp,cur.width,patch.data()+ps+1,ps,aw,ah,hm,vm,c_.rndctrl);
        }
#endif

        // mv decomposition and interpolation mode are constant for the complete
        // candidate block.  The old pixel loop called luma_mc_sample(), which
        // repeated floor_div4(), mode selection, and boundary setup for every
        // sample.  Instantiate one raw/interior and one clamped edge path instead.
        auto accumulate=[&](auto sample) -> uint64_t {
            uint64_t sad=0;
            if (!hm && !vm) {
                for (int y=0;y<ah;++y) for (int x=0;x<aw;++x) {
                    const int pred=sample(sx0+x,sy0+y);
                    sad+=static_cast<uint64_t>(std::abs(static_cast<int>(cur.y[static_cast<size_t>(y0+y)*cur.width+x0+x])-pred));
                }
                return sad;
            }
            if (vm && hm) {
                static constexpr int shift_value[4]={0,5,1,5};
                const int shift=(shift_value[hm]+shift_value[vm])>>1;
                const int r1=(1<<(shift-1))+(c_.rndctrl?1:0)-1;
                for (int y=0;y<ah;++y) for (int x=0;x<aw;++x) {
                    const int sx=sx0+x,sy=sy0+y;
                    int t[4];
                    for (int k=-1;k<=2;++k) {
                        const int raw=mspel_filter4(sample(sx+k,sy-1),sample(sx+k,sy),
                                                    sample(sx+k,sy+1),sample(sx+k,sy+2),vm);
                        t[k+1]=arshift(raw+r1,shift);
                    }
                    const int raw=mspel_filter4(t[0],t[1],t[2],t[3],hm);
                    const int pred=std::clamp(arshift(raw+64-(c_.rndctrl?1:0),7),0,255);
                    sad+=static_cast<uint64_t>(std::abs(static_cast<int>(cur.y[static_cast<size_t>(y0+y)*cur.width+x0+x])-pred));
                }
                return sad;
            }
            if (vm) {
                const int bias=(vm==2)?(7+(c_.rndctrl?1:0)):(31+(c_.rndctrl?1:0));
                const int shift=vm==2?4:6;
                for (int y=0;y<ah;++y) for (int x=0;x<aw;++x) {
                    const int sx=sx0+x,sy=sy0+y;
                    const int raw=mspel_filter4(sample(sx,sy-1),sample(sx,sy),sample(sx,sy+1),sample(sx,sy+2),vm);
                    const int pred=std::clamp(arshift(raw+bias,shift),0,255);
                    sad+=static_cast<uint64_t>(std::abs(static_cast<int>(cur.y[static_cast<size_t>(y0+y)*cur.width+x0+x])-pred));
                }
                return sad;
            }
            const int bias=(hm==2)?(8-(c_.rndctrl?1:0)):(32-(c_.rndctrl?1:0));
            const int shift=hm==2?4:6;
            for (int y=0;y<ah;++y) for (int x=0;x<aw;++x) {
                const int sx=sx0+x,sy=sy0+y;
                const int raw=mspel_filter4(sample(sx-1,sy),sample(sx,sy),sample(sx+1,sy),sample(sx+2,sy),hm);
                const int pred=std::clamp(arshift(raw+bias,shift),0,255);
                sad+=static_cast<uint64_t>(std::abs(static_cast<int>(cur.y[static_cast<size_t>(y0+y)*cur.width+x0+x])-pred));
            }
            return sad;
        };

        if (interior) {
            const auto raw_sample=[&](int x,int y) {
                return static_cast<int>(ref.y[static_cast<size_t>(y)*ref.width+x]);
            };
            return accumulate(raw_sample);
        }
        const auto edge_sample=[&](int x,int y) {
            return static_cast<int>(sample_clamped(ref.y,ref.width,ref.height,x,y));
        };
        return accumulate(edge_sample);
    }

uint64_t Vc1Encoder::block_sad_motion(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                              MotionVector mv,ProgressiveMvMode mode) const {
        if (!mv_mode_bilinear(mode)) return block_sad_qpel(cur,ref,x0,y0,bw,bh,mv);
        const uint64_t bad=std::numeric_limits<uint64_t>::max()/4;
        if ((mv.xq&1)||(mv.yq&1)) return bad;
        const int ix=floor_div4(mv.xq),iy=floor_div4(mv.yq);
        const int fx=mv.xq-ix*4,fy=mv.yq-iy*4;
        if (c_.syntax==StreamSyntax::Wmv9Main) {
            const int left=x0+ix,top=y0+iy;
            const int right=left+bw-1+(fx?1:0),bottom=top+bh-1+(fy?1:0);
            if (left<0||top<0||right>=ref.width||bottom>=ref.height) return bad;
        }
        const int aw=std::max(0,std::min(bw,cur.width-x0));
        const int ah=std::max(0,std::min(bh,cur.height-y0));
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        const int sx0=x0+ix,sy0=y0+iy;
        const bool interior=sx0>=0 && sy0>=0 && sx0+aw-1+(fx?1:0)<ref.width && sy0+ah-1+(fy?1:0)<ref.height;
#if defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::X86V4 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_x86_64_v4(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(64) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) for (int px=0;px<aw+(fx?1:0);++px)
                patch[static_cast<size_t>(py)*ps+px]=sample_clamped(ref.y,ref.width,ref.height,sx0+px,sy0+py);
            return simd::block_sad_luma_bilinear_x86_64_v4(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::X86V1 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_x86_64_v1(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_x86_64_v1(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::Prescott && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_prescott(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_prescott(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_K10)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::K10 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_k10(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_k10(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_CONROE)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::Conroe && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_conroe(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_conroe(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::X86V2 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_x86_64_v2(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_x86_64_v2(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::Penryn && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_penryn(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_penryn(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::SandyBridge && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_sandybridge(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_sandybridge(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::Bulldozer && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_bulldozer(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_bulldozer(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::Piledriver && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_piledriver(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_piledriver(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::Avx2Partial && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_avx2_partial(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_avx2_partial(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        if (c_.simd_for(SimdPrimitive::LumaBilinearSad)==SimdTier::X86V3 && aw>0 && ah>0 && (aw%8)==0) {
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0)*cur.width+x0;
            if (interior) {
                const uint8_t* rp=ref.y.data()+static_cast<size_t>(sy0)*ref.width+sx0;
                return simd::block_sad_luma_bilinear_x86_64_v3(cp,cur.width,rp,ref.width,aw,ah,fx!=0,fy!=0,c_.rndctrl);
            }
            alignas(32) std::array<uint8_t,24*24> patch{};
            constexpr int ps=24;
            for (int py=0;py<ah+(fy?1:0);++py) {
                const int ry=std::clamp(sy0+py,0,ref.height-1);
                for (int px=0;px<aw+(fx?1:0);++px) {
                    const int rx=std::clamp(sx0+px,0,ref.width-1);
                    patch[static_cast<size_t>(py)*ps+px]=ref.y[static_cast<size_t>(ry)*ref.width+rx];
                }
            }
            return simd::block_sad_luma_bilinear_x86_64_v3(cp,cur.width,patch.data(),ps,aw,ah,fx!=0,fy!=0,c_.rndctrl);
        }
#endif
#endif
        uint64_t sad=0;
        for (int y=0;y<ah;++y) for (int x=0;x<aw;++x) {
            const int pred=luma_mc_sample_mode(ref.y,ref.width,ref.height,x0+x,y0+y,mv.xq,mv.yq,mode,c_.rndctrl);
            sad+=static_cast<uint64_t>(std::abs(static_cast<int>(cur.y[static_cast<size_t>(y0+y)*cur.width+x0+x])-pred));
        }
        return sad;
    }

namespace {
static uint64_t satd4x4_scalar(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride) {
    int d[4][4];
    for (int y=0;y<4;++y) for (int x=0;x<4;++x)
        d[y][x]=static_cast<int>(cur[static_cast<ptrdiff_t>(y)*cur_stride+x])-
                static_cast<int>(pred[static_cast<ptrdiff_t>(y)*pred_stride+x]);
    int t[4][4];
    for (int y=0;y<4;++y) {
        const int a0=d[y][0]+d[y][3],a1=d[y][1]+d[y][2];
        const int a2=d[y][1]-d[y][2],a3=d[y][0]-d[y][3];
        t[y][0]=a0+a1; t[y][1]=a3+a2;
        t[y][2]=a0-a1; t[y][3]=a3-a2;
    }
    uint64_t tile=0;
    for (int x=0;x<4;++x) {
        const int a0=t[0][x]+t[3][x],a1=t[1][x]+t[2][x];
        const int a2=t[1][x]-t[2][x],a3=t[0][x]-t[3][x];
        tile+=static_cast<uint64_t>(std::abs(a0+a1));
        tile+=static_cast<uint64_t>(std::abs(a3+a2));
        tile+=static_cast<uint64_t>(std::abs(a0-a1));
        tile+=static_cast<uint64_t>(std::abs(a3-a2));
    }
    return (tile+1)>>1;
}

} // namespace

uint64_t Vc1Encoder::block_satd_motion(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                                       MotionVector mv,ProgressiveMvMode mode) const {
        // SATD is a finalist metric, not a replacement for the wide-search SAD.
        // Generate the finalist prediction as a block so the new ME path can
        // reuse the already-dispatched SIMD qpel/bilinear MC kernels instead of
        // invoking luma_mc_sample_mode() separately for every candidate pixel.
        const int aw=std::max(0,std::min(bw,cur.width-x0));
        const int ah=std::max(0,std::min(bh,cur.height-y0));
        if (!aw || !ah) return 0;
        if (mv_mode_halfpel(mode) && ((mv.xq&1)||(mv.yq&1)))
            return std::numeric_limits<uint64_t>::max()/4;
        alignas(64) std::array<uint8_t,16*16> pred{};
        predict_luma_motion_block(pred.data(),16,ref,x0,y0,aw,ah,mv,mode);
        uint64_t total=0;
        const SimdTier satd_tier=c_.simd_for(SimdPrimitive::Satd4x4);
        auto satd_full=[&](const uint8_t* cp,const uint8_t* pp)->uint64_t {
            switch(satd_tier) {
              case SimdTier::None: return satd4x4_scalar(cp,cur.width,pp,16);
#if defined(LIBVC1_HAVE_X86_64_V1)
              case SimdTier::X86V1: return simd::satd4x4_x86_64_v1(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
              case SimdTier::Prescott: return simd::satd4x4_prescott(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_K10)
              case SimdTier::K10: return simd::satd4x4_k10(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_CONROE)
              case SimdTier::Conroe: return simd::satd4x4_conroe(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
              case SimdTier::Penryn: return simd::satd4x4_penryn(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
              case SimdTier::X86V2: return simd::satd4x4_x86_64_v2(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
              case SimdTier::SandyBridge: return simd::satd4x4_sandybridge(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
              case SimdTier::Bulldozer: return simd::satd4x4_bulldozer(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
              case SimdTier::Piledriver: return simd::satd4x4_piledriver(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
              case SimdTier::Avx2Partial: return simd::satd4x4_avx2_partial(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
              case SimdTier::X86V3: return simd::satd4x4_x86_64_v3(cp,cur.width,pp,16);
#endif
#if defined(LIBVC1_HAVE_X86_64_V4)
              case SimdTier::X86V4: return simd::satd4x4_x86_64_v4(cp,cur.width,pp,16);
#endif
              default: return satd4x4_scalar(cp,cur.width,pp,16);
            }
        };
        for (int by=0;by<bh;by+=4) for (int bx=0;bx<bw;bx+=4) {
            const int tw=std::max(0,std::min(4,aw-bx));
            const int th=std::max(0,std::min(4,ah-by));
            if (!tw || !th) continue;
            const uint8_t* cp=cur.y.data()+static_cast<size_t>(y0+by)*cur.width+x0+bx;
            const uint8_t* pp=pred.data()+static_cast<size_t>(by)*16+bx;
            if (tw==4 && th==4) {
                total+=satd_full(cp,pp);
            } else {
                // Visible-edge policy: hidden samples are zero residual, not an
                // extension of the last visible pixel.
                uint8_t ctmp[16]{},ptmp[16]{};
                for (int yy=0;yy<th;++yy) {
                    std::copy_n(cp+static_cast<ptrdiff_t>(yy)*cur.width,tw,ctmp+yy*4);
                    std::copy_n(pp+static_cast<ptrdiff_t>(yy)*16,tw,ptmp+yy*4);
                }
                total+=satd4x4_scalar(ctmp,4,ptmp,4);
            }
        }
        return total;
    }

bool Vc1Encoder::vulkan_motion_costs(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                                      const std::vector<MotionVector>& candidates,int sample_stride,
                                      ProgressiveMvMode mode,bool satd,std::vector<uint64_t>& costs) const {
        const int threshold=satd?c_.vulkan_min_batch_satd:c_.vulkan_min_batch_sad;
        if (!c_.vulkan_compute || c_.syntax!=StreamSyntax::Advanced || threshold<=0 ||
            candidates.size()<static_cast<size_t>(threshold))
            return false;
        std::vector<VulkanMotionCandidate> vk;
        vk.reserve(candidates.size());
        for (const auto mv:candidates) {
            if (mv_mode_halfpel(mode) && ((mv.xq&1)||(mv.yq&1))) return false;
            vk.push_back({mv.xq,mv.yq});
        }
        std::string error;
        return c_.vulkan_compute->score_motion_batch(
            cur.y.data(),ref.y.data(),cur.width,cur.height,x0,y0,bw,bh,
            sample_stride,mv_mode_bilinear(mode),c_.rndctrl,
            satd?VulkanMotionMetric::Satd:VulkanMotionMetric::Sad,
            vk.data(),vk.size(),costs,&error);
}


ComputeBenchmarkResult benchmark_vulkan_motion_compute(const EncoderConfig& cfg,
                                                        VulkanComputeContext& vk,
                                                        int user_min_batch) {
    ComputeBenchmarkResult out{};
    out.ran=true;

    // Benchmark the exact integer-SAD operations that make up the large fixed
    // UMH/rescue batches.  A synthetic 192x192 pair keeps every +/-32-pixel
    // candidate interior, so CPU and Vulkan spend their time on the normal
    // hot path rather than edge clamping.  The GPU is warmed before timing;
    // frame upload/allocation is amortized in real motion search too.
    EncoderConfig bench_cfg=cfg;
    bench_cfg.width=192; bench_cfg.height=192;
    bench_cfg.display_width=192; bench_cfg.display_height=192;
    bench_cfg.vulkan_compute.reset();
    Vc1Encoder cpu(bench_cfg);
    Frame cur{},ref{};
    cur.width=ref.width=192; cur.height=ref.height=192;
    cur.y.resize(static_cast<size_t>(192)*192);
    ref.y.resize(static_cast<size_t>(192)*192);
    uint32_t state=0x6d2b79f5u;
    for (size_t i=0;i<cur.y.size();++i) {
        state=state*1664525u+1013904223u; cur.y[i]=static_cast<uint8_t>(state>>24);
        state=state*1664525u+1013904223u; ref.y[i]=static_cast<uint8_t>(state>>24);
    }

    std::vector<MotionVector> integer_mvs;
    std::vector<VulkanMotionCandidate> integer_vk;
    integer_mvs.reserve(4096); integer_vk.reserve(4096);
    for (int dy=-32;dy<32;++dy) for (int dx=-32;dx<32;++dx) {
        integer_mvs.push_back({dx*4,dy*4});
        integer_vk.push_back({dx*4,dy*4});
    }

    volatile uint64_t keep=0;
    auto measure_ns=[](const std::function<void()>& fn) {
        // Grow repetitions until timing noise is small, then return the mean
        // batch latency.  Cap the loop so a pathological software Vulkan ICD
        // cannot make encoder startup unbounded.
        size_t reps=1;
        double elapsed_ns=0.0;
        for (;;) {
            const auto t0=std::chrono::steady_clock::now();
            for (size_t i=0;i<reps;++i) fn();
            const auto t1=std::chrono::steady_clock::now();
            elapsed_ns=std::chrono::duration<double,std::nano>(t1-t0).count();
            if (elapsed_ns>=2.0e6 || reps>=256) break;
            reps*=2;
        }
        return elapsed_ns/static_cast<double>(reps);
    };

    const int floor=std::max(1,user_min_batch);
    static constexpr std::array<int,10> sizes{{8,16,32,64,128,256,512,1024,2048,4096}};
    std::vector<uint64_t> gpu_costs;
    for (int n:sizes) {
        if (n<floor) continue;
        // Warm both GPU sample-stride variants outside the timing window.
        std::string error;
        if (!vk.score_motion_batch(cur.y.data(),ref.y.data(),192,192,64,64,16,16,1,false,false,
                                   VulkanMotionMetric::Sad,integer_vk.data(),static_cast<size_t>(n),gpu_costs,&error))
            break;
        if (!vk.score_motion_batch(cur.y.data(),ref.y.data(),192,192,64,64,16,16,4,false,false,
                                   VulkanMotionMetric::Sad,integer_vk.data(),static_cast<size_t>(n),gpu_costs,&error))
            break;

        const auto cpu_full=[&]() {
            uint64_t sum=0;
            for (int i=0;i<n;++i) sum+=cpu.mb_sad(cur,ref,4,4,integer_mvs[static_cast<size_t>(i)].xq/4,
                                                   integer_mvs[static_cast<size_t>(i)].yq/4,1);
            keep^=sum;
        };
        const auto gpu_full=[&]() {
            std::string e;
            if (!vk.score_motion_batch(cur.y.data(),ref.y.data(),192,192,64,64,16,16,1,false,false,
                                       VulkanMotionMetric::Sad,integer_vk.data(),static_cast<size_t>(n),gpu_costs,&e))
                throw std::runtime_error("Vulkan SAD benchmark failed: "+e);
            if (!gpu_costs.empty()) keep^=gpu_costs[0];
        };
        const auto cpu_sampled=[&]() {
            uint64_t sum=0;
            for (int i=0;i<n;++i) sum+=cpu.mb_sad(cur,ref,4,4,integer_mvs[static_cast<size_t>(i)].xq/4,
                                                   integer_mvs[static_cast<size_t>(i)].yq/4,4);
            keep^=sum;
        };
        const auto gpu_sampled=[&]() {
            std::string e;
            if (!vk.score_motion_batch(cur.y.data(),ref.y.data(),192,192,64,64,16,16,4,false,false,
                                       VulkanMotionMetric::Sad,integer_vk.data(),static_cast<size_t>(n),gpu_costs,&e))
                throw std::runtime_error("Vulkan sampled-SAD benchmark failed: "+e);
            if (!gpu_costs.empty()) keep^=gpu_costs[0];
        };

        const double cpu_full_ns=measure_ns(cpu_full);
        const double gpu_full_ns=measure_ns(gpu_full);
        const double cpu_sampled_ns=measure_ns(cpu_sampled);
        const double gpu_sampled_ns=measure_ns(gpu_sampled);
        // One threshold is shared by both integer SAD variants.  Be
        // conservative: enable a batch size only once Vulkan beats CPU for
        // both the full-resolution and sampled searches actually used by ME.
        if (gpu_full_ns<cpu_full_ns && gpu_sampled_ns<cpu_sampled_ns) {
            out.sad_min_batch=n;
            const double cpu_ns=cpu_full_ns+cpu_sampled_ns;
            const double gpu_ns=gpu_full_ns+gpu_sampled_ns;
            out.cpu_sad_candidates_per_second=(2.0*n)/(cpu_ns*1.0e-9);
            out.vulkan_sad_candidates_per_second=(2.0*n)/(gpu_ns*1.0e-9);
            break;
        }
    }

    // The current SATD offload is the <=8-candidate finalist shortlist.  Test
    // that exact batch rather than extrapolating a large-batch GPU win down to
    // a latency-sensitive dispatch.  If the user floor is above 8, AUTO leaves
    // SATD on CPU because no production SATD batch can meet that floor.
    if (floor<=8) {
        std::array<MotionVector,8> satd_mvs{{
            MotionVector{0,0},MotionVector{1,0},MotionVector{-1,0},MotionVector{0,1},
            MotionVector{0,-1},MotionVector{1,1},MotionVector{-1,1},MotionVector{2,0}}};
        std::array<VulkanMotionCandidate,8> satd_vk{};
        for (size_t i=0;i<satd_mvs.size();++i) satd_vk[i]={satd_mvs[i].xq,satd_mvs[i].yq};
        std::string error;
        if (vk.score_motion_batch(cur.y.data(),ref.y.data(),192,192,64,64,16,16,1,false,false,
                                  VulkanMotionMetric::Satd,satd_vk.data(),satd_vk.size(),gpu_costs,&error)) {
            const auto cpu_satd=[&]() {
                uint64_t sum=0;
                for (const auto mv:satd_mvs)
                    sum+=cpu.block_satd_motion(cur,ref,64,64,16,16,mv,Vc1Encoder::ProgressiveMvMode::OneMvQpel);
                keep^=sum;
            };
            const auto gpu_satd=[&]() {
                std::string e;
                if (!vk.score_motion_batch(cur.y.data(),ref.y.data(),192,192,64,64,16,16,1,false,false,
                                           VulkanMotionMetric::Satd,satd_vk.data(),satd_vk.size(),gpu_costs,&e))
                    throw std::runtime_error("Vulkan SATD benchmark failed: "+e);
                if (!gpu_costs.empty()) keep^=gpu_costs[0];
            };
            const double cpu_ns=measure_ns(cpu_satd);
            const double gpu_ns=measure_ns(gpu_satd);
            out.cpu_satd_candidates_per_second=8.0/(cpu_ns*1.0e-9);
            out.vulkan_satd_candidates_per_second=8.0/(gpu_ns*1.0e-9);
            if (gpu_ns<cpu_ns) out.satd_min_batch=8;
        }
    }
    (void)keep;
    vk.reset_counters();
    return out;
}

uint64_t Vc1Encoder::motion_vector_rate_bits(MotionVector desired,MotionVector pred,
                                              ProgressiveMvMode mode,bool more) const {
        const MvDataSyntax mv=mvdata_for_mode(desired,pred,mode,more);
        const int sym=mv_symbol(mv);
        // MVTAB is selected picture-wide.  Table 0 is intentionally used as a
        // stable local proxy here; the final picture estimator still chooses
        // and charges the real table.  The important improvement over SAD-only
        // ME is that the actual VC-1 differential category and escape suffix are
        // represented during candidate selection.
        return static_cast<uint64_t>(groupa::kMvDiffBits[0][static_cast<size_t>(sym)])+
               static_cast<uint64_t>(mv_suffix_bits(mv));
    }

long double Vc1Encoder::motion_mb_rd_cost(const Frame& cur,const Frame& ref,int mx,int my,MotionVector mv,
                                          ProgressiveMvMode mode,uint64_t mv_bits,Frame& pred) const {
        motion_compensate_mb(pred,ref,mx,my,mv,mode);
        const int decision_index=(c_.pqindex<=8)?1:0;
        const int coding_set=chroma_coding_set(decision_index,c_.pqindex);
        const bool use_vlc=c_.ac_mode!=AcMode::Esc3;
        const MbTransformDecision tx=choose_transform_mb(cur,pred,mx,my,coding_set,use_vlc,c_.pqindex,false);
        const double q=static_cast<double>(picture_double_quant(c_.pqindex,false));
        // Keep the outer motion-RD comparison on the same perceptual law as
        // transform trellis/AQ.  Otherwise a finalist can win on nominal bit
        // efficiency while spending too little distortion budget in a block
        // that AQ intentionally protects.  With AQ disabled this is exactly 1.0.
        const double lscale=perceptual_lambda_scale(cur.y,c_.width,c_.height,
                                                    mx*16,my*16,16,16,&pred.y);
        const long double lambda=0.75L*static_cast<long double>(q)*static_cast<long double>(q)*
                                 static_cast<long double>(lscale);
        return static_cast<long double>(tx.distortion)+
               lambda*static_cast<long double>(tx.estimated_bits+mv_bits);
    }

MotionVector Vc1Encoder::refine_motion_block_staged(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                                                     MotionVector seed,int range,ProgressiveMvMode mode,
                                                     const PredictorInfo& pi,bool more,Frame* rd_pred,
                                                     uint64_t* best_sad,uint64_t known_seed_sad) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionSubpel);
        if (c_.me_quality==VC1_ME_SAD)
            return refine_motion_block(cur,ref,x0,y0,bw,bh,seed,range,mode,best_sad,known_seed_sad);

        struct Candidate {
            MotionVector mv{};
            uint64_t sad=0;
            uint64_t rate=0;
            uint64_t satd=0;
            long double score=0.0L;
            bool satd_eligible=true;
        };
        std::vector<Candidate> all;
        all.reserve(20);
        const auto qb=motion_search_bounds_qpel(range);
        auto make_even=[](int v) { if (!(v&1)) return v; return v>0?v-1:v+1; };
        if (mv_mode_halfpel(mode)) {
            seed.xq=make_even(std::clamp(seed.xq,qb.xmin,qb.xmax));
            seed.yq=make_even(std::clamp(seed.yq,qb.ymin,qb.ymax));
        } else {
            seed.xq=std::clamp(seed.xq,qb.xmin,qb.xmax);
            seed.yq=std::clamp(seed.yq,qb.ymin,qb.ymax);
        }
        auto tie_less=[](MotionVector a,MotionVector b) {
            const int am=std::abs(a.xq)+std::abs(a.yq),bm=std::abs(b.xq)+std::abs(b.yq);
            if (am!=bm) return am<bm;
            if (a.yq!=b.yq) return a.yq<b.yq;
            return a.xq<b.xq;
        };
        auto find_candidate=[&](MotionVector mv)->Candidate* {
            for (auto& c:all) if (same_mv(c.mv,mv)) return &c;
            return nullptr;
        };
        auto add=[&](MotionVector mv,uint64_t known=std::numeric_limits<uint64_t>::max()) -> Candidate& {
            if (auto* c=find_candidate(mv)) return *c;
            Candidate c{}; c.mv=mv;
            c.sad=known!=std::numeric_limits<uint64_t>::max()
                ? known : block_sad_motion(cur,ref,x0,y0,bw,bh,mv,mode);
            all.push_back(c);
            return all.back();
        };
        add(seed,known_seed_sad);
        auto add_batch=[&](const std::vector<MotionVector>& batch) {
            std::vector<uint64_t> gpu_costs;
            if (vulkan_motion_costs(cur,ref,x0,y0,bw,bh,batch,1,mode,false,gpu_costs) &&
                gpu_costs.size()==batch.size()) {
                for (size_t i=0;i<batch.size();++i) add(batch[i],gpu_costs[i]);
            } else {
                for (const auto mv:batch) add(mv);
            }
        };

        // Candidate discovery deliberately follows the historical refinement
        // geometry.  SAD remains the cheap gate for the wide search; Vulkan can
        // score each fixed refinement stencil as one batch while preserving the
        // exact CPU ordering/tie law.
        MotionVector half_best=seed;
        uint64_t half_sad=all.front().sad;
        std::vector<MotionVector> half_batch;
        half_batch.reserve(9);
        for (int oy=-2;oy<=2;oy+=2) for (int ox=-2;ox<=2;ox+=2) {
            MotionVector mv{std::clamp(seed.xq+ox,qb.xmin,qb.xmax),std::clamp(seed.yq+oy,qb.ymin,qb.ymax)};
            if (mv_mode_halfpel(mode)) { mv.xq=make_even(mv.xq); mv.yq=make_even(mv.yq); }
            half_batch.push_back(mv);
        }
        add_batch(half_batch);
        for (const auto mv:half_batch) {
            if (auto* c=find_candidate(mv); c && (c->sad<half_sad || (c->sad==half_sad && tie_less(c->mv,half_best)))) {
                half_sad=c->sad; half_best=c->mv;
            }
        }
        if (!mv_mode_halfpel(mode)) {
            std::vector<MotionVector> quarter_batch;
            quarter_batch.reserve(9);
            for (int oy=-1;oy<=1;++oy) for (int ox=-1;ox<=1;++ox)
                quarter_batch.push_back({std::clamp(half_best.xq+ox,qb.xmin,qb.xmax),
                                         std::clamp(half_best.yq+oy,qb.ymin,qb.ymax)});
            add_batch(quarter_batch);
        }

        std::sort(all.begin(),all.end(),[&](const Candidate& a,const Candidate& b) {
            if (a.sad!=b.sad) return a.sad<b.sad;
            return tie_less(a.mv,b.mv);
        });
        // Keep the historical meaning of best_sad: it is the best temporal
        // match discovered by SAD, not necessarily the SAD of the later
        // rate/SATD/RD winner.  Scene-cut and inter-vs-intra heuristics consume
        // this value and must not be biased merely because RD deliberately
        // trades a few SAD units for fewer bits or a more compressible residual.
        const uint64_t screen_sad=all.front().sad;
        if (all.size()>8) all.resize(8);

        const long double lambda_me=std::max(1.0,0.35*static_cast<double>(c_.pqindex));
        auto rate_for=[&](Candidate& cand) {
            MotionVector pred=pi.pre;
            cand.rate=0;
            if (pi.hybrid) {
                pred=modular_mv_distance(cand.mv,pi.a)<=modular_mv_distance(cand.mv,pi.c)?pi.a:pi.c;
                ++cand.rate; // HYBRIDPRED selector bit.
            }
            cand.rate+=motion_vector_rate_bits(cand.mv,pred,mode,more);
        };
        for (auto& cand:all) {
            rate_for(cand);
            cand.score=static_cast<long double>(cand.sad)+lambda_me*static_cast<long double>(cand.rate);
        }
        auto score_less=[&](const Candidate& a,const Candidate& b) {
            if (a.score!=b.score) return a.score<b.score;
            if (a.sad!=b.sad) return a.sad<b.sad;
            return tie_less(a.mv,b.mv);
        };
        const Candidate rate_winner=*std::min_element(all.begin(),all.end(),score_less);
        if (c_.me_quality==VC1_ME_RATE) {
            if (best_sad) *best_sad=screen_sad;
            return rate_winner.mv;
        }

        // SATD is intentionally restricted to the SAD shortlist.  It is a much
        // better proxy for transform coefficient cost, but it is not allowed to
        // replace a materially better temporal match.  Periodic/high-frequency
        // textures can have a lower Hadamard cost at the wrong phase; under ABR
        // that locally cheap residual can then steal bits from later blocks.
        // Keep SATD as a tie/refinement metric inside a narrow SAD envelope.
        // Three percent was selected against the deterministic ABR/AQ motion
        // stress fixture: four percent admits the known wrong-phase periodic
        // match, while three percent preserves a large quality margin.  The
        // small absolute allowance avoids over-constraining nearly-flat blocks
        // whose SAD is only a handful of levels.  Spell out the percentage
        // without multiplication overflow so this remains deterministic for
        // every legal block cost.
        const uint64_t pct3=(rate_winner.sad/100)*3 +
            (((rate_winner.sad%100)*3+99)/100);
        const uint64_t sad_guard_slack=std::max<uint64_t>(4,pct3);
        const uint64_t sad_guard_limit=rate_winner.sad > std::numeric_limits<uint64_t>::max()-sad_guard_slack
            ? std::numeric_limits<uint64_t>::max() : rate_winner.sad+sad_guard_slack;
        std::vector<MotionVector> satd_batch;
        satd_batch.reserve(all.size());
        for (const auto& cand:all) satd_batch.push_back(cand.mv);
        std::vector<uint64_t> satd_costs;
        const bool gpu_satd=vulkan_motion_costs(cur,ref,x0,y0,bw,bh,satd_batch,1,mode,true,satd_costs) &&
                            satd_costs.size()==all.size();
        for (size_t i=0;i<all.size();++i) {
            auto& cand=all[i];
            cand.satd=gpu_satd?satd_costs[i]:block_satd_motion(cur,ref,x0,y0,bw,bh,cand.mv,mode);
            cand.satd_eligible=cand.sad<=sad_guard_limit;
            cand.score=static_cast<long double>(cand.satd)+lambda_me*static_cast<long double>(cand.rate);
        }
        std::sort(all.begin(),all.end(),[&](const Candidate& a,const Candidate& b) {
            if (a.satd_eligible!=b.satd_eligible) return a.satd_eligible>b.satd_eligible;
            if (a.score!=b.score) return a.score<b.score;
            if (a.satd!=b.satd) return a.satd<b.satd;
            if (a.sad!=b.sad) return a.sad<b.sad;
            return tie_less(a.mv,b.mv);
        });
        if (c_.me_quality==VC1_ME_SATD || !rd_pred) {
            if (best_sad) *best_sad=screen_sad;
            return all.front().mv;
        }
        // A single 8x8 vector cannot be scored with the final codec RD law in
        // isolation: VC-1 transform shape, CBP, chroma prediction, and 4-MV
        // signaling are macroblock decisions.  In full-RD mode use the stronger
        // SAD+MV-rate result for these subblocks, then evaluate the assembled
        // 4-MV macroblock against 1-MV with the complete transform/chroma RD
        // model below.  This avoids treating subblock SATD as if it were a
        // separable final RD metric.
        if (bw!=16 || bh!=16 || (x0&15) || (y0&15)) {
            if (best_sad) *best_sad=screen_sad;
            return rate_winner.mv;
        }

        // Full RD is deliberately tiny.  Start with the best four SATD+rate
        // candidates, but always preserve the raw-SAD and SAD+MV-rate winners.
        // SATD is a transform proxy rather than a dominance proof; on periodic
        // texture it can rank a visually poorer displacement very highly.  The
        // union prevents that proxy from starving the real codec-aware stage of
        // the strongest temporal/rate candidate.
        std::vector<Candidate> finalists;
        finalists.reserve(6);
        auto add_finalist=[&](const Candidate& c) {
            for (const auto& f:finalists) if (same_mv(f.mv,c.mv)) return;
            finalists.push_back(c);
        };
        size_t satd_finalists=0;
        for (const auto& cand:all) {
            if (!cand.satd_eligible) continue;
            add_finalist(cand);
            if (++satd_finalists==4) break;
        }
        const auto sad_it=std::min_element(all.begin(),all.end(),[&](const Candidate& a,const Candidate& b) {
            if (a.sad!=b.sad) return a.sad<b.sad;
            return tie_less(a.mv,b.mv);
        });
        add_finalist(*sad_it);
        const auto rate_it=std::min_element(all.begin(),all.end(),[&](const Candidate& a,const Candidate& b) {
            const long double as=static_cast<long double>(a.sad)+lambda_me*static_cast<long double>(a.rate);
            const long double bs=static_cast<long double>(b.sad)+lambda_me*static_cast<long double>(b.rate);
            if (as!=bs) return as<bs;
            if (a.sad!=b.sad) return a.sad<b.sad;
            return tie_less(a.mv,b.mv);
        });
        add_finalist(*rate_it);

        // choose_transform_mb() evaluates Y/U/V, so chroma influences the final
        // vector without making the broad search chroma-expensive.
        size_t winner=0;
        long double best=std::numeric_limits<long double>::infinity();
        for (size_t i=0;i<finalists.size();++i) {
            finalists[i].score=motion_mb_rd_cost(cur,ref,x0/16,y0/16,finalists[i].mv,mode,finalists[i].rate,*rd_pred);
            if (finalists[i].score<best ||
                (finalists[i].score==best && tie_less(finalists[i].mv,finalists[winner].mv))) {
                best=finalists[i].score; winner=i;
            }
        }
        if (best_sad) *best_sad=screen_sad;
        return finalists[winner].mv;
    }

MotionVector Vc1Encoder::refine_qpel_block(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                                   MotionVector seed,int range,uint64_t* best_sad,
                                   uint64_t known_seed_sad) const {
        const auto qb=motion_search_bounds_qpel(range);
        seed.xq=std::clamp(seed.xq,qb.xmin,qb.xmax); seed.yq=std::clamp(seed.yq,qb.ymin,qb.ymax);
        auto better=[](uint64_t sad,MotionVector mv,uint64_t best,MotionVector bmv) {
            const int mag=std::abs(mv.xq)+std::abs(mv.yq), bmag=std::abs(bmv.xq)+std::abs(bmv.yq);
            if (sad!=best) return sad<best;
            if (mag!=bmag) return mag<bmag;
            if (mv.yq!=bmv.yq) return mv.yq<bmv.yq;
            return mv.xq<bmv.xq;
        };
        MotionVector best=seed;
        uint64_t cost=known_seed_sad!=std::numeric_limits<uint64_t>::max()
            ? known_seed_sad : block_sad_qpel(cur,ref,x0,y0,bw,bh,best);
        std::array<MotionVector,20> seen{};
        size_t seen_count=1; seen[0]=seed;
        auto seen_before=[&](MotionVector mv) {
            for (size_t i=0;i<seen_count;++i) if (same_mv(seen[i],mv)) return true;
            if (seen_count<seen.size()) seen[seen_count++]=mv;
            return false;
        };
        // The center sample is already known.  Older code evaluated it once in
        // the seed setup, again in the half-pixel grid, and again in the qpel
        // grid.  Skip those exact duplicates without changing candidate order.
        for (int oy=-2;oy<=2;oy+=2) for (int ox=-2;ox<=2;ox+=2) {
            MotionVector mv{std::clamp(seed.xq+ox,qb.xmin,qb.xmax),std::clamp(seed.yq+oy,qb.ymin,qb.ymax)};
            if (seen_before(mv)) continue;
            const uint64_t c=block_sad_qpel(cur,ref,x0,y0,bw,bh,mv);
            if (better(c,mv,cost,best)) { cost=c; best=mv; }
        }
        const MotionVector half=best;
        for (int oy=-1;oy<=1;++oy) for (int ox=-1;ox<=1;++ox) {
            MotionVector mv{std::clamp(half.xq+ox,qb.xmin,qb.xmax),std::clamp(half.yq+oy,qb.ymin,qb.ymax)};
            if (seen_before(mv)) continue;
            const uint64_t c=block_sad_qpel(cur,ref,x0,y0,bw,bh,mv);
            if (better(c,mv,cost,best)) { cost=c; best=mv; }
        }
        if (best_sad) *best_sad=cost;
        return best;
    }

MotionVector Vc1Encoder::refine_motion_block(const Frame& cur,const Frame& ref,int x0,int y0,int bw,int bh,
                                     MotionVector seed,int range,ProgressiveMvMode mode,uint64_t* best_sad,
                                     uint64_t known_seed_sad) const {
        if (!mv_mode_halfpel(mode)) return refine_qpel_block(cur,ref,x0,y0,bw,bh,seed,range,best_sad,known_seed_sad);
        const auto qb=motion_search_bounds_qpel(range);
        auto make_even=[](int v) { if (!(v&1)) return v; return v>0?v-1:v+1; };
        seed.xq=make_even(std::clamp(seed.xq,qb.xmin,qb.xmax));
        seed.yq=make_even(std::clamp(seed.yq,qb.ymin,qb.ymax));
        auto better=[](uint64_t sad,MotionVector mv,uint64_t best,MotionVector bmv) {
            if (sad!=best) return sad<best;
            const int m=std::abs(mv.xq)+std::abs(mv.yq),bm=std::abs(bmv.xq)+std::abs(bmv.yq);
            if (m!=bm) return m<bm;
            if (mv.yq!=bmv.yq) return mv.yq<bmv.yq;
            return mv.xq<bmv.xq;
        };
        MotionVector best=seed;
        uint64_t cost=known_seed_sad!=std::numeric_limits<uint64_t>::max()?known_seed_sad:block_sad_motion(cur,ref,x0,y0,bw,bh,seed,mode);
        std::array<MotionVector,9> seen{};
        size_t seen_count=1; seen[0]=seed;
        auto seen_before=[&](MotionVector mv) {
            for (size_t i=0;i<seen_count;++i) if (same_mv(seen[i],mv)) return true;
            if (seen_count<seen.size()) seen[seen_count++]=mv;
            return false;
        };
        for (int oy=-2;oy<=2;oy+=2) for (int ox=-2;ox<=2;ox+=2) {
            MotionVector mv{make_even(std::clamp(seed.xq+ox,qb.xmin,qb.xmax)),make_even(std::clamp(seed.yq+oy,qb.ymin,qb.ymax))};
            if (seen_before(mv)) continue;
            const uint64_t c=block_sad_motion(cur,ref,x0,y0,bw,bh,mv,mode);
            if (better(c,mv,cost,best)) { cost=c; best=mv; }
        }
        if (best_sad) *best_sad=cost;
        return best;
    }

} // namespace libvc1
