#include "encoder_internal.h"
#include "forward_transform_tables.h"

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


static bool simd_dequant_dispatch(SimdTier t,const int* q,int* out,int count,int dq,int mq,bool nonuniform) {
#if defined(LIBVC1_HAVE_X86_64_V4)
    if(t==SimdTier::X86V4){simd::dequant_coefficients_x86_64_v4(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
    if(t==SimdTier::Avx2Partial){simd::dequant_coefficients_avx2_partial(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
    if(t==SimdTier::X86V3){simd::dequant_coefficients_x86_64_v3(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
    if(t==SimdTier::Piledriver){simd::dequant_coefficients_piledriver(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
    if(t==SimdTier::Bulldozer){simd::dequant_coefficients_bulldozer(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
    if(t==SimdTier::SandyBridge){simd::dequant_coefficients_sandybridge(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
    if(t==SimdTier::X86V2){simd::dequant_coefficients_x86_64_v2(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_PENRYN)
    if(t==SimdTier::Penryn){simd::dequant_coefficients_penryn(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_CONROE)
    if(t==SimdTier::Conroe){simd::dequant_coefficients_conroe(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_K10)
    if(t==SimdTier::K10){simd::dequant_coefficients_k10(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
    if(t==SimdTier::Prescott){simd::dequant_coefficients_prescott(q,out,count,dq,mq,nonuniform);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
    if(t==SimdTier::X86V1){simd::dequant_coefficients_x86_64_v1(q,out,count,dq,mq,nonuniform);return true;}
#endif
    return false;
}
static bool simd_inverse_rect_dispatch(SimdTier t,int* block,int type) {
#if defined(LIBVC1_HAVE_X86_64_V4)
    if(t==SimdTier::X86V4){simd::inverse_transform_rect_x86_64_v4(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
    if(t==SimdTier::Avx2Partial){simd::inverse_transform_rect_avx2_partial(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
    if(t==SimdTier::X86V3){simd::inverse_transform_rect_x86_64_v3(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
    if(t==SimdTier::Piledriver){simd::inverse_transform_rect_piledriver(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
    if(t==SimdTier::Bulldozer){simd::inverse_transform_rect_bulldozer(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
    if(t==SimdTier::SandyBridge){simd::inverse_transform_rect_sandybridge(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
    if(t==SimdTier::X86V2){simd::inverse_transform_rect_x86_64_v2(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_PENRYN)
    if(t==SimdTier::Penryn){simd::inverse_transform_rect_penryn(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_CONROE)
    if(t==SimdTier::Conroe){simd::inverse_transform_rect_conroe(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_K10)
    if(t==SimdTier::K10){simd::inverse_transform_rect_k10(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
    if(t==SimdTier::Prescott){simd::inverse_transform_rect_prescott(block,type);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
    if(t==SimdTier::X86V1){simd::inverse_transform_rect_x86_64_v1(block,type);return true;}
#endif
    return false;
}

// Compact coefficient storage helpers.  TransformParentDecision keeps one
// spatial 8x8 coefficient grid; rectangular/4x4 partitions occupy their
// natural sub-rectangle inside that grid.  Materialize the old local 8-stride
// representation only at the few entropy/reconstruction call sites that need it.
static void store_parent_part(TransformParentDecision& p,TransformType t,int part,
                              const std::array<int,64>& q) {
    int ox=0,oy=0,bw=8,bh=8;
    Vc1Encoder::transform_part_geometry(t,part,ox,oy,bw,bh);
    for (int y=0;y<bh;++y) for (int x=0;x<bw;++x)
        p.qcoeff[static_cast<size_t>(oy+y)*8+ox+x]=
            static_cast<int16_t>(q[static_cast<size_t>(y)*8+x]);
}

static std::array<int,64> load_parent_part(const TransformParentDecision& p,TransformType t,int part) {
    std::array<int,64> q{};
    int ox=0,oy=0,bw=8,bh=8;
    Vc1Encoder::transform_part_geometry(t,part,ox,oy,bw,bh);
    for (int y=0;y<bh;++y) for (int x=0;x<bw;++x)
        q[static_cast<size_t>(y)*8+x]=
            static_cast<int>(p.qcoeff[static_cast<size_t>(oy+y)*8+ox+x]);
    return q;
}

TransformPicturePlan Vc1Encoder::choose_p_transform_plan(const Frame& f,const Frame& ref,const PAnalysis& a,
                                                   int coding_set,bool use_vlc,Frame* pred_scratch) const {
        TransformPicturePlan plan{};
        if (!c_.variable_transforms || !c_.extended_transform_signaling) return plan;
        // A frame-level TTFRM law cannot make a different transform choice for
        // the partial right/bottom macroblock.  Keep transform signaling local
        // on non-16-aligned pictures so an edge parent can be split instead of
        // allowing one 8x8 transform to straddle visible and hidden samples.
        if ((c_.width & 15) || (c_.height & 15)) return plan;
        const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
        const int total=mbw*mbh;
        if (total<=0) return plan;
        int inter_total=0;
        for (uint8_t v:a.use_intra) if (!v) ++inter_total;
        if (!inter_total) return plan;

        Frame owned_pred;
        if (!pred_scratch) { owned_pred=empty_frame(); pred_scratch=&owned_pred; }
        Frame& pred=*pred_scratch;
        const Frame& prediction_ref=(a.intensity.enabled && a.compensated_reference) ? *a.compensated_reference : ref;
        const int wanted=std::min(32,inter_total);
        const int stride=std::max(1,total/std::max(1,wanted));
        const double q=static_cast<double>(picture_double_quant(c_.pqindex,false));
        const double header_lambda=0.75*q*q;
        // Picture-level TTFRM selection is AQ-neutral for the same reason as
        // picture-level motion-mode selection: AQ should refine the residual
        // after the prediction/transform signaling law is chosen, not change
        // that global law and swamp its own local protection. The final encode
        // still runs full AQ/DQUANT/trellis RDO under the selected plan.
        EncoderConfig neutral_cfg=c_;
        neutral_cfg.adaptive_quality=false;
        neutral_cfg.aq_strength=0.0;
        Vc1Encoder neutral_eval(neutral_cfg);
        long double local_rd=0.0L;
        std::array<long double,4> frame_rd{};
        int sampled=0;
        for (int linear=0;linear<total && sampled<wanted;linear+=stride) {
            const size_t pos=static_cast<size_t>(linear);
            if (a.use_intra[pos]) continue;
            const int my=linear/mbw,mx=linear%mbw;
            if (a.use_4mv[pos]) motion_compensate_4mv(pred,prediction_ref,mx,my,a.block_mvs[pos]);
            else motion_compensate_mb(pred,prediction_ref,mx,my,a.mvs[pos],a.mv_mode);
            const double lambda=header_lambda;
            const MbTransformDecision local=neutral_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false,nullptr);
            local_rd+=static_cast<long double>(local.distortion)+lambda*static_cast<long double>(local.estimated_bits);
            for (int ti=0;ti<4;++ti) {
                const TransformType t=static_cast<TransformType>(ti+1);
                MbTransformDecision d=neutral_eval.evaluate_transform_type(f,pred,mx,my,t,coding_set,use_vlc,true,c_.pqindex,false);
                d.signal_level=TransformSignalLevel::Frame;
                neutral_eval.refresh_transform_estimate(d,coding_set,use_vlc,false);
                frame_rd[static_cast<size_t>(ti)]+=static_cast<long double>(d.distortion)+
                    lambda*static_cast<long double>(d.estimated_bits);
            }
            ++sampled;
        }
        if (!sampled) return plan;
        const long double scale=static_cast<long double>(inter_total)/static_cast<long double>(sampled);
        local_rd=local_rd*scale + header_lambda; // TTMBF=0
        int best=0;
        long double best_rd=frame_rd[0]*scale + header_lambda*3.0L; // TTMBF=1 + TTFRM(2)
        for (int ti=1;ti<4;++ti) {
            const long double r=frame_rd[static_cast<size_t>(ti)]*scale + header_lambda*3.0L;
            if (r<best_rd) { best_rd=r; best=ti; }
        }
        if (best_rd < local_rd*0.9999L) {
            plan.frame_level=true;
            plan.frame_type=static_cast<TransformType>(best+1);
        }
        return plan;
    }

TransformPicturePlan Vc1Encoder::choose_b_transform_plan(const Frame& f,const Frame& past,const Frame& future,
                                                   const BAnalysis& a,int coding_set,bool use_vlc,Frame* pred_scratch) const {
        TransformPicturePlan plan{};
        if (!c_.variable_transforms || !c_.extended_transform_signaling) return plan;
        // Partial edge macroblocks need local transform signaling so their 8x8
        // parents can be split at the display edge.
        if ((c_.width & 15) || (c_.height & 15)) return plan;
        const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
        const int total=mbw*mbh;
        if (total<=0) return plan;
        const int inter_total=total-static_cast<int>(std::min<size_t>(a.intra_macroblocks,static_cast<size_t>(total)));
        if (inter_total<=0) return plan;
        Frame owned_pred;
        if (!pred_scratch) { owned_pred=empty_frame(); pred_scratch=&owned_pred; }
        Frame& pred=*pred_scratch;
        const Frame& forward_ref=(a.forward_intensity.enabled && a.compensated_past) ? *a.compensated_past : past;
        const int wanted=std::min(32,total);
        const int stride=std::max(1,total/wanted);
        const double q=static_cast<double>(picture_double_quant(c_.pqindex,false));
        const double header_lambda=0.75*q*q;
        // Picture-level TTFRM selection is AQ-neutral for the same reason as
        // picture-level motion-mode selection: AQ should refine the residual
        // after the prediction/transform signaling law is chosen, not change
        // that global law and swamp its own local protection. The final encode
        // still runs full AQ/DQUANT/trellis RDO under the selected plan.
        EncoderConfig neutral_cfg=c_;
        neutral_cfg.adaptive_quality=false;
        neutral_cfg.aq_strength=0.0;
        Vc1Encoder neutral_eval(neutral_cfg);
        long double local_rd=0.0L;
        std::array<long double,4> frame_rd{};
        int sampled=0;
        for (int linear=0;linear<total && sampled<wanted;++linear) {
            const int my=linear/mbw,mx=linear%mbw;
            const size_t pos=static_cast<size_t>(linear);
            if (a.modes[pos]==BMbMode::Intra) continue;
            if (stride>1 && (linear%stride)!=0 && sampled+((total-linear-1)/stride)>=wanted) continue;
            if (a.modes[pos]==BMbMode::Forward)
                motion_compensate_mb(pred,forward_ref,mx,my,a.forward_mvs[pos],a.mv_mode);
            else if (a.modes[pos]==BMbMode::Backward)
                motion_compensate_mb(pred,future,mx,my,a.backward_mvs[pos],a.mv_mode);
            else
                motion_compensate_bi_mb(pred,forward_ref,future,mx,my,a.forward_mvs[pos],a.backward_mvs[pos],a.mv_mode);
            const double lambda=header_lambda;
            const MbTransformDecision local=neutral_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false,nullptr);
            local_rd+=static_cast<long double>(local.distortion)+lambda*static_cast<long double>(local.estimated_bits);
            for (int ti=0;ti<4;++ti) {
                const TransformType t=static_cast<TransformType>(ti+1);
                MbTransformDecision d=neutral_eval.evaluate_transform_type(f,pred,mx,my,t,coding_set,use_vlc,true,c_.pqindex,false);
                d.signal_level=TransformSignalLevel::Frame;
                neutral_eval.refresh_transform_estimate(d,coding_set,use_vlc,false);
                frame_rd[static_cast<size_t>(ti)]+=static_cast<long double>(d.distortion)+
                    lambda*static_cast<long double>(d.estimated_bits);
            }
            ++sampled;
        }
        if (!sampled) return plan;
        const long double scale=static_cast<long double>(inter_total)/static_cast<long double>(sampled);
        local_rd=local_rd*scale + header_lambda;
        long double best_rd=frame_rd[0]*scale + header_lambda*3.0L;
        for (int ti=1;ti<4;++ti) {
            const long double r=frame_rd[static_cast<size_t>(ti)]*scale + header_lambda*3.0L;
            if (r<best_rd) best_rd=r;
        }
        if (best_rd < local_rd*0.9999L) {
            plan.exact_checked=true;
            // 0.1.72 exact TTFRM confirmation.  The 32-MB sample is only a
            // shortlist: before committing a B picture to one transform type,
            // evaluate the local law and all four frame-level laws over every
            // macroblock.  This prevents a small or spatially unrepresentative
            // sample from making an entire B picture look artificially uniform.
            long double exact_local=header_lambda; // TTMBF=0
            std::array<long double,4> exact_frame{};
            for (auto& r:exact_frame) r=header_lambda*3.0L; // TTMBF=1 + TTFRM(2)
            for (int linear=0;linear<total;++linear) {
                const int my=linear/mbw,mx=linear%mbw;
                const size_t pos=static_cast<size_t>(linear);
                if (a.modes[pos]==BMbMode::Intra) continue;
                if (a.modes[pos]==BMbMode::Forward)
                    motion_compensate_mb(pred,forward_ref,mx,my,a.forward_mvs[pos],a.mv_mode);
                else if (a.modes[pos]==BMbMode::Backward)
                    motion_compensate_mb(pred,future,mx,my,a.backward_mvs[pos],a.mv_mode);
                else
                    motion_compensate_bi_mb(pred,forward_ref,future,mx,my,a.forward_mvs[pos],a.backward_mvs[pos],a.mv_mode);
                const MbTransformDecision local=neutral_eval.choose_transform_mb(
                    f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false,nullptr);
                exact_local+=static_cast<long double>(local.distortion)+
                    header_lambda*static_cast<long double>(local.estimated_bits);
                for (int ti=0;ti<4;++ti) {
                    const TransformType t=static_cast<TransformType>(ti+1);
                    MbTransformDecision d=neutral_eval.evaluate_transform_type(
                        f,pred,mx,my,t,coding_set,use_vlc,true,c_.pqindex,false);
                    d.signal_level=TransformSignalLevel::Frame;
                    neutral_eval.refresh_transform_estimate(d,coding_set,use_vlc,false);
                    exact_frame[static_cast<size_t>(ti)]+=static_cast<long double>(d.distortion)+
                        header_lambda*static_cast<long double>(d.estimated_bits);
                }
            }
            int exact_best=0;
            long double exact_best_rd=exact_frame[0];
            for (int ti=1;ti<4;++ti) {
                if (exact_frame[static_cast<size_t>(ti)]<exact_best_rd) {
                    exact_best_rd=exact_frame[static_cast<size_t>(ti)];
                    exact_best=ti;
                }
            }
            if (exact_best_rd < exact_local*0.9999L) {
                plan.frame_level=true;
                plan.frame_type=static_cast<TransformType>(exact_best+1);
            }
        }
        return plan;
    }

TransformRateEstimate Vc1Encoder::estimate_p_transform_rate(const Frame& f,const Frame& ref,const PAnalysis& a) const {
        TransformRateEstimate out{};
        if (!c_.variable_transforms) return out;
        const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
        const int total=mbw*mbh;
        if (total<=0) return out;
        for (uint8_t v:a.use_intra) if (!v) ++out.total_inter_macroblocks;
        if (!out.total_inter_macroblocks) return out;

        // The ABR transform model must predict the syntax-law delta, not run AQ
        // a second time.  Picture-level TTFRM selection is deliberately AQ-neutral
        // in the real encoder, so use the same neutral law here and let the final
        // encode apply AQ/DQUANT after the global signaling choice is known.
        EncoderConfig legacy_cfg=c_;
        legacy_cfg.extended_transform_signaling=false;
        legacy_cfg.adaptive_quality=false;
        legacy_cfg.aq_strength=0.0;
        EncoderConfig extended_cfg=legacy_cfg;
        extended_cfg.extended_transform_signaling=true;
        EncoderConfig aq_cfg=c_;
        aq_cfg.extended_transform_signaling=true;
        Vc1Encoder legacy_eval(legacy_cfg),extended_eval(extended_cfg),aq_eval(aq_cfg);

        const int decision_index=(c_.pqindex<=8)?1:0;
        const int coding_set=chroma_coding_set(decision_index,c_.pqindex);
        const bool use_vlc=c_.ac_mode!=AcMode::Esc3;
        Frame pred=empty_frame();
        const TransformPicturePlan plan=extended_eval.choose_p_transform_plan(f,ref,a,coding_set,use_vlc,&pred);
        out.frame_level=plan.frame_level;
        out.frame_type=plan.frame_type;
        const Frame& prediction_ref=(a.intensity.enabled && a.compensated_reference) ? *a.compensated_reference : ref;
        const int wanted=std::min<int>(32,static_cast<int>(out.total_inter_macroblocks));
        const int stride=std::max(1,total/std::max(1,wanted));
        long double legacy=0.0L,extended=0.0L,aq_extended=0.0L;
        for (int linear=0;linear<total && out.sampled_inter_macroblocks<static_cast<size_t>(wanted);linear+=stride) {
            const size_t pos=static_cast<size_t>(linear);
            if (a.use_intra[pos]) continue;
            const int my=linear/mbw,mx=linear%mbw;
            if (a.use_4mv[pos]) motion_compensate_4mv(pred,prediction_ref,mx,my,a.block_mvs[pos]);
            else motion_compensate_mb(pred,prediction_ref,mx,my,a.mvs[pos],a.mv_mode);
            const auto old_tx=legacy_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false,nullptr);
            const auto new_tx=extended_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false,&plan);
            const auto aq_tx=aq_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false,&plan);
            legacy+=static_cast<long double>(old_tx.estimated_bits);
            extended+=static_cast<long double>(new_tx.estimated_bits);
            aq_extended+=static_cast<long double>(aq_tx.estimated_bits);
            ++out.sampled_inter_macroblocks;
        }
        if (!out.sampled_inter_macroblocks) return out;
        const long double scale=static_cast<long double>(out.total_inter_macroblocks)/
                                static_cast<long double>(out.sampled_inter_macroblocks);
        // Both local signaling laws pay TTMBF.  Frame-level syntax instead pays
        // TTMBF=1 plus the two-bit TTFRM value in the picture header.
        out.legacy_transform_bits=static_cast<double>(legacy*scale+1.0L);
        out.extended_transform_bits=static_cast<double>(extended*scale+(plan.frame_level?3.0L:1.0L));
        out.aq_extended_transform_bits=static_cast<double>(aq_extended*scale+(plan.frame_level?3.0L:1.0L));
        return out;
    }

TransformRateEstimate Vc1Encoder::estimate_b_transform_rate(const Frame& f,const Frame& past,const Frame& future,
                                                     const BAnalysis& a) const {
        TransformRateEstimate out{};
        if (!c_.variable_transforms) return out;
        const int mbw=(c_.width+15)/16,mbh=(c_.height+15)/16;
        const int total=mbw*mbh;
        if (total<=0) return out;
        out.total_inter_macroblocks=static_cast<size_t>(total)-std::min<size_t>(a.intra_macroblocks,static_cast<size_t>(total));
        if (!out.total_inter_macroblocks) return out;

        EncoderConfig legacy_cfg=c_;
        legacy_cfg.extended_transform_signaling=false;
        legacy_cfg.adaptive_quality=false;
        legacy_cfg.aq_strength=0.0;
        EncoderConfig extended_cfg=legacy_cfg;
        extended_cfg.extended_transform_signaling=true;
        EncoderConfig aq_cfg=c_;
        aq_cfg.extended_transform_signaling=true;
        Vc1Encoder legacy_eval(legacy_cfg),extended_eval(extended_cfg),aq_eval(aq_cfg);

        const int decision_index=(c_.pqindex<=8)?1:0;
        const int coding_set=chroma_coding_set(decision_index,c_.pqindex);
        const bool use_vlc=c_.ac_mode!=AcMode::Esc3;
        Frame pred=empty_frame();
        const TransformPicturePlan plan=extended_eval.choose_b_transform_plan(f,past,future,a,coding_set,use_vlc,&pred);
        out.frame_level=plan.frame_level;
        out.frame_type=plan.frame_type;
        const Frame& forward_ref=(a.forward_intensity.enabled && a.compensated_past) ? *a.compensated_past : past;
        const int wanted=std::min<int>(32,static_cast<int>(out.total_inter_macroblocks));
        const int stride=std::max(1,total/std::max(1,wanted));
        long double legacy=0.0L,extended=0.0L,aq_extended=0.0L;
        for (int linear=0;linear<total && out.sampled_inter_macroblocks<static_cast<size_t>(wanted);++linear) {
            const int my=linear/mbw,mx=linear%mbw;
            const size_t pos=static_cast<size_t>(linear);
            if (a.modes[pos]==BMbMode::Intra) continue;
            if (stride>1 && (linear%stride)!=0 &&
                out.sampled_inter_macroblocks+static_cast<size_t>((total-linear-1)/stride)>=static_cast<size_t>(wanted)) continue;
            if (a.modes[pos]==BMbMode::Forward)
                motion_compensate_mb(pred,forward_ref,mx,my,a.forward_mvs[pos],a.mv_mode);
            else if (a.modes[pos]==BMbMode::Backward)
                motion_compensate_mb(pred,future,mx,my,a.backward_mvs[pos],a.mv_mode);
            else
                motion_compensate_bi_mb(pred,forward_ref,future,mx,my,a.forward_mvs[pos],a.backward_mvs[pos],a.mv_mode);
            const auto old_tx=legacy_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false,nullptr);
            const auto new_tx=extended_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false,&plan);
            const auto aq_tx=aq_eval.choose_transform_mb(f,pred,mx,my,coding_set,use_vlc,c_.pqindex,false,&plan);
            legacy+=static_cast<long double>(old_tx.estimated_bits);
            extended+=static_cast<long double>(new_tx.estimated_bits);
            aq_extended+=static_cast<long double>(aq_tx.estimated_bits);
            ++out.sampled_inter_macroblocks;
        }
        if (!out.sampled_inter_macroblocks) return out;
        const long double scale=static_cast<long double>(out.total_inter_macroblocks)/
                                static_cast<long double>(out.sampled_inter_macroblocks);
        out.legacy_transform_bits=static_cast<double>(legacy*scale+1.0L);
        out.extended_transform_bits=static_cast<double>(extended*scale+(plan.frame_level?3.0L:1.0L));
        out.aq_extended_transform_bits=static_cast<double>(aq_extended*scale+(plan.frame_level?3.0L:1.0L));
        return out;
    }

QuantizerRateEstimate Vc1Encoder::estimate_picture_quantizer_rate(const Frame& f) const {
        QuantizerRateEstimate out{};
        if (f.width<=0 || f.height<=0 || f.y.empty()) {
            out.selected=c_.quantizer_type==QuantizerType::NonUniform
                ? QuantizerType::NonUniform : QuantizerType::Uniform;
            return out;
        }
        const int bw=(f.width+7)/8, bh=(f.height+7)/8;
        const int total=std::max(1,bw*bh);
        const int stride=std::max(1,total/32);
        double cost_u=0.0,cost_n=0.0;
        uint64_t bits_u=0,bits_n=0;
        int sampled=0;
        auto qlevel=[&](double coeff,bool nonuniform) {
            if (coeff==0.0) return 0;
            const int sg=coeff<0?-1:1;
            const double a=std::abs(coeff);
            const double dq=static_cast<double>(2*c_.pqindex+(c_.halfqp?1:0));
            const double off=nonuniform?static_cast<double>(c_.pqindex):0.0;
            int center=std::clamp(static_cast<int>(std::llround((a-off)/std::max(1.0,dq))),1,max_quantized_level());
            int best=0; double be=a*a;
            for (int n=std::max(1,center-1);n<=std::min(max_quantized_level(),center+1);++n) {
                const double r=n*dq+off,e=a-r;
                if (e*e<be) {be=e*e;best=n;}
            }
            return sg*best;
        };
        auto block_cost=[&](const std::array<double,64>& coeff,bool nonuniform) {
            std::array<int,64> q{};
            for (int i=1;i<64;++i) q[static_cast<size_t>(i)]=qlevel(coeff[static_cast<size_t>(i)],nonuniform);
            int last=-1;
            for (int sp=63;sp>=1;--sp) if (q[kIntraScan[static_cast<size_t>(sp)]]) {last=sp;break;}
            uint64_t bits=0; int prev=0;
            if (last>=1) for (int sp=1;sp<=last;++sp) {
                const int idx=kIntraScan[static_cast<size_t>(sp)],lev=q[static_cast<size_t>(idx)];
                if (!lev) continue;
                bits+=static_cast<uint64_t>(trellis_rate_bits(sp-prev-1,lev,sp==last,true)); prev=sp;
            }
            double dist=0.0;
            const double dq=static_cast<double>(2*c_.pqindex+(c_.halfqp?1:0));
            const double off=nonuniform?static_cast<double>(c_.pqindex):0.0;
            for (int i=1;i<64;++i) {
                const int lev=q[static_cast<size_t>(i)];
                double r=0.0;if(lev)r=static_cast<double>(lev)*dq+(lev>0?off:-off);
                const double e=coeff[static_cast<size_t>(i)]-r;
                dist+=e*e*trellis_coeff_weight(TransformType::T8x8,i);
            }
            const double lambda=0.15*dq*dq;
            return std::pair<double,uint64_t>{dist+lambda*static_cast<double>(bits),bits};
        };
        for (int linear=0;linear<total && sampled<32;linear+=stride) {
            const int by=linear/bw,bx=linear%bw;
            const auto coeff=forward_transform(f.y,f.width,f.height,bx*8,by*8,0);
            const auto u=block_cost(coeff,false), n=block_cost(coeff,true);
            cost_u+=u.first; cost_n+=n.first; bits_u+=u.second; bits_n+=n.second; ++sampled;
        }
        out.sampled_blocks=static_cast<size_t>(sampled);
        out.uniform_bits=static_cast<double>(bits_u);
        out.nonuniform_bits=static_cast<double>(bits_n);
        if (c_.quantizer_type==QuantizerType::Auto)
            out.selected=cost_n < cost_u*0.995 ? QuantizerType::NonUniform : QuantizerType::Uniform;
        else
            out.selected=c_.quantizer_type;

        // 0.1.72 ABR coding-law model: the size predictor was historically
        // trained on uniform PQUANT.  When AUTO/forced nonuniform is selected,
        // scale only the transform-coefficient portion using the same bounded
        // sample used by the quantizer RDO.  Laplace smoothing keeps all-zero
        // samples neutral rather than producing an unstable ratio.
        if (out.selected==QuantizerType::NonUniform && sampled>0) {
            const double smooth=static_cast<double>(sampled);
            out.rate_scale=std::clamp((out.nonuniform_bits+smooth)/(out.uniform_bits+smooth),0.85,1.15);
        }
        return out;
    }

QuantizerType Vc1Encoder::choose_picture_quantizer_type(const Frame& f) const {
        return estimate_picture_quantizer_rate(f).selected;
    }

void Vc1Encoder::inverse_transform_8x8(std::array<int,64>& block) const {
#if defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::X86V4) { simd::inverse_transform_8x8_x86_64_v4(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::X86V1) { simd::inverse_transform_8x8_x86_64_v1(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::Prescott) { simd::inverse_transform_8x8_prescott(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_K10)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::K10) { simd::inverse_transform_8x8_k10(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_CONROE)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::Conroe) { simd::inverse_transform_8x8_conroe(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::X86V2) { simd::inverse_transform_8x8_x86_64_v2(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::Penryn) { simd::inverse_transform_8x8_penryn(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::SandyBridge) { simd::inverse_transform_8x8_sandybridge(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::Bulldozer) { simd::inverse_transform_8x8_bulldozer(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::Piledriver) { simd::inverse_transform_8x8_piledriver(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::Avx2Partial) { simd::inverse_transform_8x8_avx2_partial(block.data()); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        if (c_.simd_for(SimdPrimitive::Inverse8x8)==SimdTier::X86V3) { simd::inverse_transform_8x8_x86_64_v3(block.data()); return; }
#endif
        // Bit-exact scalar form of VC-1's normative 8x8 inverse transform,
        // matching FFmpeg's vc1_inv_trans_8x8_c rounding behavior.
        std::array<int,64> temp{};
        for (int col=0;col<8;++col) {
            const int* src=block.data()+col;
            int* dst=temp.data()+col*8;
            int t1=12*(src[0]+src[32])+4;
            int t2=12*(src[0]-src[32])+4;
            int t3=16*src[16]+6*src[48];
            int t4=6*src[16]-16*src[48];
            int t5=t1+t3, t6=t2+t4, t7=t2-t4, t8=t1-t3;
            t1=16*src[8]+15*src[24]+9*src[40]+4*src[56];
            t2=15*src[8]-4*src[24]-16*src[40]-9*src[56];
            t3=9*src[8]-16*src[24]+4*src[40]+15*src[56];
            t4=4*src[8]-9*src[24]+15*src[40]-16*src[56];
            dst[0]=arshift(t5+t1,3);
            dst[1]=arshift(t6+t2,3);
            dst[2]=arshift(t7+t3,3);
            dst[3]=arshift(t8+t4,3);
            dst[4]=arshift(t8-t4,3);
            dst[5]=arshift(t7-t3,3);
            dst[6]=arshift(t6-t2,3);
            dst[7]=arshift(t5-t1,3);
        }
        for (int col=0;col<8;++col) {
            const int* src=temp.data()+col;
            int t1=12*(src[0]+src[32])+64;
            int t2=12*(src[0]-src[32])+64;
            int t3=16*src[16]+6*src[48];
            int t4=6*src[16]-16*src[48];
            int t5=t1+t3, t6=t2+t4, t7=t2-t4, t8=t1-t3;
            t1=16*src[8]+15*src[24]+9*src[40]+4*src[56];
            t2=15*src[8]-4*src[24]-16*src[40]-9*src[56];
            t3=9*src[8]-16*src[24]+4*src[40]+15*src[56];
            t4=4*src[8]-9*src[24]+15*src[40]-16*src[56];
            block[static_cast<size_t>(0)*8+col]=arshift(t5+t1,7);
            block[static_cast<size_t>(1)*8+col]=arshift(t6+t2,7);
            block[static_cast<size_t>(2)*8+col]=arshift(t7+t3,7);
            block[static_cast<size_t>(3)*8+col]=arshift(t8+t4,7);
            block[static_cast<size_t>(4)*8+col]=arshift(t8-t4+1,7);
            block[static_cast<size_t>(5)*8+col]=arshift(t7-t3+1,7);
            block[static_cast<size_t>(6)*8+col]=arshift(t6-t2+1,7);
            block[static_cast<size_t>(7)*8+col]=arshift(t5-t1+1,7);
        }
    }

void Vc1Encoder::put_block(std::vector<uint8_t>& dst,int w,int h,int x0,int y0,
                   const std::array<int,64>& block,int bias) const {
#if defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::X86V4 && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_x86_64_v4(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::X86V1 && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_x86_64_v1(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::Prescott && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_prescott(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_K10)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::K10 && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_k10(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_CONROE)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::Conroe && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_conroe(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::X86V2 && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_x86_64_v2(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::Penryn && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_penryn(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::SandyBridge && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_sandybridge(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::Bulldozer && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_bulldozer(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::Piledriver && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_piledriver(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::Avx2Partial && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_avx2_partial(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        if (c_.simd_for(SimdPrimitive::PutBlock8)==SimdTier::X86V3 && x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) { simd::put_block8_x86_64_v3(dst.data()+static_cast<size_t>(y0)*w+x0,w,block.data(),bias); return; }
#endif
        for (int y=0;y<8 && y0+y<h;++y)
            for (int x=0;x<8 && x0+x<w;++x)
                dst[static_cast<size_t>(y0+y)*w+x0+x]=static_cast<uint8_t>(
                    std::clamp(bias+block[static_cast<size_t>(y)*8+x],0,255));
    }

void Vc1Encoder::add_block(std::vector<uint8_t>& dst,int w,int h,int x0,int y0,
                          const std::array<int,64>& block) {
        for (int y=0;y<8 && y0+y<h;++y) {
            for (int x=0;x<8 && x0+x<w;++x) {
                const size_t off=static_cast<size_t>(y0+y)*w+x0+x;
                dst[off]=static_cast<uint8_t>(std::clamp(static_cast<int>(dst[off])+
                    block[static_cast<size_t>(y)*8+x],0,255));
            }
        }
    }

std::array<double,64> Vc1Encoder::forward_transform(const std::vector<uint8_t>& p,
                                             int w, int h, int x0, int y0, int pad_sample) const {
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) {
            std::array<double,64> c{};
#if defined(LIBVC1_HAVE_X86_64_V4)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::X86V4) {
                simd::forward_transform_intra8_x86_64_v4(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::X86V1) {
                simd::forward_transform_intra8_x86_64_v1(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::Prescott) {
                simd::forward_transform_intra8_prescott(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_K10)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::K10) {
                simd::forward_transform_intra8_k10(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_CONROE)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::Conroe) {
                simd::forward_transform_intra8_conroe(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::X86V2) {
                simd::forward_transform_intra8_x86_64_v2(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::Penryn) {
                simd::forward_transform_intra8_penryn(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::SandyBridge) {
                simd::forward_transform_intra8_sandybridge(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::Bulldozer) {
                simd::forward_transform_intra8_bulldozer(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::Piledriver) {
                simd::forward_transform_intra8_piledriver(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::Avx2Partial) {
                simd::forward_transform_intra8_avx2_partial(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
            if (c_.simd_for(SimdPrimitive::ForwardIntra8)==SimdTier::X86V3) {
                simd::forward_transform_intra8_x86_64_v3(
                    p.data()+static_cast<size_t>(y0)*w+x0,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardIntra8));
                return c;
            }
#endif
        }
#endif
        double r[8][8]{};
        double tmp[8][8]{};
        double xform[8][8]{};
        for (int y=0; y<8; ++y) {
            const int sy=y0+y;
            for (int x=0; x<8; ++x) {
                const int sx=x0+x;
                const int sample=(sx>=0 && sy>=0 && sx<w && sy<h)
                    ? static_cast<int>(p[static_cast<size_t>(sy)*w+sx])
                    : std::clamp(pad_sample,0,255);
                r[y][x]=sample-128;
            }
        }
        for (int a=0; a<8; ++a)
            for (int x=0; x<8; ++x)
                for (int y=0; y<8; ++y)
                    tmp[a][x] += forward_transform::k8[a][y] * r[y][x];
        for (int a=0; a<8; ++a)
            for (int bb=0; bb<8; ++bb)
                for (int x=0; x<8; ++x)
                    xform[a][bb] += tmp[a][x] * forward_transform::k8[bb][x];

        std::array<double,64> c{};
        for (int row=0; row<8; ++row)
            for (int col=0; col<8; ++col)
                c[static_cast<size_t>(row)*8+col] = 1024.0 * xform[col][row];
        return c;
    }

std::array<double,64> Vc1Encoder::forward_transform_residual(const std::vector<uint8_t>& src,
                                                      const std::vector<uint8_t>& pred,
                                                      int w,int h,int x0,int y0) const {
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (x0>=0 && y0>=0 && x0+8<=w && y0+8<=h) {
            std::array<double,64> c{};
            const size_t off=static_cast<size_t>(y0)*w+x0;
#if defined(LIBVC1_HAVE_X86_64_V4)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::X86V4) {
                simd::forward_transform_residual8_x86_64_v4(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::X86V1) {
                simd::forward_transform_residual8_x86_64_v1(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::Prescott) {
                simd::forward_transform_residual8_prescott(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_K10)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::K10) {
                simd::forward_transform_residual8_k10(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_CONROE)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::Conroe) {
                simd::forward_transform_residual8_conroe(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::X86V2) {
                simd::forward_transform_residual8_x86_64_v2(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::Penryn) {
                simd::forward_transform_residual8_penryn(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::SandyBridge) {
                simd::forward_transform_residual8_sandybridge(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::Bulldozer) {
                simd::forward_transform_residual8_bulldozer(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::Piledriver) {
                simd::forward_transform_residual8_piledriver(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::Avx2Partial) {
                simd::forward_transform_residual8_avx2_partial(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
            if (c_.simd_for(SimdPrimitive::ForwardResidual8)==SimdTier::X86V3) {
                simd::forward_transform_residual8_x86_64_v3(src.data()+off,pred.data()+off,w,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidual8));
                return c;
            }
#endif
        }
#endif
        double r[8][8]{};
        double tmp[8][8]{};
        double xform[8][8]{};
        for (int y=0;y<8;++y) {
            const int sy=y0+y;
            for (int x=0;x<8;++x) {
                const int sx=x0+x;
                // There is no source delta outside the visible picture.  Do not
                // replicate the last visible row/column into the hidden coded
                // padding: doing so multiplies edge residuals and biases both
                // coefficient-rate estimates and inter/intra RDO.  A zero delta
                // leaves hidden padding predicted while preserving decoder parity.
                if (sx<0 || sy<0 || sx>=w || sy>=h) { r[y][x]=0.0; continue; }
                const size_t off=static_cast<size_t>(sy)*w+sx;
                r[y][x]=static_cast<int>(src[off])-static_cast<int>(pred[off]);
            }
        }
        for (int a=0;a<8;++a)
            for (int x=0;x<8;++x)
                for (int y=0;y<8;++y)
                    tmp[a][x] += forward_transform::k8[a][y]*r[y][x];
        for (int a=0;a<8;++a)
            for (int bb=0;bb<8;++bb)
                for (int x=0;x<8;++x)
                    xform[a][bb] += tmp[a][x]*forward_transform::k8[bb][x];
        std::array<double,64> c{};
        for (int row=0;row<8;++row)
            for (int col=0;col<8;++col)
                c[static_cast<size_t>(row)*8+col]=1024.0*xform[col][row];
        return c;
    }

int Vc1Encoder::transform_part_count(TransformType t) {
        switch (t) {
            case TransformType::T8x8: return 1;
            case TransformType::T8x4: return 2;
            case TransformType::T4x8: return 2;
            case TransformType::T4x4: return 4;
        }
        return 1;
    }

void Vc1Encoder::transform_part_geometry(TransformType t,int part,int& ox,int& oy,int& bw,int& bh) {
        ox=oy=0; bw=bh=8;
        switch (t) {
            case TransformType::T8x8: return;
            case TransformType::T8x4: bw=8; bh=4; oy=part*4; return;
            case TransformType::T4x8: bw=4; bh=8; ox=part*4; return;
            case TransformType::T4x4: bw=4; bh=4; ox=(part&1)*4; oy=(part>>1)*4; return;
        }
    }

std::array<double,64> Vc1Encoder::forward_transform_residual_part(const std::vector<uint8_t>& src,
                                                           const std::vector<uint8_t>& pred,
                                                           int w,int h,int x0,int y0,
                                                           int bw,int bh) const {
        if (bw==8 && bh==8) return forward_transform_residual(src,pred,w,h,x0,y0);
#if defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::X86V4 && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_x86_64_v4(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::X86V1 && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_x86_64_v1(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::Prescott && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_prescott(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_K10)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::K10 && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_k10(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_CONROE)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::Conroe && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_conroe(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::X86V2 && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_x86_64_v2(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::Penryn && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_penryn(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::SandyBridge && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_sandybridge(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::Bulldozer && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_bulldozer(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::Piledriver && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_piledriver(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::Avx2Partial && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_avx2_partial(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        if (c_.simd_for(SimdPrimitive::ForwardResidualRect)==SimdTier::X86V3 && x0>=0 && y0>=0 && x0+bw<=w && y0+bh<=h && (bw==4 || bw==8) && (bh==4 || bh==8)) {
            std::array<double,64> c{}; const size_t off=static_cast<size_t>(y0)*w+x0;
            simd::forward_transform_residual_rect_x86_64_v3(src.data()+off,pred.data()+off,w,bw,bh,c.data(),c_.simd_fma_for(SimdPrimitive::ForwardResidualRect)); return c;
        }
#endif
        double r[8][8]{};
        double tmp[8][8]{};
        double xform[8][8]{};
        for (int y=0;y<bh;++y) {
            const int sy=y0+y;
            for (int x=0;x<bw;++x) {
                const int sx=x0+x;
                if (sx<0 || sy<0 || sx>=w || sy>=h) { r[y][x]=0.0; continue; }
                const size_t off=static_cast<size_t>(sy)*w+sx;
                r[y][x]=static_cast<int>(src[off])-static_cast<int>(pred[off]);
            }
        }
        for (int a=0;a<bh;++a)
            for (int x=0;x<bw;++x)
                for (int y=0;y<bh;++y)
                    tmp[a][x] += (bh==8?forward_transform::k8[a][y]:forward_transform::k4[a][y])*r[y][x];
        for (int a=0;a<bh;++a)
            for (int bb=0;bb<bw;++bb)
                for (int x=0;x<bw;++x)
                    xform[a][bb] += tmp[a][x]*(bw==8?forward_transform::k8[bb][x]:forward_transform::k4[bb][x]);
        std::array<double,64> c{};
        // Rectangular VC-1 transforms use normal row-major coefficient storage.
        for (int row=0;row<bh;++row)
            for (int col=0;col<bw;++col)
                c[static_cast<size_t>(row)*8+col]=1024.0*xform[row][col];
        return c;
    }

bool Vc1Encoder::uniform_quantizer() const {
        // AUTO is resolved before picture coding. Keep uniform as the safe tie/default.
        return c_.quantizer_type != QuantizerType::NonUniform;
    }

int Vc1Encoder::picture_double_quant(int mquant,bool dquant_derived) const {
        if (mquant<1) mquant=c_.pqindex;
        const int half=(!dquant_derived && mquant==c_.pqindex && c_.halfqp)?1:0;
        return 2*mquant+half;
    }

int Vc1Encoder::dequant_level(int level,int mquant,bool dquant_derived) const {
        if (!level) return 0;
        if (mquant<1) mquant=c_.pqindex;
        long v=static_cast<long>(level)*picture_double_quant(mquant,dquant_derived);
        if (!uniform_quantizer()) v += level>0?mquant:-mquant;
        return static_cast<int>(std::clamp(v,static_cast<long>(std::numeric_limits<int>::min()),
                                            static_cast<long>(std::numeric_limits<int>::max())));
    }

int Vc1Encoder::scalar_quantize_level(double coeff,int mquant,bool dquant_derived) const {
        if (mquant<1) mquant=c_.pqindex;
        if (coeff==0.0) return 0;
        const int sign=coeff<0.0?-1:1;
        const double a=std::abs(coeff);
        const double dq=static_cast<double>(picture_double_quant(mquant,dquant_derived));
        const double offset=uniform_quantizer()?0.0:static_cast<double>(mquant);
        const int max_level=max_quantized_level();
        int center=static_cast<int>(std::llround((a-offset)/std::max(1.0,dq)));
        center=std::clamp(center,1,max_level);
        int best=0;
        double besterr=a*a;
        for (int n=std::max(1,center-1);n<=std::min(max_level,center+1);++n) {
            const double recon=static_cast<double>(n)*dq+offset;
            const double e=a-recon;
            if (e*e<besterr) { besterr=e*e; best=n; }
        }
        return sign*best;
    }

std::array<int,64> Vc1Encoder::quantize_inter_part(const std::vector<uint8_t>& src,
                                           const std::vector<uint8_t>& pred,
                                           int w,int h,int x0,int y0,int bw,int bh,
                                           bool allow_trellis,int mquant,
                                           const std::vector<uint8_t>* luma_context) const {
        if (mquant < 1) mquant=c_.pqindex;
        // A transform partition wholly outside the visible crop has no source
        // samples to correct.  Code no delta for it instead of manufacturing a
        // residual by extending the last visible row/column.
        if (x0>=w || y0>=h || x0+bw<=0 || y0+bh<=0) return {};
        const auto coeff=forward_transform_residual_part(src,pred,w,h,x0,y0,bw,bh);
        std::array<int,64> q{};
        if (allow_trellis && c_.trellis>0) {
            TransformType type=TransformType::T8x8;
            if (bw==8 && bh==4) type=TransformType::T8x4;
            else if (bw==4 && bh==8) type=TransformType::T4x8;
            else if (bw==4 && bh==4) type=TransformType::T4x4;
            const uint8_t* scan=nullptr; int n=0;
            if (type==TransformType::T8x8) { scan=kInterScan.data(); n=64; }
            else if (type==TransformType::T8x4) { scan=(c_.syntax==StreamSyntax::Wmv9Main?kMainInterScan8x4.data():kInterScan8x4.data()); n=32; }
            else if (type==TransformType::T4x8) { scan=(c_.syntax==StreamSyntax::Wmv9Main?kMainInterScan4x8.data():kInterScan4x8.data()); n=32; }
            else { scan=kInterScan4x4.data(); n=16; }
            const double lscale=perceptual_lambda_scale(src,w,h,x0,y0,bw,bh,&pred,nullptr,luma_context);
            q=trellis_quantize(coeff,scan,n,0,type,false,lscale,mquant);
        } else {
#if defined(LIBVC1_HAVE_X86_64_V4) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_SANDYBRIDGE) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_PENRYN) || defined(LIBVC1_HAVE_CONROE) || defined(LIBVC1_HAVE_PRESCOTT) || defined(LIBVC1_HAVE_X86_64_V1)
            const SimdTier qt=c_.simd_for(SimdPrimitive::Quantize);
            if (qt==SimdTier::X86V4 || qt==SimdTier::X86V3 || qt==SimdTier::Avx2Partial || qt==SimdTier::Piledriver || qt==SimdTier::Bulldozer || qt==SimdTier::SandyBridge || qt==SimdTier::X86V2 || qt==SimdTier::Penryn || qt==SimdTier::Conroe || qt==SimdTier::K10 || qt==SimdTier::Prescott || qt==SimdTier::X86V1) {
                const bool derived=mquant!=c_.pqindex;
                const double qs=static_cast<double>(picture_double_quant(mquant,derived));
                const double off=uniform_quantizer()?0.0:static_cast<double>(mquant);
                auto qrow=[&](const double* in,int* out,int count) {
#if defined(LIBVC1_HAVE_X86_64_V4)
                    if (qt==SimdTier::X86V4) { simd::quantize_coefficients_x86_64_v4(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
                    if (qt==SimdTier::Avx2Partial) { simd::quantize_coefficients_avx2_partial(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
                    if (qt==SimdTier::X86V3) { simd::quantize_coefficients_x86_64_v3(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
                    if (qt==SimdTier::X86V1) { simd::quantize_coefficients_x86_64_v1(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
                    if (qt==SimdTier::Prescott) { simd::quantize_coefficients_prescott(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_K10)
                    if (qt==SimdTier::K10) { simd::quantize_coefficients_k10(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_CONROE)
                    if (qt==SimdTier::Conroe) { simd::quantize_coefficients_conroe(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
                    if (qt==SimdTier::X86V2) { simd::quantize_coefficients_x86_64_v2(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
                    if (qt==SimdTier::Penryn) { simd::quantize_coefficients_penryn(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
                    if (qt==SimdTier::SandyBridge) { simd::quantize_coefficients_sandybridge(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
                    if (qt==SimdTier::Bulldozer) { simd::quantize_coefficients_bulldozer(in,out,count,qs,off,max_quantized_level()); return; }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
                    if (qt==SimdTier::Piledriver) { simd::quantize_coefficients_piledriver(in,out,count,qs,off,max_quantized_level()); return; }
#endif
                };
                if (bw==8 && bh==8) qrow(coeff.data(),q.data(),64);
                else for (int y=0;y<bh;++y) qrow(coeff.data()+static_cast<size_t>(y)*8,q.data()+static_cast<size_t>(y)*8,bw);
            } else
#endif
            {
                for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) {
                    const size_t i=static_cast<size_t>(y)*8+x;
                    q[i]=scalar_quantize_level(coeff[i],mquant,mquant!=c_.pqindex);
                }
            }
        }
        return q;
    }

std::array<int,64> Vc1Encoder::inverse_partition(TransformType t,const std::array<int,64>& q,int mquant,bool dquant_derived) const {
        std::array<int,64> a{};
        const int partsz=(t==TransformType::T8x8)?64:(t==TransformType::T4x4?16:32);
        (void)partsz;
        int bw=8,bh=8,ox=0,oy=0;
        transform_part_geometry(t,0,ox,oy,bw,bh);
        if (mquant<1) mquant=c_.pqindex;
        int nonzero=0;
        for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) if (q[static_cast<size_t>(y)*8+x]) ++nonzero;
        const SimdTier dqt=c_.simd_for(SimdPrimitive::Quantize);
        const int dq=picture_double_quant(mquant,dquant_derived);
        const bool nonuniform=!uniform_quantizer();
        bool dequant_simd=false;
        if (bw==8) dequant_simd=simd_dequant_dispatch(dqt,q.data(),a.data(),bh*8,dq,mquant,nonuniform);
        else {
            dequant_simd=dqt!=SimdTier::None;
            for (int y=0;y<bh && dequant_simd;++y)
                dequant_simd=simd_dequant_dispatch(dqt,q.data()+static_cast<size_t>(y)*8,a.data()+static_cast<size_t>(y)*8,bw,dq,mquant,nonuniform);
        }
        if (!dequant_simd) for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) {
            const size_t i=static_cast<size_t>(y)*8+x;
            a[i]=dequant_level(q[i],mquant,dquant_derived);
        }
        // FFmpeg (and optimized hardware decoders) use the normative DC-only
        // shortcuts when a transform partition contains coefficient 0 only.
        // Their staged rounding is not always identical to running the full
        // transform with 63/31/15 explicit zero coefficients, so mirror it.
        if (nonzero==1 && q[0]!=0) {
            int dc=a[0];
            if (t==TransformType::T8x8) { dc=arshift(3*dc+1,1); dc=arshift(3*dc+16,5); }
            else if (t==TransformType::T8x4) { dc=arshift(3*dc+1,1); dc=arshift(17*dc+64,7); }
            else if (t==TransformType::T4x8) { dc=arshift(17*dc+4,3); dc=arshift(12*dc+64,7); }
            else { dc=arshift(17*dc+4,3); dc=arshift(17*dc+64,7); }
            a.fill(0);
            for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) a[static_cast<size_t>(y)*8+x]=dc;
            return a;
        }
        if (t==TransformType::T8x8) {
            inverse_transform_8x8(a);
            return a;
        }
        const int rect_type=t==TransformType::T8x4?1:(t==TransformType::T4x8?2:3);
        if (simd_inverse_rect_dispatch(c_.simd_for(SimdPrimitive::Inverse8x8),a.data(),rect_type)) return a;
        std::array<int,64> tmp{};
        if (t==TransformType::T8x4) {
            // Horizontal 8-point, then vertical 4-point, matching vc1_inv_trans_8x4_c.
            for (int y=0;y<4;++y) {
                const int* src=a.data()+y*8;
                int* dst=tmp.data()+y*8;
                int t1=12*(src[0]+src[4])+4, t2=12*(src[0]-src[4])+4;
                int t3=16*src[2]+6*src[6], t4=6*src[2]-16*src[6];
                int t5=t1+t3,t6=t2+t4,t7=t2-t4,t8=t1-t3;
                t1=16*src[1]+15*src[3]+9*src[5]+4*src[7];
                t2=15*src[1]-4*src[3]-16*src[5]-9*src[7];
                t3=9*src[1]-16*src[3]+4*src[5]+15*src[7];
                t4=4*src[1]-9*src[3]+15*src[5]-16*src[7];
                dst[0]=arshift(t5+t1,3); dst[1]=arshift(t6+t2,3);
                dst[2]=arshift(t7+t3,3); dst[3]=arshift(t8+t4,3);
                dst[4]=arshift(t8-t4,3); dst[5]=arshift(t7-t3,3);
                dst[6]=arshift(t6-t2,3); dst[7]=arshift(t5-t1,3);
            }
            for (int x=0;x<8;++x) {
                int t1=17*(tmp[x]+tmp[16+x])+64;
                int t2=17*(tmp[x]-tmp[16+x])+64;
                int t3=22*tmp[8+x]+10*tmp[24+x];
                int t4=22*tmp[24+x]-10*tmp[8+x];
                a[x]=arshift(t1+t3,7); a[8+x]=arshift(t2-t4,7);
                a[16+x]=arshift(t2+t4,7); a[24+x]=arshift(t1-t3,7);
            }
            return a;
        }
        if (t==TransformType::T4x8) {
            for (int y=0;y<8;++y) {
                const int* src=a.data()+y*8;
                int* dst=tmp.data()+y*8;
                int t1=17*(src[0]+src[2])+4, t2=17*(src[0]-src[2])+4;
                int t3=22*src[1]+10*src[3], t4=22*src[3]-10*src[1];
                dst[0]=arshift(t1+t3,3); dst[1]=arshift(t2-t4,3);
                dst[2]=arshift(t2+t4,3); dst[3]=arshift(t1-t3,3);
            }
            for (int x=0;x<4;++x) {
                const int* src=tmp.data()+x;
                int t1=12*(src[0]+src[32])+64, t2=12*(src[0]-src[32])+64;
                int t3=16*src[16]+6*src[48], t4=6*src[16]-16*src[48];
                int t5=t1+t3,t6=t2+t4,t7=t2-t4,t8=t1-t3;
                t1=16*src[8]+15*src[24]+9*src[40]+4*src[56];
                t2=15*src[8]-4*src[24]-16*src[40]-9*src[56];
                t3=9*src[8]-16*src[24]+4*src[40]+15*src[56];
                t4=4*src[8]-9*src[24]+15*src[40]-16*src[56];
                a[x]=arshift(t5+t1,7); a[8+x]=arshift(t6+t2,7);
                a[16+x]=arshift(t7+t3,7); a[24+x]=arshift(t8+t4,7);
                a[32+x]=arshift(t8-t4+1,7); a[40+x]=arshift(t7-t3+1,7);
                a[48+x]=arshift(t6-t2+1,7); a[56+x]=arshift(t5-t1+1,7);
            }
            return a;
        }
        // 4x4.
        for (int y=0;y<4;++y) {
            const int* src=a.data()+y*8;
            int* dst=tmp.data()+y*8;
            int t1=17*(src[0]+src[2])+4, t2=17*(src[0]-src[2])+4;
            int t3=22*src[1]+10*src[3], t4=22*src[3]-10*src[1];
            dst[0]=arshift(t1+t3,3); dst[1]=arshift(t2-t4,3);
            dst[2]=arshift(t2+t4,3); dst[3]=arshift(t1-t3,3);
        }
        for (int x=0;x<4;++x) {
            int t1=17*(tmp[x]+tmp[16+x])+64;
            int t2=17*(tmp[x]-tmp[16+x])+64;
            int t3=22*tmp[8+x]+10*tmp[24+x];
            int t4=22*tmp[24+x]-10*tmp[8+x];
            a[x]=arshift(t1+t3,7); a[8+x]=arshift(t2-t4,7);
            a[16+x]=arshift(t2+t4,7); a[24+x]=arshift(t1-t3,7);
        }
        return a;
    }

void Vc1Encoder::add_partition(std::vector<uint8_t>& dst,int w,int h,int x0,int y0,
                       TransformType t,int part,const std::array<int,64>& q,int mquant,bool dquant_derived) const {
        int ox=0,oy=0,bw=8,bh=8;
        transform_part_geometry(t,part,ox,oy,bw,bh);
        const auto r=inverse_partition(t,q,mquant,dquant_derived);
#if defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::X86V4 && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_x86_64_v4(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::X86V1 && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_x86_64_v1(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::Prescott && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_prescott(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_K10)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::K10 && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_k10(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_CONROE)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::Conroe && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_conroe(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::X86V2 && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_x86_64_v2(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::Penryn && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_penryn(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::SandyBridge && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_sandybridge(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::Bulldozer && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_bulldozer(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::Piledriver && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_piledriver(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::Avx2Partial && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_avx2_partial(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        if (c_.simd_for(SimdPrimitive::AddBlockRect)==SimdTier::X86V3 && x0+ox>=0 && y0+oy>=0 && x0+ox+bw<=w && y0+oy+bh<=h) { simd::add_block_rect_x86_64_v3(dst.data()+static_cast<size_t>(y0+oy)*w+x0+ox,w,r.data(),bw,bh); return; }
#endif
        for (int y=0;y<bh && y0+oy+y<h;++y) for (int x=0;x<bw && x0+ox+x<w;++x) {
            const size_t off=static_cast<size_t>(y0+oy+y)*w+x0+ox+x;
            dst[off]=static_cast<uint8_t>(std::clamp(static_cast<int>(dst[off])+r[static_cast<size_t>(y)*8+x],0,255));
        }
    }

int Vc1Encoder::tt_table_index() const { return c_.pqindex<=4?0:(c_.pqindex<=12?1:2); }

void Vc1Encoder::write_subblkpat(BitWriter& b,uint8_t skipmask) const {
        if (skipmask>=15) throw std::runtime_error("invalid 4x4 subblock skip mask");
        const int symbol=14-static_cast<int>(skipmask);
        const int tab=tt_table_index();
        b.vlc(kSubblkCodes[tab][symbol],kSubblkBits[tab][symbol]);
    }

uint8_t Vc1Encoder::tt_variant(TransformType t,uint8_t skipmask) {
        if (t==TransformType::T8x8) return 0;
        if (t==TransformType::T4x4) return 7;
        if (skipmask>2) throw std::runtime_error("invalid rectangular transform skip mask");
        if (t==TransformType::T8x4) return skipmask==0?1:(skipmask==1?2:3);
        return skipmask==0?4:(skipmask==1?5:6);
    }

int Vc1Encoder::tt_symbol_for_variant(uint8_t variant) const {
        const int tab=tt_table_index();
        for (int symbol=0;symbol<8;++symbol)
            if (kTtSymbolVariant[tab][symbol]==variant) return symbol;
        throw std::runtime_error("VC-1 transform VLC variant has no symbol");
    }

void Vc1Encoder::write_ttmb_macro(BitWriter& b,TransformType t,uint8_t first_skipmask) const {
        int symbol=8;
        if (t==TransformType::T8x4) symbol=first_skipmask==1?10:(first_skipmask==2?9:11);
        else if (t==TransformType::T4x8) symbol=first_skipmask==1?13:(first_skipmask==2?12:14);
        else if (t==TransformType::T4x4) symbol=15;
        const int tab=tt_table_index();
        b.vlc(kTtmbCodes[tab][symbol],kTtmbBits[tab][symbol]);
    }

int Vc1Encoder::ttmb_block_value(TransformType t,uint8_t skipmask) {
        // TTMB block-level values 0..7 use the decoder's TransformTypes
        // numbering directly.  This ordering is *not* the table-dependent
        // TTBLK VLC-symbol ordering used for subsequent coded blocks.
        //
        //   0  8x8
        //   1  8x4 bottom only
        //   2  8x4 top only
        //   3  8x4 both halves
        //   4  4x8 right only
        //   5  4x8 left only
        //   6  4x8 both halves
        //   7  4x4
        if (t==TransformType::T8x8) return 0;
        if (t==TransformType::T4x4) return 7;
        if (skipmask>2) throw std::runtime_error("invalid rectangular transform skip mask");
        if (t==TransformType::T8x4) return skipmask==0?3:(skipmask==1?2:1);
        return skipmask==0?6:(skipmask==1?5:4);
    }

void Vc1Encoder::write_ttmb_block(BitWriter& b,TransformType t,uint8_t first_skipmask) const {
        const int tab=tt_table_index();
        const int value=ttmb_block_value(t,first_skipmask);
        b.vlc(kTtmbCodes[tab][value],kTtmbBits[tab][value]);
    }

void Vc1Encoder::write_ttblk(BitWriter& b,TransformType t,uint8_t skipmask) const {
        const int tab=tt_table_index();
        const int symbol=tt_symbol_for_variant(tt_variant(t,skipmask));
        b.vlc(kTtblkCodes[tab][symbol],kTtblkBits[tab][symbol]);
    }

void Vc1Encoder::write_rect_subblkpat(BitWriter& b,uint8_t skipmask) {
        if (skipmask==0) write_decode012(b,0);       // both subblocks coded
        else if (skipmask==1) write_decode012(b,2);  // first/top/left only
        else if (skipmask==2) write_decode012(b,1);  // second/bottom/right only
        else throw std::runtime_error("invalid rectangular transform skip mask");
    }

void Vc1Encoder::write_inter_scan(BitWriter& b,const std::array<int,64>& q,TransformType t,
                          int coding_set,bool use_vlc,bool& esc3_lengths_written,
                          bool dquantfrm) const {
        const uint8_t* scan=nullptr; int n=0;
        if (t==TransformType::T8x8) { scan=kInterScan.data(); n=64; }
        else if (t==TransformType::T8x4) { scan=(c_.syntax==StreamSyntax::Wmv9Main?kMainInterScan8x4.data():kInterScan8x4.data()); n=32; }
        else if (t==TransformType::T4x8) { scan=(c_.syntax==StreamSyntax::Wmv9Main?kMainInterScan4x8.data():kInterScan4x8.data()); n=32; }
        else { scan=kInterScan4x4.data(); n=16; }
        int last=-1;
        for (int s=n-1;s>=0;--s) if (q[scan[s]]!=0) { last=s; break; }
        if (last<0) throw std::runtime_error("coded inter transform unexpectedly has no coefficients");
        int previous=-1;
        for (int s=0;s<=last;++s) {
            const int level=q[scan[s]];
            if (!level) continue;
            write_ac_coeff(b,s-previous-1,level,s==last,coding_set,use_vlc,esc3_lengths_written,
                           dquantfrm);
            previous=s;
        }
    }

void Vc1Encoder::write_transform_payload(BitWriter& b,const MbTransformDecision& d,int coding_set,
                                 bool use_vlc,bool& esc3_lengths_written,bool implicit_8x8,
                                 bool dquantfrm) const {
        int first=-1;
        for (int k=0;k<6;++k) if (d.cbp&(1u<<(5-k))) { first=k; break; }
        if (first<0) return;
        if (!c_.variable_transforms || implicit_8x8) {
            for (int k=0;k<6;++k) if (d.cbp&(1u<<(5-k))) {
                const auto& parent=d.parents[static_cast<size_t>(k)];
                if (parent.type!=TransformType::T8x8)
                    throw std::runtime_error("implicit transform must be 8x8");
                if (parent.skip_mask)
                    throw std::runtime_error("8x8 transform cannot carry a subblock skip mask");
                const auto q=load_parent_part(parent,TransformType::T8x8,0);
                write_inter_scan(b,q,TransformType::T8x8,coding_set,use_vlc,
                                 esc3_lengths_written,dquantfrm);
            }
            return;
        }

        const auto& first_parent=d.parents[static_cast<size_t>(first)];
        if (d.signal_level==TransformSignalLevel::Macroblock)
            write_ttmb_macro(b,d.type,first_parent.skip_mask);
        else if (d.signal_level==TransformSignalLevel::Block)
            write_ttmb_block(b,first_parent.type,first_parent.skip_mask);
        // Frame-level transform type is carried by TTFRM in the picture header.

        for (int k=0;k<6;++k) {
            if (!(d.cbp&(1u<<(5-k)))) continue;
            const auto& parent=d.parents[static_cast<size_t>(k)];
            const TransformType t=parent.type;
            if ((d.signal_level==TransformSignalLevel::Macroblock ||
                 d.signal_level==TransformSignalLevel::Frame) && t!=d.type)
                throw std::runtime_error("common transform signaling has mismatched block type");

            if (d.signal_level==TransformSignalLevel::Block) {
                if (k!=first) write_ttblk(b,t,parent.skip_mask);
                // TTMB/TTBLK carry the rectangular subblock pattern themselves.
                // 4x4 always carries a separate SUBBLKPAT.
                if (t==TransformType::T4x4) write_subblkpat(b,parent.skip_mask);
            } else if (d.signal_level==TransformSignalLevel::Macroblock) {
                if (t==TransformType::T4x4) write_subblkpat(b,parent.skip_mask);
                else if ((t==TransformType::T8x4 || t==TransformType::T4x8) && k!=first)
                    write_rect_subblkpat(b,parent.skip_mask);
            } else { // TTFRM / frame-level transform type
                if (t==TransformType::T4x4) write_subblkpat(b,parent.skip_mask);
                else if (t==TransformType::T8x4 || t==TransformType::T4x8)
                    write_rect_subblkpat(b,parent.skip_mask);
            }

            const int np=transform_part_count(t);
            for (int part=0;part<np;++part) {
                const uint8_t bit=static_cast<uint8_t>(1u<<(np-1-part));
                if (parent.skip_mask&bit) continue;
                const auto q=load_parent_part(parent,t,part);
                write_inter_scan(b,q,t,coding_set,use_vlc,
                                 esc3_lengths_written,dquantfrm);
            }
        }
    }

void Vc1Encoder::refresh_transform_estimate(MbTransformDecision& d,int coding_set,bool use_vlc,
                                    bool dquantfrm) const {
        d.cbp=0;
        d.distortion=0.0;
        int first=-1;
        for (int k=0;k<6;++k) {
            auto& p=d.parents[static_cast<size_t>(k)];
            d.distortion+=p.distortion;
            const int np=transform_part_count(p.type);
            const uint8_t allmask=static_cast<uint8_t>((1u<<np)-1u);
            if (p.skip_mask!=allmask) {
                d.cbp|=static_cast<uint8_t>(1u<<(5-k));
                if (first<0) first=k;
            }
        }
        if (first>=0 && d.signal_level==TransformSignalLevel::Block)
            d.type=d.parents[static_cast<size_t>(first)].type;
        if (d.signal_level!=TransformSignalLevel::Block) {
            for (int k=0;k<6;++k) if (d.cbp&(1u<<(5-k)))
                if (d.parents[static_cast<size_t>(k)].type!=d.type)
                    throw std::runtime_error("common transform estimate has mismatched block type");
        }
        d.estimated_bits=0;
        if (d.cbp) {
            BitWriter tmp; bool esc=false;
            const VlcCode& cv=kPcbpVlc[d.cbp]; tmp.vlc(cv.code,cv.bits);
            write_transform_payload(tmp,d,coding_set,use_vlc,esc,false,dquantfrm);
            d.estimated_bits=tmp.bit_count();
        }
    }

size_t Vc1Encoder::estimate_parent_payload_bits(const TransformParentDecision& p,int coding_set,bool use_vlc,
                                        bool dquantfrm) const {
        const int np=transform_part_count(p.type);
        const uint8_t allmask=static_cast<uint8_t>((1u<<np)-1u);
        if (p.skip_mask==allmask) return 0;
        BitWriter tmp; bool esc=false;
        if (p.type==TransformType::T4x4) write_subblkpat(tmp,p.skip_mask);
        else if (p.type==TransformType::T8x4 || p.type==TransformType::T4x8)
            write_rect_subblkpat(tmp,p.skip_mask);
        for (int part=0;part<np;++part) {
            const uint8_t bit=static_cast<uint8_t>(1u<<(np-1-part));
            if (p.skip_mask&bit) continue;
            const auto q=load_parent_part(p,p.type,part);
            write_inter_scan(tmp,q,p.type,coding_set,use_vlc,esc,dquantfrm);
        }
        return tmp.bit_count();
    }

size_t Vc1Encoder::estimate_ttblk_bits(const TransformParentDecision& p) const {
        const int np=transform_part_count(p.type);
        const uint8_t allmask=static_cast<uint8_t>((1u<<np)-1u);
        if (p.skip_mask==allmask) return 0;
        BitWriter tmp;
        write_ttblk(tmp,p.type,p.skip_mask);
        if (p.type==TransformType::T4x4) write_subblkpat(tmp,p.skip_mask);
        return tmp.bit_count();
    }

TransformParentDecision Vc1Encoder::evaluate_transform_parent(const Frame& src,const Frame& pred,int mx,int my,int parent_index,
                                                         TransformType type,int coding_set,bool use_vlc,
                                                         bool allow_trellis,int mquant,bool dquantfrm) const {
        if (parent_index<0 || parent_index>=6) throw std::runtime_error("invalid transform parent index");
        if (mquant < 1) mquant=c_.pqindex;
        const int k=parent_index;
        const std::vector<uint8_t>* sp=nullptr; const std::vector<uint8_t>* pp=nullptr;
        int w=0,h=0,bx=0,by=0;
        if (k<4) { sp=&src.y; pp=&pred.y; w=c_.width; h=c_.height; bx=mx*2+(k&1); by=my*2+((k>>1)&1); }
        else { sp=(k==4)?&src.u:&src.v; pp=(k==4)?&pred.u:&pred.v; w=c_.width/2; h=c_.height/2; bx=mx; by=my; }
        TransformParentDecision pd;
        pd.type=type;
        const int np=transform_part_count(type);
        pd.skip_mask=0;
        std::array<int,64> residual{};
        for (int part=0;part<np;++part) {
            int ox=0,oy=0,bw=8,bh=8; transform_part_geometry(type,part,ox,oy,bw,bh);
            auto q=quantize_inter_part(*sp,*pp,w,h,bx*8+ox,by*8+oy,bw,bh,allow_trellis,mquant,
                                       k>=4?&src.y:nullptr);
            store_parent_part(pd,type,part,q);
            const uint8_t bit=static_cast<uint8_t>(1u<<(np-1-part));
            if (!has_any(q)) { pd.skip_mask|=bit; continue; }
            auto r=inverse_partition(type,q,mquant,dquantfrm);
            for (int yy=0;yy<bh;++yy) for (int xx=0;xx<bw;++xx)
                residual[static_cast<size_t>(oy+yy)*8+ox+xx]=r[static_cast<size_t>(yy)*8+xx];
        }
        uint64_t psse=0;
        for (int yy=0;yy<8 && by*8+yy<h;++yy) for (int xx=0;xx<8 && bx*8+xx<w;++xx) {
            const size_t off=static_cast<size_t>(by*8+yy)*w+bx*8+xx;
            const int recon=std::clamp(static_cast<int>((*pp)[off])+residual[static_cast<size_t>(yy)*8+xx],0,255);
            const int e=static_cast<int>((*sp)[off])-recon;
            psse+=static_cast<uint64_t>(e*e);
        }
        pd.distortion=static_cast<double>(psse);
        pd.estimated_bits=estimate_parent_payload_bits(pd,coding_set,use_vlc,dquantfrm);
        return pd;
    }

MbTransformDecision Vc1Encoder::evaluate_transform_type(const Frame& src,const Frame& pred,int mx,int my,
                                                TransformType type,int coding_set,bool use_vlc,
                                                bool allow_trellis,int mquant,bool dquantfrm) const {
        if (mquant < 1) mquant=c_.pqindex;
        MbTransformDecision d; d.type=type; d.signal_level=TransformSignalLevel::Macroblock;
        for (int k=0;k<6;++k)
            d.parents[static_cast<size_t>(k)]=evaluate_transform_parent(src,pred,mx,my,k,type,coding_set,use_vlc,
                                                                        allow_trellis,mquant,dquantfrm);
        refresh_transform_estimate(d,coding_set,use_vlc,dquantfrm);
        return d;
    }

MbTransformDecision Vc1Encoder::choose_transform_mb(const Frame& src,const Frame& pred,int mx,int my,
                                            int coding_set,bool use_vlc,int mquant,bool dquantfrm,
                                            const TransformPicturePlan* picture_plan) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::TransformRdo);
        if (mquant < 1) mquant=c_.pqindex;
        if (!c_.variable_transforms)
            return evaluate_transform_type(src,pred,mx,my,TransformType::T8x8,coding_set,use_vlc,true,mquant,dquantfrm);

        if (picture_plan && picture_plan->frame_level) {
            MbTransformDecision d=evaluate_transform_type(src,pred,mx,my,picture_plan->frame_type,
                                                         coding_set,use_vlc,true,mquant,dquantfrm);
            d.signal_level=TransformSignalLevel::Frame;
            refresh_transform_estimate(d,coding_set,use_vlc,dquantfrm);
            return d;
        }

        uint64_t sad=0, hedge=0, vedge=0, grad=0;
        for (int py=0;py<16 && my*16+py<c_.height;++py) {
            for (int px=0;px<16 && mx*16+px<c_.width;++px) {
                const int x=mx*16+px,y=my*16+py;
                const size_t o=static_cast<size_t>(y)*c_.width+x;
                const int r=static_cast<int>(src.y[o])-static_cast<int>(pred.y[o]);
                sad+=static_cast<uint64_t>(std::abs(r));
                if ((py&7)==4 && py>0) {
                    const size_t u=static_cast<size_t>(y-1)*c_.width+x;
                    const int rp=static_cast<int>(src.y[u])-static_cast<int>(pred.y[u]);
                    hedge+=static_cast<uint64_t>(std::abs(r-rp));
                }
                if ((px&7)==4 && px>0) {
                    const size_t l=static_cast<size_t>(y)*c_.width+x-1;
                    const int rp=static_cast<int>(src.y[l])-static_cast<int>(pred.y[l]);
                    vedge+=static_cast<uint64_t>(std::abs(r-rp));
                }
                if (px>0) {
                    const size_t l=o-1; const int rp=static_cast<int>(src.y[l])-static_cast<int>(pred.y[l]);
                    grad+=static_cast<uint64_t>(std::abs(r-rp));
                }
                if (py>0) {
                    const size_t u=o-c_.width; const int rp=static_cast<int>(src.y[u])-static_cast<int>(pred.y[u]);
                    grad+=static_cast<uint64_t>(std::abs(r-rp));
                }
            }
        }
        std::array<TransformType,4> list{TransformType::T8x8,TransformType::T8x8,TransformType::T8x8,TransformType::T8x8};
        int count=0;
        const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
        const bool split_x=(mx==mbw-1) && ((c_.width & 15)!=0);
        const bool split_y=(my==mbh-1) && ((c_.height & 15)!=0);
        if (split_x && split_y) {
            // Both crop edges run through this macroblock.  4x4 is the only
            // variable-transform law that prevents an 8-wide or 8-high parent
            // from spanning both visible and hidden regions.
            list[count++]=TransformType::T4x4;
        } else if (split_x) {
            // Split every 8x8 parent vertically.  This also protects the 4:2:0
            // chroma parent when the luma edge happens to land on an 8-pixel
            // boundary but not a 16-pixel macroblock boundary.
            list[count++]=TransformType::T4x8;
            if (sad>static_cast<uint64_t>(64*mquant) && grad>sad*2)
                list[count++]=TransformType::T4x4;
        } else if (split_y) {
            list[count++]=TransformType::T8x4;
            if (sad>static_cast<uint64_t>(64*mquant) && grad>sad*2)
                list[count++]=TransformType::T4x4;
        } else {
            list[count++]=TransformType::T8x8;
            if (sad>static_cast<uint64_t>(64*mquant)) {
                if (hedge*5 > vedge*6) list[count++]=TransformType::T8x4;
                else if (vedge*5 > hedge*6) list[count++]=TransformType::T4x8;
                else { list[count++]=TransformType::T8x4; list[count++]=TransformType::T4x8; }
                if (count<4 && grad > sad*2) list[count++]=TransformType::T4x4;
            }
        }

        const bool scalar_search=c_.trellis>0;
        std::array<MbTransformDecision,4> candidates{};
        for (int i=0;i<count;++i)
            candidates[static_cast<size_t>(i)]=evaluate_transform_type(src,pred,mx,my,list[i],coding_set,use_vlc,
                                                                      !scalar_search,mquant,dquantfrm);

        const double mb_lscale=perceptual_lambda_scale(src.y,c_.width,c_.height,mx*16,my*16,16,16,&pred.y);
        const double lambda=0.75*static_cast<double>(2*mquant)*static_cast<double>(2*mquant)*mb_lscale;
        auto rd=[&](const MbTransformDecision& d) { return d.distortion+lambda*static_cast<double>(d.estimated_bits); };

        int best_macro_i=0;
        double best_macro_rd=rd(candidates[0]);
        for (int i=1;i<count;++i) {
            const double crd=rd(candidates[static_cast<size_t>(i)]);
            if (crd<best_macro_rd) { best_macro_rd=crd; best_macro_i=i; }
        }

        // TTMB block signal level: choose each coded 8x8 parent independently,
        // then charge the exact first-block TTMB + later TTBLK syntax before
        // comparing it with the existing macroblock-level transform decision.
        MbTransformDecision block=candidates[0];
        block.signal_level=TransformSignalLevel::Block;
        for (int k=0;k<6;++k) {
            int bi=0;
            const auto parent_rd=[&](const TransformParentDecision& p) {
                return p.distortion + lambda*static_cast<double>(p.estimated_bits + estimate_ttblk_bits(p));
            };
            double brd=parent_rd(candidates[0].parents[static_cast<size_t>(k)]);
            for (int i=1;i<count;++i) {
                const auto& cp=candidates[static_cast<size_t>(i)].parents[static_cast<size_t>(k)];
                const double crd=parent_rd(cp);
                if (crd<brd) { brd=crd; bi=i; }
            }
            block.parents[static_cast<size_t>(k)]=candidates[static_cast<size_t>(bi)].parents[static_cast<size_t>(k)];
        }
        refresh_transform_estimate(block,coding_set,use_vlc,dquantfrm);

        MbTransformDecision macro=candidates[static_cast<size_t>(best_macro_i)];
        if (c_.trellis>0) {
            macro=evaluate_transform_type(src,pred,mx,my,macro.type,coding_set,use_vlc,true,mquant,dquantfrm);

            // Re-run trellis only for the six parent/type decisions that the
            // scalar block-level search actually selected.  Older code evaluated
            // an entire six-parent macroblock for every distinct selected type
            // and discarded the five unrelated parents.  Reuse the already
            // trellised macroblock winner when its type matches; otherwise
            // evaluate exactly the one parent needed.
            for (int k=0;k<6;++k) {
                const TransformType t=block.parents[static_cast<size_t>(k)].type;
                if (t==macro.type)
                    block.parents[static_cast<size_t>(k)]=macro.parents[static_cast<size_t>(k)];
                else
                    block.parents[static_cast<size_t>(k)]=evaluate_transform_parent(src,pred,mx,my,k,t,coding_set,use_vlc,true,mquant,dquantfrm);
            }
            block.signal_level=TransformSignalLevel::Block;
            refresh_transform_estimate(block,coding_set,use_vlc,dquantfrm);
        }

        if (!c_.extended_transform_signaling)
            return macro; // explicit compatibility/estimator path: macroblock signaling only
        const double macro_rd=rd(macro);
        const double block_rd=rd(block);
        // Prefer macroblock signaling on an exact/near tie; TTBLK is selected
        // only when the per-block transform freedom repays its syntax overhead.
        return block_rd < macro_rd*0.9999 ? block : macro;
    }

void Vc1Encoder::write_mqdiff(BitWriter& b,int pquant,int mquant) {
        const int diff=mquant-pquant;
        if (mquant<1 || mquant>31)
            throw std::runtime_error("invalid DQUANT macroblock quantizer");
        // MQDIFF 0..6 means PQUANT+delta. MQDIFF=7 carries ABSMQ; ABSMQ is
        // absolute and may therefore represent either a finer or coarser MB.
        if (diff>=0 && diff<=6) b.bits(static_cast<uint64_t>(diff),3);
        else { b.bits(7,3); b.bits(static_cast<uint64_t>(mquant),5); }
    }

uint8_t Vc1Encoder::pack_transform_map(TransformType type,uint8_t skipmask) {
        return static_cast<uint8_t>(static_cast<uint8_t>(type)|(skipmask<<3));
    }

size_t Vc1Encoder::count_coded_parts(const TransformParentDecision& p,TransformType type) {
        const int np=transform_part_count(type);
        size_t n=0;
        for (int part=0;part<np;++part) if (!(p.skip_mask&(1u<<(np-1-part)))) ++n;
        return n;
    }

void Vc1Encoder::apply_transform_parent(std::vector<uint8_t>& dst,int w,int h,int x0,int y0,
                                const TransformParentDecision& p,TransformType type,int mquant,bool dquant_derived) const {
        const int np=transform_part_count(type);
        for (int part=0;part<np;++part) {
            if (p.skip_mask&(1u<<(np-1-part))) continue;
            const auto q=load_parent_part(p,type,part);
            add_partition(dst,w,h,x0,y0,type,part,q,mquant,dquant_derived);
        }
    }

int Vc1Encoder::trellis_rate_bits_for_set(int run,int level,bool last,int coding_set) const {
        const int mag=std::abs(level);
        if (run<0 || run>63 || level==0 || mag>max_quantized_level())
            return 1000000;
        if (const auto* direct=find_ac_vlc(coding_set,run,mag,last))
            return static_cast<int>(direct->bits)+1; // sign

        const auto esc=ac_escape(coding_set);
        int best=1000000;
        const int level_delta=max_direct_level(coding_set,run,last);
        // Match write_ac_coeff(): coding set 3 deliberately uses direct VLC,
        // Escape Mode 2, or Escape Mode 3, never the decoder-fragile Mode 1.
        if (coding_set != 3 && level_delta>0 && mag>level_delta) {
            const int base_level=mag-level_delta;
            if (const auto* base=find_ac_vlc(coding_set,run,base_level,last))
                best=static_cast<int>(esc.bits)+1+base->bits+1;
        }
        const int run_delta=max_direct_run(coding_set,mag,last);
        if (run_delta>=0 && run>run_delta) {
            const int base_run=run-run_delta-1;
            if (const auto* base=find_ac_vlc(coding_set,base_run,mag,last))
                best=std::min(best,static_cast<int>(esc.bits)+2+base->bits+1);
        }
        // Trellis treats the picture-global Escape-3 length declaration as
        // already amortized.  It is paid at most once per picture and should
        // not make every transform block artificially afraid of Escape-3.
        const int esc3=static_cast<int>(esc.bits)+2+1+6+1+esc3_level_bits();
        return std::min(best,esc3);
    }

int Vc1Encoder::trellis_rate_bits(int run,int level,bool last,bool intra_family) const {
        static constexpr int intra_sets[3]={0,2,4};
        static constexpr int inter_sets[3]={1,3,5};
        const int* sets=intra_family?intra_sets:inter_sets;
        int bits=1000000;
        // Use the lower envelope of the three legal VC-1 run/level tables for
        // the family.  This deliberately keeps coefficient decisions
        // independent of --ac-mode and the selected table index: table auto
        // selection remains a pure entropy decision and all entropy modes
        // continue to reconstruct the same pixels.
        for (int i=0;i<3;++i)
            bits=std::min(bits,trellis_rate_bits_for_set(run,level,last,sets[i]));
        return bits;
    }

double Vc1Encoder::trellis_coeff_weight(TransformType type,int index) {
        // Squared column energies of the normative 8- and 4-point inverse
        // matrices.  Dividing their product by 1024^2 maps coefficient-domain
        // squared error very closely to spatial SSE, which lets the trellis
        // share the same lambda scale as macroblock transform RDO.
        static constexpr double e8[8]={1152,1156,1168,1156,1152,1156,1168,1156};
        static constexpr double e4[4]={1156,1168,1156,1168};
        const int row=index/8,col=index&7;
        const bool x4=type==TransformType::T4x8 || type==TransformType::T4x4;
        const bool y4=type==TransformType::T8x4 || type==TransformType::T4x4;
        const double ex=x4?e4[col]:e8[col];
        const double ey=y4?e4[row]:e8[row];
        return (ex*ey)/(1024.0*1024.0);
    }

std::array<int,64> Vc1Encoder::trellis_quantize(const std::array<double,64>& coeff,
                                        const uint8_t* scan,int n,int first_scan,
                                        TransformType type,bool intra_family,double lambda_scale,int mquant) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::Trellis);
        if (mquant < 1) mquant=c_.pqindex;
        std::array<int,64> scalar{};
        const bool dquant_derived=mquant!=c_.pqindex;
        const double qscale=static_cast<double>(picture_double_quant(mquant,dquant_derived));
        const int max_level=max_quantized_level();
#if defined(LIBVC1_HAVE_X86_64_V4) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_SANDYBRIDGE) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_PENRYN) || defined(LIBVC1_HAVE_CONROE) || defined(LIBVC1_HAVE_PRESCOTT) || defined(LIBVC1_HAVE_X86_64_V1)
        if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V4 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V3 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Avx2Partial || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Piledriver || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Bulldozer || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::SandyBridge || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V2 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Penryn || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Conroe || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::K10 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Prescott || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V1) {
            const double off=uniform_quantizer()?0.0:static_cast<double>(mquant);
#if defined(LIBVC1_HAVE_X86_64_V4)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V4) simd::quantize_coefficients_x86_64_v4(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Avx2Partial) simd::quantize_coefficients_avx2_partial(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V3) simd::quantize_coefficients_x86_64_v3(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V1) simd::quantize_coefficients_x86_64_v1(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Prescott) simd::quantize_coefficients_prescott(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_K10)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::K10) simd::quantize_coefficients_k10(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_CONROE)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Conroe) simd::quantize_coefficients_conroe(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V2) simd::quantize_coefficients_x86_64_v2(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Penryn) simd::quantize_coefficients_penryn(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::SandyBridge) simd::quantize_coefficients_sandybridge(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Bulldozer) simd::quantize_coefficients_bulldozer(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Piledriver) simd::quantize_coefficients_piledriver(coeff.data(),scalar.data(),64,qscale,off,max_level);
#endif
            for (int s=0;s<first_scan;++s) scalar[static_cast<size_t>(scan[s])]=0;
        } else
#endif
        for (int s=first_scan;s<n;++s) {
            const int idx=scan[s];
            scalar[static_cast<size_t>(idx)]=scalar_quantize_level(coeff[static_cast<size_t>(idx)],mquant,dquant_derived);
        }
        // In bitrate-controlled mode PQ1 is the quality floor.  If trellis
        // sparsifies that floor, the controller has no finer quantizer with
        // which to spend a very high target bitrate.  Preserve scalar PQ1 in
        // HRD mode so ABR operation can still reach Blu-ray-rate targets;
        // CQ1 remains eligible for trellis when the user explicitly wants the
        // rate-distortion tradeoff instead of a bitrate target.
        if (c_.trellis<=0 || (c_.hrd_enabled && mquant==1)) return scalar;

        struct PathNode { int prev=-1; int scan_pos=-1; int level=0; };
        struct State {
            double distortion=0.0;
            uint32_t provisional_bits=0;
            int last_scan=0;
            int node=-1;
            double score=0.0;
        };
        struct RateCacheEntry {
            int run=-1;
            int magnitude=0;
            int bits=1000000;
            bool last=false;
            bool valid=false;
        };
        const int beam_width=c_.trellis>=2?8:4;
        const double lambda=0.15*qscale*qscale*std::clamp(lambda_scale,0.20,1.50);

        // Trellis is invoked for every coded transform part.  Keep its complete
        // beam/path workspace on the stack: the old vectors allocated and freed
        // storage for every part even though the maximum beam is fixed at eight.
        // There are at most three non-zero descendants per state/scan position.
        std::array<PathNode,64*8*3> nodes{};
        size_t node_count=0;
        std::array<State,8> states{};
        size_t state_count=1;
        states[0].last_scan=first_scan-1;
        std::array<RateCacheEntry,128> rate_cache{};
        auto cached_rate=[&](int run,int level,bool last) {
            const int magnitude=std::abs(level);
            const uint32_t h=(static_cast<uint32_t>(run)*1315423911u) ^
                             (static_cast<uint32_t>(magnitude)*2654435761u) ^
                             (last?0x9e3779b9u:0u);
            RateCacheEntry& e=rate_cache[h&(rate_cache.size()-1)];
            if (e.valid && e.run==run && e.magnitude==magnitude && e.last==last) return e.bits;
            const int bits=trellis_rate_bits(run,magnitude,last,intra_family);
            e={run,magnitude,bits,last,true};
            return bits;
        };
        auto state_less=[](const State& a,const State& b) {
            if (a.score!=b.score) return a.score<b.score;
            if (a.provisional_bits!=b.provisional_bits) return a.provisional_bits<b.provisional_bits;
            return a.distortion<b.distortion;
        };

        for (int s=first_scan;s<n;++s) {
            const int idx=scan[s];
            const int q0=scalar[static_cast<size_t>(idx)];
            std::array<int,4> candidates{};
            int nc=0;
            auto add_candidate=[&](int q) {
                q=std::clamp(q,-max_level,max_level);
                for (int i=0;i<nc;++i) if (candidates[static_cast<size_t>(i)]==q) return;
                candidates[static_cast<size_t>(nc++)]=q;
            };
            add_candidate(0);
            if (q0!=0) {
                add_candidate(q0);
                const int sign=q0>0?1:-1;
                add_candidate(q0-sign); // one level toward zero
                if (c_.trellis>=2) add_candidate(q0+sign); // table irregularities can occasionally reward it
            }

            // Distortion depends only on this coefficient/candidate, not on the
            // incoming beam state.  The old loop repeated dequantization and the
            // weighted square once per state.
            std::array<double,4> candidate_dist{};
            for (int ci=0;ci<nc;++ci) {
                const int level=candidates[static_cast<size_t>(ci)];
                const double e=coeff[static_cast<size_t>(idx)]-
                    static_cast<double>(dequant_level(level,mquant,dquant_derived));
                candidate_dist[static_cast<size_t>(ci)]=e*e*trellis_coeff_weight(type,idx);
            }

            std::array<State,32> next{};
            size_t next_count=0;
            for (size_t si=0;si<state_count;++si) {
                const State& st=states[si];
                for (int ci=0;ci<nc;++ci) {
                    const int level=candidates[static_cast<size_t>(ci)];
                    State ns=st;
                    ns.distortion+=candidate_dist[static_cast<size_t>(ci)];
                    if (level!=0) {
                        const int run=s-st.last_scan-1;
                        const int rb=cached_rate(run,level,false);
                        if (rb>=1000000) continue;
                        ns.provisional_bits+=static_cast<uint32_t>(rb);
                        if (node_count>=nodes.size()) throw std::runtime_error("trellis path workspace exhausted");
                        nodes[node_count]=PathNode{st.node,s,level};
                        ns.node=static_cast<int>(node_count++);
                        ns.last_scan=s;
                    }
                    ns.score=ns.distortion+lambda*static_cast<double>(ns.provisional_bits);
                    next[next_count++]=ns;
                }
            }
            // Stable insertion sort is ideal for <=32 states and avoids the
            // allocation machinery of stable_sort.  Only strict predecessors
            // move, preserving the historical insertion-order tie rule exactly.
            for (size_t i=1;i<next_count;++i) {
                const State key=next[i];
                size_t j=i;
                while (j>0 && state_less(key,next[j-1])) { next[j]=next[j-1]; --j; }
                next[j]=key;
            }
            state_count=std::min(next_count,static_cast<size_t>(beam_width));
            for (size_t i=0;i<state_count;++i) states[i]=next[i];
        }

        auto materialize=[&](const State& st) {
            std::array<int,64> q{};
            for (int node=st.node;node>=0;node=nodes[static_cast<size_t>(node)].prev) {
                const PathNode& pn=nodes[static_cast<size_t>(node)];
                q[static_cast<size_t>(scan[pn.scan_pos])]=pn.level;
            }
            return q;
        };
        auto exact_rate_state=[&](const State& st) {
            if (st.node<0) return 0u;
            std::array<int,64> reverse_nodes{};
            int count=0;
            for (int node=st.node;node>=0;node=nodes[static_cast<size_t>(node)].prev)
                reverse_nodes[static_cast<size_t>(count++)]=node;
            uint32_t bits=0;
            int previous=first_scan-1;
            for (int i=count-1;i>=0;--i) {
                const PathNode& pn=nodes[static_cast<size_t>(reverse_nodes[static_cast<size_t>(i)])];
                bits+=static_cast<uint32_t>(cached_rate(pn.scan_pos-previous-1,pn.level,i==0));
                previous=pn.scan_pos;
            }
            return bits;
        };

        size_t best_state=0;
        double best_rd=std::numeric_limits<double>::infinity();
        for (size_t si=0;si<state_count;++si) {
            const State& st=states[si];
            const uint32_t bits=exact_rate_state(st);
            const double rd=st.distortion+lambda*static_cast<double>(bits);
            if (rd<best_rd) { best_state=si; best_rd=rd; }
        }
        return state_count?materialize(states[best_state]):scalar;
    }

std::array<int,64> Vc1Encoder::quantize_ac(const std::vector<uint8_t>& p,
                                   int w, int h, int x0, int y0,
                                   bool chroma, const uint8_t* scan,
                                   int mquant, bool dquant_derived,
                                   const std::vector<uint8_t>* luma_context) const {
        if (mquant<1) mquant=c_.pqindex;
        const auto coeff = forward_transform(p,w,h,x0,y0,chroma?128:(0));
        std::array<int,64> q{};
        if (c_.trellis>0) {
            const double lscale=perceptual_lambda_scale(p,w,h,x0,y0,8,8,nullptr,nullptr,
                                                        chroma?luma_context:nullptr);
            q=trellis_quantize(coeff,scan,64,1,TransformType::T8x8,!chroma,lscale,mquant);
        } else {
#if defined(LIBVC1_HAVE_X86_64_V4) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_SANDYBRIDGE) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_PENRYN) || defined(LIBVC1_HAVE_CONROE) || defined(LIBVC1_HAVE_PRESCOTT) || defined(LIBVC1_HAVE_X86_64_V1)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V4 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V3 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Avx2Partial || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Piledriver || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Bulldozer || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::SandyBridge || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V2 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Penryn || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Conroe || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::K10 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Prescott || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V1) {
                const double qs=static_cast<double>(picture_double_quant(mquant,dquant_derived));
                const double off=uniform_quantizer()?0.0:static_cast<double>(mquant);
#if defined(LIBVC1_HAVE_X86_64_V4)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V4) simd::quantize_coefficients_x86_64_v4(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Avx2Partial) simd::quantize_coefficients_avx2_partial(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V3) simd::quantize_coefficients_x86_64_v3(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V1) simd::quantize_coefficients_x86_64_v1(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Prescott) simd::quantize_coefficients_prescott(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_K10)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::K10) simd::quantize_coefficients_k10(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_CONROE)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Conroe) simd::quantize_coefficients_conroe(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V2) simd::quantize_coefficients_x86_64_v2(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_PENRYN)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Penryn) simd::quantize_coefficients_penryn(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::SandyBridge) simd::quantize_coefficients_sandybridge(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Bulldozer) simd::quantize_coefficients_bulldozer(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
                if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Piledriver) simd::quantize_coefficients_piledriver(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
                q[0]=0;
            } else
#endif
            for (int i=1;i<64;++i) q[static_cast<size_t>(i)]=scalar_quantize_level(coeff[static_cast<size_t>(i)],mquant,dquant_derived);
        }
        return q;
    }

std::array<int,64> Vc1Encoder::quantize_inter(const std::vector<uint8_t>& src,
                                      const std::vector<uint8_t>& pred,
                                      int w,int h,int x0,int y0,
                                      int mquant,bool dquant_derived) const {
        const auto coeff=forward_transform_residual(src,pred,w,h,x0,y0);
        std::array<int,64> q{};
        const int mq=mquant<1?c_.pqindex:mquant;
        const bool derived=dquant_derived || mq!=c_.pqindex;
#if defined(LIBVC1_HAVE_X86_64_V4) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_SANDYBRIDGE) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_PENRYN) || defined(LIBVC1_HAVE_CONROE) || defined(LIBVC1_HAVE_PRESCOTT) || defined(LIBVC1_HAVE_X86_64_V1)
        if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V4 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V3 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Avx2Partial || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Piledriver || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Bulldozer || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::SandyBridge || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V2 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Penryn || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Conroe || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::K10 || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Prescott || c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V1) {
            const double qs=static_cast<double>(picture_double_quant(mq,derived));
            const double off=uniform_quantizer()?0.0:static_cast<double>(mq);
#if defined(LIBVC1_HAVE_X86_64_V4)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V4) simd::quantize_coefficients_x86_64_v4(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Avx2Partial) simd::quantize_coefficients_avx2_partial(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V3) simd::quantize_coefficients_x86_64_v3(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V1) simd::quantize_coefficients_x86_64_v1(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Prescott) simd::quantize_coefficients_prescott(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_K10)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::K10) simd::quantize_coefficients_k10(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_CONROE)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Conroe) simd::quantize_coefficients_conroe(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::X86V2) simd::quantize_coefficients_x86_64_v2(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_PENRYN)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Penryn) simd::quantize_coefficients_penryn(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::SandyBridge) simd::quantize_coefficients_sandybridge(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Bulldozer) simd::quantize_coefficients_bulldozer(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
            if (c_.simd_for(SimdPrimitive::Quantize)==SimdTier::Piledriver) simd::quantize_coefficients_piledriver(coeff.data(),q.data(),64,qs,off,max_quantized_level());
#endif
            return q;
        }
#endif
        for (int i=0;i<64;++i) q[static_cast<size_t>(i)]=scalar_quantize_level(coeff[static_cast<size_t>(i)],mq,derived);
        return q;
    }

bool Vc1Encoder::has_ac(const std::array<int,64>& q) {
        for (int i=1; i<64; ++i)
            if (q[static_cast<size_t>(i)] != 0) return true;
        return false;
    }

bool Vc1Encoder::has_any(const std::array<int,64>& q) {
        for (int i=0;i<64;++i)
            if (q[static_cast<size_t>(i)] != 0) return true;
        return false;
    }

int Vc1Encoder::predict_coded(const std::vector<uint8_t>& g, int gw, int x, int y) {
        const int a = x>0 ? g[static_cast<size_t>(y)*gw+x-1] : 0;
        const int b = (x>0 && y>0) ? g[static_cast<size_t>(y-1)*gw+x-1] : 0;
        const int c = y>0 ? g[static_cast<size_t>(y-1)*gw+x] : 0;
        return (b==c) ? a : c;
    }

} // namespace libvc1
