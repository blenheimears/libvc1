#include "encoder_internal.h"

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

bool Vc1Encoder::strong_contextual_aq_picture(const Frame& f) const {
        if (!c_.adaptive_quality || c_.aq_strength<=0.0 || f.width<=0 || f.height<=0) return false;
        const int mbw=(f.width+15)/16, mbh=(f.height+15)/16;
        const int total=std::max(1,mbw*mbh);
        const int stride=std::max(1,total/32);
        int sampled=0;
        for (int linear=0;linear<total && sampled<32;linear+=stride,++sampled) {
            const int my=linear/mbw,mx=linear%mbw;
            double adjacency=0.0;
            const double ls=perceptual_lambda_scale(f.y,f.width,f.height,mx*16,my*16,16,16,nullptr,&adjacency);
            // This gate is intentionally limited to the strongest contextual
            // AQ classes. It now controls only the AUTO picture-quantizer law;
            // complete DQUANT can coexist with HALFQP, so it no longer suppresses
            // legal low-Q half-step selection. Bright-field detail still benefits
            // from a stable uniform PQUANT law because AUTO PQUANT RDO is
            // source-only and does not model its perceptual weighting.
            if (adjacency>=0.80 || ls<=0.52) return true;
        }
        return false;
    }


int Vc1Encoder::rate_weighted_mquant(double block_weight) const {
    if (!c_.dquant || !c_.rc_block_weighting || !(block_weight>0.0) ||
        !(c_.rc_picture_block_weight>0.0))
        return c_.pqindex;
    // The picture-level ABR target already accounts for the weighted block mix.
    // DQUANT therefore only has to direct that extra mixed-picture budget toward
    // the above-average class; coarsening the lower-weight class again would
    // double-charge it and can destroy nearby temporal detail.  For the favored
    // class, use the same measured qscale exponent as the frame predictor to
    // translate its relative weight into a local finer integer MQUANT.
    constexpr double kRcQExponent=0.55;
    if (block_weight<=c_.rc_picture_block_weight) return c_.pqindex;
    const double ratio=std::clamp(c_.rc_picture_block_weight/block_weight,0.05,1.0);
    const double qvalue=static_cast<double>(c_.pqindex)+(c_.halfqp?0.5:0.0);
    const double desired=qvalue*std::pow(ratio,1.0/kRcQExponent);
    int mq=std::clamp(static_cast<int>(std::lround(desired)),1,31);
    // A DQUANT MQUANT is integer. If the weighted target rounds back to the
    // picture quantizer, preserve PQUANT/HALFQP by not signaling DQUANT.
    if (mq==c_.pqindex) return c_.pqindex;
    return mq;
}

double Vc1Encoder::residual_priority_position(double local_mae) const {
    if (!std::isfinite(local_mae) || c_.rc_residual_max_q_boost<=0.0) return 0.0;
    const double width=std::max(1e-9,c_.rc_residual_width);
    // Continuous linear transition: no hard allocation switch, and the user
    // controls both the lower edge and the width. This also preserves the
    // proven 0.2.22 default allocation law at 8/24/4.
    return std::clamp((local_mae-c_.rc_residual_threshold)/width,0.0,1.0);
}

double Vc1Encoder::residual_priority_requested_q_boost(double local_mae,double picture_mae) const {
    if (!(picture_mae>0.0)) return 0.0;
    const double ramp=residual_priority_position(local_mae);
    if (ramp<=0.0) return 0.0;
    const double mean=std::max(4.0,picture_mae);
    const double relative=std::clamp((local_mae/mean-1.15)/1.85,0.0,1.0);
    const double scene=std::clamp((mean-10.0)/30.0,0.0,1.0);
    return ramp*std::max(relative,0.50*scene)*std::max(0.0,c_.rc_residual_max_q_boost);
}

int Vc1Encoder::residual_priority_mquant(int rate_base,double local_mae,double picture_mae) const {
    if (!c_.rc_block_weighting || !c_.dquant || rate_base<=1) return rate_base;
    const double requested=residual_priority_requested_q_boost(local_mae,picture_mae);
    const int delta=std::clamp(static_cast<int>(std::ceil(requested-1e-12)),0,std::min(12,rate_base-1));
    return std::max(1,rate_base-delta);
}

Vc1Encoder::PerceptualMbPriority Vc1Encoder::perceptual_mb_priority(const Frame& src,int mx,int my,bool intra) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::AdaptiveQuantization);
    PerceptualMbPriority out;
    if (!c_.adaptive_quality || c_.aq_strength<=0.0) return out;
    const int x0=mx*16,y0=my*16;
    const int bw=std::max(0,std::min(16,c_.visible_width()-x0));
    const int bh=std::max(0,std::min(16,c_.visible_height()-y0));
    if (bw<=0 || bh<=0) return out;
    long double ysum=0.0,grad=0.0; uint64_t gedges=0;
    for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) {
        const size_t o=static_cast<size_t>(y0+y)*c_.width+x0+x; const int v=src.y[o]; ysum+=v;
        if (x) { grad+=std::abs(v-static_cast<int>(src.y[o-1])); ++gedges; }
        if (y) { grad+=std::abs(v-static_cast<int>(src.y[o-c_.width])); ++gedges; }
    }
    const double mean_y=static_cast<double>(ysum/static_cast<long double>(bw*bh));
    const double activity_y=gedges?static_cast<double>(grad/static_cast<long double>(gedges)):0.0;
    const double detail=std::clamp((activity_y-1.5)/11.0,0.0,1.0);
    const double dark_window=std::clamp((116.0-mean_y)/92.0,0.0,1.0)*
                             std::clamp((mean_y-1.0)/11.0,0.0,1.0);
    out.dark_detail=dark_window*detail;

    const int cx0=mx*8,cy0=my*8,cbw=std::max(0,std::min(8,c_.visible_chroma_width()-cx0)),
              cbh=std::max(0,std::min(8,c_.visible_chroma_height()-cy0));
    long double us=0.0,vs=0.0;
    for(int y=0;y<cbh;++y)for(int x=0;x<cbw;++x){const size_t o=static_cast<size_t>(cy0+y)*(c_.width/2)+cx0+x;us+=src.u[o];vs+=src.v[o];}
    const double cn=static_cast<double>(std::max(1,cbw*cbh));
    const double cb=static_cast<double>(us)/cn-128.0, cr=static_cast<double>(vs)/cn-128.0;
    const double sat=std::hypot(cb,cr);
    const double cb_pos=std::clamp(cb/52.0,0.0,1.0), cb_neg=std::clamp(-cb/48.0,0.0,1.0);
    const double cr_pos=std::clamp(cr/52.0,0.0,1.0), cr_neg=std::clamp(-cr/48.0,0.0,1.0);
    const double green=std::min(cb_neg,cr_neg);
    const double blue=cb_pos*std::clamp((28.0-cr)/80.0,0.35,1.0);
    const double violet=std::min(cb_pos,cr_pos);
    const double gray=std::clamp((22.0-sat)/22.0,0.0,1.0);
    const double low=std::clamp((112.0-mean_y)/86.0,0.0,1.0);
    const double medium_high=std::clamp((mean_y-72.0)/86.0,0.0,1.0);
    const double low_sensitive=low*std::max({blue,violet,0.78*gray});
    const double green_sensitive=medium_high*green;
    // The shared MQUANT mostly represents a luma decision. Chroma contributes
    // a deliberately smaller modifier so saturated color cannot dominate the
    // block budget by itself. Neutral gray has no chroma-only bonus.
    out.color_luma=detail*std::max(green_sensitive,low_sensitive);
    out.color_chroma=detail*std::max(0.65*green_sensitive,low*std::max(blue,violet));
    // Existing temporal AQ already protects dark/neutral detail and credits-like
    // edges. Keep the new explicit temporal modifier chromatic-only so it does
    // not double-spend those classes and force a coarser picture Q. Intra blocks
    // get the stronger dark-detail path needed by very dark I pictures.
    const double raw=intra ? (4.8*out.dark_detail + 1.55*out.color_luma + 0.45*out.color_chroma)
                           : (out.color_chroma>0.03 ? (0.75*out.color_luma + 0.25*out.color_chroma) : 0.0);
    out.requested_q_boost=std::clamp(c_.aq_strength*raw,0.0,intra?6.0:2.0);
    return out;
}

int Vc1Encoder::choose_intra_mquant(const Frame& src,int mx,int my) const {
    if (!c_.dquant) return c_.pqindex;
    const int rate_base=rate_weighted_mquant(c_.rc_intra_block_weight);
    if (!c_.adaptive_quality || c_.aq_strength<=0.0 || rate_base<=2)
        return rate_base;
    double adjacency=0.0;
    const double lscale=perceptual_lambda_scale(src.y,c_.width,c_.height,mx*16,my*16,16,16,nullptr,&adjacency);
    int mq=rate_base;
    // Preserve the established AQ law exactly, then layer the new explicit
    // dark/color intra protection on top so unrelated content is unchanged.
    if (c_.halfqp && rate_base==c_.pqindex) {
        if (lscale>1.04 && c_.pqindex<31) mq=c_.pqindex+1;
    } else if (lscale<0.98) {
        const int implied=std::clamp(static_cast<int>(std::lround(
            static_cast<double>(rate_base)*std::sqrt(std::clamp(lscale,0.25,1.0)))),1,rate_base);
        mq=std::max(implied,rate_base-4);
    } else if (lscale>1.04 && rate_base<31) {
        const int delta=lscale>1.35?2:1; mq=std::min(31,rate_base+delta);
    }
    const PerceptualMbPriority priority=perceptual_mb_priority(src,mx,my,true);
    const int perceptual_delta=std::clamp(static_cast<int>(std::lround(priority.requested_q_boost)),0,std::min(6,rate_base-1));
    if (perceptual_delta>0) mq=std::min(mq,std::max(1,rate_base-perceptual_delta));
    return mq;
}

bool Vc1Encoder::dquant_edge_member(DQuantProfile profile,int selector,int mx,int my,int mbw,int mbh) {
    int edges=0;
    if (profile==DQuantProfile::FourEdges) edges=15;
    else if (profile==DQuantProfile::SingleEdge) edges=1<<(selector&3);
    else if (profile==DQuantProfile::DoubleEdges) edges=(3<<(selector&3))%15;
    return ((edges&1) && mx==0) || ((edges&2) && my==0) ||
           ((edges&4) && mx==mbw-1) || ((edges&8) && my==mbh-1);
}

Vc1Encoder::DQuantPlan Vc1Encoder::make_dquant_plan(const std::vector<int>& desired,int mbw,int mbh) const {
    DQuantPlan plan;
    plan.mquant=desired;
    if (!c_.dquant || desired.empty()) return plan;
    for (int q:desired) if (q<1 || q>31) throw std::runtime_error("invalid desired DQUANT MQUANT");
    bool any=false;
    std::vector<int> alts;
    for (int q:desired) if (q!=c_.pqindex) {
        any=true;
        if (std::find(alts.begin(),alts.end(),q)==alts.end()) alts.push_back(q);
    }
    if (!any) return plan;
    plan.enabled=true;

    // If the exact desired map is a legal edge profile, that syntax is shorter
    // than carrying one selector bit (bilevel) or three/eight bits per MB.
    if (alts.size()==1) {
        const int alt=alts.front();
        auto exact=[&](DQuantProfile profile,int selector) {
            for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
                const size_t pos=static_cast<size_t>(my)*mbw+mx;
                const int want=dquant_edge_member(profile,selector,mx,my,mbw,mbh)?alt:c_.pqindex;
                if (desired[pos]!=want) return false;
            }
            return true;
        };
        if (exact(DQuantProfile::FourEdges,0)) {
            plan.profile=DQuantProfile::FourEdges; plan.altpq=alt; return plan;
        }
        for (int sel=0;sel<4;++sel) if (exact(DQuantProfile::DoubleEdges,sel)) {
            plan.profile=DQuantProfile::DoubleEdges; plan.edge_selector=sel; plan.altpq=alt; return plan;
        }
        for (int sel=0;sel<4;++sel) if (exact(DQuantProfile::SingleEdge,sel)) {
            plan.profile=DQuantProfile::SingleEdge; plan.edge_selector=sel; plan.altpq=alt; return plan;
        }
        plan.profile=DQuantProfile::AllMbs; plan.bilevel=true; plan.altpq=alt; return plan;
    }
    plan.profile=DQuantProfile::AllMbs;
    plan.bilevel=false;
    return plan;
}

void Vc1Encoder::write_altpq(BitWriter& b,int pquant,int altpq) {
    if (altpq<1 || altpq>31) throw std::runtime_error("invalid DQUANT alternate quantizer");
    const int delta=altpq-pquant;
    if (delta>=1 && delta<=7) b.bits(static_cast<uint64_t>(delta-1),3);
    else { b.bits(7,3); b.bits(static_cast<uint64_t>(altpq),5); }
}

void Vc1Encoder::write_dquant_header(BitWriter& b,const DQuantPlan& plan) const {
    b.bit(plan.enabled);
    if (!plan.enabled) return;
    b.bits(static_cast<uint64_t>(plan.profile),2);
    if (plan.profile==DQuantProfile::SingleEdge || plan.profile==DQuantProfile::DoubleEdges)
        b.bits(static_cast<uint64_t>(plan.edge_selector&3),2);
    if (plan.profile==DQuantProfile::AllMbs) {
        b.bit(plan.bilevel);
        if (!plan.bilevel) return;
    }
    write_altpq(b,c_.pqindex,plan.altpq);
}

void Vc1Encoder::write_dquant_mb(BitWriter& b,const DQuantPlan& plan,size_t pos) const {
    if (!plan.enabled) return;
    if (plan.profile!=DQuantProfile::AllMbs) return;
    if (pos>=plan.mquant.size()) throw std::runtime_error("DQUANT macroblock index out of range");
    if (plan.bilevel) b.bit(plan.mquant[pos]==plan.altpq);
    else write_mqdiff(b,c_.pqindex,plan.mquant[pos]);
}

bool Vc1Encoder::dquant_mb_derived(const DQuantPlan& plan,size_t pos,int mbw) {
    if (!plan.enabled || pos>=plan.mquant.size()) return false;
    if (plan.profile==DQuantProfile::AllMbs)
        return plan.bilevel ? plan.mquant[pos]==plan.altpq : true;
    const int mx=static_cast<int>(pos%static_cast<size_t>(mbw));
    const int my=static_cast<int>(pos/static_cast<size_t>(mbw));
    const int mbh=static_cast<int>((plan.mquant.size()+static_cast<size_t>(mbw)-1)/static_cast<size_t>(mbw));
    return dquant_edge_member(plan.profile,plan.edge_selector,mx,my,mbw,mbh);
}

MbQuantDecision Vc1Encoder::choose_dquant_transform_mb(const Frame& src,const Frame& pred,int mx,int my,
                                               int coding_set,bool use_vlc,
                                               const TransformPicturePlan* picture_plan) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::DQuant);
        int rate_base=rate_weighted_mquant(c_.rc_inter_block_weight);

        // Residual-priority ABR: temporal MBs whose selected motion
        // prediction leaves a large corrective residual get a bounded finer
        // local MQUANT. The picture controller/VBV still owns the hard budget.
        if (c_.rc_prediction_residual_mean>0.0) {
            const int x0=mx*16,y0=my*16;
            const int bw=std::max(0,std::min(16,c_.visible_width()-x0));
            const int bh=std::max(0,std::min(16,c_.visible_height()-y0));
            uint64_t sad=0;
            for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) {
                const size_t o=static_cast<size_t>(y0+y)*c_.width+x0+x;
                sad+=static_cast<uint64_t>(std::abs(static_cast<int>(src.y[o])-
                                                     static_cast<int>(pred.y[o])));
            }
            const double local_mae=(bw>0 && bh>0)
                ? static_cast<double>(sad)/static_cast<double>(bw*bh) : 0.0;
            rate_base=residual_priority_mquant(rate_base,local_mae,c_.rc_prediction_residual_mean);
        }
        if (c_.adaptive_quality && c_.aq_strength>0.0 && c_.dquant && rate_base>1) {
            const auto priority=perceptual_mb_priority(src,mx,my,false);
            const int delta=std::clamp(static_cast<int>(std::lround(priority.requested_q_boost)),0,std::min(2,rate_base-1));
            rate_base=std::max(1,rate_base-delta);
        }

        MbQuantDecision out{choose_transform_mb(src,pred,mx,my,coding_set,use_vlc,rate_base,
                                                rate_base!=c_.pqindex,picture_plan),rate_base};
        if (!c_.dquant)
            return out;

        // 0.1.70: the extended transform law can make a quiet block next to
        // dense detail so cheap that, at high picture Q, ordinary AQ trellis
        // cannot preserve its boundary: the useful residual was already
        // scalar-quantized to zero.  VC-1 arbitrary all-MB DQUANT with MQDIFF=7
        // carries a five-bit absolute MQUANT, and ABSMQ may be below PQUANT.
        // Use that legal path narrowly for the strong 0.1.22 adjacency class.
        // This keeps the general dark/motion/edge AQ budget unchanged while
        // restoring a true spatial quality knob exactly where trellis alone is
        // insufficient.
        if (rate_base<=2 || !c_.adaptive_quality || c_.aq_strength<=0.0)
            return out;
        double adjacency=0.0;
        const double lscale=perceptual_lambda_scale(src.y,c_.width,c_.height,mx*16,my*16,16,16,&pred.y,&adjacency);
        bool bright_on_dark=false;
        {
            const int x0=mx*16,y0=my*16;
            uint64_t sum=0; int n=0, hi=0, lo=0;
            for (int y=0;y<16 && y0+y<c_.height;++y) for (int x=0;x<16 && x0+x<c_.width;++x) {
                const int v=src.y[static_cast<size_t>(y0+y)*c_.width+x0+x];
                sum+=static_cast<uint64_t>(v); ++n;
                if (v>=180) ++hi;
                if (v<=48) ++lo;
            }
            const double mean=n?static_cast<double>(sum)/n:128.0;
            bright_on_dark=n && mean<90.0 && hi>=std::max(2,n/32) && lo>=n/2;
        }
        const bool predictable_texture = c_.aq_predictable_texture;
        if (rate_base>=12 && (adjacency>=0.80 || bright_on_dark || predictable_texture)) {
            // lambda_local=lambda_picture*lscale and lambda approximately q^2,
            // hence Q_local ~= Q_picture*sqrt(lscale). 0.1.71 also uses this
            // legal absolute-MQUANT path for sparse bright-on-dark detail (for
            // example credits), where improved motion prediction can otherwise
            // make AQ's coefficient spend show up as whole-picture ABR debt.
            const int implied=std::clamp(static_cast<int>(std::lround(
                static_cast<double>(rate_base)*std::sqrt(std::clamp(lscale,0.18,1.0)))),1,rate_base);
            const int cap=(adjacency>=0.80)?16:(bright_on_dark?2:1);
            // With the 0.1.72 progressive motion modes, a very good predictor
            // can leave a quiet boundary residual just below the scalar
            // threshold even though the retained reference still carries
            // visible quantization error. For the strongest adjacency class,
            // test four additional PQ indices of correction rather than
            // requiring the old motion law's larger residual to trigger CBP.
            const int protected_implied=adjacency>=0.80 ? std::max(1,implied-4) : implied;
            const int mq=predictable_texture ? std::max(1,rate_base-1)
                                              : std::max(protected_implied,rate_base-cap);
            if (mq<rate_base) {
                const MbTransformDecision cand=choose_transform_mb(src,pred,mx,my,coding_set,use_vlc,mq,true,picture_plan);
                if (cand.cbp || out.tx.cbp) {
                    out.tx=cand;
                    out.mquant=mq;
                    return out;
                }
            }
        }

        // 0.1.62 compatibility: DQUANT must never undo AQ protection. Only an
        // unprotected, coefficient-bearing MB may be coarsened. This remains
        // the established Group-C path for ordinary content.
        if (!out.tx.cbp || rate_base>=31 || lscale < 1.0-1e-12) return out;
        const int max_delta=std::min(4,31-rate_base);
        const double base_q=static_cast<double>(picture_double_quant(rate_base,rate_base!=c_.pqindex));
        const double lambda=0.75*base_q*base_q*lscale;
        auto rd=[&](const MbTransformDecision& d) {
            return d.distortion + lambda*static_cast<double>(d.estimated_bits + (d.cbp?3u:0u));
        };
        double best_rd=rd(out.tx);
        std::array<int,3> deltas = c_.halfqp
            ? std::array<int,3>{1,1,1}
            : std::array<int,3>{std::max(1,max_delta/2),max_delta,std::max(1,max_delta-1)};
        for (int delta:deltas) {
            const int mq=std::min(31,rate_base+delta);
            if (mq<=out.mquant) continue;
            const MbTransformDecision cand=choose_transform_mb(src,pred,mx,my,coding_set,use_vlc,mq,true,picture_plan);
            const double crd=rd(cand);
            if (crd < best_rd*0.9995) { out.tx=cand; out.mquant=mq; best_rd=crd; }
        }
        return out;
    }

double Vc1Encoder::perceptual_lambda_scale(const std::vector<uint8_t>& src,
                                   int w,int h,int x0,int y0,int bw,int bh,
                                   const std::vector<uint8_t>* pred,
                                   double* adjacency_out,
                                   const std::vector<uint8_t>* luma_context) const {
        if (adjacency_out) *adjacency_out=0.0;
        if (!c_.adaptive_quality || c_.aq_strength<=0.0) return 1.0;

        struct LocalStats {
            double mean=128.0;
            double grad_avg=0.0;
            double residual_avg=0.0;
            int mn=255;
            int mx=0;
            int edges=0;
            int n=0;
        };
        auto stats=[&](const std::vector<uint8_t>& plane,int pw,int ph,
                       int sx,int sy,int sw,int sh,const std::vector<uint8_t>* pp)->LocalStats {
            LocalStats st;
            uint64_t sum=0,grad=0,residual=0;
            int grad_n=0;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
            const int vw=std::max(0,std::min(sw,pw-sx));
            const int vh=std::max(0,std::min(sh,ph-sy));
            const SimdTier pst=c_.simd_for(SimdPrimitive::PerceptualStats);
            if ((pst==SimdTier::X86V1 || pst==SimdTier::Prescott || pst==SimdTier::K10 || pst==SimdTier::Conroe || pst==SimdTier::Penryn || pst==SimdTier::X86V2 || pst==SimdTier::SandyBridge || pst==SimdTier::Bulldozer || pst==SimdTier::Piledriver || pst==SimdTier::Avx2Partial || pst==SimdTier::X86V3 || pst==SimdTier::X86V4) && sx>=0 && sy>=0 && vw>0 && vh>0) {
                const size_t o=static_cast<size_t>(sy)*pw+sx;
                bool used=false;
#if defined(LIBVC1_HAVE_X86_64_V4)
                if (pst==SimdTier::X86V4) { simd::perceptual_stats_x86_64_v4(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
                if (pst==SimdTier::Avx2Partial) { simd::perceptual_stats_avx2_partial(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
                if (pst==SimdTier::X86V3) { simd::perceptual_stats_x86_64_v3(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
                if (pst==SimdTier::X86V1) { simd::perceptual_stats_x86_64_v1(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
                if (pst==SimdTier::Prescott) { simd::perceptual_stats_prescott(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_K10)
                if (pst==SimdTier::K10) { simd::perceptual_stats_k10(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_CONROE)
                if (pst==SimdTier::Conroe) { simd::perceptual_stats_conroe(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
                if (pst==SimdTier::X86V2) { simd::perceptual_stats_x86_64_v2(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
                if (pst==SimdTier::Penryn) { simd::perceptual_stats_penryn(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
                if (pst==SimdTier::SandyBridge) { simd::perceptual_stats_sandybridge(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
                if (pst==SimdTier::Bulldozer) { simd::perceptual_stats_bulldozer(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
                if (pst==SimdTier::Piledriver) { simd::perceptual_stats_piledriver(plane.data()+o,pp?pp->data()+o:nullptr,pw,vw,vh,&sum,&grad,&residual,&st.mn,&st.mx,&st.edges,&grad_n); used=true; }
#endif
                if (used) { st.n=vw*vh; st.mean=static_cast<double>(sum)/st.n; st.grad_avg=static_cast<double>(grad)/std::max(1,grad_n); st.residual_avg=pp?static_cast<double>(residual)/st.n:0.0; return st; }
            }
#endif
            for (int y=0;y<sh && sy+y<ph;++y) {
                for (int x=0;x<sw && sx+x<pw;++x) {
                    const int px=sx+x,py=sy+y;
                    const size_t o=static_cast<size_t>(py)*pw+px;
                    const int v=plane[o];
                    sum+=static_cast<uint64_t>(v);
                    st.mn=std::min(st.mn,v); st.mx=std::max(st.mx,v); ++st.n;
                    if (pp) residual+=static_cast<uint64_t>(std::abs(v-static_cast<int>((*pp)[o])));
                    if (x>0) {
                        const int d=std::abs(v-static_cast<int>(plane[o-1]));
                        grad+=static_cast<uint64_t>(d); ++grad_n; if (d>=96) ++st.edges;
                    }
                    if (y>0) {
                        const int d=std::abs(v-static_cast<int>(plane[o-pw]));
                        grad+=static_cast<uint64_t>(d); ++grad_n; if (d>=96) ++st.edges;
                    }
                }
            }
            if (st.n) {
                st.mean=static_cast<double>(sum)/st.n;
                st.grad_avg=static_cast<double>(grad)/std::max(1,grad_n);
                st.residual_avg=pp?static_cast<double>(residual)/st.n:0.0;
            }
            return st;
        };
        auto activity=[&](const LocalStats& st) {
            const double range=static_cast<double>(std::max(0,st.mx-st.mn));
            const double g=std::clamp((st.grad_avg-0.35)/5.0,0.0,1.0);
            const double r=std::clamp((range-1.5)/18.0,0.0,1.0);
            return std::max(g,r);
        };
        auto cached_luma_mean=[&](const std::vector<uint8_t>& luma) {
            if (aq_luma_mean_ptr_!=luma.data() || aq_luma_mean_size_!=luma.size()) {
                uint64_t gsum=0;
#if defined(LIBVC1_HAVE_X86_64_V4)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::X86V4) gsum=simd::sum_u8_x86_64_v4(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::X86V1) gsum=simd::sum_u8_x86_64_v1(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::Prescott) gsum=simd::sum_u8_prescott(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_K10)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::K10) gsum=simd::sum_u8_k10(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_CONROE)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::Conroe) gsum=simd::sum_u8_conroe(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::X86V2) gsum=simd::sum_u8_x86_64_v2(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_PENRYN)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::Penryn) gsum=simd::sum_u8_penryn(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::SandyBridge) gsum=simd::sum_u8_sandybridge(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::Bulldozer) gsum=simd::sum_u8_bulldozer(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::Piledriver) gsum=simd::sum_u8_piledriver(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::Avx2Partial) gsum=simd::sum_u8_avx2_partial(luma.data(),luma.size()); else
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
                if (c_.simd_for(SimdPrimitive::SumU8)==SimdTier::X86V3) gsum=simd::sum_u8_x86_64_v3(luma.data(),luma.size()); else
#endif
                for (uint8_t v:luma) gsum+=v;
                aq_luma_mean_ptr_=luma.data();
                aq_luma_mean_size_=luma.size();
                aq_luma_mean_=luma.empty()?0.0:static_cast<double>(gsum)/luma.size();
            }
            return aq_luma_mean_;
        };

        const LocalStats st=stats(src,w,h,x0,y0,bw,bh,pred);
        if (!st.n) return 1.0;
        const double range=static_cast<double>(std::max(0,st.mx-st.mn));
        const double contrast=std::clamp((range-64.0)/144.0,0.0,1.0) *
                              std::clamp((st.grad_avg-7.0)/40.0,0.0,1.0);
        const double edge_density=std::clamp(static_cast<double>(st.edges)/std::max(1,st.n/3),0.0,1.0);
        const double motion=std::clamp((st.residual_avg-5.0)/30.0,0.0,1.0);

        // Chroma AQ is driven by chroma detail plus the corresponding luma
        // visibility.  In particular, a visually simple/flat luma block must
        // not cause genuinely detailed chroma to be quantized as if the whole
        // macroblock were flat.  Near-black luma attenuates (but does not
        // abruptly remove) the chroma-detail credit so synthetic black frames
        // do not become bitrate sinks.
        if (luma_context) {
            const int lx0=x0*2,ly0=y0*2,lbw=bw*2,lbh=bh*2;
            const LocalStats ly=stats(*luma_context,c_.width,c_.height,lx0,ly0,lbw,lbh,nullptr);
            const double chroma_detail=std::max(
                std::clamp((st.grad_avg-1.5)/11.0,0.0,1.0),
                std::clamp((range-6.0)/48.0,0.0,1.0));
            const double luma_activity=activity(ly);
            const double luma_flat=std::clamp((0.42-luma_activity)/0.42,0.0,1.0);
            const double luma_visibility=0.20+0.80*std::clamp((ly.mean-1.0)/14.0,0.0,1.0);
            const double chroma_on_flat=luma_flat*chroma_detail*luma_visibility;

            const double frame_mean=cached_luma_mean(*luma_context);
            const double dim_area=std::clamp((132.0-ly.mean)/104.0,0.0,1.0);
            const double dim_frame=std::clamp((132.0-frame_mean)/104.0,0.0,1.0);
            const double not_black=std::clamp((ly.mean-0.75)/9.0,0.0,1.0);
            const double dim_detail=std::max(dim_area,0.80*dim_frame)*not_black*chroma_detail;

            const double protect=c_.aq_strength*(1.90*chroma_on_flat + 0.70*dim_detail +
                                                  0.12*contrast + 0.08*edge_density + 0.05*motion);
            return std::clamp(1.0/(1.0+protect),0.24,1.0);
        }

        // Luma darkness protection is based only on luminance, not chroma
        // neutrality, so saturated/dim colored material receives the same
        // perceptual treatment as gray material.  The activity and near-black
        // gates deliberately collapse to zero on perfectly flat or black
        // blocks: low luminance alone is not permission to spend arbitrary
        // bits on information that is not present.
        uint64_t dark_grad=0; int dark_n=0;
        for (int y=0;y<bh && y0+y<h;++y) for (int x=0;x<bw && x0+x<w;++x) {
            const int px=x0+x,py=y0+y;
            const size_t o=static_cast<size_t>(py)*w+px;
            const int v=src[o];
            if (x>0 && v>=2 && v<=112) { dark_grad+=static_cast<uint64_t>(std::abs(v-static_cast<int>(src[o-1]))); ++dark_n; }
            if (y>0 && v>=2 && v<=112) { dark_grad+=static_cast<uint64_t>(std::abs(v-static_cast<int>(src[o-w]))); ++dark_n; }
        }
        const double dark_avg=dark_n?static_cast<double>(dark_grad)/dark_n:0.0;
        const double dark_window=std::clamp((118.0-st.mean)/96.0,0.0,1.0) *
                                 std::clamp((st.mean-1.0)/13.0,0.0,1.0);
        const double dark_detail=dark_window*std::clamp(dark_avg/10.0,0.0,1.0);

        const double frame_mean=(w==c_.width && h==c_.height)?cached_luma_mean(src):st.mean;
        const double dim_area=std::clamp((142.0-st.mean)/116.0,0.0,1.0);
        const double dim_frame=std::clamp((142.0-frame_mean)/116.0,0.0,1.0);
        const double not_black=std::clamp((st.mean-0.75)/9.0,0.0,1.0);
        const double dim_detail=std::max(dim_area,0.82*dim_frame)*not_black*activity(st);

        // Inspect a small ring around the transform block. A quiet block next
        // to dense detail is sensitive to block-boundary error even though its
        // own activity is low. The context term stays local so unrelated busy
        // regions do not receive credit.
        const int pad=4;
        const int cx0=std::max(0,x0-pad),cy0=std::max(0,y0-pad);
        const int cx1=std::min(w,x0+bw+pad),cy1=std::min(h,y0+bh+pad);
        uint64_t ring_grad=0; int ring_n=0,ring_edges=0;
        for (int py=cy0;py<cy1;++py) for (int px=cx0;px<cx1;++px) {
            if (px>=x0 && px<x0+bw && py>=y0 && py<y0+bh) continue;
            const size_t o=static_cast<size_t>(py)*w+px;
            const int v=src[o];
            if (px>cx0) { const int d=std::abs(v-static_cast<int>(src[o-1])); ring_grad+=d; if (d>=64) ++ring_edges; ++ring_n; }
            if (py>cy0) { const int d=std::abs(v-static_cast<int>(src[o-w])); ring_grad+=d; if (d>=64) ++ring_edges; ++ring_n; }
        }
        const double ring_grad_avg=ring_n?static_cast<double>(ring_grad)/ring_n:0.0;
        const double ring_edge_density=ring_n?static_cast<double>(ring_edges)/ring_n:0.0;
        const double context_activity=std::clamp((ring_grad_avg-10.0)/36.0,0.0,1.0) +
                                      0.8*std::clamp((ring_edge_density-0.04)/0.24,0.0,1.0);
        const double quietness=std::clamp((18.0-st.grad_avg)/14.0,0.0,1.0);
        const double neutral_window=std::clamp((190.0-st.mean)/120.0,0.0,1.0) *
                                    std::clamp((st.mean-6.0)/30.0,0.0,1.0);
        const double adjacency=quietness*neutral_window*std::clamp(context_activity,0.0,1.5);
        if (adjacency_out) *adjacency_out=adjacency;

        double bright_field_detail=0.0;
        if ((contrast>0.15 || edge_density>0.25) && w==c_.width && h==c_.height) {
            const double bright_field=std::clamp((frame_mean-165.0)/55.0,0.0,1.0);
            bright_field_detail=bright_field*std::max(contrast,0.65*edge_density);
        }

        const double protect=c_.aq_strength*(8.50*dark_detail + 1.45*dim_detail +
                                              0.20*contrast + 0.10*edge_density +
                                              0.08*motion + 1.00*adjacency +
                                              2.00*bright_field_detail);
        return std::clamp(1.0/(1.0+protect),0.18,1.0);
    }

} // namespace libvc1
