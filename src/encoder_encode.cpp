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
using IEncodeStats = Vc1Encoder::IEncodeStats;
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
using OverlapMode = Vc1Encoder::OverlapMode;
using OverlapPlan = Vc1Encoder::OverlapPlan;
using SignedFrame = Vc1Encoder::SignedFrame;

static constexpr std::array<uint8_t,64> kFrameInterlacedIntraScan = {{
     0, 1, 8, 2, 3, 9,16, 4, 5, 6, 7,10,17,24,11,18,
    25,32,12,13,14,15,19,20,21,22,23,26,33,40,27,34,
    41,48,28,35,42,49,56,57,50,43,36,29,30,31,39,38,
    37,44,51,58,59,52,45,46,47,55,54,53,60,61,62,63
}};


void Vc1Encoder::fill_macroblock_quality(std::vector<vc1_mb_debug_t>& stats,const Frame& source,
                                         const Frame& reconstructed,const Frame* previous_source,
                                         const Frame* prediction,
                                         const std::vector<uint8_t>* prediction_valid) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MacroblockDebug);
    if (stats.empty()) return;
    const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
    const int vw=c_.visible_width(), vh=c_.visible_height();
    const int cvw=c_.visible_chroma_width(), cvh=c_.visible_chroma_height();
    auto plane_metrics=[](const std::vector<uint8_t>& a,const std::vector<uint8_t>& b,int stride,
                          int x0,int y0,int bw,int bh,double& sad,double& mse) {
        if (bw<=0 || bh<=0) { sad=-1.0; mse=-1.0; return; }
        long double se=0.0,ae=0.0;
        for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) {
            const int d=static_cast<int>(a[static_cast<size_t>(y0+y)*stride+x0+x])-
                        static_cast<int>(b[static_cast<size_t>(y0+y)*stride+x0+x]);
            ae+=std::abs(d); se+=static_cast<long double>(d)*d;
        }
        sad=static_cast<double>(ae); mse=static_cast<double>(se/(static_cast<long double>(bw)*bh));
    };
    for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
        const size_t pos=static_cast<size_t>(my)*mbw+mx;
        auto& d=stats[pos];
        d.f_prediction_mae_y=-1.0; d.f_residual_priority_position=-1.0;
        d.f_residual_priority_requested_q_boost=0.0; d.i_residual_priority_applied_q_boost=0;
        if (d.i_forward_distant_match_decision==VC1_DISTANT_MATCH_NONE) d.f_forward_distant_local_mae_y=d.f_forward_distant_candidate_mae_y=-1.0;
        if (d.i_backward_distant_match_decision==VC1_DISTANT_MATCH_NONE) d.f_backward_distant_local_mae_y=d.f_backward_distant_candidate_mae_y=-1.0;
        if (d.i_mode==VC1_MB_DEBUG_I || d.i_mode==VC1_MB_DEBUG_BI_INTRA || d.i_mode==VC1_MB_DEBUG_FIELD_P_FORWARD ||
            d.i_mode==VC1_MB_DEBUG_FIELD_B_FORWARD || d.i_mode==VC1_MB_DEBUG_FIELD_B_BACKWARD) d.f_inter_intra_cost_ratio=-1.0;
        d.i_mb_x=static_cast<uint32_t>(mx); d.i_mb_y=static_cast<uint32_t>(my);
        const int x0=mx*16,y0=my*16;
        const int bw=std::max(0,std::min(16,vw-x0)), bh=std::max(0,std::min(16,vh-y0));
        d.i_visible_width=static_cast<uint32_t>(bw); d.i_visible_height=static_cast<uint32_t>(bh);
        long double sum=0.0,sum2=0.0,activity=0.0;
        for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) {
            const int v=source.y[static_cast<size_t>(y0+y)*c_.width+x0+x];
            sum+=v; sum2+=static_cast<long double>(v)*v;
            if (x) activity+=std::abs(v-static_cast<int>(source.y[static_cast<size_t>(y0+y)*c_.width+x0+x-1]));
            if (y) activity+=std::abs(v-static_cast<int>(source.y[static_cast<size_t>(y0+y-1)*c_.width+x0+x]));
        }
        const long double n=static_cast<long double>(std::max(1,bw*bh));
        d.f_mean_y=static_cast<double>(sum/n);
        const long double var=std::max<long double>(0.0,sum2/n-(sum/n)*(sum/n));
        d.f_stddev_y=std::sqrt(static_cast<double>(var));
        const long double edges=static_cast<long double>(std::max(1,bh*std::max(0,bw-1)+bw*std::max(0,bh-1)));
        d.f_activity_y=static_cast<double>(activity/edges);
        const int cx0=mx*8,cy0=my*8,cbw=std::max(0,std::min(8,cvw-cx0)),cbh=std::max(0,std::min(8,cvh-cy0));
        auto mean_plane=[&](const std::vector<uint8_t>& p) {
            long double z=0.0; for(int y=0;y<cbh;++y)for(int x=0;x<cbw;++x)z+=p[static_cast<size_t>(cy0+y)*(c_.width/2)+cx0+x];
            return static_cast<double>(z/static_cast<long double>(std::max(1,cbw*cbh)));
        };
        d.f_mean_u=mean_plane(source.u); d.f_mean_v=mean_plane(source.v);
        const auto perceptual=perceptual_mb_priority(source,mx,my,d.b_intra!=0 || d.i_mode==VC1_MB_DEBUG_I || d.i_mode==VC1_MB_DEBUG_BI_INTRA);
        d.f_aq_dark_detail=perceptual.dark_detail; d.f_aq_color_luma_priority=perceptual.color_luma;
        d.f_aq_color_chroma_priority=perceptual.color_chroma; d.f_aq_requested_q_boost=perceptual.requested_q_boost;
        d.f_previous_sad_y=d.f_previous_mse_y=-1.0;
        if (previous_source && previous_source->width==source.width && previous_source->height==source.height)
            plane_metrics(source.y,previous_source->y,c_.width,x0,y0,bw,bh,d.f_previous_sad_y,d.f_previous_mse_y);
        d.f_prediction_sad_y=d.f_prediction_mse_y=d.f_prediction_gain_db=-1.0;
        const bool pred_ok=prediction && (!prediction_valid || (pos<prediction_valid->size() && (*prediction_valid)[pos]));
        if (pred_ok) {
            plane_metrics(source.y,prediction->y,c_.width,x0,y0,bw,bh,d.f_prediction_sad_y,d.f_prediction_mse_y);
            d.f_prediction_mae_y=d.f_prediction_sad_y/static_cast<double>(std::max(1,bw*bh));
            d.f_residual_priority_position=residual_priority_position(d.f_prediction_mae_y);
            d.f_residual_priority_requested_q_boost=residual_priority_requested_q_boost(d.f_prediction_mae_y,c_.rc_prediction_residual_mean);
            const int residual_mq=residual_priority_mquant(rate_weighted_mquant(c_.rc_inter_block_weight),
                                                          d.f_prediction_mae_y,c_.rc_prediction_residual_mean);
            d.i_residual_priority_applied_q_boost=std::max(0,rate_weighted_mquant(c_.rc_inter_block_weight)-residual_mq);
            if (d.f_previous_mse_y>=0.0) {
                if (d.f_prediction_mse_y==0.0) d.f_prediction_gain_db=99.0;
                else if (d.f_previous_mse_y>0.0) d.f_prediction_gain_db=10.0*std::log10(d.f_previous_mse_y/d.f_prediction_mse_y);
            }
        }
        double dummy=0.0;
        plane_metrics(source.y,reconstructed.y,c_.width,x0,y0,bw,bh,dummy,d.f_recon_mse_y);
        plane_metrics(source.u,reconstructed.u,c_.width/2,cx0,cy0,cbw,cbh,dummy,d.f_recon_mse_u);
        plane_metrics(source.v,reconstructed.v,c_.width/2,cx0,cy0,cbw,cbh,dummy,d.f_recon_mse_v);
        const double ny=static_cast<double>(bw*bh),nc=static_cast<double>(cbw*cbh);
        d.f_recon_mse_yuv=(d.f_recon_mse_y*ny+(d.f_recon_mse_u+d.f_recon_mse_v)*nc)/std::max(1.0,ny+2.0*nc);
        d.f_recon_psnr_y_db=d.f_recon_mse_y==0.0?99.0:10.0*std::log10((255.0*255.0)/d.f_recon_mse_y);
        d.f_recon_psnr_yuv_db=d.f_recon_mse_yuv==0.0?99.0:10.0*std::log10((255.0*255.0)/d.f_recon_mse_yuv);
        const double signal_power=static_cast<double>(sum2/n);
        d.f_recon_snr_y_db=d.f_recon_mse_y==0.0?99.0:(signal_power>0.0?10.0*std::log10(signal_power/d.f_recon_mse_y):-99.0);
        long double chroma_signal=0.0;
        for (int y=0;y<cbh;++y) for (int x=0;x<cbw;++x) {
            const int u=source.u[static_cast<size_t>(cy0+y)*(c_.width/2)+cx0+x];
            const int v=source.v[static_cast<size_t>(cy0+y)*(c_.width/2)+cx0+x];
            chroma_signal+=static_cast<long double>(u)*u+static_cast<long double>(v)*v;
        }
        const double yuv_signal=(static_cast<double>(sum2)+static_cast<double>(chroma_signal))/std::max(1.0,ny+2.0*nc);
        d.f_recon_snr_yuv_db=d.f_recon_mse_yuv==0.0?99.0:(yuv_signal>0.0?10.0*std::log10(yuv_signal/d.f_recon_mse_yuv):-99.0);
    }
}

std::vector<uint8_t> Vc1Encoder::encode_i_picture(const Frame& f, IEncodeStats* stats, bool bi_picture,
                                                     const Frame* previous_source) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::IPicture);
        const int mbw = (c_.width + 15) / 16;
        const int mbh = (c_.height + 15) / 16;
        const int ybw = mbw * 2;
        const int ybh = mbh * 2;
        const uint8_t* const base_intra_scan =
            (c_.syntax==StreamSyntax::Advanced && c_.interlaced())
                ? kFrameInterlacedIntraScan.data() : kIntraScan.data();
        const int cbw = mbw;
        const int cbh = mbh;
        const size_t mbs=static_cast<size_t>(mbw)*mbh;
        if (stats) *stats=IEncodeStats{};
        if (stats && c_.debug_macroblock_stats) {
            stats->macroblock_debug.resize(mbs);
            for (size_t pos=0;pos<mbs;++pos) {
                auto& d=stats->macroblock_debug[pos]; d.i_mode=bi_picture?VC1_MB_DEBUG_BI_INTRA:VC1_MB_DEBUG_I;
                d.b_intra=1; d.i_picture_q=c_.pqindex; d.i_mquant=c_.pqindex; d.i_field_index=d.i_field_parity=-1; d.i_forward_reference_display_order=d.i_backward_reference_display_order=-1; d.f_previous_sad_y=d.f_previous_mse_y=-1.0;
                d.f_prediction_sad_y=d.f_prediction_mse_y=d.f_prediction_gain_db=-1.0;
            }
        }

        // Advanced I/BI pictures carry frame-selectable DQUANT exactly like
        // predictive pictures.  Main-profile intra pictures do not consume
        // GET_MQUANT(), even though Main sequence syntax can enable DQUANT for
        // P/B pictures, so keep their I/BI macroblocks at picture PQUANT.
        std::vector<int> desired_mquant(mbs,c_.pqindex);
        if (c_.syntax==StreamSyntax::Advanced && c_.dquant) {
            for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx)
                desired_mquant[static_cast<size_t>(my)*mbw+mx]=choose_intra_mquant(f,mx,my);
        }
        DQuantPlan dquant_plan;
        dquant_plan.mquant=desired_mquant;
        if (c_.syntax==StreamSyntax::Advanced)
            dquant_plan=make_dquant_plan(desired_mquant,mbw,mbh);
        auto mb_mquant=[&](size_t pos) { return dquant_plan.mquant.empty()?c_.pqindex:dquant_plan.mquant[pos]; };
        auto mb_derived=[&](size_t pos) { return dquant_mb_derived(dquant_plan,pos,mbw); };
        const OverlapPlan overlap_plan=choose_i_overlap_plan(f);
        if (stats) {
            for (size_t pos=0;pos<mbs;++pos)
                record_mquant(*stats,c_.pqindex,mb_mquant(pos));
        }

        // AC table indices are picture-header syntax, so quantize the picture
        // first.  The same stored coefficients are then used for both scoring
        // and final emission; auto mode cannot change reconstruction quality.
        std::vector<std::array<int,64>> qblocks(static_cast<size_t>(mbw)*mbh*6);
        if (c_.ac_coding) {
            for (int my=0; my<mbh; ++my) {
                for (int mx=0; mx<mbw; ++mx) {
                    for (int k=0; k<6; ++k) {
                        const std::vector<uint8_t>* plane=nullptr;
                        int width=0,height=0,bx=0,by=0;
                        if (k < 4) {
                            plane=&f.y; width=c_.width; height=c_.height;
                            bx=mx*2+(k&1); by=my*2+((k>>1)&1);
                        } else {
                            plane=(k==4)?&f.u:&f.v; width=c_.width/2; height=c_.height/2;
                            bx=mx; by=my;
                        }
                        const size_t pos=static_cast<size_t>(my)*mbw+mx;
                        qblocks[block_index(mbw,mx,my,k)] =
                            quantize_ac(*plane,width,height,bx*8,by*8,k>=4,base_intra_scan,
                                        mb_mquant(pos),mb_derived(pos),k>=4?&f.y:nullptr);
                    }
                }
            }
        }

        int y_table_index=c_.ac_y_table_index;
        int c_table_index=c_.ac_c_table_index;
        if (c_.ac_coding && c_.ac_mode == AcMode::Auto) {
            // AC prediction can change the coefficient distribution seen by the
            // picture-level AC VLC tables.  This matters especially with I/BI
            // DQUANT, where neighbouring intra blocks may use different MQUANTs
            // and the quantizer-scaled predictor can materially change an edge.
            // Precompute the table-independent predictor state once, then score
            // each legal luma/chroma table pair against the *transmitted*
            // (possibly AC-predicted) coefficients.  The old raw-qblock score
            // could select a table pair larger than a fixed pair after ACPRED.
            std::vector<std::array<int,64>> pred_qblocks=qblocks;
            std::vector<uint8_t> pred_left(qblocks.size(),0);
            std::vector<uint8_t> pred_available(qblocks.size(),0);
            std::vector<uint8_t> pred_plausible(mbs,0),pred_strong(mbs,0);
            std::vector<int> t_ydc(static_cast<size_t>(ybw)*ybh,0);
            std::vector<int> t_udc(static_cast<size_t>(cbw)*cbh,0);
            std::vector<int> t_vdc(static_cast<size_t>(cbw)*cbh,0);
            std::vector<int> t_yq(static_cast<size_t>(ybw)*ybh,0);
            std::vector<int> t_uq(static_cast<size_t>(cbw)*cbh,0);
            std::vector<int> t_vq(static_cast<size_t>(cbw)*cbh,0);
            const int max_level=max_quantized_level();
            for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
                const size_t mbpos=static_cast<size_t>(my)*mbw+mx;
                const int mq=mb_mquant(mbpos);
                const int qstate=mb_derived(mbpos)?-mq:mq;
                uint64_t plain_edge_abs=0,pred_edge_abs=0;
                bool valid=true;
                for (int k=0;k<6;++k) {
                    const std::vector<uint8_t>* plane=nullptr;
                    int width=0,height=0,bx=0,by=0,gw=0;
                    std::vector<int>* dcg=nullptr;
                    std::vector<int>* qg=nullptr;
                    if (k<4) {
                        plane=&f.y; width=c_.width; height=c_.height;
                        bx=mx*2+(k&1); by=my*2+((k>>1)&1);
                        dcg=&t_ydc; qg=&t_yq; gw=ybw;
                    } else {
                        plane=(k==4)?&f.u:&f.v; width=c_.width/2; height=c_.height/2;
                        bx=mx; by=my; dcg=(k==4)?&t_udc:&t_vdc;
                        qg=(k==4)?&t_uq:&t_vq; gw=cbw;
                    }
                    bool left=false;
                    int target=choose_dc_for_mean(padded_block_mean(*plane,width,height,bx*8,by*8,k>=4?128:0),mq);
                    const bool a_avail=(my!=0)||(k==2||k==3);
                    const bool c_avail=(mx!=0)||(k==1||k==3);
                    const int dc_pred=c_.syntax==StreamSyntax::Wmv9Main
                        ? predict_dc_main(*dcg,gw,bx,by,main_dc_pred_base(),&left)
                        : predict_dc_dquant(*dcg,*qg,gw,bx,by,mq,a_avail,c_avail,&left);
                    target=clamp_dc_target_for_diff(target,dc_pred,mq);
                    const size_t bi=block_index(mbw,mx,my,k);
                    pred_left[bi]=left?1:0;

                    int nbx=bx,nby=by;
                    if (left) --nbx; else --nby;
                    if (nbx>=0 && nby>=0) {
                        pred_available[bi]=1;
                        size_t nbi=0,ni=0;
                        if (k<4) {
                            const int nmx=nbx/2,nmy=nby/2;
                            const int nk=(nby&1)*2+(nbx&1);
                            nbi=block_index(mbw,nmx,nmy,nk);
                            ni=static_cast<size_t>(nby)*ybw+nbx;
                        } else {
                            nbi=block_index(mbw,nbx,nby,k);
                            ni=static_cast<size_t>(nby)*cbw+nbx;
                        }
                        for (int n=1;n<8;++n) {
                            const int ci=left?n:n*8;
                            const int original=qblocks[bi][static_cast<size_t>(ci)];
                            const int predicted=scale_ac_predictor(qblocks[nbi][static_cast<size_t>(ci)],(*qg)[ni],qstate);
                            const int residual=original-predicted;
                            pred_qblocks[bi][static_cast<size_t>(ci)]=residual;
                            plain_edge_abs+=static_cast<uint64_t>(std::abs(original));
                            pred_edge_abs+=static_cast<uint64_t>(std::abs(residual));
                            if (std::abs(residual)>max_level) valid=false;
                        }
                    }
                    const size_t gi=static_cast<size_t>(by)*gw+bx;
                    (*dcg)[gi]=target;
                    (*qg)[gi]=qstate;
                }
                const bool plausible=valid && plain_edge_abs>0 && pred_edge_abs*4u<plain_edge_abs*3u;
                pred_plausible[mbpos]=plausible?1:0;
                pred_strong[mbpos]=(plausible && pred_edge_abs*2u<plain_edge_abs)?1:0;
            }

            auto score_pair=[&](int yi,int ci) {
                const int yset=luma_coding_set(yi,c_.pqindex);
                const int cset=chroma_coding_set(ci,c_.pqindex);
                std::vector<uint8_t> ycoded(static_cast<size_t>(ybw)*ybh,0);
                BitWriter score;
                write_decode012(score,ci);
                write_decode012(score,yi);
                bool esc3_lengths_written=false;
                struct Candidate {
                    std::array<bool,6> coded{};
                    uint8_t cbp=0;
                    uint64_t bits=0;
                };
                for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
                    const size_t mbpos=static_cast<size_t>(my)*mbw+mx;
                    auto candidate=[&](bool acpred,bool estimate_bits) {
                        Candidate cand;
                        std::array<uint8_t,4> saved{};
                        for (int k=0;k<4;++k) {
                            const int bx=mx*2+(k&1),by=my*2+((k>>1)&1);
                            saved[static_cast<size_t>(k)]=ycoded[static_cast<size_t>(by)*ybw+bx];
                        }
                        for (int k=0;k<6;++k) {
                            const size_t bi=block_index(mbw,mx,my,k);
                            const auto& q=acpred?pred_qblocks[bi]:qblocks[bi];
                            cand.coded[static_cast<size_t>(k)]=has_ac(q);
                            bool transmitted=cand.coded[static_cast<size_t>(k)];
                            if (k<4) {
                                const int bx=mx*2+(k&1),by=my*2+((k>>1)&1);
                                const int cp=predict_coded(ycoded,ybw,bx,by);
                                transmitted = cand.coded[static_cast<size_t>(k)] ^ (cp!=0);
                                ycoded[static_cast<size_t>(by)*ybw+bx]=cand.coded[static_cast<size_t>(k)]?1:0;
                            }
                            if (transmitted) cand.cbp|=static_cast<uint8_t>(1u<<(5-k));
                        }
                        for (int k=0;k<4;++k) {
                            const int bx=mx*2+(k&1),by=my*2+((k>>1)&1);
                            ycoded[static_cast<size_t>(by)*ybw+bx]=saved[static_cast<size_t>(k)];
                        }
                        if (estimate_bits) {
                            cand.bits=static_cast<uint64_t>(kMbIntraVlc[cand.cbp].bits)+1u;
                            for (int k=0;k<6;++k) if (cand.coded[static_cast<size_t>(k)]) {
                                const size_t bi=block_index(mbw,mx,my,k);
                                const int set=k>=4?cset:yset;
                                const uint8_t* scan=base_intra_scan;
                                // In FCM=10 an ACPRED macroblock can still contain a
                                // block with no usable neighbour.  VC-1 then sets
                                // use_pred=0 for that block and decodes its AC with
                                // the interlaced base scan rather than the directional
                                // ACPRED scan.  Progressive pictures use the
                                // directional scan whenever ACPRED is signalled.
                                if (acpred && (!c_.interlaced() || pred_available[bi]))
                                    scan=pred_left[bi]?kIntraLeftPredScan.data():kIntraTopPredScan.data();
                                const auto& q=acpred?pred_qblocks[bi]:qblocks[bi];
                                cand.bits+=estimate_intra_ac_bits(q,set,true,scan);
                            }
                        }
                        return cand;
                    };

                    bool use_acpred=false;
                    Candidate chosen;
                    if (!pred_plausible[mbpos]) chosen=candidate(false,false);
                    else if (pred_strong[mbpos]) { use_acpred=true; chosen=candidate(true,false); }
                    else {
                        const Candidate plain=candidate(false,true);
                        const Candidate pred=candidate(true,true);
                        use_acpred=pred.bits<plain.bits;
                        chosen=use_acpred?pred:plain;
                    }
                    for (int k=0;k<4;++k) {
                        const int bx=mx*2+(k&1),by=my*2+((k>>1)&1);
                        ycoded[static_cast<size_t>(by)*ybw+bx]=chosen.coded[static_cast<size_t>(k)]?1:0;
                    }
                    score.vlc(kMbIntraVlc[chosen.cbp].code,kMbIntraVlc[chosen.cbp].bits);
                    score.bit(use_acpred);
                    for (int k=0;k<6;++k) if (chosen.coded[static_cast<size_t>(k)]) {
                        const size_t bi=block_index(mbw,mx,my,k);
                        const int set=k>=4?cset:yset;
                        const uint8_t* scan=base_intra_scan;
                        if (use_acpred && (!c_.interlaced() || pred_available[bi]))
                            scan=pred_left[bi]?kIntraLeftPredScan.data():kIntraTopPredScan.data();
                        const auto& q=use_acpred?pred_qblocks[bi]:qblocks[bi];
                        write_ac_block(score,q,k>=4,set,true,esc3_lengths_written,scan,dquant_plan.enabled);
                    }
                }
                return score.bit_count();
            };

            uint64_t best_bits=std::numeric_limits<uint64_t>::max();
            int best_y=0,best_c=0;
            for (int ci=0;ci<3;++ci) for (int yi=0;yi<3;++yi) {
                const uint64_t bits=score_pair(yi,ci);
                if (bits<best_bits) { best_bits=bits; best_y=yi; best_c=ci; }
            }
            y_table_index=best_y;
            c_table_index=best_c;
        }
        const int y_coding_set=luma_coding_set(y_table_index,c_.pqindex);
        const int c_coding_set=chroma_coding_set(c_table_index,c_.pqindex);

        // DCTABLE is picture-level syntax. Score both legal DC VLC tables from
        // a cheap DC-only predictor pass; this touches each intra block once
        // but performs no transform or reconstruction work.
        uint64_t dc_cost[2]={0,0};
        std::vector<int> s_ydc(static_cast<size_t>(ybw)*ybh,0);
        std::vector<int> s_udc(static_cast<size_t>(cbw)*cbh,0);
        std::vector<int> s_vdc(static_cast<size_t>(cbw)*cbh,0);
        std::vector<int> s_yq(static_cast<size_t>(ybw)*ybh,0);
        std::vector<int> s_uq(static_cast<size_t>(cbw)*cbh,0);
        std::vector<int> s_vq(static_cast<size_t>(cbw)*cbh,0);
        for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) for (int k=0;k<6;++k) {
            const std::vector<uint8_t>* plane=nullptr; int width=0,height=0,bx=0,by=0;
            if (k<4) { plane=&f.y; width=c_.width; height=c_.height; bx=mx*2+(k&1); by=my*2+((k>>1)&1); }
            else { plane=(k==4)?&f.u:&f.v; width=c_.width/2; height=c_.height/2; bx=mx; by=my; }
            std::vector<int>* grid=nullptr; std::vector<int>* qgrid=nullptr; int gw=0;
            if (k<4) { grid=&s_ydc; qgrid=&s_yq; gw=ybw; }
            else if (k==4) { grid=&s_udc; qgrid=&s_uq; gw=cbw; }
            else { grid=&s_vdc; qgrid=&s_vq; gw=cbw; }
            const size_t mbpos=static_cast<size_t>(my)*mbw+mx;
            const int mq=mb_mquant(mbpos);
            int target=choose_dc_for_mean(padded_block_mean(*plane,width,height,bx*8,by*8,k>=4?128:0),mq);
            const bool a_avail=(my!=0)||(k==2||k==3);
            const bool c_avail=(mx!=0)||(k==1||k==3);
            const int pred=c_.syntax==StreamSyntax::Wmv9Main
                ? predict_dc_main(*grid,gw,bx,by,main_dc_pred_base(),nullptr)
                : predict_dc_dquant(*grid,*qgrid,gw,bx,by,mq,a_avail,c_avail,nullptr);
            target=clamp_dc_target_for_diff(target,pred,mq);
            const int diff=target-pred;
            dc_cost[0]+=dc_diff_bits(0,diff,k>=4,mq);
            dc_cost[1]+=dc_diff_bits(1,diff,k>=4,mq);
            const size_t gi=static_cast<size_t>(by)*gw+bx;
            (*grid)[gi]=target;
            (*qgrid)[gi]=mb_derived(mbpos)?-mq:mq;
        }
        const int dc_table_index=dc_cost[1]<dc_cost[0]?1:0;

        BitWriter b;
        if (c_.syntax == StreamSyntax::Wmv9Main) {
            b.bits(0,2); // FRMCNT (unused)
            if (bi_picture) {
                if (c_.max_b_frames <= 0)
                    throw std::runtime_error("BI picture requires B pictures to be enabled");
                b.bits(0b00,2); // PTYPE = B/BI.
                b.bits(0x7f,7); // BFRACTION = BI (1111111).
            } else if (c_.max_b_frames > 0) {
                b.bits(0b01,2); // PTYPE = I when B pictures are enabled.
            } else {
                b.bit(false);   // PTYPE = I without B pictures.
            }
            // Both I and BI carry the legacy buffer-fullness field.
            b.bits(0,7);
        } else {
            // Interlaced Advanced I/BI pictures use true frame-interlace
            // syntax (FCM=10).  We deliberately keep every macroblock in
            // frame-transform mode for now: an all-zero FIELDTX bitplane is
            // signalled below, so the existing frame-layout intra coder and
            // reconstruction remain decoder-equivalent while the picture is
            // normatively interlaced.  Progressive Advanced pictures omit FCM.
            if (c_.interlaced()) write_decode012(b,1); // FCM=10.
            if (bi_picture) b.bits(0b1110,4);
            else b.bits(0b110,3);
            if (c_.interlaced()) { b.bit(c_.top_field_first()); b.bit(false); } // TFF, RFF=0.
            b.bit(false); // RNDCTRL
            if (c_.interlaced()) b.bit(false); // UVSAMP=0: interlaced chroma sampling metadata.
        }
        b.bits(static_cast<uint64_t>(c_.pqindex),5);
        if (c_.pqindex <= 8) b.bit(c_.halfqp);
        b.bit(uniform_quantizer()); // PQUANTIZER (QUANTIZER=frame-explicit).
        if (c_.syntax == StreamSyntax::Wmv9Main && extended_mv_enabled()) write_mvrange(b);

        if (c_.syntax == StreamSyntax::Advanced) {
            if (c_.interlaced()) {
                // FCM=10 I/BI syntax carries FIELDTX before ACPRED.  Keep
                // frame transforms for every macroblock; choose_bitplane()
                // emits the shortest legal all-zero representation.
                std::vector<uint8_t> fieldtx(mbs,0);
                b.append(choose_bitplane(fieldtx,mbw,mbh).syntax);
            }
            // Advanced Profile carries ACPRED as a picture bitplane.  Select
            // IMODE_RAW so one prediction bit follows per macroblock.
            b.bit(false);
            b.bits(0,4);
            if (c_.overlap && c_.pqindex<=8) {
                write_decode012(b,static_cast<int>(overlap_plan.mode)); // CONDOVER
                if (overlap_plan.mode==OverlapMode::Select) b.append(overlap_plan.bitplane.syntax);
            }
        }
        // Simple/Main Profile has no picture-level ACPRED bitplane syntax; its
        // I-picture macroblock parser reads the ACPRED bit directly per MB.

        // AC table selection is picture-level syntax. C precedes Y in the
        // Advanced Profile I-picture header; index 0 is one bit, 1/2 two bits.
        write_decode012(b,c_table_index);
        write_decode012(b,y_table_index);
        b.bit(dc_table_index != 0); // adaptive DCTABLE.
        if (c_.syntax == StreamSyntax::Advanced && c_.dquant)
            write_dquant_header(b,dquant_plan);

        std::vector<int> ydc(static_cast<size_t>(ybw)*ybh, 0);
        std::vector<int> udc(static_cast<size_t>(cbw)*cbh, 0);
        std::vector<int> vdc(static_cast<size_t>(cbw)*cbh, 0);
        std::vector<int> yq(static_cast<size_t>(ybw)*ybh,0);
        std::vector<int> uq(static_cast<size_t>(cbw)*cbh,0);
        std::vector<int> vq(static_cast<size_t>(cbw)*cbh,0);
        std::vector<uint8_t> ycoded(static_cast<size_t>(ybw)*ybh, 0);
        bool esc3_lengths_written = false;

        for (int my=0; my<mbh; ++my) {
            for (int mx=0; mx<mbw; ++mx) {
                const size_t mbpos=static_cast<size_t>(my)*mbw+mx;
                const size_t debug_bits_before=(stats && c_.debug_macroblock_stats)?b.bit_count():0;
                const int mq=mb_mquant(mbpos);
                const bool mq_derived=mb_derived(mbpos);
                const int qstate=mq_derived?-mq:mq;
                std::array<BlockCoding,6> blocks;

                // Prepare quantized AC and DC prediction first. ACPRED uses the
                // same left/top direction selected by the VC-1 DC predictor, so
                // the direction must be known before choosing the macroblock's
                // AC-prediction mode and therefore before emitting its CBP VLC.
                for (int k=0; k<6; ++k) {
                    BlockCoding& bc = blocks[static_cast<size_t>(k)];
                    if (k < 4) {
                        bc.bx = mx*2 + (k&1);
                        bc.by = my*2 + ((k>>1)&1);
                        bc.width = c_.width;
                        bc.height = c_.height;
                        bc.plane = &f.y;
                    } else {
                        bc.bx = mx;
                        bc.by = my;
                        bc.width = c_.width/2;
                        bc.height = c_.height/2;
                        bc.plane = (k==4) ? &f.u : &f.v;
                    }
                    if (c_.ac_coding)
                        bc.qac = qblocks[block_index(mbw,mx,my,k)];

                    std::vector<int>* grid = nullptr;
                    std::vector<int>* qgrid = nullptr;
                    int gw=0;
                    if (k < 4) { grid=&ydc; qgrid=&yq; gw=ybw; }
                    else if (k == 4) { grid=&udc; qgrid=&uq; gw=cbw; }
                    else { grid=&vdc; qgrid=&vq; gw=cbw; }

                    bc.dc_target = choose_dc_for_mean(
                        padded_block_mean(*bc.plane,bc.width,bc.height,bc.bx*8,bc.by*8,k>=4?128:0),mq);
                    const bool a_avail = (my != 0) || (k==2 || k==3);
                    const bool c_avail = (mx != 0) || (k==1 || k==3);
                    const int pred = c_.syntax == StreamSyntax::Wmv9Main
                        ? predict_dc_main(*grid, gw, bc.bx, bc.by,
                                          main_dc_pred_base(), &bc.dc_pred_left)
                        : predict_dc_dquant(*grid,*qgrid,gw,bc.bx,bc.by,mq,
                                            a_avail,c_avail,&bc.dc_pred_left);
                    bc.dc_target=clamp_dc_target_for_diff(bc.dc_target,pred,mq);
                    bc.dc_diff = bc.dc_target - pred;
                    const int dc_bits = 8 + ((mq == 1 || mq == 2) ? 3 - mq : 0);
                    if (std::abs(bc.dc_diff) >= (1 << dc_bits))
                        throw std::runtime_error("internal DC differential exceeds VC-1 escape range");
                    const size_t gi=static_cast<size_t>(bc.by)*gw+bc.bx;
                    (*grid)[gi]=bc.dc_target;
                    (*qgrid)[gi]=qstate;
                }

                struct MbAcCandidate {
                    std::array<bool,6> coded{};
                    uint8_t cbp=0;
                    uint64_t bits=std::numeric_limits<uint64_t>::max();
                };

                std::array<const std::array<int,64>*,6> ac_neighbor{};
                std::array<int,6> ac_neighbor_qstate{};
                uint64_t plain_edge_abs=0, pred_edge_abs=0;
                bool pred_valid=c_.ac_coding;
                if (pred_valid) {
                    const int max_level=max_quantized_level();
                    for (int k=0;k<6 && pred_valid;++k) {
                        const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        if (k < 4) {
                            int nbx=bc.bx, nby=bc.by;
                            if (bc.dc_pred_left) --nbx; else --nby;
                            if (nbx >= 0 && nby >= 0) {
                                const int nmx=nbx/2, nmy=nby/2;
                                const int nk=(nby&1)*2+(nbx&1);
                                ac_neighbor[static_cast<size_t>(k)]=
                                    &qblocks[block_index(mbw,nmx,nmy,nk)];
                                const size_t ni=static_cast<size_t>(nby)*ybw+nbx;
                                ac_neighbor_qstate[static_cast<size_t>(k)]=yq[ni];
                            }
                        } else {
                            const int nmx=mx-(bc.dc_pred_left?1:0);
                            const int nmy=my-(bc.dc_pred_left?0:1);
                            if (nmx >= 0 && nmy >= 0) {
                                ac_neighbor[static_cast<size_t>(k)]=
                                    &qblocks[block_index(mbw,nmx,nmy,k)];
                                const auto& qg=(k==4)?uq:vq;
                                ac_neighbor_qstate[static_cast<size_t>(k)]=qg[static_cast<size_t>(nmy)*cbw+nmx];
                            }
                        }
                        const auto* neighbor=ac_neighbor[static_cast<size_t>(k)];
                        if (!neighbor) continue;
                        for (int n=1;n<8;++n) {
                            const int pos=bc.dc_pred_left ? n : n*8;
                            const int original=bc.qac[static_cast<size_t>(pos)];
                            const int predicted=scale_ac_predictor((*neighbor)[static_cast<size_t>(pos)],
                                                                   ac_neighbor_qstate[static_cast<size_t>(k)],qstate);
                            const int v=original-predicted;
                            plain_edge_abs+=static_cast<uint64_t>(std::abs(original));
                            pred_edge_abs+=static_cast<uint64_t>(std::abs(v));
                            if (std::abs(v) > max_level) { pred_valid=false; break; }
                        }
                    }
                }

                // A 25% aggregate edge-magnitude reduction is a cheap gate for
                // useful prediction. It avoids copying/scanning six coefficient
                // blocks at all on the common uncorrelated-texture path.
                const bool pred_plausible=pred_valid && plain_edge_abs>0 &&
                    pred_edge_abs*4u < plain_edge_abs*3u;
                std::array<std::array<int,64>,6> pred_q{};
                if (pred_plausible) {
                    for (int k=0;k<6;++k) {
                        pred_q[static_cast<size_t>(k)]=blocks[static_cast<size_t>(k)].qac;
                        const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        const auto* neighbor=ac_neighbor[static_cast<size_t>(k)];
                        if (!neighbor) continue;
                        for (int n=1;n<8;++n) {
                            const int pos=bc.dc_pred_left ? n : n*8;
                            pred_q[static_cast<size_t>(k)][static_cast<size_t>(pos)] -=
                                scale_ac_predictor((*neighbor)[static_cast<size_t>(pos)],
                                                   ac_neighbor_qstate[static_cast<size_t>(k)],qstate);
                        }
                    }
                }

                // Evaluate coded-block prediction for the requested mode and,
                // only when needed, estimate its VLC length. Strongly correlated
                // edges (>=50% magnitude reduction) choose ACPRED directly;
                // borderline 25..50% cases use the lightweight selected-table
                // estimator so we retain compression efficiency without doing a
                // duplicate BitWriter pass.
                auto evaluate_candidate = [&](bool acpred, bool estimate_bits) {
                    MbAcCandidate cand;
                    std::array<uint8_t,4> saved{};
                    for (int k=0;k<4;++k) {
                        const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        saved[static_cast<size_t>(k)] =
                            ycoded[static_cast<size_t>(bc.by)*ybw+bc.bx];
                    }
                    for (int k=0;k<6;++k) {
                        const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        const auto& q=acpred ? pred_q[static_cast<size_t>(k)] : bc.qac;
                        cand.coded[static_cast<size_t>(k)]=has_ac(q);
                        bool transmitted=cand.coded[static_cast<size_t>(k)];
                        if (k < 4) {
                            const int cp=predict_coded(ycoded,ybw,bc.bx,bc.by);
                            transmitted = cand.coded[static_cast<size_t>(k)] ^ (cp != 0);
                            ycoded[static_cast<size_t>(bc.by)*ybw+bc.bx] =
                                cand.coded[static_cast<size_t>(k)] ? 1 : 0;
                        }
                        if (transmitted)
                            cand.cbp |= static_cast<uint8_t>(1u << (5-k));
                    }
                    for (int k=0;k<4;++k) {
                        const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        ycoded[static_cast<size_t>(bc.by)*ybw+bc.bx] = saved[static_cast<size_t>(k)];
                    }
                    if (estimate_bits) {
                        const VlcCode& mbvlc=kMbIntraVlc[cand.cbp];
                        uint64_t bits=static_cast<uint64_t>(mbvlc.bits)+1u;
                        for (int k=0;k<6;++k) {
                            if (!cand.coded[static_cast<size_t>(k)]) continue;
                            const int set=(k>=4)?c_coding_set:y_coding_set;
                            const bool use_vlc=c_.ac_mode != AcMode::Esc3;
                            const uint8_t* scan=base_intra_scan;
                            if (acpred && (!c_.interlaced() || ac_neighbor[static_cast<size_t>(k)]))
                                scan=blocks[static_cast<size_t>(k)].dc_pred_left
                                    ? kIntraLeftPredScan.data() : kIntraTopPredScan.data();
                            const auto& q=acpred ? pred_q[static_cast<size_t>(k)]
                                                 : blocks[static_cast<size_t>(k)].qac;
                            bits+=estimate_intra_ac_bits(q,set,use_vlc,scan);
                        }
                        cand.bits=bits;
                    }
                    return cand;
                };

                bool use_acpred=false;
                MbAcCandidate chosen;
                if (!pred_plausible) {
                    chosen=evaluate_candidate(false,false);
                } else if (pred_edge_abs*2u < plain_edge_abs) {
                    use_acpred=true;
                    chosen=evaluate_candidate(true,false);
                } else {
                    MbAcCandidate plain=evaluate_candidate(false,true);
                    MbAcCandidate pred=evaluate_candidate(true,true);
                    use_acpred=pred.bits < plain.bits;
                    chosen=use_acpred ? pred : plain;
                }

                // Commit the chosen coded-block state for future luma CBP
                // prediction, then emit the macroblock and its six blocks.
                for (int k=0;k<4;++k) {
                    const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                    ycoded[static_cast<size_t>(bc.by)*ybw+bc.bx] =
                        chosen.coded[static_cast<size_t>(k)] ? 1 : 0;
                }
                const VlcCode& mbvlc = kMbIntraVlc[chosen.cbp];
                b.vlc(mbvlc.code, mbvlc.bits);
                b.bit(use_acpred);
                if (c_.syntax==StreamSyntax::Advanced && overlap_plan.mode==OverlapMode::Select && overlap_plan.bitplane.raw)
                    b.bit(overlap_plan.flags[mbpos]!=0);
                if (use_acpred && stats) ++stats->acpred_macroblocks;
                if (c_.syntax==StreamSyntax::Advanced && dquant_plan.enabled)
                    write_dquant_mb(b,dquant_plan,mbpos);

                for (int k=0; k<6; ++k) {
                    const BlockCoding& bc = blocks[static_cast<size_t>(k)];
                    write_dc_diff(b, bc.dc_diff, k>=4, mq, dc_table_index);
                    if (chosen.coded[static_cast<size_t>(k)]) {
                        const int set=(k>=4)?c_coding_set:y_coding_set;
                        const bool use_vlc=c_.ac_mode != AcMode::Esc3;
                        const uint8_t* scan=base_intra_scan;
                        if (use_acpred && (!c_.interlaced() || ac_neighbor[static_cast<size_t>(k)]))
                            scan=bc.dc_pred_left
                                ? kIntraLeftPredScan.data() : kIntraTopPredScan.data();
                        const auto& q=use_acpred ? pred_q[static_cast<size_t>(k)] : bc.qac;
                        write_ac_block(b,q,k>=4,set,use_vlc,esc3_lengths_written,scan,dquant_plan.enabled);
                    }
                }
                if (stats && c_.debug_macroblock_stats) {
                    auto& d=stats->macroblock_debug[mbpos];
                    d.i_mquant=mq; d.i_dquant_delta=mq-c_.pqindex; d.b_acpred=use_acpred?1:0;
                    d.i_cbp=chosen.cbp; d.i_coded_blocks=0;
                    for (bool x:chosen.coded) if (x) ++d.i_coded_blocks;
                    d.i_local_bits=static_cast<uint64_t>(b.bit_count()-debug_bits_before);
                    d.i_transform_parts[0]=6;
                }
            }
        }
        if (stats && c_.debug_macroblock_stats) {
            const Frame recon=reconstruct_i_picture(f);
            fill_macroblock_quality(stats->macroblock_debug,f,recon,previous_source,nullptr,nullptr);
        }
        return c_.syntax == StreamSyntax::Wmv9Main ? b.finish_raw() : bdu(0x0d,b.finish_rbdu());
    }

Vc1Encoder::SignedFrame Vc1Encoder::reconstruct_i_signed_padded(const Frame& f) const {
        if (f.width != c_.width || f.height != c_.height)
            throw std::runtime_error("I-picture dimensions do not match encoder configuration");
        const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
        const int ybw=mbw*2, ybh=mbh*2;
        const int cbw=mbw, cbh=mbh;
        const size_t mbs=static_cast<size_t>(mbw)*mbh;
        const uint8_t* const base_intra_scan =
            (c_.syntax==StreamSyntax::Advanced && c_.interlaced())
                ? kFrameInterlacedIntraScan.data() : kIntraScan.data();
        std::vector<int> desired_mquant(mbs,c_.pqindex);
        if (c_.syntax==StreamSyntax::Advanced && c_.dquant) {
            for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx)
                desired_mquant[static_cast<size_t>(my)*mbw+mx]=choose_intra_mquant(f,mx,my);
        }
        DQuantPlan dquant_plan;
        dquant_plan.mquant=desired_mquant;
        if (c_.syntax==StreamSyntax::Advanced) dquant_plan=make_dquant_plan(desired_mquant,mbw,mbh);
        SignedFrame signed_frame;
        signed_frame.width=mbw*16; signed_frame.height=mbh*16;
        signed_frame.y.assign(static_cast<size_t>(signed_frame.width)*signed_frame.height,0);
        signed_frame.u.assign(static_cast<size_t>(signed_frame.width/2)*(signed_frame.height/2),0);
        signed_frame.v.assign(static_cast<size_t>(signed_frame.width/2)*(signed_frame.height/2),0);
        std::vector<int> ydc(static_cast<size_t>(ybw)*ybh,0), udc(static_cast<size_t>(cbw)*cbh,0), vdc(static_cast<size_t>(cbw)*cbh,0);
        std::vector<int> yq(static_cast<size_t>(ybw)*ybh,0), uq(static_cast<size_t>(cbw)*cbh,0), vq(static_cast<size_t>(cbw)*cbh,0);
        for (int my=0; my<mbh; ++my) {
            for (int mx=0; mx<mbw; ++mx) {
                for (int k=0; k<6; ++k) {
                    const std::vector<uint8_t>* src=nullptr;
                    std::vector<int>* dst=nullptr;
                    int sw=0,sh=0,dw=0,bx=0,by=0;
                    if (k<4) {
                        src=&f.y; dst=&signed_frame.y;
                        sw=c_.width; sh=c_.height; dw=signed_frame.width;
                        bx=mx*2+(k&1); by=my*2+((k>>1)&1);
                    } else {
                        src=(k==4)?&f.u:&f.v; dst=(k==4)?&signed_frame.u:&signed_frame.v;
                        sw=c_.width/2; sh=c_.height/2; dw=signed_frame.width/2;
                        bx=mx; by=my;
                    }
                    const size_t mbpos=static_cast<size_t>(my)*mbw+mx;
                    const int mq=dquant_plan.mquant.empty()?c_.pqindex:dquant_plan.mquant[mbpos];
                    const bool derived=dquant_mb_derived(dquant_plan,mbpos,mbw);
                    std::array<int,64> q{};
                    if (c_.ac_coding) q=quantize_ac(*src,sw,sh,bx*8,by*8,k>=4,base_intra_scan,mq,derived,k>=4?&f.y:nullptr);
                    std::vector<int>* dcg=nullptr; std::vector<int>* qg=nullptr; int gw=0;
                    if (k<4) { dcg=&ydc; qg=&yq; gw=ybw; }
                    else if (k==4) { dcg=&udc; qg=&uq; gw=cbw; }
                    else { dcg=&vdc; qg=&vq; gw=cbw; }
                    int dc_target=choose_dc_for_mean(padded_block_mean(*src,sw,sh,bx*8,by*8,k>=4?128:0),mq);
                    const bool a_avail=(my!=0)||(k==2||k==3);
                    const bool c_avail=(mx!=0)||(k==1||k==3);
                    const int dc_pred=c_.syntax==StreamSyntax::Wmv9Main
                        ? predict_dc_main(*dcg,gw,bx,by,main_dc_pred_base(),nullptr)
                        : predict_dc_dquant(*dcg,*qg,gw,bx,by,mq,a_avail,c_avail,nullptr);
                    dc_target=clamp_dc_target_for_diff(dc_target,dc_pred,mq);
                    const size_t gi=static_cast<size_t>(by)*gw+bx;
                    (*dcg)[gi]=dc_target; (*qg)[gi]=derived?-mq:mq;
                    std::array<int,64> coeff{};
                    coeff[0]=dc_target*dc_scale(mq);
                    for (int i=1;i<64;++i) coeff[static_cast<size_t>(i)]=dequant_level(q[static_cast<size_t>(i)],mq,derived);
                    inverse_transform_8x8(coeff);
                    for (int y=0;y<8;++y) for (int x=0;x<8;++x)
                        (*dst)[static_cast<size_t>(by*8+y)*dw+bx*8+x]=coeff[static_cast<size_t>(y)*8+x];
                }
            }
        }
        return signed_frame;
    }

Frame Vc1Encoder::reconstruct_i_picture_padded(const Frame& f) const {
        SignedFrame signed_frame=reconstruct_i_signed_padded(f);
        const OverlapPlan plan=choose_i_overlap_plan(f);
        apply_i_overlap(signed_frame,plan);
        const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
        Frame padded=empty_padded_frame(mbw,mbh);
        const int bias=intra_recon_bias();
        auto store=[&](const std::vector<int>& src,std::vector<uint8_t>& dst) {
            for (size_t i=0;i<src.size();++i)
                dst[i]=static_cast<uint8_t>(std::clamp(src[i]+bias,0,255));
        };
        store(signed_frame.y,padded.y); store(signed_frame.u,padded.u); store(signed_frame.v,padded.v);
        if (c_.loop_filter) apply_i_like_loop_filter(padded);
        return padded;
    }

Frame Vc1Encoder::reconstruct_i_picture(const Frame& f) const {
        return crop_padded_frame(reconstruct_i_picture_padded(f));
    }

Frame Vc1Encoder::crop_reconstructed_picture(const Frame& padded) const {
        return crop_padded_frame(padded);
    }

PEncodeResult Vc1Encoder::encode_p_picture(const Frame& f, const Frame& ref, const PAnalysis& a,
                                   const Frame* padded_ref, bool ref_field_picture,
                                   const Frame* previous_source) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::PPicture);
        if (c_.syntax==StreamSyntax::Advanced && c_.interlaced())
            return encode_p_interlaced_fields(f,ref,ref_field_picture,previous_source);
        const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
        const size_t mbs=static_cast<size_t>(mbw)*mbh;
        if (a.mvs.size() != mbs || a.block_mvs.size()!=mbs || a.use_4mv.size()!=mbs || a.use_intra.size()!=mbs)
            throw std::runtime_error("P-picture analysis has the wrong macroblock count");
        if (f.width != c_.width || f.height != c_.height ||
            ref.width != c_.width || ref.height != c_.height)
            throw std::runtime_error("P-picture dimensions do not match encoder configuration");

        const int p_ac_table_index=choose_inter_ac_index(a.mean_abs_residual,c_.pqindex);
        const int p_coding_set=chroma_coding_set(p_ac_table_index,c_.pqindex);
        const int p_intra_luma_set=luma_coding_set(p_ac_table_index,c_.pqindex);
        const int p_decision_index=(c_.pqindex<=8)?1:0; // preserve pre-Group-A transform/trellis quality decisions
        const int p_decision_set=chroma_coding_set(p_decision_index,c_.pqindex);
        const bool use_vlc=c_.ac_mode != AcMode::Esc3;
        const TransformPicturePlan transform_plan=choose_p_transform_plan(f,ref,a,p_decision_set,use_vlc);

        struct PMbSyntax {
            bool skipped=false;
            bool fourmv=false;
            bool intra=false;
            bool intra_acpred=false;
            bool hybrid=false;
            bool hybrid_a=false;
            MotionVector desired{},pred{};
            std::array<MotionVector,4> bdesired{},bpred{};
            std::array<uint8_t,4> bhybrid{},bhybrid_a{};
            uint8_t cbp=0;          // bitstream CBPCY (4-MV luma bits also gate MVDATA)
            uint8_t residual_cbp=0; // actual coefficient-bearing blocks
            int mquant=0;
            MbTransformDecision tx{};
            std::unique_ptr<std::array<std::array<int,64>,6>> intra_q;
            std::array<int,6> intra_dc_diff{};
            BitWriter coeff;
        };
        std::vector<PMbSyntax> syntax(mbs);
        std::vector<uint8_t> skip_plane(mbs,0), cbps;
        std::vector<MvDataSyntax> mv_values;
        cbps.reserve(mbs); mv_values.reserve(mbs);

        PEncodeResult out;
        if (c_.debug_macroblock_stats) out.macroblock_debug.resize(mbs);
        out.frame_level_transform=transform_plan.frame_level;
        out.frame_transform_type=transform_plan.frame_type;
        out.frame_transform_exact_checked=transform_plan.exact_checked;
        out.reconstructed=empty_frame();
        Frame loop_reconstructed=empty_padded_frame(mbw,mbh);
        SignedFrame p_intra_signed;
        p_intra_signed.width=loop_reconstructed.width; p_intra_signed.height=loop_reconstructed.height;
        p_intra_signed.y.assign(loop_reconstructed.y.size(),0);
        p_intra_signed.u.assign(loop_reconstructed.u.size(),0);
        p_intra_signed.v.assign(loop_reconstructed.v.size(),0);
        std::vector<std::array<uint8_t,6>> loop_cbp(mbs);
        std::vector<std::array<uint8_t,6>> loop_tt(mbs);
        std::vector<uint8_t> loop_intra(mbs,0);
        const int ybw=mbw*2,ybh=mbh*2;
        std::vector<int> p_ydc(static_cast<size_t>(ybw)*ybh,0);
        std::vector<int> p_udc(mbs,0),p_vdc(mbs,0);
        std::vector<uint8_t> p_yintra(static_cast<size_t>(ybw)*ybh,0);
        std::vector<uint8_t> p_uintra(mbs,0),p_vintra(mbs,0);
        // Inter-picture ACPRED predicts quantized edge coefficients from an
        // already decoded intra neighbour.  Keep the unpredicted reconstructed
        // coefficient state, exactly as the decoder does after adding ACPRED.
        std::vector<std::array<int,64>> p_yac(static_cast<size_t>(ybw)*ybh);
        std::vector<std::array<int,64>> p_uac(mbs),p_vac(mbs);
        std::vector<int> p_yq(static_cast<size_t>(ybw)*ybh,0),p_uq(mbs,0),p_vq(mbs,0);
        if (c_.track_transform_map) out.transform_map.assign(mbs*4,0);
        const Frame& prediction_ref = (a.intensity.enabled && a.compensated_reference) ? *a.compensated_reference : ref;
        Frame weighted_padded_ref;
        const Frame* loop_prediction_ref=&prediction_ref;
        if (padded_ref) {
            if (a.intensity.enabled) {
                weighted_padded_ref=intensity_compensated_reference(*padded_ref,a.intensity);
                loop_prediction_ref=&weighted_padded_ref;
            } else loop_prediction_ref=padded_ref;
        }

        // Build transform/reconstruction once, while retaining only compact
        // macroblock syntax. This makes picture-level entropy choices exact
        // without a duplicate transform or reconstruction pass.
        for (int my=0; my<mbh; ++my) {
            for (int mx=0; mx<mbw; ++mx) {
                const size_t pos=static_cast<size_t>(my)*mbw+mx;
                PMbSyntax& ms=syntax[pos];
                ms.intra=a.use_intra[pos]!=0;
                ms.fourmv=!ms.intra && a.use_4mv[pos]!=0;
                ms.desired=ms.intra ? MotionVector{} : a.mvs[pos];
                ms.mquant=c_.pqindex;

                if (ms.intra) {
                    ms.intra_q=std::make_unique<std::array<std::array<int,64>,6>>();
                    ms.mquant=choose_intra_mquant(f,mx,my);
                    // Progressive P-intra is signaled by the special MVDATA
                    // symbol.  It carries no motion or TTMB syntax.  DC and AC
                    // prediction may use only already-decoded intra neighbours;
                    // unlike I/BI pictures, inter-picture intra always uses the
                    // normal progressive zig-zag scan even when ACPRED is set.
                    loop_intra[pos]=0x3f;
                    ++out.intra_macroblocks;

                    std::array<BlockCoding,6> blocks{};
                    std::array<std::array<int,64>,6> pred_q{};
                    std::array<const std::array<int,64>*,6> ac_neighbor{};
                    uint64_t plain_edge_abs=0,pred_edge_abs=0;
                    bool pred_valid=c_.ac_coding;

                    for (int k=0;k<6;++k) {
                        BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        std::vector<int>* dcg=nullptr;
                        std::vector<uint8_t>* ing=nullptr;
                        std::vector<std::array<int,64>>* acg=nullptr;
                        std::vector<int>* qg=nullptr;
                        int gw=0;
                        if (k<4) {
                            bc.plane=&f.y; bc.width=c_.width; bc.height=c_.height;
                            bc.bx=mx*2+(k&1); bc.by=my*2+((k>>1)&1);
                            dcg=&p_ydc; ing=&p_yintra; acg=&p_yac; qg=&p_yq; gw=ybw;
                        } else {
                            bc.plane=(k==4)?&f.u:&f.v; bc.width=c_.width/2; bc.height=c_.height/2;
                            bc.bx=mx; bc.by=my; dcg=(k==4)?&p_udc:&p_vdc;
                            ing=(k==4)?&p_uintra:&p_vintra; acg=(k==4)?&p_uac:&p_vac; qg=(k==4)?&p_uq:&p_vq; gw=mbw;
                        }
                        const bool mq_derived=ms.mquant!=c_.pqindex;
                        const int qstate=mq_derived?-ms.mquant:ms.mquant;
                        if (c_.ac_coding)
                            bc.qac=quantize_ac(*bc.plane,bc.width,bc.height,bc.bx*8,bc.by*8,k>=4,kInterScan.data(),ms.mquant,mq_derived,k>=4?&f.y:nullptr);
                        bc.dc_target=choose_inter_intra_dc_for_mean(padded_block_mean(*bc.plane,bc.width,bc.height,bc.bx*8,bc.by*8,k>=4?128:0),ms.mquant);
                        const size_t gi=static_cast<size_t>(bc.by)*gw+bc.bx;
                        const bool a_avail=bc.by>0 && (*ing)[gi-static_cast<size_t>(gw)]!=0;
                        const bool c_avail=bc.bx>0 && (*ing)[gi-1]!=0;
                        const int pred=predict_dc_dquant(*dcg,*qg,gw,bc.bx,bc.by,ms.mquant,a_avail,c_avail,&bc.dc_pred_left);
                        bc.dc_target=clamp_dc_target_for_diff(bc.dc_target,pred,ms.mquant);
                        bc.dc_diff=bc.dc_target-pred;
                        const int dc_bits=8+((ms.mquant==1||ms.mquant==2)?3-ms.mquant:0);
                        if (std::abs(bc.dc_diff)>=(1<<dc_bits))
                            throw std::runtime_error("internal P-intra DC differential exceeds VC-1 escape range");

                        if (pred_valid) {
                            if (bc.dc_pred_left && c_avail) ac_neighbor[static_cast<size_t>(k)]=&(*acg)[gi-1];
                            else if (!bc.dc_pred_left && a_avail) ac_neighbor[static_cast<size_t>(k)]=&(*acg)[gi-static_cast<size_t>(gw)];
                            const auto* nb=ac_neighbor[static_cast<size_t>(k)];
                            if (nb) {
                                for (int n=1;n<8;++n) {
                                    const int ci=bc.dc_pred_left?n:n*8;
                                    const int original=bc.qac[static_cast<size_t>(ci)];
                                    const size_t ni=bc.dc_pred_left?gi-1:gi-static_cast<size_t>(gw);
                                    const int predicted=scale_ac_predictor((*nb)[static_cast<size_t>(ci)],(*qg)[ni],qstate);
                                    const int residual=original-predicted;
                                    plain_edge_abs+=static_cast<uint64_t>(std::abs(original));
                                    pred_edge_abs+=static_cast<uint64_t>(std::abs(residual));
                                    if (std::abs(residual)>max_quantized_level()) { pred_valid=false; break; }
                                }
                            }
                        }
                        // Decoder state stores the reconstructed (unpredicted)
                        // quantized AC coefficients for following intra blocks.
                        (*dcg)[gi]=bc.dc_target; (*ing)[gi]=1; (*acg)[gi]=bc.qac; (*qg)[gi]=qstate;
                    }

                    const bool pred_plausible=pred_valid && plain_edge_abs>0 &&
                        pred_edge_abs*4u<plain_edge_abs*3u;
                    if (pred_plausible) {
                        for (int k=0;k<6;++k) {
                            const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                            pred_q[static_cast<size_t>(k)]=bc.qac;
                            const auto* nb=ac_neighbor[static_cast<size_t>(k)];
                            if (!nb) continue;
                            for (int n=1;n<8;++n) {
                                const int ci=bc.dc_pred_left?n:n*8;
                                std::vector<int>* qg=(k<4)?&p_yq:((k==4)?&p_uq:&p_vq);
                                const int gw=(k<4)?ybw:mbw;
                                const size_t gi=static_cast<size_t>(bc.by)*gw+bc.bx;
                                const size_t ni=bc.dc_pred_left?gi-1:gi-static_cast<size_t>(gw);
                                const int qstate=ms.mquant==c_.pqindex?ms.mquant:-ms.mquant;
                                pred_q[static_cast<size_t>(k)][static_cast<size_t>(ci)]-=
                                    scale_ac_predictor((*nb)[static_cast<size_t>(ci)],(*qg)[ni],qstate);
                            }
                        }
                    }

                    bool use_acpred=false;
                    if (pred_plausible) {
                        if (pred_edge_abs*2u<plain_edge_abs) {
                            use_acpred=true;
                        } else {
                            uint64_t plain_bits=0,pred_bits=0;
                            for (int k=0;k<6;++k) {
                                const int set=k>=4?p_coding_set:p_intra_luma_set;
                                if (has_ac(blocks[static_cast<size_t>(k)].qac))
                                    plain_bits+=estimate_intra_ac_bits(blocks[static_cast<size_t>(k)].qac,set,use_vlc,kInterScan.data());
                                if (has_ac(pred_q[static_cast<size_t>(k)]))
                                    pred_bits+=estimate_intra_ac_bits(pred_q[static_cast<size_t>(k)],set,use_vlc,kInterScan.data());
                            }
                            use_acpred=pred_bits<plain_bits;
                        }
                    }
                    ms.intra_acpred=use_acpred;
                    if (use_acpred) ++out.acpred_macroblocks;

                    uint8_t icbp=0;
                    for (int k=0;k<6;++k) {
                        const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        const auto& transmitted=use_acpred?pred_q[static_cast<size_t>(k)]:bc.qac;
                        if (has_ac(transmitted)) icbp|=static_cast<uint8_t>(1u<<(5-k));
                        ms.intra_dc_diff[static_cast<size_t>(k)]=bc.dc_diff;
                        (*ms.intra_q)[static_cast<size_t>(k)]=transmitted;

                        std::vector<uint8_t>* dstp=nullptr; std::vector<uint8_t>* visp=nullptr;
                        int dw=0,dh=0,vw=0,vh=0;
                        if (k<4) {
                            dstp=&loop_reconstructed.y; visp=&out.reconstructed.y;
                            dw=loop_reconstructed.width; dh=loop_reconstructed.height;
                            vw=out.reconstructed.width; vh=out.reconstructed.height;
                        } else {
                            dstp=(k==4)?&loop_reconstructed.u:&loop_reconstructed.v;
                            visp=(k==4)?&out.reconstructed.u:&out.reconstructed.v;
                            dw=loop_reconstructed.width/2; dh=loop_reconstructed.height/2;
                            vw=out.reconstructed.width/2; vh=out.reconstructed.height/2;
                        }
                        std::array<int,64> coeff{};
                        coeff[0]=bc.dc_target*dc_scale(ms.mquant);
                        for (int i=1;i<64;++i)
                            coeff[static_cast<size_t>(i)]=dequant_level(bc.qac[static_cast<size_t>(i)],ms.mquant,ms.mquant!=c_.pqindex);
                        inverse_transform_8x8(coeff);
                        std::vector<int>* signedp=(k<4)?&p_intra_signed.y:((k==4)?&p_intra_signed.u:&p_intra_signed.v);
                        const int signedw=(k<4)?p_intra_signed.width:p_intra_signed.width/2;
                        for (int yy=0;yy<8;++yy) for (int xx=0;xx<8;++xx)
                            (*signedp)[static_cast<size_t>(bc.by*8+yy)*signedw+bc.bx*8+xx]=coeff[static_cast<size_t>(yy)*8+xx];
                        put_block(*dstp,dw,dh,bc.bx*8,bc.by*8,coeff,128);
                        put_block(*visp,vw,vh,bc.bx*8,bc.by*8,coeff,128);
                    }
                    ms.cbp=ms.residual_cbp=icbp;
                    if (c_.dquant) record_mquant(out,c_.pqindex,ms.mquant);
                    ms.skipped=false; skip_plane[pos]=0; ++out.explicit_macroblocks;
                    mv_values.push_back(mvdata_for_mode(MotionVector{},MotionVector{},a.mv_mode,icbp!=0,true));
                    if (icbp) {
                        cbps.push_back(icbp); ++out.coded_macroblocks;
                        for (int k=0;k<6;++k) if (icbp&(1u<<(5-k))) ++out.coded_blocks;
                    }
                    continue;
                }
                // Non-intra blocks reset the decoder's inter-frame DC/intra state.
                for (int k=0;k<4;++k) {
                    const int bx=mx*2+(k&1),by=my*2+((k>>1)&1);
                    const size_t gi=static_cast<size_t>(by)*ybw+bx; p_ydc[gi]=0; p_yintra[gi]=0;
                }
                p_udc[pos]=p_vdc[pos]=0; p_uintra[pos]=p_vintra[pos]=0;

                if (ms.fourmv) {
                    ms.bdesired=a.block_mvs[pos];
                    for (int k=0;k<4;++k) {
                        const auto pi=predictor_info_4mv(a.block_mvs,a.use_intra,mbw,mbh,mx,my,k);
                        ms.bpred[static_cast<size_t>(k)]=pi.pre;
                        ms.bhybrid[static_cast<size_t>(k)]=pi.hybrid?1:0;
                        if (pi.hybrid) {
                            const int da=modular_mv_distance(ms.bdesired[static_cast<size_t>(k)],pi.a);
                            const int dc=modular_mv_distance(ms.bdesired[static_cast<size_t>(k)],pi.c);
                            const bool use_a=da<=dc;
                            ms.bhybrid_a[static_cast<size_t>(k)]=use_a?1:0;
                            ms.bpred[static_cast<size_t>(k)]=use_a?pi.a:pi.c;
                        }
                    }
                    motion_compensate_4mv(out.reconstructed,prediction_ref,mx,my,ms.bdesired);
                    motion_compensate_4mv_padded(loop_reconstructed,*loop_prediction_ref,mx,my,ms.bdesired);
                    ++out.four_mv_macroblocks;
                } else {
                    const auto pi=predictor_info_mixed_1mv(a.block_mvs,a.use_intra,mbw,mbh,mx,my);
                    ms.pred=pi.pre; ms.hybrid=pi.hybrid;
                    if (pi.hybrid) {
                        const int da=modular_mv_distance(ms.desired,pi.a);
                        const int dc=modular_mv_distance(ms.desired,pi.c);
                        ms.hybrid_a=(da<=dc);
                        ms.pred=ms.hybrid_a?pi.a:pi.c;
                    }
                    motion_compensate_mb(out.reconstructed,prediction_ref,mx,my,ms.desired,a.mv_mode);
                    motion_compensate_mb_padded(loop_reconstructed,*loop_prediction_ref,mx,my,ms.desired,a.mv_mode);
                }

                const MbQuantDecision qd=choose_dquant_transform_mb(f,out.reconstructed,mx,my,p_decision_set,use_vlc,&transform_plan);
                const MbTransformDecision& tx=qd.tx;
                ms.tx=tx;
                ms.mquant=qd.mquant;
                ms.residual_cbp=tx.cbp;
                ms.cbp=tx.cbp;
                if (ms.residual_cbp && c_.dquant)
                    record_mquant(out,c_.pqindex,ms.mquant);
                if (ms.fourmv) {
                    // Luma CBPCY bits also indicate whether an MVDATA symbol is
                    // present. MVDATA's upper half independently says whether
                    // coefficients follow for that luma block.
                    for (int k=0;k<4;++k) {
                        const uint8_t bit=static_cast<uint8_t>(1u<<(5-k));
                        if (!same_mv(ms.bdesired[static_cast<size_t>(k)],ms.bpred[static_cast<size_t>(k)]))
                            ms.cbp|=bit;
                    }
                }
                for (int k=0;k<6;++k) if (ms.residual_cbp&(1u<<(5-k))) {
                    loop_cbp[pos][static_cast<size_t>(k)]=loop_filter_pattern(tx.parents[static_cast<size_t>(k)].skip_mask,tx.parents[static_cast<size_t>(k)].type);
                    loop_tt[pos][static_cast<size_t>(k)]=loop_filter_tt(tx.parents[static_cast<size_t>(k)].type);
                }
                if (ms.fourmv) {
                    bool predictors_only=ms.residual_cbp==0;
                    for (int k=0;k<4;++k) predictors_only &= same_mv(ms.bdesired[static_cast<size_t>(k)],ms.bpred[static_cast<size_t>(k)]);
                    ms.skipped=predictors_only && c_.syntax==StreamSyntax::Advanced;
                } else {
                    ms.skipped=ms.residual_cbp==0 && same_mv(ms.desired,ms.pred) && c_.syntax==StreamSyntax::Advanced;
                }
                skip_plane[pos]=ms.skipped?1:0;
                if (ms.skipped) {
                    ++out.skipped_macroblocks;
                    continue;
                }
                ++out.explicit_macroblocks;
                if (ms.fourmv) {
                    cbps.push_back(ms.cbp);
                    for (int k=0;k<4;++k) {
                        const uint8_t bit=static_cast<uint8_t>(1u<<(5-k));
                        if (!(ms.cbp&bit)) continue;
                        mv_values.push_back(MvDataSyntax{
                            ms.bdesired[static_cast<size_t>(k)].xq-ms.bpred[static_cast<size_t>(k)].xq,
                            ms.bdesired[static_cast<size_t>(k)].yq-ms.bpred[static_cast<size_t>(k)].yq,
                            (ms.residual_cbp&bit)!=0});
                    }
                } else {
                    mv_values.push_back(mvdata_for_mode(ms.desired,ms.pred,a.mv_mode,ms.residual_cbp!=0));
                    if (ms.residual_cbp) cbps.push_back(ms.residual_cbp);
                }
                if (ms.residual_cbp) {
                    ++out.coded_macroblocks;
                    for (int k=0;k<6;++k) {
                        if (!(ms.residual_cbp&(1u<<(5-k)))) continue;
                        ++out.coded_blocks;
                        const auto& pd=tx.parents[static_cast<size_t>(k)];
                        out.transform_parts[static_cast<size_t>(static_cast<uint8_t>(pd.type)-1)] += count_coded_parts(pd,pd.type);
                        int bx=0,by=0;
                        if (k<4) { bx=mx*2+(k&1); by=my*2+((k>>1)&1); }
                        else { bx=mx; by=my; }
                        if (k<4)
                            apply_transform_parent(loop_reconstructed.y,loop_reconstructed.width,loop_reconstructed.height,
                                                   bx*8,by*8,pd,pd.type,ms.mquant,ms.mquant!=c_.pqindex);
                        else
                            apply_transform_parent(k==4?loop_reconstructed.u:loop_reconstructed.v,
                                                   loop_reconstructed.width/2,loop_reconstructed.height/2,
                                                   bx*8,by*8,pd,pd.type,ms.mquant,ms.mquant!=c_.pqindex);
                        if (c_.track_transform_map && k<4)
                            out.transform_map[(pos*4)+static_cast<size_t>(k)]=pack_transform_map(pd.type,pd.skip_mask);
                    }
                }
            }
        }

        std::vector<int> desired_mquant(mbs,c_.pqindex);
        for (size_t pos=0;pos<mbs;++pos) {
            const PMbSyntax& ms=syntax[pos];
            if (ms.intra || ms.residual_cbp) desired_mquant[pos]=ms.mquant;
        }
        const DQuantPlan dquant_plan=make_dquant_plan(desired_mquant,mbw,mbh);
        const bool dquantfrm=dquant_plan.enabled;
        bool esc3_lengths_written=false;
        for (size_t pos=0;pos<mbs;++pos) {
            PMbSyntax& ms=syntax[pos];
            if (ms.intra) {
                for (int k=0;k<6;++k) {
                    write_dc_diff(ms.coeff,ms.intra_dc_diff[static_cast<size_t>(k)],k>=4,ms.mquant,0);
                    if (ms.residual_cbp&(1u<<(5-k)))
                        write_ac_block(ms.coeff,(*ms.intra_q)[static_cast<size_t>(k)],k>=4,
                                       k>=4?p_coding_set:p_intra_luma_set,use_vlc,
                                       esc3_lengths_written,kInterScan.data(),dquantfrm);
                }
            } else if (ms.residual_cbp) {
                write_transform_payload(ms.coeff,ms.tx,p_coding_set,use_vlc,esc3_lengths_written,false,dquantfrm);
            }
        }

        std::vector<uint8_t> mvtype_plane(mbs,0);
        const bool mixed_mv=mv_mode_mixed(a.mv_mode);
        for (size_t pos=0;pos<mbs;++pos) mvtype_plane[pos]=syntax[pos].fourmv?1:0;
        BitplaneChoice mvtype_choice;
        if (mixed_mv) mvtype_choice=choose_bitplane(mvtype_plane,mbw,mbh);
        const BitplaneChoice skip_choice=choose_bitplane(skip_plane,mbw,mbh);
        const int mv_table=choose_mv_table(mv_values);
        const int cbp_table=choose_cbp_table(cbps);

        if (c_.debug_macroblock_stats) {
            for (size_t pos=0;pos<mbs;++pos) {
                const PMbSyntax& ms=syntax[pos]; auto& d=out.macroblock_debug[pos];
                d.i_picture_q=c_.pqindex; d.i_mquant=ms.mquant; d.i_dquant_delta=ms.mquant-c_.pqindex; d.i_field_index=d.i_field_parity=-1;
                d.f_inter_intra_cost_ratio=pos<a.inter_intra_cost_ratio.size()?a.inter_intra_cost_ratio[pos]:-1.0;
                d.f_forward_distant_local_mae_y=pos<a.distant_local_mae.size()?a.distant_local_mae[pos]:-1.0;
                d.f_forward_distant_candidate_mae_y=pos<a.distant_candidate_mae.size()?a.distant_candidate_mae[pos]:-1.0;
                d.i_forward_distant_match_decision=pos<a.distant_match_decision.size()?static_cast<vc1_distant_match_decision_e>(a.distant_match_decision[pos]):VC1_DISTANT_MATCH_NONE;
                d.f_backward_distant_local_mae_y=d.f_backward_distant_candidate_mae_y=-1.0;
                d.i_backward_distant_match_decision=VC1_DISTANT_MATCH_NONE;
                d.i_forward_reference_display_order=d.i_backward_reference_display_order=-1;
                d.b_skipped=ms.skipped?1:0; d.b_intra=ms.intra?1:0; d.b_four_mv=ms.fourmv?1:0; d.b_acpred=ms.intra_acpred?1:0;
                d.i_mode=ms.intra?VC1_MB_DEBUG_P_INTRA:(ms.fourmv?VC1_MB_DEBUG_P_4MV:(ms.skipped?VC1_MB_DEBUG_P_SKIPPED:VC1_MB_DEBUG_P_INTER));
                d.i_cbp=ms.residual_cbp; d.i_coded_blocks=0; for(int k=0;k<6;++k) if(ms.residual_cbp&(1u<<(5-k))) ++d.i_coded_blocks;
                if (!ms.intra) {
                    if (ms.fourmv) { d.i_forward_mv_count=4; for(int k=0;k<4;++k){d.i_forward_mv_xq[k]=ms.bdesired[k].xq;d.i_forward_mv_yq[k]=ms.bdesired[k].yq;} }
                    else { d.i_forward_mv_count=1; d.i_forward_mv_xq[0]=ms.desired.xq; d.i_forward_mv_yq[0]=ms.desired.yq; }
                    d.i_estimated_transform_bits=ms.tx.estimated_bits;
                    for(int k=0;k<6;++k) if(ms.residual_cbp&(1u<<(5-k))) { const auto& pd=ms.tx.parents[k]; d.i_transform_parts[static_cast<size_t>(static_cast<uint8_t>(pd.type)-1)]+=static_cast<uint32_t>(count_coded_parts(pd,pd.type)); }
                } else d.i_estimated_transform_bits=ms.coeff.bit_count();
            }
        }

        BitWriter b;
        if (c_.syntax == StreamSyntax::Wmv9Main) {
            b.bits(0,2); b.bit(true); // FRMCNT, PTYPE=P
        } else {
            if (c_.interlaced()) b.bit(false); // FCM=0.
            b.bit(false); // Advanced PTYPE=P.
            if (c_.interlaced()) { b.bit(c_.top_field_first()); b.bit(false); } // TFF, RFF=0.
            b.bit(false); // RNDCTRL=0.
            if (c_.interlaced()) b.bit(false); // UVSAMP=0.
        }
        b.bits(static_cast<uint64_t>(c_.pqindex),5);
        if (c_.pqindex <= 8) b.bit(c_.halfqp);
        b.bit(uniform_quantizer()); // PQUANTIZER (QUANTIZER=frame-explicit).
        if (extended_mv_enabled()) write_mvrange(b);
        if (a.intensity.enabled) {
            write_unary_mode(b,3,4); // MVMODE=INTENSITY_COMP
            write_unary_mode(b,p_mv_mode_index(a.mv_mode,c_.pqindex,true),3);
            b.bits(a.intensity.lumscale,6); b.bits(a.intensity.lumshift,6);
        } else {
            write_unary_mode(b,p_mv_mode_index(a.mv_mode,c_.pqindex,false),4);
        }
        if (mixed_mv) b.append(mvtype_choice.syntax);
        b.append(skip_choice.syntax);
        b.bits(static_cast<uint64_t>(mv_table),2);
        b.bits(static_cast<uint64_t>(cbp_table),2);
        if (c_.dquant) write_dquant_header(b,dquant_plan);
        if (c_.variable_transforms) {
            b.bit(transform_plan.frame_level); // TTMBF
            if (transform_plan.frame_level)
                b.bits(static_cast<uint64_t>(static_cast<uint8_t>(transform_plan.frame_type)-1),2); // TTFRM
        }
        write_decode012(b,p_ac_table_index);
        b.bit(false); // TRANSDCTAB=0

        for (size_t pos=0;pos<mbs;++pos) {
            const size_t debug_bits_before=c_.debug_macroblock_stats?b.bit_count():0;
            const PMbSyntax& ms=syntax[pos];
            if (mixed_mv && mvtype_choice.raw) b.bit(ms.fourmv);
            if (skip_choice.raw) b.bit(ms.skipped);
            if (ms.fourmv) {
                if (ms.skipped) {
                    // Skipped 4-MV macroblocks still run four decoder predictors,
                    // and HYBRIDPRED belongs to those predictors rather than MVDATA.
                    for (int k=0;k<4;++k) if (ms.bhybrid[static_cast<size_t>(k)])
                        b.bit(ms.bhybrid_a[static_cast<size_t>(k)]!=0);
                    if (c_.debug_macroblock_stats) out.macroblock_debug[pos].i_local_bits=static_cast<uint64_t>(b.bit_count()-debug_bits_before);
                    continue;
                }
                write_cbp_table(b,cbp_table,ms.cbp);
                for (int k=0;k<4;++k) {
                    const uint8_t bit=static_cast<uint8_t>(1u<<(5-k));
                    if (ms.cbp&bit) {
                        write_mvdata_table(b,mv_table,MvDataSyntax{
                            ms.bdesired[static_cast<size_t>(k)].xq-ms.bpred[static_cast<size_t>(k)].xq,
                            ms.bdesired[static_cast<size_t>(k)].yq-ms.bpred[static_cast<size_t>(k)].yq,
                            (ms.residual_cbp&bit)!=0});
                    }
                    // ff_vc1_pred_mv() runs for every luma block in a 4-MV MB,
                    // even when CBPCY omits MVDATA for that block. HYBRIDPRED is
                    // therefore independent of the per-block MVDATA-presence bit.
                    if (ms.bhybrid[static_cast<size_t>(k)])
                        b.bit(ms.bhybrid_a[static_cast<size_t>(k)]!=0);
                }
                if (ms.residual_cbp) {
                    if (dquantfrm) write_dquant_mb(b,dquant_plan,pos);
                    b.append(ms.coeff);
                }
                if (c_.debug_macroblock_stats) out.macroblock_debug[pos].i_local_bits=static_cast<uint64_t>(b.bit_count()-debug_bits_before);
                continue;
            }
            if (ms.skipped) {
                if (ms.hybrid) b.bit(ms.hybrid_a);
                if (c_.debug_macroblock_stats) out.macroblock_debug[pos].i_local_bits=static_cast<uint64_t>(b.bit_count()-debug_bits_before);
                continue;
            }
            if (ms.intra) {
                write_mvdata_table(b,mv_table,MvDataSyntax{0,0,ms.residual_cbp!=0,true});
                if (!ms.residual_cbp && dquantfrm)
                    write_dquant_mb(b,dquant_plan,pos);
                b.bit(ms.intra_acpred); // Inter-picture ACPRED; scan remains kInterScan.
                if (ms.residual_cbp) {
                    write_cbp_table(b,cbp_table,ms.residual_cbp);
                    if (dquantfrm) write_dquant_mb(b,dquant_plan,pos);
                }
                b.append(ms.coeff);
                if (c_.debug_macroblock_stats) out.macroblock_debug[pos].i_local_bits=static_cast<uint64_t>(b.bit_count()-debug_bits_before);
                continue;
            }
            const MvDataSyntax mv=mvdata_for_mode(ms.desired,ms.pred,a.mv_mode,ms.residual_cbp!=0);
            write_mvdata_table(b,mv_table,mv);
            if (ms.hybrid) b.bit(ms.hybrid_a);
            if (ms.residual_cbp) {
                write_cbp_table(b,cbp_table,ms.residual_cbp);
                if (dquantfrm) write_dquant_mb(b,dquant_plan,pos);
                b.append(ms.coeff);
            }
            if (c_.debug_macroblock_stats) out.macroblock_debug[pos].i_local_bits=static_cast<uint64_t>(b.bit_count()-debug_bits_before);
        }

        if (c_.overlap && c_.pqindex>=9) {
            apply_p_overlap(p_intra_signed,loop_intra,mbw,mbh);
            // Only intra blocks participate in P-picture OVERLAP. Copy their
            // full-precision filtered samples into the already-built inter
            // reconstruction before the deblocking stage.
            for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
                const size_t pos=static_cast<size_t>(my)*mbw+mx;
                if (!loop_intra[pos]) continue;
                for (int k=0;k<6;++k) {
                    const std::vector<int>* srcp=nullptr; std::vector<uint8_t>* dstp=nullptr;
                    int sw=0,bx=0,by=0;
                    if (k<4) {
                        srcp=&p_intra_signed.y; dstp=&loop_reconstructed.y; sw=p_intra_signed.width;
                        bx=mx*2+(k&1); by=my*2+((k>>1)&1);
                    } else {
                        srcp=(k==4)?&p_intra_signed.u:&p_intra_signed.v;
                        dstp=(k==4)?&loop_reconstructed.u:&loop_reconstructed.v;
                        sw=p_intra_signed.width/2; bx=mx; by=my;
                    }
                    for (int yy=0;yy<8;++yy) for (int xx=0;xx<8;++xx) {
                        const size_t o=static_cast<size_t>(by*8+yy)*sw+bx*8+xx;
                        (*dstp)[o]=static_cast<uint8_t>(std::clamp((*srcp)[o]+128,0,255));
                    }
                }
            }
        }
        if (c_.loop_filter)
            apply_p_loop_filter(loop_reconstructed,a.block_mvs,loop_cbp,loop_tt,loop_intra,mbw,mbh);
        out.padded_reconstructed=loop_reconstructed;
        out.reconstructed=crop_padded_frame(loop_reconstructed);
        if (c_.debug_macroblock_stats) {
            Frame pred=empty_frame(); std::vector<uint8_t> pred_valid(mbs,0);
            for(int my=0;my<mbh;++my)for(int mx=0;mx<mbw;++mx){const size_t pos=static_cast<size_t>(my)*mbw+mx;const PMbSyntax& ms=syntax[pos];if(ms.intra)continue;pred_valid[pos]=1;if(ms.fourmv)motion_compensate_4mv(pred,prediction_ref,mx,my,ms.bdesired);else motion_compensate_mb(pred,prediction_ref,mx,my,ms.desired,a.mv_mode);}
            fill_macroblock_quality(out.macroblock_debug,f,out.reconstructed,previous_source,&pred,&pred_valid);
        }
        out.data = c_.syntax == StreamSyntax::Wmv9Main ? b.finish_raw() : bdu(0x0d,b.finish_rbdu());
        return out;
    }

BEncodeResult Vc1Encoder::encode_b_picture(const Frame& f, const Frame& past, const Frame& future,
                                   const BAnalysis& a, int fraction_num, int fraction_den,
                                   const Frame* padded_past, const Frame* padded_future,
                                   bool past_field_picture, bool future_field_picture,
                                   const Frame* previous_source) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::BPicture);
        if (c_.syntax==StreamSyntax::Advanced && c_.interlaced())
            return encode_b_interlaced_fields(f,past,future,fraction_num,fraction_den,
                                               past_field_picture,future_field_picture,previous_source);
        const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
        const size_t mbs=static_cast<size_t>(mbw)*mbh;
        if (a.modes.size()!=mbs || a.forward_mvs.size()!=mbs || a.backward_mvs.size()!=mbs)
            throw std::runtime_error("B-picture analysis has the wrong macroblock count");
        if (fraction_den < 2 || fraction_num <= 0 || fraction_num >= fraction_den)
            throw std::runtime_error("invalid B-picture temporal fraction");
        if (!((fraction_den==2 && fraction_num==1) ||
              (fraction_den==3 && (fraction_num==1 || fraction_num==2))))
            throw std::runtime_error("this release supports B fractions 1/2, 1/3, and 2/3");

        const int ac_table_index=choose_inter_ac_index(a.mean_abs_residual,c_.pqindex);
        const int coding_set=chroma_coding_set(ac_table_index,c_.pqindex);
        const int intra_luma_set=luma_coding_set(ac_table_index,c_.pqindex);
        const int decision_index=(c_.pqindex<=8)?1:0;
        const int decision_set=chroma_coding_set(decision_index,c_.pqindex);
        const bool use_vlc=c_.ac_mode != AcMode::Esc3;
        const TransformPicturePlan transform_plan=choose_b_transform_plan(f,past,future,a,decision_set,use_vlc);
        const bool below_half=(2*fraction_num < fraction_den);

        struct BMbSyntax {
            BMbMode mode=BMbMode::Forward;
            MotionVector fmv{},bmv{},fpred{},bpred{};
            bool direct=false,skipped=false;
            bool intra_acpred=false;
            uint8_t cbp=0;
            int mquant=0;
            MbTransformDecision tx{};
            std::unique_ptr<std::array<std::array<int,64>,6>> intra_q;
            std::array<int,6> intra_dc_diff{};
            BitWriter coeff;
        };
        std::vector<BMbSyntax> syntax(mbs);
        std::vector<uint8_t> direct_plane(mbs,0),skip_plane(mbs,0),cbps;
        std::vector<MvDataSyntax> mv_values;
        cbps.reserve(mbs); mv_values.reserve(mbs*2);

        BEncodeResult out;
        if (c_.debug_macroblock_stats) out.macroblock_debug.resize(mbs);
        out.frame_level_transform=transform_plan.frame_level;
        out.frame_transform_type=transform_plan.frame_type;
        out.frame_transform_exact_checked=transform_plan.exact_checked;
        out.reconstructed=empty_frame();
        Frame loop_reconstructed=empty_padded_frame(mbw,mbh);
        const int ybw=mbw*2,ybh=mbh*2;
        std::vector<int> b_ydc(static_cast<size_t>(ybw)*ybh,0);
        std::vector<int> b_udc(mbs,0),b_vdc(mbs,0);
        std::vector<uint8_t> b_yintra(static_cast<size_t>(ybw)*ybh,0);
        std::vector<uint8_t> b_uintra(mbs,0),b_vintra(mbs,0);
        std::vector<std::array<int,64>> b_yac(static_cast<size_t>(ybw)*ybh);
        std::vector<std::array<int,64>> b_uac(mbs),b_vac(mbs);
        std::vector<int> b_yq(static_cast<size_t>(ybw)*ybh,0),b_uq(mbs,0),b_vq(mbs,0);
        if (c_.track_transform_map) out.transform_map.assign(mbs*4,0);
        const Frame& forward_ref=(a.forward_intensity.enabled && a.compensated_past) ? *a.compensated_past : past;
        Frame weighted_padded_past;
        const Frame* loop_forward_ref=&forward_ref;
        if (padded_past) {
            if (a.forward_intensity.enabled) {
                weighted_padded_past=intensity_compensated_reference(*padded_past,a.forward_intensity);
                loop_forward_ref=&weighted_padded_past;
            } else loop_forward_ref=padded_past;
        }
        const Frame* loop_future_ref=padded_future?padded_future:&future;
        out.forward_macroblocks=a.forward_macroblocks;
        out.backward_macroblocks=a.backward_macroblocks;
        out.interpolated_macroblocks=a.interpolated_macroblocks;
        out.direct_macroblocks=a.direct_macroblocks;
        out.intra_macroblocks=a.intra_macroblocks;

        for (int my=0; my<mbh; ++my) {
            for (int mx=0; mx<mbw; ++mx) {
                const size_t pos=static_cast<size_t>(my)*mbw+mx;
                BMbSyntax& ms=syntax[pos];
                ms.mode=a.modes[pos]; ms.fmv=a.forward_mvs[pos]; ms.bmv=a.backward_mvs[pos]; ms.mquant=c_.pqindex;
                ms.fpred=c_.syntax==StreamSyntax::Wmv9Main
                    ? predictor_b_main(a.forward_mvs,mbw,mx,my)
                    : predictor_info(a.forward_mvs,mbw,mbh,mx,my).pre;
                ms.bpred=c_.syntax==StreamSyntax::Wmv9Main
                    ? predictor_b_main(a.backward_mvs,mbw,mx,my)
                    : predictor_info(a.backward_mvs,mbw,mbh,mx,my).pre;

                if (ms.mode==BMbMode::Intra) {
                    ms.intra_q=std::make_unique<std::array<std::array<int,64>,6>>();
                    ms.mquant=choose_intra_mquant(f,mx,my);
                    // A B-intra macroblock is coded with the inter-picture intra
                    // syntax: special MVDATA intra symbol, ACPRED, CBPCY, then
                    // six intra DC/AC blocks using the normal inter scan.
                    std::array<BlockCoding,6> blocks{};
                    std::array<std::array<int,64>,6> pred_q{};
                    std::array<const std::array<int,64>*,6> ac_neighbor{};
                    uint64_t plain_edge_abs=0,pred_edge_abs=0;
                    bool pred_valid=c_.ac_coding;
                    for (int k=0;k<6;++k) {
                        BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        std::vector<int>* dcg=nullptr; std::vector<uint8_t>* ing=nullptr;
                        std::vector<std::array<int,64>>* acg=nullptr; std::vector<int>* qg=nullptr; int gw=0;
                        if (k<4) {
                            bc.plane=&f.y; bc.width=c_.width; bc.height=c_.height;
                            bc.bx=mx*2+(k&1); bc.by=my*2+((k>>1)&1);
                            dcg=&b_ydc; ing=&b_yintra; acg=&b_yac; qg=&b_yq; gw=ybw;
                        } else {
                            bc.plane=(k==4)?&f.u:&f.v; bc.width=c_.width/2; bc.height=c_.height/2;
                            bc.bx=mx; bc.by=my; dcg=(k==4)?&b_udc:&b_vdc;
                            ing=(k==4)?&b_uintra:&b_vintra; acg=(k==4)?&b_uac:&b_vac; qg=(k==4)?&b_uq:&b_vq; gw=mbw;
                        }
                        const bool mq_derived=ms.mquant!=c_.pqindex;
                        const int qstate=mq_derived?-ms.mquant:ms.mquant;
                        if (c_.ac_coding)
                            bc.qac=quantize_ac(*bc.plane,bc.width,bc.height,bc.bx*8,bc.by*8,k>=4,kInterScan.data(),ms.mquant,mq_derived,k>=4?&f.y:nullptr);
                        bc.dc_target=choose_inter_intra_dc_for_mean(padded_block_mean(*bc.plane,bc.width,bc.height,bc.bx*8,bc.by*8,k>=4?128:0),ms.mquant);
                        const size_t gi=static_cast<size_t>(bc.by)*gw+bc.bx;
                        const bool a_avail=bc.by>0 && (*ing)[gi-static_cast<size_t>(gw)]!=0;
                        const bool c_avail=bc.bx>0 && (*ing)[gi-1]!=0;
                        const int pred=predict_dc_dquant(*dcg,*qg,gw,bc.bx,bc.by,ms.mquant,a_avail,c_avail,&bc.dc_pred_left);
                        bc.dc_target=clamp_dc_target_for_diff(bc.dc_target,pred,ms.mquant);
                        bc.dc_diff=bc.dc_target-pred;
                        const int dc_bits=8+((ms.mquant==1||ms.mquant==2)?3-ms.mquant:0);
                        if (std::abs(bc.dc_diff)>=(1<<dc_bits))
                            throw std::runtime_error("internal B-intra DC differential exceeds VC-1 escape range");
                        if (pred_valid) {
                            if (bc.dc_pred_left && c_avail) ac_neighbor[static_cast<size_t>(k)]=&(*acg)[gi-1];
                            else if (!bc.dc_pred_left && a_avail) ac_neighbor[static_cast<size_t>(k)]=&(*acg)[gi-static_cast<size_t>(gw)];
                            const auto* nb=ac_neighbor[static_cast<size_t>(k)];
                            if (nb) for (int n=1;n<8;++n) {
                                const int ci=bc.dc_pred_left?n:n*8;
                                const int original=bc.qac[static_cast<size_t>(ci)];
                                const size_t ni=bc.dc_pred_left?gi-1:gi-static_cast<size_t>(gw);
                                const int predicted=scale_ac_predictor((*nb)[static_cast<size_t>(ci)],(*qg)[ni],qstate);
                                const int residual=original-predicted;
                                plain_edge_abs+=static_cast<uint64_t>(std::abs(original));
                                pred_edge_abs+=static_cast<uint64_t>(std::abs(residual));
                                if (std::abs(residual)>max_quantized_level()) { pred_valid=false; break; }
                            }
                        }
                        // Match the decoder's quantizer-aware intra predictor state.
                        // Without qstate here, a following B-intra block beside a
                        // DQUANT boundary scales DC/AC predictors as if its neighbour
                        // used the current picture quantizer, causing reconstruction drift.
                        (*dcg)[gi]=bc.dc_target; (*ing)[gi]=1; (*acg)[gi]=bc.qac; (*qg)[gi]=qstate;
                    }
                    const bool pred_plausible=pred_valid && plain_edge_abs>0 &&
                        pred_edge_abs*4u<plain_edge_abs*3u;
                    if (pred_plausible) for (int k=0;k<6;++k) {
                        const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        pred_q[static_cast<size_t>(k)]=bc.qac;
                        const auto* nb=ac_neighbor[static_cast<size_t>(k)];
                        if (!nb) continue;
                        for (int n=1;n<8;++n) {
                            const int ci=bc.dc_pred_left?n:n*8;
                            std::vector<int>* qg=(k<4)?&b_yq:((k==4)?&b_uq:&b_vq);
                            const int gw=(k<4)?ybw:mbw;
                            const size_t gi=static_cast<size_t>(bc.by)*gw+bc.bx;
                            const size_t ni=bc.dc_pred_left?gi-1:gi-static_cast<size_t>(gw);
                            const int qstate=ms.mquant==c_.pqindex?ms.mquant:-ms.mquant;
                            pred_q[static_cast<size_t>(k)][static_cast<size_t>(ci)]-=
                                scale_ac_predictor((*nb)[static_cast<size_t>(ci)],(*qg)[ni],qstate);
                        }
                    }
                    if (pred_plausible) {
                        if (pred_edge_abs*2u<plain_edge_abs) ms.intra_acpred=true;
                        else {
                            uint64_t plain_bits=0,pred_bits=0;
                            for (int k=0;k<6;++k) {
                                const int set=k>=4?coding_set:intra_luma_set;
                                if (has_ac(blocks[static_cast<size_t>(k)].qac))
                                    plain_bits+=estimate_intra_ac_bits(blocks[static_cast<size_t>(k)].qac,set,use_vlc,kInterScan.data());
                                if (has_ac(pred_q[static_cast<size_t>(k)]))
                                    pred_bits+=estimate_intra_ac_bits(pred_q[static_cast<size_t>(k)],set,use_vlc,kInterScan.data());
                            }
                            ms.intra_acpred=pred_bits<plain_bits;
                        }
                    }
                    if (ms.intra_acpred) ++out.acpred_macroblocks;
                    for (int k=0;k<6;++k) {
                        const BlockCoding& bc=blocks[static_cast<size_t>(k)];
                        const auto& transmitted=ms.intra_acpred?pred_q[static_cast<size_t>(k)]:bc.qac;
                        if (has_ac(transmitted)) ms.cbp|=static_cast<uint8_t>(1u<<(5-k));
                        (*ms.intra_q)[static_cast<size_t>(k)]=transmitted;
                        ms.intra_dc_diff[static_cast<size_t>(k)]=bc.dc_diff;
                        std::vector<uint8_t>* dstp=nullptr; std::vector<uint8_t>* visp=nullptr;
                        int dw=0,dh=0,vw=0,vh=0;
                        if (k<4) { dstp=&loop_reconstructed.y; visp=&out.reconstructed.y; dw=loop_reconstructed.width; dh=loop_reconstructed.height; vw=out.reconstructed.width; vh=out.reconstructed.height; }
                        else { dstp=(k==4)?&loop_reconstructed.u:&loop_reconstructed.v; visp=(k==4)?&out.reconstructed.u:&out.reconstructed.v; dw=loop_reconstructed.width/2; dh=loop_reconstructed.height/2; vw=out.reconstructed.width/2; vh=out.reconstructed.height/2; }
                        std::array<int,64> coeff{}; coeff[0]=bc.dc_target*dc_scale(ms.mquant);
                        for (int i=1;i<64;++i) coeff[static_cast<size_t>(i)]=dequant_level(bc.qac[static_cast<size_t>(i)],ms.mquant,ms.mquant!=c_.pqindex);
                        inverse_transform_8x8(coeff);
                        put_block(*dstp,dw,dh,bc.bx*8,bc.by*8,coeff,128);
                        put_block(*visp,vw,vh,bc.bx*8,bc.by*8,coeff,128);
                    }
                    if (c_.dquant) record_mquant(out,c_.pqindex,ms.mquant);
                    ms.direct=false; ms.skipped=false; direct_plane[pos]=0; skip_plane[pos]=0;
                    ++out.explicit_macroblocks;
                    mv_values.push_back(mvdata_for_mode(MotionVector{},MotionVector{},a.mv_mode,ms.cbp!=0,true));
                    if (ms.cbp) {
                        cbps.push_back(ms.cbp); ++out.coded_macroblocks;
                        for (int k=0;k<6;++k) if (ms.cbp&(1u<<(5-k))) ++out.coded_blocks;
                    }
                    continue;
                }

                // Non-intra B macroblocks clear the inter-picture intra/DC state.
                for (int k=0;k<4;++k) {
                    const int bx=mx*2+(k&1),by=my*2+((k>>1)&1);
                    const size_t gi=static_cast<size_t>(by)*ybw+bx; b_ydc[gi]=0; b_yintra[gi]=0;
                }
                b_udc[pos]=b_vdc[pos]=0; b_uintra[pos]=b_vintra[pos]=0;

                if (ms.mode==BMbMode::Backward) {
                    motion_compensate_mb(out.reconstructed,future,mx,my,ms.bmv,a.mv_mode);
                    motion_compensate_mb_padded(loop_reconstructed,*loop_future_ref,mx,my,ms.bmv,a.mv_mode);
                } else if (ms.mode==BMbMode::Forward) {
                    motion_compensate_mb(out.reconstructed,forward_ref,mx,my,ms.fmv,a.mv_mode);
                    motion_compensate_mb_padded(loop_reconstructed,*loop_forward_ref,mx,my,ms.fmv,a.mv_mode);
                } else {
                    motion_compensate_bi_mb(out.reconstructed,forward_ref,future,mx,my,ms.fmv,ms.bmv,a.mv_mode);
                    motion_compensate_bi_mb_padded(loop_reconstructed,*loop_forward_ref,*loop_future_ref,mx,my,ms.fmv,ms.bmv,a.mv_mode);
                }

                const MbQuantDecision qd=choose_dquant_transform_mb(f,out.reconstructed,mx,my,decision_set,use_vlc,&transform_plan);
                const MbTransformDecision& tx=qd.tx;
                ms.tx=tx;
                ms.mquant=qd.mquant;
                ms.cbp=tx.cbp;
                if (ms.cbp && c_.dquant)
                    record_mquant(out,c_.pqindex,ms.mquant);
                ms.direct=ms.mode==BMbMode::Direct;
                direct_plane[pos]=ms.direct?1:0;
                if (ms.direct) ms.skipped=(ms.cbp==0);
                else if (c_.syntax != StreamSyntax::Wmv9Main) {
                    if (ms.mode==BMbMode::Interpolated)
                        ms.skipped=ms.cbp==0 && same_mv(ms.fmv,ms.fpred) && same_mv(ms.bmv,ms.bpred);
                    else if (ms.mode==BMbMode::Forward)
                        ms.skipped=ms.cbp==0 && same_mv(ms.fmv,ms.fpred);
                    else
                        ms.skipped=ms.cbp==0 && same_mv(ms.bmv,ms.bpred);
                }
                skip_plane[pos]=ms.skipped?1:0;
                if (ms.skipped) {
                    ++out.skipped_macroblocks;
                    continue;
                }
                ++out.explicit_macroblocks;
                if (!ms.direct) {
                    if (ms.mode==BMbMode::Interpolated) {
                        mv_values.push_back(mvdata_for_mode(ms.bmv,ms.bpred,a.mv_mode,true));
                        mv_values.push_back(mvdata_for_mode(ms.fmv,ms.fpred,a.mv_mode,ms.cbp!=0));
                    } else {
                        const MotionVector d=ms.mode==BMbMode::Forward?ms.fmv:ms.bmv;
                        const MotionVector p=ms.mode==BMbMode::Forward?ms.fpred:ms.bpred;
                        mv_values.push_back(mvdata_for_mode(d,p,a.mv_mode,ms.cbp!=0));
                    }
                }
                if (ms.cbp) {
                    cbps.push_back(ms.cbp); ++out.coded_macroblocks;
                    for (int k=0;k<6;++k) {
                        if (!(ms.cbp&(1u<<(5-k)))) continue;
                        ++out.coded_blocks;
                        const auto& pd=tx.parents[static_cast<size_t>(k)];
                        out.transform_parts[static_cast<size_t>(static_cast<uint8_t>(pd.type)-1)] += count_coded_parts(pd,pd.type);
                        int bx=0,by=0;
                        if (k<4) { bx=mx*2+(k&1); by=my*2+((k>>1)&1); }
                        else { bx=mx; by=my; }
                        if (k<4)
                            apply_transform_parent(loop_reconstructed.y,loop_reconstructed.width,loop_reconstructed.height,
                                                   bx*8,by*8,pd,pd.type,ms.mquant,ms.mquant!=c_.pqindex);
                        else
                            apply_transform_parent(k==4?loop_reconstructed.u:loop_reconstructed.v,
                                                   loop_reconstructed.width/2,loop_reconstructed.height/2,
                                                   bx*8,by*8,pd,pd.type,ms.mquant,ms.mquant!=c_.pqindex);
                        if (c_.track_transform_map && k<4)
                            out.transform_map[(pos*4)+static_cast<size_t>(k)]=pack_transform_map(pd.type,pd.skip_mask);
                    }
                }
            }
        }

        std::vector<int> desired_mquant(mbs,c_.pqindex);
        for (size_t pos=0;pos<mbs;++pos) {
            const BMbSyntax& ms=syntax[pos];
            if (ms.mode==BMbMode::Intra || ms.cbp) desired_mquant[pos]=ms.mquant;
        }
        const DQuantPlan dquant_plan=make_dquant_plan(desired_mquant,mbw,mbh);
        const bool dquantfrm=dquant_plan.enabled;
        bool esc3_lengths_written=false;
        for (size_t pos=0;pos<mbs;++pos) {
            BMbSyntax& ms=syntax[pos];
            if (ms.mode==BMbMode::Intra) {
                for (int k=0;k<6;++k) {
                    write_dc_diff(ms.coeff,ms.intra_dc_diff[static_cast<size_t>(k)],k>=4,ms.mquant,0);
                    if (ms.cbp&(1u<<(5-k)))
                        write_ac_block(ms.coeff,(*ms.intra_q)[static_cast<size_t>(k)],k>=4,
                                       k>=4?coding_set:intra_luma_set,use_vlc,
                                       esc3_lengths_written,kInterScan.data(),dquantfrm);
                }
            } else if (ms.cbp) {
                write_transform_payload(ms.coeff,ms.tx,coding_set,use_vlc,esc3_lengths_written,false,dquantfrm);
            }
        }

        const BitplaneChoice direct_choice=choose_bitplane(direct_plane,mbw,mbh);
        const BitplaneChoice skip_choice=choose_bitplane(skip_plane,mbw,mbh);
        const int mv_table=choose_mv_table(mv_values);
        const int cbp_table=choose_cbp_table(cbps);

        if (c_.debug_macroblock_stats) {
            for(size_t pos=0;pos<mbs;++pos){const BMbSyntax& ms=syntax[pos];auto& d=out.macroblock_debug[pos];
                d.i_picture_q=c_.pqindex;d.i_mquant=ms.mquant;d.i_dquant_delta=ms.mquant-c_.pqindex;d.i_field_index=d.i_field_parity=-1;d.i_forward_reference_display_order=d.i_backward_reference_display_order=-1;d.b_skipped=ms.skipped?1:0;d.b_intra=ms.mode==BMbMode::Intra;
                d.f_inter_intra_cost_ratio=pos<a.inter_intra_cost_ratio.size()?a.inter_intra_cost_ratio[pos]:-1.0;
                d.f_forward_distant_local_mae_y=pos<a.forward_distant_local_mae.size()?a.forward_distant_local_mae[pos]:-1.0;d.f_forward_distant_candidate_mae_y=pos<a.forward_distant_candidate_mae.size()?a.forward_distant_candidate_mae[pos]:-1.0;d.i_forward_distant_match_decision=pos<a.forward_distant_match_decision.size()?static_cast<vc1_distant_match_decision_e>(a.forward_distant_match_decision[pos]):VC1_DISTANT_MATCH_NONE;
                d.f_backward_distant_local_mae_y=pos<a.backward_distant_local_mae.size()?a.backward_distant_local_mae[pos]:-1.0;d.f_backward_distant_candidate_mae_y=pos<a.backward_distant_candidate_mae.size()?a.backward_distant_candidate_mae[pos]:-1.0;d.i_backward_distant_match_decision=pos<a.backward_distant_match_decision.size()?static_cast<vc1_distant_match_decision_e>(a.backward_distant_match_decision[pos]):VC1_DISTANT_MATCH_NONE;
                d.b_direct=ms.direct?1:0;d.b_acpred=ms.intra_acpred?1:0;d.i_cbp=ms.cbp;
                d.i_mode=ms.mode==BMbMode::Intra?VC1_MB_DEBUG_B_INTRA:(ms.mode==BMbMode::Backward?VC1_MB_DEBUG_B_BACKWARD:(ms.mode==BMbMode::Interpolated?VC1_MB_DEBUG_B_INTERPOLATED:(ms.mode==BMbMode::Direct?VC1_MB_DEBUG_B_DIRECT:VC1_MB_DEBUG_B_FORWARD)));
                d.i_coded_blocks=0;for(int k=0;k<6;++k)if(ms.cbp&(1u<<(5-k)))++d.i_coded_blocks;
                if(ms.mode!=BMbMode::Intra){d.i_forward_mv_count=(ms.mode==BMbMode::Backward)?0:1;d.i_backward_mv_count=(ms.mode==BMbMode::Forward)?0:1;
                    if(d.i_forward_mv_count){d.i_forward_mv_xq[0]=ms.fmv.xq;d.i_forward_mv_yq[0]=ms.fmv.yq;}if(d.i_backward_mv_count){d.i_backward_mv_xq[0]=ms.bmv.xq;d.i_backward_mv_yq[0]=ms.bmv.yq;}
                    d.i_estimated_transform_bits=ms.tx.estimated_bits;for(int k=0;k<6;++k)if(ms.cbp&(1u<<(5-k))){const auto& pd=ms.tx.parents[k];d.i_transform_parts[static_cast<size_t>(static_cast<uint8_t>(pd.type)-1)]+=static_cast<uint32_t>(count_coded_parts(pd,pd.type));}
                }else d.i_estimated_transform_bits=ms.coeff.bit_count();
            }
        }

        BitWriter b;
        if (c_.syntax == StreamSyntax::Wmv9Main) {
            b.bits(0,2); b.bits(0b00,2); // FRMCNT, PTYPE=B
        } else {
            if (c_.interlaced()) b.bit(false); // FCM=0.
            b.bits(0b10,2); // Advanced PTYPE=B.
            if (c_.interlaced()) { b.bit(c_.top_field_first()); b.bit(false); } // TFF, RFF=0.
            b.bit(false); // RNDCTRL=0.
            if (c_.interlaced()) b.bit(false); // UVSAMP=0.
        }
        if (fraction_den==2) b.bits(0b000,3);
        else if (fraction_num==1) b.bits(0b001,3);
        else b.bits(0b010,3);
        b.bits(static_cast<uint64_t>(c_.pqindex),5);
        if (c_.pqindex <= 8) b.bit(c_.halfqp);
        b.bit(uniform_quantizer()); // PQUANTIZER (QUANTIZER=frame-explicit).
        if (extended_mv_enabled()) write_mvrange(b);
        b.bit(a.mv_mode==ProgressiveMvMode::OneMvQpel); // progressive B: 1=qpel mspel, 0=half-pel bilinear
        b.append(direct_choice.syntax);
        b.append(skip_choice.syntax);
        b.bits(static_cast<uint64_t>(mv_table),2);
        b.bits(static_cast<uint64_t>(cbp_table),2);
        if (c_.dquant) write_dquant_header(b,dquant_plan);
        if (c_.variable_transforms) {
            b.bit(transform_plan.frame_level); // TTMBF
            if (transform_plan.frame_level)
                b.bits(static_cast<uint64_t>(static_cast<uint8_t>(transform_plan.frame_type)-1),2); // TTFRM
        }
        write_decode012(b,ac_table_index);
        b.bit(false); // TRANSDCTAB=0

        auto write_bmvtype=[&](BMbMode mode) {
            if (mode==BMbMode::Interpolated) { b.bits(0b11,2); return; }
            const bool backward=mode==BMbMode::Backward;
            if (backward == below_half) b.bits(0b10,2); else b.bit(false);
        };
        for (size_t pos=0;pos<mbs;++pos) {
            const size_t debug_bits_before=c_.debug_macroblock_stats?b.bit_count():0;
            const BMbSyntax& ms=syntax[pos];
            if (direct_choice.raw) b.bit(ms.direct);
            if (skip_choice.raw) b.bit(ms.skipped);
            if (ms.skipped) {
                if (!ms.direct) write_bmvtype(ms.mode);
                if (c_.debug_macroblock_stats) out.macroblock_debug[pos].i_local_bits=static_cast<uint64_t>(b.bit_count()-debug_bits_before);
                continue;
            }
            if (ms.mode==BMbMode::Intra) {
                write_mvdata_table(b,mv_table,mvdata_for_mode(MotionVector{},MotionVector{},a.mv_mode,ms.cbp!=0,true));
                if (!ms.cbp && dquantfrm) write_dquant_mb(b,dquant_plan,pos);
                b.bit(ms.intra_acpred);
                if (ms.cbp) {
                    write_cbp_table(b,cbp_table,ms.cbp);
                    if (dquantfrm) write_dquant_mb(b,dquant_plan,pos);
                }
                b.append(ms.coeff);
                if (c_.debug_macroblock_stats) out.macroblock_debug[pos].i_local_bits=static_cast<uint64_t>(b.bit_count()-debug_bits_before);
                continue;
            }
            if (!ms.direct) {
                if (ms.mode==BMbMode::Interpolated) {
                    write_mvdata_table(b,mv_table,mvdata_for_mode(ms.bmv,ms.bpred,a.mv_mode,true));
                    write_bmvtype(ms.mode);
                    write_mvdata_table(b,mv_table,mvdata_for_mode(ms.fmv,ms.fpred,a.mv_mode,ms.cbp!=0));
                } else {
                    const MotionVector d=ms.mode==BMbMode::Forward?ms.fmv:ms.bmv;
                    const MotionVector p=ms.mode==BMbMode::Forward?ms.fpred:ms.bpred;
                    write_mvdata_table(b,mv_table,mvdata_for_mode(d,p,a.mv_mode,ms.cbp!=0));
                    write_bmvtype(ms.mode);
                }
            }
            if (ms.cbp) {
                write_cbp_table(b,cbp_table,ms.cbp);
                if (dquantfrm) write_dquant_mb(b,dquant_plan,pos);
                b.append(ms.coeff);
            }
            if (c_.debug_macroblock_stats) out.macroblock_debug[pos].i_local_bits=static_cast<uint64_t>(b.bit_count()-debug_bits_before);
        }

        if (c_.loop_filter) apply_i_like_loop_filter(loop_reconstructed);
        out.padded_reconstructed=loop_reconstructed;
        out.reconstructed=crop_padded_frame(loop_reconstructed);
        if(c_.debug_macroblock_stats){Frame pred=empty_frame();std::vector<uint8_t> pred_valid(mbs,0);for(int my=0;my<mbh;++my)for(int mx=0;mx<mbw;++mx){const size_t pos=static_cast<size_t>(my)*mbw+mx;const BMbSyntax& ms=syntax[pos];if(ms.mode==BMbMode::Intra)continue;pred_valid[pos]=1;if(ms.mode==BMbMode::Backward)motion_compensate_mb(pred,future,mx,my,ms.bmv,a.mv_mode);else if(ms.mode==BMbMode::Forward)motion_compensate_mb(pred,forward_ref,mx,my,ms.fmv,a.mv_mode);else motion_compensate_bi_mb(pred,forward_ref,future,mx,my,ms.fmv,ms.bmv,a.mv_mode);}fill_macroblock_quality(out.macroblock_debug,f,out.reconstructed,previous_source,&pred,&pred_valid);}
        out.data=c_.syntax == StreamSyntax::Wmv9Main ? b.finish_raw() : bdu(0x0d,b.finish_rbdu());
        return out;
    }

void Vc1Encoder::encode_rowskip_bits(BitWriter& b, const std::vector<uint8_t>& p, int w, int h) {
        for (int y=0;y<h;++y) {
            bool any=false;
            for (int x=0;x<w;++x) any |= p[static_cast<size_t>(y)*w+x] != 0;
            b.bit(any);
            if (any) for (int x=0;x<w;++x) b.bit(p[static_cast<size_t>(y)*w+x] != 0);
        }
    }

void Vc1Encoder::encode_colskip_bits(BitWriter& b, const std::vector<uint8_t>& p, int w, int h) {
        for (int x=0;x<w;++x) {
            bool any=false;
            for (int y=0;y<h;++y) any |= p[static_cast<size_t>(y)*w+x] != 0;
            b.bit(any);
            if (any) for (int y=0;y<h;++y) b.bit(p[static_cast<size_t>(y)*w+x] != 0);
        }
    }

void Vc1Encoder::encode_norm2_bits(BitWriter& b, const std::vector<uint8_t>& p) {
        size_t off=0;
        if (p.size() & 1u) { b.bit(p[0] != 0); off=1; }
        for (;off+1<p.size();off+=2) {
            const unsigned sym=(p[off]?1u:0u) | (p[off+1]?2u:0u);
            b.vlc(groupa::kNorm2Codes[sym],groupa::kNorm2Bits[sym]);
        }
    }

void Vc1Encoder::encode_norm6_bits(BitWriter& b, const std::vector<uint8_t>& p, int w, int h) {
        auto at=[&](int x,int y)->unsigned { return p[static_cast<size_t>(y)*w+x]?1u:0u; };
        if (!(h%3) && (w%3)) { // 2x3 tiles; optional first column follows the tile VLCs.
            for (int y=0;y<h;y+=3) for (int x=w&1;x<w;x+=2) {
                const unsigned sym=at(x,y) | (at(x+1,y)<<1) |
                    (at(x,y+1)<<2) | (at(x+1,y+1)<<3) |
                    (at(x,y+2)<<4) | (at(x+1,y+2)<<5);
                b.vlc(groupa::kNorm6Codes[sym],groupa::kNorm6Bits[sym]);
            }
            if (w&1) {
                std::vector<uint8_t> col(static_cast<size_t>(h));
                for (int y=0;y<h;++y) col[static_cast<size_t>(y)]=static_cast<uint8_t>(at(0,y));
                encode_colskip_bits(b,col,1,h);
            }
        } else { // 3x2 tiles; leading columns and optional first row follow tile VLCs.
            const int lead=w%3;
            for (int y=h&1;y<h;y+=2) for (int x=lead;x<w;x+=3) {
                const unsigned sym=at(x,y) | (at(x+1,y)<<1) | (at(x+2,y)<<2) |
                    (at(x,y+1)<<3) | (at(x+1,y+1)<<4) | (at(x+2,y+1)<<5);
                b.vlc(groupa::kNorm6Codes[sym],groupa::kNorm6Bits[sym]);
            }
            if (lead) {
                std::vector<uint8_t> cols(static_cast<size_t>(lead)*h);
                for (int y=0;y<h;++y) for (int x=0;x<lead;++x)
                    cols[static_cast<size_t>(y)*lead+x]=static_cast<uint8_t>(at(x,y));
                encode_colskip_bits(b,cols,lead,h);
            }
            if (h&1) {
                std::vector<uint8_t> row(static_cast<size_t>(w-lead));
                for (int x=lead;x<w;++x) row[static_cast<size_t>(x-lead)]=static_cast<uint8_t>(at(x,0));
                encode_rowskip_bits(b,row,w-lead,1);
            }
        }
    }

Frame Vc1Encoder::empty_frame() const {
        Frame f;
        f.width=c_.width; f.height=c_.height;
        f.y.assign(static_cast<size_t>(c_.width)*c_.height,0);
        f.u.assign(static_cast<size_t>(c_.width/2)*(c_.height/2),128);
        f.v.assign(f.u.size(),128);
        return f;
    }

Frame Vc1Encoder::empty_padded_frame(int mbw,int mbh) const {
        Frame f;
        f.width=mbw*16; f.height=mbh*16;
        f.y.assign(static_cast<size_t>(f.width)*f.height,0);
        f.u.assign(static_cast<size_t>(f.width/2)*(f.height/2),128);
        f.v.assign(f.u.size(),128);
        return f;
    }

Frame Vc1Encoder::crop_padded_frame(const Frame& padded) const {
        Frame out=empty_frame();
        for (int y=0;y<c_.height;++y)
            std::copy_n(padded.y.data()+static_cast<size_t>(y)*padded.width,c_.width,
                        out.y.data()+static_cast<size_t>(y)*c_.width);
        const int cw=c_.width/2,ch=c_.height/2,pw=padded.width/2;
        for (int y=0;y<ch;++y) {
            std::copy_n(padded.u.data()+static_cast<size_t>(y)*pw,cw,out.u.data()+static_cast<size_t>(y)*cw);
            std::copy_n(padded.v.data()+static_cast<size_t>(y)*pw,cw,out.v.data()+static_cast<size_t>(y)*cw);
        }
        return out;
    }

} // namespace libvc1
