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

using OverlapMode = Vc1Encoder::OverlapMode;
using OverlapPlan = Vc1Encoder::OverlapPlan;
using SignedFrame = Vc1Encoder::SignedFrame;


static bool simd_overlap_vertical_dispatch(SimdTier t,int* edge,int stride) {
#if defined(LIBVC1_HAVE_X86_64_V4)
    if(t==SimdTier::X86V4){simd::overlap_vertical8_x86_64_v4(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
    if(t==SimdTier::Avx2Partial){simd::overlap_vertical8_avx2_partial(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
    if(t==SimdTier::X86V3){simd::overlap_vertical8_x86_64_v3(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
    if(t==SimdTier::Piledriver){simd::overlap_vertical8_piledriver(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
    if(t==SimdTier::Bulldozer){simd::overlap_vertical8_bulldozer(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
    if(t==SimdTier::SandyBridge){simd::overlap_vertical8_sandybridge(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
    if(t==SimdTier::X86V2){simd::overlap_vertical8_x86_64_v2(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_PENRYN)
    if(t==SimdTier::Penryn){simd::overlap_vertical8_penryn(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_CONROE)
    if(t==SimdTier::Conroe){simd::overlap_vertical8_conroe(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_K10)
    if(t==SimdTier::K10){simd::overlap_vertical8_k10(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
    if(t==SimdTier::Prescott){simd::overlap_vertical8_prescott(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
    if(t==SimdTier::X86V1){simd::overlap_vertical8_x86_64_v1(edge,stride);return true;}
#endif
    return false;
}
static bool simd_overlap_horizontal_dispatch(SimdTier t,int* edge,int stride) {
#if defined(LIBVC1_HAVE_X86_64_V4)
    if(t==SimdTier::X86V4){simd::overlap_horizontal8_x86_64_v4(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
    if(t==SimdTier::Avx2Partial){simd::overlap_horizontal8_avx2_partial(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
    if(t==SimdTier::X86V3){simd::overlap_horizontal8_x86_64_v3(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
    if(t==SimdTier::Piledriver){simd::overlap_horizontal8_piledriver(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
    if(t==SimdTier::Bulldozer){simd::overlap_horizontal8_bulldozer(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
    if(t==SimdTier::SandyBridge){simd::overlap_horizontal8_sandybridge(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
    if(t==SimdTier::X86V2){simd::overlap_horizontal8_x86_64_v2(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_PENRYN)
    if(t==SimdTier::Penryn){simd::overlap_horizontal8_penryn(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_CONROE)
    if(t==SimdTier::Conroe){simd::overlap_horizontal8_conroe(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_K10)
    if(t==SimdTier::K10){simd::overlap_horizontal8_k10(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
    if(t==SimdTier::Prescott){simd::overlap_horizontal8_prescott(edge,stride);return true;}
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
    if(t==SimdTier::X86V1){simd::overlap_horizontal8_x86_64_v1(edge,stride);return true;}
#endif
    return false;
}
static int simd_loop_filter4_dispatch(SimdTier t,bool vertical,uint8_t* p,int stride,int x,int y,int pq) {
#if defined(LIBVC1_HAVE_X86_64_V4)
    if(t==SimdTier::X86V4)return vertical?simd::loop_filter4_v_x86_64_v4(p,stride,x,y,pq):simd::loop_filter4_h_x86_64_v4(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
    if(t==SimdTier::Avx2Partial)return vertical?simd::loop_filter4_v_avx2_partial(p,stride,x,y,pq):simd::loop_filter4_h_avx2_partial(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
    if(t==SimdTier::X86V3)return vertical?simd::loop_filter4_v_x86_64_v3(p,stride,x,y,pq):simd::loop_filter4_h_x86_64_v3(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
    if(t==SimdTier::Piledriver)return vertical?simd::loop_filter4_v_piledriver(p,stride,x,y,pq):simd::loop_filter4_h_piledriver(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
    if(t==SimdTier::Bulldozer)return vertical?simd::loop_filter4_v_bulldozer(p,stride,x,y,pq):simd::loop_filter4_h_bulldozer(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
    if(t==SimdTier::SandyBridge)return vertical?simd::loop_filter4_v_sandybridge(p,stride,x,y,pq):simd::loop_filter4_h_sandybridge(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
    if(t==SimdTier::X86V2)return vertical?simd::loop_filter4_v_x86_64_v2(p,stride,x,y,pq):simd::loop_filter4_h_x86_64_v2(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_PENRYN)
    if(t==SimdTier::Penryn)return vertical?simd::loop_filter4_v_penryn(p,stride,x,y,pq):simd::loop_filter4_h_penryn(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_CONROE)
    if(t==SimdTier::Conroe)return vertical?simd::loop_filter4_v_conroe(p,stride,x,y,pq):simd::loop_filter4_h_conroe(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_K10)
    if(t==SimdTier::K10)return vertical?simd::loop_filter4_v_k10(p,stride,x,y,pq):simd::loop_filter4_h_k10(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
    if(t==SimdTier::Prescott)return vertical?simd::loop_filter4_v_prescott(p,stride,x,y,pq):simd::loop_filter4_h_prescott(p,stride,x,y,pq);
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
    if(t==SimdTier::X86V1)return vertical?simd::loop_filter4_v_x86_64_v1(p,stride,x,y,pq):simd::loop_filter4_h_x86_64_v1(p,stride,x,y,pq);
#endif
    return -1;
}

namespace {

static int overlap_arshift3(int v) {
    return v >= 0 ? (v >> 3) : - ((-v + 7) >> 3);
}

static void overlap_vertical_edge(std::vector<int>& p,int w,int x,int y0) {
    // VC-1 calls this the horizontal overlap: it crosses a vertical 8x8 edge.
    for (int i=0;i<8;++i) {
        const int y=y0+i;
        const size_t o=static_cast<size_t>(y)*w+x;
        const int a=p[o-2], b=p[o-1], c=p[o], d=p[o+1];
        const int d1=a-d;
        const int d2=a-d+b-c;
        const int r1=(i&1)?3:4, r2=7-r1;
        p[o-2]=overlap_arshift3(8*a-d1+r1);
        p[o-1]=overlap_arshift3(8*b-d2+r2);
        p[o]  =overlap_arshift3(8*c+d2+r1);
        p[o+1]=overlap_arshift3(8*d+d1+r2);
    }
}

static void overlap_horizontal_edge(std::vector<int>& p,int w,int x0,int y) {
    // VC-1 calls this the vertical overlap: it crosses a horizontal 8x8 edge.
    for (int i=0;i<8;++i) {
        const int x=x0+i;
        const size_t o=static_cast<size_t>(y)*w+x;
        const int a=p[o-static_cast<size_t>(2*w)];
        const int b=p[o-static_cast<size_t>(w)];
        const int c=p[o];
        const int d=p[o+static_cast<size_t>(w)];
        const int d1=a-d;
        const int d2=a-d+b-c;
        const int r1=(i&1)?3:4, r2=7-r1;
        p[o-static_cast<size_t>(2*w)]=overlap_arshift3(8*a-d1+r1);
        p[o-static_cast<size_t>(w)]  =overlap_arshift3(8*b-d2+r2);
        p[o]                         =overlap_arshift3(8*c+d2+r1);
        p[o+static_cast<size_t>(w)]  =overlap_arshift3(8*d+d1+r2);
    }
}

static bool overlap_edge_enabled(bool luma,int bx0,int by0,int bx1,int by1,
                                 const std::vector<uint8_t>& flags,int mbw,int mbh,
                                 bool all) {
    if (all) return true;
    auto mb_index=[&](int bx,int by)->size_t {
        const int mx=luma?(bx>>1):bx;
        const int my=luma?(by>>1):by;
        if (mx<0 || my<0 || mx>=mbw || my>=mbh) return flags.size();
        return static_cast<size_t>(my)*mbw+mx;
    };
    const size_t a=mb_index(bx0,by0), b=mb_index(bx1,by1);
    if (a>=flags.size() || b>=flags.size()) return false;
    if (a==b) return flags[a]!=0; // internal luma 8x8 edge
    return flags[a]!=0 && flags[b]!=0; // macroblock edge requires both OVERFLAGS
}

static void apply_overlap_plane_signed(std::vector<int>& p,int w,int h,bool luma,
                                       const std::vector<uint8_t>& flags,int mbw,int mbh,
                                       bool all,SimdTier tier) {
    if (w<8 || h<8) return;
    const int bw=w/8, bh=h/8;
    // Horizontal-overlap stage first: vertical block boundaries. Work in 8-pixel
    // segments so CONDOVER SELECT can change at macroblock-row boundaries.
    for (int bx=1;bx<bw;++bx) {
        const int x=bx*8;
        for (int by=0;by<bh;++by) {
            if (overlap_edge_enabled(luma,bx-1,by,bx,by,flags,mbw,mbh,all)) {
                int* edge=p.data()+static_cast<size_t>(by*8)*w+x-2;
                if (!simd_overlap_vertical_dispatch(tier,edge,w)) overlap_vertical_edge(p,w,x,by*8);
            }
        }
    }
    // Vertical-overlap stage second: horizontal block boundaries. This observes
    // the full-precision crossing samples produced by the first stage.
    for (int by=1;by<bh;++by) {
        const int y=by*8;
        for (int bx=0;bx<bw;++bx) {
            if (overlap_edge_enabled(luma,bx,by-1,bx,by,flags,mbw,mbh,all)) {
                int* edge=p.data()+static_cast<size_t>(y)*w+bx*8;
                if (!simd_overlap_horizontal_dispatch(tier,edge,w)) overlap_horizontal_edge(p,w,bx*8,y);
            }
        }
    }
}

static long double overlap_sse_region(const Vc1Encoder::SignedFrame& sf,const Frame& src,
                                      int mx,int my,int bias) {
    long double sse=0.0L;
    auto plane_sse=[&](const std::vector<int>& rec,int rw,int rh,
                       const std::vector<uint8_t>& org,int ow,int oh,
                       int x0,int y0,int pw,int ph) {
        for (int y=0;y<ph;++y) for (int x=0;x<pw;++x) {
            const int rx=x0+x, ry=y0+y;
            if (rx>=rw || ry>=rh) continue;
            const int sx=std::clamp(rx,0,ow-1), sy=std::clamp(ry,0,oh-1);
            const int r=std::clamp(rec[static_cast<size_t>(ry)*rw+rx]+bias,0,255);
            const int d=r-org[static_cast<size_t>(sy)*ow+sx];
            sse+=static_cast<long double>(d)*d;
        }
    };
    plane_sse(sf.y,sf.width,sf.height,src.y,src.width,src.height,mx*16,my*16,16,16);
    plane_sse(sf.u,sf.width/2,sf.height/2,src.u,src.width/2,src.height/2,mx*8,my*8,8,8);
    plane_sse(sf.v,sf.width/2,sf.height/2,src.v,src.width/2,src.height/2,mx*8,my*8,8,8);
    return sse;
}

} // namespace

int Vc1Encoder::intra_recon_bias() const {
    // Simple/Main changes its intra reconstruction centering when overlap is
    // actually active (PQ>=9). Advanced I/BI always uses the +128 path.
    if (c_.syntax==StreamSyntax::Wmv9Main)
        return (c_.overlap && c_.pqindex>=9) ? 128 : 0;
    return 128;
}

OverlapPlan Vc1Encoder::choose_i_overlap_plan(const Frame& f) const {
    OverlapPlan plan;
    const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
    const size_t mbs=static_cast<size_t>(mbw)*mbh;
    plan.flags.assign(mbs,0);
    if (!c_.overlap) return plan;

    // Main Profile has no CONDOVER syntax. At low Q the standard disables I/BI
    // overlap even when the sequence OVERLAP flag is set.
    if (c_.syntax==StreamSyntax::Wmv9Main) {
        if (c_.pqindex>=9) plan.mode=OverlapMode::All;
        return plan;
    }
    // Advanced Profile uses unconditional overlap above the CONDOVER range.
    if (c_.pqindex>=9) {
        plan.mode=OverlapMode::All;
        return plan;
    }

    // Low-Q Advanced I/BI: derive a selective OVERFLAGS map by exact local
    // reconstruction error. This is deliberately reconstruction-domain RDO:
    // it judges the real unclipped inverse-transform samples, not source edge
    // activity, so it adapts naturally to DQUANT and AC prediction decisions.
    const SignedFrame base=reconstruct_i_signed_padded(f);
    SignedFrame all=base;
    OverlapPlan all_plan; all_plan.mode=OverlapMode::All;
    apply_i_overlap(all,all_plan);
    bool any=false, every=true;
    for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
        const size_t pos=static_cast<size_t>(my)*mbw+mx;
        const long double e0=overlap_sse_region(base,f,mx,my,128);
        const long double e1=overlap_sse_region(all,f,mx,my,128);
        // A small quantizer-scaled margin prevents one-sample ties/noise from
        // paying OVERFLAGS syntax for no material reconstruction improvement.
        const long double margin=static_cast<long double>(c_.pqindex*c_.pqindex)*4.0L;
        const bool on=e1+margin<e0;
        plan.flags[pos]=on?1:0;
        any|=on; every&=on;
    }
    if (!any) { plan.mode=OverlapMode::None; return plan; }
    if (every) { plan.mode=OverlapMode::All; return plan; }

    plan.mode=OverlapMode::Select;
    plan.bitplane=choose_bitplane(plan.flags,mbw,mbh);

    // Compare whole-picture NONE/ALL/SELECT distortion plus a lightweight
    // syntax penalty. SELECT wins only when its per-MB flexibility is useful.
    SignedFrame sel=base;
    apply_i_overlap(sel,plan);
    auto total_sse=[&](const SignedFrame& x) {
        long double e=0.0L;
        for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx)
            e+=overlap_sse_region(x,f,mx,my,128);
        return e;
    };
    const long double lambda=static_cast<long double>(c_.pqindex*c_.pqindex)*8.0L;
    const long double nscore=total_sse(base)+lambda; // CONDOVER_NONE = 1 bit
    const long double ascore=total_sse(all)+2.0L*lambda; // ALL = 10
    const long double sscore=total_sse(sel)+lambda*(2.0L+plan.bitplane.total_bits); // SELECT = 11 + plane
    if (nscore<=ascore && nscore<=sscore) {
        plan.mode=OverlapMode::None; plan.flags.assign(mbs,0); plan.bitplane=BitplaneChoice{};
    } else if (ascore<=sscore) {
        plan.mode=OverlapMode::All; plan.flags.assign(mbs,1); plan.bitplane=BitplaneChoice{};
    }
    return plan;
}

void Vc1Encoder::apply_i_overlap(SignedFrame& f,const OverlapPlan& plan) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::Overlap);
    if (plan.mode==OverlapMode::None) return;
    const int mbw=(f.width+15)/16, mbh=(f.height+15)/16;
    const bool all=plan.mode==OverlapMode::All;
    apply_overlap_plane_signed(f.y,f.width,f.height,true,plan.flags,mbw,mbh,all,c_.simd_for(SimdPrimitive::AddBlockRect));
    apply_overlap_plane_signed(f.u,f.width/2,f.height/2,false,plan.flags,mbw,mbh,all,c_.simd_for(SimdPrimitive::AddBlockRect));
    apply_overlap_plane_signed(f.v,f.width/2,f.height/2,false,plan.flags,mbw,mbh,all,c_.simd_for(SimdPrimitive::AddBlockRect));
}

void Vc1Encoder::apply_p_overlap(SignedFrame& f,const std::vector<uint8_t>& intra,int mbw,int mbh) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::Overlap);
    if (!c_.overlap || c_.pqindex<9) return;
    if (intra.size()!=static_cast<size_t>(mbw)*mbh)
        throw std::runtime_error("internal P OVERLAP map size mismatch");
    std::vector<uint8_t> flags(intra.size());
    for (size_t i=0;i<intra.size();++i) flags[i]=intra[i]?1:0;
    apply_overlap_plane_signed(f.y,f.width,f.height,true,flags,mbw,mbh,false,c_.simd_for(SimdPrimitive::AddBlockRect));
    apply_overlap_plane_signed(f.u,f.width/2,f.height/2,false,flags,mbw,mbh,false,c_.simd_for(SimdPrimitive::AddBlockRect));
    apply_overlap_plane_signed(f.v,f.width/2,f.height/2,false,flags,mbw,mbh,false,c_.simd_for(SimdPrimitive::AddBlockRect));
}

bool Vc1Encoder::loop_filter_line(std::vector<uint8_t>& p,int w,int h,int x,int y,
                                 int cross_dx,int cross_dy,int pq) {
        auto inside=[&](int xx,int yy) { return xx>=0 && yy>=0 && xx<w && yy<h; };
        for (int k=-4;k<=3;++k)
            if (!inside(x+k*cross_dx,y+k*cross_dy)) return false;
        auto sample=[&](int k)->int {
            return p[static_cast<size_t>(y+k*cross_dy)*w+x+k*cross_dx];
        };
        int a0=arshift(2*(sample(-2)-sample(1))-5*(sample(-1)-sample(0))+4,3);
        const bool a0_neg=a0<0;
        a0=std::abs(a0);
        if (a0>=pq) return false;
        const int a1=std::abs(arshift(2*(sample(-4)-sample(-1))-5*(sample(-3)-sample(-2))+4,3));
        const int a2=std::abs(arshift(2*(sample(0)-sample(3))-5*(sample(1)-sample(2))+4,3));
        if (!(a1<a0 || a2<a0)) return false;
        const int edge=sample(-1)-sample(0);
        int limit=std::abs(edge)>>1;
        if (!limit) return false;
        int delta=(5*(a0-std::min(a1,a2)))>>3;
        if (a0_neg != (edge<0)) {
            delta=std::min(delta,limit);
            if (edge<0) delta=-delta;
            const int px=x-cross_dx, py=y-cross_dy;
            const size_t po=static_cast<size_t>(py)*w+px;
            const size_t qo=static_cast<size_t>(y)*w+x;
            p[po]=static_cast<uint8_t>(std::clamp(static_cast<int>(p[po])-delta,0,255));
            p[qo]=static_cast<uint8_t>(std::clamp(static_cast<int>(p[qo])+delta,0,255));
        }
        return true;
    }

void Vc1Encoder::loop_filter_edge(std::vector<uint8_t>& p,int w,int h,int x,int y,
                                 int along_dx,int along_dy,int cross_dx,int cross_dy,
                                 int len,int pq,SimdTier tier) {
        for (int i=0;i<len;i+=4) {
            const int bx=x+i*along_dx, by=y+i*along_dy;
            if (tier!=SimdTier::None && i+4<=len) {
                int vr=-1;
                if (along_dx==1 && along_dy==0 && cross_dx==0 && cross_dy==1 && bx>=0 && bx+3<w && by>=4 && by+3<h)
                    vr=simd_loop_filter4_dispatch(tier,false,p.data(),w,bx,by,pq);
                else if (along_dx==0 && along_dy==1 && cross_dx==1 && cross_dy==0 && by>=0 && by+3<h && bx>=4 && bx+3<w)
                    vr=simd_loop_filter4_dispatch(tier,true,p.data(),w,bx,by,pq);
                if (vr>=0) continue;
            }
            const bool enable=loop_filter_line(p,w,h,bx+2*along_dx,by+2*along_dy,cross_dx,cross_dy,pq);
            if (!enable) continue;
            loop_filter_line(p,w,h,bx,by,cross_dx,cross_dy,pq);
            loop_filter_line(p,w,h,bx+along_dx,by+along_dy,cross_dx,cross_dy,pq);
            loop_filter_line(p,w,h,bx+3*along_dx,by+3*along_dy,cross_dx,cross_dy,pq);
        }
    }

void Vc1Encoder::loop_filter_hborder(std::vector<uint8_t>& p,int w,int h,int x,int y,int len,int pq,SimdTier tier) {
        loop_filter_edge(p,w,h,x,y,1,0,0,1,len,pq,tier);
    }

void Vc1Encoder::loop_filter_vborder(std::vector<uint8_t>& p,int w,int h,int x,int y,int len,int pq,SimdTier tier) {
        loop_filter_edge(p,w,h,x,y,0,1,1,0,len,pq,tier);
    }

void Vc1Encoder::apply_i_like_loop_filter_plane(std::vector<uint8_t>& p,int w,int h) const {
        // Horizontal block borders (the decoder's vertical loop-filter pass)
        // precede vertical block borders.  Keep the same order because the
        // filters can observe pixels changed at crossing edges.
        for (int y=8;y<h;y+=8)
            for (int x=0;x<w;x+=16) loop_filter_hborder(p,w,h,x,y,std::min(16,w-x),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
        for (int x=8;x<w;x+=8)
            for (int y=0;y<h;y+=16) loop_filter_vborder(p,w,h,x,y,std::min(16,h-y),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
    }

void Vc1Encoder::apply_i_like_loop_filter(Frame& f) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::LoopFilter);
        apply_i_like_loop_filter_plane(f.y,f.width,f.height);
        apply_i_like_loop_filter_plane(f.u,f.width/2,f.height/2);
        apply_i_like_loop_filter_plane(f.v,f.width/2,f.height/2);
    }

uint8_t Vc1Encoder::loop_filter_pattern(uint8_t skip_mask,TransformType t) {
        switch (t) {
            case TransformType::T8x8: return 0x0f;
            case TransformType::T4x4: return static_cast<uint8_t>((~skip_mask)&0x0f);
            case TransformType::T8x4:
                return static_cast<uint8_t>((~(((skip_mask&2u)*6u)+((skip_mask&1u)*3u)))&0x0f);
            case TransformType::T4x8:
                return static_cast<uint8_t>((~(skip_mask*5u))&0x0f);
        }
        return 0;
    }

uint8_t Vc1Encoder::loop_filter_tt(TransformType t) {
        switch (t) {
            case TransformType::T8x8: return 0;
            case TransformType::T8x4: return 3;
            case TransformType::T4x8: return 6;
            case TransformType::T4x4: return 7;
        }
        return 0;
    }

void Vc1Encoder::apply_p_loop_filter_plane_main(std::vector<uint8_t>& p,int w,int h,int plane_k,
                                        const std::vector<std::array<MotionVector,4>>& block_mvs,
                                        const std::vector<std::array<uint8_t,6>>& cbp,
                                        const std::vector<std::array<uint8_t,6>>& tt,
                                        const std::vector<uint8_t>& intra,
                                        int mbw,int mbh) const {
        // SMPTE 421M 8.6.4.1 preserves four Main-profile P-picture
        // deblocking compatibility exceptions.  They are deliberately kept
        // separate from the Advanced-profile path below: independent modern
        // decoders commonly implement the generic rule, while the Microsoft
        // WMV3 decoder follows these Main-profile reconstruction quirks.
        const bool luma=plane_k<4;
        const int grid_w=luma?mbw*2:mbw;
        const int grid_h=luma?mbh*2:mbh;
        const bool first_block_intra=!intra.empty() && (intra[0]&1u)!=0;

        auto meta=[&](int bx,int by,int& mx,int& my,int& k) {
            if (luma) {
                mx=bx>>1; my=by>>1; k=((by&1)<<1)|(bx&1);
            } else { mx=bx; my=by; k=plane_k; }
        };
        auto mb_intra=[&](int bx,int by)->bool {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            return intra[static_cast<size_t>(my)*mbw+mx]!=0;
        };
        auto intra_at=[&](int bx,int by)->bool {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            return (intra[static_cast<size_t>(my)*mbw+mx]&(1u<<k))!=0;
        };
        auto cbp_at=[&](int bx,int by)->uint8_t {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            return cbp[static_cast<size_t>(my)*mbw+mx][static_cast<size_t>(k)];
        };
        auto tt_at=[&](int bx,int by)->uint8_t {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            return tt[static_cast<size_t>(my)*mbw+mx][static_cast<size_t>(k)];
        };
        auto chroma_mv_main=[&](int bx,int by)->MotionVector {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            const auto& mv=block_mvs[static_cast<size_t>(my)*mbw+mx];
            const MotionVector lmv=chroma_mv_4mv(mv);
            // 8.3.5.4.3 first-stage 1-MV chroma derivation.
            int x=arshift(lmv.xq+((lmv.xq&3)==3),1);
            int y=arshift(lmv.yq+((lmv.yq&3)==3),1);
            // FASTUVMC is signalled on by libvc1 Main profile: round quarter
            // chroma positions toward zero to integer/half-pel positions.
            if (x<0) x+=(x&1); else x-=(x&1);
            if (y<0) y+=(y&1); else y-=(y&1);
            // 8.3.6.5 / Figure 62 range limiting. Exception 4 requires the
            // resulting iCMvXComp/iCMvYComp, not the unreduced chroma MV, for
            // P-loop boundary decisions.
            const int cx=mx*8,cy=my*8;
            const int px=cx+arshift(x,2),py=cy+arshift(y,2);
            if (px < -8) x=4*(-8-cx)+(x&3);
            else if (px > mbw*8) x=4*(mbw*8-cx)+(x&3);
            if (py < -8) y=4*(-8-cy)+(y&3);
            else if (py > mbh*8) y=4*(mbh*8-cy)+(y&3);
            return MotionVector{x,y};
        };
        auto mv_at=[&](int bx,int by)->MotionVector {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            if (!luma) return chroma_mv_main(bx,by);
            return block_mvs[static_cast<size_t>(my)*mbw+mx][static_cast<size_t>(k)];
        };
        auto force_first_mb_exception=[&](int cur_bx,int cur_by)->bool {
            // Main profile in this encoder is 1-MV only. If the first MB's
            // block 0 is intra, exception 1 forces the complete top/left 8
            // sample boundaries for every later inter-coded macroblock.
            return first_block_intra && !mb_intra(cur_bx,cur_by);
        };

        auto horizontal8=[&](int bx,int top_by) {
            const int bot_by=top_by+1;
            const int x0=bx*8,y=(top_by+1)*8;
            if (x0>=w || y>=h || bot_by>=grid_h) return;
            const bool ex1=force_first_mb_exception(bx,bot_by);
            const bool ex3=(tt_at(bx,top_by)==7 || tt_at(bx,bot_by)==7);
            if (ex1 || ex3 || intra_at(bx,top_by) || intra_at(bx,bot_by) ||
                !same_mv(mv_at(bx,top_by),mv_at(bx,bot_by))) {
                loop_filter_hborder(p,w,h,x0,y,std::min(8,w-x0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                return;
            }
            const int mask=(cbp_at(bx,top_by)|(cbp_at(bx,bot_by)>>2))&3;
            if ((mask&2) && x0<w)
                loop_filter_hborder(p,w,h,x0,y,std::min(4,w-x0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
            if ((mask&1) && x0+4<w)
                loop_filter_hborder(p,w,h,x0+4,y,std::min(4,w-(x0+4)),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
        };
        auto vertical8=[&](int left_bx,int by) {
            const int right_bx=left_bx+1;
            const int x=(left_bx+1)*8,y0=by*8;
            if (x>=w || y0>=h || right_bx>=grid_w) return;
            const bool ex1=force_first_mb_exception(right_bx,by);
            const bool block3_boundary=luma && ((left_bx&1)==0) && ((by&1)==1);
            // Exception 2's lower-right-luma left edge uses block 1's coded
            // status/subblock pattern instead of block 2's. Exception 3 has
            // the same block-1/block-3 interaction on this particular edge.
            const int decision_left_bx=block3_boundary?right_bx:left_bx;
            const int decision_left_by=block3_boundary?by-1:by;
            const bool ex3=(tt_at(decision_left_bx,decision_left_by)==7 || tt_at(right_bx,by)==7);
            if (ex1 || ex3 || intra_at(left_bx,by) || intra_at(right_bx,by) ||
                !same_mv(mv_at(left_bx,by),mv_at(right_bx,by))) {
                loop_filter_vborder(p,w,h,x,y0,std::min(8,h-y0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                return;
            }
            const int mask=(cbp_at(decision_left_bx,decision_left_by)|(cbp_at(right_bx,by)>>1))&5;
            if ((mask&4) && y0<h)
                loop_filter_vborder(p,w,h,x,y0,std::min(4,h-y0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
            if ((mask&1) && y0+4<h)
                loop_filter_vborder(p,w,h,x,y0+4,std::min(4,h-(y0+4)),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
        };
        auto horizontal4=[&](int bx,int by) {
            const int x0=bx*8,y0=by*8;
            if (x0>=w || y0+4>=h) return;
            const uint8_t kind=tt_at(bx,by),pat=cbp_at(bx,by);
            if (kind!=7 && kind!=3) return;
            if ((pat&10) && x0<w)
                loop_filter_hborder(p,w,h,x0,y0+4,std::min(4,w-x0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
            if ((pat&5) && x0+4<w)
                loop_filter_hborder(p,w,h,x0+4,y0+4,std::min(4,w-(x0+4)),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
        };
        auto vertical4=[&](int bx,int by) {
            const int x0=bx*8,y0=by*8;
            if (x0+4>=w || y0>=h) return;
            const uint8_t kind=tt_at(bx,by),pat=cbp_at(bx,by);
            if (kind!=7 && kind!=6) return;
            if ((pat&12) && y0<h)
                loop_filter_vborder(p,w,h,x0+4,y0,std::min(4,h-y0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
            if ((pat&3) && y0+4<h)
                loop_filter_vborder(p,w,h,x0+4,y0+4,std::min(4,h-(y0+4)),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
        };

        // 8.6.2 specifies a frame-wide order: horizontal 8-pixel block
        // boundaries, horizontal 4-pixel subblock boundaries, vertical
        // 8-pixel block boundaries, then vertical 4-pixel subblock boundaries.
        for (int by=0;by+1<grid_h;++by)
            for (int bx=0;bx<grid_w;++bx) horizontal8(bx,by);
        for (int by=0;by<grid_h;++by)
            for (int bx=0;bx<grid_w;++bx) horizontal4(bx,by);
        for (int bx=0;bx+1<grid_w;++bx)
            for (int by=0;by<grid_h;++by) vertical8(bx,by);
        for (int bx=0;bx<grid_w;++bx)
            for (int by=0;by<grid_h;++by) vertical4(bx,by);
    }

void Vc1Encoder::apply_p_loop_filter_plane(std::vector<uint8_t>& p,int w,int h,int plane_k,
                                   const std::vector<std::array<MotionVector,4>>& block_mvs,
                                   const std::vector<std::array<uint8_t,6>>& cbp,
                                   const std::vector<std::array<uint8_t,6>>& tt,
                                   const std::vector<uint8_t>& intra,
                                   int mbw,int mbh) const {
        if (c_.syntax==StreamSyntax::Wmv9Main) {
            apply_p_loop_filter_plane_main(p,w,h,plane_k,block_mvs,cbp,tt,intra,mbw,mbh);
            return;
        }
        const bool luma=plane_k<4;
        auto meta=[&](int bx,int by,int& mx,int& my,int& k) {
            if (luma) {
                mx=bx>>1; my=by>>1; k=((by&1)<<1)|(bx&1);
            } else { mx=bx; my=by; k=plane_k; }
        };
        auto mv_at=[&](int bx,int by)->MotionVector {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            const auto& mv=block_mvs[static_cast<size_t>(my)*mbw+mx];
            if (luma) return mv[static_cast<size_t>(k)];
            // The decoder stores the pre-FASTUVMC chroma representative for
            // P-loop MV comparisons, not the original luma-qpel vector.
            const MotionVector cmv=chroma_mv_4mv(mv);
            return MotionVector{arshift(cmv.xq + ((cmv.xq&3)==3),1),
                                arshift(cmv.yq + ((cmv.yq&3)==3),1)};
        };
        auto cbp_at=[&](int bx,int by)->uint8_t {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            return cbp[static_cast<size_t>(my)*mbw+mx][static_cast<size_t>(k)];
        };
        auto tt_at=[&](int bx,int by)->uint8_t {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            return tt[static_cast<size_t>(my)*mbw+mx][static_cast<size_t>(k)];
        };
        auto intra_at=[&](int bx,int by)->bool {
            int mx=0,my=0,k=0; meta(bx,by,mx,my,k);
            return (intra[static_cast<size_t>(my)*mbw+mx] & (1u<<k))!=0;
        };

        auto process_v_parent=[&](int bx,int by) {
            const int x0=bx*8,y0=by*8;
            if (x0>=w || y0>=h) return;
            const int grid_h=luma?mbh*2:mbh;
            if (by+1<grid_h && y0+8<h) {
                const uint8_t top=cbp_at(bx,by), bottom=cbp_at(bx,by+1);
                if (intra_at(bx,by) || intra_at(bx,by+1) ||
                    !same_mv(mv_at(bx,by),mv_at(bx,by+1))) {
                    loop_filter_hborder(p,w,h,x0,y0+8,std::min(8,w-x0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                } else {
                    const int mask=(top|(bottom>>2))&3;
                    if ((mask&2) && x0<w)
                        loop_filter_hborder(p,w,h,x0,y0+8,std::min(4,w-x0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                    if ((mask&1) && x0+4<w)
                        loop_filter_hborder(p,w,h,x0+4,y0+8,std::min(4,w-(x0+4)),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                }
            }
            const uint8_t kind=tt_at(bx,by), pat=cbp_at(bx,by);
            if ((kind==7 || kind==3) && y0+4<h) {
                if ((pat&10) && x0<w)
                    loop_filter_hborder(p,w,h,x0,y0+4,std::min(4,w-x0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                if ((pat&5) && x0+4<w)
                    loop_filter_hborder(p,w,h,x0+4,y0+4,std::min(4,w-(x0+4)),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
            }
        };
        auto process_h_parent=[&](int bx,int by) {
            const int x0=bx*8,y0=by*8;
            if (x0>=w || y0>=h) return;
            const int grid_w=luma?mbw*2:mbw;
            if (bx+1<grid_w && x0+8<w) {
                const uint8_t left=cbp_at(bx,by), right=cbp_at(bx+1,by);
                if (intra_at(bx,by) || intra_at(bx+1,by) ||
                    !same_mv(mv_at(bx,by),mv_at(bx+1,by))) {
                    loop_filter_vborder(p,w,h,x0+8,y0,std::min(8,h-y0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                } else {
                    const int mask=(left|(right>>1))&5;
                    if ((mask&4) && y0<h)
                        loop_filter_vborder(p,w,h,x0+8,y0,std::min(4,h-y0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                    if ((mask&1) && y0+4<h)
                        loop_filter_vborder(p,w,h,x0+8,y0+4,std::min(4,h-(y0+4)),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                }
            }
            const uint8_t kind=tt_at(bx,by), pat=cbp_at(bx,by);
            if ((kind==7 || kind==6) && x0+4<w) {
                if ((pat&12) && y0<h)
                    loop_filter_vborder(p,w,h,x0+4,y0,std::min(4,h-y0),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
                if ((pat&3) && y0+4<h)
                    loop_filter_vborder(p,w,h,x0+4,y0+4,std::min(4,h-(y0+4)),c_.pqindex,c_.simd_for(SimdPrimitive::AddBlockRect));
            }
        };
        auto process_mb_v=[&](int mx,int my) {
            if (luma) {
                process_v_parent(mx*2,  my*2);
                process_v_parent(mx*2+1,my*2);
                process_v_parent(mx*2,  my*2+1);
                process_v_parent(mx*2+1,my*2+1);
            } else process_v_parent(mx,my);
        };
        auto process_mb_h=[&](int mx,int my) {
            if (luma) {
                process_h_parent(mx*2,  my*2);
                process_h_parent(mx*2+1,my*2);
                process_h_parent(mx*2,  my*2+1);
                process_h_parent(mx*2+1,my*2+1);
            } else process_h_parent(mx,my);
        };

        // VC-1's P-loop filter is deliberately delayed in the raster decoder.
        // For a completed macroblock row, V filtering for MB x is performed
        // before H filtering for MB x-1.  A whole-picture "all V, then all H"
        // pass is not equivalent when dense 4-MV edges meet at crossings.
        for (int my=0;my<mbh;++my) {
            for (int mx=0;mx<mbw;++mx) {
                process_mb_v(mx,my);
                if (mx>0) process_mb_h(mx-1,my);
            }
            if (mbw>0) process_mb_h(mbw-1,my);
        }
    }

void Vc1Encoder::apply_p_loop_filter(Frame& f,const std::vector<std::array<MotionVector,4>>& block_mvs,
                             const std::vector<std::array<uint8_t,6>>& cbp,
                             const std::vector<std::array<uint8_t,6>>& tt,
                             const std::vector<uint8_t>& intra,
                             int mbw,int mbh) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::LoopFilter);
        apply_p_loop_filter_plane(f.y,f.width,f.height,0,block_mvs,cbp,tt,intra,mbw,mbh);
        apply_p_loop_filter_plane(f.u,f.width/2,f.height/2,4,block_mvs,cbp,tt,intra,mbw,mbh);
        apply_p_loop_filter_plane(f.v,f.width/2,f.height/2,5,block_mvs,cbp,tt,intra,mbw,mbh);
    }

} // namespace libvc1
