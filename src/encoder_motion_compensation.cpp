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

namespace {
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
static inline int floor_div4_v3(int v) {
    return v>=0 ? v/4 : -((-v+3)/4);
}
static inline int arshift_v3(int v,int s) {
    if (v>=0) return v>>s;
    return -static_cast<int>((static_cast<unsigned long long>(-static_cast<long long>(v)) + ((1ull<<s)-1ull))>>s);
}
static bool try_luma_mc_simd(SimdTier tier,uint8_t* dst,int dst_stride,
                           const std::vector<uint8_t>& ref,int rw,int rh,
                           int x0,int y0,int bw,int bh,int xq,int yq,
                           bool rnd,bool bilinear,bool average) {
    if (bw<=0 || bh<=0 || (bw&7)) return false;
    const int ix=floor_div4_v3(xq),iy=floor_div4_v3(yq);
    const int hm=xq-ix*4,vm=yq-iy*4;
    const int sx0=x0+ix,sy0=y0+iy;
    const int left=bilinear?0:(hm?1:0),right=hm?(bilinear?1:2):0;
    const int top=bilinear?0:(vm?1:0),bottom=vm?(bilinear?1:2):0;
    if (sx0-left<0 || sy0-top<0 || sx0+bw-1+right>=rw || sy0+bh-1+bottom>=rh) return false;
    const uint8_t* rp=ref.data()+static_cast<size_t>(sy0)*rw+sx0;
#if defined(LIBVC1_HAVE_X86_64_V4)
    if (tier==SimdTier::X86V4) { if (average) simd::luma_mc_block_avg_x86_64_v4(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_x86_64_v4(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
    if (tier==SimdTier::X86V1) { if (average) simd::luma_mc_block_avg_x86_64_v1(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_x86_64_v1(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
    if (tier==SimdTier::Prescott) { if (average) simd::luma_mc_block_avg_prescott(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_prescott(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_K10)
    if (tier==SimdTier::K10) { if (average) simd::luma_mc_block_avg_k10(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_k10(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_CONROE)
    if (tier==SimdTier::Conroe) { if (average) simd::luma_mc_block_avg_conroe(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_conroe(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
    if (tier==SimdTier::X86V2) { if (average) simd::luma_mc_block_avg_x86_64_v2(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_x86_64_v2(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
    if (tier==SimdTier::Penryn) { if (average) simd::luma_mc_block_avg_penryn(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_penryn(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
    if (tier==SimdTier::SandyBridge) { if (average) simd::luma_mc_block_avg_sandybridge(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_sandybridge(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
    if (tier==SimdTier::Bulldozer) { if (average) simd::luma_mc_block_avg_bulldozer(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_bulldozer(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
    if (tier==SimdTier::Piledriver) { if (average) simd::luma_mc_block_avg_piledriver(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_piledriver(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
    if (tier==SimdTier::Avx2Partial) { if (average) simd::luma_mc_block_avg_avx2_partial(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_avx2_partial(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
    if (tier==SimdTier::X86V3) { if (average) simd::luma_mc_block_avg_x86_64_v3(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); else simd::luma_mc_block_x86_64_v3(dst,dst_stride,rp,rw,bw,bh,hm,vm,rnd,bilinear); return true; }
#endif
    return false;
}

struct ChromaParams { int ix=0,iy=0,fx=0,fy=0; };
static ChromaParams chroma_params_v3(int xq,int yq,bool future) {
    int uvx=arshift_v3(xq + ((xq&3)==3),1);
    int uvy=arshift_v3(yq + ((yq&3)==3),1);
    if (future) {
        if (uvx<0) uvx -= (uvx&1); else uvx += (uvx&1);
        if (uvy<0) uvy -= (uvy&1); else uvy += (uvy&1);
    } else {
        if (uvx<0) uvx += (uvx&1); else uvx -= (uvx&1);
        if (uvy<0) uvy += (uvy&1); else uvy -= (uvy&1);
    }
    ChromaParams q;
    q.ix=floor_div4_v3(uvx); q.iy=floor_div4_v3(uvy);
    q.fx=(uvx-q.ix*4)*2; q.fy=(uvy-q.iy*4)*2;
    return q;
}
static bool try_chroma_mc_simd(SimdTier tier,uint8_t* dst,int dst_stride,
                             const std::vector<uint8_t>& ref,int rw,int rh,
                             int x0,int y0,int bw,int bh,int xq,int yq,
                             bool future,bool rnd,bool average) {
    if (bw<=0 || bh<=0 || (bw&7)) return false;
    const auto q=chroma_params_v3(xq,yq,future);
    const int sx0=x0+q.ix,sy0=y0+q.iy;
    if (sx0<0 || sy0<0 || sx0+bw-1+(q.fx?1:0)>=rw || sy0+bh-1+(q.fy?1:0)>=rh) return false;
    const uint8_t* rp=ref.data()+static_cast<size_t>(sy0)*rw+sx0;
#if defined(LIBVC1_HAVE_X86_64_V4)
    if (tier==SimdTier::X86V4) { if (average) simd::chroma_mc_block_avg_x86_64_v4(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_x86_64_v4(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
    if (tier==SimdTier::X86V1) { if (average) simd::chroma_mc_block_avg_x86_64_v1(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_x86_64_v1(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
    if (tier==SimdTier::Prescott) { if (average) simd::chroma_mc_block_avg_prescott(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_prescott(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_K10)
    if (tier==SimdTier::K10) { if (average) simd::chroma_mc_block_avg_k10(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_k10(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_CONROE)
    if (tier==SimdTier::Conroe) { if (average) simd::chroma_mc_block_avg_conroe(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_conroe(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
    if (tier==SimdTier::X86V2) { if (average) simd::chroma_mc_block_avg_x86_64_v2(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_x86_64_v2(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_PENRYN)
    if (tier==SimdTier::Penryn) { if (average) simd::chroma_mc_block_avg_penryn(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_penryn(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
    if (tier==SimdTier::SandyBridge) { if (average) simd::chroma_mc_block_avg_sandybridge(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_sandybridge(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
    if (tier==SimdTier::Bulldozer) { if (average) simd::chroma_mc_block_avg_bulldozer(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_bulldozer(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
    if (tier==SimdTier::Piledriver) { if (average) simd::chroma_mc_block_avg_piledriver(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_piledriver(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
    if (tier==SimdTier::Avx2Partial) { if (average) simd::chroma_mc_block_avg_avx2_partial(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_avx2_partial(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
    if (tier==SimdTier::X86V3) { if (average) simd::chroma_mc_block_avg_x86_64_v3(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); else simd::chroma_mc_block_x86_64_v3(dst,dst_stride,rp,rw,bw,bh,q.fx,q.fy,rnd); return true; }
#endif
    return false;
}
#endif
} // namespace

uint8_t Vc1Encoder::intensity_map_y(uint8_t v, const IntensityComp& ic) {
        if (!ic.enabled) return v;
        int scale,shift;
        if (ic.lumscale==0) {
            scale=-64;
            shift=(255-static_cast<int>(ic.lumshift)*2)*64;
            if (ic.lumshift>31) shift += 128*64;
        } else {
            scale=static_cast<int>(ic.lumscale)+32;
            shift=(ic.lumshift>31 ? static_cast<int>(ic.lumshift)-64 : static_cast<int>(ic.lumshift))*64;
        }
        return static_cast<uint8_t>(std::clamp((scale*static_cast<int>(v)+shift+32)>>6,0,255));
    }

uint8_t Vc1Encoder::intensity_map_uv(uint8_t v, const IntensityComp& ic) {
        if (!ic.enabled) return v;
        int scale=ic.lumscale==0 ? -64 : static_cast<int>(ic.lumscale)+32;
        return static_cast<uint8_t>(std::clamp((scale*(static_cast<int>(v)-128)+128*64+32)>>6,0,255));
    }

Frame Vc1Encoder::intensity_compensated_reference(const Frame& ref, const IntensityComp& ic) const {
        if (!ic.enabled) return ref;
        // Every sample is replaced below, so copying the full reference first
        // only doubles memory traffic (and briefly duplicates the complete
        // picture). Allocate the destination storage without copying pixels.
        Frame out;
        out.width=ref.width; out.height=ref.height;
        out.y.resize(ref.y.size()); out.u.resize(ref.u.size()); out.v.resize(ref.v.size());
        int scale,shift;
        if (ic.lumscale==0) { scale=-64; shift=(255-static_cast<int>(ic.lumshift)*2)*64; if (ic.lumshift>31) shift += 128*64; }
        else { scale=static_cast<int>(ic.lumscale)+32; shift=(ic.lumshift>31 ? static_cast<int>(ic.lumshift)-64 : static_cast<int>(ic.lumshift))*64; }
        const SimdTier yt=c_.simd_for(SimdPrimitive::IntensityLuma), ct=c_.simd_for(SimdPrimitive::IntensityChroma);
#if defined(LIBVC1_HAVE_X86_64_V4)
        if (yt==SimdTier::X86V4) simd::intensity_map_plane_x86_64_v4(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
        if (yt==SimdTier::X86V1) simd::intensity_map_plane_x86_64_v1(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        if (yt==SimdTier::Prescott) simd::intensity_map_plane_prescott(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_K10)
        if (yt==SimdTier::K10) simd::intensity_map_plane_k10(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_CONROE)
        if (yt==SimdTier::Conroe) simd::intensity_map_plane_conroe(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        if (yt==SimdTier::X86V2) simd::intensity_map_plane_x86_64_v2(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        if (yt==SimdTier::Penryn) simd::intensity_map_plane_penryn(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        if (yt==SimdTier::SandyBridge) simd::intensity_map_plane_sandybridge(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        if (yt==SimdTier::Bulldozer) simd::intensity_map_plane_bulldozer(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        if (yt==SimdTier::Piledriver) simd::intensity_map_plane_piledriver(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        if (yt==SimdTier::Avx2Partial) simd::intensity_map_plane_avx2_partial(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        if (yt==SimdTier::X86V3) simd::intensity_map_plane_x86_64_v3(out.y.data(),ref.y.data(),out.y.size(),scale,shift); else
#endif
        for (size_t i=0;i<out.y.size();++i) out.y[i]=intensity_map_y(ref.y[i],ic);
#if defined(LIBVC1_HAVE_X86_64_V4)
        if (ct==SimdTier::X86V4) { simd::intensity_map_chroma_plane_x86_64_v4(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_x86_64_v4(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_X86_64_V1)
        if (ct==SimdTier::X86V1) { simd::intensity_map_chroma_plane_x86_64_v1(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_x86_64_v1(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_PRESCOTT)
        if (ct==SimdTier::Prescott) { simd::intensity_map_chroma_plane_prescott(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_prescott(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_K10)
        if (ct==SimdTier::K10) { simd::intensity_map_chroma_plane_k10(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_k10(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_CONROE)
        if (ct==SimdTier::Conroe) { simd::intensity_map_chroma_plane_conroe(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_conroe(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_X86_64_V2)
        if (ct==SimdTier::X86V2) { simd::intensity_map_chroma_plane_x86_64_v2(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_x86_64_v2(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_PENRYN)
        if (ct==SimdTier::Penryn) { simd::intensity_map_chroma_plane_penryn(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_penryn(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_SANDYBRIDGE)
        if (ct==SimdTier::SandyBridge) { simd::intensity_map_chroma_plane_sandybridge(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_sandybridge(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_BULLDOZER)
        if (ct==SimdTier::Bulldozer) { simd::intensity_map_chroma_plane_bulldozer(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_bulldozer(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_PILEDRIVER)
        if (ct==SimdTier::Piledriver) { simd::intensity_map_chroma_plane_piledriver(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_piledriver(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_AVX2_PARTIAL)
        if (ct==SimdTier::Avx2Partial) { simd::intensity_map_chroma_plane_avx2_partial(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_avx2_partial(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
#if defined(LIBVC1_HAVE_X86_64_V3)
        if (ct==SimdTier::X86V3) { simd::intensity_map_chroma_plane_x86_64_v3(out.u.data(),ref.u.data(),out.u.size(),scale); simd::intensity_map_chroma_plane_x86_64_v3(out.v.data(),ref.v.data(),out.v.size(),scale); } else
#endif
        { for (size_t i=0;i<out.u.size();++i) out.u[i]=intensity_map_uv(ref.u[i],ic); for (size_t i=0;i<out.v.size();++i) out.v[i]=intensity_map_uv(ref.v[i],ic); }
        return out;
    }

IntensityComp Vc1Encoder::estimate_intensity_comp(const Frame& f, const Frame& ref,
                                          const std::vector<MotionVector>& mvs) const {
        SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::IntensityComp);
        const int mbw=(c_.width+15)/16, mbh=(c_.height+15)/16;
        if (mvs.size()!=static_cast<size_t>(mbw)*mbh) return {};
        long double sx=0,sy=0,sxx=0,sxy=0;
        uint64_t n=0;
        struct Pair { uint8_t x,y; };
        std::vector<Pair> samples;
        samples.reserve(static_cast<size_t>((c_.width+3)/4)*((c_.height+3)/4));
        for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
            const MotionVector mv=mvs[static_cast<size_t>(my)*mbw+mx];
            const int dx=round_qpel_to_pixel(mv.xq),dy=round_qpel_to_pixel(mv.yq);
            const int x0=mx*16,y0=my*16;
            for (int y=0;y<16 && y0+y<c_.height;y+=4) for (int x=0;x<16 && x0+x<c_.width;x+=4) {
                const int px=x0+x,py=y0+y;
                const uint8_t rv=sample_clamped(ref.y,c_.width,c_.height,px+dx,py+dy);
                const uint8_t cv=f.y[static_cast<size_t>(py)*c_.width+px];
                samples.push_back({rv,cv});
                sx+=rv; sy+=cv; sxx+=static_cast<long double>(rv)*rv; sxy+=static_cast<long double>(rv)*cv; ++n;
            }
        }
        if (n<16) return {};
        const long double denom=static_cast<long double>(n)*sxx-sx*sx;
        long double a=1.0L,b=0.0L;
        if (std::abs(denom)>1e-9L) {
            a=(static_cast<long double>(n)*sxy-sx*sy)/denom;
            b=(sy-a*sx)/static_cast<long double>(n);
        } else {
            b=(sy-sx)/static_cast<long double>(n);
        }
        int ls=std::clamp(static_cast<int>(std::llround(a*64.0L-32.0L)),1,63);
        int sh=std::clamp(static_cast<int>(std::llround(b)),-32,31);
        auto code_shift=[](int v)->uint8_t { return static_cast<uint8_t>(v<0?v+64:v); };
        auto sad_for=[&](int lsc,int shi)->uint64_t {
            IntensityComp ic{true,static_cast<uint8_t>(lsc),code_shift(shi)};
            uint64_t sad=0;
            for (const auto& q:samples) sad+=static_cast<uint64_t>(std::abs(static_cast<int>(q.y)-static_cast<int>(intensity_map_y(q.x,ic))));
            return sad;
        };
        uint64_t best=std::numeric_limits<uint64_t>::max();
        int best_ls=32,best_sh=0;
        for (int l=std::max(1,ls-3);l<=std::min(63,ls+3);++l) {
            for (int z=std::max(-32,sh-3);z<=std::min(31,sh+3);++z) {
                const uint64_t sad=sad_for(l,z);
                if (sad<best || (sad==best && std::abs(l-32)+std::abs(z)<std::abs(best_ls-32)+std::abs(best_sh))) {
                    best=sad; best_ls=l; best_sh=z;
                }
            }
        }
        if (best_ls==32 && best_sh==0) return {};
        return IntensityComp{true,static_cast<uint8_t>(best_ls),code_shift(best_sh)};
    }

uint8_t Vc1Encoder::sample_clamped(const std::vector<uint8_t>& p,int w,int h,int x,int y) {
        x=std::clamp(x,0,w-1); y=std::clamp(y,0,h-1);
        return p[static_cast<size_t>(y)*w+x];
    }

int Vc1Encoder::floor_div4(int v) {
        if (v>=0) return v/4;
        return - ((-v+3)/4);
    }

int Vc1Encoder::mspel_filter4(int a,int b,int c,int d,int mode) {
        if (mode==1) return -4*a+53*b+18*c-3*d;
        if (mode==2) return -a+9*b+9*c-d;
        if (mode==3) return -3*a+18*b+53*c-4*d;
        return b;
    }

uint8_t Vc1Encoder::luma_mc_sample(const std::vector<uint8_t>& p,int w,int h,
                                  int x,int y,int mvq_x,int mvq_y,bool rnd) {
        const int ix=floor_div4(mvq_x), iy=floor_div4(mvq_y);
        const int hm=mvq_x-ix*4, vm=mvq_y-iy*4;
        const int sx=x+ix, sy=y+iy;
        if (!hm && !vm) return sample_clamped(p,w,h,sx,sy);
        auto S=[&](int xx,int yy) { return static_cast<int>(sample_clamped(p,w,h,xx,yy)); };
        int v;
        if (vm && hm) {
            static constexpr int shift_value[4]={0,5,1,5};
            const int shift=(shift_value[hm]+shift_value[vm])>>1;
            const int r1=(1<<(shift-1))+(rnd?1:0)-1;
            int t[4];
            for (int k=-1;k<=2;++k) {
                const int raw=mspel_filter4(S(sx+k,sy-1),S(sx+k,sy),S(sx+k,sy+1),S(sx+k,sy+2),vm);
                t[k+1]=arshift(raw+r1,shift);
            }
            const int raw=mspel_filter4(t[0],t[1],t[2],t[3],hm);
            v=arshift(raw+64-(rnd?1:0),7);
        } else if (vm) {
            const int raw=mspel_filter4(S(sx,sy-1),S(sx,sy),S(sx,sy+1),S(sx,sy+2),vm);
            const int bias=(vm==2)?(7+(rnd?1:0)):(31+(rnd?1:0));
            v=arshift(raw+bias,vm==2?4:6);
        } else {
            const int raw=mspel_filter4(S(sx-1,sy),S(sx,sy),S(sx+1,sy),S(sx+2,sy),hm);
            const int bias=(hm==2)?(8-(rnd?1:0)):(32-(rnd?1:0));
            v=arshift(raw+bias,hm==2?4:6);
        }
        return static_cast<uint8_t>(std::clamp(v,0,255));
    }

uint8_t Vc1Encoder::luma_mc_sample_mode(const std::vector<uint8_t>& p,int w,int h,
                                       int x,int y,int mvq_x,int mvq_y,
                                       ProgressiveMvMode mode,bool rnd) {
        if (!mv_mode_bilinear(mode)) return luma_mc_sample(p,w,h,x,y,mvq_x,mvq_y,rnd);
        const int ix=floor_div4(mvq_x),iy=floor_div4(mvq_y);
        const int fx=mvq_x-ix*4,fy=mvq_y-iy*4;
        const int sx=x+ix,sy=y+iy;
        const int a=sample_clamped(p,w,h,sx,sy);
        if (!fx && !fy) return static_cast<uint8_t>(a);
        const int b=sample_clamped(p,w,h,sx+1,sy);
        const int c=sample_clamped(p,w,h,sx,sy+1);
        if (fx && fy) {
            const int d=sample_clamped(p,w,h,sx+1,sy+1);
            return static_cast<uint8_t>((a+b+c+d+(rnd?1:2))>>2);
        }
        if (fx) return static_cast<uint8_t>((a+b+(rnd?0:1))>>1);
        return static_cast<uint8_t>((a+c+(rnd?0:1))>>1);
    }

uint8_t Vc1Encoder::chroma_mc_sample(const std::vector<uint8_t>& p,int w,int h,
                                    int x,int y,int mvq_x,int mvq_y,
                                    bool b_interp_future,bool rnd) {
        // Decoder-equivalent VC-1 derivation before FASTUVMC. The +1 for a
        // qpel remainder of 3 is significant for direct B MVs that are not
        // integer luma pixels.  There is an easy-to-miss asymmetry in the
        // normative B interpolation path: ff_vc1_mc_1mv() rounds FASTUVMC
        // chroma motion toward an even half-chroma vector, while
        // ff_vc1_interp_mc() rounds the future/reference-1 vector away from
        // that even value. Keep that distinction explicit here.
        int uvx=arshift(mvq_x + ((mvq_x&3)==3),1);
        int uvy=arshift(mvq_y + ((mvq_y&3)==3),1);
        if (b_interp_future) {
            if (uvx<0) uvx -= (uvx&1); else uvx += (uvx&1);
            if (uvy<0) uvy -= (uvy&1); else uvy += (uvy&1);
        } else {
            if (uvx<0) uvx += (uvx&1); else uvx -= (uvx&1);
            if (uvy<0) uvy += (uvy&1); else uvy -= (uvy&1);
        }
        const int ix=floor_div4(uvx), iy=floor_div4(uvy);
        const int fx=(uvx-ix*4)*2;
        const int fy=(uvy-iy*4)*2;
        const int sx=x+ix, sy=y+iy;
        const int a=sample_clamped(p,w,h,sx,sy);
        if (!fx && !fy) return static_cast<uint8_t>(a);
        const int b=sample_clamped(p,w,h,sx+1,sy);
        const int cc=sample_clamped(p,w,h,sx,sy+1);
        const int d=sample_clamped(p,w,h,sx+1,sy+1);
        const int value=a*(8-fx)*(8-fy)+b*fx*(8-fy)+cc*(8-fx)*fy+d*fx*fy;
        return static_cast<uint8_t>((value + (rnd?28:32))>>6);
    }

void Vc1Encoder::predict_luma_motion_block(uint8_t* dst,int dst_stride,const Frame& ref,
                                                int x0,int y0,int bw,int bh,MotionVector mv,
                                                ProgressiveMvMode mode) const {
        if (bw<=0 || bh<=0) return;
        if (mv.xq==0 && mv.yq==0 && x0>=0 && y0>=0 && x0+bw<=ref.width && y0+bh<=ref.height) {
            for (int y=0;y<bh;++y)
                std::copy_n(ref.y.data()+static_cast<size_t>(y0+y)*ref.width+x0,bw,
                            dst+static_cast<ptrdiff_t>(y)*dst_stride);
            return;
        }
        bool done=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None)
            done=try_luma_mc_simd(c_.simd_for(SimdPrimitive::LumaMc),dst,dst_stride,
                                  ref.y,ref.width,ref.height,x0,y0,bw,bh,mv.xq,mv.yq,
                                  c_.rndctrl,mv_mode_bilinear(mode),false);
#endif
        if (!done) {
            for (int y=0;y<bh;++y) for (int x=0;x<bw;++x)
                dst[static_cast<ptrdiff_t>(y)*dst_stride+x]=static_cast<uint8_t>(
                    luma_mc_sample_mode(ref.y,ref.width,ref.height,x0+x,y0+y,
                                        mv.xq,mv.yq,mode,c_.rndctrl));
        }
    }

void Vc1Encoder::motion_compensate_mb(Frame& dst,const Frame& ref,int mx,int my,MotionVector mv, ProgressiveMvMode mode) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionComp);
        const int x0=mx*16,y0=my*16;
        if (mv.xq==0 && mv.yq==0) {
            const int vw=std::max(0,std::min(16,c_.width-x0));
            const int vh=std::max(0,std::min(16,c_.height-y0));
            for (int y=0;y<vh;++y)
                std::copy_n(ref.y.data()+static_cast<size_t>(y0+y)*c_.width+x0,vw,
                            dst.y.data()+static_cast<size_t>(y0+y)*c_.width+x0);
            const int cw=c_.width/2,ch=c_.height/2,cx0=mx*8,cy0=my*8;
            const int cvw=std::max(0,std::min(8,cw-cx0));
            const int cvh=std::max(0,std::min(8,ch-cy0));
            for (int y=0;y<cvh;++y) {
                const size_t off=static_cast<size_t>(cy0+y)*cw+cx0;
                std::copy_n(ref.u.data()+off,cvw,dst.u.data()+off);
                std::copy_n(ref.v.data()+off,cvw,dst.v.data()+off);
            }
            return;
        }
        const int vw=std::max(0,std::min(16,c_.width-x0));
        const int vh=std::max(0,std::min(16,c_.height-y0));
        bool ydone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None)
            ydone=try_luma_mc_simd(c_.simd_for(SimdPrimitive::LumaMc),dst.y.data()+static_cast<size_t>(y0)*c_.width+x0,c_.width,
                                 ref.y,c_.width,c_.height,x0,y0,vw,vh,mv.xq,mv.yq,c_.rndctrl,
                                 mv_mode_bilinear(mode),false);
#endif
        if (!ydone) for (int y=0;y<vh;++y) for (int x=0;x<vw;++x)
            dst.y[static_cast<size_t>(y0+y)*c_.width+x0+x]=
                luma_mc_sample_mode(ref.y,c_.width,c_.height,x0+x,y0+y,mv.xq,mv.yq,mode,c_.rndctrl);

        const int cw=c_.width/2,ch=c_.height/2,cx0=mx*8,cy0=my*8;
        const int cvw=std::max(0,std::min(8,cw-cx0)),cvh=std::max(0,std::min(8,ch-cy0));
        bool udone=false,vdone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::LumaMcAvg)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMcAvg)!=SimdTier::None) {
            udone=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),dst.u.data()+static_cast<size_t>(cy0)*cw+cx0,cw,ref.u,cw,ch,cx0,cy0,cvw,cvh,mv.xq,mv.yq,false,c_.rndctrl,false);
            vdone=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),dst.v.data()+static_cast<size_t>(cy0)*cw+cx0,cw,ref.v,cw,ch,cx0,cy0,cvw,cvh,mv.xq,mv.yq,false,c_.rndctrl,false);
        }
#endif
        if (!udone || !vdone) for (int y=0;y<cvh;++y) for (int x=0;x<cvw;++x) {
            const size_t off=static_cast<size_t>(cy0+y)*cw+cx0+x;
            if (!udone) dst.u[off]=chroma_mc_sample(ref.u,cw,ch,cx0+x,cy0+y,mv.xq,mv.yq,false,c_.rndctrl);
            if (!vdone) dst.v[off]=chroma_mc_sample(ref.v,cw,ch,cx0+x,cy0+y,mv.xq,mv.yq,false,c_.rndctrl);
        }
    }

int Vc1Encoder::median4(int a,int b,int c,int d) {
        if (a<b) {
            if (c<d) return (std::min(b,d)+std::max(a,c))/2;
            return (std::min(b,c)+std::max(a,d))/2;
        }
        if (c<d) return (std::min(a,d)+std::max(b,c))/2;
        return (std::min(a,c)+std::max(b,d))/2;
    }

MotionVector Vc1Encoder::chroma_mv_4mv(const std::array<MotionVector,4>& mv) {
        return MotionVector{median4(mv[0].xq,mv[1].xq,mv[2].xq,mv[3].xq),
                            median4(mv[0].yq,mv[1].yq,mv[2].yq,mv[3].yq)};
    }

void Vc1Encoder::motion_compensate_4mv(Frame& dst,const Frame& ref,int mx,int my,
                               const std::array<MotionVector,4>& mv) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionComp);
        const int x0=mx*16,y0=my*16;
        for (int k=0;k<4;++k) {
            const int bx=x0+(k&1)*8,by=y0+((k>>1)&1)*8;
            const int bw=std::max(0,std::min(8,c_.width-bx)),bh=std::max(0,std::min(8,c_.height-by));
            bool done=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
            if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None)
                done=try_luma_mc_simd(c_.simd_for(SimdPrimitive::LumaMc),dst.y.data()+static_cast<size_t>(by)*c_.width+bx,c_.width,ref.y,c_.width,c_.height,
                                     bx,by,bw,bh,mv[static_cast<size_t>(k)].xq,mv[static_cast<size_t>(k)].yq,c_.rndctrl,false,false);
#endif
            if (!done) for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) {
                const int px=bx+x,py=by+y;
                dst.y[static_cast<size_t>(py)*c_.width+px]=
                    luma_mc_sample(ref.y,c_.width,c_.height,px,py,mv[static_cast<size_t>(k)].xq,
                                   mv[static_cast<size_t>(k)].yq,c_.rndctrl);
            }
        }
        const MotionVector cmv=chroma_mv_4mv(mv);
        const int cw=c_.width/2,ch=c_.height/2,cx0=mx*8,cy0=my*8;
        const int bw=std::max(0,std::min(8,cw-cx0)),bh=std::max(0,std::min(8,ch-cy0));
        bool udone=false,vdone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::LumaMcAvg)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMcAvg)!=SimdTier::None) {
            udone=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),dst.u.data()+static_cast<size_t>(cy0)*cw+cx0,cw,ref.u,cw,ch,cx0,cy0,bw,bh,cmv.xq,cmv.yq,false,c_.rndctrl,false);
            vdone=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),dst.v.data()+static_cast<size_t>(cy0)*cw+cx0,cw,ref.v,cw,ch,cx0,cy0,bw,bh,cmv.xq,cmv.yq,false,c_.rndctrl,false);
        }
#endif
        if (!udone || !vdone) for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) {
            const size_t off=static_cast<size_t>(cy0+y)*cw+cx0+x;
            if (!udone) dst.u[off]=chroma_mc_sample(ref.u,cw,ch,cx0+x,cy0+y,cmv.xq,cmv.yq,false,c_.rndctrl);
            if (!vdone) dst.v[off]=chroma_mc_sample(ref.v,cw,ch,cx0+x,cy0+y,cmv.xq,cmv.yq,false,c_.rndctrl);
        }
    }

void Vc1Encoder::motion_compensate_4mv_padded(Frame& dst,const Frame& ref,int mx,int my,
                                      const std::array<MotionVector,4>& mv) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionComp);
        const int x0=mx*16,y0=my*16;
        for (int k=0;k<4;++k) {
            const int bx=x0+(k&1)*8,by=y0+((k>>1)&1)*8;
            bool done=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
            if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None)
                done=try_luma_mc_simd(c_.simd_for(SimdPrimitive::LumaMc),dst.y.data()+static_cast<size_t>(by)*dst.width+bx,dst.width,
                                     ref.y,ref.width,ref.height,bx,by,8,8,
                                     mv[static_cast<size_t>(k)].xq,mv[static_cast<size_t>(k)].yq,
                                     c_.rndctrl,false,false);
#endif
            if (!done) for (int y=0;y<8;++y) for (int x=0;x<8;++x) {
                const int px=bx+x,py=by+y;
                dst.y[static_cast<size_t>(py)*dst.width+px]=
                    luma_mc_sample(ref.y,ref.width,ref.height,px,py,mv[static_cast<size_t>(k)].xq,
                                   mv[static_cast<size_t>(k)].yq,c_.rndctrl);
            }
        }
        const MotionVector cmv=chroma_mv_4mv(mv);
        const int dcw=dst.width/2,rw=ref.width/2,rh=ref.height/2,cx0=mx*8,cy0=my*8;
        bool udone=false,vdone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::LumaMcAvg)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMcAvg)!=SimdTier::None) {
            udone=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),dst.u.data()+static_cast<size_t>(cy0)*dcw+cx0,dcw,ref.u,rw,rh,
                                    cx0,cy0,8,8,cmv.xq,cmv.yq,false,c_.rndctrl,false);
            vdone=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),dst.v.data()+static_cast<size_t>(cy0)*dcw+cx0,dcw,ref.v,rw,rh,
                                    cx0,cy0,8,8,cmv.xq,cmv.yq,false,c_.rndctrl,false);
        }
#endif
        if (!udone || !vdone) for (int y=0;y<8;++y) for (int x=0;x<8;++x) {
            const int px=cx0+x,py=cy0+y;
            const size_t off=static_cast<size_t>(py)*dcw+px;
            if (!udone) dst.u[off]=chroma_mc_sample(ref.u,rw,rh,px,py,cmv.xq,cmv.yq,false,c_.rndctrl);
            if (!vdone) dst.v[off]=chroma_mc_sample(ref.v,rw,rh,px,py,cmv.xq,cmv.yq,false,c_.rndctrl);
        }
    }

void Vc1Encoder::motion_compensate_mb_padded(Frame& dst,const Frame& ref,int mx,int my,MotionVector mv, ProgressiveMvMode mode) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionComp);
        const int x0=mx*16,y0=my*16;
        if (mv.xq==0 && mv.yq==0 && x0+16<=ref.width && y0+16<=ref.height) {
            for (int y=0;y<16;++y)
                std::copy_n(ref.y.data()+static_cast<size_t>(y0+y)*ref.width+x0,16,
                            dst.y.data()+static_cast<size_t>(y0+y)*dst.width+x0);
            const int dcw=dst.width/2,rw=ref.width/2,cx0=mx*8,cy0=my*8;
            for (int y=0;y<8;++y) {
                const size_t so=static_cast<size_t>(cy0+y)*rw+cx0;
                const size_t doff=static_cast<size_t>(cy0+y)*dcw+cx0;
                std::copy_n(ref.u.data()+so,8,dst.u.data()+doff);
                std::copy_n(ref.v.data()+so,8,dst.v.data()+doff);
            }
            return;
        }
        bool ydone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None)
            ydone=try_luma_mc_simd(c_.simd_for(SimdPrimitive::LumaMc),dst.y.data()+static_cast<size_t>(y0)*dst.width+x0,dst.width,
                                 ref.y,ref.width,ref.height,x0,y0,16,16,mv.xq,mv.yq,c_.rndctrl,
                                 mv_mode_bilinear(mode),false);
#endif
        if (!ydone) for (int y=0;y<16;++y) for (int x=0;x<16;++x) {
            const int px=x0+x,py=y0+y;
            dst.y[static_cast<size_t>(py)*dst.width+px]=
                luma_mc_sample_mode(ref.y,ref.width,ref.height,px,py,mv.xq,mv.yq,mode,c_.rndctrl);
        }
        const int dcw=dst.width/2,rw=ref.width/2,rh=ref.height/2,cx0=mx*8,cy0=my*8;
        bool udone=false,vdone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::LumaMcAvg)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMcAvg)!=SimdTier::None) {
            udone=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),dst.u.data()+static_cast<size_t>(cy0)*dcw+cx0,dcw,ref.u,rw,rh,
                                    cx0,cy0,8,8,mv.xq,mv.yq,false,c_.rndctrl,false);
            vdone=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),dst.v.data()+static_cast<size_t>(cy0)*dcw+cx0,dcw,ref.v,rw,rh,
                                    cx0,cy0,8,8,mv.xq,mv.yq,false,c_.rndctrl,false);
        }
#endif
        if (!udone || !vdone) for (int y=0;y<8;++y) for (int x=0;x<8;++x) {
            const int px=cx0+x,py=cy0+y;
            const size_t off=static_cast<size_t>(py)*dcw+px;
            if (!udone) dst.u[off]=chroma_mc_sample(ref.u,rw,rh,px,py,mv.xq,mv.yq,false,c_.rndctrl);
            if (!vdone) dst.v[off]=chroma_mc_sample(ref.v,rw,rh,px,py,mv.xq,mv.yq,false,c_.rndctrl);
        }
    }

void Vc1Encoder::motion_compensate_bi_mb(Frame& dst,const Frame& past,const Frame& future,
                                 int mx,int my,MotionVector fmv,MotionVector bmv,
                                 ProgressiveMvMode mode) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionComp);
        const int x0=mx*16,y0=my*16;
        const int bw=std::max(0,std::min(16,c_.width-x0));
        const int bh=std::max(0,std::min(16,c_.height-y0));
        bool ydone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::LumaMcAvg)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMcAvg)!=SimdTier::None) {
            uint8_t* dp=dst.y.data()+static_cast<size_t>(y0)*c_.width+x0;
            const bool first=try_luma_mc_simd(c_.simd_for(SimdPrimitive::LumaMc),dp,c_.width,past.y,c_.width,c_.height,x0,y0,bw,bh,
                                             fmv.xq,fmv.yq,c_.rndctrl,mv_mode_bilinear(mode),false);
            const bool second=first && try_luma_mc_simd(c_.simd_for(SimdPrimitive::LumaMcAvg),dp,c_.width,future.y,c_.width,c_.height,x0,y0,bw,bh,
                                                       bmv.xq,bmv.yq,c_.rndctrl,mv_mode_bilinear(mode),true);
            ydone=first && second;
        }
#endif
        if (!ydone) for (int y=0;y<bh;++y) for (int x=0;x<bw;++x) {
            const int px=x0+x,py=y0+y;
            const int a=luma_mc_sample_mode(past.y,c_.width,c_.height,px,py,fmv.xq,fmv.yq,mode,c_.rndctrl);
            const int z=luma_mc_sample_mode(future.y,c_.width,c_.height,px,py,bmv.xq,bmv.yq,mode,c_.rndctrl);
            dst.y[static_cast<size_t>(py)*c_.width+px]=static_cast<uint8_t>((a+z+1)>>1);
        }
        const int cw=c_.width/2,ch=c_.height/2,cx0=mx*8,cy0=my*8;
        const int cbw=std::max(0,std::min(8,cw-cx0)),cbh=std::max(0,std::min(8,ch-cy0));
        bool udone=false,vdone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::LumaMcAvg)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMcAvg)!=SimdTier::None) {
            uint8_t* udp=dst.u.data()+static_cast<size_t>(cy0)*cw+cx0;
            uint8_t* vdp=dst.v.data()+static_cast<size_t>(cy0)*cw+cx0;
            const bool up=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),udp,cw,past.u,cw,ch,cx0,cy0,cbw,cbh,fmv.xq,fmv.yq,false,c_.rndctrl,false);
            const bool uf=up && try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMcAvg),udp,cw,future.u,cw,ch,cx0,cy0,cbw,cbh,bmv.xq,bmv.yq,true,c_.rndctrl,true);
            const bool vp=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),vdp,cw,past.v,cw,ch,cx0,cy0,cbw,cbh,fmv.xq,fmv.yq,false,c_.rndctrl,false);
            const bool vf=vp && try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMcAvg),vdp,cw,future.v,cw,ch,cx0,cy0,cbw,cbh,bmv.xq,bmv.yq,true,c_.rndctrl,true);
            udone=up&&uf; vdone=vp&&vf;
        }
#endif
        if (!udone || !vdone) for (int y=0;y<cbh;++y) for (int x=0;x<cbw;++x) {
            const size_t off=static_cast<size_t>(cy0+y)*cw+cx0+x;
            if (!udone) {
                const int au=chroma_mc_sample(past.u,cw,ch,cx0+x,cy0+y,fmv.xq,fmv.yq,false,c_.rndctrl);
                const int bu=chroma_mc_sample(future.u,cw,ch,cx0+x,cy0+y,bmv.xq,bmv.yq,true,c_.rndctrl);
                dst.u[off]=static_cast<uint8_t>((au+bu+1)>>1);
            }
            if (!vdone) {
                const int av=chroma_mc_sample(past.v,cw,ch,cx0+x,cy0+y,fmv.xq,fmv.yq,false,c_.rndctrl);
                const int bv=chroma_mc_sample(future.v,cw,ch,cx0+x,cy0+y,bmv.xq,bmv.yq,true,c_.rndctrl);
                dst.v[off]=static_cast<uint8_t>((av+bv+1)>>1);
            }
        }
    }

void Vc1Encoder::motion_compensate_bi_mb_padded(Frame& dst,const Frame& past,const Frame& future,
                                        int mx,int my,MotionVector fmv,MotionVector bmv,
                                        ProgressiveMvMode mode) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionComp);
        const int x0=mx*16,y0=my*16;
        bool ydone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::LumaMcAvg)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMcAvg)!=SimdTier::None) {
            uint8_t* dp=dst.y.data()+static_cast<size_t>(y0)*dst.width+x0;
            const bool first=try_luma_mc_simd(c_.simd_for(SimdPrimitive::LumaMc),dp,dst.width,past.y,past.width,past.height,x0,y0,16,16,
                                             fmv.xq,fmv.yq,c_.rndctrl,mv_mode_bilinear(mode),false);
            const bool second=first && try_luma_mc_simd(c_.simd_for(SimdPrimitive::LumaMcAvg),dp,dst.width,future.y,future.width,future.height,x0,y0,16,16,
                                                       bmv.xq,bmv.yq,c_.rndctrl,mv_mode_bilinear(mode),true);
            ydone=first&&second;
        }
#endif
        if (!ydone) for (int y=0;y<16;++y) for (int x=0;x<16;++x) {
            const int px=x0+x,py=y0+y;
            const int a=luma_mc_sample_mode(past.y,past.width,past.height,px,py,fmv.xq,fmv.yq,mode,c_.rndctrl);
            const int z=luma_mc_sample_mode(future.y,future.width,future.height,px,py,bmv.xq,bmv.yq,mode,c_.rndctrl);
            dst.y[static_cast<size_t>(py)*dst.width+px]=static_cast<uint8_t>((a+z+1)>>1);
        }
        const int dcw=dst.width/2,pw=past.width/2,ph=past.height/2,fw=future.width/2,fh=future.height/2,cx0=mx*8,cy0=my*8;
        bool udone=false,vdone=false;
#if defined(LIBVC1_HAVE_X86_64_V1) || defined(LIBVC1_HAVE_X86_64_V2) || defined(LIBVC1_HAVE_X86_64_V3) || defined(LIBVC1_HAVE_X86_64_V4)
        if (c_.simd_for(SimdPrimitive::LumaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMc)!=SimdTier::None || c_.simd_for(SimdPrimitive::LumaMcAvg)!=SimdTier::None || c_.simd_for(SimdPrimitive::ChromaMcAvg)!=SimdTier::None) {
            uint8_t* udp=dst.u.data()+static_cast<size_t>(cy0)*dcw+cx0;
            uint8_t* vdp=dst.v.data()+static_cast<size_t>(cy0)*dcw+cx0;
            const bool up=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),udp,dcw,past.u,pw,ph,cx0,cy0,8,8,fmv.xq,fmv.yq,false,c_.rndctrl,false);
            const bool uf=up && try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMcAvg),udp,dcw,future.u,fw,fh,cx0,cy0,8,8,bmv.xq,bmv.yq,true,c_.rndctrl,true);
            const bool vp=try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMc),vdp,dcw,past.v,pw,ph,cx0,cy0,8,8,fmv.xq,fmv.yq,false,c_.rndctrl,false);
            const bool vf=vp && try_chroma_mc_simd(c_.simd_for(SimdPrimitive::ChromaMcAvg),vdp,dcw,future.v,fw,fh,cx0,cy0,8,8,bmv.xq,bmv.yq,true,c_.rndctrl,true);
            udone=up&&uf; vdone=vp&&vf;
        }
#endif
        if (!udone || !vdone) for (int y=0;y<8;++y) for (int x=0;x<8;++x) {
            const int px=cx0+x,py=cy0+y;
            const size_t off=static_cast<size_t>(py)*dcw+px;
            if (!udone) {
                const int au=chroma_mc_sample(past.u,pw,ph,px,py,fmv.xq,fmv.yq,false,c_.rndctrl);
                const int bu=chroma_mc_sample(future.u,fw,fh,px,py,bmv.xq,bmv.yq,true,c_.rndctrl);
                dst.u[off]=static_cast<uint8_t>((au+bu+1)>>1);
            }
            if (!vdone) {
                const int av=chroma_mc_sample(past.v,pw,ph,px,py,fmv.xq,fmv.yq,false,c_.rndctrl);
                const int bv=chroma_mc_sample(future.v,fw,fh,px,py,bmv.xq,bmv.yq,true,c_.rndctrl);
                dst.v[off]=static_cast<uint8_t>((av+bv+1)>>1);
            }
        }
    }

} // namespace libvc1
