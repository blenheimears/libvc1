#include "encoder_internal.h"
#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <limits>

using libvc1::EncoderConfig;
using libvc1::Frame;
using libvc1::Vc1Encoder;
using MV=Vc1Encoder::MotionVector;
using Mode=Vc1Encoder::ProgressiveMvMode;

static Frame make_frame(int w,int h,int phase) {
    Frame f; f.width=w; f.height=h;
    f.y.resize(static_cast<size_t>(w)*h);
    f.u.resize(static_cast<size_t>(w/2)*(h/2)); f.v.resize(f.u.size());
    for(int y=0;y<h;++y) for(int x=0;x<w;++x)
        f.y[static_cast<size_t>(y)*w+x]=static_cast<uint8_t>((11*x+19*y+phase+((x^y)&7)*21)&255);
    for(size_t i=0;i<f.u.size();++i) { f.u[i]=static_cast<uint8_t>((i*13+phase)&255); f.v[i]=static_cast<uint8_t>((i*7+phase)&255); }
    return f;
}

int main() {
    EncoderConfig c; c.width=160; c.height=120;
    c.motion_search_range=6; c.motion_local_search_range=6;
    c.fade_compensation=false; c.adaptive_quality=false; c.loop_filter=false;
    c.me_quality=VC1_ME_SAD;
    Vc1Encoder e(c);
    auto check=[&](bool pass,const char* what) {
        if(!pass) std::fprintf(stderr,"hidden-reference guard: %s\n",what);
        return pass;
    };
    // The 160x120 display ends half-way through a 16-row coded MB.
    if(!check(!e.motion_uses_hidden_padding(0,112,16,16,{0,0},Mode::OneMvQpel,true),"zero MV rejected") ||
       !check(e.motion_uses_hidden_padding(0,112,16,16,{0,16},Mode::OneMvQpel,true),"bottom +4 luma") ||
       !check(e.motion_uses_hidden_padding(0,112,16,16,{0,1},Mode::OneMvQpel,true),"bottom qpel filter tap") ||
       !check(e.motion_uses_hidden_padding(0,112,16,16,{0,2},Mode::OneMvHpelBilinear,true),"bottom bilinear tap") ||
       !check(!e.motion_uses_hidden_padding(0,96,16,16,{0,0},Mode::OneMvQpel,true),"interior zero MV rejected")) return 1;
    // A full-size display has no distinct coded-padding pixels. Normative
    // interpolation can extend the display edge without referencing black MBs.
    EncoderConfig aligned=c; aligned.height=128;
    Vc1Encoder a(aligned);
    if(!check(!a.motion_uses_hidden_padding(0,112,16,16,{0,16},Mode::OneMvQpel,true),"aligned decoder edge incorrectly rejected")) return 2;
    EncoderConfig right=c; right.width=158; right.height=128;
    Vc1Encoder r(right);
    if(!check(r.motion_uses_hidden_padding(144,0,16,16,{16,0},Mode::OneMvQpel,true),"right padded reference") ||
       !check(!r.motion_uses_hidden_padding(144,0,16,16,{0,0},Mode::OneMvQpel,true),"right visible reference")) return 3;
    // An odd presentation raster has a hidden even YUV420 internal pixel.
    EncoderConfig odd=c; odd.width=66; odd.height=50; odd.display_width=65; odd.display_height=49;
    Vc1Encoder o(odd);
    if(!check(o.motion_uses_hidden_padding(64,32,16,16,{4,0},Mode::OneMvQpel,true),"odd display right pixel") ||
       !check(o.motion_uses_hidden_padding(48,48,16,16,{0,4},Mode::OneMvQpel,true),"odd display bottom pixel") ||
       !check(!o.motion_uses_hidden_padding(64,48,16,16,{0,0},Mode::OneMvQpel,true),"odd display zero MV")) return 4;
    const Frame cur=make_frame(c.width,c.height,17),ref=make_frame(c.width,c.height,9);
    const uint64_t bad=std::numeric_limits<uint64_t>::max()/8;
    if(!check(e.mb_sad(cur,ref,0,7,0,4,1)>bad,"integer motion cost accepted padding") ||
       !check(e.block_sad_qpel(cur,ref,0,112,16,16,{0,1})>bad,"qpel motion cost accepted padding") ||
       !check(e.block_sad_motion(cur,ref,0,112,16,16,{0,2},Mode::OneMvHpelBilinear)>bad,"bilinear cost accepted padding")) return 5;
    // Even a globally predicted or long-range seed must not become a final
    // emitted P vector, regardless of whether integer or subpixel costs win.
    for(auto mode:{Mode::OneMvQpel,Mode::OneMvHpelBilinear,Mode::MixedMv}) {
        const auto p=e.analyze_p_picture_core(cur,ref,mode,false,nullptr);
        const int mbw=(c.width+15)/16, mbh=(c.height+15)/16;
        for(int my=0;my<mbh;++my) for(int mx=0;mx<mbw;++mx) {
            const auto pos=static_cast<size_t>(my)*mbw+mx;
            const int x=mx*16,y=my*16;
            if(p.use_intra[pos]) continue;
            if(p.use_4mv[pos]) {
                for(int k=0;k<4;++k)
                    if(!check(!e.motion_uses_hidden_luma(x+(k&1)*8,y+(k>>1)*8,8,8,p.block_mvs[pos][static_cast<size_t>(k)],Mode::MixedMv),"4MV luma final")) return 6;
                if(!check(!e.motion_uses_hidden_chroma(x,y,16,16,Vc1Encoder::chroma_mv_4mv(p.block_mvs[pos]),false),"4MV chroma final")) return 7;
            } else if(!check(!e.motion_uses_hidden_padding(x,y,16,16,p.mvs[pos],mode,true),"1MV final")) return 8;
        }
    }
    // A direct B vector is derived from the future P anchor.  Both it and the
    // ordinary forward/backward search must respect the same visible crop,
    // including the different FASTUVMC future-reference rounding.
    std::vector<MV> future_mvs(static_cast<size_t>(((c.width+15)/16)*((c.height+15)/16)),MV{0,16});
    std::vector<uint8_t> future_four(future_mvs.size(),0);
    const Vc1Encoder::IntensityComp no_intensity{};
    for(auto mode:{Mode::OneMvQpel,Mode::OneMvHpelBilinear}) {
        const auto b=e.analyze_b_picture_mode(cur,ref,ref,future_mvs,future_four,no_intensity,1,2,mode);
        const int mbw=(c.width+15)/16;
        for(size_t pos=0;pos<b.modes.size();++pos) {
            const int x=static_cast<int>(pos%mbw)*16,y=static_cast<int>(pos/mbw)*16;
            const auto m=b.modes[pos];
            const bool uses_past=m==Vc1Encoder::BMbMode::Forward || m==Vc1Encoder::BMbMode::Direct ||
                                 m==Vc1Encoder::BMbMode::Interpolated;
            const bool uses_future=m==Vc1Encoder::BMbMode::Backward || m==Vc1Encoder::BMbMode::Direct ||
                                   m==Vc1Encoder::BMbMode::Interpolated;
            if(uses_past && !check(!e.motion_uses_hidden_padding(x,y,16,16,b.forward_mvs[pos],mode,true,false),"B forward/direct references hidden padding")) return 9;
            if(uses_future && !check(!e.motion_uses_hidden_padding(x,y,16,16,b.backward_mvs[pos],mode,true,
                                                                   m==Vc1Encoder::BMbMode::Direct || m==Vc1Encoder::BMbMode::Interpolated),
                                     "B future/direct references hidden padding")) return 10;
        }
    }
    std::puts("hidden-reference guard: luma/chroma, fractions, odd crop, P/4MV/B/direct selection passed");
}
