#include "encoder_internal.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace libvc1;

static EncoderConfig main_cfg(int w,int h) {
    EncoderConfig c;
    c.width=w; c.height=h; c.pqindex=8;
    c.syntax=StreamSyntax::Wmv9Main;
    c.loop_filter=true; c.variable_transforms=true;
    c.simd_dispatch.tier.fill(SimdTier::None);
    return c;
}

static Frame frame(int w,int h,uint8_t y=80,uint8_t u=80,uint8_t v=80) {
    Frame f;
    f.width=w; f.height=h;
    f.y.assign(static_cast<size_t>(w)*h,y);
    f.u.assign(static_cast<size_t>(w/2)*(h/2),u);
    f.v.assign(static_cast<size_t>(w/2)*(h/2),v);
    return f;
}

struct Meta {
    std::vector<std::array<Vc1Encoder::MotionVector,4>> mv;
    std::vector<std::array<uint8_t,6>> cbp;
    std::vector<std::array<uint8_t,6>> tt;
    std::vector<uint8_t> intra;
};

static Meta meta(int mbw,int mbh) {
    Meta m;
    const size_t n=static_cast<size_t>(mbw)*mbh;
    m.mv.resize(n);
    m.cbp.resize(n);
    m.tt.resize(n);
    m.intra.assign(n,0);
    return m;
}

static void apply(Vc1Encoder& e,Frame& f,Meta& m,int mbw,int mbh) {
    e.apply_p_loop_filter(f,m.mv,m.cbp,m.tt,m.intra,mbw,mbh);
}

static void fill_grid(Frame& f) {
    for (int y=0;y<f.height;++y) for (int x=0;x<f.width;++x)
        f.y[static_cast<size_t>(y)*f.width+x]=static_cast<uint8_t>(80+4*((x/8+y/8)&1));
    const int cw=f.width/2,ch=f.height/2;
    for (int y=0;y<ch;++y) for (int x=0;x<cw;++x) {
        f.u[static_cast<size_t>(y)*cw+x]=static_cast<uint8_t>(80+4*((x/8+y/8)&1));
        f.v[static_cast<size_t>(y)*cw+x]=static_cast<uint8_t>(84-4*((x/8+y/8)&1));
    }
}

static void test_exception1_and_order() {
    Vc1Encoder e(main_cfg(32,32));
    Frame got=frame(32,32), expected=got;
    fill_grid(got); expected=got;
    Meta m=meta(2,2);
    m.intra[0]=0x3f; // first MB/block 0 intra triggers Main exception 1.
    apply(e,got,m,2,2);
    e.apply_i_like_loop_filter(expected);
    if (got.y!=expected.y || got.u!=expected.u || got.v!=expected.v)
        throw std::runtime_error("Main exception 1/frame-wide filter order differs from all-8x8 reference");
}

static Frame lower_vertical_edge() {
    Frame f=frame(16,16,80,80,80);
    for (int y=8;y<16;++y) for (int x=8;x<16;++x)
        f.y[static_cast<size_t>(y)*16+x]=84;
    return f;
}

static void test_exception2_block1_status() {
    Vc1Encoder e(main_cfg(16,16));
    Meta a=meta(1,1),b=meta(1,1);
    // Lower-right block's left boundary is blocks 2/3 geometrically, but Main
    // exception 2 uses coded/subblock state from blocks 1/3.
    a.cbp[0][1]=0x4; // block 1 drives the first four samples of block 3's left edge.
    b.cbp[0][2]=0x4; // block 2 must be ignored for that coded-status decision.
    Frame fa=lower_vertical_edge(),fb=lower_vertical_edge(),orig=lower_vertical_edge();
    apply(e,fa,a,1,1); apply(e,fb,b,1,1);
    if (fa.y==orig.y)
        throw std::runtime_error("Main exception 2 did not use block 1 coded status");
    // At the target lower half, block-2-only status must not activate the edge.
    for (int y=8;y<16;++y) {
        if (fb.y[static_cast<size_t>(y)*16+7]!=orig.y[static_cast<size_t>(y)*16+7] ||
            fb.y[static_cast<size_t>(y)*16+8]!=orig.y[static_cast<size_t>(y)*16+8])
            throw std::runtime_error("Main exception 2 incorrectly used block 2 coded status");
    }
}

static Frame horizontal_edge() {
    Frame f=frame(16,16,80,80,80);
    for (int y=8;y<16;++y) for (int x=0;x<16;++x)
        f.y[static_cast<size_t>(y)*16+x]=84;
    return f;
}

static void test_exception3_4x4_forces_full_boundary() {
    Vc1Encoder e(main_cfg(16,16));
    Meta m=meta(1,1);
    // Block 2 is a genuinely coded 4x4-transform block. A sparse subblock
    // pattern would normally activate only half of its top edge; Main
    // exception 3 requires the entire 8-sample boundary.
    m.tt[0][2]=7;
    m.cbp[0][2]=0x4;
    Frame f=horizontal_edge(),orig=f;
    apply(e,f,m,1,1);
    // Probe both four-pixel segments at the block-0/block-2 boundary.
    if (f.y[7*16+2]==orig.y[7*16+2] || f.y[8*16+2]==orig.y[8*16+2] ||
        f.y[7*16+6]==orig.y[7*16+6] || f.y[8*16+6]==orig.y[8*16+6])
        throw std::runtime_error("Main exception 3 did not force both 4-pixel segments");
}

static void test_exception3_block3_interaction() {
    Vc1Encoder e(main_cfg(16,16));
    Meta m=meta(1,1);
    // For block 3's left edge, the exception-2/3 interaction tests block 1,
    // not block 2. A coded 4x4 transform in block 1 forces all 8 samples.
    m.tt[0][1]=7;
    m.cbp[0][1]=0x4;
    Frame f=lower_vertical_edge(),orig=f;
    apply(e,f,m,1,1);
    for (int y : {9,13}) {
        if (f.y[static_cast<size_t>(y)*16+7]==orig.y[static_cast<size_t>(y)*16+7] ||
            f.y[static_cast<size_t>(y)*16+8]==orig.y[static_cast<size_t>(y)*16+8])
            throw std::runtime_error("Main exception 2/3 interaction did not force block 3 left edge");
    }
}

static void test_exception4_range_limited_chroma_mv() {
    Vc1Encoder e(main_cfg(32,16));
    Meta m=meta(2,1);
    for (auto& mb:m.mv) for (auto& v:mb) v={-200,0};
    Frame f=frame(32,16,80,80,80),orig=f;
    const int cw=16;
    for (int y=0;y<8;++y) for (int x=8;x<16;++x)
        f.u[static_cast<size_t>(y)*cw+x]=84;
    orig=f;
    apply(e,f,m,2,1);
    bool changed=false;
    for (int y=0;y<8;++y)
        changed |= f.u[static_cast<size_t>(y)*cw+7]!=orig.u[static_cast<size_t>(y)*cw+7] ||
                   f.u[static_cast<size_t>(y)*cw+8]!=orig.u[static_cast<size_t>(y)*cw+8];
    if (!changed)
        throw std::runtime_error("Main exception 4 did not compare range-limited chroma MVs");
}

int main() {
    try {
        test_exception1_and_order();
        test_exception2_block1_status();
        test_exception3_4x4_forces_full_boundary();
        test_exception3_block3_interaction();
        test_exception4_range_limited_chroma_mv();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    std::cout << "Main-profile P-loop compatibility exceptions: ok\n";
    return 0;
}
