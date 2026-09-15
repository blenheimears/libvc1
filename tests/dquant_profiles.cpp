#include "encoder_internal.h"
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

using libvc1::EncoderConfig;
using libvc1::StreamSyntax;
using libvc1::Vc1Encoder;

static bool expect(bool cond, const char* what) {
    if (!cond) std::cerr << "dquant profile test failed: " << what << '\n';
    return cond;
}

int main() {
    EncoderConfig cfg;
    cfg.width=64; cfg.height=64; cfg.pqindex=10; cfg.dquant=true;
    cfg.syntax=StreamSyntax::Advanced;
    Vc1Encoder enc(cfg);
    constexpr int w=4,h=4;
    auto base=[] { return std::vector<int>(w*h,10); };
    auto set_edge=[](std::vector<int>& q, Vc1Encoder::DQuantProfile p, int sel, int alt) {
        for (int y=0;y<h;++y) for (int x=0;x<w;++x)
            if (Vc1Encoder::dquant_edge_member(p,sel,x,y,w,h)) q[static_cast<size_t>(y)*w+x]=alt;
    };

    {
        auto q=base(); auto p=enc.make_dquant_plan(q,w,h);
        if (!expect(!p.enabled,"uniform map must disable DQUANT")) return 1;
    }
    {
        auto q=base(); set_edge(q,Vc1Encoder::DQuantProfile::FourEdges,0,12);
        auto p=enc.make_dquant_plan(q,w,h);
        if (!expect(p.enabled && p.profile==Vc1Encoder::DQuantProfile::FourEdges && p.altpq==12,
                    "four-edge profile")) return 2;
    }
    for (int sel=0;sel<4;++sel) {
        auto q=base(); set_edge(q,Vc1Encoder::DQuantProfile::DoubleEdges,sel,12);
        auto p=enc.make_dquant_plan(q,w,h);
        if (!expect(p.enabled && p.profile==Vc1Encoder::DQuantProfile::DoubleEdges &&
                    p.edge_selector==sel && p.altpq==12,"double-edge profile")) return 10+sel;
    }
    for (int sel=0;sel<4;++sel) {
        auto q=base(); set_edge(q,Vc1Encoder::DQuantProfile::SingleEdge,sel,12);
        auto p=enc.make_dquant_plan(q,w,h);
        if (!expect(p.enabled && p.profile==Vc1Encoder::DQuantProfile::SingleEdge &&
                    p.edge_selector==sel && p.altpq==12,"single-edge profile")) return 20+sel;
    }
    {
        auto q=base(); q[5]=12;
        auto p=enc.make_dquant_plan(q,w,h);
        if (!expect(p.enabled && p.profile==Vc1Encoder::DQuantProfile::AllMbs && p.bilevel && p.altpq==12,
                    "bilevel all-MB profile")) return 30;
        for (size_t i=0;i<q.size();++i) {
            const bool want=i==5;
            if (!expect(Vc1Encoder::dquant_mb_derived(p,i,w)==want,"bilevel derived-state map")) return 31;
        }
    }
    {
        auto q=base(); q[5]=8; q[6]=12;
        auto p=enc.make_dquant_plan(q,w,h);
        if (!expect(p.enabled && p.profile==Vc1Encoder::DQuantProfile::AllMbs && !p.bilevel,
                    "arbitrary all-MB profile")) return 40;
        for (size_t i=0;i<q.size();++i)
            if (!expect(Vc1Encoder::dquant_mb_derived(p,i,w),"arbitrary all-MB derived state")) return 41;
    }
    {
        auto q=base(); set_edge(q,Vc1Encoder::DQuantProfile::SingleEdge,2,8);
        auto p=enc.make_dquant_plan(q,w,h);
        for (int y=0;y<h;++y) for (int x=0;x<w;++x) {
            const size_t i=static_cast<size_t>(y)*w+x;
            const bool want=x==w-1;
            if (!expect(Vc1Encoder::dquant_mb_derived(p,i,w)==want,"edge derived-state map")) return 50;
        }
    }
    // Verify the exact VC-1 header spellings, not merely the profile planner.
    auto header_byte=[&](Vc1Encoder::DQuantProfile profile,int selector,bool bilevel) {
        Vc1Encoder::DQuantPlan p; p.enabled=true; p.profile=profile; p.edge_selector=selector;
        p.bilevel=bilevel; p.altpq=12; p.mquant=base();
        libvc1::BitWriter b; enc.write_dquant_header(b,p);
        auto out=b.finish_raw();
        return out.empty()?0:out[0];
    };
    if (!expect(header_byte(Vc1Encoder::DQuantProfile::FourEdges,0,false)==0x84,"four-edge header bits")) return 60;
    if (!expect(header_byte(Vc1Encoder::DQuantProfile::DoubleEdges,2,false)==0xB1,"double-edge header bits")) return 61;
    if (!expect(header_byte(Vc1Encoder::DQuantProfile::SingleEdge,3,false)==0xD9,"single-edge header bits")) return 62;
    if (!expect(header_byte(Vc1Encoder::DQuantProfile::AllMbs,0,true)==0xF2,"bilevel all-MB header bits")) return 63;
    if (!expect(header_byte(Vc1Encoder::DQuantProfile::AllMbs,0,false)==0xE0,"arbitrary all-MB header bits")) return 64;
    return 0;
}
