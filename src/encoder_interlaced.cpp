#include "encoder_internal.h"
#include <optional>

namespace libvc1 {

namespace {

struct PackedField {
    int yw=0,yh=0,cw=0,ch=0;
    std::vector<uint8_t> y,u,v;
};

struct FieldChoice {
    // direction: 0=past/forward, 1=future/backward. P pictures always use 0.
    int direction=0;
    bool opposite=false;
    const Frame* source=nullptr;
    int ref_field_type=0;
    bool source_field_picture=false;
    uint64_t sad=0;
};

static int field_mb_height(int h) {
    // VC-1 field pictures align the full-frame macroblock height to an even
    // number, then code half of that raster in each field.
    int full=(h+15)/16;
    full=(full+1)&~1;
    return full/2;
}

static int field_lines(int full_height,int field_type) {
    return (full_height + (field_type==0 ? 1 : 0))/2;
}

static int floor_div4_field(int v) {
    return v>=0 ? v/4 : -((-v+3)/4);
}

static int mspel_filter4_field(int a,int b,int c,int d,int mode) {
    if (mode==1) return -4*a+53*b+18*c-3*d;
    if (mode==2) return -a+9*b+9*c-d;
    if (mode==3) return -3*a+18*b+53*c-4*d;
    return b;
}

static int arshift_field(int v,int n) {
    if (v>=0) return v>>n;
    return -static_cast<int>((static_cast<unsigned>(-v)+(1u<<n)-1u)>>n);
}

static uint8_t field_sample(const std::vector<uint8_t>& p,int w,int h,int field_type,int x,int y,
                            bool ref_field_picture) {
    x=std::clamp(x,0,w-1);
    int py=0;
    if (ref_field_picture) {
        // FFmpeg doubles the reference linesize for an interlaced reference,
        // so edge emulation takes place independently inside each parity field.
        const int n=field_lines(h,field_type);
        y=std::clamp(y,0,std::max(0,n-1));
        py=std::min(h-1,2*y+field_type);
    } else {
        // A woven FCM=0 anchor is not marked interlaced by the decoder.  VC-1
        // edge emulation therefore clamps physical full-frame rows first; at
        // the top/bottom boundary that may cross field parity.  0.1.96 instead
        // clamped in field coordinates and diverged from the decoder whenever
        // an opposite-polarity P reference touched an edge.
        py=std::clamp(2*y+field_type,0,h-1);
    }
    return p[static_cast<size_t>(py)*w+x];
}

static uint8_t field_luma_mc_sample(const std::vector<uint8_t>& p,int w,int h,int ref_field_type,
                                    int x,int y,int mvq_x,int mvq_y,bool rnd,bool ref_field_picture) {
    const int ix=floor_div4_field(mvq_x), iy=floor_div4_field(mvq_y);
    const int hm=mvq_x-ix*4, vm=mvq_y-iy*4;
    const int sx=x+ix, sy=y+iy;
    if (!hm && !vm) return field_sample(p,w,h,ref_field_type,sx,sy,ref_field_picture);
    auto S=[&](int xx,int yy) { return static_cast<int>(field_sample(p,w,h,ref_field_type,xx,yy,ref_field_picture)); };
    int v=0;
    if (vm && hm) {
        static constexpr int shift_value[4]={0,5,1,5};
        const int shift=(shift_value[hm]+shift_value[vm])>>1;
        const int r1=(1<<(shift-1))+(rnd?1:0)-1;
        int t[4];
        for (int k=-1;k<=2;++k) {
            const int raw=mspel_filter4_field(S(sx+k,sy-1),S(sx+k,sy),S(sx+k,sy+1),S(sx+k,sy+2),vm);
            t[k+1]=arshift_field(raw+r1,shift);
        }
        const int raw=mspel_filter4_field(t[0],t[1],t[2],t[3],hm);
        v=arshift_field(raw+64-(rnd?1:0),7);
    } else if (vm) {
        const int raw=mspel_filter4_field(S(sx,sy-1),S(sx,sy),S(sx,sy+1),S(sx,sy+2),vm);
        const int bias=(vm==2)?(7+(rnd?1:0)):(31+(rnd?1:0));
        v=arshift_field(raw+bias,vm==2?4:6);
    } else {
        const int raw=mspel_filter4_field(S(sx-1,sy),S(sx,sy),S(sx+1,sy),S(sx+2,sy),hm);
        const int bias=(hm==2)?(8-(rnd?1:0)):(32-(rnd?1:0));
        v=arshift_field(raw+bias,hm==2?4:6);
    }
    return static_cast<uint8_t>(std::clamp(v,0,255));
}

static uint8_t field_chroma_mc_sample(const std::vector<uint8_t>& p,int w,int h,int ref_field_type,
                                      int x,int y,int mvq_x,int mvq_y,bool rnd,bool ref_field_picture) {
    // ff_vc1_mc_1mv derives FASTUVMC chroma motion from the stored luma MV
    // before the field-parity correction is applied. The caller therefore
    // passes the already-derived chroma vector here.
    const int ix=floor_div4_field(mvq_x), iy=floor_div4_field(mvq_y);
    const int fx=(mvq_x-ix*4)*2, fy=(mvq_y-iy*4)*2;
    const int sx=x+ix, sy=y+iy;
    const int a=field_sample(p,w,h,ref_field_type,sx,sy,ref_field_picture);
    if (!fx && !fy) return static_cast<uint8_t>(a);
    const int b=field_sample(p,w,h,ref_field_type,sx+1,sy,ref_field_picture);
    const int c=field_sample(p,w,h,ref_field_type,sx,sy+1,ref_field_picture);
    const int d=field_sample(p,w,h,ref_field_type,sx+1,sy+1,ref_field_picture);
    const int value=a*(8-fx)*(8-fy)+b*fx*(8-fy)+c*(8-fx)*fy+d*fx*fy;
    return static_cast<uint8_t>(std::clamp((value+(rnd?28:32))>>6,0,255));
}

static Vc1Encoder::MotionVector field_mc_vector(Vc1Encoder::MotionVector stored,int cur_field_type,int ref_field_type) {
    if (cur_field_type!=ref_field_type) stored.yq+=-2+4*cur_field_type;
    return stored;
}

static Vc1Encoder::MotionVector field_chroma_vector(Vc1Encoder::MotionVector stored,int cur_field_type,int ref_field_type) {
    Vc1Encoder::MotionVector uv;
    // Match ff_vc1_mc_1mv exactly: derive the chroma MV from the stored luma
    // vector, apply the field-parity correction, and only then apply FASTUVMC
    // rounding toward an even half-chroma vector.  The ordering matters when
    // a negative vertical vector crosses zero after the parity adjustment
    // (for example stored my=-2 in the second field: -1 + 2 -> +1 -> 0).
    uv.xq=arshift_field(stored.xq+((stored.xq&3)==3),1);
    uv.yq=arshift_field(stored.yq+((stored.yq&3)==3),1);
    if (cur_field_type!=ref_field_type) uv.yq+=-2+4*cur_field_type;
    if (uv.xq<0) uv.xq+=(uv.xq&1); else uv.xq-=(uv.xq&1);
    if (uv.yq<0) uv.yq+=(uv.yq&1); else uv.yq-=(uv.yq&1);
    return uv;
}

static uint64_t field_mb_sad(const PackedField& src,int mx,int my,int cur_field_type,
                             const Frame& ref,int ref_field_type,Vc1Encoder::MotionVector mv,
                             bool rnd,bool ref_field_picture,int stride=1) {
    uint64_t sad=0;
    const auto mc=field_mc_vector(mv,cur_field_type,ref_field_type);
    const int x0=mx*16,y0=my*16;
    for (int y=0;y<16;y+=stride) for (int x=0;x<16;x+=stride) {
        const int pred=field_luma_mc_sample(ref.y,ref.width,ref.height,ref_field_type,
                                            x0+x,y0+y,mc.xq,mc.yq,rnd,ref_field_picture);
        sad+=static_cast<uint64_t>(std::abs(int(src.y[static_cast<size_t>(y0+y)*src.yw+x0+x])-pred));
    }
    return sad;
}

static void predict_field_mb(PackedField& dst,int mx,int my,int cur_field_type,
                             const Frame& ref,int ref_field_type,Vc1Encoder::MotionVector mv,
                             bool rnd,bool ref_field_picture) {
    const auto mc=field_mc_vector(mv,cur_field_type,ref_field_type);
    const auto uv=field_chroma_vector(mv,cur_field_type,ref_field_type);
    const int x0=mx*16,y0=my*16;
    for (int y=0;y<16;++y) for (int x=0;x<16;++x)
        dst.y[static_cast<size_t>(y0+y)*dst.yw+x0+x]=
            field_luma_mc_sample(ref.y,ref.width,ref.height,ref_field_type,
                                 x0+x,y0+y,mc.xq,mc.yq,rnd,ref_field_picture);
    const int cx0=mx*8,cy0=my*8;
    for (int y=0;y<8;++y) for (int x=0;x<8;++x) {
        const size_t off=static_cast<size_t>(cy0+y)*dst.cw+cx0+x;
        dst.u[off]=field_chroma_mc_sample(ref.u,ref.width/2,ref.height/2,ref_field_type,
                                          cx0+x,cy0+y,uv.xq,uv.yq,rnd,ref_field_picture);
        dst.v[off]=field_chroma_mc_sample(ref.v,ref.width/2,ref.height/2,ref_field_type,
                                          cx0+x,cy0+y,uv.xq,uv.yq,rnd,ref_field_picture);
    }
}


static void append_bytes(std::vector<uint8_t>& out,const std::vector<uint8_t>& add) {
    out.insert(out.end(),add.begin(),add.end());
}

static void write_bfraction(BitWriter& b,int num,int den) {
    // Match the established progressive encoder mapping: 1/2, 1/3, 2/3.
    if (den==2) b.bits(0b000,3);
    else if (num==1) b.bits(0b001,3);
    else b.bits(0b010,3);
}

} // namespace


namespace {

struct FieldMbSyntax {
    int direction=0; // 0=forward/past, 1=backward/future
    bool opposite=false;
    Vc1Encoder::MotionVector mv{};
    int dmv_x=0,dmv_y=0;
    bool pred_flag=false;
    bool hybrid_present=false;
    bool hybrid_bit=false;
    uint8_t cbp=0;
    int mquant=0;
    std::array<std::array<int,64>,6> q{};
};

struct FieldPredState {
    std::array<std::vector<Vc1Encoder::MotionVector>,2> mv;
    std::array<std::vector<uint8_t>,2> opposite;
    FieldPredState(size_t n=0) {
        for (int d=0;d<2;++d) { mv[d].assign(n,{}); opposite[d].assign(n,0); }
    }
};

struct FieldPredInfo {
    Vc1Encoder::MotionVector pred{};
    int dmv_x=0,dmv_y=0;
    bool pred_flag=false;
    bool hybrid_present=false;
    bool hybrid_bit=false;
};

static int signed_mod_field(int v,int r) {
    const int span=r*2;
    int x=(v+r)%span;
    if (x<0) x+=span;
    return x-r;
}

static int field_clip_scaled_y(int v,int range_y,int cur,int ref) {
    if (cur && !ref) return std::clamp(v,-range_y/2+1,range_y/2);
    return std::clamp(v,-range_y/2,range_y/2-1);
}

static int pfield_scale_same_component(int n,bool y,int range_x,int range_y,int cur,int ref) {
    const int az=std::abs(n);
    int v=0;
    if (!y) {
        if (az>255) v=n;
        else if (az<32) v=arshift_field(n*512,8);
        else v=arshift_field(n*219,8)+(n<0?-37:37);
        return std::clamp(v,-range_x,range_x-1);
    }
    if (az>63) v=n;
    else if (az<8) v=arshift_field(n*512,8);
    else v=arshift_field(n*219,8)+(n<0?-10:10);
    return field_clip_scaled_y(v,range_y,cur,ref);
}

static int pfield_scale_opp_component(int n,bool y,int range_x,int range_y,int cur,int ref) {
    // REFDIST/FRFD=0: SCALEOPP=128 for both field orders.
    const int v=arshift_field(n*128,8);
    return y?field_clip_scaled_y(v,range_y,cur,ref):std::clamp(v,-range_x,range_x-1);
}

static int b_first_backward_scale_same_component(int n,bool y,int range_x,int range_y,int cur,int ref) {
    const int v=arshift_field(n*171,8);
    return y?field_clip_scaled_y(v,range_y,cur,ref):std::clamp(v,-range_x,range_x-1);
}

static int b_first_backward_scale_opp_component(int n,bool y,int range_x,int range_y,int cur,int ref) {
    const int az=std::abs(n);
    int v=0;
    if (!y) {
        if (az>255) v=n;
        else if (az<43) v=arshift_field(n*384,8);
        else v=arshift_field(n*230,8)+(n<0?-26:26);
        return std::clamp(v,-range_x,range_x-1);
    }
    if (az>63) v=n;
    else if (az<11) v=arshift_field(n*384,8);
    else v=arshift_field(n*230,8)+(n<0?-7:7);
    return field_clip_scaled_y(v,range_y,cur,ref);
}

static Vc1Encoder::MotionVector scale_neighbor_for_field(Vc1Encoder::MotionVector v,bool neighbor_opp,bool desired_opp,
                                                         bool is_b,int field_index,int dir,
                                                         int range_x,int range_y,int cur) {
    if (neighbor_opp==desired_opp) return v;
    const int ref=desired_opp?1-cur:cur;
    const bool b_special=is_b && field_index==0 && dir==1;
    if (desired_opp) {
        if (b_special) {
            v.xq=b_first_backward_scale_opp_component(v.xq,false,range_x,range_y,cur,ref);
            v.yq=b_first_backward_scale_opp_component(v.yq,true,range_x,range_y,cur,ref);
        } else {
            v.xq=pfield_scale_opp_component(v.xq,false,range_x,range_y,cur,ref);
            v.yq=pfield_scale_opp_component(v.yq,true,range_x,range_y,cur,ref);
        }
    } else {
        if (b_special) {
            v.xq=b_first_backward_scale_same_component(v.xq,false,range_x,range_y,cur,ref);
            v.yq=b_first_backward_scale_same_component(v.yq,true,range_x,range_y,cur,ref);
        } else {
            v.xq=pfield_scale_same_component(v.xq,false,range_x,range_y,cur,ref);
            v.yq=pfield_scale_same_component(v.yq,true,range_x,range_y,cur,ref);
        }
    }
    return v;
}

static FieldPredInfo field_pred_for_target(FieldPredState& st,int mbw,int mx,int my,int dir,
                                           bool desired_opp,Vc1Encoder::MotionVector target,
                                           bool is_b,int field_index,int cur,int range_x,int range_y,
                                           bool update_state=true) {
    const bool av=my>0;
    const bool cv=mx>0;
    const bool bv=my>0 && mbw>1;
    const size_t pos=static_cast<size_t>(my)*mbw+mx;
    const size_t apos=av?static_cast<size_t>(my-1)*mbw+mx:0;
    const size_t cpos=cv?pos-1:0;
    const int bx=(mx==mbw-1)?std::max(0,mx-1):mx+1;
    const size_t bpos=bv?static_cast<size_t>(my-1)*mbw+bx:0;
    Vc1Encoder::MotionVector A{},B{},C{};
    bool af=false,bf=false,cf=false;
    int ns=0,no=0;
    if (av) { A=st.mv[dir][apos]; af=st.opposite[dir][apos]!=0; (af?no:ns)++; }
    if (bv) { B=st.mv[dir][bpos]; bf=st.opposite[dir][bpos]!=0; (bf?no:ns)++; }
    if (cv) { C=st.mv[dir][cpos]; cf=st.opposite[dir][cpos]!=0; (cf?no:ns)++; }
    FieldPredInfo out;
    out.pred_flag=(ns<=no)?!desired_opp:desired_opp;
    if (av) A=scale_neighbor_for_field(A,af,desired_opp,is_b,field_index,dir,range_x,range_y,cur);
    if (bv) B=scale_neighbor_for_field(B,bf,desired_opp,is_b,field_index,dir,range_x,range_y,cur);
    if (cv) C=scale_neighbor_for_field(C,cf,desired_opp,is_b,field_index,dir,range_x,range_y,cur);
    if (av) out.pred=A;
    else if (cv) out.pred=C;
    else if (bv) out.pred=B;
    if (ns+no>1) {
        out.pred.xq=Vc1Encoder::median3(A.xq,B.xq,C.xq);
        out.pred.yq=Vc1Encoder::median3(A.yq,B.yq,C.yq);
    }
    // Field P pictures retain VC-1 hybrid prediction. B-field pictures skip it.
    if (!is_b && av && cv) {
        // VC-1 8.3.5.3.5: compare the median predictor with A first; if
        // that is close enough, perform the independent C-neighbour test.
        // A HYBRIDPRED bit is present when either comparison exceeds 32.
        const int suma=std::abs(out.pred.xq-A.xq)+std::abs(out.pred.yq-A.yq);
        const int sumc=std::abs(out.pred.xq-C.xq)+std::abs(out.pred.yq-C.yq);
        if (suma>32 || sumc>32) {
            out.hybrid_present=true;
            const int da=std::abs(signed_mod_field(target.xq-A.xq,range_x))+
                         std::abs(signed_mod_field(target.yq-A.yq,range_y/2));
            const int dc=std::abs(signed_mod_field(target.xq-C.xq,range_x))+
                         std::abs(signed_mod_field(target.yq-C.yq,range_y/2));
            // Decoder bit 1 selects A; bit 0 selects C.
            out.hybrid_bit=da<=dc;
            out.pred=out.hybrid_bit?A:C;
        }
    }
    out.dmv_x=signed_mod_field(target.xq-out.pred.xq,range_x);
    // y_bias shifts the signed-modulus origin in the decoder, but cancels
    // when solving the differential for a desired stored MV.
    out.dmv_y=signed_mod_field(target.yq-out.pred.yq,range_y/2);
    if (update_state) {
        st.mv[dir][pos]=target;
        st.opposite[dir][pos]=desired_opp?1:0;
    }
    return out;
}

static void field_apply_b_unused(FieldPredState& st,int mbw,int mx,int my,int dir,int field_index,int cur,
                                 int range_x,int range_y) {
    const bool av=my>0,cv=mx>0,bv=my>0&&mbw>1;
    const size_t pos=static_cast<size_t>(my)*mbw+mx;
    int ns=0,no=0;
    auto count=[&](size_t q){ if (st.opposite[dir][q]) ++no; else ++ns; };
    if (av) count(static_cast<size_t>(my-1)*mbw+mx);
    if (bv) { const int bx=(mx==mbw-1)?std::max(0,mx-1):mx+1; count(static_cast<size_t>(my-1)*mbw+bx); }
    if (cv) count(pos-1);
    // Decoder call uses pred_flag=0 for the unused direction.
    const bool desired_opp=(ns<=no);
    auto info=field_pred_for_target(st,mbw,mx,my,dir,desired_opp,{},true,field_index,cur,range_x,range_y,false);
    const int ybias=(cur&&desired_opp)?1:0;
    Vc1Encoder::MotionVector decoded;
    decoded.xq=signed_mod_field(info.pred.xq,range_x);
    decoded.yq=signed_mod_field(info.pred.yq-ybias,range_y/2)+ybias;
    st.mv[dir][pos]=decoded;
    st.opposite[dir][pos]=desired_opp?1:0;
}

static Vc1Encoder::MotionVector search_field_mb(const PackedField& src,int mx,int my,int cur,
                                                const Frame& ref,int ref_field,bool rnd,bool ref_field_picture,
                                                Vc1Encoder::MotionVector seed,int local_pixels,
                                                int range_x,int range_y) {
    const int maxx=std::min(range_x-1,std::max(0,local_pixels)*4);
    const int maxy=std::min(range_y/2-1,std::max(0,local_pixels)*4);
    auto clampmv=[&](Vc1Encoder::MotionVector v){
        v.xq=std::clamp(v.xq,-maxx,maxx);
        v.yq=std::clamp(v.yq,-maxy,maxy);
        return v;
    };
    auto cost=[&](Vc1Encoder::MotionVector v,int stride){
        // Keep field-mode 1-MV prediction on the decoder's ordinary reference
        // path. FFmpeg switches outer macroblocks to an interlaced edge scratch
        // buffer whose row/parity layout is intentionally different from the
        // normative field sampler. An encoder cannot use its own direct field
        // samples there and still retain decoder-equivalent references. Choose
        // a small inward MV at picture edges instead; the residual carries the
        // source difference exactly. This applies to stored zero MVs too,
        // because opposite-field parity correction can make their effective
        // vertical vector fractional/nonzero.
        const auto mc=field_mc_vector(v,cur,ref_field);
        const int ix=floor_div4_field(mc.xq), iy=floor_div4_field(mc.yq);
        const int sx=mx*16+ix, sy=my*16+iy;
        const int fx=mc.xq-ix*4, fy=mc.yq-iy*4;
        const int edge_h=src.yh; // coded field height, including MB padding.
        const bool interior = ref.width>=22 && edge_h>=22 &&
                              sx>=1 && sy>=1 &&
                              sx<=ref.width-fx-18 && sy<=edge_h-fy-18;
        if (!interior) return std::numeric_limits<uint64_t>::max()/4;
        return field_mb_sad(src,mx,my,cur,ref,ref_field,v,rnd,ref_field_picture,stride);
    };
    Vc1Encoder::MotionVector best=clampmv(seed);
    uint64_t bestc=cost(best,2);
    const Vc1Encoder::MotionVector zero{};
    const auto zc=cost(zero,2);
    if (zc<bestc || (zc==bestc && (std::abs(zero.xq)+std::abs(zero.yq)<std::abs(best.xq)+std::abs(best.yq)))) { best=zero; bestc=zc; }
    for (int step=16;step>=1;step/=2) {
        const int qstep=step*4;
        bool changed=true;
        for (int iter=0;changed && iter<12;++iter) {
            changed=false;
            const std::array<Vc1Encoder::MotionVector,8> d={{{qstep,0},{-qstep,0},{0,qstep},{0,-qstep},{qstep,qstep},{qstep,-qstep},{-qstep,qstep},{-qstep,-qstep}}};
            for (auto dv:d) {
                auto v=clampmv({best.xq+dv.xq,best.yq+dv.yq});
                const auto c=cost(v,2);
                if (c<bestc || (c==bestc && std::abs(v.xq)+std::abs(v.yq)<std::abs(best.xq)+std::abs(best.yq))) { best=v; bestc=c; changed=true; }
            }
        }
    }
    // Quarter-pel refinement with full 16x16 luma SAD.
    uint64_t full=cost(best,1);
    for (int pass=0;pass<2;++pass) {
        Vc1Encoder::MotionVector winner=best; uint64_t wc=full;
        for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx) if (dx||dy) {
            auto v=clampmv({best.xq+dx,best.yq+dy});
            const auto c=cost(v,1);
            if (c<wc || (c==wc && std::abs(v.xq)+std::abs(v.yq)<std::abs(winner.xq)+std::abs(winner.yq))) { winner=v; wc=c; }
        }
        if (winner.xq==best.xq && winner.yq==best.yq) break;
        best=winner; full=wc;
    }
    return best;
}

static void write_field_mvdata_escape(BitWriter& b,int dmv_x,int dmv_y,bool pred_flag,int kx,int ky) {
    // 2-reference IMVTAB 0 escape symbol 125.
    b.vlc(0x0A38,13);
    const uint32_t mxmask=(1u<<kx)-1u, mymask=(1u<<ky)-1u;
    b.bits(static_cast<uint32_t>(dmv_x)&mxmask,kx);
    const int raw_y=2*dmv_y-(pred_flag?1:0);
    b.bits(static_cast<uint32_t>(raw_y)&mymask,ky);
}

static PackedField pack_source_field(const Frame& f,int field_type,int mbw,int mbh) {
    PackedField out;
    out.yw=mbw*16; out.yh=mbh*16;
    out.cw=mbw*8;  out.ch=mbh*8;
    out.y.resize(static_cast<size_t>(out.yw)*out.yh);
    out.u.resize(static_cast<size_t>(out.cw)*out.ch);
    out.v.resize(static_cast<size_t>(out.cw)*out.ch);
    const int yl=std::max(1,field_lines(f.height,field_type));
    const int fw=std::max(1,f.width);
    for (int y=0;y<out.yh;++y) {
        const int fy=std::min(y,yl-1);
        const int py=std::min(f.height-1,2*fy+field_type);
        for (int x=0;x<out.yw;++x) {
            const int sx=std::min(x,fw-1);
            out.y[static_cast<size_t>(y)*out.yw+x]=f.y[static_cast<size_t>(py)*f.width+sx];
        }
    }
    const int src_cw=std::max(1,f.width/2),src_ch=std::max(1,f.height/2);
    const int cl=std::max(1,field_lines(src_ch,field_type));
    for (int y=0;y<out.ch;++y) {
        const int fy=std::min(y,cl-1);
        const int py=std::min(src_ch-1,2*fy+field_type);
        for (int x=0;x<out.cw;++x) {
            const int sx=std::min(x,src_cw-1);
            out.u[static_cast<size_t>(y)*out.cw+x]=f.u[static_cast<size_t>(py)*src_cw+sx];
            out.v[static_cast<size_t>(y)*out.cw+x]=f.v[static_cast<size_t>(py)*src_cw+sx];
        }
    }
    return out;
}

static void weave_packed_field(Frame& dst,int field_type,const PackedField& src) {
    const int yl=field_lines(dst.height,field_type);
    for (int y=0;y<yl;++y) {
        const int py=2*y+field_type;
        if (py>=dst.height) break;
        for (int x=0;x<dst.width;++x)
            dst.y[static_cast<size_t>(py)*dst.width+x]=src.y[static_cast<size_t>(y)*src.yw+x];
    }
    const int cw=dst.width/2,ch=dst.height/2;
    const int cl=field_lines(ch,field_type);
    for (int y=0;y<cl;++y) {
        const int py=2*y+field_type;
        if (py>=ch) break;
        for (int x=0;x<cw;++x) {
            dst.u[static_cast<size_t>(py)*cw+x]=src.u[static_cast<size_t>(y)*src.cw+x];
            dst.v[static_cast<size_t>(py)*cw+x]=src.v[static_cast<size_t>(y)*src.cw+x];
        }
    }
}


static void fill_field_debug_quality(vc1_mb_debug_t& d,const PackedField& src,const PackedField& rec,
                                     const PackedField* previous,const PackedField* prediction,
                                     int mx,int my,int visible_width,int visible_field_lines,
                                     int visible_chroma_width,int visible_chroma_field_lines) {
    const int x0=mx*16,y0=my*16;
    const int bw=std::max(0,std::min(16,visible_width-x0));
    const int bh=std::max(0,std::min(16,visible_field_lines-y0));
    d.i_mb_x=static_cast<uint32_t>(mx); d.i_mb_y=static_cast<uint32_t>(my);
    d.i_visible_width=static_cast<uint32_t>(bw); d.i_visible_height=static_cast<uint32_t>(bh);
    auto metrics=[](const std::vector<uint8_t>& a,const std::vector<uint8_t>& b,int stride,
                    int bx,int by,int w,int h,double& sad,double& mse) {
        if (w<=0 || h<=0) { sad=-1.0; mse=-1.0; return; }
        long double ae=0.0,se=0.0;
        for(int y=0;y<h;++y)for(int x=0;x<w;++x){
            const int e=static_cast<int>(a[static_cast<size_t>(by+y)*stride+bx+x])-static_cast<int>(b[static_cast<size_t>(by+y)*stride+bx+x]);
            ae+=std::abs(e); se+=static_cast<long double>(e)*e;
        }
        sad=static_cast<double>(ae); mse=static_cast<double>(se/(static_cast<long double>(w)*h));
    };
    long double sum=0.0,sum2=0.0,activity=0.0;
    for(int y=0;y<bh;++y)for(int x=0;x<bw;++x){
        const int v=src.y[static_cast<size_t>(y0+y)*src.yw+x0+x];
        sum+=v; sum2+=static_cast<long double>(v)*v;
        if(x)activity+=std::abs(v-static_cast<int>(src.y[static_cast<size_t>(y0+y)*src.yw+x0+x-1]));
        if(y)activity+=std::abs(v-static_cast<int>(src.y[static_cast<size_t>(y0+y-1)*src.yw+x0+x]));
    }
    const long double n=static_cast<long double>(std::max(1,bw*bh));
    d.f_mean_y=static_cast<double>(sum/n);
    d.f_stddev_y=std::sqrt(static_cast<double>(std::max<long double>(0.0,sum2/n-(sum/n)*(sum/n))));
    const long double edges=static_cast<long double>(std::max(1,bh*std::max(0,bw-1)+bw*std::max(0,bh-1)));
    d.f_activity_y=static_cast<double>(activity/edges);
    const int cx0=mx*8,cy0=my*8;
    const int cbw=std::max(0,std::min(8,visible_chroma_width-cx0));
    const int cbh=std::max(0,std::min(8,visible_chroma_field_lines-cy0));
    auto mean=[&](const std::vector<uint8_t>& p){long double v=0;for(int y=0;y<cbh;++y)for(int x=0;x<cbw;++x)v+=p[static_cast<size_t>(cy0+y)*src.cw+cx0+x];return static_cast<double>(v/std::max<long double>(1.0,static_cast<long double>(cbw)*cbh));};
    d.f_mean_u=mean(src.u); d.f_mean_v=mean(src.v);
    d.f_previous_sad_y=d.f_previous_mse_y=-1.0;
    if(previous)metrics(src.y,previous->y,src.yw,x0,y0,bw,bh,d.f_previous_sad_y,d.f_previous_mse_y);
    d.f_prediction_sad_y=d.f_prediction_mse_y=d.f_prediction_gain_db=-1.0;
    if(prediction){
        metrics(src.y,prediction->y,src.yw,x0,y0,bw,bh,d.f_prediction_sad_y,d.f_prediction_mse_y);
        if(d.f_previous_mse_y>0.0 && d.f_prediction_mse_y>=0.0)
            d.f_prediction_gain_db=d.f_prediction_mse_y==0.0?99.0:10.0*std::log10(d.f_previous_mse_y/d.f_prediction_mse_y);
    }
    double dummy=0.0;
    metrics(src.y,rec.y,src.yw,x0,y0,bw,bh,dummy,d.f_recon_mse_y);
    metrics(src.u,rec.u,src.cw,cx0,cy0,cbw,cbh,dummy,d.f_recon_mse_u);
    metrics(src.v,rec.v,src.cw,cx0,cy0,cbw,cbh,dummy,d.f_recon_mse_v);
    const double ny=static_cast<double>(bw*bh),nc=static_cast<double>(cbw*cbh);
    d.f_recon_mse_yuv=(d.f_recon_mse_y*ny+(d.f_recon_mse_u+d.f_recon_mse_v)*nc)/std::max(1.0,ny+2.0*nc);
    d.f_recon_psnr_y_db=d.f_recon_mse_y==0.0?99.0:10.0*std::log10((255.0*255.0)/d.f_recon_mse_y);
    d.f_recon_psnr_yuv_db=d.f_recon_mse_yuv==0.0?99.0:10.0*std::log10((255.0*255.0)/d.f_recon_mse_yuv);
    long double chroma_signal=0.0;
    for(int y=0;y<cbh;++y)for(int x=0;x<cbw;++x){const int u=src.u[static_cast<size_t>(cy0+y)*src.cw+cx0+x],v=src.v[static_cast<size_t>(cy0+y)*src.cw+cx0+x];chroma_signal+=static_cast<long double>(u)*u+static_cast<long double>(v)*v;}
    const double sigy=static_cast<double>(sum2/n);
    const double sigall=(static_cast<double>(sum2)+static_cast<double>(chroma_signal))/std::max(1.0,ny+2.0*nc);
    d.f_recon_snr_y_db=d.f_recon_mse_y==0.0?99.0:(sigy>0?10.0*std::log10(sigy/d.f_recon_mse_y):-99.0);
    d.f_recon_snr_yuv_db=d.f_recon_mse_yuv==0.0?99.0:(sigall>0?10.0*std::log10(sigall/d.f_recon_mse_yuv):-99.0);
}

static Vc1Encoder::PerceptualMbPriority field_perceptual_priority(const PackedField& src,int mx,int my,
                                                                  int visible_width,int visible_field_lines,
                                                                  int visible_chroma_width,int visible_chroma_field_lines,
                                                                  double aq_strength,bool intra) {
    Vc1Encoder::PerceptualMbPriority out;
    if(aq_strength<=0.0)return out;
    const int x0=mx*16,y0=my*16,bw=std::max(0,std::min(16,visible_width-x0)),bh=std::max(0,std::min(16,visible_field_lines-y0));
    if(bw<=0||bh<=0)return out;
    long double ys=0.0,g=0.0;uint64_t ge=0;
    for(int y=0;y<bh;++y)for(int x=0;x<bw;++x){const size_t o=static_cast<size_t>(y0+y)*src.yw+x0+x;const int v=src.y[o];ys+=v;if(x){g+=std::abs(v-static_cast<int>(src.y[o-1]));++ge;}if(y){g+=std::abs(v-static_cast<int>(src.y[o-src.yw]));++ge;}}
    const double myv=static_cast<double>(ys/static_cast<long double>(bw*bh)),act=ge?static_cast<double>(g/static_cast<long double>(ge)):0.0;
    const double detail=std::clamp((act-1.5)/11.0,0.0,1.0);
    out.dark_detail=std::clamp((116.0-myv)/92.0,0.0,1.0)*std::clamp((myv-1.0)/11.0,0.0,1.0)*detail;
    const int cx0=mx*8,cy0=my*8,cbw=std::max(0,std::min(8,visible_chroma_width-cx0)),cbh=std::max(0,std::min(8,visible_chroma_field_lines-cy0));
    long double us=0.0,vs=0.0;for(int y=0;y<cbh;++y)for(int x=0;x<cbw;++x){const size_t o=static_cast<size_t>(cy0+y)*src.cw+cx0+x;us+=src.u[o];vs+=src.v[o];}
    const double cn=static_cast<double>(std::max(1,cbw*cbh)),cb=static_cast<double>(us)/cn-128.0,cr=static_cast<double>(vs)/cn-128.0,sat=std::hypot(cb,cr);
    const double cbp=std::clamp(cb/52.0,0.0,1.0),cbn=std::clamp(-cb/48.0,0.0,1.0),crp=std::clamp(cr/52.0,0.0,1.0),crn=std::clamp(-cr/48.0,0.0,1.0);
    const double green=std::min(cbn,crn),blue=cbp*std::clamp((28.0-cr)/80.0,0.35,1.0),violet=std::min(cbp,crp),gray=std::clamp((22.0-sat)/22.0,0.0,1.0);
    const double low=std::clamp((112.0-myv)/86.0,0.0,1.0),mh=std::clamp((myv-72.0)/86.0,0.0,1.0);
    const double lows=low*std::max({blue,violet,0.78*gray}),greens=mh*green;
    out.color_luma=detail*std::max(greens,lows);out.color_chroma=detail*std::max(0.65*greens,low*std::max(blue,violet));
    const double raw=intra?(4.8*out.dark_detail+1.55*out.color_luma+0.45*out.color_chroma):(out.color_chroma>0.03?(0.75*out.color_luma+0.25*out.color_chroma):0.0);
    out.requested_q_boost=std::clamp(aq_strength*raw,0.0,intra?6.0:2.0);return out;
}

static bool any_coeff(const std::array<int,64>& q) {
    for (int v:q) if (v) return true;
    return false;
}

static int popcount6(uint8_t v) {
    int n=0; for (int i=0;i<6;++i) n+=(v>>i)&1; return n;
}

static constexpr std::array<uint8_t,64> kFieldInterScan = {{
     0, 1, 8, 2, 3, 9,16, 4, 5, 6, 7,10,17,24,11,18,
    25,32,12,13,14,15,19,20,21,22,23,26,33,40,27,34,
    41,48,28,35,42,49,56,57,50,43,36,29,30,31,39,38,
    37,44,51,58,59,52,45,46,47,55,54,53,60,61,62,63
}};

static void write_field_mbmode_1mv_residual(BitWriter& b,bool has_mvdata,bool has_coeffs) {
    // Interlaced-field 1MV MBMODE table 0:
    //   2: 1             no MVDATA, no coeffs
    //   3: 001           MVDATA,    no coeffs
    //   4: 01            no MVDATA, coeffs
    //   5: 0001          MVDATA,    coeffs
    if (has_coeffs) b.vlc(1,has_mvdata?4:2);
    else b.vlc(1,has_mvdata?3:1);
}

static void write_field_cbp_table0(BitWriter& b,uint8_t cbp) {
    static constexpr std::array<uint16_t,63> code={
        0x2F1A,0x2F1B,0x178C,0x0090,0x02A8,0x02A9,0x0BC7,0x0091,
        0x02AA,0x02AB,0x05E0,0x004A,0x0096,0x0097,0x00BD,0x0092,
        0x02AC,0x02AD,0x05E1,0x0098,0x0132,0x0133,0x0179,0x0134,
        0x026A,0x026B,0x02FC,0x004E,0x0040,0x0041,0x002B,0x0093,
        0x02AE,0x02AF,0x05E2,0x0136,0x026E,0x026F,0x02FD,0x009E,
        0x013E,0x013F,0x017F,0x0050,0x0042,0x0043,0x002C,0x0051,
        0x00A4,0x00A5,0x00BE,0x0053,0x0044,0x0045,0x002D,0x0054,
        0x0046,0x0047,0x002E,0x0003,0x0000,0x0001,0x0001
    };
    static constexpr std::array<uint8_t,63> bits={
        15,15,14,9,11,11,13,9,11,11,12,8,9,9,9,9,
        11,11,12,9,10,10,10,10,11,11,11,8,8,8,7,9,
        11,11,12,10,11,11,11,9,10,10,10,8,8,8,7,8,
        9,9,9,8,8,8,7,8,8,8,7,3,3,3,1
    };
    if (!cbp) throw std::runtime_error("field CBP writer called with cbp=0");
    const size_t i=static_cast<size_t>(cbp-1);
    b.vlc(code[i],bits[i]);
}

} // namespace

Vc1Encoder::PEncodeResult Vc1Encoder::encode_p_interlaced_fields(const Frame& f,const Frame& ref,
                                                                  bool ref_field_picture,
                                                                  const Frame* previous_source) const {
    if (c_.syntax!=StreamSyntax::Advanced || !c_.interlaced())
        throw std::runtime_error("field-picture P encoder requires interlaced Advanced Profile");
    if (f.width!=c_.width || f.height!=c_.height || ref.width!=c_.width || ref.height!=c_.height)
        throw std::runtime_error("field-picture P encoder frame size mismatch");

    const int mbw=(c_.width+15)/16;
    const int mbh=field_mb_height(c_.height);
    const size_t mbs=static_cast<size_t>(mbw)*mbh;
    PEncodeResult out;
    out.reconstructed=Frame{c_.width,c_.height,
        std::vector<uint8_t>(static_cast<size_t>(c_.width)*c_.height),
        std::vector<uint8_t>(static_cast<size_t>(c_.width/2)*(c_.height/2)),
        std::vector<uint8_t>(static_cast<size_t>(c_.width/2)*(c_.height/2))};
    if(c_.debug_macroblock_stats) out.macroblock_debug.resize(2*mbs);

    const int coding_set=chroma_coding_set(0,c_.pqindex);
    const bool use_vlc=c_.ac_mode!=AcMode::Esc3;
    const int first_type=c_.top_field_first()?0:1;
    const int mvr=motion_mvrange();
    const int range_x=mvrange_x_qpel(mvr),range_y=mvrange_y_qpel(mvr);
    const int kx=mvrange_kx(mvr),ky=mvrange_ky(mvr);
    const int local=std::min(c_.motion_search_range,std::max(0,c_.motion_local_search_range));

    for (int fi=0;fi<2;++fi) {
        const int cur=fi==0?first_type:1-first_type;
        const PackedField src=pack_source_field(f,cur,mbw,mbh);
        const int vis_field=field_lines(c_.visible_height(),cur);
        const int vis_cfield=field_lines(c_.visible_chroma_height(),cur);
        PackedField recon;
        recon.yw=src.yw; recon.yh=src.yh; recon.cw=src.cw; recon.ch=src.ch;
        recon.y.assign(src.y.size(),0); recon.u.assign(src.u.size(),0); recon.v.assign(src.v.size(),0);
        PackedField debug_prediction;
        std::optional<PackedField> debug_previous;
        if(c_.debug_macroblock_stats){
            debug_prediction.yw=src.yw;debug_prediction.yh=src.yh;debug_prediction.cw=src.cw;debug_prediction.ch=src.ch;
            debug_prediction.y.assign(src.y.size(),0);debug_prediction.u.assign(src.u.size(),0);debug_prediction.v.assign(src.v.size(),0);
            if(previous_source)debug_previous=pack_source_field(*previous_source,cur,mbw,mbh);
        }
        std::vector<FieldMbSyntax> syntax(mbs);
        FieldPredState pred(mbs);

        std::array<FieldChoice,2> refs{};
        refs[0]=FieldChoice{0,false,&ref,cur,ref_field_picture,0};
        const Frame* opp_src=(fi==0)?&ref:&out.reconstructed;
        const bool opp_fp=(fi==0)?ref_field_picture:true;
        refs[1]=FieldChoice{0,true,opp_src,1-cur,opp_fp,0};

        for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
            const size_t pos=static_cast<size_t>(my)*mbw+mx;
            auto& ms=syntax[pos];
            MotionVector seed{};
            if (mx>0) seed=syntax[pos-1].mv;
            uint64_t best_cost=std::numeric_limits<uint64_t>::max();
            FieldChoice best_ref=refs[0];
            MotionVector best_mv{};
            for (const auto& r:refs) {
                const auto mv=search_field_mb(src,mx,my,cur,*r.source,r.ref_field_type,c_.rndctrl,
                                              r.source_field_picture,seed,local,range_x,range_y);
                const auto cost=field_mb_sad(src,mx,my,cur,*r.source,r.ref_field_type,mv,c_.rndctrl,
                                             r.source_field_picture,1);
                if (cost<best_cost || (cost==best_cost && (!r.opposite && best_ref.opposite)) ||
                    (cost==best_cost && r.opposite==best_ref.opposite &&
                     std::abs(mv.xq)+std::abs(mv.yq)<std::abs(best_mv.xq)+std::abs(best_mv.yq))) {
                    best_cost=cost; best_ref=r; best_mv=mv;
                }
            }
            ms.opposite=best_ref.opposite; ms.mv=best_mv;
            const auto pi=field_pred_for_target(pred,mbw,mx,my,0,ms.opposite,ms.mv,false,fi,cur,
                                                range_x,range_y,true);
            ms.dmv_x=pi.dmv_x; ms.dmv_y=pi.dmv_y; ms.pred_flag=pi.pred_flag;
            ms.hybrid_present=pi.hybrid_present; ms.hybrid_bit=pi.hybrid_bit;
            predict_field_mb(recon,mx,my,cur,*best_ref.source,best_ref.ref_field_type,ms.mv,
                             c_.rndctrl,best_ref.source_field_picture);
            if(c_.debug_macroblock_stats)
                predict_field_mb(debug_prediction,mx,my,cur,*best_ref.source,best_ref.ref_field_type,ms.mv,
                                 c_.rndctrl,best_ref.source_field_picture);
            if (ms.opposite) ++out.field_opposite_macroblocks; else ++out.field_same_macroblocks;
            if (ms.mv.xq || ms.mv.yq) ++out.field_moved_macroblocks;
            const double local_mae=static_cast<double>(best_cost)/256.0;
            ms.mquant=residual_priority_mquant(rate_weighted_mquant(c_.rc_inter_block_weight),
                                               local_mae,c_.rc_prediction_residual_mean);
            if(c_.adaptive_quality&&c_.dquant&&ms.mquant>1){const auto pp=field_perceptual_priority(src,mx,my,c_.visible_width(),vis_field,c_.visible_chroma_width(),vis_cfield,c_.aq_strength,false);const int dq=std::clamp(static_cast<int>(std::lround(pp.requested_q_boost)),0,std::min(2,ms.mquant-1));ms.mquant=std::max(1,ms.mquant-dq);}

            for (int k=0;k<6;++k) {
                std::vector<uint8_t>* rp=nullptr;
                const std::vector<uint8_t>* sp=nullptr;
                int w=0,h=0,bx=0,by=0;
                if (k<4) { rp=&recon.y; sp=&src.y; w=src.yw; h=src.yh; bx=mx*16+(k&1)*8; by=my*16+((k>>1)&1)*8; }
                else { rp=(k==4)?&recon.u:&recon.v; sp=(k==4)?&src.u:&src.v; w=src.cw; h=src.ch; bx=mx*8; by=my*8; }
                ms.q[static_cast<size_t>(k)]=quantize_inter(*sp,*rp,w,h,bx,by,ms.mquant,ms.mquant!=c_.pqindex);
                if (any_coeff(ms.q[static_cast<size_t>(k)])) {
                    ms.cbp|=static_cast<uint8_t>(1u<<(5-k));
                    auto r=inverse_partition(TransformType::T8x8,ms.q[static_cast<size_t>(k)],ms.mquant,ms.mquant!=c_.pqindex);
                    add_block(*rp,w,h,bx,by,r);
                }
            }
            ++out.explicit_macroblocks;
            if (ms.cbp) { ++out.coded_macroblocks; out.coded_blocks+=static_cast<size_t>(popcount6(ms.cbp)); }
            else ++out.skipped_macroblocks;
            if (!ms.cbp) ms.mquant=c_.pqindex;
            record_mquant(out,c_.pqindex,ms.mquant);
        }
        std::vector<int> desired_mquant(mbs,c_.pqindex);
        for (size_t pos=0;pos<mbs;++pos) if (syntax[pos].cbp) desired_mquant[pos]=syntax[pos].mquant;
        const DQuantPlan dquant_plan=make_dquant_plan(desired_mquant,mbw,mbh);
        const bool dquantfrm=dquant_plan.enabled;
        if(c_.debug_macroblock_stats){
            for(int my=0;my<mbh;++my)for(int mx=0;mx<mbw;++mx){
                const size_t pos=static_cast<size_t>(my)*mbw+mx;const auto& ms=syntax[pos];auto& d=out.macroblock_debug[static_cast<size_t>(fi)*mbs+pos];
                d.i_mode=VC1_MB_DEBUG_FIELD_P_FORWARD;d.i_field_index=fi;d.i_field_parity=cur;d.b_opposite_field_reference=ms.opposite?1:0;
                d.i_picture_q=c_.pqindex;d.i_mquant=ms.mquant;d.i_dquant_delta=ms.mquant-c_.pqindex;d.i_forward_reference_display_order=d.i_backward_reference_display_order=-1;
                d.i_cbp=ms.cbp;d.i_coded_blocks=static_cast<uint32_t>(popcount6(ms.cbp));d.i_transform_parts[0]=d.i_coded_blocks;
                d.i_forward_mv_count=1;d.i_forward_mv_xq[0]=ms.mv.xq;d.i_forward_mv_yq[0]=ms.mv.yq;
                fill_field_debug_quality(d,src,recon,debug_previous?&*debug_previous:nullptr,&debug_prediction,mx,my,
                                         c_.visible_width(),vis_field,c_.visible_chroma_width(),vis_cfield);
                d.f_prediction_mae_y=d.f_prediction_sad_y/static_cast<double>(std::max(1,static_cast<int>(d.i_visible_width*d.i_visible_height)));
                d.f_residual_priority_position=residual_priority_position(d.f_prediction_mae_y);
                d.f_residual_priority_requested_q_boost=residual_priority_requested_q_boost(d.f_prediction_mae_y,c_.rc_prediction_residual_mean);
                d.i_residual_priority_applied_q_boost=std::max(0,rate_weighted_mquant(c_.rc_inter_block_weight)-residual_priority_mquant(rate_weighted_mquant(c_.rc_inter_block_weight),d.f_prediction_mae_y,c_.rc_prediction_residual_mean));
                d.f_inter_intra_cost_ratio=-1.0;const auto pp=field_perceptual_priority(src,mx,my,c_.visible_width(),vis_field,c_.visible_chroma_width(),vis_cfield,c_.aq_strength,false);d.f_aq_dark_detail=pp.dark_detail;d.f_aq_color_luma_priority=pp.color_luma;d.f_aq_color_chroma_priority=pp.color_chroma;d.f_aq_requested_q_boost=pp.requested_q_boost;
            }
        }
        weave_packed_field(out.reconstructed,cur,recon);

        BitWriter b;
        if (fi==0) {
            write_decode012(b,2); b.bits(0b011,3); // FCM=11, P/P.
            b.bit(c_.top_field_first()); b.bit(false); // TFF/RFF.
            b.bit(false); b.bit(false); // RNDCTRL/UVSAMP.
        }
        b.bits(static_cast<uint64_t>(c_.pqindex),5);
        if (c_.pqindex<=8) b.bit(c_.halfqp);
        b.bit(uniform_quantizer());
        b.bit(true); // NUMREF=1: same/opposite field selected per macroblock.
        if (extended_mv_enabled()) write_mvrange(b);
        write_unary_mode(b,p_mv_mode_index(ProgressiveMvMode::OneMvQpel,c_.pqindex,false),4);
        b.bits(0,3); b.bits(0,3); b.bits(0,3); // MBMODETAB / IMVTAB / ICBPTAB.
        if (c_.dquant) write_dquant_header(b,dquant_plan);
        if (c_.variable_transforms) { b.bit(true); b.bits(0,2); }
        write_decode012(b,0); b.bit(false); // TRANSACFRM / TRANSDCTAB.
        bool esc3_lengths_written=false;
        for (size_t pos=0;pos<syntax.size();++pos) {
            const auto& ms=syntax[pos];const size_t bits_before=b.bit_count();
            write_field_mbmode_1mv_residual(b,true,ms.cbp!=0);
            write_field_mvdata_escape(b,ms.dmv_x,ms.dmv_y,ms.pred_flag,kx,ky);
            if (ms.hybrid_present) b.bit(ms.hybrid_bit);
            size_t coeff_before=b.bit_count();
            if (ms.cbp) {
                write_field_cbp_table0(b,ms.cbp);
                if (dquantfrm) write_dquant_mb(b,dquant_plan,pos);
                coeff_before=b.bit_count();
                for (int k=0;k<6;++k) if (ms.cbp&(1u<<(5-k)))
                    write_inter_block(b,ms.q[static_cast<size_t>(k)],coding_set,use_vlc,
                                      esc3_lengths_written,kFieldInterScan.data(),dquantfrm);
            }
            if(c_.debug_macroblock_stats){auto& d=out.macroblock_debug[static_cast<size_t>(fi)*mbs+pos];d.i_local_bits=static_cast<uint64_t>(b.bit_count()-bits_before);d.i_estimated_transform_bits=ms.cbp?static_cast<uint64_t>(b.bit_count()-coeff_before):0;}
        }
        append_bytes(out.data,bdu(fi==0?0x0d:0x0c,b.finish_rbdu()));
    }
    out.padded_reconstructed=out.reconstructed;
    return out;
}

Vc1Encoder::BEncodeResult Vc1Encoder::encode_b_interlaced_fields(const Frame& f,const Frame& past,const Frame& future,
                                                      int fraction_num,int fraction_den,
                                                      bool past_field_picture,bool future_field_picture,
                                                      const Frame* previous_source) const {
    if (c_.syntax!=StreamSyntax::Advanced || !c_.interlaced())
        throw std::runtime_error("field-picture B encoder requires interlaced Advanced Profile");
    if (f.width!=c_.width || f.height!=c_.height || past.width!=c_.width || past.height!=c_.height ||
        future.width!=c_.width || future.height!=c_.height)
        throw std::runtime_error("field-picture B encoder frame size mismatch");
    const int mbw=(c_.width+15)/16;
    const int mbh=field_mb_height(c_.height);
    const size_t mbs=static_cast<size_t>(mbw)*mbh;
    BEncodeResult out;
    out.reconstructed=Frame{c_.width,c_.height,
        std::vector<uint8_t>(static_cast<size_t>(c_.width)*c_.height),
        std::vector<uint8_t>(static_cast<size_t>(c_.width/2)*(c_.height/2)),
        std::vector<uint8_t>(static_cast<size_t>(c_.width/2)*(c_.height/2))};
    if(c_.debug_macroblock_stats) out.macroblock_debug.resize(2*mbs);
    const int coding_set=chroma_coding_set(0,c_.pqindex);
    const bool use_vlc=c_.ac_mode!=AcMode::Esc3;
    const int first_type=c_.top_field_first()?0:1;
    const int mvr=motion_mvrange();
    const int range_x=mvrange_x_qpel(mvr),range_y=mvrange_y_qpel(mvr);
    const int kx=mvrange_kx(mvr),ky=mvrange_ky(mvr);
    const int local=std::min(c_.motion_search_range,std::max(0,c_.motion_local_search_range));

    for (int fi=0;fi<2;++fi) {
        const int cur=fi==0?first_type:1-first_type;
        const PackedField src=pack_source_field(f,cur,mbw,mbh);
        const int vis_field=field_lines(c_.visible_height(),cur);
        const int vis_cfield=field_lines(c_.visible_chroma_height(),cur);
        PackedField recon;
        recon.yw=src.yw; recon.yh=src.yh; recon.cw=src.cw; recon.ch=src.ch;
        recon.y.assign(src.y.size(),0); recon.u.assign(src.u.size(),0); recon.v.assign(src.v.size(),0);
        PackedField debug_prediction;
        std::optional<PackedField> debug_previous;
        if(c_.debug_macroblock_stats){
            debug_prediction.yw=src.yw;debug_prediction.yh=src.yh;debug_prediction.cw=src.cw;debug_prediction.ch=src.ch;
            debug_prediction.y.assign(src.y.size(),0);debug_prediction.u.assign(src.u.size(),0);debug_prediction.v.assign(src.v.size(),0);
            if(previous_source)debug_previous=pack_source_field(*previous_source,cur,mbw,mbh);
        }
        std::vector<FieldMbSyntax> syntax(mbs);
        std::vector<uint8_t> forward_plane(mbs,1);
        FieldPredState pred(mbs);

        const Frame* fopp=(fi==0)?&past:&out.reconstructed;
        const bool fopp_fp=(fi==0)?past_field_picture:true;
        const std::array<FieldChoice,4> refs={{
            FieldChoice{0,false,&past,cur,past_field_picture,0},
            FieldChoice{0,true,fopp,1-cur,fopp_fp,0},
            FieldChoice{1,false,&future,cur,future_field_picture,0},
            FieldChoice{1,true,&future,1-cur,future_field_picture,0}
        }};

        for (int my=0;my<mbh;++my) for (int mx=0;mx<mbw;++mx) {
            const size_t pos=static_cast<size_t>(my)*mbw+mx;
            auto& ms=syntax[pos];
            MotionVector seed{};
            if (mx>0) seed=syntax[pos-1].mv;
            uint64_t best_cost=std::numeric_limits<uint64_t>::max();
            FieldChoice best_ref=refs[0];
            MotionVector best_mv{};
            for (const auto& r:refs) {
                const auto mv=search_field_mb(src,mx,my,cur,*r.source,r.ref_field_type,c_.rndctrl,
                                              r.source_field_picture,seed,local,range_x,range_y);
                const auto cost=field_mb_sad(src,mx,my,cur,*r.source,r.ref_field_type,mv,c_.rndctrl,
                                             r.source_field_picture,1);
                const int ref_penalty=(r.direction?1:0)+(r.opposite?1:0);
                const int best_penalty=(best_ref.direction?1:0)+(best_ref.opposite?1:0);
                if (cost<best_cost || (cost==best_cost && ref_penalty<best_penalty) ||
                    (cost==best_cost && ref_penalty==best_penalty &&
                     std::abs(mv.xq)+std::abs(mv.yq)<std::abs(best_mv.xq)+std::abs(best_mv.yq))) {
                    best_cost=cost; best_ref=r; best_mv=mv;
                }
            }
            ms.direction=best_ref.direction; ms.opposite=best_ref.opposite; ms.mv=best_mv;
            const auto pi=field_pred_for_target(pred,mbw,mx,my,ms.direction,ms.opposite,ms.mv,true,fi,cur,
                                                range_x,range_y,true);
            ms.dmv_x=pi.dmv_x; ms.dmv_y=pi.dmv_y; ms.pred_flag=pi.pred_flag;
            field_apply_b_unused(pred,mbw,mx,my,1-ms.direction,fi,cur,range_x,range_y);
            predict_field_mb(recon,mx,my,cur,*best_ref.source,best_ref.ref_field_type,ms.mv,
                             c_.rndctrl,best_ref.source_field_picture);
            if(c_.debug_macroblock_stats)
                predict_field_mb(debug_prediction,mx,my,cur,*best_ref.source,best_ref.ref_field_type,ms.mv,
                                 c_.rndctrl,best_ref.source_field_picture);
            forward_plane[pos]=ms.direction==0?1:0;
            if (ms.direction==0) {
                ++out.forward_macroblocks;
                if (ms.opposite) ++out.field_forward_opposite_macroblocks; else ++out.field_forward_same_macroblocks;
            } else {
                ++out.backward_macroblocks;
                if (ms.opposite) ++out.field_backward_opposite_macroblocks; else ++out.field_backward_same_macroblocks;
            }
            if (ms.mv.xq || ms.mv.yq) ++out.field_moved_macroblocks;
            const double local_mae=static_cast<double>(best_cost)/256.0;
            ms.mquant=residual_priority_mquant(rate_weighted_mquant(c_.rc_inter_block_weight),
                                               local_mae,c_.rc_prediction_residual_mean);
            if(c_.adaptive_quality&&c_.dquant&&ms.mquant>1){const auto pp=field_perceptual_priority(src,mx,my,c_.visible_width(),vis_field,c_.visible_chroma_width(),vis_cfield,c_.aq_strength,false);const int dq=std::clamp(static_cast<int>(std::lround(pp.requested_q_boost)),0,std::min(2,ms.mquant-1));ms.mquant=std::max(1,ms.mquant-dq);}

            for (int k=0;k<6;++k) {
                std::vector<uint8_t>* rp=nullptr;
                const std::vector<uint8_t>* sp=nullptr;
                int w=0,h=0,bx=0,by=0;
                if (k<4) { rp=&recon.y; sp=&src.y; w=src.yw; h=src.yh; bx=mx*16+(k&1)*8; by=my*16+((k>>1)&1)*8; }
                else { rp=(k==4)?&recon.u:&recon.v; sp=(k==4)?&src.u:&src.v; w=src.cw; h=src.ch; bx=mx*8; by=my*8; }
                ms.q[static_cast<size_t>(k)]=quantize_inter(*sp,*rp,w,h,bx,by,ms.mquant,ms.mquant!=c_.pqindex);
                if (any_coeff(ms.q[static_cast<size_t>(k)])) {
                    ms.cbp|=static_cast<uint8_t>(1u<<(5-k));
                    auto r=inverse_partition(TransformType::T8x8,ms.q[static_cast<size_t>(k)],ms.mquant,ms.mquant!=c_.pqindex);
                    add_block(*rp,w,h,bx,by,r);
                }
            }
            ++out.explicit_macroblocks;
            if (ms.cbp) { ++out.coded_macroblocks; out.coded_blocks+=static_cast<size_t>(popcount6(ms.cbp)); }
            else ++out.skipped_macroblocks;
            if (!ms.cbp) ms.mquant=c_.pqindex;
            record_mquant(out,c_.pqindex,ms.mquant);
        }
        std::vector<int> desired_mquant(mbs,c_.pqindex);
        for (size_t pos=0;pos<mbs;++pos) if (syntax[pos].cbp) desired_mquant[pos]=syntax[pos].mquant;
        const DQuantPlan dquant_plan=make_dquant_plan(desired_mquant,mbw,mbh);
        const bool dquantfrm=dquant_plan.enabled;
        if(c_.debug_macroblock_stats){
            for(int my=0;my<mbh;++my)for(int mx=0;mx<mbw;++mx){
                const size_t pos=static_cast<size_t>(my)*mbw+mx;const auto& ms=syntax[pos];auto& d=out.macroblock_debug[static_cast<size_t>(fi)*mbs+pos];
                d.i_mode=ms.direction==0?VC1_MB_DEBUG_FIELD_B_FORWARD:VC1_MB_DEBUG_FIELD_B_BACKWARD;d.i_field_index=fi;d.i_field_parity=cur;d.b_opposite_field_reference=ms.opposite?1:0;
                d.i_picture_q=c_.pqindex;d.i_mquant=ms.mquant;d.i_dquant_delta=ms.mquant-c_.pqindex;d.i_forward_reference_display_order=d.i_backward_reference_display_order=-1;
                d.i_cbp=ms.cbp;d.i_coded_blocks=static_cast<uint32_t>(popcount6(ms.cbp));d.i_transform_parts[0]=d.i_coded_blocks;
                if(ms.direction==0){d.i_forward_mv_count=1;d.i_forward_mv_xq[0]=ms.mv.xq;d.i_forward_mv_yq[0]=ms.mv.yq;}else{d.i_backward_mv_count=1;d.i_backward_mv_xq[0]=ms.mv.xq;d.i_backward_mv_yq[0]=ms.mv.yq;}
                fill_field_debug_quality(d,src,recon,debug_previous?&*debug_previous:nullptr,&debug_prediction,mx,my,
                                         c_.visible_width(),vis_field,c_.visible_chroma_width(),vis_cfield);
                d.f_prediction_mae_y=d.f_prediction_sad_y/static_cast<double>(std::max(1,static_cast<int>(d.i_visible_width*d.i_visible_height)));
                d.f_residual_priority_position=residual_priority_position(d.f_prediction_mae_y);
                d.f_residual_priority_requested_q_boost=residual_priority_requested_q_boost(d.f_prediction_mae_y,c_.rc_prediction_residual_mean);
                d.i_residual_priority_applied_q_boost=std::max(0,rate_weighted_mquant(c_.rc_inter_block_weight)-residual_priority_mquant(rate_weighted_mquant(c_.rc_inter_block_weight),d.f_prediction_mae_y,c_.rc_prediction_residual_mean));
                d.f_inter_intra_cost_ratio=-1.0;const auto pp=field_perceptual_priority(src,mx,my,c_.visible_width(),vis_field,c_.visible_chroma_width(),vis_cfield,c_.aq_strength,false);d.f_aq_dark_detail=pp.dark_detail;d.f_aq_color_luma_priority=pp.color_luma;d.f_aq_color_chroma_priority=pp.color_chroma;d.f_aq_requested_q_boost=pp.requested_q_boost;
            }
        }
        weave_packed_field(out.reconstructed,cur,recon);

        BitWriter b;
        if (fi==0) {
            write_decode012(b,2); b.bits(0b100,3); // FCM=11, B/B.
            b.bit(c_.top_field_first()); b.bit(false); b.bit(false); b.bit(false);
            write_bfraction(b,fraction_num,fraction_den);
        }
        b.bits(static_cast<uint64_t>(c_.pqindex),5);
        if (c_.pqindex<=8) b.bit(c_.halfqp);
        b.bit(uniform_quantizer());
        if (extended_mv_enabled()) write_mvrange(b);
        write_unary_mode(b,p_mv_mode_index(ProgressiveMvMode::OneMvQpel,c_.pqindex,true),3);
        const BitplaneChoice forward_choice=choose_bitplane(forward_plane,mbw,mbh);
        b.append(forward_choice.syntax);
        b.bits(0,3); b.bits(0,3); b.bits(0,3);
        if (c_.dquant) write_dquant_header(b,dquant_plan);
        if (c_.variable_transforms) { b.bit(true); b.bits(0,2); }
        write_decode012(b,0); b.bit(false);
        bool esc3_lengths_written=false;
        for (size_t pos=0;pos<syntax.size();++pos) {
            const auto& ms=syntax[pos];const size_t bits_before=b.bit_count();
            write_field_mbmode_1mv_residual(b,true,ms.cbp!=0);
            // IMODE_RAW carries FORWARDMB in the macroblock layer after
            // MBMODE.  Compressed modes already carried the whole plane.
            if (forward_choice.raw) b.bit(ms.direction==0);
            if (ms.direction==1) write_decode012(b,0); // BMVTYPE=backward before MVDATA.
            write_field_mvdata_escape(b,ms.dmv_x,ms.dmv_y,ms.pred_flag,kx,ky);
            size_t coeff_before=b.bit_count();
            if (ms.cbp) {
                write_field_cbp_table0(b,ms.cbp);
                if (dquantfrm) write_dquant_mb(b,dquant_plan,pos);
                coeff_before=b.bit_count();
                for (int k=0;k<6;++k) if (ms.cbp&(1u<<(5-k)))
                    write_inter_block(b,ms.q[static_cast<size_t>(k)],coding_set,use_vlc,
                                      esc3_lengths_written,kFieldInterScan.data(),dquantfrm);
            }
            if(c_.debug_macroblock_stats){auto& d=out.macroblock_debug[static_cast<size_t>(fi)*mbs+pos];d.i_local_bits=static_cast<uint64_t>(b.bit_count()-bits_before);d.i_estimated_transform_bits=ms.cbp?static_cast<uint64_t>(b.bit_count()-coeff_before):0;}
        }
        append_bytes(out.data,bdu(fi==0?0x0d:0x0c,b.finish_rbdu()));
    }
    out.padded_reconstructed=out.reconstructed;
    return out;
}

} // namespace libvc1
