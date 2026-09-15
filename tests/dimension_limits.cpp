#include <libvc1.h>

#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct BitReader {
    const std::vector<uint8_t>& b;
    size_t bit=0;
    uint64_t bits(int n) {
        if (n<0 || n>64 || bit+static_cast<size_t>(n)>b.size()*8u)
            throw std::runtime_error("sequence header truncated");
        uint64_t v=0;
        for (int i=0;i<n;++i) {
            v=(v<<1) | ((b[bit>>3] >> (7-(bit&7))) & 1u);
            ++bit;
        }
        return v;
    }
};

std::vector<uint8_t> unescape_sequence(const vc1_au_t& h) {
    if (!h.p_payload || h.i_payload<5 || h.p_payload[0]!=0 || h.p_payload[1]!=0 ||
        h.p_payload[2]!=1 || h.p_payload[3]!=0x0f)
        throw std::runtime_error("missing Advanced sequence start code");
    std::vector<uint8_t> out;
    int zeros=0;
    for (size_t i=4;i<h.i_payload;++i) {
        const uint8_t v=h.p_payload[i];
        if (zeros>=2 && v==0x03) {
            zeros=0;
            continue;
        }
        out.push_back(v);
        if (v==0) ++zeros; else zeros=0;
    }
    return out;
}

struct AdvancedHeader {
    int level=0;
    int coded_width=0;
    int coded_height=0;
    bool display_ext=false;
    int display_width=0;
    int display_height=0;
};

AdvancedHeader parse_advanced(const vc1_au_t& h) {
    const auto rbdu=unescape_sequence(h);
    BitReader br{rbdu};
    if (br.bits(2)!=3) throw std::runtime_error("not Advanced Profile");
    AdvancedHeader r;
    r.level=static_cast<int>(br.bits(3));
    (void)br.bits(2); // COLORDIFF_FORMAT
    (void)br.bits(3); // FRMRTQ_POSTPROC
    (void)br.bits(5); // BITRTQ_POSTPROC
    (void)br.bits(1); // POSTPROCFLAG
    r.coded_width=(static_cast<int>(br.bits(12))+1)*2;
    r.coded_height=(static_cast<int>(br.bits(12))+1)*2;
    (void)br.bits(1); // PULLDOWN
    (void)br.bits(1); // INTERLACE
    (void)br.bits(1); // TFCNTRFLAG
    (void)br.bits(1); // FINTERPFLAG
    (void)br.bits(1); // reserved
    (void)br.bits(1); // PSF
    r.display_ext=br.bits(1)!=0;
    if (r.display_ext) {
        r.display_width=static_cast<int>(br.bits(14))+1;
        r.display_height=static_cast<int>(br.bits(14))+1;
        // These flags are all deliberately zero in libvc1's DISPLAY_EXT.
        if (br.bits(1) || br.bits(1) || br.bits(1))
            throw std::runtime_error("unexpected display-extension subfield");
    } else {
        r.display_width=r.coded_width;
        r.display_height=r.coded_height;
    }
    return r;
}

vc1_param_t base(bool advanced=true) {
    vc1_param_t p{};
    if (vc1_param_default(&p)<0) throw std::runtime_error("vc1_param_default failed");
    p.i_profile=advanced?VC1_PROFILE_ADVANCED:VC1_PROFILE_MAIN;
    p.i_width=16; p.i_height=16;
    p.i_fps_num=30; p.i_fps_den=1;
    p.i_threads=1; p.i_bframes=0; p.b_intra_only=1;
    p.i_simd=VC1_SIMD_NONE;
    p.i_rc_method=VC1_RC_CQP; p.i_qp_constant=20;
    return p;
}

AdvancedHeader open_advanced(vc1_param_t p) {
    vc1_t* e=vc1_encoder_open(&p);
    if (!e) throw std::runtime_error(std::string("open failed: ")+vc1_encoder_last_error(nullptr));
    vc1_au_t* h=nullptr; int n=0;
    if (vc1_encoder_headers(e,&h,&n)<=0 || n!=1) {
        const std::string err=vc1_encoder_last_error(e);
        vc1_encoder_close(e);
        throw std::runtime_error("headers failed: "+err);
    }
    const auto parsed=parse_advanced(*h);
    vc1_encoder_close(e);
    return parsed;
}

bool rejects(vc1_param_t p) {
    vc1_t* e=vc1_encoder_open(&p);
    if (e) { vc1_encoder_close(e); return false; }
    return true;
}

void expect_header(vc1_param_t p,int level,int cw,int ch,int dw,int dh,bool de) {
    const auto h=open_advanced(p);
    if (h.level!=level || h.coded_width!=cw || h.coded_height!=ch ||
        h.display_width!=dw || h.display_height!=dh || h.display_ext!=de) {
        std::fprintf(stderr,"header mismatch: level=%d coded=%dx%d display=%dx%d ext=%d\n",
                     h.level,h.coded_width,h.coded_height,h.display_width,h.display_height,h.display_ext?1:0);
        throw std::runtime_error("unexpected Advanced sequence geometry/level");
    }
}

} // namespace

int main() {
    try {
        // Largest AP@L3 picture/rate combination at 30 fps.
        auto p=base(); p.i_width=2048; p.i_height=1024;
        expect_header(p,3,2048,1024,2048,1024,false);

        // Standard AP@L4 examples / maxima.
        p=base(); p.i_width=2048; p.i_height=1536; p.i_fps_num=24;
        expect_header(p,4,2048,1536,2048,1536,false);
        p=base(); p.i_width=2048; p.i_height=2048;
        expect_header(p,4,2048,2048,2048,2048,false);

        // Exercise the 12-bit half-size syntax endpoint and the exact L4
        // 16384-MB/frame, 491520-MB/s ceilings at 30 fps.
        p=base(); p.i_width=8192; p.i_height=512;
        expect_header(p,4,8192,512,8192,512,false);

        // Odd presentation sizes are represented by an even coded raster plus
        // Advanced Profile DISPLAY_EXT.  This also exercises the maximum coded
        // width with both right and bottom one-sample luma padding.
        p=base(); p.i_width=8191; p.i_height=511;
        expect_header(p,4,8192,512,8191,511,true);
        p=base(); p.i_width=65; p.i_height=49;
        expect_header(p,3,66,50,65,49,true);

        // Bitrate alone can require AP@L4 even for a small picture.
        p=base(); p.i_width=1920; p.i_height=1080; p.i_rc_method=VC1_RC_ABR;
        p.i_bitrate=50000000ull; p.i_vbv_buffer_size=30000000ull;
        expect_header(p,4,1920,1080,1920,1080,false);

        // L4 hard limits.
        p=base(); p.i_width=8192; p.i_height=514;
        if (!rejects(p)) throw std::runtime_error("accepted picture above AP@L4 MB/frame limit");
        p=base(); p.i_width=8192; p.i_height=512; p.i_fps_num=31;
        if (!rejects(p)) throw std::runtime_error("accepted picture above AP@L4 MB/s limit");
        p=base(); p.i_width=8193; p.i_height=2;
        if (!rejects(p)) throw std::runtime_error("accepted coded width above 8192");
        p=base(); p.i_width=1920; p.i_height=1080; p.i_rc_method=VC1_RC_ABR;
        p.i_bitrate=136000000ull; p.i_vbv_buffer_size=30000000ull;
        if (!rejects(p)) throw std::runtime_error("accepted bitrate above AP@L4 Rmax");
        p=base(); p.i_width=1920; p.i_height=1080; p.i_rc_method=VC1_RC_ABR;
        p.i_bitrate=38000000ull; p.i_vbv_buffer_size=270336001ull;
        if (!rejects(p)) throw std::runtime_error("accepted VBV above AP@L4 Bmax");

        // Main Profile dimensions are conveyed out-of-band. libvc1 therefore
        // uses the same one-sample even internal 4:2:0 padding while exposing
        // the caller-visible odd raster through the container/transport.
        p=base(false); p.i_width=8192; p.i_height=256;
        {
            vc1_t* e=vc1_encoder_open(&p);
            if (!e) throw std::runtime_error(std::string("Main HL endpoint rejected: ")+vc1_encoder_last_error(nullptr));
            vc1_encoder_close(e);
        }
        p=base(false); p.i_width=1919; p.i_height=1079;
        {
            vc1_t* e=vc1_encoder_open(&p);
            if (!e) throw std::runtime_error(std::string("odd Main dimensions rejected: ")+vc1_encoder_last_error(nullptr));
            vc1_encoder_close(e);
        }

        // Main Profile High keeps its own standard rate/buffer ceilings; these
        // are VC-1 level limits, not Blu-ray restrictions.
        p=base(false); p.i_rc_method=VC1_RC_ABR; p.i_bitrate=20000000ull; p.i_vbv_buffer_size=40009728ull;
        if (rejects(p)) throw std::runtime_error("rejected exact MP@HL Rmax/VBVmax");
        p.i_bitrate=20000001ull;
        if (!rejects(p)) throw std::runtime_error("accepted bitrate above MP@HL Rmax");
        p=base(false); p.i_rc_method=VC1_RC_ABR; p.i_bitrate=20000000ull; p.i_vbv_buffer_size=40009729ull;
        if (!rejects(p)) throw std::runtime_error("accepted VBV above MP@HL VBVmax");

        std::puts("dimension/level limits ok");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr,"dimension/level test failed: %s\n",e.what());
        return 1;
    }
}
