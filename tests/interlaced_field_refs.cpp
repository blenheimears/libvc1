#include "../src/encoder_internal.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace libvc1;

static Frame field_frame(int w,int h,uint8_t top,uint8_t bottom) {
    Frame f;
    f.width=w; f.height=h;
    f.y.resize(static_cast<size_t>(w)*h);
    f.u.resize(static_cast<size_t>(w/2)*(h/2));
    f.v.resize(static_cast<size_t>(w/2)*(h/2));
    for (int y=0;y<h;++y)
        std::fill_n(f.y.begin()+static_cast<size_t>(y)*w,w,(y&1)?bottom:top);
    const int cw=w/2,ch=h/2;
    for (int y=0;y<ch;++y) {
        const uint8_t v=(y&1)?bottom:top;
        std::fill_n(f.u.begin()+static_cast<size_t>(y)*cw,cw,v);
        std::fill_n(f.v.begin()+static_cast<size_t>(y)*cw,cw,v);
    }
    return f;
}

static Frame mixed_b_direction_frame(int w,int h) {
    Frame f=field_frame(w,h,128,128);
    const int mbw=(w+15)/16;
    const int field_mbh=((((h+15)/16)+1)&~1)/2;
    const int cw=w/2,ch=h/2;
    for (int field=0;field<2;++field) {
        for (int my=0;my<field_mbh;++my) for (int mx=0;mx<mbw;++mx) {
            const bool forward=((mx+my)&1)==0;
            const uint8_t value=forward ? (field?40:20) : (field?240:180);
            for (int yy=0;yy<16;++yy) for (int xx=0;xx<16;++xx) {
                const int x=mx*16+xx,fy=my*16+yy,py=2*fy+field;
                if (x<w && py<h) f.y[static_cast<size_t>(py)*w+x]=value;
            }
            for (int yy=0;yy<8;++yy) for (int xx=0;xx<8;++xx) {
                const int x=mx*8+xx,fy=my*8+yy,py=2*fy+field;
                if (x<cw && py<ch) {
                    f.u[static_cast<size_t>(py)*cw+x]=value;
                    f.v[static_cast<size_t>(py)*cw+x]=value;
                }
            }
        }
    }
    return f;
}

static Frame sparse_field_chroma_frame(int w,int h) {
    Frame f=field_frame(w,h,128,128);
    const int mbw=(w+15)/16;
    const int full_mbh=((((h+15)/16)+1)&~1);
    const int field_mbh=full_mbh/2;
    const int cw=w/2,ch=h/2;
    for (int field=0;field<2;++field) {
        for (int my=0;my<field_mbh;++my) for (int mx=0;mx<mbw;++mx) {
            const unsigned mask=static_cast<unsigned>((my*mbw+mx)%63)+1u;
            for (int k=0;k<4;++k) if (mask&(1u<<(5-k))) {
                for (int yy=0;yy<8;++yy) for (int xx=0;xx<8;++xx) {
                    const int x=mx*16+(k&1)*8+xx;
                    const int fy=my*16+((k>>1)&1)*8+yy;
                    const int py=2*fy+field;
                    if (x<w && py<h)
                        f.y[static_cast<size_t>(py)*w+x]=static_cast<uint8_t>(170+((xx*5+yy*7+k*11)%52));
                }
            }
            if (mask&(1u<<1)) {
                for (int yy=0;yy<8;++yy) for (int xx=0;xx<8;++xx) {
                    const int x=mx*8+xx,fy=my*8+yy,py=2*fy+field;
                    if (x<cw && py<ch)
                        f.u[static_cast<size_t>(py)*cw+x]=static_cast<uint8_t>(170+((xx*3+yy*5)%50));
                }
            }
            if (mask&(1u<<0)) {
                for (int yy=0;yy<8;++yy) for (int xx=0;xx<8;++xx) {
                    const int x=mx*8+xx,fy=my*8+yy,py=2*fy+field;
                    if (x<cw && py<ch)
                        f.v[static_cast<size_t>(py)*cw+x]=static_cast<uint8_t>(60+((xx*7+yy*3)%50));
                }
            }
        }
    }
    return f;
}

static EncoderConfig config() {
    EncoderConfig c;
    c.width=64; c.height=96;
    c.fps={30000,1001};
    c.syntax=StreamSyntax::Advanced;
    c.scan_mode=VC1_SCAN_INTERLACED_TFF;
    c.pqindex=5;
    c.dquant=false;
    c.variable_transforms=false;
    c.motion_search_range=32;
    return c;
}

static void append(std::vector<uint8_t>& out,const std::vector<uint8_t>& add) {
    out.insert(out.end(),add.begin(),add.end());
}

static void write_stream(const std::filesystem::path& path,const std::vector<uint8_t>& bytes) {
    std::ofstream f(path,std::ios::binary);
    if (!f) throw std::runtime_error("could not create field-reference decoder fixture");
    f.write(reinterpret_cast<const char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
    if (!f) throw std::runtime_error("could not write field-reference decoder fixture");
}

static void write_frame(const std::filesystem::path& path,const Frame& frame) {
    std::ofstream f(path,std::ios::binary);
    if (!f) throw std::runtime_error("could not create expected reconstruction fixture");
    f.write(reinterpret_cast<const char*>(frame.y.data()),static_cast<std::streamsize>(frame.y.size()));
    f.write(reinterpret_cast<const char*>(frame.u.data()),static_cast<std::streamsize>(frame.u.size()));
    f.write(reinterpret_cast<const char*>(frame.v.data()),static_cast<std::streamsize>(frame.v.size()));
    if (!f) throw std::runtime_error("could not write expected reconstruction fixture");
}

int main(int argc,char** argv) try {
    const auto c=config();
    Vc1Encoder enc(c);
    const size_t field_mbs=static_cast<size_t>((c.width+15)/16) *
                           static_cast<size_t>((((c.height+15)/16)+1)&~1) / 2;

    // P: the top field exactly matches the opposite-polarity previous field;
    // the bottom field then ties toward the same-polarity previous field.
    // This proves NUMREF=1 is not merely signalled: both legal P references
    // are reachable by the field reference selector.
    const Frame past=field_frame(c.width,c.height,20,220);
    const Frame past_ref=enc.reconstruct_i_picture(past);
    const Frame pcur=field_frame(c.width,c.height,20,20);
    auto p=enc.encode_p_interlaced_fields(pcur,past_ref,false);
    if (!p.field_moved_macroblocks)
        throw std::runtime_error("P field motion search did not exercise a nonzero field MV");
    if (!p.field_same_macroblocks || !p.field_opposite_macroblocks)
        throw std::runtime_error("P field reference selector did not exercise both NUMREF=1 references: same="+std::to_string(p.field_same_macroblocks)+" opposite="+std::to_string(p.field_opposite_macroblocks));

    const Frame future=field_frame(c.width,c.height,180,240);
    const Frame future_ref=enc.reconstruct_i_picture(future);
    struct BCase { uint8_t top; size_t Vc1Encoder::BEncodeResult::*member; const char* name; };
    if (argc >= 2) std::filesystem::create_directories(argv[1]);
    const BCase cases[] = {
        { 20,  &Vc1Encoder::BEncodeResult::field_forward_same_macroblocks,     "forward-same" },
        { 220, &Vc1Encoder::BEncodeResult::field_forward_opposite_macroblocks, "forward-opposite" },
        { 180, &Vc1Encoder::BEncodeResult::field_backward_same_macroblocks,    "backward-same" },
        { 240, &Vc1Encoder::BEncodeResult::field_backward_opposite_macroblocks,"backward-opposite" },
    };
    for (const auto& tc:cases) {
        // Only the first field matters for this reachability assertion. Give
        // the second field a neutral value; the encoder may choose any legal
        // reference for it.
        const Frame bcur=field_frame(c.width,c.height,tc.top,128);
        auto b=enc.encode_b_interlaced_fields(bcur,past_ref,future_ref,1,2,false,false);
        if (!(b.*(tc.member)))
            throw std::runtime_error(std::string("B field reference selector did not reach ")+tc.name);
        if (!b.field_moved_macroblocks)
            throw std::runtime_error(std::string("B field motion search did not exercise a nonzero MV for ")+tc.name);
        if (b.data.empty()) throw std::runtime_error("B field encoder produced no bitstream");

        if (argc >= 2) {
            std::vector<uint8_t> stream;
            append(stream,enc.sequence_header());
            append(stream,enc.entry_point());
            append(stream,enc.encode_i_picture(past));
            append(stream,enc.encode_i_picture(future));
            append(stream,b.data);
            const auto bpath=std::filesystem::path(argv[1])/(std::string("b-")+tc.name+".vc1");
            write_stream(bpath,stream);
            std::ofstream expected(bpath.string()+".expected.yuv",std::ios::binary);
            expected.write(reinterpret_cast<const char*>(b.reconstructed.y.data()),static_cast<std::streamsize>(b.reconstructed.y.size()));
            expected.write(reinterpret_cast<const char*>(b.reconstructed.u.data()),static_cast<std::streamsize>(b.reconstructed.u.size()));
            expected.write(reinterpret_cast<const char*>(b.reconstructed.v.data()),static_cast<std::streamsize>(b.reconstructed.v.size()));
        }
    }
    // Regression for 0.1.104's field-B FORWARDMB RAW-plane bug.  A
    // high-entropy forward/backward map makes choose_bitplane() select
    // IMODE_RAW.  In RAW mode the direction bit lives in each macroblock
    // after MBMODE; omitting those bits shifts BMVTYPE/MVDATA parsing and can
    // create intermittent 16x32 colored/DCT-like corruption on B fields.
    EncoderConfig mixed_cfg=config();
    mixed_cfg.width=128; mixed_cfg.height=128;
    mixed_cfg.motion_search_range=8; mixed_cfg.motion_local_search_range=8;
    mixed_cfg.dquant=false; mixed_cfg.variable_transforms=false; mixed_cfg.loop_filter=false;
    Vc1Encoder mixed_enc(mixed_cfg);
    const Frame mixed_past=field_frame(mixed_cfg.width,mixed_cfg.height,20,40);
    const Frame mixed_future=field_frame(mixed_cfg.width,mixed_cfg.height,180,240);
    const Frame mixed_cur=mixed_b_direction_frame(mixed_cfg.width,mixed_cfg.height);
    const Frame mixed_past_ref=mixed_enc.reconstruct_i_picture(mixed_past);
    const Frame mixed_future_ref=mixed_enc.reconstruct_i_picture(mixed_future);
    auto mixed_b=mixed_enc.encode_b_interlaced_fields(mixed_cur,mixed_past_ref,mixed_future_ref,1,2,false,false);
    if (!mixed_b.forward_macroblocks || !mixed_b.backward_macroblocks)
        throw std::runtime_error("mixed B-field fixture did not exercise both forward and backward macroblocks");
    std::vector<uint8_t> checker_plane(32);
    for (int y=0;y<4;++y) for (int x=0;x<8;++x)
        checker_plane[static_cast<size_t>(y)*8+x]=static_cast<uint8_t>(((x+y)&1)==0);
    if (!Vc1Encoder::choose_bitplane(checker_plane,8,4).raw)
        throw std::runtime_error("mixed B-field regression no longer exercises IMODE_RAW");
    if (argc >= 2) {
        std::vector<uint8_t> mixed_stream;
        append(mixed_stream,mixed_enc.sequence_header());
        append(mixed_stream,mixed_enc.entry_point());
        append(mixed_stream,mixed_enc.encode_i_picture(mixed_past));
        append(mixed_stream,mixed_enc.encode_i_picture(mixed_future));
        append(mixed_stream,mixed_b.data);
        const auto mixed_path=std::filesystem::path(argv[1])/"b-mixed-forward-backward-raw.vc1";
        write_stream(mixed_path,mixed_stream);
        write_frame(mixed_path.string()+".expected.yuv",mixed_b.reconstructed);
    }

    if (argc >= 2) {
        std::filesystem::create_directories(argv[1]);
        // Standalone FCM=10 I-anchor fixture.  Predictive field pictures can
        // parse successfully even when an interlaced I reconstruction is wrong,
        // so keep the anchor itself under decoder-equivalence coverage.
        std::vector<uint8_t> istream;
        append(istream,enc.sequence_header());
        append(istream,enc.entry_point());
        append(istream,enc.encode_i_picture(past));
        const auto ipath=std::filesystem::path(argv[1])/"i-frame-interlace.vc1";
        write_stream(ipath,istream);
        write_frame(ipath.string()+".expected.yuv",past_ref);

        const Frame flat_i=field_frame(c.width,c.height,128,128);
        const Frame flat_i_ref=enc.reconstruct_i_picture(flat_i);
        std::vector<uint8_t> flat_istream;
        append(flat_istream,enc.sequence_header());
        append(flat_istream,enc.entry_point());
        append(flat_istream,enc.encode_i_picture(flat_i));
        const auto flat_ipath=std::filesystem::path(argv[1])/"i-frame-interlace-flat.vc1";
        write_stream(flat_ipath,flat_istream);
        write_frame(flat_ipath.string()+".expected.yuv",flat_i_ref);
        if (argc >= 3 && std::string(argv[2]) == "i-only") {
            std::cout << "interlaced I-anchor fixtures ok\n";
            return 0;
        }

        std::vector<uint8_t> stream=istream;
        append(stream,p.data);
        write_stream(std::filesystem::path(argv[1])/"p-same-opposite.vc1",stream);

        // 0.1.103 regression: the second field can reference the already
        // reconstructed first field of the current P picture.  With FASTUVMC,
        // VC-1 applies the opposite-field parity correction before rounding the
        // derived chroma MV to an even half-chroma vector.  Reversing those two
        // operations only differs for vectors that cross zero (for example
        // stored my=-2 in the second field), corrupting U/V while luma remains
        // decoder-equivalent.  Sparse per-block texture makes the resulting
        // 16x32 field-macroblock corruption visible and keeps this exact path
        // covered independently of the flat reference fixtures above.
        EncoderConfig sparse_cfg=config();
        sparse_cfg.width=256; sparse_cfg.height=128;
        sparse_cfg.motion_search_range=8; sparse_cfg.motion_local_search_range=8;
        sparse_cfg.dquant=false; sparse_cfg.variable_transforms=false;
        sparse_cfg.loop_filter=false; sparse_cfg.quantizer_type=QuantizerType::Uniform;
        Vc1Encoder sparse_enc(sparse_cfg);
        const Frame sparse_past=field_frame(sparse_cfg.width,sparse_cfg.height,128,128);
        const Frame sparse_past_ref=sparse_enc.reconstruct_i_picture(sparse_past);
        const Frame sparse_cur=sparse_field_chroma_frame(sparse_cfg.width,sparse_cfg.height);
        auto sparse_p=sparse_enc.encode_p_interlaced_fields(sparse_cur,sparse_past_ref,false);
        if (!sparse_p.field_opposite_macroblocks || !sparse_p.field_moved_macroblocks)
            throw std::runtime_error("sparse chroma regression did not exercise opposite-field nonzero motion");
        std::vector<uint8_t> sparse_stream;
        append(sparse_stream,sparse_enc.sequence_header());
        append(sparse_stream,sparse_enc.entry_point());
        append(sparse_stream,sparse_enc.encode_i_picture(sparse_past));
        append(sparse_stream,sparse_p.data);
        const auto sparse_path=std::filesystem::path(argv[1])/"p-sparse-opposite-chroma.vc1";
        write_stream(sparse_path,sparse_stream);
        write_frame(sparse_path.string()+".expected.yuv",sparse_p.reconstructed);

        // 1920x1080 P-only decoder fixtures.  These bypass the expensive
        // production I-picture analysis while retaining the exact Advanced
        // sequence/entry/P-field syntax used by --bframes 0.
        EncoderConfig hdp=config();
        hdp.width=1920; hdp.height=1080;
        hdp.dquant=true; hdp.variable_transforms=true; hdp.motion_search_range=1024;
        Vc1Encoder hdpenc(hdp);
        const Frame hdppast=field_frame(hdp.width,hdp.height,20,220);
        const Frame hdppast_ref=hdpenc.reconstruct_i_picture(hdppast);
        const Frame hdpsame=field_frame(hdp.width,hdp.height,20,220);
        const Frame hdpopposite=field_frame(hdp.width,hdp.height,220,20);
        const Frame hdpresidual=field_frame(hdp.width,hdp.height,96,144);
        for (const auto& pc : std::array<std::pair<const char*,const Frame*>,3>{{
                 {"p-1080i-same.vc1", &hdpsame}, {"p-1080i-opposite.vc1", &hdpopposite},
                 {"p-1080i-residual.vc1", &hdpresidual}}}) {
            auto hp=hdpenc.encode_p_interlaced_fields(*pc.second,hdppast_ref,false);
            std::vector<uint8_t> hs;
            append(hs,hdpenc.sequence_header());
            append(hs,hdpenc.entry_point());
            append(hs,hdpenc.encode_i_picture(hdppast));
            append(hs,hp.data);
            const auto outpath=std::filesystem::path(argv[1])/pc.first;
            write_stream(outpath,hs);
            std::ofstream ey(outpath.string()+".expected.yuv",std::ios::binary);
            ey.write(reinterpret_cast<const char*>(hp.reconstructed.y.data()),static_cast<std::streamsize>(hp.reconstructed.y.size()));
            ey.write(reinterpret_cast<const char*>(hp.reconstructed.u.data()),static_cast<std::streamsize>(hp.reconstructed.u.size()));
            ey.write(reinterpret_cast<const char*>(hp.reconstructed.v.data()),static_cast<std::streamsize>(hp.reconstructed.v.size()));
        }

        // Reference-chain regression for --bframes 0 semantics.  P1 deliberately
        // uses an opposite field from the woven I anchor; P2 then references the
        // FCM=11 P1 anchor.  Keep exact encoder reconstructions so the decoder-
        // facing Python test can detect reference-state drift even when parsing
        // succeeds without an error message.
        const Frame chain_i=field_frame(c.width,c.height,20,220);
        const Frame chain_i_ref=enc.reconstruct_i_picture(chain_i);
        const Frame chain_p1_src=field_frame(c.width,c.height,220,20);
        auto chain_p1=enc.encode_p_interlaced_fields(chain_p1_src,chain_i_ref,false);
        const Frame chain_p2_src=chain_p1.reconstructed;
        auto chain_p2=enc.encode_p_interlaced_fields(chain_p2_src,chain_p1.reconstructed,true);
        std::vector<uint8_t> chain_stream;
        append(chain_stream,enc.sequence_header());
        append(chain_stream,enc.entry_point());
        append(chain_stream,enc.encode_i_picture(chain_i));
        append(chain_stream,chain_p1.data);
        append(chain_stream,chain_p2.data);
        const auto chain_path=std::filesystem::path(argv[1])/"p-field-chain.vc1";
        write_stream(chain_path,chain_stream);
        std::ofstream chain_expected(chain_path.string()+".expected.yuv",std::ios::binary);
        for (const auto* rf : {&chain_p1.reconstructed,&chain_p2.reconstructed}) {
            chain_expected.write(reinterpret_cast<const char*>(rf->y.data()),static_cast<std::streamsize>(rf->y.size()));
            chain_expected.write(reinterpret_cast<const char*>(rf->u.data()),static_cast<std::streamsize>(rf->u.size()));
            chain_expected.write(reinterpret_cast<const char*>(rf->v.data()),static_cast<std::streamsize>(rf->v.size()));
        }

        // Exact shape of the user-reported 0.1.96 failure: 1920x1080 TFF,
        // default DQUANT/variable-transform/extended-MV header features, with a
        // B/B field picture selecting the future same-polarity field.  0.1.96
        // produced a second-field payload of 8232 bits and FFmpeg reported
        // bit overconsumption after misreading MVDATA as BMVTYPE.
        EncoderConfig hd=config();
        hd.width=1920; hd.height=1080;
        hd.dquant=true; hd.variable_transforms=true; hd.motion_search_range=1024;
        Vc1Encoder hdenc(hd);
        const Frame hdpast=field_frame(hd.width,hd.height,20,40);
        const Frame hdfuture=field_frame(hd.width,hd.height,180,240);
        const Frame hdb=field_frame(hd.width,hd.height,180,240);
        const Frame hdpr=hdenc.reconstruct_i_picture(hdpast);
        const Frame hdfr=hdenc.reconstruct_i_picture(hdfuture);
        auto hdbit=hdenc.encode_b_interlaced_fields(hdb,hdpr,hdfr,1,2,false,false);
        const size_t hd_field_mbs=static_cast<size_t>((hd.width+15)/16) *
                                  static_cast<size_t>((((hd.height+15)/16)+1)&~1) / 2;
        if (!hdbit.field_backward_same_macroblocks)
            throw std::runtime_error("1080i reproducer did not exercise backward-same B-field syntax");
        std::vector<uint8_t> hdstream;
        append(hdstream,hdenc.sequence_header());
        append(hdstream,hdenc.entry_point());
        append(hdstream,hdenc.encode_i_picture(hdpast));
        append(hdstream,hdenc.encode_i_picture(hdfuture));
        append(hdstream,hdbit.data);
        const auto hdbpath=std::filesystem::path(argv[1])/"b-1080i-backward-same.vc1";
        write_stream(hdbpath,hdstream);
        write_frame(hdbpath.string()+".expected.yuv",hdbit.reconstructed);
    }
    std::cout << "interlaced field reference selection ok: P same/opposite; "
                 "B forward/backward x same/opposite\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
