#include <libvc1.h>
#include "matroska_muxer.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {

struct Rational { int64_t num=0, den=1; };
struct Frame { int width=0,height=0; std::vector<uint8_t> y,u,v; };

class Y4mReader {
public:
    explicit Y4mReader(std::istream& in) : in_(in) { parse_header(); }
    int width() const { return w_; }
    int height() const { return h_; }
    Rational fps() const { return fps_; }
    vc1_scan_mode_e scan_mode() const { return scan_mode_; }
    const char* scan_tag() const { return scan_mode_==VC1_SCAN_INTERLACED_TFF?"It":(scan_mode_==VC1_SCAN_INTERLACED_BFF?"Ib":"Ip"); }
    bool read(Frame& f) {
        std::string line;
        if (!std::getline(in_, line)) return false;
        if (line.rfind("FRAME", 0) != 0)
            throw std::runtime_error("expected Y4M FRAME header");
        f.width = w_; f.height = h_;
        f.y.resize(static_cast<size_t>(w_) * h_);
        f.u.resize(static_cast<size_t>((w_+1)/2) * ((h_+1)/2));
        f.v.resize(f.u.size());
        read_exact(f.y); read_exact(f.u); read_exact(f.v);
        return true;
    }
private:
    void read_exact(std::vector<uint8_t>& b) {
        in_.read(reinterpret_cast<char*>(b.data()), static_cast<std::streamsize>(b.size()));
        if (in_.gcount() != static_cast<std::streamsize>(b.size()))
            throw std::runtime_error("truncated Y4M frame");
    }
    void parse_header() {
        std::string line;
        if (!std::getline(in_, line) || line.rfind("YUV4MPEG2", 0) != 0)
            throw std::runtime_error("input is not YUV4MPEG2");
        std::istringstream ss(line);
        std::string tok; ss >> tok;
        std::string chroma = "420";
        std::string interlace = "?";
        while (ss >> tok) {
            if (tok.size() < 2) continue;
            switch (tok[0]) {
                case 'W': w_ = std::stoi(tok.substr(1)); break;
                case 'H': h_ = std::stoi(tok.substr(1)); break;
                case 'F': {
                    auto p = tok.find(':');
                    if (p == std::string::npos) throw std::runtime_error("bad Y4M frame rate");
                    fps_.num = std::stoll(tok.substr(1, p-1));
                    fps_.den = std::stoll(tok.substr(p+1));
                    break;
                }
                case 'I': interlace = tok.substr(1); break;
                case 'C': chroma = tok.substr(1); break;
                default: break;
            }
        }
        if (w_ <= 0 || h_ <= 0)
            throw std::runtime_error("Y4M dimensions must be positive");
        if (fps_.num <= 0 || fps_.den <= 0)
            throw std::runtime_error("Y4M input must specify a valid frame rate");
        if (!(chroma.rfind("420",0)==0))
            throw std::runtime_error("this prototype accepts only 8-bit 4:2:0 Y4M input");
        if (interlace == "p" || interlace == "?") scan_mode_=VC1_SCAN_PROGRESSIVE;
        else if (interlace == "t") scan_mode_=VC1_SCAN_INTERLACED_TFF;
        else if (interlace == "b") scan_mode_=VC1_SCAN_INTERLACED_BFF;
        else if (interlace == "m") throw std::runtime_error("mixed/per-frame Y4M interlace mode is not supported; use Ip, It, or Ib");
        else throw std::runtime_error("unsupported Y4M interlace mode; use Ip, It, or Ib");
        if (w_ > 8192 || h_ > 8192)
            throw std::runtime_error("VC-1 coded dimensions cannot exceed 8192 samples in either axis");
    }
    std::istream& in_;
    int w_ = 0, h_ = 0;
    Rational fps_{0,1};
    vc1_scan_mode_e scan_mode_ = VC1_SCAN_PROGRESSIVE;
};

static uint32_t mpeg_crc32(const uint8_t* p, size_t n) {
    uint32_t crc=0xffffffffu;
    for (size_t i=0;i<n;++i) {
        crc ^= static_cast<uint32_t>(p[i]) << 24;
        for (int b=0;b<8;++b) crc = (crc&0x80000000u) ? (crc<<1)^0x04c11db7u : crc<<1;
    }
    return crc;
}

class M2tsMuxer {
public:
    explicit M2tsMuxer(std::ostream& out) : out_(out) { write_pat(); write_pmt(); }

    void write_video_access_unit(const std::vector<uint8_t>& au, uint64_t pts90, uint64_t dts90, bool key=true) {
        std::vector<uint8_t> pes;
        pes.reserve(au.size()+40);
        pes.insert(pes.end(), {0x00,0x00,0x01,0xfd, 0x00,0x00}); // extended_stream_id PES; unbounded video PES length
        pes.push_back(0x84); // '10', data_alignment_indicator
        const bool has_dts=pts90!=dts90;
        pes.push_back(static_cast<uint8_t>(has_dts?0xc1:0x81)); // PTS+DTS or PTS only, plus PES_extension_flag
        pes.push_back(static_cast<uint8_t>(has_dts?13:8));
        write_timestamp(pes,pts90,has_dts?0x3:0x2);
        if (has_dts) write_timestamp(pes,dts90,0x1);
        pes.push_back(0x0f); // reserved='111' + PES_extension_flag_2
        pes.push_back(0x81); // marker + extension_field_length=1
        pes.push_back(0x55); // stream_id_extension_flag=0, VC-1 stream id extension
        pes.insert(pes.end(), au.begin(), au.end());
        constexpr uint64_t kDecodeLead90 = 9000; // 100 ms decoder lead over PCR.
        const uint64_t pcr90 = dts90 >= kDecodeLead90 ? dts90 - kDecodeLead90 : 0;
        if (video_aus_ != 0)
            ats_ = static_cast<uint32_t>((pcr90 * 300ull) & 0x3fffffffu);
        packetize_pes(0x1011, pes, pcr90, key);
        if ((video_aus_++ % 24) == 23) { write_pat(); write_pmt(); }
    }

    void write_video_access_unit(const std::vector<uint8_t>& au, uint64_t pts90, bool key=true) {
        write_video_access_unit(au,pts90,pts90,key);
    }

private:
    static void write_timestamp(std::vector<uint8_t>& p, uint64_t ts, uint8_t prefix) {
        ts &= ((1ull<<33)-1);
        p.push_back(static_cast<uint8_t>((prefix<<4) | (((ts>>30)&7)<<1) | 1));
        p.push_back(static_cast<uint8_t>(ts>>22));
        p.push_back(static_cast<uint8_t>((((ts>>15)&0x7f)<<1)|1));
        p.push_back(static_cast<uint8_t>(ts>>7));
        p.push_back(static_cast<uint8_t>(((ts&0x7f)<<1)|1));
    }
    void write_pat() {
        std::vector<uint8_t> s = {
            0x00, 0xb0, 0x0d, 0x00,0x01, 0xc1,0x00,0x00,
            0x00,0x01, 0xe1,0x00
        };
        append_crc(s); write_psi(0x0000,s);
    }
    void write_pmt() {
        // PMT: one VC-1 Advanced Profile stream, stream_type 0xEA, PID 0x1011.
        // Registration descriptor 'VC-1' is commonly used for carriage signaling.
        std::vector<uint8_t> s = {
            0x02, 0xb0, 0x18,
            0x00,0x01, 0xc1,0x00,0x00,
            0xf0,0x11,             // PCR PID 0x1011
            0xf0,0x00,             // program_info_length
            0xea, 0xf0,0x11,       // stream type + elementary PID
            0xf0,0x06,             // ES info length
            0x05,0x04,'V','C','-','1'
        };
        append_crc(s); write_psi(0x0100,s);
    }
    static void append_crc(std::vector<uint8_t>& s) {
        uint32_t c=mpeg_crc32(s.data(),s.size());
        s.push_back(static_cast<uint8_t>(c>>24));
        s.push_back(static_cast<uint8_t>(c>>16));
        s.push_back(static_cast<uint8_t>(c>>8));
        s.push_back(static_cast<uint8_t>(c));
    }
    void write_psi(uint16_t pid, const std::vector<uint8_t>& section) {
        std::vector<uint8_t> payload; payload.reserve(section.size()+1);
        payload.push_back(0); payload.insert(payload.end(),section.begin(),section.end());
        size_t off=0; bool first=true;
        while (off<payload.size()) {
            size_t take=std::min<size_t>(184,payload.size()-off);
            std::array<uint8_t,188> ts; ts.fill(0xff);
            ts[0]=0x47;
            ts[1]=static_cast<uint8_t>(((first?1:0)<<6)|((pid>>8)&0x1f));
            ts[2]=static_cast<uint8_t>(pid);
            ts[3]=static_cast<uint8_t>(0x10 | (cc_[pid]++ & 0xf));
            std::copy_n(payload.data()+off,take,ts.data()+4);
            emit(ts); off+=take; first=false;
        }
    }
    void packetize_pes(uint16_t pid, const std::vector<uint8_t>& pes, uint64_t pcr90, bool key) {
        size_t off=0; bool first=true;
        while (off<pes.size()) {
            std::array<uint8_t,188> ts; ts.fill(0xff);
            ts[0]=0x47;
            ts[1]=static_cast<uint8_t>(((first?1:0)<<6)|((pid>>8)&0x1f));
            ts[2]=static_cast<uint8_t>(pid);

            size_t remain=pes.size()-off;
            bool need_pcr=first;
            size_t mandatory = need_pcr ? 7 : 0; // flags + PCR
            size_t max_payload = need_pcr ? 176 : 184;
            size_t take=std::min(remain,max_payload);
            bool adapt = need_pcr || take < 184;
            size_t pos=4;
            if (adapt) {
                ts[3]=static_cast<uint8_t>(0x30 | (cc_[pid]++ & 0xf));
                size_t afl=183-take;
                if (afl < mandatory) { take=183-mandatory; afl=mandatory; }
                ts[pos++]=static_cast<uint8_t>(afl);
                if (afl>0) {
                    ts[pos++]=static_cast<uint8_t>((first && key ? 0x40:0) | (need_pcr?0x10:0));
                    size_t used=1;
                    if (need_pcr) {
                        uint64_t base=pcr90 & ((1ull<<33)-1); uint16_t ext=0;
                        ts[pos++]=static_cast<uint8_t>(base>>25);
                        ts[pos++]=static_cast<uint8_t>(base>>17);
                        ts[pos++]=static_cast<uint8_t>(base>>9);
                        ts[pos++]=static_cast<uint8_t>(base>>1);
                        ts[pos++]=static_cast<uint8_t>(((base&1)<<7)|0x7e|((ext>>8)&1));
                        ts[pos++]=static_cast<uint8_t>(ext);
                        used+=6;
                    }
                    while (used<afl) { ts[pos++]=0xff; ++used; }
                }
            } else {
                ts[3]=static_cast<uint8_t>(0x10 | (cc_[pid]++ & 0xf));
            }
            std::copy_n(pes.data()+off,take,ts.data()+pos);
            emit(ts);
            off+=take; first=false;
        }
    }
    void emit(const std::array<uint8_t,188>& ts) {
        // Blu-ray M2TS transport packet = 4-byte TP_extra_header + 188-byte TS packet.
        // CPI=0, ATS is a monotonically increasing 27 MHz modulo-2^30 timestamp.
        uint32_t h=ats_ & 0x3fffffffu;
        uint8_t x[4]={static_cast<uint8_t>(h>>24),static_cast<uint8_t>(h>>16),
                      static_cast<uint8_t>(h>>8),static_cast<uint8_t>(h)};
        out_.write(reinterpret_cast<char*>(x),4);
        out_.write(reinterpret_cast<const char*>(ts.data()),188);
        if (!out_) throw std::runtime_error("write failed");
        ats_=(ats_+900)&0x3fffffffu; // ~46.08 Mb/s notional arrival rate
    }
    std::ostream& out_;
    std::map<uint16_t,uint8_t> cc_;
    uint32_t ats_=0;
    uint64_t video_aus_=0;

};

class AsfVideoMuxer {
public:
    AsfVideoMuxer(std::ostream& out,int width,int height,Rational fps,
                  const std::array<uint8_t,4>& codec_tag,
                  const std::vector<uint8_t>& codec_private,uint64_t nominal_bitrate)
        : out_(out), width_(width), height_(height), fps_(fps), nominal_bitrate_(nominal_bitrate),
          codec_tag_(codec_tag) {
        if (codec_private.empty())
            throw std::runtime_error("ASF video codec-private data must not be empty");
        write_header(codec_private);
    }

    void write_video_frame(const std::vector<uint8_t>& frame,uint64_t pts_ms,uint64_t dts_ms,bool key) {
        if (finalized_) throw std::runtime_error("cannot write WMV frame after ASF finalization");
        if (frame.empty()) throw std::runtime_error("cannot mux an empty WMV frame");
        const uint8_t object_number=static_cast<uint8_t>(media_object_++ & 0xffu);
        size_t off=0;
        while (off<frame.size()) {
            constexpr size_t kBaseHeader=26; // no padding-length field
            const size_t take=std::min<size_t>(frame.size()-off,kPacketSize-kBaseHeader);
            write_packet(frame.data()+off,take,frame.size(),off,object_number,dts_ms,key);
            off+=take;
        }
        const uint64_t frame_ms=std::max<uint64_t>(1,(1000ull*static_cast<uint64_t>(fps_.den)+static_cast<uint64_t>(fps_.num)-1)/
                                                     static_cast<uint64_t>(fps_.num));
        duration_ms_=std::max(duration_ms_,pts_ms+frame_ms);
    }

    void finalize() {
        if (finalized_) return;
        finalized_=true;
        const std::streampos end=out_.tellp();
        if (end==std::streampos(-1)) throw std::runtime_error("cannot determine ASF output size");
        const uint64_t file_size=static_cast<uint64_t>(end);
        const uint64_t data_size=file_size-data_object_pos_;
        patch_u64(file_size_pos_,file_size);
        patch_u64(packet_count_pos_,packet_count_);
        patch_u64(play_duration_pos_,(duration_ms_+kPrerollMs)*10000ull);
        patch_u64(send_duration_pos_,duration_ms_*10000ull);
        patch_u32(max_bitrate_pos_,static_cast<uint32_t>(std::min<uint64_t>(nominal_bitrate_,0xffffffffull)));
        patch_u64(data_size_pos_,data_size);
        patch_u64(data_packet_count_pos_,packet_count_);
        out_.seekp(end);
        if (!out_) throw std::runtime_error("failed finalizing ASF/WMV output");
    }

private:
    static constexpr uint32_t kPacketSize=3200;
    static constexpr uint32_t kPrerollMs=3100;

    using Guid=std::array<uint8_t,16>;
    static constexpr Guid kHeader={{0x30,0x26,0xb2,0x75,0x8e,0x66,0xcf,0x11,0xa6,0xd9,0x00,0xaa,0x00,0x62,0xce,0x6c}};
    static constexpr Guid kFileProps={{0xa1,0xdc,0xab,0x8c,0x47,0xa9,0xcf,0x11,0x8e,0xe4,0x00,0xc0,0x0c,0x20,0x53,0x65}};
    static constexpr Guid kStreamProps={{0x91,0x07,0xdc,0xb7,0xb7,0xa9,0xcf,0x11,0x8e,0xe6,0x00,0xc0,0x0c,0x20,0x53,0x65}};
    static constexpr Guid kVideoMedia={{0xc0,0xef,0x19,0xbc,0x4d,0x5b,0xcf,0x11,0xa8,0xfd,0x00,0x80,0x5f,0x5c,0x44,0x2b}};
    static constexpr Guid kNoErrorCorrection={{0x00,0x57,0xfb,0x20,0x55,0x5b,0xcf,0x11,0xa8,0xfd,0x00,0x80,0x5f,0x5c,0x44,0x2b}};
    static constexpr Guid kHeaderExtension={{0xb5,0x03,0xbf,0x5f,0x2e,0xa9,0xcf,0x11,0x8e,0xe3,0x00,0xc0,0x0c,0x20,0x53,0x65}};
    static constexpr Guid kHeaderExtensionReserved={{0x11,0xd2,0xd3,0xab,0xba,0xa9,0xcf,0x11,0x8e,0xe6,0x00,0xc0,0x0c,0x20,0x53,0x65}};
    static constexpr Guid kData={{0x36,0x26,0xb2,0x75,0x8e,0x66,0xcf,0x11,0xa6,0xd9,0x00,0xaa,0x00,0x62,0xce,0x6c}};
    // Deterministic opaque ASF file-id GUID. It only needs to agree between
    // File Properties and Data Object for this single-file muxer.
    static constexpr Guid kFileId={{0x76,0x63,0x31,0x62,0x64,0x2d,0x77,0x6d,0x76,0x39,0x2d,0x30,0x31,0x35,0x00,0x01}};

    static void append_u16(std::vector<uint8_t>& b,uint16_t v) {
        b.push_back(static_cast<uint8_t>(v)); b.push_back(static_cast<uint8_t>(v>>8));
    }
    static void append_u32(std::vector<uint8_t>& b,uint32_t v) {
        for (int i=0;i<4;++i) b.push_back(static_cast<uint8_t>(v>>(8*i)));
    }
    static void append_u64(std::vector<uint8_t>& b,uint64_t v) {
        for (int i=0;i<8;++i) b.push_back(static_cast<uint8_t>(v>>(8*i)));
    }
    static void append_guid(std::vector<uint8_t>& b,const Guid& g) { b.insert(b.end(),g.begin(),g.end()); }

    void raw(const std::vector<uint8_t>& b) {
        out_.write(reinterpret_cast<const char*>(b.data()),static_cast<std::streamsize>(b.size()));
        if (!out_) throw std::runtime_error("write failed on ASF/WMV output");
    }
    void raw_byte(uint8_t v) { out_.put(static_cast<char>(v)); if(!out_) throw std::runtime_error("write failed on ASF/WMV output"); }
    void raw_u16(uint16_t v) { raw_byte(static_cast<uint8_t>(v)); raw_byte(static_cast<uint8_t>(v>>8)); }
    void raw_u32(uint32_t v) { for(int i=0;i<4;++i) raw_byte(static_cast<uint8_t>(v>>(8*i))); }

    void write_header(const std::vector<uint8_t>& codec_private) {
        std::vector<uint8_t> file;
        append_guid(file,kFileProps); append_u64(file,104);
        append_guid(file,kFileId);
        const size_t file_size_rel=file.size(); append_u64(file,0);
        append_u64(file,0); // creation date
        const size_t packet_count_rel=file.size(); append_u64(file,0);
        const size_t play_rel=file.size(); append_u64(file,0);
        const size_t send_rel=file.size(); append_u64(file,0);
        append_u64(file,kPrerollMs);
        append_u32(file,2); // seekable
        append_u32(file,kPacketSize); append_u32(file,kPacketSize);
        const size_t max_bitrate_rel=file.size(); append_u32(file,0);

        std::vector<uint8_t> ext;
        append_guid(ext,kHeaderExtension); append_u64(ext,46);
        append_guid(ext,kHeaderExtensionReserved); append_u16(ext,6); append_u32(ext,0);

        const uint64_t format_data_size=40ull+codec_private.size();
        const uint64_t type_specific_size=11ull+format_data_size;
        const uint64_t stream_object_size=78ull+type_specific_size;
        if (format_data_size>0xffffu || type_specific_size>0xffffffffu)
            throw std::runtime_error("ASF video codec-private data is too large");

        std::vector<uint8_t> stream;
        append_guid(stream,kStreamProps); append_u64(stream,stream_object_size);
        append_guid(stream,kVideoMedia); append_guid(stream,kNoErrorCorrection);
        append_u64(stream,0);            // time offset
        append_u32(stream,static_cast<uint32_t>(type_specific_size)); // type-specific data size
        append_u32(stream,0);            // error-correction data size
        append_u16(stream,1);            // stream number 1, not encrypted
        append_u32(stream,0);            // reserved
        append_u32(stream,static_cast<uint32_t>(width_));
        append_u32(stream,static_cast<uint32_t>(height_));
        stream.push_back(2);
        append_u16(stream,static_cast<uint16_t>(format_data_size));
        // ASF stores the codec-private bytes as the tail of its Format Data,
        // and the first DWORD covers that complete Format Data blob.
        append_u32(stream,static_cast<uint32_t>(format_data_size));
        append_u32(stream,static_cast<uint32_t>(width_));
        append_u32(stream,static_cast<uint32_t>(height_));
        append_u16(stream,1); append_u16(stream,24);
        stream.insert(stream.end(),codec_tag_.begin(),codec_tag_.end());
        const uint64_t img=static_cast<uint64_t>(width_)*static_cast<uint64_t>(height_)*3ull;
        append_u32(stream,static_cast<uint32_t>(std::min<uint64_t>(img,0xffffffffull)));
        append_u32(stream,0); append_u32(stream,0); append_u32(stream,0); append_u32(stream,0);
        stream.insert(stream.end(),codec_private.begin(),codec_private.end());

        const uint64_t header_size=30ull+file.size()+ext.size()+stream.size();
        std::vector<uint8_t> head;
        append_guid(head,kHeader); append_u64(head,header_size);
        append_u32(head,3); head.push_back(1); head.push_back(2);
        const uint64_t file_object_base=head.size();
        head.insert(head.end(),file.begin(),file.end());
        head.insert(head.end(),ext.begin(),ext.end());
        head.insert(head.end(),stream.begin(),stream.end());

        file_size_pos_=file_object_base+file_size_rel;
        packet_count_pos_=file_object_base+packet_count_rel;
        play_duration_pos_=file_object_base+play_rel;
        send_duration_pos_=file_object_base+send_rel;
        max_bitrate_pos_=file_object_base+max_bitrate_rel;
        raw(head);

        data_object_pos_=static_cast<uint64_t>(out_.tellp());
        std::vector<uint8_t> dh;
        append_guid(dh,kData);
        data_size_pos_=data_object_pos_+dh.size();
        append_u64(dh,0);
        append_guid(dh,kFileId);
        data_packet_count_pos_=data_object_pos_+dh.size();
        append_u64(dh,0);
        dh.push_back(1); dh.push_back(1);
        raw(dh);
    }

    void write_packet(const uint8_t* data,size_t n,size_t object_size,size_t object_offset,
                      uint8_t object_number,uint64_t dts_ms,bool key) {
        constexpr size_t no_padding_header=26;
        const size_t leftover=kPacketSize-(no_padding_header+n);
        uint8_t len_flags=0;
        size_t padding=0, padfield=0;
        if (leftover) {
            if (leftover<=256) { len_flags=0x08; padfield=1; }
            else { len_flags=0x10; padfield=2; }
            padding=leftover-padfield;
        }

        std::vector<uint8_t> pkt;
        pkt.reserve(kPacketSize);
        pkt.push_back(0x82); pkt.push_back(0); pkt.push_back(0); // error correction
        pkt.push_back(len_flags);
        pkt.push_back(0x5d); // stream#, object#, offset and replicated-data lengths
        // ASF stores the optional padding-length field in the packet parsing
        // information before send-time/duration.  Keeping that order is
        // essential: a parser otherwise mistakes our short-packet padding for
        // another payload fragment.
        if (padfield==1) pkt.push_back(static_cast<uint8_t>(padding));
        else if (padfield==2) append_u16(pkt,static_cast<uint16_t>(padding));
        append_u32(pkt,static_cast<uint32_t>(std::min<uint64_t>(dts_ms,0xffffffffull)));
        append_u16(pkt,0);

        pkt.push_back(static_cast<uint8_t>(1 | (key?0x80:0))); // stream 1
        pkt.push_back(object_number);
        append_u32(pkt,static_cast<uint32_t>(object_offset));
        pkt.push_back(8);
        append_u32(pkt,static_cast<uint32_t>(object_size));
        // ASF's replicated timestamp is fed the sample DTS for codecs with
        // frame reordering.  This matches the convention used by FFmpeg's ASF
        // muxer (which passes pkt->dts to put_frame), allowing the WMV3 decoder
        // to reorder B pictures without exposing non-monotonic packet DTS.
        append_u32(pkt,static_cast<uint32_t>(std::min<uint64_t>(dts_ms+kPrerollMs,0xffffffffull)));
        pkt.insert(pkt.end(),data,data+n);
        pkt.insert(pkt.end(),padding,0);
        if (pkt.size()!=kPacketSize) throw std::runtime_error("internal ASF packet-size mismatch");
        raw(pkt);
        ++packet_count_;
    }

    void patch_u64(uint64_t pos,uint64_t v) {
        const std::streampos cur=out_.tellp();
        out_.seekp(static_cast<std::streamoff>(pos),std::ios::beg);
        for(int i=0;i<8;++i) raw_byte(static_cast<uint8_t>(v>>(8*i)));
        out_.seekp(cur);
    }
    void patch_u32(uint64_t pos,uint32_t v) {
        const std::streampos cur=out_.tellp();
        out_.seekp(static_cast<std::streamoff>(pos),std::ios::beg);
        raw_u32(v);
        out_.seekp(cur);
    }

    std::ostream& out_;
    int width_=0,height_=0;
    Rational fps_{};
    uint64_t nominal_bitrate_=0;
    std::array<uint8_t,4> codec_tag_{};
    uint64_t packet_count_=0,media_object_=0,duration_ms_=0;
    uint64_t data_object_pos_=0;
    uint64_t file_size_pos_=0,packet_count_pos_=0,play_duration_pos_=0,send_duration_pos_=0,max_bitrate_pos_=0;
    uint64_t data_size_pos_=0,data_packet_count_pos_=0;
    bool finalized_=false;
};


static size_t find_advanced_bdu(const std::vector<uint8_t>& au,uint8_t code,size_t from=0) {
    for (size_t i=from;i+4<=au.size();++i)
        if (au[i]==0x00 && au[i+1]==0x00 && au[i+2]==0x01 && au[i+3]==code) return i;
    return std::string::npos;
}

struct Wvc1FirstAccessUnit {
    std::vector<uint8_t> codec_private;
    std::vector<uint8_t> frame_payload;
};

static Wvc1FirstAccessUnit prepare_wvc1_first_access_unit(const std::vector<uint8_t>& au,bool has_b_frames,bool interlaced) {
    const size_t seq=find_advanced_bdu(au,0x0f);
    const size_t ep=find_advanced_bdu(au,0x0e,seq==std::string::npos?0:seq+4);
    const size_t frame=find_advanced_bdu(au,0x0d,ep==std::string::npos?0:ep+4);
    if (seq==std::string::npos || ep==std::string::npos || frame==std::string::npos || seq>=ep || ep>=frame)
        throw std::runtime_error("first Advanced Profile access unit lacks ordered sequence/entry-point/frame headers required by WVC1 ASF");

    Wvc1FirstAccessUnit r;
    // SMPTE 421M / ASF WVC1 binding byte. Bit 5 is NO_INTERLACESOURCE, so
    // clear it when the sequence advertises interlaced source/display timing.
    // One sequence header is emitted, slices are absent, and entry points may
    // recur at GOP boundaries. Bit 0 is the required reserved-one bit.
    uint8_t binding=static_cast<uint8_t>(0x10u | 0x04u | 0x01u);
    if (!interlaced) binding=static_cast<uint8_t>(binding | 0x20u);
    if (!has_b_frames) binding=static_cast<uint8_t>(binding | 0x02u);
    r.codec_private.push_back(binding);
    r.codec_private.insert(r.codec_private.end(),au.begin()+static_cast<std::ptrdiff_t>(seq),au.begin()+static_cast<std::ptrdiff_t>(frame));
    // Sequence/entry-point data are already supplied through CodecPrivateData;
    // the first ASF sample starts with the frame EBDU. Later GOP entry points
    // remain in-band exactly as produced by the Advanced Profile encoder.
    r.frame_payload.assign(au.begin()+static_cast<std::ptrdiff_t>(frame),au.end());
    return r;
}

static std::vector<uint8_t> prepare_wvc1_matroska_codec_private(const std::vector<uint8_t>& au) {
    const size_t seq=find_advanced_bdu(au,0x0f);
    const size_t ep=find_advanced_bdu(au,0x0e,seq==std::string::npos?0:seq+4);
    const size_t frame=find_advanced_bdu(au,0x0d,ep==std::string::npos?0:ep+4);
    if (seq==std::string::npos || ep==std::string::npos || frame==std::string::npos || seq>=ep || ep>=frame)
        throw std::runtime_error("first Advanced Profile access unit lacks ordered sequence/entry-point/frame headers required by WVC1 Matroska");
    // V_MS/VFW/FOURCC stores the sequence + entry-point EBDUs after the
    // BITMAPINFOHEADER.  Unlike ASF/WVC1, Matroska does not prepend the ASF
    // one-byte binding flags.  Keep these headers in the first coded sample as
    // well, matching the representation accepted from mkvmerge/MKVToolNix.
    return std::vector<uint8_t>(au.begin()+static_cast<std::ptrdiff_t>(seq),
                                au.begin()+static_cast<std::ptrdiff_t>(frame));
}



static uint64_t parse_bitrate_value(std::string text,const char* opt) {
    if (text.empty()) throw std::runtime_error(std::string(opt)+" requires a value");
    for (char& c:text) c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    double mult=1.0;
    auto strip=[&](const std::string& suffix,double m) {
        if (text.size()>=suffix.size() && text.compare(text.size()-suffix.size(),suffix.size(),suffix)==0) {
            text.resize(text.size()-suffix.size()); mult=m; return true;
        }
        return false;
    };
    if (!(strip("mbit/s",1e6)||strip("mbps",1e6)||strip("mbit",1e6)||strip("m",1e6)||
          strip("kbit/s",1e3)||strip("kbps",1e3)||strip("kbit",1e3)||strip("k",1e3)||
          strip("bit/s",1.0)||strip("bps",1.0))) mult=1.0;
    if (text.empty()) throw std::runtime_error(std::string("bad value for ")+opt);
    size_t used=0; const double v=std::stod(text,&used);
    if (used!=text.size() || !std::isfinite(v) || v<=0.0) throw std::runtime_error(std::string("bad value for ")+opt);
    const long double bits=static_cast<long double>(v)*mult;
    if (bits>static_cast<long double>(std::numeric_limits<uint64_t>::max())) throw std::runtime_error(std::string("value too large for ")+opt);
    return static_cast<uint64_t>(std::llround(bits));
}

static const char* compute_name(vc1_compute_backend_e c) {
    switch(c) {
        case VC1_COMPUTE_VULKAN: return "vulkan";
        case VC1_COMPUTE_CPU: return "cpu";
        default: return "auto";
    }
}

static const char* simd_name(vc1_simd_e s) {
    switch(s) {
        case VC1_SIMD_MIXED:return "mixed"; case VC1_SIMD_X86_64_V1:return "x86-64-v1"; case VC1_SIMD_X86_64_V2:return "x86-64-v2";
        case VC1_SIMD_X86_64_V3:return "x86-64-v3"; case VC1_SIMD_X86_64_V4:return "x86-64-v4";
        case VC1_SIMD_PRESCOTT:return "prescott"; case VC1_SIMD_CONROE:return "conroe";
        case VC1_SIMD_PENRYN:return "penryn"; case VC1_SIMD_SANDYBRIDGE:return "sandybridge";
        case VC1_SIMD_K10:return "k10"; case VC1_SIMD_BULLDOZER:return "bulldozer";
        case VC1_SIMD_PILEDRIVER:return "piledriver"; case VC1_SIMD_AVX2_PARTIAL:return "avx2-partial";
        case VC1_SIMD_AUTO:return "auto"; default:return "none";
    }
}

static std::string trim_ascii(std::string v) {
    auto ws=[](unsigned char c){ return std::isspace(c)!=0; };
    while(!v.empty() && ws(static_cast<unsigned char>(v.front()))) v.erase(v.begin());
    while(!v.empty() && ws(static_cast<unsigned char>(v.back()))) v.pop_back();
    return v;
}

static Rational parse_fps_value(std::string text,const char* opt) {
    text=trim_ascii(text);
    if(text.empty()) throw std::runtime_error(std::string(opt)+" requires a frame rate");
    auto reduce=[](int64_t n,int64_t d){
        if(n<=0||d<=0) throw std::runtime_error("frame rate numerator/denominator must be positive");
        int64_t a=n,b=d;while(b){const int64_t t=a%b;a=b;b=t;} return Rational{n/a,d/a};
    };
    const size_t sep=text.find_first_of("/:");
    if(sep!=std::string::npos){
        if(text.find_first_of("/:" ,sep+1)!=std::string::npos) throw std::runtime_error(std::string("bad value for ")+opt);
        size_t un=0,ud=0;const int64_t n=std::stoll(text.substr(0,sep),&un),d=std::stoll(text.substr(sep+1),&ud);
        if(un!=sep||ud!=text.size()-sep-1||n<=0||d<=0) throw std::runtime_error(std::string("bad value for ")+opt);
        return reduce(n,d);
    }
    size_t used=0;const double v=std::stod(text,&used);
    if(used!=text.size()||!std::isfinite(v)||v<=0.0||v>1000.0) throw std::runtime_error(std::string("bad value for ")+opt);
    static constexpr struct { double rate; int64_t n,d; } common[]={{24000.0/1001.0,24000,1001},{30000.0/1001.0,30000,1001},{60000.0/1001.0,60000,1001},{120000.0/1001.0,120000,1001}};
    for(const auto& c:common) if(std::abs(v-c.rate)<0.001 || std::abs(v-std::round(c.rate*1000.0)/1000.0)<1e-9) return Rational{c.n,c.d};
    // Decimal input is represented to micro-Hz precision and reduced. Exact
    // broadcast rates should use NUM/DEN (for example 24000/1001).
    const int64_t d=1000000,n=static_cast<int64_t>(std::llround(v*d)); return reduce(n,d);
}

static vc1_simd_e parse_simd_tier(std::string v,const char* opt,bool allow_auto=true) {
    v=trim_ascii(v);
    for(char& c:v)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if(allow_auto && v=="auto")return VC1_SIMD_AUTO;
    if(v=="none"||v=="scalar")return VC1_SIMD_NONE;
    if(v=="x86-64-v1"||v=="v1")return VC1_SIMD_X86_64_V1;
    if(v=="x86-64-v2"||v=="v2")return VC1_SIMD_X86_64_V2;
    if(v=="x86-64-v3"||v=="v3")return VC1_SIMD_X86_64_V3;
    if(v=="x86-64-v4"||v=="v4")return VC1_SIMD_X86_64_V4;
    if(v=="prescott")return VC1_SIMD_PRESCOTT;
    if(v=="conroe")return VC1_SIMD_CONROE;
    if(v=="penryn")return VC1_SIMD_PENRYN;
    if(v=="sandybridge"||v=="sandy-bridge")return VC1_SIMD_SANDYBRIDGE;
    if(v=="k10"||v=="sse4a")return VC1_SIMD_K10;
    if(v=="bulldozer")return VC1_SIMD_BULLDOZER;
    if(v=="piledriver")return VC1_SIMD_PILEDRIVER;
    if(v=="avx2-partial"||v=="avx2_partial")return VC1_SIMD_AVX2_PARTIAL;
    throw std::runtime_error(std::string(opt)+" must use auto, none (scalar alias), x86-64-v1, x86-64-v2, x86-64-v3, x86-64-v4, or a named x86 target");
}

static int find_simd_primitive(std::string name) {
    name=trim_ascii(name);
    for(char& c:name){c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));if(c=='_')c='-';}
    for(int i=0;i<VC1_SIMD_PRIMITIVE_COUNT;++i)
        if(name==vc1_simd_primitive_name(static_cast<vc1_simd_primitive_e>(i)))return i;
    return -1;
}

static void apply_simd_primitive_overrides(vc1_param_t& param,const std::string& spec) {
    size_t pos=0;
    while(pos<=spec.size()) {
        const size_t comma=spec.find(',',pos);
        std::string item=trim_ascii(spec.substr(pos,comma==std::string::npos?std::string::npos:comma-pos));
        if(item.empty()) throw std::runtime_error("--simd-primitive contains an empty assignment");
        const size_t eq=item.find('=');
        if(eq==std::string::npos || item.find('=',eq+1)!=std::string::npos)
            throw std::runtime_error("--simd-primitive entries must be NAME=TIER");
        const std::string name=trim_ascii(item.substr(0,eq));
        const int pi=find_simd_primitive(name);
        if(pi<0) {
            std::string valid;
            for(int i=0;i<VC1_SIMD_PRIMITIVE_COUNT;++i){if(i)valid+=',';valid+=vc1_simd_primitive_name(static_cast<vc1_simd_primitive_e>(i));}
            throw std::runtime_error("unknown SIMD primitive '"+name+"'; valid names: "+valid);
        }
        param.i_simd_primitive[pi]=parse_simd_tier(item.substr(eq+1),"--simd-primitive");
        if(comma==std::string::npos)break;
        pos=comma+1;
    }
}

static void usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " [-i input.y4m|-] -o output.vc1|output.m2ts|output.wmv|output.mkv [--format auto|raw|m2ts|wmv|wvc1|mkv] [--codec auto|wvc1|wmv3] [--es-out out.vc1] [--recon-out out.yuv] [--debug-transforms out.y4m] [--rc-stats file.csv] [--debug-stats file.csv] [--macroblock-stats file.csv] [--speed-profile report.txt] [--input-fps RATE] [--max-frames N] [--threads N] [--intra-gop-parallelism] [--compute cpu|vulkan] [--compute-device N] [--vulkan-min-batch N] [--vulkan-force] [--simd auto|none|x86-64-v1|prescott|k10|conroe|penryn|x86-64-v2|sandybridge|bulldozer|piledriver|avx2-partial|x86-64-v3|x86-64-v4] [--benchmark-all|--benchmark-selective] [--simd-primitive NAME=TIER[,NAME=TIER...]] [--simd-fma]\n"
              << "       [--bluray-compat] [--bitrate TARGET] [--buffer-size BITS] [--rc-maximize] [--i-block-weight X] [--p-block-weight X] [--b-block-weight X] [--residual-priority-threshold X] [--residual-priority-width X] [--residual-priority-strength X] [--inter-intra-threshold X] | [--cq Q] [--quantizer-type auto|uniform|nonuniform] [--no-halfqp]\n"
              << "       [--fast|--faster|--fastest] [--keyint N] [--bframes 0|1|2] [--search-range N] [--local-search-range N] [--me-quality sad|rate|satd|rd] [--long-range-search compare|local-good|legacy] [--distant-match-max-error MAE] [--scene-cut|--no-scene-cut] [--scene-threshold X] [--scene-cut-interval SECONDS] [--fixed-gop-grid] [--trellis 0|1|2] [--aq-strength X] [--no-aq] [--no-scene-cut] [--no-fade-comp] [--overlap|--no-overlap] [--no-loop-filter] [--dquant|--no-dquant] [--fixed-8x8] [--skip-identical-frames|--no-skip-identical-frames] [--debug-disable-p-intra] [--debug-disable-b-intra] [--debug-disable-pb-intra] [--intra-only]\n"
              << "       [--dc-only] [--ac-mode auto|vlc|esc3] [--ac-y-table 0|1|2] [--ac-c-table 0|1|2]\n"
              << "       cat input.y4m | " << argv0 << " -o output.m2ts\n"
              << "vc1enc is the libvc1 command-line frontend. Encoding is performed through libvc1; Y4M parsing and M2TS/ASF/Matroska muxing remain in this frontend.\n"
              << "Output defaults from the extension: .vc1 selects a raw VC-1 Advanced Profile elementary stream, .mkv selects Matroska with VC-1 Advanced Profile/WVC1, .wmv selects WMV9/WMV3 Main Profile in ASF, and other extensions select VC-1 Advanced Profile in M2TS.\n"
              << "For .mkv/--format mkv, --codec wmv3 (alias wmv9) selects WMV9 Main Profile instead; --codec wvc1 (alias vc1) selects the default Advanced Profile. MKV output requires libmatroska/libebml at build time.\n"
              << "Use --format raw (aliases: vc1, elementary) for a containerless Advanced Profile elementary stream, or --format wvc1 for Advanced Profile/WVC1 in ASF. Y4M Ip, It (top-field-first), and Ib (bottom-field-first) inputs are accepted; interlaced input requires Advanced Profile. --input-fps RATE (alias --fps) overrides a bad Y4M F tag; use NUM/DEN or NUM:DEN for exact rates such as 24000/1001. --bluray-compat applies only Blu-ray VC-1 codec constraints (profile/level, legal picture formats/frame rates, 40 Mbit/s video ceiling, 30 Mbit VBV, and roughly one-second keyframe spacing); it never selects or restricts the output container.\n"
              << "Rate control has two modes: the default one-pass ABR+VBV model and unbounded constant-Q (--cq). The ABR defaults remain 38 Mbit/s and 30 Mbit VBV; --rc-maximize biases it toward fuller bitrate use when headroom is available. ABR distributes each GOP budget by macroblock coding class with relative I/P/B weights; defaults are I=5.0, P=1.0, B=0.70. Intra macroblocks use the I weight even when they occur inside P or B pictures; temporal P/B macroblocks use the P/B weights. --i-block-weight, --p-block-weight, and --b-block-weight adjust them independently; the older --i-frame-weight/--p-frame-weight/--b-frame-weight names remain aliases. The weights are normalized over the analyzed GOP block mix, so multiplying all three by the same factor does not change the requested average bitrate. These controls do not apply to --cq. Poorly predicted temporal blocks use a smooth luma-MAE protection ramp: --residual-priority-threshold sets its lower edge (default 8), --residual-priority-width its transition width (default 24), and --residual-priority-strength the maximum finer local MQUANT shift (default 0/off; set a positive value to enable it). --inter-intra-threshold controls the P/B spatial-vs-temporal proxy ratio (default 0.80; higher selects intra more readily). --rc-stats writes lightweight rate-control diagnostics; --debug-stats writes extended per-frame coding decisions plus reconstruction quality (MSE/SNR/PSNR) and enables reconstruction only for the debug run. --macroblock-stats is sectioned by one config metadata block, one frame-summary row per frame, then only block-varying rows; frame/config values are not repeated per macroblock. It includes actual local bits, coding/prediction mode, reference frame/MVs, quantizer/transform state, source brightness/activity, temporal and motion-prediction error, residual-priority ramp/boost, inter-vs-intra decision ratio, dark-detail/color AQ scores, and final reconstruction quality.\n"
              << "--speed-profile FILE enables aggregate encoder timing and writes one compact ranked summary after the encode. It records only fixed counters (calls plus accumulated time) for major operations/features such as P/B analysis, motion search/refinement, transform RDO, trellis, AQ, DQUANT, entropy, overlap, and loop filtering; it never writes per-frame or per-macroblock timing records. Profiling adds timer overhead and feature timers may overlap, so use it to identify large contributors rather than benchmark tiny differences.\n"
              << "Threading uses independent GOP workers by default. --intra-gop-parallelism additionally enables bounded scene/B-picture helper work inside each GOP; --no-intra-gop-parallelism disables it explicitly.\n"
              << "Threshold scene detection is on by default. --scene-cut-interval sets the minimum source-time spacing between scene-cut I pictures (default 0.25 second, independent of --keyint; 0 disables the cooldown), while --no-scene-cut disables threshold detection. Generic VC-1 uses a 120-frame automatic keyframe interval; --keyint may set any positive interval permitted by the selected VC-1 level. --bluray-compat instead defaults to and enforces roughly one second. A separate adaptive I safety net is inserted if hybrid motion search finds no usable macroblock match; a threshold cut suppressed by the cooldown cannot re-enter through that safety net on the same boundary. --fixed-gop-grid suppresses unscheduled adaptive I pictures.\n"
              << "Advanced Profile exact duplicate input frames use normative PTYPE=Skipped pictures by default, preserving the nominal frame cadence while repeating the previous reconstructed reference. --no-skip-identical-frames disables this optimization.\n"
              << "--debug-disable-p-intra suppresses whole-macroblock intra selection in P pictures only.\n"
              << "--debug-disable-b-intra suppresses intra macroblock selection in B pictures (and therefore BI promotion) only.\n"
              << "--debug-disable-pb-intra suppresses both P- and B-picture intra macroblock selection. These are diagnostic isolation switches; motion search/refinement, transforms, DQUANT, overlap, deblocking, entropy coding, and rate control are otherwise unchanged.\n"
              << "Long-range motion search defaults to --long-range-search compare. Distant candidates with luma MAE <=6 are accepted as strong matches; otherwise they must improve local SAD by at least 25% and remain at or below --distant-match-max-error (default 255 MAE, effectively no absolute ceiling). This confidence gate applies to propagated and content-signature distant vectors, preventing merely-less-bad cross-scene matches; --long-range-search legacy retains the historical distant-first short-circuit. --long-range-search local-good skips distant work after a strong local match.\n"
              << "Motion estimation is staged: --me-quality sad uses SAD only; rate adds coded MV cost; satd (the default) re-ranks the SAD shortlist with Hadamard SATD plus MV rate; rd runs the slower codec-aware transform/quantizer RD pass on the best SATD finalists, including chroma. Speed presets select satd/rate/sad respectively for --fast/--faster/--fastest. Later options override preset settings.\n"
              << "Vulkan compute is experimental and never activates by default; the default is --compute cpu (legacy --compute auto is also CPU-only). --compute vulkan explicitly opts in, benchmarks Vulkan against the selected CPU/SIMD motion-cost path, and uses Vulkan only for batch sizes where it wins. Add --vulkan-force to that opt-in to bypass benchmark rejection and force eligible Vulkan work. --compute-device selects the Vulkan physical-device ordinal, and --vulkan-min-batch sets the user floor below which GPU dispatch is never attempted. WMV3/Main remains CPU-only unless a future release adds support.\n"
              << "SIMD defaults to auto and benchmarks every built target compatible with the current CPU for each accelerated primitive independently. --benchmark-selective restores the older gap-only policy for intermediate Prescott/K10/Conroe/Penryn/Sandy Bridge/Bulldozer/Piledriver/AVX2-partial targets; --benchmark-all explicitly selects the default all-compatible policy. Neither mode bypasses required feature flags (including FMA4 for Bulldozer/Piledriver). Detection uses CPU feature flags only, never vendor/name/model. scalar remains an alias for none. --simd-primitive NAME=TIER overrides individual primitives, may be repeated, and accepts comma-separated assignments; later assignments win.\n";
}

static void blend_overlay_rect(std::vector<uint8_t>& plane,int w,int h,int x0,int y0,int rw,int rh,int target) {
    if (rw<=0 || rh<=0) return;
    const int x1=std::min(w,x0+rw), y1=std::min(h,y0+rh);
    x0=std::max(0,x0);
    y0=std::max(0,y0);
    for(int y=y0;y<y1;++y) for(int x=x0;x<x1;++x){ const size_t off=static_cast<size_t>(y)*w+x; const bool edge=(x==x0||x==x1-1||y==y0||y==y1-1); const int old=plane[off]; const int v=edge?(old*2+target*3+2)/5:(old*5+target+3)/6; plane[off]=static_cast<uint8_t>(std::clamp(v,0,255)); }
}
static void overlay_transform_rect(Frame& frame,int x,int y,int w,int h,int type) {
    static constexpr int colors[5][3]={{128,128,128},{190,165,82},{165,82,96},{185,92,190},{150,190,175}}; type=std::clamp(type,1,4);
    blend_overlay_rect(frame.y,frame.width,frame.height,x,y,w,h,colors[type][0]);
    blend_overlay_rect(frame.u,(frame.width+1)/2,(frame.height+1)/2,x/2,y/2,(w+1)/2,(h+1)/2,colors[type][1]);
    blend_overlay_rect(frame.v,(frame.width+1)/2,(frame.height+1)/2,x/2,y/2,(w+1)/2,(h+1)/2,colors[type][2]);
}
static Frame make_transform_overlay(const Frame& recon,const std::vector<uint8_t>& map,int type,int width,int height) {
    Frame out=recon; const int mbw=(width+15)/16,mbh=(height+15)/16;
    if(type==VC1_TYPE_I){ for(int y=0;y<height;y+=8) for(int x=0;x<width;x+=8) overlay_transform_rect(out,x,y,std::min(8,width-x),std::min(8,height-y),1); return out; }
    if(map.size()!=static_cast<size_t>(mbw)*mbh*4) throw std::runtime_error("internal transform debug map size mismatch");
    for(int my=0;my<mbh;++my) for(int mx=0;mx<mbw;++mx){ const size_t mbpos=static_cast<size_t>(my)*mbw+mx; for(int k=0;k<4;++k){ const uint8_t packed=map[mbpos*4+static_cast<size_t>(k)]; if(!packed) continue; const int tt=packed&7; const uint8_t skipmask=packed>>3; int np=1;if(tt==2||tt==3)np=2;else if(tt==4)np=4; const int bx=mx*16+(k&1)*8,by=my*16+((k>>1)&1)*8; for(int part=0;part<np;++part){ if(skipmask&(1u<<(np-1-part)))continue;int ox=0,oy=0,bw=8,bh=8;if(tt==2){bh=4;oy=part*4;}else if(tt==3){bw=4;ox=part*4;}else if(tt==4){bw=4;bh=4;ox=(part&1)*4;oy=(part>>1)*4;}const int rw=std::min(bw,width-(bx+ox)),rh=std::min(bh,height-(by+oy));if(rw>0&&rh>0)overlay_transform_rect(out,bx+ox,by+oy,rw,rh,tt);}}}
    return out;
}
static void write_frame_raw(std::ostream& out,const Frame& f) {
    out.write(reinterpret_cast<const char*>(f.y.data()),static_cast<std::streamsize>(f.y.size())); out.write(reinterpret_cast<const char*>(f.u.data()),static_cast<std::streamsize>(f.u.size())); out.write(reinterpret_cast<const char*>(f.v.data()),static_cast<std::streamsize>(f.v.size())); if(!out) throw std::runtime_error("write failed on reconstruction output");
}
static void write_y4m_frame(std::ostream& out,const Frame& f) { out<<"FRAME\n"; write_frame_raw(out,f); }

struct PlaneQuality { double mse=0.0,snr_db=0.0,psnr_db=0.0; long double noise=0.0L,signal=0.0L; uint64_t samples=0; };
static PlaneQuality plane_quality(const std::vector<uint8_t>& src,const std::vector<uint8_t>& rec) {
    if (src.size()!=rec.size() || src.empty()) throw std::runtime_error("debug statistics reconstruction size mismatch");
    PlaneQuality q; q.samples=src.size();
    for (size_t i=0;i<src.size();++i) {
        const long double s=src[i],d=s-static_cast<long double>(rec[i]);
        q.signal+=s*s; q.noise+=d*d;
    }
    q.mse=static_cast<double>(q.noise/static_cast<long double>(q.samples));
    q.psnr_db=q.noise==0.0L?std::numeric_limits<double>::infinity():10.0*std::log10(65025.0/q.mse);
    q.snr_db=q.noise==0.0L?std::numeric_limits<double>::infinity():
        (q.signal==0.0L?-std::numeric_limits<double>::infinity():10.0*std::log10(static_cast<double>(q.signal/q.noise)));
    return q;
}
static PlaneQuality combined_quality(const PlaneQuality& y,const PlaneQuality& u,const PlaneQuality& v) {
    PlaneQuality q; q.noise=y.noise+u.noise+v.noise; q.signal=y.signal+u.signal+v.signal; q.samples=y.samples+u.samples+v.samples;
    q.mse=static_cast<double>(q.noise/static_cast<long double>(q.samples));
    q.psnr_db=q.noise==0.0L?std::numeric_limits<double>::infinity():10.0*std::log10(65025.0/q.mse);
    q.snr_db=q.noise==0.0L?std::numeric_limits<double>::infinity():
        (q.signal==0.0L?-std::numeric_limits<double>::infinity():10.0*std::log10(static_cast<double>(q.signal/q.noise)));
    return q;
}

static void write_speed_profile_report(const std::string& path,const vc1_param_t& p,
                                       const vc1_stats_t& st,uint64_t frames,const std::string& preset) {
    struct Row { int op=0; uint64_t calls=0,ns=0; };
    std::vector<Row> rows;
    rows.reserve(VC1_SPEED_PROFILE_OPERATION_COUNT);
    for(int i=0;i<VC1_SPEED_PROFILE_OPERATION_COUNT;++i) {
        const uint64_t calls=st.i_speed_profile_calls[i],ns=st.i_speed_profile_nanoseconds[i];
        if(calls || ns) rows.push_back({i,calls,ns});
    }
    std::sort(rows.begin(),rows.end(),[](const Row& a,const Row& b){
        if(a.ns!=b.ns) return a.ns>b.ns;
        return a.op<b.op;
    });
    const uint64_t base_ns=st.i_speed_profile_nanoseconds[VC1_SPEED_GOP_ENCODE];
    std::ofstream out(path,std::ios::trunc);
    if(!out) throw std::runtime_error("cannot open speed profile output: "+path);
    out<<"libvc1 aggregate speed profile v1\n";
    out<<"version="<<vc1_version_str()<<"\n";
    out<<"input_frames="<<frames<<"\n";
    out<<"gop_encode_attempts="<<st.i_speed_profile_calls[VC1_SPEED_GOP_ENCODE]<<"\n";
    out<<"resolution="<<p.i_width<<"x"<<p.i_height<<"\n";
    out<<"fps="<<p.i_fps_num<<"/"<<p.i_fps_den<<"\n";
    out<<"threads="<<p.i_threads<<"\n";
    out<<"preset="<<preset<<"\n";
    out<<"settings: bframes="<<(p.b_intra_only?0:p.i_bframes)
       <<" search="<<p.i_motion_search_range<<" local_search="<<p.i_motion_local_search_range
       <<" me="<<(p.i_me_quality==VC1_ME_RD?"rd":(p.i_me_quality==VC1_ME_SATD?"satd":(p.i_me_quality==VC1_ME_RATE?"rate":"sad")))
       <<" trellis="<<p.i_trellis<<" aq="<<(p.b_adaptive_quality?"on":"off")
       <<" dquant="<<(p.b_dquant?"on":"off")
       <<" variable_transforms="<<(p.b_variable_transforms?"on":"off")
       <<" overlap="<<(p.b_overlap?"on":"off")
       <<" loop_filter="<<(p.b_loop_filter?"on":"off")
       <<" fade_comp="<<(p.b_fade_compensation?"on":"off")
       <<" scene_cut="<<(p.b_scene_cut?"on":"off")<<"\n";
    out<<"\nTiming semantics:\n";
    out<<"  Times are aggregate instrumented work across all encoder worker threads and retry attempts.\n";
    out<<"  Feature/subsystem timers may overlap each other; with parallel work their percentages may exceed 100%.\n";
    out<<"  Profiling itself adds timer overhead, so use this report to find large contributors rather than benchmark tiny differences.\n";
    out<<"  No per-frame or per-macroblock timing samples are stored.\n\n";
    out<<"Rank  Operation                         Calls       Total ms      Mean us/call   ms/input-frame   % GOP work\n";
    out<<"----  --------------------------------  ----------  ------------  -------------  ---------------  ----------\n";
    int rank=1;
    out<<std::fixed<<std::setprecision(3);
    for(const Row& r:rows) {
        if(r.op==VC1_SPEED_GOP_ENCODE) continue;
        const double ms=static_cast<double>(r.ns)/1.0e6;
        const double mean_us=r.calls?static_cast<double>(r.ns)/1000.0/static_cast<double>(r.calls):0.0;
        const double ms_frame=frames?ms/static_cast<double>(frames):0.0;
        const double pct=base_ns?100.0*static_cast<double>(r.ns)/static_cast<double>(base_ns):0.0;
        out<<std::setw(4)<<rank++<<"  "<<std::left<<std::setw(32)
           <<vc1_speed_profile_operation_name(static_cast<vc1_speed_profile_operation_e>(r.op))
           <<std::right<<"  "<<std::setw(10)<<r.calls<<"  "<<std::setw(12)<<ms
           <<"  "<<std::setw(13)<<mean_us<<"  "<<std::setw(15)<<ms_frame
           <<"  "<<std::setw(9)<<pct<<"\n";
    }
    const double base_ms=static_cast<double>(base_ns)/1.0e6;
    out<<"\nGOP encode work total: "<<base_ms<<" ms across "
       <<st.i_speed_profile_calls[VC1_SPEED_GOP_ENCODE]<<" encode attempt(s)";
    if(frames) out<<" ("<<(base_ms/static_cast<double>(frames))<<" ms/input-frame)";
    out<<".\n";
    if(!out) throw std::runtime_error("write failed on speed profile output: "+path);
}

struct ReconRecord { Frame frame; std::vector<uint8_t> transform_map; int type=VC1_TYPE_AUTO; bool skipped_picture=false; };

} // namespace

int main(int argc,char** argv) {
    try {
        std::string inpath="-",outpath,espath,reconpath,transformpath,rcstatspath,debugstatspath,macroblockstatspath,speedprofilepath,format="auto",codec="auto";
        bool input_fps_override_set=false; Rational input_fps_override{};
        int64_t maxframes=std::numeric_limits<int64_t>::max();
        vc1_param_t param; vc1_param_default(&param);
        std::string speed_preset="normal";
        bool bitrate_set=false,buffer_set=false,cq_set=false,rc_maximize_set=false,rc_weights_set=false,codec_set=false;
        bool y_table_set=false,c_table_set=false,keyint_set=false;
        auto apply_speed_preset=[&](int level) {
            param.b_fade_compensation=0;
            param.i_trellis=0;
            param.b_adaptive_quality=0;
            param.b_dquant=0;
            param.i_ac_mode=VC1_AC_VLC;
            param.i_ac_y_table=1;
            param.i_ac_c_table=1;
            y_table_set=false;
            c_table_set=false;
            if (level==1) {
                speed_preset="fast";
                param.i_motion_search_range=8;
                param.i_motion_local_search_range=8;
                param.i_me_quality=VC1_ME_SATD;
                param.b_scene_cut=0;
                param.b_loop_filter=1;
                param.b_overlap=1;
                param.b_variable_transforms=1;
            } else if (level==2) {
                speed_preset="faster";
                param.i_motion_search_range=4;
                param.i_motion_local_search_range=4;
                param.i_me_quality=VC1_ME_RATE;
                param.b_scene_cut=0;
                param.b_loop_filter=1;
                param.b_overlap=1;
                param.b_variable_transforms=0;
            } else {
                speed_preset="fastest";
                param.i_motion_search_range=0;
                param.i_motion_local_search_range=0;
                param.i_me_quality=VC1_ME_SAD;
                param.b_scene_cut=0;
                param.b_loop_filter=0;
                param.b_overlap=0;
                param.b_variable_transforms=0;
            }
        };
        for(int i=1;i<argc;++i){ std::string a=argv[i]; auto need=[&](const char* o){if(++i>=argc)throw std::runtime_error(std::string("missing value for ")+o);return std::string(argv[i]);};
            if(a=="-i")inpath=need("-i"); else if(a=="-o")outpath=need("-o"); else if(a=="--format"||a=="--container")format=need(a.c_str());
            else if(a=="--codec"){codec=need("--codec");codec_set=true;}
            else if(a=="--es-out")espath=need("--es-out"); else if(a=="--recon-out")reconpath=need("--recon-out"); else if(a=="--debug-transforms"||a=="--transform-overlay")transformpath=need(a.c_str()); else if(a=="--rc-stats")rcstatspath=need("--rc-stats"); else if(a=="--debug-stats")debugstatspath=need("--debug-stats"); else if(a=="--macroblock-stats"||a=="--mb-stats")macroblockstatspath=need(a.c_str()); else if(a=="--speed-profile"||a=="--profile-speed"){speedprofilepath=need(a.c_str());param.b_speed_profile=1;}
            else if(a=="--input-fps"||a=="--fps"){input_fps_override=parse_fps_value(need(a.c_str()),a.c_str());input_fps_override_set=true;}
            else if(a=="--max-frames")maxframes=std::stoll(need("--max-frames")); else if(a=="--threads")param.i_threads=std::stoi(need("--threads")); else if(a=="--no-intra-gop-parallelism")param.b_intra_gop_parallelism=0; else if(a=="--intra-gop-parallelism")param.b_intra_gop_parallelism=1;
            else if(a=="--fast")apply_speed_preset(1); else if(a=="--faster")apply_speed_preset(2); else if(a=="--fastest")apply_speed_preset(3);
            else if(a=="--compute"){auto v=need("--compute");if(v=="auto")param.i_compute_backend=VC1_COMPUTE_AUTO;else if(v=="cpu"||v=="none")param.i_compute_backend=VC1_COMPUTE_CPU;else if(v=="vulkan"||v=="vk")param.i_compute_backend=VC1_COMPUTE_VULKAN;else throw std::runtime_error("--compute must be auto, cpu, or vulkan");}
            else if(a=="--compute-device"||a=="--vulkan-device")param.i_compute_device=std::stoi(need(a.c_str()));
            else if(a=="--vulkan-min-batch")param.i_vulkan_min_batch=std::stoi(need("--vulkan-min-batch"));
            else if(a=="--vulkan-force"||a=="--force-vulkan")param.b_vulkan_force=1;
            else if(a=="--simd")param.i_simd=parse_simd_tier(need("--simd"),"--simd");
            else if(a=="--benchmark-all"||a=="--simd-benchmark-all")param.b_simd_benchmark_all=1;
            else if(a=="--benchmark-selective"||a=="--simd-benchmark-selective")param.b_simd_benchmark_all=0;
            else if(a=="--simd-primitive"||a=="--simd-primitives")apply_simd_primitive_overrides(param,need(a.c_str()));
            else if(a=="--no-simd")param.i_simd=VC1_SIMD_NONE; else if(a=="--simd-fma"||a=="--fma3")param.b_simd_fma=1; else if(a=="--no-simd-fma")param.b_simd_fma=0;
            else if(a=="--bframes")param.i_bframes=std::stoi(need("--bframes"));
            else if(a=="--bluray-compat")param.b_bluray_compat=1; else if(a=="--no-bluray-compat")param.b_bluray_compat=0;
            else if(a=="--bitrate"){param.i_bitrate=parse_bitrate_value(need("--bitrate"),"--bitrate");bitrate_set=true;} else if(a=="--buffer-size"){param.i_vbv_buffer_size=parse_bitrate_value(need("--buffer-size"),"--buffer-size");buffer_set=true;} else if(a=="--rc-maximize"||a=="--maximize-bitrate"||a=="--max-utilization"){param.b_rc_maximize=1;rc_maximize_set=true;} else if(a=="--i-frame-weight"||a=="--i-block-weight"||a=="--rc-i-weight"){param.f_rc_i_weight=std::stod(need(a.c_str()));rc_weights_set=true;} else if(a=="--p-frame-weight"||a=="--p-block-weight"||a=="--rc-p-weight"){param.f_rc_p_weight=std::stod(need(a.c_str()));rc_weights_set=true;} else if(a=="--b-frame-weight"||a=="--b-block-weight"||a=="--rc-b-weight"){param.f_rc_b_weight=std::stod(need(a.c_str()));rc_weights_set=true;} else if(a=="--residual-priority-threshold"||a=="--poor-prediction-threshold"){param.f_rc_residual_threshold=std::stod(need(a.c_str()));} else if(a=="--residual-priority-width"||a=="--poor-prediction-width"){param.f_rc_residual_width=std::stod(need(a.c_str()));} else if(a=="--residual-priority-strength"||a=="--poor-prediction-boost"){param.f_rc_residual_max_q_boost=std::stod(need(a.c_str()));} else if(a=="--inter-intra-threshold"||a=="--intra-block-threshold"){param.f_inter_intra_threshold=std::stod(need(a.c_str()));} else if(a=="--cq"||a=="--quantizer"){double q=std::stod(need(a.c_str())); double iq=std::floor(q); double frac=q-iq; if(q<1.0||q>31.0||!(std::abs(frac)<1e-9||std::abs(frac-0.5)<1e-9)|| (frac>0.25&&iq>8.0)) throw std::runtime_error("--cq must be an integer 1..31 or a half step 1.5..8.5"); param.i_qp_constant=static_cast<int>(iq); param.b_qp_half=frac>0.25?1:0; param.i_rc_method=VC1_RC_CQP;cq_set=true;} else if(a=="--quantizer-type"){std::string q=need("--quantizer-type"); if(q=="auto")param.i_quantizer_type=VC1_QUANTIZER_AUTO; else if(q=="uniform")param.i_quantizer_type=VC1_QUANTIZER_UNIFORM; else if(q=="nonuniform")param.i_quantizer_type=VC1_QUANTIZER_NONUNIFORM; else throw std::runtime_error("--quantizer-type must be auto, uniform, or nonuniform");} else if(a=="--no-halfqp"){param.b_halfqp=0;}
            else if(a=="--keyint"){param.i_keyint_max=std::stoi(need("--keyint"));keyint_set=true;} else if(a=="--search-range")param.i_motion_search_range=std::stoi(need("--search-range")); else if(a=="--local-search-range"||a=="--local-search")param.i_motion_local_search_range=std::stoi(need(a.c_str()));
            else if(a=="--me-quality"||a=="--motion-quality"){auto v=need(a.c_str());if(v=="sad")param.i_me_quality=VC1_ME_SAD;else if(v=="rate"||v=="sad-rate")param.i_me_quality=VC1_ME_RATE;else if(v=="satd")param.i_me_quality=VC1_ME_SATD;else if(v=="rd"||v=="rdo")param.i_me_quality=VC1_ME_RD;else throw std::runtime_error("--me-quality must be sad, rate, satd, or rd");}
            else if(a=="--long-range-search"||a=="--distant-search-mode"){auto v=need(a.c_str());if(v=="compare"||v=="best")param.i_long_range_search_mode=VC1_LONG_RANGE_COMPARE;else if(v=="local-good"||v=="local-first")param.i_long_range_search_mode=VC1_LONG_RANGE_LOCAL_GOOD_SKIP;else if(v=="legacy"||v=="distant-first")param.i_long_range_search_mode=VC1_LONG_RANGE_LEGACY_DISTANT_FIRST;else throw std::runtime_error("--long-range-search must be compare, local-good, or legacy");}
            else if(a=="--distant-match-max-error"||a=="--distant-match-max-mae")param.f_distant_match_max_mae=std::stod(need(a.c_str()));
            else if(a=="--legacy-distant-first")param.i_long_range_search_mode=VC1_LONG_RANGE_LEGACY_DISTANT_FIRST; else if(a=="--local-good-short-circuit")param.i_long_range_search_mode=VC1_LONG_RANGE_LOCAL_GOOD_SKIP; else if(a=="--compare-local-distant")param.i_long_range_search_mode=VC1_LONG_RANGE_COMPARE;
            else if(a=="--scene-cut"||a=="--scene-detect")param.b_scene_cut=1; else if(a=="--scene-threshold")param.f_scene_threshold=std::stod(need("--scene-threshold")); else if(a=="--scene-cut-interval"||a=="--scene-cut-min-interval")param.f_scene_cut_min_interval=std::stod(need(a.c_str())); else if(a=="--fixed-gop-grid"||a=="--fixed-keyframe-grid"||a=="--fixed-keyint-grid")param.b_fixed_gop_grid=1; else if(a=="--reset-gop-on-scene")param.b_fixed_gop_grid=0; else if(a=="--no-scene-cut")param.b_scene_cut=0; else if(a=="--no-fade-comp"||a=="--no-intensity-comp")param.b_fade_compensation=0; else if(a=="--no-loop-filter"||a=="--no-deblock")param.b_loop_filter=0; else if(a=="--overlap")param.b_overlap=1; else if(a=="--no-overlap")param.b_overlap=0;
            else if(a=="--trellis")param.i_trellis=std::stoi(need("--trellis")); else if(a=="--no-trellis")param.i_trellis=0; else if(a=="--aq-strength")param.f_aq_strength=std::stod(need("--aq-strength")); else if(a=="--no-aq"||a=="--no-adaptive-quality")param.b_adaptive_quality=0; else if(a=="--dquant")param.b_dquant=1; else if(a=="--no-dquant")param.b_dquant=0; else if(a=="--fixed-8x8"||a=="--no-variable-transforms")param.b_variable_transforms=0; else if(a=="--skip-identical-frames"||a=="--skip-duplicate-frames")param.b_skip_identical_frames=1; else if(a=="--no-skip-identical-frames"||a=="--no-skip-duplicate-frames")param.b_skip_identical_frames=0; else if(a=="--debug-disable-p-intra"||a=="--debug-no-p-intra")param.b_debug_disable_p_intra=1; else if(a=="--debug-disable-b-intra"||a=="--debug-no-b-intra")param.b_debug_disable_b_intra=1; else if(a=="--debug-disable-pb-intra"||a=="--debug-no-pb-intra"||a=="--debug-disable-inter-picture-intra"){param.b_debug_disable_p_intra=1;param.b_debug_disable_b_intra=1;} else if(a=="--intra-only")param.b_intra_only=1; else if(a=="--dc-only")param.b_ac_coding=0;
            else if(a=="--ac-mode"){auto v=need("--ac-mode");if(v=="auto")param.i_ac_mode=VC1_AC_AUTO;else if(v=="vlc")param.i_ac_mode=VC1_AC_VLC;else if(v=="esc3")param.i_ac_mode=VC1_AC_ESC3;else throw std::runtime_error("--ac-mode must be auto, vlc, or esc3");}
            else if(a=="--ac-y-table"){param.i_ac_y_table=std::stoi(need("--ac-y-table"));y_table_set=true;} else if(a=="--ac-c-table"){param.i_ac_c_table=std::stoi(need("--ac-c-table"));c_table_set=true;} else if(a=="--ac-esc3-only")param.i_ac_mode=VC1_AC_ESC3;
            else if(a=="-h"||a=="--help"){usage(argv[0]);return 0;} else throw std::runtime_error("unknown option: "+a);
        }
        if(outpath.empty()){usage(argv[0]);return 2;}
        for(char& c:format)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        for(char& c:codec)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if(format!="auto"&&format!="raw"&&format!="vc1"&&format!="elementary"&&format!="m2ts"&&format!="wmv"&&format!="wvc1"&&format!="asf-vc1"&&format!="mkv"&&format!="matroska")
            throw std::runtime_error("--format must be auto, raw, m2ts, wmv, wvc1, or mkv");
        if(codec!="auto"&&codec!="wvc1"&&codec!="vc1"&&codec!="wmv3"&&codec!="wmv9")
            throw std::runtime_error("--codec must be auto, wvc1, or wmv3");
        std::string low=outpath;for(char& c:low)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        const bool ext_vc1=low.size()>=4&&low.compare(low.size()-4,4,".vc1")==0;
        const bool ext_mkv=low.size()>=4&&low.compare(low.size()-4,4,".mkv")==0;
        const bool ext_wmv=low.size()>=4&&low.compare(low.size()-4,4,".wmv")==0;
        const bool raw_output=(format=="raw"||format=="vc1"||format=="elementary")||(format=="auto"&&ext_vc1);
        const bool mkv_output=(format=="mkv"||format=="matroska")||(format=="auto"&&ext_mkv);
        const bool wvc1=(format=="wvc1"||format=="asf-vc1");
        const bool wmv3=(format=="wmv")||(format=="auto"&&ext_wmv&&!ext_mkv&&!ext_vc1);
        const bool asf_output=wmv3||wvc1;
        if(codec_set&&!mkv_output) throw std::runtime_error("--codec is currently meaningful only with Matroska output");
        const bool mkv_wmv3=mkv_output&&(codec=="wmv3"||codec=="wmv9");
        const bool mkv_wvc1=mkv_output&&!mkv_wmv3;
        const bool main_profile=wmv3||mkv_wmv3;
        param.i_profile=main_profile?VC1_PROFILE_MAIN:VC1_PROFILE_ADVANCED;
        // vc1_param_default() is Advanced-oriented (38 Mbit/s).  When the
        // frontend selects WMV3/Main implicitly from the container/codec and
        // the caller did not choose a rate, use MP@HL's standard Rmax instead
        // of carrying the Advanced default into an invalid Main stream.
        if (main_profile && !bitrate_set && !cq_set && param.i_rc_method==VC1_RC_ABR)
            param.i_bitrate=20000000ull;
        if (mkv_output&&!libvc1_matroska_available())
            throw std::runtime_error("Matroska output was not built: install libmatroska and libebml development files, then rebuild libvc1");
        if (main_profile && !espath.empty())
            throw std::runtime_error("--es-out is unavailable for WMV9/WMV3 because its sequence header and frame boundaries are container-carried");
        if ((y_table_set || c_table_set) && param.i_ac_mode != VC1_AC_VLC)
            throw std::runtime_error("--ac-y-table/--ac-c-table require --ac-mode vlc");
        if (keyint_set && param.i_keyint_max <= 0)
            throw std::runtime_error("--keyint must be positive");
        if (cq_set && bitrate_set)
            throw std::runtime_error("--cq and --bitrate are mutually exclusive");
        if (cq_set && buffer_set)
            throw std::runtime_error("--buffer-size applies only to bitrate-controlled mode");
        if (cq_set && rc_maximize_set)
            throw std::runtime_error("--rc-maximize applies only to bitrate-controlled mode, not --cq");
        if (cq_set && rc_weights_set)
            throw std::runtime_error("I/P/B block weights apply only to bitrate-controlled mode, not --cq");

        std::ifstream inf;std::istream* in=&std::cin;
#ifdef _WIN32
        if(inpath=="-")_setmode(_fileno(stdin),_O_BINARY);
#endif
        if(inpath!="-"){inf.open(inpath,std::ios::binary);if(!inf)throw std::runtime_error("cannot open input");in=&inf;}
        Y4mReader y4m(*in);param.i_width=y4m.width();param.i_height=y4m.height();param.i_fps_num=static_cast<int>(y4m.fps().num);param.i_fps_den=static_cast<int>(y4m.fps().den);if(input_fps_override_set){param.i_fps_num=static_cast<int>(input_fps_override.num);param.i_fps_den=static_cast<int>(input_fps_override.den);}param.i_scan_mode=y4m.scan_mode();param.b_debug_stats=!debugstatspath.empty();param.b_debug_macroblock_stats=!macroblockstatspath.empty();param.b_recon=(!reconpath.empty()||!transformpath.empty()||!debugstatspath.empty()||!macroblockstatspath.empty());param.b_transform_info=!transformpath.empty();
        std::unique_ptr<vc1_t,void(*)(vc1_t*)> enc(vc1_encoder_open(&param),vc1_encoder_close); if(!enc)throw std::runtime_error(std::string("libvc1: ")+vc1_encoder_last_error(nullptr));
        vc1_param_t actual{};vc1_encoder_parameters(enc.get(),&actual); vc1_stats_t initial{};vc1_encoder_stats(enc.get(),&initial);
        bool any_simd_override=false;for(int pi=0;pi<VC1_SIMD_PRIMITIVE_COUNT;++pi)if(param.i_simd_primitive[pi]!=VC1_SIMD_AUTO){any_simd_override=true;break;}
        if(param.i_simd==VC1_SIMD_AUTO){
            constexpr int kModeW=10, kPrimW=24, kRateW=16, kPctMinW=10, kSelectedW=18, kSelectedPctW=12;
            struct Column { int slot; const char* label; vc1_simd_e tier; bool generic; };
            static constexpr Column all_columns[] = {
                {VC1_SIMD_BENCH_X86_64_V1,"X86-64-V1",VC1_SIMD_X86_64_V1,true},
                {VC1_SIMD_BENCH_PRESCOTT,"PRESCOTT",VC1_SIMD_PRESCOTT,false},
                {VC1_SIMD_BENCH_K10,"K10",VC1_SIMD_K10,false},
                {VC1_SIMD_BENCH_CONROE,"CONROE",VC1_SIMD_CONROE,false},
                {VC1_SIMD_BENCH_PENRYN,"PENRYN",VC1_SIMD_PENRYN,false},
                {VC1_SIMD_BENCH_X86_64_V2,"X86-64-V2",VC1_SIMD_X86_64_V2,true},
                {VC1_SIMD_BENCH_SANDYBRIDGE,"SANDYBRIDGE",VC1_SIMD_SANDYBRIDGE,false},
                {VC1_SIMD_BENCH_BULLDOZER,"BULLDOZER",VC1_SIMD_BULLDOZER,false},
                {VC1_SIMD_BENCH_PILEDRIVER,"PILEDRIVER",VC1_SIMD_PILEDRIVER,false},
                {VC1_SIMD_BENCH_AVX2_PARTIAL,"AVX2-PARTIAL",VC1_SIMD_AVX2_PARTIAL,false},
                {VC1_SIMD_BENCH_X86_64_V3,"X86-64-V3",VC1_SIMD_X86_64_V3,true},
                {VC1_SIMD_BENCH_X86_64_V4,"X86-64-V4",VC1_SIMD_X86_64_V4,true},
            };
            auto slot_has_rate=[&](int slot){ for(int pi=0;pi<VC1_SIMD_PRIMITIVE_COUNT;++pi) if(initial.f_simd_primitive_units_per_second[pi][slot]>0.0)return true; return false; };
            std::vector<const Column*> columns;
            for(const auto& c:all_columns) if(c.generic || slot_has_rate(c.slot)) columns.push_back(&c);
            auto pct_width=[&](const char* label){return std::max(kPctMinW,static_cast<int>(std::strlen(label))+1);};
            auto print_rate=[](double r){
                if(r>0.0)std::cerr<<std::right<<std::setw(kRateW)<<std::fixed<<std::setprecision(2)<<r;
                else std::cerr<<std::right<<std::setw(kRateW)<<"n/a";
            };
            auto print_pct=[](double r,double rn,int width){
                if(r>0.0 && rn>0.0){
                    std::ostringstream os; os<<std::fixed<<std::setprecision(1)<<(100.0*r/rn)<<'%';
                    std::cerr<<std::right<<std::setw(width)<<os.str();
                } else std::cerr<<std::right<<std::setw(width)<<"n/a";
            };
            auto tier_slot=[](vc1_simd_e t)->int{
                switch(t){
                  case VC1_SIMD_NONE: return VC1_SIMD_BENCH_NONE;
                  case VC1_SIMD_X86_64_V1: return VC1_SIMD_BENCH_X86_64_V1;
                  case VC1_SIMD_PRESCOTT: return VC1_SIMD_BENCH_PRESCOTT;
                  case VC1_SIMD_K10: return VC1_SIMD_BENCH_K10;
                  case VC1_SIMD_CONROE: return VC1_SIMD_BENCH_CONROE;
                  case VC1_SIMD_PENRYN: return VC1_SIMD_BENCH_PENRYN;
                  case VC1_SIMD_X86_64_V2: return VC1_SIMD_BENCH_X86_64_V2;
                  case VC1_SIMD_SANDYBRIDGE: return VC1_SIMD_BENCH_SANDYBRIDGE;
                  case VC1_SIMD_BULLDOZER: return VC1_SIMD_BENCH_BULLDOZER;
                  case VC1_SIMD_PILEDRIVER: return VC1_SIMD_BENCH_PILEDRIVER;
                  case VC1_SIMD_AVX2_PARTIAL: return VC1_SIMD_BENCH_AVX2_PARTIAL;
                  case VC1_SIMD_X86_64_V3: return VC1_SIMD_BENCH_X86_64_V3;
                  case VC1_SIMD_X86_64_V4: return VC1_SIMD_BENCH_X86_64_V4;
                  default: return -1;
                }
            };
            std::cerr<<"vc1enc: SIMD auto benchmark per primitive (none=units/s; SIMD columns=% of none):\n";
            std::cerr<<std::left<<std::setw(kModeW)<<"MODE"<<std::setw(kPrimW)<<"PRIMITIVE"<<std::right<<std::setw(kRateW)<<"NONE";
            for(const Column* c:columns) std::cerr<<std::setw(pct_width(c->label))<<c->label;
            std::cerr<<"  "<<std::left<<std::setw(kSelectedW)<<"SELECTED"<<std::right<<std::setw(kSelectedPctW)<<"SELECTED%"<<"\n";
            long double selected_ratio_logsum=0.0L; int selected_ratio_count=0;
            for(int pi=0;pi<VC1_SIMD_PRIMITIVE_COUNT;++pi){
                const double rn=initial.f_simd_primitive_units_per_second[pi][VC1_SIMD_BENCH_NONE];
                std::cerr<<std::left<<std::setw(kModeW)<<(param.i_simd_primitive[pi]==VC1_SIMD_AUTO?"[auto]":"[forced]")
                         <<std::setw(kPrimW)<<vc1_simd_primitive_name(static_cast<vc1_simd_primitive_e>(pi));
                print_rate(rn);
                for(const Column* c:columns){const double r=initial.f_simd_primitive_units_per_second[pi][c->slot];print_pct(r,rn,pct_width(c->label));}
                const vc1_simd_e selected_tier=initial.i_simd_primitive_selected[pi];
                std::string selected=simd_name(selected_tier);
                if(initial.b_simd_primitive_fma_selected[pi])selected+=(selected_tier==VC1_SIMD_BULLDOZER?"+fma4":"+fma3");
                const int ss=tier_slot(selected_tier); const double selected_rate=ss>=0?initial.f_simd_primitive_units_per_second[pi][ss]:0.0;
                std::cerr<<"  "<<std::left<<std::setw(kSelectedW)<<selected; print_pct(selected_rate,rn,kSelectedPctW); std::cerr<<"\n";
                if(selected_rate>0.0 && rn>0.0){selected_ratio_logsum+=std::log(selected_rate/rn);++selected_ratio_count;}
            }
            auto gmean_slot=[&](int slot){long double ls=0.0L;int n=0;for(int pi=0;pi<VC1_SIMD_PRIMITIVE_COUNT;++pi){double r=initial.f_simd_primitive_units_per_second[pi][slot];if(r>0){ls+=std::log(r);++n;}}return n?std::exp(static_cast<double>(ls/n)):0.0;};
            const double gn=gmean_slot(VC1_SIMD_BENCH_NONE);
            std::cerr<<std::left<<std::setw(kModeW)<<"[auto]"<<std::setw(kPrimW)<<"geometric-mean"; print_rate(gn);
            for(const Column* c:columns){double g=gmean_slot(c->slot);print_pct(g,gn,pct_width(c->label));}
            std::cerr<<"  "<<std::left<<std::setw(kSelectedW)<<"-";
            if(selected_ratio_count>0){const double pct=100.0*std::exp(static_cast<double>(selected_ratio_logsum/selected_ratio_count));std::ostringstream os;os<<std::fixed<<std::setprecision(1)<<pct<<'%';std::cerr<<std::right<<std::setw(kSelectedPctW)<<os.str();}
            else std::cerr<<std::right<<std::setw(kSelectedPctW)<<"n/a";
            std::cerr<<std::defaultfloat<<"\n";
        } else if(any_simd_override) {
            constexpr int kModeW=10, kPrimW=24, kSelectedW=18;
            std::cerr<<"vc1enc: SIMD per-primitive dispatch (startup benchmark disabled by forced global mode):\n";
            std::cerr<<std::left<<std::setw(kModeW)<<"MODE"<<std::setw(kPrimW)<<"PRIMITIVE"<<std::setw(kSelectedW)<<"SELECTED"<<"\n";
            for(int pi=0;pi<VC1_SIMD_PRIMITIVE_COUNT;++pi){
                std::string selected=simd_name(initial.i_simd_primitive_selected[pi]);
                if(initial.b_simd_primitive_fma_selected[pi])selected+=(initial.i_simd_primitive_selected[pi]==VC1_SIMD_BULLDOZER?"+fma4":"+fma3");
                std::cerr<<std::left<<std::setw(kModeW)<<(param.i_simd_primitive[pi]==VC1_SIMD_AUTO?"[global]":"[forced]")
                         <<std::setw(kPrimW)<<vc1_simd_primitive_name(static_cast<vc1_simd_primitive_e>(pi))
                         <<std::setw(kSelectedW)<<selected<<"\n";
            }
        }
        const uint64_t configured_keyint=actual.i_keyint_max>0?static_cast<uint64_t>(actual.i_keyint_max):(actual.b_bluray_compat?static_cast<uint64_t>(std::max(1LL,std::llround(static_cast<double>(actual.i_fps_num)/actual.i_fps_den))):120ull);
        const char* output_name=raw_output?"VC-1 Advanced Profile elementary stream":(mkv_wmv3?"WMV9/WMV3 Main Profile MKV":(mkv_wvc1?"VC-1 Advanced Profile WVC1 MKV":(wmv3?"WMV9/WMV3 Main Profile ASF":(wvc1?"VC-1 Advanced Profile WVC1 ASF":"VC-1 Advanced Profile M2TS"))));
        const char* me_name=actual.i_me_quality==VC1_ME_RD?"rd":(actual.i_me_quality==VC1_ME_SATD?"satd":(actual.i_me_quality==VC1_ME_RATE?"rate":"sad"));
        const char* long_range_name=actual.i_long_range_search_mode==VC1_LONG_RANGE_LOCAL_GOOD_SKIP?"local-good":(actual.i_long_range_search_mode==VC1_LONG_RANGE_LEGACY_DISTANT_FIRST?"legacy":"compare");
        std::cerr<<"vc1enc: options: "<<actual.i_width<<"x"<<actual.i_height<<" @ "<<actual.i_fps_num<<"/"<<actual.i_fps_den<<" fps"
                 <<", profile="<<(actual.i_profile==VC1_PROFILE_MAIN?"main":"advanced")
                 <<", scan="<<(actual.i_scan_mode==VC1_SCAN_INTERLACED_TFF?"tff":(actual.i_scan_mode==VC1_SCAN_INTERLACED_BFF?"bff":"progressive"))
                 <<", output="<<output_name<<", preset="<<speed_preset
                 <<", keyint="<<configured_keyint<<", gop-grid="<<(actual.b_fixed_gop_grid?"fixed":"scene-reset")
                 <<", bframes="<<(actual.b_intra_only?0:actual.i_bframes)
                 <<", search="<<actual.i_motion_search_range<<" px (local "<<actual.i_motion_local_search_range<<")"<<", distant-max-mae="<<actual.f_distant_match_max_mae
                 <<", me="<<me_name<<", long-range="<<long_range_name
                 <<", scene-cut="<<(actual.b_scene_cut?"on":"off")<<", scene-threshold="<<actual.f_scene_threshold<<", scene-cut-interval="<<actual.f_scene_cut_min_interval<<"s"
                 <<", fade-comp="<<(actual.b_fade_compensation?"on":"off")
                 <<", variable-transforms="<<(actual.b_variable_transforms?"on":"off")
                 <<", trellis="<<actual.i_trellis<<", aq="<<(actual.b_adaptive_quality?std::to_string(actual.f_aq_strength):std::string("off"))
                 <<", dquant="<<(actual.b_dquant?"on":"off")<<", overlap="<<(actual.b_overlap?"on":"off")
                 <<", loop-filter="<<(actual.b_loop_filter?"on":"off")<<", skip-identical="<<(actual.b_skip_identical_frames?"on":"off")
                 <<", residual-priority="<<actual.f_rc_residual_threshold<<"+"<<actual.f_rc_residual_width<<"/"<<actual.f_rc_residual_max_q_boost
                 <<", inter-intra-threshold="<<actual.f_inter_intra_threshold
                 <<", P-intra-debug="<<(actual.b_debug_disable_p_intra?"disabled":"normal")
                 <<", B-intra-debug="<<(actual.b_debug_disable_b_intra?"disabled":"normal")
                 <<", bluray-compat="<<(actual.b_bluray_compat?"on":"off")
                 <<", speed-profile="<<(actual.b_speed_profile?"on":"off")<<"\n";
        if(actual.i_rc_method==VC1_RC_CQP) {
            std::cerr<<"vc1enc: rate control: rate-mode=CQ, q="<<actual.i_qp_constant<<(actual.b_qp_half?".5":"")
                     <<", halfqp="<<(actual.b_qp_half?"on":"off")
                     <<", quantizer-type="<<(actual.i_quantizer_type==VC1_QUANTIZER_NONUNIFORM?"nonuniform":(actual.i_quantizer_type==VC1_QUANTIZER_UNIFORM?"uniform":"auto"))<<"\n";
        } else {
            std::cerr<<"vc1enc: rate control: rate-mode="<<(actual.b_rc_maximize?"abr-max":"abr")
                     <<", ceiling="<<actual.i_bitrate<<" bps, buffer="<<actual.i_vbv_buffer_size<<" bits"
                     <<", weights=I:"<<actual.f_rc_i_weight<<"/P:"<<actual.f_rc_p_weight<<"/B:"<<actual.f_rc_b_weight<<"\n";
        }
        if(initial.b_compute_benchmark_ran) {
            std::cerr<<"vc1enc: compute benchmark: vulkan["<<initial.i_compute_benchmark_device<<":"
                     <<initial.sz_compute_benchmark_device<<"]";
            if(initial.i_compute_benchmark_sad_min_batch>0)
                std::cerr<<", SAD cpu="<<(initial.f_compute_cpu_sad_candidates_per_second/1.0e6)<<"M/s"
                         <<" vulkan="<<(initial.f_compute_vulkan_sad_candidates_per_second/1.0e6)<<"M/s"
                         <<" crossover="<<initial.i_compute_benchmark_sad_min_batch;
            else
                std::cerr<<", SAD=cpu";
            if(initial.f_compute_cpu_satd_candidates_per_second>0.0 && initial.f_compute_vulkan_satd_candidates_per_second>0.0)
                std::cerr<<", SATD cpu="<<(initial.f_compute_cpu_satd_candidates_per_second/1.0e6)<<"M/s"
                         <<" vulkan="<<(initial.f_compute_vulkan_satd_candidates_per_second/1.0e6)<<"M/s"
                         <<" selected="<<(initial.i_compute_benchmark_satd_min_batch>0?"vulkan":"cpu");
            else
                std::cerr<<", SATD=cpu";
            std::cerr<<"\n";
        }
        if(param.i_compute_backend==VC1_COMPUTE_VULKAN && !param.b_vulkan_force && initial.i_compute_selected!=VC1_COMPUTE_VULKAN) {
            std::cerr<<"vc1enc: experimental Vulkan request not selected; using CPU/SIMD"
                     <<(initial.b_compute_benchmark_ran?" because Vulkan did not win an eligible benchmark":" because Vulkan was unavailable or unsupported")<<"\n";
        }
        std::cerr<<"vc1enc: execution: threads="<<actual.i_threads<<", intra-gop="<<(actual.b_intra_gop_parallelism?"on":"off")
                 <<" (detected "<<std::max(1u,std::thread::hardware_concurrency())<<")"
                 <<", compute="<<compute_name(initial.i_compute_selected);
        if(initial.i_compute_selected==VC1_COMPUTE_VULKAN) {
            std::cerr<<"["<<initial.i_compute_device<<":"<<initial.sz_compute_device<<"]";
            if(initial.b_compute_benchmark_ran)
                std::cerr<<" benchmark-SAD-min="<<(initial.i_compute_benchmark_sad_min_batch>0?std::to_string(initial.i_compute_benchmark_sad_min_batch):std::string("off"))
                         <<" benchmark-SATD-min="<<(initial.i_compute_benchmark_satd_min_batch>0?std::to_string(initial.i_compute_benchmark_satd_min_batch):std::string("off"));
            else if(param.b_vulkan_force)
                std::cerr<<" forced min-batch="<<actual.i_vulkan_min_batch;
        }
        std::cerr<<", simd="<<simd_name(initial.i_simd_selected)
                 <<", fma3="<<((initial.i_simd_selected==VC1_SIMD_PILEDRIVER || initial.i_simd_selected==VC1_SIMD_X86_64_V3 || initial.i_simd_selected==VC1_SIMD_X86_64_V4)?(actual.b_simd_fma?"allowed":"disabled"):"inactive")
                 <<", fma4="<<((initial.i_simd_selected==VC1_SIMD_BULLDOZER)?(actual.b_simd_fma?"allowed":"disabled"):"inactive")<<"\n";

        vc1_au_t* headers=nullptr;int nheaders=0;if(vc1_encoder_headers(enc.get(),&headers,&nheaders)<0||nheaders!=1)throw std::runtime_error("libvc1 failed to return sequence header"); std::vector<uint8_t> sequence(headers->p_payload,headers->p_payload+headers->i_payload);
        std::ofstream outf;
        if(!mkv_output){outf.open(outpath,std::ios::binary);if(!outf)throw std::runtime_error("cannot open output");}
        std::ofstream esf;if(!espath.empty()){esf.open(espath,std::ios::binary);if(!esf)throw std::runtime_error("cannot open ES output");}std::ofstream reconf;if(!reconpath.empty()){reconf.open(reconpath,std::ios::binary);if(!reconf)throw std::runtime_error("cannot open reconstruction output");}std::ofstream transformf;if(!transformpath.empty()){transformf.open(transformpath,std::ios::binary);if(!transformf)throw std::runtime_error("cannot open transform debug output");transformf<<"YUV4MPEG2 W"<<param.i_width<<" H"<<param.i_height<<" F"<<param.i_fps_num<<':'<<param.i_fps_den<<' '<<y4m.scan_tag()<<" A1:1 C420jpeg\n";}
        std::ofstream rcstatsf;if(!rcstatspath.empty()){rcstatsf.open(rcstatspath);if(!rcstatsf)throw std::runtime_error("cannot open rate-control statistics output");rcstatsf<<"display_order,coded_order,type,keyframe,complexity,predicted_q,final_q,predicted_bits,first_actual_bits,final_actual_bits,target_bits,allowed_bits,prediction_error_percent,vbv_before_bits,vbv_after_bits,retries,reencoded\n"<<std::setprecision(12);}
        if(!debugstatspath.empty() && debugstatspath==rcstatspath) throw std::runtime_error("--debug-stats and --rc-stats must use different files");
        std::ofstream debugstatsf;if(!debugstatspath.empty()){debugstatsf.open(debugstatspath);if(!debugstatsf)throw std::runtime_error("cannot open debug statistics output");debugstatsf<<"display_order,coded_order,gop_index,frame_in_gop,gop_frames,type,keyframe,keyframe_reason,skipped_picture,pts,dts,timestamp_seconds,profile,rc_mode,threads,simd,search_range,local_search_range,bframes,trellis,aq_strength,loop_filter,variable_transforms,dquant,ac_mode,macroblocks_total,qp,halfqp,quantizer_type,quant_step,qscale,complexity,motion_residual,predicted_q,predicted_bits,first_actual_bits,final_actual_bits,target_bits,planned_bits,allowed_bits,prediction_error_percent,vbv_before_bits,vbv_after_bits,retries,reencoded,gop_budget_scale,gop_difficulty,encode_trials,moved_mb,fractional_chroma_mb,skipped_mb,explicit_mb,coded_mb,coded_blocks,p_four_mv_mb,p_intra_mb,dquant_mb,mquant_min,mquant_max,mquant_mean,transform_8x8,transform_8x4,transform_4x8,transform_4x4,ttmbf,ttfrm,ttfrm_exact_checked,acpred_mb,intensity_comp,b_forward,b_backward,b_interpolated,b_direct,mean_mv_pixels,max_mv_pixels,mse_y,mse_u,mse_v,mse_yuv,snr_y_db,snr_u_db,snr_v_db,snr_yuv_db,psnr_y_db,psnr_u_db,psnr_v_db,psnr_yuv_db,bits_per_pixel\n"<<std::setprecision(12);}
        if(!macroblockstatspath.empty() && (macroblockstatspath==debugstatspath || macroblockstatspath==rcstatspath)) throw std::runtime_error("--macroblock-stats must use a different file from --debug-stats/--rc-stats");
        std::ofstream macroblockstatsf;if(!macroblockstatspath.empty()){
            macroblockstatsf.open(macroblockstatspath);if(!macroblockstatsf)throw std::runtime_error("cannot open macroblock statistics output");
            const char* profile_name=actual.i_profile==VC1_PROFILE_MAIN?"main":"advanced";
            const char* rc_name=actual.i_rc_method==VC1_RC_CQP?"cqp":(actual.b_rc_maximize?"abr-max":"abr");
            macroblockstatsf<<std::setprecision(12)
                <<"# format=libvc1-macroblock-stats-v2\n"
                <<"# codec_version="<<vc1_version_str()<<"\n"
                <<"# width="<<actual.i_width<<"\n# height="<<actual.i_height<<"\n"
                <<"# y4m_fps="<<y4m.fps().num<<'/'<<y4m.fps().den<<"\n"
                <<"# effective_fps="<<actual.i_fps_num<<'/'<<actual.i_fps_den<<"\n"
                <<"# input_fps_overridden="<<(input_fps_override_set?1:0)<<"\n"
                <<"# profile="<<profile_name<<"\n# rc_mode="<<rc_name<<"\n"
                <<"# bitrate="<<actual.i_bitrate<<"\n# vbv_buffer="<<actual.i_vbv_buffer_size<<"\n"
                <<"# threads="<<actual.i_threads<<"\n# search_range="<<actual.i_motion_search_range<<"\n# local_search_range="<<actual.i_motion_local_search_range<<"\n"
                <<"# bframes="<<(actual.b_intra_only?0:actual.i_bframes)<<"\n# trellis="<<actual.i_trellis<<"\n"<<"# distant_match_max_mae="<<actual.f_distant_match_max_mae<<"\n"
                <<"# scene_cut_enabled="<<actual.b_scene_cut<<"\n# scene_cut_threshold="<<actual.f_scene_threshold<<"\n# scene_cut_min_interval_seconds="<<actual.f_scene_cut_min_interval<<"\n"
                <<"# aq_enabled="<<actual.b_adaptive_quality<<"\n# aq_strength="<<actual.f_aq_strength<<"\n# dquant="<<actual.b_dquant<<"\n"
                <<"# distant_match_max_mae="<<actual.f_distant_match_max_mae<<"\n"
                <<"# distant_match_good_mae=6\n# distant_match_min_improvement_percent=25\n"
                <<"# residual_priority_threshold_mae="<<actual.f_rc_residual_threshold<<"\n"
                <<"# residual_priority_width_mae="<<actual.f_rc_residual_width<<"\n"
                <<"# residual_priority_max_q_boost="<<actual.f_rc_residual_max_q_boost<<"\n"
                <<"# inter_intra_threshold="<<actual.f_inter_intra_threshold<<"\n"
                <<"# rc_i_weight="<<actual.f_rc_i_weight<<"\n# rc_p_weight="<<actual.f_rc_p_weight<<"\n# rc_b_weight="<<actual.f_rc_b_weight<<"\n"
                <<"# rows: each frame row is followed by its macroblock rows; frame/config values are intentionally not repeated in mb rows.\n";
            macroblockstatsf<<"record,display_order,coded_order,frame_type,frame_bits,shared_frame_bits,target_bits,predicted_bits,vbv_before_bits,vbv_after_bits,frame_q,mb_x,mb_y,visible_w,visible_h,mode,field_index,field_parity,opposite_field_ref,skipped,intra,four_mv,direct,acpred,mquant,dquant_delta,cbp,coded_blocks,local_bits,amortized_frame_bits,estimated_transform_bits,tx_8x8,tx_8x4,tx_4x8,tx_4x4,fwd_mv_count,fwd_ref_display,fwd0_xq,fwd0_yq,fwd1_xq,fwd1_yq,fwd2_xq,fwd2_yq,fwd3_xq,fwd3_yq,bwd_mv_count,bwd_ref_display,bwd0_xq,bwd0_yq,bwd1_xq,bwd1_yq,bwd2_xq,bwd2_yq,bwd3_xq,bwd3_yq,mean_y,stddev_y,mean_u,mean_v,activity_y,previous_sad_y,previous_mse_y,prediction_sad_y,prediction_mse_y,prediction_mae_y,prediction_gain_db,residual_priority_position,residual_priority_requested_q_boost,residual_priority_applied_q_boost,inter_intra_cost_ratio,fwd_distant_local_mae_y,fwd_distant_candidate_mae_y,fwd_distant_decision,bwd_distant_local_mae_y,bwd_distant_candidate_mae_y,bwd_distant_decision,aq_dark_detail,aq_color_luma_priority,aq_color_chroma_priority,aq_requested_q_boost,recon_mse_y,recon_mse_u,recon_mse_v,recon_mse_yuv,recon_snr_y_db,recon_psnr_y_db,recon_snr_yuv_db,recon_psnr_yuv_db\n";
        }

        const int coded_width = (param.i_width+1)&~1;
        const int coded_height = (param.i_height+1)&~1;
        std::unique_ptr<M2tsMuxer> m2ts;
        std::unique_ptr<AsfVideoMuxer> asf;
        std::unique_ptr<MatroskaVideoMuxer> mkv;
        if(wmv3) {
            if(sequence.size()!=4) throw std::runtime_error("WMV9 Main Profile sequence header must be exactly 4 bytes");
            asf=std::make_unique<AsfVideoMuxer>(outf,param.i_width,param.i_height,Rational{param.i_fps_num,param.i_fps_den},
                std::array<uint8_t,4>{{'W','M','V','3'}},sequence,param.i_rc_method==VC1_RC_CQP?0:initial.i_hrd_rate_bits);
        } else if(mkv_wmv3) {
            if(sequence.size()!=4) throw std::runtime_error("WMV9 Main Profile sequence header must be exactly 4 bytes");
            mkv=std::make_unique<MatroskaVideoMuxer>(outpath,param.i_width,param.i_height,param.i_width,param.i_height,param.i_fps_num,param.i_fps_den,
                std::array<uint8_t,4>{{'W','M','V','3'}},sequence);
        } else if(!raw_output&&!wvc1&&!mkv_output) m2ts=std::make_unique<M2tsMuxer>(outf);
        uint64_t input_frames=0; const uint64_t limit=static_cast<uint64_t>(std::max<int64_t>(0,maxframes)); uint64_t expected_recon=0,expected_debug=0; std::map<uint64_t,ReconRecord> reconq; std::map<uint64_t,Frame> debug_source; std::map<uint64_t,std::string> debug_rows;
        auto process_output=[&](vc1_au_t* au,const vc1_picture_t& pic){
            if(!au)return;
            if(rcstatsf.is_open()){const char t=au->i_type==VC1_TYPE_I?'I':(au->i_type==VC1_TYPE_B?'B':'P');rcstatsf<<au->i_display_order<<','<<au->i_coded_order<<','<<t<<','<<au->b_keyframe<<','<<au->f_rc_complexity<<','<<au->i_rc_predicted_q<<','<<au->i_qp<<','<<au->f_rc_predicted_bits<<','<<au->i_rc_first_actual_bits<<','<<(static_cast<uint64_t>(au->i_payload)*8ull)<<','<<au->f_rc_target_bits<<','<<au->f_rc_allowed_bits<<','<<au->f_rc_prediction_error_percent<<','<<au->f_rc_vbv_before_bits<<','<<au->f_rc_vbv_after_bits<<','<<au->i_rc_retries<<','<<au->b_rc_reencoded<<'\n';if(!rcstatsf)throw std::runtime_error("write failed on rate-control statistics output");}
            if(macroblockstatsf.is_open()){
                const char t=au->i_type==VC1_TYPE_I?'I':(au->i_type==VC1_TYPE_B?'B':'P');
                auto mode_name=[](vc1_mb_debug_mode_e m){switch(m){case VC1_MB_DEBUG_I:return "I";case VC1_MB_DEBUG_P_INTER:return "P-inter";case VC1_MB_DEBUG_P_4MV:return "P-4mv";case VC1_MB_DEBUG_P_INTRA:return "P-intra";case VC1_MB_DEBUG_P_SKIPPED:return "P-skip";case VC1_MB_DEBUG_B_FORWARD:return "B-forward";case VC1_MB_DEBUG_B_BACKWARD:return "B-backward";case VC1_MB_DEBUG_B_INTERPOLATED:return "B-bi";case VC1_MB_DEBUG_B_DIRECT:return "B-direct";case VC1_MB_DEBUG_B_INTRA:return "B-intra";case VC1_MB_DEBUG_BI_INTRA:return "BI-intra";case VC1_MB_DEBUG_FIELD_P_FORWARD:return "field-P-forward";case VC1_MB_DEBUG_FIELD_B_FORWARD:return "field-B-forward";case VC1_MB_DEBUG_FIELD_B_BACKWARD:return "field-B-backward";default:return "unknown";}};
                auto distant_name=[](vc1_distant_match_decision_e d){switch(d){case VC1_DISTANT_MATCH_REJECTED:return "rejected";case VC1_DISTANT_MATCH_ABSOLUTE_GOOD:return "good";case VC1_DISTANT_MATCH_MATERIAL_IMPROVEMENT:return "improved";default:return "none";}};
                uint64_t local_bits=0;for(size_t mi=0;mi<au->i_debug_macroblocks;++mi)local_bits+=au->p_debug_macroblocks[mi].i_local_bits;const uint64_t frame_bits=static_cast<uint64_t>(au->i_payload)*8ull;const uint64_t shared_bits=frame_bits>local_bits?frame_bits-local_bits:0;
                macroblockstatsf<<"frame,"<<au->i_display_order<<','<<au->i_coded_order<<','<<t<<','<<frame_bits<<','<<shared_bits<<','<<au->f_rc_target_bits<<','<<au->f_rc_predicted_bits<<','<<au->f_rc_vbv_before_bits<<','<<au->f_rc_vbv_after_bits<<','<<au->i_qp<<'\n';
                for(size_t mi=0;mi<au->i_debug_macroblocks;++mi){const auto& d=au->p_debug_macroblocks[mi];macroblockstatsf<<"mb";for(int z=0;z<11;++z)macroblockstatsf<<',';macroblockstatsf<<d.i_mb_x<<','<<d.i_mb_y<<','<<d.i_visible_width<<','<<d.i_visible_height<<','<<mode_name(d.i_mode)<<','<<d.i_field_index<<','<<d.i_field_parity<<','<<d.b_opposite_field_reference<<','<<d.b_skipped<<','<<d.b_intra<<','<<d.b_four_mv<<','<<d.b_direct<<','<<d.b_acpred<<','<<d.i_mquant<<','<<d.i_dquant_delta<<','<<d.i_cbp<<','<<d.i_coded_blocks<<','<<d.i_local_bits<<','<<d.f_amortized_frame_bits<<','<<d.i_estimated_transform_bits<<','<<d.i_transform_parts[0]<<','<<d.i_transform_parts[1]<<','<<d.i_transform_parts[2]<<','<<d.i_transform_parts[3]<<','<<d.i_forward_mv_count<<','<<d.i_forward_reference_display_order<<','<<d.i_forward_mv_xq[0]<<','<<d.i_forward_mv_yq[0]<<','<<d.i_forward_mv_xq[1]<<','<<d.i_forward_mv_yq[1]<<','<<d.i_forward_mv_xq[2]<<','<<d.i_forward_mv_yq[2]<<','<<d.i_forward_mv_xq[3]<<','<<d.i_forward_mv_yq[3]<<','<<d.i_backward_mv_count<<','<<d.i_backward_reference_display_order<<','<<d.i_backward_mv_xq[0]<<','<<d.i_backward_mv_yq[0]<<','<<d.i_backward_mv_xq[1]<<','<<d.i_backward_mv_yq[1]<<','<<d.i_backward_mv_xq[2]<<','<<d.i_backward_mv_yq[2]<<','<<d.i_backward_mv_xq[3]<<','<<d.i_backward_mv_yq[3]<<','<<d.f_mean_y<<','<<d.f_stddev_y<<','<<d.f_mean_u<<','<<d.f_mean_v<<','<<d.f_activity_y<<','<<d.f_previous_sad_y<<','<<d.f_previous_mse_y<<','<<d.f_prediction_sad_y<<','<<d.f_prediction_mse_y<<','<<d.f_prediction_mae_y<<','<<d.f_prediction_gain_db<<','<<d.f_residual_priority_position<<','<<d.f_residual_priority_requested_q_boost<<','<<d.i_residual_priority_applied_q_boost<<','<<d.f_inter_intra_cost_ratio<<','<<d.f_forward_distant_local_mae_y<<','<<d.f_forward_distant_candidate_mae_y<<','<<distant_name(d.i_forward_distant_match_decision)<<','<<d.f_backward_distant_local_mae_y<<','<<d.f_backward_distant_candidate_mae_y<<','<<distant_name(d.i_backward_distant_match_decision)<<','<<d.f_aq_dark_detail<<','<<d.f_aq_color_luma_priority<<','<<d.f_aq_color_chroma_priority<<','<<d.f_aq_requested_q_boost<<','<<d.f_recon_mse_y<<','<<d.f_recon_mse_u<<','<<d.f_recon_mse_v<<','<<d.f_recon_mse_yuv<<','<<d.f_recon_snr_y_db<<','<<d.f_recon_psnr_y_db<<','<<d.f_recon_snr_yuv_db<<','<<d.f_recon_psnr_yuv_db<<'\n';}
                if(!macroblockstatsf)throw std::runtime_error("write failed on macroblock statistics output");
            }
            constexpr uint64_t initial90=9000;
            const uint64_t reorder=(param.b_intra_only||param.i_bframes==0)?0ull:1ull;
            const uint64_t pts90=initial90+((au->i_display_order+reorder)*90000ull*param.i_fps_den)/param.i_fps_num;
            const uint64_t dts90=initial90+(au->i_coded_order*90000ull*param.i_fps_den)/param.i_fps_num;
            std::vector<uint8_t> payload(au->p_payload,au->p_payload+au->i_payload);
            if(raw_output){
                outf.write(reinterpret_cast<const char*>(au->p_payload),static_cast<std::streamsize>(au->i_payload));
                if(!outf)throw std::runtime_error("write failed on elementary stream output");
            } else if(asf_output){
                const uint64_t ptsms=(au->i_display_order*1000ull*param.i_fps_den)/param.i_fps_num;
                const uint64_t dtsms=(au->i_coded_order*1000ull*param.i_fps_den)/param.i_fps_num;
                if(wvc1&&!asf){
                    auto first=prepare_wvc1_first_access_unit(payload,!param.b_intra_only&&param.i_bframes>0,param.i_scan_mode!=VC1_SCAN_PROGRESSIVE);
                    asf=std::make_unique<AsfVideoMuxer>(outf,coded_width,coded_height,Rational{param.i_fps_num,param.i_fps_den},std::array<uint8_t,4>{{'W','V','C','1'}},first.codec_private,param.i_rc_method==VC1_RC_CQP?0:initial.i_hrd_rate_bits);
                    payload=std::move(first.frame_payload);
                }
                asf->write_video_frame(payload,ptsms,dtsms,au->b_keyframe!=0);
            } else if(mkv_output) {
                const uint64_t dtsns=(au->i_coded_order*1000000000ull*static_cast<uint64_t>(param.i_fps_den))/static_cast<uint64_t>(param.i_fps_num);
                if(mkv_wvc1&&!mkv){
                    const auto init=prepare_wvc1_matroska_codec_private(payload);
                    mkv=std::make_unique<MatroskaVideoMuxer>(outpath,coded_width,coded_height,param.i_width,param.i_height,param.i_fps_num,param.i_fps_den,
                        std::array<uint8_t,4>{{'W','V','C','1'}},init);
                }
                mkv->write_video_frame(payload,dtsns,au->b_keyframe!=0);
            } else {
                m2ts->write_video_access_unit(payload,pts90,dts90,au->b_keyframe!=0);
            }
            if(esf.is_open()){
                esf.write(reinterpret_cast<const char*>(au->p_payload),static_cast<std::streamsize>(au->i_payload));
                if(!esf)throw std::runtime_error("write failed on elementary stream");
            }
            if(param.b_recon){
                ReconRecord r;r.type=pic.i_type;r.skipped_picture=pic.b_skipped_picture!=0;r.frame.width=param.i_width;r.frame.height=param.i_height;
                const int cw=(param.i_width+1)/2,ch=(param.i_height+1)/2;
                r.frame.y.resize(static_cast<size_t>(param.i_width)*param.i_height);r.frame.u.resize(static_cast<size_t>(cw)*ch);r.frame.v.resize(static_cast<size_t>(cw)*ch);
                for(int y=0;y<param.i_height;++y)std::copy_n(pic.img.plane[0]+static_cast<size_t>(y)*pic.img.i_stride[0],param.i_width,r.frame.y.data()+static_cast<size_t>(y)*param.i_width);
                for(int y=0;y<ch;++y){std::copy_n(pic.img.plane[1]+static_cast<size_t>(y)*pic.img.i_stride[1],cw,r.frame.u.data()+static_cast<size_t>(y)*cw);std::copy_n(pic.img.plane[2]+static_cast<size_t>(y)*pic.img.i_stride[2],cw,r.frame.v.data()+static_cast<size_t>(y)*cw);}
                if(pic.prop.transform_map&&pic.prop.transform_map_size)r.transform_map.assign(pic.prop.transform_map,pic.prop.transform_map+pic.prop.transform_map_size);
                if(debugstatsf.is_open()) {
                    auto sit=debug_source.find(au->i_display_order);
                    if(sit==debug_source.end()) throw std::runtime_error("debug statistics lost source frame");
                    const auto qy=plane_quality(sit->second.y,r.frame.y),qu=plane_quality(sit->second.u,r.frame.u),qv=plane_quality(sit->second.v,r.frame.v);
                    const auto qa=combined_quality(qy,qu,qv);
                    const char t=au->i_type==VC1_TYPE_I?'I':(au->i_type==VC1_TYPE_B?'B':'P');
                    const char* reason=au->b_keyframe?(au->b_debug_scene_i?"scene":(au->b_debug_motion_failure_i?"motion-failure":"gop-start")):"none";
                    const uint64_t final_bits=static_cast<uint64_t>(au->i_payload)*8ull;
                    const double bpp=static_cast<double>(final_bits)/std::max(1,param.i_width*param.i_height);
                    const double timestamp=static_cast<double>(au->i_display_order)*param.i_fps_den/param.i_fps_num;
                    const char* profile_name=actual.i_profile==VC1_PROFILE_MAIN?"main":"advanced";
                    const char* rc_name=actual.i_rc_method==VC1_RC_CQP?"cqp":(actual.b_rc_maximize?"abr-max":"abr");
                    const char* ac_name=actual.i_ac_mode==VC1_AC_ESC3?"esc3":(actual.i_ac_mode==VC1_AC_VLC?"vlc":"auto");
                    const uint64_t mb_total=static_cast<uint64_t>((param.i_width+15)/16)*static_cast<uint64_t>((param.i_height+15)/16);
                    std::ostringstream row; row<<std::setprecision(12)
                        <<au->i_display_order<<','<<au->i_coded_order<<','<<au->i_debug_gop_index<<','<<au->i_debug_frame_in_gop<<','<<au->i_debug_gop_frames<<','<<t<<','<<au->b_keyframe<<','<<reason<<','<<au->b_skipped_picture<<','<<au->i_pts<<','<<au->i_dts<<','<<timestamp<<','<<profile_name<<','<<rc_name<<','<<actual.i_threads<<','<<simd_name(initial.i_simd_selected)<<','<<actual.i_motion_search_range<<','<<actual.i_motion_local_search_range<<','<<(actual.b_intra_only?0:actual.i_bframes)<<','<<actual.i_trellis<<','<<(actual.b_adaptive_quality?actual.f_aq_strength:0.0)<<','<<actual.b_loop_filter<<','<<actual.b_variable_transforms<<','<<actual.b_dquant<<','<<ac_name<<','<<mb_total<<','<<au->i_qp<<','<<au->b_halfqp<<','<<(au->i_quantizer_type==VC1_QUANTIZER_NONUNIFORM?"nonuniform":"uniform")<<','<<au->f_quant_step<<','<<au->f_debug_qscale<<','<<au->f_rc_complexity<<','<<au->f_debug_motion_residual<<','<<au->i_rc_predicted_q<<','<<au->f_rc_predicted_bits<<','<<au->i_rc_first_actual_bits<<','<<final_bits<<','<<au->f_rc_target_bits<<','<<au->f_debug_rc_planned_bits<<','<<au->f_rc_allowed_bits<<','<<au->f_rc_prediction_error_percent<<','<<au->f_rc_vbv_before_bits<<','<<au->f_rc_vbv_after_bits<<','<<au->i_rc_retries<<','<<au->b_rc_reencoded<<','<<au->f_debug_gop_budget_scale<<','<<au->f_debug_gop_difficulty<<','<<au->i_debug_encode_trials<<','<<au->i_debug_moved_macroblocks<<','<<au->i_debug_fractional_chroma_macroblocks<<','<<au->i_debug_skipped_macroblocks<<','<<au->i_debug_explicit_macroblocks<<','<<au->i_debug_coded_macroblocks<<','<<au->i_debug_coded_blocks<<','<<au->i_debug_four_mv_macroblocks<<','<<au->i_debug_intra_macroblocks<<','<<au->i_debug_dquant_macroblocks<<','<<au->i_debug_mquant_min<<','<<au->i_debug_mquant_max<<','<<au->f_debug_mquant_mean<<','<<au->i_debug_transform_parts[0]<<','<<au->i_debug_transform_parts[1]<<','<<au->i_debug_transform_parts[2]<<','<<au->i_debug_transform_parts[3]<<','<<au->b_debug_ttmbf<<','<<au->i_debug_ttfrm<<','<<au->b_debug_ttfrm_exact_checked<<','<<au->i_debug_acpred_macroblocks<<','<<au->b_debug_intensity_comp<<','<<au->i_debug_b_forward<<','<<au->i_debug_b_backward<<','<<au->i_debug_b_interpolated<<','<<au->i_debug_b_direct<<','<<au->f_debug_mean_mv_pixels<<','<<au->f_debug_max_mv_pixels<<','<<qy.mse<<','<<qu.mse<<','<<qv.mse<<','<<qa.mse<<','<<qy.snr_db<<','<<qu.snr_db<<','<<qv.snr_db<<','<<qa.snr_db<<','<<qy.psnr_db<<','<<qu.psnr_db<<','<<qv.psnr_db<<','<<qa.psnr_db<<','<<bpp<<'\n';
                    debug_rows.emplace(au->i_display_order,row.str());
                    debug_source.erase(sit);
                    while(true){auto dit=debug_rows.find(expected_debug);if(dit==debug_rows.end())break;debugstatsf<<dit->second;if(!debugstatsf)throw std::runtime_error("write failed on debug statistics output");debug_rows.erase(dit);++expected_debug;}
                }
                reconq.emplace(pic.i_display_order,std::move(r));
                while(true){auto it=reconq.find(expected_recon);if(it==reconq.end())break;if(reconf.is_open())write_frame_raw(reconf,it->second.frame);if(transformf.is_open()){auto o=it->second.skipped_picture?it->second.frame:make_transform_overlay(it->second.frame,it->second.transform_map,it->second.type,param.i_width,param.i_height);write_y4m_frame(transformf,o);}reconq.erase(it);++expected_recon;}
            }
        };
        Frame f;while(input_frames<limit&&y4m.read(f)){if(debugstatsf.is_open())debug_source.emplace(input_frames,f);vc1_picture_t inpic;vc1_picture_init(&inpic);inpic.i_pts=static_cast<int64_t>(input_frames);inpic.img.i_plane=3;inpic.img.plane[0]=f.y.data();inpic.img.plane[1]=f.u.data();inpic.img.plane[2]=f.v.data();inpic.img.i_stride[0]=param.i_width;inpic.img.i_stride[1]=(param.i_width+1)/2;inpic.img.i_stride[2]=(param.i_width+1)/2;vc1_au_t* au=nullptr;int nau=0;vc1_picture_t outpic;if(vc1_encoder_encode(enc.get(),&au,&nau,&outpic,&inpic)<0)throw std::runtime_error(vc1_encoder_last_error(enc.get()));if(nau)process_output(au,outpic);++input_frames;f=Frame{};}
        while(vc1_encoder_delayed_frames(enc.get())>0){vc1_au_t* au=nullptr;int nau=0;vc1_picture_t outpic;if(vc1_encoder_encode(enc.get(),&au,&nau,&outpic,nullptr)<0)throw std::runtime_error(vc1_encoder_last_error(enc.get()));if(nau)process_output(au,outpic);else if(vc1_encoder_delayed_frames(enc.get())>0)throw std::runtime_error("libvc1 flush made no progress");}
        if (debugstatsf.is_open() && (!debug_source.empty() || !debug_rows.empty() || expected_debug!=input_frames))
            throw std::runtime_error("debug statistics did not receive exactly one row per input frame");
        if (input_frames==0)
            throw std::runtime_error("input contained no frames");
        if (asf) asf->finalize();
        if (mkv) mkv->finalize();
        vc1_stats_t st{};vc1_encoder_stats(enc.get(),&st);
        if(!speedprofilepath.empty())write_speed_profile_report(speedprofilepath,actual,st,input_frames,speed_preset);
        std::cerr<<"encoded "<<input_frames<<" frame(s), "<<actual.i_width<<"x"<<actual.i_height;
        if ((actual.i_width&1)||(actual.i_height&1)) std::cerr<<" (coded "<<((actual.i_width+1)&~1)<<"x"<<((actual.i_height+1)&~1)<<")";
        std::cerr<<" @ "<<actual.i_fps_num<<"/"<<actual.i_fps_den<<" fps; I="<<st.i_i_frames<<", P="<<st.i_p_frames<<", B="<<st.i_b_frames
                 <<", scene-I="<<st.i_scene_i_frames<<", motion-failure-I="<<st.i_motion_failure_i_frames<<", IC-P="<<st.i_ic_p_frames
                 <<", B-MB="<<st.i_b_forward<<"F/"<<st.i_b_backward<<"B/"<<st.i_b_interpolated<<"I/"<<st.i_b_direct<<"D"
                 <<", moved-MB="<<st.i_moved_macroblocks<<", halfchroma-MB="<<st.i_fractional_chroma_macroblocks
                 <<", skipped-MB="<<st.i_skipped_macroblocks<<", skipped-P="<<st.i_skipped_pictures
                 <<", explicit-MB="<<st.i_explicit_macroblocks<<", coded-MB="<<st.i_coded_macroblocks<<", coded-blocks="<<st.i_coded_blocks
                 <<", P-4MV="<<st.i_four_mv_macroblocks<<", P-intra="<<st.i_intra_macroblocks<<", DQUANT-MB="<<st.i_dquant_macroblocks
                 <<", TT=8x8:"<<st.i_transform_parts[0]<<"/8x4:"<<st.i_transform_parts[1]<<"/4x8:"<<st.i_transform_parts[2]<<"/4x4:"<<st.i_transform_parts[3];
        if(actual.i_rc_method!=VC1_RC_CQP) {
            const double avgq=input_frames?static_cast<double>(st.i_q_sum)/input_frames:0.0;
            const double avgrate=st.i_rate_frames?static_cast<double>(st.i_rate_total_bits)*actual.i_fps_num/(static_cast<double>(st.i_rate_frames)*actual.i_fps_den):0.0;
            std::cerr<<", hrd-rate="<<st.i_hrd_rate_bits<<" bps, hrd-buffer="<<st.i_hrd_buffer_bits<<" bits"
                     <<", avg="<<static_cast<uint64_t>(std::llround(avgrate))<<" bps"
                     <<", hrd-init="<<st.i_hrd_init_bits<<" bits, hrd-min-after="<<static_cast<uint64_t>(std::floor(st.f_hrd_min_after_bits))
                     <<" bits, hrd-max-pre="<<static_cast<uint64_t>(std::ceil(st.f_hrd_max_pre_bits))<<" bits"
                     <<", hrd-pauses="<<st.i_hrd_pauses<<", hrd-underflows="<<st.i_hrd_underflows
                     <<", q="<<st.i_q_min<<".."<<st.i_q_max<<" (avg "<<avgq<<")"
                     <<", halfqp-frames="<<st.i_halfqp_frames<<", quantizer-frames="<<st.i_uniform_quantizer_frames<<"U/"<<st.i_nonuniform_quantizer_frames<<"N";
        }
        if(st.i_compute_selected==VC1_COMPUTE_VULKAN)
            std::cerr<<", vulkan-dispatches="<<st.i_compute_dispatches<<", vulkan-candidates="<<st.i_compute_candidates
                     <<", vulkan-uploads="<<st.i_compute_frame_uploads<<"/"<<st.i_compute_frame_upload_bytes<<"B"
                     <<", vulkan-wait-ms="<<(static_cast<double>(st.i_compute_submit_wait_nanoseconds)/1.0e6);
        std::cerr<<", gops="<<st.i_gops<<"\n";
        return 0;
    } catch(const std::exception& e){std::cerr<<"vc1enc: "<<e.what()<<"\n";return 1;}
}
