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

BitplaneChoice Vc1Encoder::choose_bitplane(const std::vector<uint8_t>& plane, int w, int h) {
        if (plane.size()!=static_cast<size_t>(w)*h)
            throw std::runtime_error("internal bitplane size mismatch");
        BitplaneChoice best;
        best.raw=true; best.imode=0;
        best.syntax.bit(false);
        best.syntax.vlc(groupa::kImodeCodes[0],groupa::kImodeBits[0]);
        best.total_bits=best.syntax.bit_count()+plane.size();
        auto try_mode=[&](int mode, bool invert) {
            std::vector<uint8_t> p(plane.size());
            for (size_t i=0;i<plane.size();++i) p[i]=static_cast<uint8_t>((plane[i]!=0)^invert);
            BitWriter x;
            x.bit(invert);
            x.vlc(groupa::kImodeCodes[static_cast<size_t>(mode)],groupa::kImodeBits[static_cast<size_t>(mode)]);
            if (mode==1) encode_norm2_bits(x,p);
            else if (mode==3) encode_norm6_bits(x,p,w,h);
            else if (mode==5) encode_rowskip_bits(x,p,w,h);
            else if (mode==6) encode_colskip_bits(x,p,w,h);
            else return;
            if (x.bit_count() < best.total_bits) {
                best.syntax=std::move(x); best.raw=false; best.imode=mode; best.total_bits=best.syntax.bit_count();
            }
        };
        for (int mode : {1,3,5,6}) { try_mode(mode,false); try_mode(mode,true); }
        return best;
    }

int Vc1Encoder::choose_cbp_table(const std::vector<uint8_t>& cbps) {
        uint64_t best=std::numeric_limits<uint64_t>::max(); int best_t=0;
        for (int t=0;t<4;++t) {
            uint64_t bits=0;
            for (uint8_t cbp:cbps) if (cbp)
                bits += groupa::kPcbpBits[static_cast<size_t>(t)][cbp];
            if (bits<best) { best=bits; best_t=t; }
        }
        return best_t;
    }

void Vc1Encoder::write_cbp_table(BitWriter& b,int table,uint8_t cbp) {
        b.vlc(groupa::kPcbpCodes[static_cast<size_t>(table)][cbp],
              groupa::kPcbpBits[static_cast<size_t>(table)][cbp]);
    }

int Vc1Encoder::choose_inter_ac_index(double residual,int pqindex) {
        // Table 0 is the High-Rate coding set at PQ<=8 and Low-Motion above it.
        // Preserve that strong default, but choose mid/high-motion sets when the
        // already-computed motion residual says they are more likely to win.
        if (pqindex<=8) {
            if (residual>14.0) return 1;
            if (residual>7.0) return 2;
            return 0;
        }
        if (residual<3.0) return 0;
        if (residual>12.0) return 1;
        return 2;
    }

size_t Vc1Encoder::block_index(int mbw, int mx, int my, int k) {
        return (static_cast<size_t>(my)*mbw + mx)*6 + static_cast<size_t>(k);
    }

void Vc1Encoder::write_decode012(BitWriter& b, int index) {
        if (index < 0 || index > 2)
            throw std::runtime_error("internal AC table index outside decode012 range");
        if (index == 0) b.bit(false);
        else {
            b.bit(true);
            b.bit(index == 2);
        }
    }

int Vc1Encoder::luma_coding_set(int table_index, int pqindex) {
        // For Advanced Profile I pictures: index 0 changes at PQINDEX 8/9.
        if (table_index == 0) {
            if (pqindex <= 8) return 6; // HIGH_RATE_INTRA
            return 2; // LOW_MOT_INTRA
        }
        if (table_index == 1) return 0; // HIGH_MOT_INTRA
        if (table_index == 2) return 4; // MID_RATE_INTRA
        throw std::runtime_error("invalid luma AC table index");
    }

int Vc1Encoder::chroma_coding_set(int table_index, int pqindex) {
        if (table_index == 0) {
            if (pqindex <= 8) return 7; // HIGH_RATE_INTER
            return 3; // LOW_MOT_INTER
        }
        if (table_index == 1) return 1; // HIGH_MOT_INTER
        if (table_index == 2) return 5; // MID_RATE_INTER
        throw std::runtime_error("invalid chroma AC table index");
    }

AcTableView Vc1Encoder::ac_table(int coding_set) {
        switch (coding_set) {
            case 0: return {entropy::kHighMotionIntra.data(), entropy::kHighMotionIntra.size()};
            case 1: return {entropy::kHighMotionInter.data(), entropy::kHighMotionInter.size()};
            case 2: return {entropy::kLowMotionIntra.data(), entropy::kLowMotionIntra.size()};
            case 3: return {entropy::kLowMotionInter.data(), entropy::kLowMotionInter.size()};
            case 4: return {entropy::kMidRateIntra.data(), entropy::kMidRateIntra.size()};
            case 5: return {entropy::kMidRateInter.data(), entropy::kMidRateInter.size()};
            case 6: return {entropy::kHighRateIntra.data(), entropy::kHighRateIntra.size()};
            case 7: return {entropy::kHighRateInter.data(), entropy::kHighRateInter.size()};
            default: throw std::runtime_error("unsupported AC coding set");
        }
    }

entropy::VlcCode Vc1Encoder::ac_escape(int coding_set) {
        switch (coding_set) {
            case 0: return entropy::kHighMotionIntraEscape;
            case 1: return entropy::kHighMotionInterEscape;
            case 2: return entropy::kLowMotionIntraEscape;
            case 3: return entropy::kLowMotionInterEscape;
            case 4: return entropy::kMidRateIntraEscape;
            case 5: return entropy::kMidRateInterEscape;
            case 6: return entropy::kHighRateIntraEscape;
            case 7: return entropy::kHighRateInterEscape;
            default: throw std::runtime_error("unsupported AC coding set");
        }
    }

const AcLookup& Vc1Encoder::ac_lookup(int coding_set) {
        static const std::array<AcLookup,8> lookups=[] {
            std::array<AcLookup,8> out;
            for (int set=0; set<8; ++set) {
                const auto [table,count]=ac_table(set);
                for (size_t i=0; i<count; ++i) {
                    const AcVlcEntry& e=table[i];
                    const size_t l=e.last?1u:0u;
                    if (e.level >= 64)
                        throw std::runtime_error("AC direct VLC level exceeds lookup cache range");
                    out[static_cast<size_t>(set)].direct_index[(l*64u+e.run)*64u+e.level]=
                        static_cast<int16_t>(i);
                    auto& ml=out[static_cast<size_t>(set)].max_level[l*64u+e.run];
                    ml=std::max<uint8_t>(ml,e.level);
                    auto& mr=out[static_cast<size_t>(set)].max_run[l*64u+e.level];
                    mr=std::max<int8_t>(mr,static_cast<int8_t>(e.run));
                }
            }
            return out;
        }();
        if (coding_set < 0 || coding_set >= static_cast<int>(lookups.size()))
            throw std::runtime_error("unsupported AC coding set");
        return lookups[static_cast<size_t>(coding_set)];
    }

const AcVlcEntry* Vc1Encoder::find_ac_vlc(int coding_set, int run, int level, bool last) {
        if (run < 0 || run > 63 || level < 0 || level >= 64) return nullptr;
        const size_t l=last?1u:0u;
        const int16_t index=ac_lookup(coding_set).direct_index[
            (l*64u+static_cast<size_t>(run))*64u+static_cast<size_t>(level)];
        if (index < 0) return nullptr;
        const auto [table,count]=ac_table(coding_set);
        if (static_cast<size_t>(index) >= count)
            throw std::runtime_error("internal AC lookup index out of range");
        return &table[static_cast<size_t>(index)];
    }

int Vc1Encoder::max_direct_level(int coding_set, int run, bool last) {
        if (run < 0 || run > 63) return 0;
        const size_t l=last?1u:0u;
        return ac_lookup(coding_set).max_level[l*64u+static_cast<size_t>(run)];
    }

int Vc1Encoder::max_direct_run(int coding_set, int level, bool last) {
        if (level < 0 || level >= 64) return -1;
        const size_t l=last?1u:0u;
        return ac_lookup(coding_set).max_run[l*64u+static_cast<size_t>(level)];
    }

void Vc1Encoder::write_ac_sign(BitWriter& b, int level) { b.bit(level < 0); }

int Vc1Encoder::esc3_level_bits() const {
        // ST 421 Table 59 deliberately provides an 11-bit conservative
        // magnitude at PQ 1..7.  Low-PQ pictures need that range: clipping
        // transform levels to the 8-bit Table-60 range makes the nominally
        // highest-quality pictures lose detail.
        return c_.pqindex < 8 ? 11 : 8;
    }

int Vc1Encoder::max_quantized_level() const {
        return (1 << esc3_level_bits()) - 1;
    }

void Vc1Encoder::write_ac_esc3(BitWriter& b, int run, int level, bool last,
                       int coding_set, bool& esc3_lengths_written,
                       bool dquantfrm) const {
        const auto esc=ac_escape(coding_set);
        const int level_bits=esc3_level_bits();
        b.vlc(esc.code,esc.bits);
        b.bits(0,2); // Table 58: 00 = Escape Mode 3.
        b.bit(last);
        if (!esc3_lengths_written) {
            if (c_.pqindex < 8 || dquantfrm) {
                // Table 59 is mandatory at PQ<8 and whenever DQUANTFRM=1.
                // Its 000 prefix selects the extended 8..11-bit form; use
                // 11 bits at PQ1..7, otherwise the compact legal 8-bit width.
                b.bits(c_.pqindex < 8 ? 0b00011 : 0b00000,5);
            } else {
                // Table 60 efficient ESCLVLSZ: 000000 selects 8 bits.
                b.bits(0,6);
            }
            b.bits(3,2); // 3 + 3 = 6 run bits.
            esc3_lengths_written=true;
        }
        b.bits(static_cast<uint64_t>(run),6);
        write_ac_sign(b,level);
        b.bits(static_cast<uint64_t>(std::abs(level)),level_bits);
    }

void Vc1Encoder::write_ac_coeff(BitWriter& b, int run, int level, bool last,
                        int coding_set, bool use_vlc, bool& esc3_lengths_written,
                        bool dquantfrm) const {
        const int mag=std::abs(level);
        if (run < 0 || run > 63 || level == 0 || mag > max_quantized_level())
            throw std::runtime_error("AC coefficient outside signaled Escape-3 range");

        if (!use_vlc) {
            write_ac_esc3(b,run,level,last,coding_set,esc3_lengths_written,dquantfrm);
            return;
        }

        if (const auto* direct=find_ac_vlc(coding_set,run,mag,last)) {
            b.vlc(direct->code,direct->bits);
            write_ac_sign(b,level);
            return;
        }

        struct EscapeCandidate { int mode=3; const AcVlcEntry* base=nullptr; int bits=1000000; };
        EscapeCandidate best;
        const auto esc=ac_escape(coding_set);

        const int level_delta=max_direct_level(coding_set,run,last);
        // Keep Low-Motion Inter (coding set 3) off Escape Mode 1.  The same
        // coefficient is always representable with Escape Mode 2/3, and this
        // conservative form avoids the WMV3 B-picture packet desynchronization
        // regression exposed when overlap-filtered references alter RDO choices.
        // Direct VLCs and Escape Modes 2/3 remain fully enabled.
        if (coding_set != 3 && level_delta > 0 && mag > level_delta) {
            const int base_level=mag-level_delta;
            if (const auto* base=find_ac_vlc(coding_set,run,base_level,last))
                best={1,base,static_cast<int>(esc.bits) + 1 + base->bits + 1};
        }

        const int run_delta=max_direct_run(coding_set,mag,last);
        if (run_delta >= 0 && run > run_delta) {
            const int base_run=run-run_delta-1;
            if (const auto* base=find_ac_vlc(coding_set,base_run,mag,last)) {
                const int bits=static_cast<int>(esc.bits) + 2 + base->bits + 1;
                if (bits < best.bits) best={2,base,bits};
            }
        }

        const int esc3_header_bits=(c_.pqindex < 8 || dquantfrm) ? 7 : 8; // Table 59 vs 60 ESCLVLSZ + ESCRUNSZ
        const int esc3_bits=static_cast<int>(esc.bits) + 2 + 1 + 6 + 1 + esc3_level_bits() +
                            (esc3_lengths_written ? 0 : esc3_header_bits);
        if (best.base && best.bits < esc3_bits) {
            b.vlc(esc.code,esc.bits);
            if (best.mode==1) b.bit(true);
            else b.bits(0b01,2);
            b.vlc(best.base->code,best.base->bits);
            write_ac_sign(b,level);
            return;
        }

        write_ac_esc3(b,run,level,last,coding_set,esc3_lengths_written,dquantfrm);
    }

uint64_t Vc1Encoder::estimate_intra_ac_bits(const std::array<int,64>& q, int coding_set,
                                    bool use_vlc, const uint8_t* scan) const {
        int last_scan=-1;
        for (int s=63;s>=1;--s) {
            if (q[scan[static_cast<size_t>(s)]] != 0) { last_scan=s; break; }
        }
        if (last_scan < 1) return 0;
        uint64_t bits=0;
        int previous=0;
        const int esc3_per_coeff=static_cast<int>(ac_escape(coding_set).bits)+2+1+6+1+esc3_level_bits();
        for (int s=1;s<=last_scan;++s) {
            const int level=q[scan[static_cast<size_t>(s)]];
            if (!level) continue;
            const int run=s-previous-1;
            bits+=static_cast<uint64_t>(use_vlc
                ? trellis_rate_bits_for_set(run,level,s==last_scan,coding_set)
                : esc3_per_coeff);
            previous=s;
        }
        return bits;
    }

void Vc1Encoder::write_ac_block(BitWriter& b, const std::array<int,64>& q,
                        bool /*chroma*/, int coding_set, bool use_vlc,
                        bool& esc3_lengths_written,
                        const uint8_t* scan, bool dquantfrm) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::Entropy);
        int last_scan=-1;
        for (int s=63; s>=1; --s) {
            if (q[scan[static_cast<size_t>(s)]] != 0) { last_scan=s; break; }
        }
        if (last_scan < 1)
            throw std::runtime_error("coded AC block unexpectedly has no coefficients");

        int previous=0;
        for (int s=1; s<=last_scan; ++s) {
            const int level=q[scan[static_cast<size_t>(s)]];
            if (!level) continue;
            const int run=s-previous-1;
            write_ac_coeff(b,run,level,s==last_scan,coding_set,use_vlc,esc3_lengths_written,dquantfrm);
            previous=s;
        }
    }

void Vc1Encoder::write_inter_block(BitWriter& b,const std::array<int,64>& q,
                           int coding_set,bool use_vlc,bool& esc3_lengths_written,
                           const uint8_t* scan,bool dquantfrm) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::Entropy);
        if (!scan) scan=kInterScan.data();
        int last_scan=-1;
        for (int s=63;s>=0;--s) {
            if (q[scan[static_cast<size_t>(s)]] != 0) { last_scan=s; break; }
        }
        if (last_scan<0)
            throw std::runtime_error("coded inter block unexpectedly has no coefficients");
        int previous=-1;
        for (int s=0;s<=last_scan;++s) {
            const int level=q[scan[static_cast<size_t>(s)]];
            if (!level) continue;
            const int run=s-previous-1;
            write_ac_coeff(b,run,level,s==last_scan,coding_set,use_vlc,esc3_lengths_written,dquantfrm);
            previous=s;
        }
    }

int Vc1Encoder::dc_scale(int pq) {
        static constexpr int t[32] = {
            0,2,4,8,8,8,9,9,10,10,11,11,12,12,13,13,
            14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21
        };
        return t[pq];
    }

int Vc1Encoder::arshift(int v, int n) {
        if (v >= 0) return v >> n;
        const int d = 1 << n;
        return - ((-v + d - 1) / d);
    }

int Vc1Encoder::reconstruct_dc_sample_with_bias(int qdc,int mquant,int bias) const {
        if (mquant<1) mquant=c_.pqindex;
        int x = qdc * dc_scale(mquant);
        x = arshift(3*x + 1, 1);
        x = arshift(3*x + 16, 5);
        return std::clamp(bias + x, 0, 255);
    }

int Vc1Encoder::reconstruct_dc_sample(int qdc,int mquant) const {
        return reconstruct_dc_sample_with_bias(qdc,mquant,intra_recon_bias());
    }

int Vc1Encoder::choose_dc_for_mean_with_bias(int mean,int mquant,int bias) const {
        if (mquant<1) mquant=c_.pqindex;
        // DCCOEF_ESC is 10 bits at Q1, 9 bits at Q2, and 8 bits above that.
        const int bits=8 + ((mquant == 1 || mquant == 2) ? 3-mquant : 0);
        const int maxq=(1 << bits)-1;
        int lo=-maxq, hi=maxq;
        while (lo < hi) {
            const int mid=lo+(hi-lo)/2;
            if (reconstruct_dc_sample_with_bias(mid,mquant,bias) < mean) lo=mid+1;
            else hi=mid;
        }
        int best=lo;
        if (lo > -maxq &&
            std::abs(reconstruct_dc_sample_with_bias(lo-1,mquant,bias)-mean) <=
                std::abs(reconstruct_dc_sample_with_bias(lo,mquant,bias)-mean))
            best=lo-1;
        return best;
    }

int Vc1Encoder::choose_dc_for_mean(int mean,int mquant) const {
        return choose_dc_for_mean_with_bias(mean,mquant,intra_recon_bias());
    }

int Vc1Encoder::choose_inter_intra_dc_for_mean(int mean,int mquant) const {
        // Progressive P/B intra blocks are stored through the decoder's signed
        // inter-picture clamp in both Advanced and Simple/Main profiles.  That
        // clamp adds 128 after inverse transform.  Main I/BI pictures have a
        // separate centering rule, so do not reuse intra_recon_bias() here.
        return choose_dc_for_mean_with_bias(mean,mquant,128);
    }

int Vc1Encoder::clamp_dc_target_for_diff(int target,int predictor,int mquant) {
        if (mquant<1) mquant=1;
        const int bits=8 + ((mquant==1 || mquant==2)?3-mquant:0);
        const int maxv=(1<<bits)-1;
        const int diff=std::clamp(target-predictor,-maxv,maxv);
        return std::clamp(predictor+diff,-maxv,maxv);
    }

int Vc1Encoder::dqscale_value(int index) {
        static constexpr int32_t t[63] = {
            0x40000,0x20000,0x15555,0x10000,0xCCCD,0xAAAB,0x9249,0x8000,
            0x71C7,0x6666,0x5D17,0x5555,0x4EC5,0x4925,0x4444,0x4000,
            0x3C3C,0x38E4,0x35E5,0x3333,0x30C3,0x2E8C,0x2C86,0x2AAB,
            0x28F6,0x2762,0x25ED,0x2492,0x234F,0x2222,0x2108,0x2000,
            0x1F08,0x1E1E,0x1D42,0x1C72,0x1BAD,0x1AF3,0x1A42,0x199A,
            0x18FA,0x1862,0x17D0,0x1746,0x16C1,0x1643,0x15CA,0x1555,
            0x14E6,0x147B,0x1414,0x13B1,0x1352,0x12F7,0x129E,0x1249,
            0x11F7,0x11A8,0x115B,0x1111,0x10C9,0x1084,0x1041
        };
        if (index<0 || index>=63) throw std::runtime_error("VC-1 DQScale index out of range");
        return t[index];
    }

int Vc1Encoder::scale_dc_predictor(int value,int from_q,int to_q) {
        from_q=std::abs(from_q); to_q=std::abs(to_q);
        if (!from_q || from_q==to_q) return value;
        const int idx=dc_scale(to_q)-1;
        if (idx<0) return 0;
        const int64_t v=static_cast<int64_t>(value)*dc_scale(from_q)*dqscale_value(idx);
        return static_cast<int>((v+0x20000)>>18);
    }

int Vc1Encoder::scale_ac_predictor(int value,int from_qstate,int to_qstate) const {
        if (!from_qstate || !to_qstate) return value;
        auto step=[&](int qs) {
            const int q=std::abs(qs);
            return q*2 + ((qs>0 && c_.halfqp)?1:0) - 1;
        };
        const int q2=step(from_qstate),q1=step(to_qstate);
        if (q1<1 || q2<1 || q1==q2) return value;
        const int64_t v=static_cast<int64_t>(value)*q2*dqscale_value(q1-1);
        return static_cast<int>((v+0x20000)>>18);
    }

int Vc1Encoder::block_mean(const std::vector<uint8_t>& p, int w, int h, int x0, int y0) {
        int sum=0;
        for (int y=0;y<8;++y) {
            int sy=std::clamp(y0+y,0,h-1);
            for (int x=0;x<8;++x) {
                int sx=std::clamp(x0+x,0,w-1);
                sum += p[static_cast<size_t>(sy)*w+sx];
            }
        }
        return (sum + 32) / 64;
    }

int Vc1Encoder::padded_block_mean(const std::vector<uint8_t>& p, int w, int h, int x0, int y0, int pad_sample) {
        uint64_t sum=0;
        for (int y=0;y<8;++y) {
            const int sy=y0+y;
            for (int x=0;x<8;++x) {
                const int sx=x0+x;
                sum += (sx>=0 && sy>=0 && sx<w && sy<h)
                    ? p[static_cast<size_t>(sy)*w+sx]
                    : static_cast<uint64_t>(std::clamp(pad_sample,0,255));
            }
        }
        return static_cast<int>((sum+32u)/64u);
    }

int Vc1Encoder::visible_block_mean(const std::vector<uint8_t>& p, int w, int h, int x0, int y0) {
        // Analysis of a cropped edge block must not let coded padding influence
        // an inter-vs-intra decision.  Intra coding uses padded_block_mean() with
        // black/neutral samples outside the visible crop.
        const int vw=std::max(0,std::min(8,w-x0));
        const int vh=std::max(0,std::min(8,h-y0));
        if (!vw || !vh) return 128;
        uint64_t sum=0;
        for (int y=0;y<vh;++y)
            for (int x=0;x<vw;++x)
                sum+=p[static_cast<size_t>(y0+y)*w+x0+x];
        const uint64_t n=static_cast<uint64_t>(vw)*vh;
        return static_cast<int>((sum+n/2)/n);
    }

int Vc1Encoder::main_dc_pred_base() const {
        // With active Main-profile overlap (PQ>=9), the decoder uses centered
        // intra samples and a zero unavailable-neighbor DC predictor. At low Q,
        // or when OVERLAP is disabled, it uses the legacy quantizer table.
        if (c_.overlap && c_.pqindex>=9) return 0;
        // This table is indexed by the DC quantization scale, not PQINDEX.
        static constexpr int dcpred[32] = {
            -1,1024,512,341,256,205,171,146,128,114,102,93,85,79,73,68,
            64,60,57,54,51,49,47,45,43,41,39,38,37,35,34,33
        };
        if (scale_ < 0 || scale_ >= 32)
            throw std::runtime_error("internal WMV9 DC scale outside predictor table");
        return dcpred[scale_];
    }

int Vc1Encoder::predict_dc_main(const std::vector<int>& g, int gw, int x, int y, int border,
                               bool* pred_left) {
        const int a = y > 0 ? g[static_cast<size_t>(y-1)*gw+x] : border;
        const int c = x > 0 ? g[static_cast<size_t>(y)*gw+x-1] : border;
        const int b = (x > 0 && y > 0) ? g[static_cast<size_t>(y-1)*gw+x-1] : border;
        const bool left=std::abs(a-b) <= std::abs(b-c);
        if (pred_left) *pred_left=left;
        return left ? c : a;
    }

int Vc1Encoder::predict_dc(const std::vector<int>& g, int gw, int x, int y,
                          bool a_avail, bool c_avail, bool* pred_left) {
        int a=0,b=0,c=0;
        if (a_avail) a = g[static_cast<size_t>(y-1)*gw+x];
        if (c_avail) c = g[static_cast<size_t>(y)*gw+x-1];
        if (a_avail && c_avail) b = g[static_cast<size_t>(y-1)*gw+x-1];
        const bool left=c_avail && (!a_avail || std::abs(a-b) <= std::abs(b-c));
        if (pred_left) *pred_left=left || (!a_avail && !c_avail);
        if (left) return c;
        if (a_avail) return a;
        return 0;
    }

int Vc1Encoder::predict_dc_dquant(const std::vector<int>& g,const std::vector<int>& qg,int gw,int x,int y,
                                  int current_q,bool a_avail,bool c_avail,bool* pred_left) {
        int a=0,b=0,c=0;
        if (a_avail) {
            const size_t i=static_cast<size_t>(y-1)*gw+x;
            a=scale_dc_predictor(g[i],qg[i],current_q);
        }
        if (c_avail) {
            const size_t i=static_cast<size_t>(y)*gw+x-1;
            c=scale_dc_predictor(g[i],qg[i],current_q);
        }
        if (a_avail && c_avail) {
            const size_t i=static_cast<size_t>(y-1)*gw+x-1;
            b=scale_dc_predictor(g[i],qg[i],current_q);
        }
        const bool left=c_avail && (!a_avail || std::abs(a-b) <= std::abs(b-c));
        if (pred_left) *pred_left=left || (!a_avail && !c_avail);
        if (left) return c;
        if (a_avail) return a;
        return 0;
    }

DcCodePlan Vc1Encoder::dc_code_plan(int diff,int pqindex) {
        const int mag=std::abs(diff);
        if (!mag) return DcCodePlan{};
        const int m=(pqindex==1 || pqindex==2)?3-pqindex:0;
        if (m==0) {
            if (mag<=118) return DcCodePlan{mag,0,0,false};
        } else {
            const int bias=(1<<m)-1;
            const int t=mag+bias;
            const int sym=t>>m;
            if (sym<=118) return DcCodePlan{sym,static_cast<unsigned>(t & bias),m,false};
        }
        return DcCodePlan{119,static_cast<unsigned>(mag),8+m,true};
    }

uint64_t Vc1Encoder::dc_diff_bits(int table,int diff,bool chroma,int pqindex) {
        const DcCodePlan p=dc_code_plan(diff,pqindex);
        return static_cast<uint64_t>(groupa::kDcVlc[static_cast<size_t>(table)][chroma?1u:0u][static_cast<size_t>(p.symbol)].bits)
            + p.suffix_bits + (diff?1u:0u);
    }

void Vc1Encoder::write_dc_diff(BitWriter& b,int diff,bool chroma,int pqindex,int table) {
        const DcCodePlan p=dc_code_plan(diff,pqindex);
        const auto& v=groupa::kDcVlc[static_cast<size_t>(table)][chroma?1u:0u][static_cast<size_t>(p.symbol)];
        b.vlc(v.code,v.bits);
        if (p.suffix_bits) b.bits(p.suffix,p.suffix_bits);
        if (diff) b.bit(diff<0);
    }

} // namespace libvc1
