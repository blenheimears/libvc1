#include "encoder_internal.h"

namespace libvc1 {

void BitWriter::append(const BitWriter& other) {
        for (uint8_t v : other.data_) bits(v,8);
        if (other.bits_ > 0) {
            for (int i=other.bits_-1;i>=0;--i) bit((other.cur_ >> i) & 1u);
        }
    }
std::vector<uint8_t> BitWriter::finish_rbdu() {
        // VC-1 Advanced Profile RBDU trailing bits: one stop bit followed by zero padding.
        bit(true);
        while (bits_ != 0) bit(false);
        return data_;
    }
std::vector<uint8_t> BitWriter::finish_raw() {
        // Simple/Main Profile frames in ASF are byte-aligned with zero padding;
        // they are not RBDUs and therefore have no rbdu_stop_one_bit.
        while (bits_ != 0) bit(false);
        return data_;
    }


std::vector<uint8_t> escape_ebdu(const std::vector<uint8_t>& in) {
    std::vector<uint8_t> out;
    out.reserve(in.size() + in.size()/64 + 8);
    int zeros = 0;
    for (uint8_t b : in) {
        if (zeros >= 2 && b <= 0x03) {
            out.push_back(0x03);
            zeros = 0;
        }
        out.push_back(b);
        if (b == 0) ++zeros; else zeros = 0;
    }
    return out;
}

std::vector<uint8_t> bdu(uint8_t suffix, std::vector<uint8_t> rbdu) {
    auto e = escape_ebdu(rbdu);
    std::vector<uint8_t> out{0x00,0x00,0x01,suffix};
    out.insert(out.end(), e.begin(), e.end());
    return out;
}


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

std::vector<uint8_t> Vc1Encoder::sequence_header() const {
        BitWriter b;
        if (c_.syntax == StreamSyntax::Wmv9Main) {
            // VC-1 Simple/Main Profile sequence layer. WMV9/WMV3 carries this
            // 32-bit header in the ASF video-format extradata rather than in-band.
            b.bits(1,2);                 // PROFILE = Main
            b.bit(false);                // RES_Y411
            b.bit(false);                // RES_SPRITE
            b.bits(0,3);                 // FRMRTQ_POSTPROC
            b.bits(0,5);                 // BITRTQ_POSTPROC
            b.bit(c_.loop_filter);       // LOOPFILTER (legal in Main Profile)
            b.bit(false);                // RES_X8
            b.bit(false);                // MULTIRES
            b.bit(true);                 // RES_FASTTX (reserved, must be 1)
            b.bit(true);                 // FASTUVMC
            b.bit(extended_mv_enabled()); // EXTENDED_MV
            b.bits(c_.dquant ? 1 : 0,2);   // DQUANT: frame-selectable MB quantization
            b.bit(c_.variable_transforms);// VSTRANSFORM
            b.bit(false);                // RES_TRANSTAB (reserved, must be 0)
            b.bit(c_.overlap);           // OVERLAP
            b.bit(false);                // RESYNCMARKER
            b.bit(false);                // RANGERED
            b.bits(static_cast<uint64_t>(std::clamp(c_.max_b_frames,0,7)),3);
            b.bits(1,2);                 // QUANTIZER = frame explicit (PQUANTIZER in each picture)
            b.bit(false);                // FINTERPFLAG
            b.bit(true);                 // RES_RTM_FLAG (reserved, must be 1)
            return b.finish_raw();
        }

        b.bits(3,2);                 // PROFILE = Advanced
        b.bits(static_cast<uint64_t>(std::clamp(c_.advanced_level,0,4)),3); // LEVEL = AP@L3/L4 as required
        b.bits(1,2);                 // COLORDIFF_FORMAT = 4:2:0
        b.bits(0,3);                 // FRMRTQ_POSTPROC
        b.bits(0,5);                 // BITRTQ_POSTPROC
        b.bit(false);                // POSTPROCFLAG
        b.bits(static_cast<uint64_t>(c_.width/2 - 1),12);
        b.bits(static_cast<uint64_t>(c_.height/2 - 1),12);
        // Interlaced sources use Advanced-Profile INTERLACE signaling while
        // retaining progressive picture coding (FCM=0). PULLDOWN is enabled
        // only so each picture can explicitly carry TFF/RFF; RFF remains 0.
        b.bit(c_.interlaced());      // PULLDOWN
        b.bit(c_.interlaced());      // INTERLACE
        b.bit(false);                // TFCNTRFLAG
        b.bit(false);                // FINTERPFLAG
        b.bit(true);                 // reserved advanced-profile bit
        b.bit(false);                // PSF
        // Blu-ray hardware is less uniform than software decoders about using
        // transport PTS as the sole frame-rate authority.  In Blu-ray mode,
        // always carry presentation geometry plus the exact VC-1 frame-rate
        // numerator/denominator code in-band.  Generic Advanced Profile keeps
        // the historical transport-timestamp behavior unless DISPLAY_EXT is
        // needed for a smaller/odd presentation raster.
        const bool signal_bluray_rate=c_.bluray_compat;
        const bool display_ext=signal_bluray_rate ||
            (c_.visible_width()!=c_.width || c_.visible_height()!=c_.height);
        b.bit(display_ext);          // DISPLAY_EXT
        if (display_ext) {
            // DISPLAY_EXT carries presentation geometry; coded dimensions above
            // remain even as required by VC-1 4:2:0 syntax. This permits an odd
            // source/display raster by coding one replicated luma sample at the
            // right and/or bottom edge.
            b.bits(static_cast<uint64_t>(c_.visible_width()-1),14);
            b.bits(static_cast<uint64_t>(c_.visible_height()-1),14);
            b.bit(false);            // ASPECT_RATIO_FLAG
            b.bit(signal_bluray_rate); // FRAMERATE_FLAG
            if (signal_bluray_rate) {
                // Blu-ray-compatible rates all have exact VC-1 table entries.
                // FRAMERATEIND=0 selects FRAMERATENR/FRAMERATEDR.  For an
                // interlaced sequence the signaled value is the frame rate;
                // display fields occur at twice this rate by definition.
                int nr=0,dr=0;
                const auto fps_is=[&](int64_t n,int64_t d) {
                    return c_.fps.num*d == n*c_.fps.den;
                };
                if      (fps_is(24,1))       { nr=1; dr=1; }
                else if (fps_is(24000,1001)) { nr=1; dr=2; }
                else if (fps_is(25,1))       { nr=2; dr=1; }
                else if (fps_is(30000,1001)) { nr=3; dr=2; }
                else if (fps_is(50,1))       { nr=4; dr=1; }
                else if (fps_is(60000,1001)) { nr=5; dr=2; }
                else throw std::logic_error(
                    "Blu-ray frame rate has no VC-1 FRAMERATENR/FRAMERATEDR mapping");
                b.bit(false);        // FRAMERATEIND = table numerator/denominator
                b.bits(static_cast<uint64_t>(nr),8);
                b.bits(static_cast<uint64_t>(dr),4);
            }
            b.bit(false);            // COLOR_FORMAT_FLAG
        }
        b.bit(c_.hrd_enabled);       // HRD_PARAM_FLAG
        if (c_.hrd_enabled) {
            b.bits(1,5);             // HRD_NUM_LEAKY_BUCKETS
            b.bits(c_.hrd_bit_rate_exponent,4);
            b.bits(c_.hrd_buffer_size_exponent,4);
            b.bits(c_.hrd_rate,16);
            b.bits(c_.hrd_buffer,16);
        }
        return bdu(0x0f, b.finish_rbdu());
    }

std::vector<uint8_t> Vc1Encoder::encode_skipped_picture() const {
        if (c_.syntax != StreamSyntax::Advanced)
            throw std::runtime_error("skipped-picture syntax is available only in Advanced Profile");
        BitWriter b;
        if (c_.interlaced()) b.bit(false); // FCM=0: progressive picture syntax in an interlaced sequence.
        b.bits(0x0f,4); // Advanced PTYPE=1111: Skipped P picture.
        if (c_.interlaced()) {
            b.bit(c_.top_field_first()); // TFF (PULLDOWN=1 for interlaced sequences).
            b.bit(false);                // RFF=0: no repeated field.
        }
        // TFCNTRFLAG and panscan are disabled in the sequence/entry-point
        // headers, so no additional skipped-picture syntax is present.
        return bdu(0x0d,b.finish_rbdu());
    }

std::vector<uint8_t> Vc1Encoder::entry_point(uint8_t hrd_fullness) const {
        if (c_.syntax == StreamSyntax::Wmv9Main) return {};
        BitWriter b;
        b.bit(false); // BROKEN_LINK
        b.bit(true);  // CLOSED_ENTRY
        b.bit(false); // PANSCAN_FLAG
        b.bit(false); // REFDIST_FLAG
        b.bit(c_.loop_filter); // LOOPFILTER
        b.bit(true);  // FASTUVMC
        b.bit(extended_mv_enabled()); // EXTENDED_MV
        b.bits(c_.dquant ? 1 : 0,2);  // DQUANT: frame-selectable per-MB quantization.
        b.bit(c_.variable_transforms); // VSTRANSFORM
        b.bit(c_.overlap); // OVERLAP
        b.bits(1,2);  // QUANTIZER = frame explicit (PQUANTIZER in each picture)
        if (c_.hrd_enabled) b.bits(hrd_fullness,8);
        b.bit(false); // CODED_SIZE_FLAG
        if (extended_mv_enabled()) b.bit(false); // EXTENDED_DMV=0
        b.bit(false); // RANGE_MAPY_FLAG
        b.bit(false); // RANGE_MAPUV_FLAG
        return bdu(0x0e, b.finish_rbdu());
    }

} // namespace libvc1
