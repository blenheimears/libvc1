#include "matroska_muxer.h"
#include "libvc1.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

#ifdef LIBVC1_HAVE_MATROSKA
#include <ebml/EbmlHead.h>
#include <ebml/EbmlMaster.h>
#include <ebml/EbmlSubHead.h>
#include <ebml/EbmlUnicodeString.h>
#include <ebml/StdIOCallback.h>
#include <matroska/KaxBlock.h>
#include <matroska/KaxBlockData.h>
#include <matroska/KaxCluster.h>
#include <matroska/KaxCues.h>
#include <matroska/KaxContexts.h>
#include <matroska/KaxInfoData.h>
#include <matroska/KaxSegment.h>
#include <matroska/KaxTracks.h>
#include <matroska/KaxTrackVideo.h>
#endif

#ifdef LIBVC1_HAVE_MATROSKA
namespace {

static void append_le16(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(static_cast<uint8_t>(v));
    out.push_back(static_cast<uint8_t>(v >> 8));
}
static void append_le32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v >> 16));
    out.push_back(static_cast<uint8_t>(v >> 24));
}

static std::vector<uint8_t> make_vfw_codec_private(int width,
                                                    int height,
                                                    const std::array<uint8_t,4>& fourcc,
                                                    const std::vector<uint8_t>& codec_init) {
    if (width <= 0 || height <= 0)
        throw std::runtime_error("invalid Matroska video dimensions");
    if (codec_init.size() > std::numeric_limits<uint32_t>::max() - 40u)
        throw std::runtime_error("Matroska codec private data is too large");

    std::vector<uint8_t> out;
    out.reserve(40 + codec_init.size());
    // Matroska V_MS/VFW/FOURCC stores a little-endian BITMAPINFOHEADER followed
    // by codec initialization bytes.  Existing WVC1 Matroska files written by
    // mkvmerge use biSize to cover the complete format block, not just the
    // 40-byte fixed header, so mirror that convention for Windows decoders.
    append_le32(out, static_cast<uint32_t>(40u + codec_init.size()));
    append_le32(out, static_cast<uint32_t>(width));
    append_le32(out, static_cast<uint32_t>(height));
    append_le16(out, 1);   // biPlanes
    append_le16(out, 24);  // conventional VfW compressed-video bit count
    out.insert(out.end(), fourcc.begin(), fourcc.end());
    append_le32(out, 0); // biSizeImage
    append_le32(out, 0); // biXPelsPerMeter
    append_le32(out, 0); // biYPelsPerMeter
    append_le32(out, 0); // biClrUsed
    append_le32(out, 0); // biClrImportant
    out.insert(out.end(), codec_init.begin(), codec_init.end());
    return out;
}

} // namespace
#endif

#ifdef LIBVC1_HAVE_MATROSKA

using namespace libebml;
using namespace libmatroska;

struct MatroskaVideoMuxer::Impl {
    static constexpr uint64_t kTimecodeScale = 1000000; // 1 ms, mkvmerge-compatible scale

    StdIOCallback out;
    KaxSegment segment;
    KaxTrackEntry* track = nullptr;
    uint64_t frame_duration_ns = 0;
    filepos_t segment_data_start = 0;
    bool finalized = false;

    Impl(const std::string& path,
         int coded_width,
         int coded_height,
         int display_width,
         int display_height,
         int fps_num,
         int fps_den,
         const std::array<uint8_t,4>& fourcc,
         const std::vector<uint8_t>& codec_init)
        : out(path.c_str(), MODE_CREATE) {
        if (fps_num <= 0 || fps_den <= 0)
            throw std::runtime_error("invalid Matroska frame rate");
        if (coded_width <= 0 || coded_height <= 0 || display_width <= 0 || display_height <= 0 ||
            display_width > coded_width || display_height > coded_height)
            throw std::runtime_error("invalid Matroska coded/display dimensions");
        const long double dur = 1000000000.0L * static_cast<long double>(fps_den) /
                                static_cast<long double>(fps_num);
        frame_duration_ns = static_cast<uint64_t>(dur + 0.5L);
        if (!frame_duration_ns)
            throw std::runtime_error("Matroska frame duration underflow");

        EbmlHead head;
        GetChild<EVersion>(head).SetValue(1);
        GetChild<EReadVersion>(head).SetValue(1);
        GetChild<EMaxIdLength>(head).SetValue(4);
        GetChild<EMaxSizeLength>(head).SetValue(8);
        GetChild<EDocType>(head).SetValue("matroska");
        GetChild<EDocTypeVersion>(head).SetValue(4);
        GetChild<EDocTypeReadVersion>(head).SetValue(2);
        head.Render(out, false);

        // libebml writes the initial KaxSegment header using the segment's
        // provisional size.  Keep the standard five-byte size field reserved
        // and remember the payload start so finish() can overwrite that header
        // with the exact final Segment length, as libmatroska's mux example does.
        segment.WriteHead(out, 5, false);
        segment_data_start = out.getFilePointer();

        auto& info = GetChild<KaxInfo>(segment);
        GetChild<KaxTimecodeScale>(info).SetValue(kTimecodeScale);
        UTFstring app;
        app.SetUTF8(std::string("libvc1 ") + LIBVC1_VERSION_STRING);
        *static_cast<EbmlUnicodeString*>(&GetChild<KaxMuxingApp>(info)) = app;
        *static_cast<EbmlUnicodeString*>(&GetChild<KaxWritingApp>(info)) = app;
        info.Render(out, false);

        auto& tracks = GetChild<KaxTracks>(segment);
        track = &AddNewChild<KaxTrackEntry>(tracks);
        GetChild<KaxTrackNumber>(*track).SetValue(1);
        GetChild<KaxTrackUID>(*track).SetValue(1);
        GetChild<KaxTrackType>(*track).SetValue(track_video);
        GetChild<KaxTrackFlagLacing>(*track).SetValue(0);
        GetChild<KaxTrackDefaultDuration>(*track).SetValue(frame_duration_ns);
        GetChild<KaxCodecID>(*track).SetValue("V_MS/VFW/FOURCC");

        const auto private_data = make_vfw_codec_private(coded_width, coded_height, fourcc, codec_init);
        auto& cp = GetChild<KaxCodecPrivate>(*track);
        cp.CopyBuffer(private_data.data(), static_cast<uint32_t>(private_data.size()));

        auto& video = GetChild<KaxTrackVideo>(*track);
        GetChild<KaxVideoPixelWidth>(video).SetValue(static_cast<uint64_t>(coded_width));
        GetChild<KaxVideoPixelHeight>(video).SetValue(static_cast<uint64_t>(coded_height));
        GetChild<KaxVideoDisplayWidth>(video).SetValue(static_cast<uint64_t>(display_width));
        GetChild<KaxVideoDisplayHeight>(video).SetValue(static_cast<uint64_t>(display_height));
        tracks.Render(out, false);

        track->SetGlobalTimecodeScale(kTimecodeScale);
    }

    void write(const std::vector<uint8_t>& payload, uint64_t dts_ns, bool keyframe) {
        if (finalized)
            throw std::runtime_error("attempt to write a finalized Matroska file");
        if (payload.empty())
            throw std::runtime_error("attempt to write an empty Matroska video frame");

        // One coded frame per finite cluster deliberately keeps the writer simple
        // and makes every Block timecode zero relative to its Cluster.  This avoids
        // 16-bit local-timecode overflow and retains the encoder's coded order.
        KaxCluster cluster;
        cluster.SetParent(segment);
        cluster.InitTimecode(dts_ns / kTimecodeScale, kTimecodeScale);

        // KaxInternalBlock stores a DataBuffer pointer until the Cluster is
        // rendered.  Use DataBuffer's internal-copy mode so the encoder's
        // std::vector may be released immediately, and let KaxCluster::AddFrame
        // create/parent the BlockGroup according to libmatroska's own API.
        auto* buffer = new DataBuffer(
            reinterpret_cast<binary*>(const_cast<uint8_t*>(payload.data())),
            static_cast<uint32_t>(payload.size()), nullptr, true);
        KaxBlockGroup* group = nullptr;
        try {
            (void)cluster.AddFrame(*track, dts_ns, *buffer, group, LACING_NONE);
        } catch (...) {
            buffer->FreeBuffer(*buffer);
            delete buffer;
            throw;
        }
        if (group == nullptr) {
            buffer->FreeBuffer(*buffer);
            delete buffer;
            throw std::runtime_error("libmatroska failed to create a VC-1 BlockGroup");
        }
        group->SetBlockDuration(frame_duration_ns);

        // With BlockGroup, the absence of ReferenceBlock identifies a random-
        // access block.  Non-key pictures get a conservative preceding reference;
        // VC-1's actual reference topology remains in the compressed bitstream.
        if (!keyframe) {
            auto& ref = AddNewChild<KaxReferenceBlock>(*group);
            // KaxReferenceBlock has extra state beyond its EbmlSInteger value.
            // Assigning through the base class leaves bTimecodeSet false, causing
            // UpdateSize() to dereference a null RefdBlock in libmatroska 1.7.1.
            // We already know the relative reference timestamp, so use the public
            // setter that records both the value and the initialized-state flag.
            ref.SetReferencedTimecode(-static_cast<int64_t>(
                std::max<uint64_t>(1, frame_duration_ns / kTimecodeScale)));
        }

        // libmatroska 1.7.x requires a KaxCues reference even when this muxer
        // does not emit a Cues element.  With no registered cue points,
        // PositionSet() is a no-op.  Render defaults normally (false), matching
        // libmatroska's mux example, then explicitly release the frame buffers.
        KaxCues unused_cues;
        unused_cues.SetGlobalTimecodeScale(kTimecodeScale);
        cluster.Render(out, unused_cues, false);
        cluster.ReleaseFrames();
    }

    void finish() {
        if (finalized) return;
        const filepos_t end = out.getFilePointer();
        if (end < segment_data_start)
            throw std::runtime_error("Matroska output position moved before the Segment payload");
        const uint64_t segment_size = static_cast<uint64_t>(end - segment_data_start);
        if (!segment.ForceSize(segment_size))
            throw std::runtime_error("Matroska Segment is too large for the reserved size field");
        segment.OverwriteHead(out);
        out.close();
        finalized = true;
    }

    ~Impl() {
        try { finish(); } catch (...) {}
    }
};

bool libvc1_matroska_available() noexcept { return true; }

MatroskaVideoMuxer::MatroskaVideoMuxer(const std::string& path,
                                       int coded_width,
                                       int coded_height,
                                       int display_width,
                                       int display_height,
                                       int fps_num,
                                       int fps_den,
                                       const std::array<uint8_t,4>& fourcc,
                                       const std::vector<uint8_t>& codec_init)
    : impl_(std::make_unique<Impl>(path, coded_width, coded_height, display_width, display_height, fps_num, fps_den, fourcc, codec_init)) {}

MatroskaVideoMuxer::~MatroskaVideoMuxer() = default;

void MatroskaVideoMuxer::write_video_frame(const std::vector<uint8_t>& payload,
                                           uint64_t dts_ns,
                                           bool keyframe) {
    impl_->write(payload, dts_ns, keyframe);
}

void MatroskaVideoMuxer::finalize() { impl_->finish(); }

#else

struct MatroskaVideoMuxer::Impl {};

bool libvc1_matroska_available() noexcept { return false; }

MatroskaVideoMuxer::MatroskaVideoMuxer(const std::string&,
                                       int,
                                       int,
                                       int,
                                       int,
                                       int,
                                       int,
                                       const std::array<uint8_t,4>&,
                                       const std::vector<uint8_t>&) {
    throw std::runtime_error("Matroska output was not built: install libmatroska and libebml development files, then rebuild libvc1");
}

MatroskaVideoMuxer::~MatroskaVideoMuxer() = default;
void MatroskaVideoMuxer::write_video_frame(const std::vector<uint8_t>&, uint64_t, bool) {
    throw std::runtime_error("Matroska output is unavailable in this build");
}
void MatroskaVideoMuxer::finalize() {}

#endif
