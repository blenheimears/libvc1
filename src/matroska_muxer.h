#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

bool libvc1_matroska_available() noexcept;

class MatroskaVideoMuxer {
public:
    MatroskaVideoMuxer(const std::string& path,
                       int coded_width,
                       int coded_height,
                       int display_width,
                       int display_height,
                       int fps_num,
                       int fps_den,
                       const std::array<uint8_t,4>& fourcc,
                       const std::vector<uint8_t>& codec_init);
    ~MatroskaVideoMuxer();

    MatroskaVideoMuxer(const MatroskaVideoMuxer&) = delete;
    MatroskaVideoMuxer& operator=(const MatroskaVideoMuxer&) = delete;

    void write_video_frame(const std::vector<uint8_t>& payload,
                           uint64_t dts_ns,
                           bool keyframe);
    void finalize();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
