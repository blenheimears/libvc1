#pragma once
#include <libvc1.h>
#include <stdexcept>
#include <string>
#include <utility>

namespace libvc1 {

inline vc1_param_t default_params() {
    vc1_param_t p{};
    if (vc1_param_default(&p) < 0)
        throw std::runtime_error("vc1_param_default failed");
    return p;
}

class Encoder {
public:
    explicit Encoder(const vc1_param_t& p) : enc_(vc1_encoder_open(&p)) {
        if (!enc_) {
            const char* message = vc1_encoder_last_error(nullptr);
            throw std::runtime_error(message && *message ? message : "vc1_encoder_open failed");
        }
    }
    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;
    Encoder(Encoder&& o) noexcept : enc_(std::exchange(o.enc_, nullptr)) {}
    Encoder& operator=(Encoder&& o) noexcept {
        if (this != &o) {
            if (enc_) vc1_encoder_close(enc_);
            enc_ = std::exchange(o.enc_, nullptr);
        }
        return *this;
    }
    ~Encoder() { if (enc_) vc1_encoder_close(enc_); }

    vc1_t* get() const noexcept { return enc_; }

    int headers(vc1_au_t** au, int* count) {
        const int r = vc1_encoder_headers(enc_, au, count);
        if (r < 0) throw_last_error();
        return r;
    }

    int encode(vc1_au_t** au, int* count, vc1_picture_t* out, const vc1_picture_t* in) {
        const int r = vc1_encoder_encode(enc_, au, count, out, in);
        if (r < 0) throw_last_error();
        return r;
    }

    int delayed_frames() const noexcept { return vc1_encoder_delayed_frames(enc_); }

    vc1_param_t parameters() const {
        vc1_param_t p{};
        if (vc1_encoder_parameters(enc_, &p) < 0) throw_last_error();
        return p;
    }

    vc1_stats_t stats() const {
        vc1_stats_t s{};
        if (vc1_encoder_stats(enc_, &s) < 0) throw_last_error();
        return s;
    }

    const char* last_error() const noexcept { return vc1_encoder_last_error(enc_); }

private:
    [[noreturn]] void throw_last_error() const {
        const char* message = vc1_encoder_last_error(enc_);
        throw std::runtime_error(message && *message ? message : "libvc1 encoder operation failed");
    }

    vc1_t* enc_ = nullptr;
};

} // namespace libvc1
