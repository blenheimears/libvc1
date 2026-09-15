#include "encoder_internal.h"

namespace libvc1 {

HrdValue hrd_floor(uint64_t requested,int exponent_bias) {
    if (!requested) throw std::runtime_error("HRD value must be positive");
    for (int e=0;e<=15;++e) {
        const uint64_t step=1ull << (e+exponent_bias);
        const uint64_t units=requested/step;
        if (units>=1 && units<=65536)
            return HrdValue{static_cast<uint8_t>(e),static_cast<uint16_t>(units-1),units*step};
    }
    throw std::runtime_error("HRD value cannot be represented by VC-1 syntax");
}

RateController::RateController(uint64_t rate_bits,uint64_t buffer_bits,Rational fps)
    : rate_bits_(rate_bits),buffer_bits_(buffer_bits),fps_(fps) {
    if (!rate_bits_ || !buffer_bits_ || fps_.num<=0 || fps_.den<=0)
        throw std::runtime_error("invalid rate-control configuration");
    if (buffer_bits_>std::numeric_limits<uint64_t>::max()/static_cast<uint64_t>(fps_.num) ||
        rate_bits_>std::numeric_limits<uint64_t>::max()/static_cast<uint64_t>(fps_.den))
        throw std::runtime_error("rate-control parameters overflow internal HRD precision");
    buffer_units_=buffer_bits_*static_cast<uint64_t>(fps_.num);
    refill_units_=rate_bits_*static_cast<uint64_t>(fps_.den);
    midpoint_units_=buffer_units_/2;
    initial_fullness_units_=std::min(buffer_units_,midpoint_units_+refill_units_);
    if (!initial_fullness_units_) throw std::runtime_error("invalid zero initial HRD fullness");
}

double RateController::fullness() const { const uint64_t u=first_picture_?initial_fullness_units_:after_units_; return static_cast<double>(u)/fps_.num; }
double RateController::frame_budget() const { return static_cast<double>(refill_units_)/fps_.num; }
double RateController::max_picture_bits() const { return static_cast<double>(pre_removal_units()/static_cast<uint64_t>(fps_.num)); }
uint64_t RateController::rate_bits() const { return rate_bits_; }
uint64_t RateController::buffer_bits() const { return buffer_bits_; }
uint64_t RateController::initial_fullness_bits_floor() const { return initial_fullness_units_/static_cast<uint64_t>(fps_.num); }
uint8_t RateController::fullness_code() const {
    const uint64_t pre=pre_removal_units(); if (!pre) return 0;
    if (pre>std::numeric_limits<uint64_t>::max()/256ull) throw std::runtime_error("HRD fullness calculation overflow");
    uint64_t units=(pre*256ull+buffer_units_-1ull)/buffer_units_; units=std::clamp<uint64_t>(units,1,256); return static_cast<uint8_t>(units-1);
}
void RateController::commit(uint64_t picture_bits) {
    if (picture_bits>std::numeric_limits<uint64_t>::max()/static_cast<uint64_t>(fps_.num)) throw std::runtime_error("picture size overflows internal HRD precision");
    if (!first_picture_ && refill_units_>buffer_units_-after_units_) ++transmission_pauses_;
    const uint64_t pre=pre_removal_units(); const uint64_t picture_units=picture_bits*static_cast<uint64_t>(fps_.num);
    if (picture_units>pre) { ++underflows_; throw std::runtime_error("rate control cannot satisfy the selected HRD rate/buffer even at maximum quantizer"); }
    after_units_=pre-picture_units; min_after_units_=std::min(min_after_units_,after_units_); max_pre_units_=std::max(max_pre_units_,pre);
    total_bits_+=picture_bits; ++frames_; first_picture_=false;
}
uint64_t RateController::transmission_pauses() const { return transmission_pauses_; }
uint64_t RateController::underflows() const { return underflows_; }
uint64_t RateController::total_bits() const { return total_bits_; }
uint64_t RateController::frames() const { return frames_; }
double RateController::min_after_bits() const { return frames_?static_cast<double>(min_after_units_)/fps_.num:0.0; }
double RateController::max_pre_bits() const { return frames_?static_cast<double>(max_pre_units_)/fps_.num:0.0; }
double RateController::average_bitrate() const { return frames_?static_cast<double>(total_bits_)*fps_.num/(static_cast<double>(frames_)*fps_.den):0.0; }
uint64_t RateController::pre_removal_units() const {
    if (first_picture_) return initial_fullness_units_;
    if (after_units_>buffer_units_) throw std::runtime_error("internal HRD fullness exceeds buffer");
    if (refill_units_>buffer_units_-after_units_) return buffer_units_;
    return after_units_+refill_units_;
}

} // namespace libvc1
