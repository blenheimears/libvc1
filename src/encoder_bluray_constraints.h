#pragma once
// Internal Blu-ray/HRD option validation and shared rate/keyframe resolution.
#include <libvc1.h>
#include <cstdint>

uint64_t resolved_peak_bitrate(const vc1_param_t& p);
uint64_t bluray_keyint_limit(const vc1_param_t& p);
void validate_bluray_compat(const vc1_param_t& p,int advanced_level);
