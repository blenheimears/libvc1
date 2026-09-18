#pragma once
// Internal two-pass data types and function declarations; implementation lives
// in encoder_two_pass.cpp. Source frames and bitstreams are not cached here.
#include <libvc1.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vc1_twopass {
uint64_t source_hash(const std::vector<uint8_t>& y,const std::vector<uint8_t>& u,
                     const std::vector<uint8_t>& v);
uint64_t signature(const vc1_param_t& p);
struct FrameStat {
    uint64_t coded=0,display=0,gop=0,bits=0,intra=0,moved=0,hash=0;
    char type='I';
    int q=2;
    double complexity=0.0;
    double mse_y=0.0,mse_uv=0.0; // pass-1 source-vs-final-reconstruction per-sample MSE
    double cost() const;
};
struct GopPlan {
    uint64_t start=0;
    size_t frames=0;
    double scale=1.0,difficulty=1.0;
    std::array<double,3> dynamic{{1.0,1.0,1.0}};
    std::vector<double> frame_scale;
};
struct Plan {
    std::vector<FrameStat> by_display;
    std::vector<GopPlan> gops;
    int stats_version=1;
};
// Allocate against a whole-video elementary-stream bit budget.  Already
// dispatched GOPs reserve their planned bits so one slow worker cannot make
// the coordinator promise the same remaining bits to multiple other workers.
// Committed (actual) sizes replace reservations in strict GOP order.  Only
// FUTURE plans are rescaled; already returned access units are immutable.
struct BudgetLedger {
    std::vector<double> nominal, reserved_by_gop, suffix;
    double target_bits=0.0, actual_bits=0.0, reserved_bits=0.0;
    size_t launched=0, committed=0;
    BudgetLedger(const Plan& plan,uint64_t bitrate,int fps_num,int fps_den);
    double launch(size_t index);
    void commit(size_t index,double bits);
    double deviation_percent() const;
};
// First-pass distortion, picture classification and stats-file parser/planner.
double plane_mse(const std::vector<uint8_t>& source,const std::vector<uint8_t>& recon);
double chroma_mse(const std::vector<uint8_t>& source_u,const std::vector<uint8_t>& source_v,
                  const std::vector<uint8_t>& recon_u,const std::vector<uint8_t>& recon_v);
int type_index(char c);
Plan read_plan(const std::string& filename,const vc1_param_t& cfg,double strength);
} // namespace vc1_twopass
