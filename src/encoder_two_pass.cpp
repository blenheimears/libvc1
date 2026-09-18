#include "encoder_two_pass.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace vc1_twopass {
static uint64_t fnv_byte(uint64_t hash,uint8_t b) { return (hash^b)*1099511628211ull; }
uint64_t source_hash(const std::vector<uint8_t>& y,const std::vector<uint8_t>& u,
                            const std::vector<uint8_t>& v) {
    uint64_t h=14695981039346656037ull;
    for (const auto* p:{&y,&u,&v}) {
        for (const uint8_t b:*p) h=fnv_byte(h,b);
        h=fnv_byte(h,0xff);
    }
    return h;
}
uint64_t signature(const vc1_param_t& p) {
    // The bitstream-affecting first/second-pass options must agree. Bitrate,
    // VBV, baseline weights, threads, output container, and pass mode may vary.
    std::ostringstream s;
    s<<std::setprecision(17)<<p.i_width<<' '<<p.i_height<<' '<<p.i_fps_num<<' '<<p.i_fps_den<<' '
     <<p.i_profile<<' '<<p.i_scan_mode<<' '<<p.b_bluray_compat<<' '<<p.i_keyint_max<<' '
     <<p.i_bframes<<' '<<p.b_intra_only<<' '<<p.b_scene_cut<<' '<<p.f_scene_threshold<<' '
     <<p.f_scene_cut_min_interval<<' '<<p.b_fixed_gop_grid<<' '<<p.b_skip_identical_frames<<' '
     <<p.i_motion_search_range<<' '<<p.i_motion_local_search_range<<' '<<p.i_me_quality<<' '
     <<p.i_long_range_search_mode<<' '<<p.f_distant_match_max_mae<<' '
     <<p.b_fade_compensation<<' '<<p.b_loop_filter<<' '<<p.b_overlap<<' '
     <<p.b_variable_transforms<<' '<<p.b_dquant<<' '<<p.i_trellis<<' '
     <<p.b_adaptive_quality<<' '<<p.f_aq_strength<<' '<<p.i_ac_mode<<' '
     <<p.b_ac_coding<<' '<<p.i_ac_y_table<<' '<<p.i_ac_c_table<<' '
     <<p.i_quantizer_type<<' '<<p.b_halfqp<<' '<<p.f_rc_residual_threshold<<' '
     <<p.f_rc_residual_width<<' '<<p.f_rc_residual_max_q_boost<<' '
     <<p.f_inter_intra_threshold<<' '<<p.b_debug_disable_p_intra<<' '<<p.b_debug_disable_b_intra;
    uint64_t h=14695981039346656037ull;
    const std::string v=s.str();
    for (unsigned char c:v) h=fnv_byte(h,c);
    return h;
}
double FrameStat::cost() const { return std::max(32.0,static_cast<double>(bits))*
        std::pow(std::max(1,q)/2.0,0.55); }

BudgetLedger::BudgetLedger(const Plan& plan,uint64_t bitrate,int fps_num,int fps_den) {
        if (plan.by_display.empty() || plan.gops.empty() || !bitrate || fps_num<=0 || fps_den<=0)
            throw std::runtime_error("invalid two-pass whole-video bitrate budget");
        target_bits=static_cast<double>(bitrate)*fps_den/fps_num*plan.by_display.size();
        if (!std::isfinite(target_bits) || target_bits<=0.0)
            throw std::runtime_error("two-pass bit budget overflow");
        const double per_frame=target_bits/plan.by_display.size();
        nominal.reserve(plan.gops.size());
        for (const auto& g:plan.gops) {
            const double bits=per_frame*g.frames*g.scale;
            if (!(bits>0.0) || !std::isfinite(bits))
                throw std::runtime_error("invalid two-pass GOP budget");
            nominal.push_back(bits);
        }
        reserved_by_gop.assign(nominal.size(),0.0);
        suffix.assign(nominal.size()+1,0.0);
        for (size_t i=nominal.size();i-->0;) suffix[i]=suffix[i+1]+nominal[i];
    }
double BudgetLedger::launch(size_t index) {
        if (index!=launched || launched>=nominal.size())
            throw std::runtime_error("two-pass budget launch order mismatch");
        // All uncommitted jobs are already reserved.  Redistribute any global
        // size error over *all* unlaunched pictures, not the immediately next
        // GOP.  No artificial per-scene ceiling is introduced here.
        const double available=target_bits-actual_bits-reserved_bits;
        const double multiplier=std::max(1e-9,available/std::max(1e-9,suffix[launched]));
        reserved_by_gop[index]=nominal[index]*multiplier;
        reserved_bits+=reserved_by_gop[index];
        ++launched;
        return multiplier;
    }
void BudgetLedger::commit(size_t index,double bits) {
        if (index!=committed || index>=launched || !std::isfinite(bits) || bits<0.0)
            throw std::runtime_error("invalid two-pass actual GOP bitrate accounting");
        reserved_bits=std::max(0.0,reserved_bits-reserved_by_gop[index]);
        actual_bits+=bits;
        ++committed;
    }
double BudgetLedger::deviation_percent() const {
        return 100.0*(actual_bits/target_bits-1.0);
    }
// Input and finalized reconstruction are compact, identically sized 8-bit planes.
// One streaming accumulation: O(1) additional memory and no extra image copies.
double plane_mse(const std::vector<uint8_t>& source,
                        const std::vector<uint8_t>& recon) {
    if (source.empty() || source.size()!=recon.size())
        throw std::runtime_error("two-pass reconstruction plane size mismatch");
    uint64_t sse=0;
    for (size_t i=0;i<source.size();++i) {
        const int d=static_cast<int>(source[i])-static_cast<int>(recon[i]);
        sse+=static_cast<uint64_t>(d*d);
    }
    return static_cast<double>(sse)/static_cast<double>(source.size());
}
double chroma_mse(const std::vector<uint8_t>& source_u,
                         const std::vector<uint8_t>& source_v,
                         const std::vector<uint8_t>& recon_u,
                         const std::vector<uint8_t>& recon_v) {
    // 4:2:0 U/V planes contain the same number of samples.
    return 0.5*(plane_mse(source_u,recon_u)+plane_mse(source_v,recon_v));
}
int type_index(char c) { return c=='I'?0:(c=='P'?1:2); }
Plan read_plan(const std::string& filename,const vc1_param_t& cfg,double strength) {
    std::ifstream f(filename,std::ios::binary);
    if (!f) throw std::runtime_error("cannot open two-pass statistics: "+filename);
    std::string magic; int version=0; uint64_t sig=0;
    if (!(f>>magic>>version>>sig) || magic!="LIBVC1_TWO_PASS" || (version!=1 && version!=2) || sig!=signature(cfg))
        throw std::runtime_error("two-pass statistics version or encoder/source configuration mismatch");
    std::vector<FrameStat> coded;
    const uint64_t macroblocks=static_cast<uint64_t>((cfg.i_width+15)/16)*((cfg.i_height+15)/16);
    std::string tag;
    uint64_t bits_sum=0;
    bool complete=false;
    while (f>>tag) {
        if (tag=="END") {
            uint64_t count=0,bits=0;
            if (!(f>>count>>bits) || count==0 || count!=coded.size() || bits!=bits_sum)
                throw std::runtime_error("incomplete or corrupted two-pass statistics footer");
            std::string extra;
            if (f>>extra) throw std::runtime_error("trailing data after two-pass statistics footer");
            complete=true; break;
        }
        if (tag!="F") throw std::runtime_error("invalid two-pass statistics row");
        FrameStat x;
        if (!(f>>x.coded>>x.display>>x.gop>>x.type>>x.bits>>x.q>>x.complexity>>x.intra>>x.moved>>x.hash) ||
            (version==2 && !(f>>x.mse_y>>x.mse_uv)) ||
            x.coded!=coded.size() || x.display>100000000 || x.gop>10000000 ||
            (x.type!='I' && x.type!='P' && x.type!='B') || x.bits==0 ||
            x.q<1 || x.q>31 || !std::isfinite(x.complexity) || x.complexity<0.0 ||
            x.intra>macroblocks*2 || x.moved>macroblocks*2 ||
            (version==2 && (!std::isfinite(x.mse_y) || !std::isfinite(x.mse_uv) ||
                             x.mse_y<0.0 || x.mse_uv<0.0 ||
                             x.mse_y>65025.0 || x.mse_uv>65025.0)) ||
            x.bits>std::numeric_limits<uint64_t>::max()-bits_sum)
            throw std::runtime_error("invalid or out-of-order two-pass picture statistics");
        bits_sum+=x.bits; coded.push_back(x);
    }
    if (!complete) throw std::runtime_error("two-pass statistics missing END footer (pass 1 did not finish)");
    Plan result;
    result.stats_version=version;
    result.by_display.resize(coded.size());
    std::vector<uint8_t> present(coded.size());
    for (const auto& x:coded) {
        if (x.display>=coded.size() || present[x.display] || x.gop>=coded.size())
            throw std::runtime_error("duplicate or invalid two-pass display/GOP index");
        present[x.display]=1; result.by_display[x.display]=x;
    }
    if (std::find(present.begin(),present.end(),0)!=present.end())
        throw std::runtime_error("two-pass statistics have missing display frames");
    uint64_t next_start=0;
    std::array<double,3> global_cost{},global_count{};
    std::vector<double> gop_cost;
    for (const auto& x:result.by_display) {
        if (x.gop>result.gops.size()) throw std::runtime_error("missing GOP in two-pass statistics");
        if (x.gop==result.gops.size()) {
            result.gops.emplace_back();result.gops.back().start=x.display;
            gop_cost.push_back(0.0);
        }
        auto& g=result.gops[x.gop];
        if (g.start+g.frames!=x.display)
            throw std::runtime_error("noncontiguous GOP in two-pass statistics");
        ++g.frames;
        gop_cost[x.gop]+=x.cost();
        global_cost[type_index(x.type)]+=x.cost();
        global_count[type_index(x.type)]+=1.0;
        ++next_start;
    }
    if (next_start!=result.by_display.size()) throw std::runtime_error("two-pass frame count mismatch");
    // V1 remains readable, and retains its original allocation behavior.
    // V2 adds bounded *relative*, type-conditioned signals so a noisy or
    // intrinsically uncompressible shot cannot monopolize the file's budget.
    // Distortion is measured after in-loop filtering on the full decoded image.
    std::vector<double> effective_cost(result.by_display.size());
    if (version==2) {
        struct Means { double complexity=0, intra=0, motion=0, y=0, uv=0, count=0; };
        std::array<Means,3> means{};
        const double mb=std::max(1.0,static_cast<double>(macroblocks));
        for (const auto& x:result.by_display) {
            auto& m=means[type_index(x.type)];
            m.complexity+=std::log1p(x.complexity);
            m.intra+=std::min(1.0,static_cast<double>(x.intra)/mb);
            m.motion+=std::min(1.0,static_cast<double>(x.moved)/mb);
            m.y+=std::log1p(x.mse_y);
            m.uv+=std::log1p(x.mse_uv);
            ++m.count;
        }
        for (auto& m:means) {
            const double count=std::max(1.0,m.count);
            m.complexity/=count;m.intra/=count;m.motion/=count;
            m.y/=count;m.uv/=count;
        }
        for (size_t i=0;i<result.by_display.size();++i) {
            const auto& x=result.by_display[i];
            const auto& m=means[type_index(x.type)];
            // Geometric, centered demand: 8-bit MSE and measured motion/intra
            // fractions influence the rate demand without replacing encoded size.
            // Use additive floors for the sparse macroblock-class signals.
            const auto logratio=[](double value,double avg,double floor) {
                return std::log((floor+value)/(floor+avg));
            };
            const double adjustment=
                0.12*logratio(std::log1p(x.complexity),m.complexity,0.25)+
                0.12*logratio(std::min(1.0,static_cast<double>(x.intra)/mb),m.intra,0.04)+
                0.10*logratio(std::min(1.0,static_cast<double>(x.moved)/mb),m.motion,0.08)+
                0.20*logratio(std::log1p(x.mse_y),m.y,0.30)+
                0.10*logratio(std::log1p(x.mse_uv),m.uv,0.30);
            effective_cost[i]=x.cost()*std::exp(adjustment);
        }
        // Update both global picture-class and GOP budgets with measured demand.
        gop_cost.assign(result.gops.size(),0.0);
        global_cost.fill(0.0);
        for (size_t i=0;i<result.by_display.size();++i) {
            const auto& x=result.by_display[i];
            gop_cost[x.gop]+=effective_cost[i];
            global_cost[type_index(x.type)]+=effective_cost[i];
        }
    } else {
        for (size_t i=0;i<result.by_display.size();++i)
            effective_cost[i]=result.by_display[i].cost();
    }
    const double mean=std::accumulate(gop_cost.begin(),gop_cost.end(),0.0)/result.by_display.size();
    // Normalize the uncapped, measured GOP demands over the entire film.  The
    // bitrate remains a whole-film target, never an artificial scene ceiling.
    // Blu-ray VBV and the codec's finite quantizer/syntax remain authoritative.
    std::vector<double> relative(result.gops.size());
    for (size_t i=0;i<result.gops.size();++i) {
        auto& g=result.gops[i];
        g.difficulty=gop_cost[i]/g.frames;
        relative[i]=std::pow(g.difficulty/std::max(1.0,mean),0.70);
        g.frame_scale.resize(g.frames,1.0);
        std::array<double,3> local_cost{},local_count{};
        for (size_t j=0;j<g.frames;++j) {
            const auto& x=result.by_display[g.start+j];
            const size_t t=type_index(x.type);
            local_cost[t]+=effective_cost[g.start+j];local_count[t]+=1.0;
        }
        for (size_t t=0;t<3;++t) {
            const double global_average=global_count[t]>0?global_cost[t]/global_count[t]:1.0;
            const double local_average=local_count[t]>0?local_cost[t]/local_count[t]:global_average;
            g.dynamic[t]=std::pow(local_average/std::max(1.0,global_average),0.6*strength);
        }
        for (size_t j=0;j<g.frames;++j) {
            const auto& x=result.by_display[g.start+j];
            const size_t t=type_index(x.type);
            const double average=local_count[t]>0?local_cost[t]/local_count[t]:x.cost();
            g.frame_scale[j]=std::pow(effective_cost[g.start+j]/std::max(1.0,average),0.55);
        }
        {
            // Preserve each picture class's budget exactly, without per-frame
            // floors or ceilings. This also supports legacy V1 statistics.
            for (size_t type=0;type<3;++type) {
                double sum=0.0;
                size_t count=0;
                for (size_t j=0;j<g.frames;++j)
                    if (static_cast<size_t>(type_index(result.by_display[g.start+j].type))==type) {
                        sum+=g.frame_scale[j];++count;
                    }
                if (!count) continue;
                const double normalization=static_cast<double>(count)/sum;
                for (size_t j=0;j<g.frames;++j)
                    if (static_cast<size_t>(type_index(result.by_display[g.start+j].type))==type)
                        g.frame_scale[j]*=normalization;
            }
        }
    }
    double weighted_total=0.0;
    for (size_t i=0;i<result.gops.size();++i)
        weighted_total+=result.gops[i].frames*relative[i];
    const double normalization=static_cast<double>(result.by_display.size())/weighted_total;
    for (size_t i=0;i<result.gops.size();++i)
        result.gops[i].scale=normalization*relative[i];
    return result;
}
} // namespace vc1_twopass
