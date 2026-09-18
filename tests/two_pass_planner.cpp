#include "encoder_two_pass.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

static void check(bool ok,const char* why) { if (!ok) throw std::runtime_error(why); }
int main() {
    try {
        vc1_param_t p{};
        p.i_width=32;p.i_height=32;p.i_fps_num=24;p.i_fps_den=1;
        const auto filename=std::filesystem::temp_directory_path()/"libvc1-two-pass-planner-regression.stats";
        struct Cleanup { std::filesystem::path p; ~Cleanup() { std::error_code ec;std::filesystem::remove(p,ec); } } cleanup{filename};
        const char* types="IPBP";
        const auto write=[&](int version,int signal,int modified_gop) {
            std::ofstream f(filename,std::ios::binary|std::ios::trunc);
            f<<"LIBVC1_TWO_PASS "<<version<<' '<<vc1_twopass::signature(p)<<'\n';
            for (int i=0;i<16;++i) {
                const int g=i/4;const bool mod=g==modified_gop;
                const int intra=i%4==0?4:(mod && signal==2?4:1);
                const int moved=i%4==0?0:(mod && signal==3?4:1);
                const double complexity=mod && signal==1?10000.0:100.0;
                const double y=mod && signal==4?65025.0:10.0;
                const double uv=mod && signal==5?65025.0:10.0;
                f<<"F "<<i<<' '<<i<<' '<<g<<' '<<types[i%4]
                 <<" 10000 8 "<<complexity<<' '<<intra<<' '<<moved<<' '<<i+123;
                if (version==2)f<<' '<<y<<' '<<uv;
                f<<'\n';
            }
            f<<"END 16 160000\n";
        };
        write(2,0,0);
        const auto baseline=vc1_twopass::read_plan(filename.string(),p,1.0);
        check(baseline.stats_version==2 && baseline.gops.size()==4,"v2 format");
        for (const auto& g:baseline.gops) check(std::abs(g.scale-1.0)<1e-8,"uniform video must get uniform GOP budget");
        for (int signal=1;signal<=5;++signal) {
            write(2,signal,1);
            const auto changed=vc1_twopass::read_plan(filename.string(),p,1.0);
            check(changed.gops[1].scale>baseline.gops[1].scale+0.005,"signal did not change GOP demand");
            double total=0;
            for (const auto& g:changed.gops) {
                check(g.scale>0.0 && std::isfinite(g.scale),"finite GOP demand");
                total+=g.frames*g.scale;
                for (size_t type=0;type<3;++type) {
                    double sum=0;size_t count=0;
                    for (size_t j=0;j<g.frames;++j)
                        if (static_cast<size_t>(vc1_twopass::type_index(types[j%4]))==type) {
                            sum+=g.frame_scale[j];++count;
                            check(g.frame_scale[j]>0.0 && std::isfinite(g.frame_scale[j]),"finite picture demand");
                        }
                    if (count)check(std::abs(sum-count)<1e-8,"per-type picture budget not conserved");
                }
            }
            check(std::abs(total-16.0)<1e-8,"whole-video GOP budget not conserved");
        }
        {
            // The size ledger never caps an individual GOP: if an early GOP
            // costs twice its planned share, *all* future unscheduled GOPs
            // repay the difference without charging just the next GOP.
            vc1_twopass::BudgetLedger ledger(baseline,24000,24,1);
            check(std::abs(ledger.target_bits-16000.0)<1e-9,"whole-video target");
            const double first=ledger.launch(0);
            check(std::abs(first-1.0)<1e-9,"initial budget multiplier");
            ledger.commit(0,8000.0);
            check(std::abs(ledger.launch(1)-2.0/3.0)<1e-9,"size error was not spread over future GOPs");
            ledger.commit(1,2000.0);
            check(std::abs(ledger.launch(2)-0.75)<1e-9,"remaining budget reconciliation");
            check(std::abs(ledger.launch(3)-0.75)<1e-9,"concurrent GOP reservations double spent budget");
            ledger.commit(2,3000.0);
            ledger.commit(3,3000.0);
            check(std::abs(ledger.actual_bits-ledger.target_bits)<1e-9,"final size reconciliation");
            check(std::abs(ledger.deviation_percent())<1e-9,"final size error");
        }
        write(2,4,1);
        {
            // A GOP needing 100x as many bits must not be clipped at the old
            // 1.60x multiplier. Normalize the resulting film-wide target.
            std::ofstream f(filename,std::ios::binary|std::ios::trunc);
            f<<"LIBVC1_TWO_PASS 2 "<<vc1_twopass::signature(p)<<'\n';
            uint64_t bits=0;
            for (int i=0;i<16;++i) {
                const int count=(i>=4 && i<8)?1000000:10000;
                f<<"F "<<i<<' '<<i<<' '<<i/4<<' '<<types[i%4]<<' '
                 <<count<<" 8 100 1 1 "<<i+123<<" 10 10\n";
                bits+=count;
            }
            f<<"END 16 "<<bits<<'\n';f.close();
            const auto spike=vc1_twopass::read_plan(filename.string(),p,1.0);
            check(spike.gops[1].scale>1.60,"high-complexity GOP is still artificially capped");
            double sum=0.0;for (const auto& g:spike.gops)sum+=g.frames*g.scale;
            check(std::abs(sum-16.0)<1e-8,"uncapped target conservation");
        }
        write(2,4,1);
        const auto no_dynamic=vc1_twopass::read_plan(filename.string(),p,0.0);
        for (const auto& g:no_dynamic.gops)
            for (const auto d:g.dynamic)check(std::abs(d-1.0)<1e-10,"disable dynamic weights");
        write(1,0,1);
        const auto legacy=vc1_twopass::read_plan(filename.string(),p,1.0);
        check(legacy.stats_version==1,"v1 compatibility");
        for (const auto& g:legacy.gops)check(std::abs(g.scale-1.0)<1e-8,"v1 formula changed");
        write(2,4,1);
        {std::ofstream f(filename,std::ios::app);f<<"GARBAGE\n";}
        bool rejected=false;
        try { vc1_twopass::read_plan(filename.string(),p,1.0); }
        catch(const std::runtime_error&) {rejected=true;}
        check(rejected,"trailing data must be rejected");
        std::cout<<"two-pass measured-demand planner passed (complexity/intra/motion/Y/UV, v1 fallback, uncapped conservation)\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr<<"two-pass planner FAIL: "<<e.what()<<'\n';
        return 1;
    }
}
