#include "encoder_internal.h"
#include <cstdint>
#include <cstdio>
#include <limits>

using namespace libvc1;

static Frame gradient(int width,int height,bool horizontal) {
    Frame f;f.width=width;f.height=height;
    f.y.resize(static_cast<size_t>(width)*height);
    f.u.assign(static_cast<size_t>(width/2)*(height/2),128);
    f.v=f.u;
    for (int y=0;y<height;++y) for (int x=0;x<width;++x)
        f.y[static_cast<size_t>(y)*width+x]=static_cast<uint8_t>(20+3*(horizontal?x:y));
    return f;
}

static bool test_shift(int sx,int sy,bool horizontal) {
    EncoderConfig cfg;
    cfg.width=64;cfg.height=48;
    cfg.motion_search_range=12;cfg.motion_local_search_range=12;
    cfg.me_algorithm=VC1_ME_ALGORITHM_DIAMOND;
    Vc1Encoder enc(cfg);
    const Frame ref=gradient(cfg.width,cfg.height,horizontal);
    Frame cur=ref;
    for (int y=0;y<cfg.height;++y) for (int x=0;x<cfg.width;++x)
        cur.y[static_cast<size_t>(y)*cfg.width+x]=ref.y[static_cast<size_t>(std::clamp(y+sy,0,cfg.height-1))*cfg.width+std::clamp(x+sx,0,cfg.width-1)];
    Vc1Encoder::PredictorInfo pred{};
    const Vc1Encoder::GlobalMotionResult global{};
    const auto a=enc.integer_motion_diamond(cur,ref,1,1,pred,global,12);
    const auto b=enc.integer_motion_local(cur,ref,1,1,pred,global,12);
    if (a.dx!=sx || a.dy!=sy || a.full_sad!=0 ||
        b.dx!=a.dx || b.dy!=a.dy || b.full_sad!=a.full_sad) {
        std::fprintf(stderr,"diamond: shift (%d,%d) yielded (%d,%d) SAD=%llu\n",
                     sx,sy,a.dx,a.dy,(unsigned long long)a.full_sad);
        return false;
    }
    // Repeated calls must be deterministic and avoid stateful search caches.
    const auto again=enc.integer_motion_diamond(cur,ref,1,1,pred,global,12);
    return again.dx==a.dx && again.dy==a.dy && again.full_sad==a.full_sad;
}

int main() {
    if (!test_shift(4,0,true) || !test_shift(0,-3,false)) return 1;
    EncoderConfig cfg;cfg.width=64;cfg.height=48;
    cfg.display_height=42;cfg.motion_search_range=8;cfg.motion_local_search_range=8;
    cfg.me_algorithm=VC1_ME_ALGORITHM_DIAMOND;
    Vc1Encoder enc(cfg);
    const Frame f=gradient(64,48,true);
    Vc1Encoder::PredictorInfo p{};
    p.pre={0,16}; // illegal +4 vertical displacement into cropped padding
    const Vc1Encoder::GlobalMotionResult g{0,4,0};
    const auto edge=enc.integer_motion_diamond(f,f,1,2,p,g,8);
    if (edge.dx!=0 || edge.dy!=0 || edge.full_sad!=0 ||
        enc.mb_sad(f,f,1,2,0,4,1)<std::numeric_limits<uint64_t>::max()/8) {
        std::fprintf(stderr,"diamond: illegal edge predictor was selected\n");return 2;
    }
    const auto stationary=enc.integer_motion_diamond(f,f,1,1,p,g,0);
    if (stationary.dx || stationary.dy || stationary.full_sad) return 3;
    vc1_param_t params{};
    if (vc1_param_default(&params)!=0 ||
        params.i_me_algorithm!=VC1_ME_ALGORITHM_UMH ||
        params.i_api_version!=LIBVC1_API_VERSION) return 4;
    std::puts("diamond: translation, search dispatch, deterministic zero, cropped edges and API defaults passed");
    return 0;
}
