#include "encoder_internal.h"

namespace libvc1 {

Vc1Encoder::IntegerMotionResult Vc1Encoder::integer_motion_local(
    const Frame& cur,const Frame& ref,int mx,int my,
    const PredictorInfo& pi,const GlobalMotionResult& global,int range) const {
    if (c_.me_algorithm==VC1_ME_ALGORITHM_DIAMOND)
        return integer_motion_diamond(cur,ref,mx,my,pi,global,range);
    return integer_motion_umh(cur,ref,mx,my,pi,global,range);
}

Vc1Encoder::IntegerMotionResult Vc1Encoder::integer_motion_diamond(
    const Frame& cur,const Frame& ref,int mx,int my,
    const PredictorInfo& pi,const GlobalMotionResult& global,int range) const {
    SpeedProfileScope speed_scope(c_.speed_profiler,SpeedProfileOp::MotionLocalSearch);
    // A fast small-diamond search: at most four new full-resolution SADs per
    // iteration, or three after moving (the previous center was already
    // evaluated). No heap allocations, LUT initialization, sampled-SAD aliasing,
    // large UMH patterns, or exhaustive poor-match rescue. mb_sad() already
    // dispatches to the selected architecture's SIMD block-SAD implementation
    // for interior macroblocks and enforces cropped-reference legality.
    struct Candidate { int dx,dy; uint64_t sad; };
    const auto better=[](const Candidate& a,const Candidate& b) {
        if (a.sad!=b.sad) return a.sad<b.sad;
        const int am=std::abs(a.dx)+std::abs(a.dy);
        const int bm=std::abs(b.dx)+std::abs(b.dy);
        if (am!=bm) return am<bm;
        if (a.dy!=b.dy) return a.dy<b.dy;
        return a.dx<b.dx;
    };
    const auto clamp=[range](int v) { return std::clamp(v,-range,range); };
    Candidate best{0,0,mb_sad(cur,ref,mx,my,0,0,1)};
    // A perfect zero vector is already the minimum SAD and minimum MV rate
    // proxy; don't spend time evaluating other seeds or diamond neighbors.
    if (best.sad==0) return {0,0,0};
    int seen_x[5]={0},seen_y[5]={0},seen_count=1;
    const auto consider_seed=[&](int dx,int dy) {
        dx=clamp(dx);dy=clamp(dy);
        for (int i=0;i<seen_count;++i)
            if (dx==seen_x[i] && dy==seen_y[i]) return;
        seen_x[seen_count]=dx;seen_y[seen_count]=dy;++seen_count;
        const Candidate c{dx,dy,mb_sad(cur,ref,mx,my,dx,dy,1)};
        if (better(c,best)) best=c;
    };
    consider_seed(round_qpel_to_pixel(pi.pre.xq),round_qpel_to_pixel(pi.pre.yq));
    consider_seed(global.dx,global.dy);
    if (pi.a_valid) consider_seed(round_qpel_to_pixel(pi.a.xq),round_qpel_to_pixel(pi.a.yq));
    if (pi.c_valid) consider_seed(round_qpel_to_pixel(pi.c.xq),round_qpel_to_pixel(pi.c.yq));

    // Every accepted move strictly decreases the total (SAD, Manhattan, y, x)
    // ordering. The finite square therefore terminates even on flat textures.
    // The geometric guard also bounds work if the candidate law changes later.
    int previous_x=best.dx,previous_y=best.dy;
    bool have_previous=false;
    constexpr int offsets[4][2]={{-1,0},{1,0},{0,-1},{0,1}};
    for (int step=0;step<=4*range+1;++step) {
        const Candidate center=best;
        Candidate winner=center;
        for (const auto& offset:offsets) {
            const int dx=center.dx+offset[0],dy=center.dy+offset[1];
            if (dx < -range || dx > range || dy < -range || dy > range) continue;
            if (have_previous && dx==previous_x && dy==previous_y) continue;
            const Candidate c{dx,dy,mb_sad(cur,ref,mx,my,dx,dy,1)};
            if (better(c,winner)) winner=c;
        }
        if (winner.dx==center.dx && winner.dy==center.dy) break;
        previous_x=center.dx;previous_y=center.dy;have_previous=true;
        best=winner;
        if (best.sad==0) break; // no SAD improvement is possible; seeds include zero.
    }
    return {best.dx,best.dy,best.sad};
}

} // namespace libvc1
