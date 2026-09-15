#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

p = argparse.ArgumentParser(description="Check per-frame PSNR between two raw yuv420p sequences")
p.add_argument("source")
p.add_argument("decoded")
p.add_argument("--width", type=int, required=True)
p.add_argument("--height", type=int, required=True)
p.add_argument("--frames", type=int, required=True)
p.add_argument("--min-psnr", type=float, default=None)
p.add_argument("--keyframe", action="append", type=int, default=[], help="1-based frame number that must meet --keyframe-min")
p.add_argument("--keyframe-min", type=float, default=None)
args = p.parse_args()

if args.width <= 0 or args.height <= 0 or args.width % 2 or args.height % 2 or args.frames <= 0:
    raise SystemExit("invalid yuv420p dimensions/frame count")
frame_bytes = args.width * args.height * 3 // 2
src = Path(args.source).read_bytes()
dec = Path(args.decoded).read_bytes()
need = frame_bytes * args.frames
if len(src) < need or len(dec) < need:
    raise SystemExit(f"short raw sequence: need {need} bytes, got source={len(src)} decoded={len(dec)}")

def frame_psnr(a: memoryview, b: memoryview) -> float:
    sse = 0
    for x, y in zip(a, b):
        d = x - y
        sse += d * d
    if sse == 0:
        return math.inf
    mse = sse / len(a)
    return 10.0 * math.log10((255.0 * 255.0) / mse)

psnr = []
sa = memoryview(src)
da = memoryview(dec)
for i in range(args.frames):
    off = i * frame_bytes
    q = frame_psnr(sa[off:off+frame_bytes], da[off:off+frame_bytes])
    psnr.append(q)

if args.min_psnr is not None:
    worst = min(psnr)
    if worst < args.min_psnr:
        idx = psnr.index(worst) + 1
        raise SystemExit(f"frame {idx} PSNR {worst:.3f} dB is below {args.min_psnr:.3f} dB")
if args.keyframe:
    if args.keyframe_min is None:
        raise SystemExit("--keyframe requires --keyframe-min")
    for f in args.keyframe:
        if f < 1 or f > args.frames:
            raise SystemExit(f"keyframe {f} is outside 1..{args.frames}")
        q = psnr[f-1]
        if q < args.keyframe_min:
            raise SystemExit(f"keyframe {f} PSNR {q:.3f} dB is below {args.keyframe_min:.3f} dB")

finite = [x for x in psnr if math.isfinite(x)]
print(f"frames={args.frames} min={min(finite):.3f} avg={sum(finite)/len(finite):.3f} max={max(finite):.3f} dB")
for f in args.keyframe:
    print(f"keyframe {f}: {psnr[f-1]:.3f} dB")
