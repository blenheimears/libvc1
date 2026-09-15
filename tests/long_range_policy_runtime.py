#!/usr/bin/env python3
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile

if len(sys.argv) != 2:
    raise SystemExit("usage: long_range_policy_runtime.py VC1ENC")
enc = os.path.abspath(sys.argv[1])

W, H = 1056, 16
CW, CH = W // 2, H // 2


def pattern_block(y, x0):
    for yy in range(16):
        for xx in range(16):
            y[yy * W + x0 + xx] = 70 + ((xx * 17 + yy * 29 + (xx ^ yy) * 7) % 121)


def make_frames():
    # Reference: one distinctive block at x=0. The immediately adjacent block
    # is only one luma level away from the flat background so a propagated
    # -512 candidate is "good" but strictly worse than the exact local match.
    ref_y = bytearray([37]) * (W * H)
    ref_u = bytearray([101]) * (CW * CH)
    ref_v = bytearray([151]) * (CW * CH)
    pattern_block(ref_y, 0)
    for yy in range(16):
        for xx in range(16):
            ref_y[yy * W + 16 + xx] = 38

    cur_y = bytearray([37]) * (W * H)
    cur_u = bytearray(ref_u)
    cur_v = bytearray(ref_v)
    # Move the distinctive block to x=512. This creates a genuine distant MV
    # first; the next macroblock at x=528 has an exact zero/local predictor.
    pattern_block(cur_y, 512)
    return (ref_y, ref_u, ref_v), (cur_y, cur_u, cur_v)


def write_y4m(path):
    with open(path, "wb") as f:
        f.write(f"YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n".encode())
        for planes in make_frames():
            f.write(b"FRAME\n")
            for plane in planes:
                f.write(plane)


def run(root, mode, explicit=True):
    out = root / f"{mode}.m2ts"
    stats = root / f"{mode}.csv"
    cmd = [
        enc, "-i", str(root / "policy.y4m"), "-o", str(out),
        "--cq", "6", "--bframes", "0", "--keyint", "24",
        "--search-range", "1024", "--local-search-range", "32",
        "--threads", "1", "--simd", "scalar", "--no-scene-cut",
        "--debug-stats", str(stats),
    ]
    if explicit:
        cmd += ["--long-range-search", mode]
    cp = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if cp.returncode:
        raise RuntimeError(f"{mode} encode failed:\n{cp.stderr}")
    with open(stats, newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 2 or rows[1]["type"] != "P":
        raise RuntimeError(f"unexpected {mode} statistics rows: {rows}")
    return float(rows[1]["mean_mv_pixels"]), float(rows[1]["max_mv_pixels"]), out.read_bytes()


with tempfile.TemporaryDirectory(prefix="libvc1-long-range-policy-") as td:
    root = Path(td)
    write_y4m(root / "policy.y4m")
    default_mean, default_max, default_bits = run(root, "compare", explicit=False)
    compare_mean, compare_max, compare_bits = run(root, "compare")
    local_mean, local_max, _ = run(root, "local-good")
    legacy_mean, legacy_max, _ = run(root, "legacy")

    if default_bits != compare_bits or abs(default_mean - compare_mean) > 1e-9:
        raise RuntimeError("default long-range policy is not compare")
    if compare_max < 500.0 or local_max < 500.0 or legacy_max < 500.0:
        raise RuntimeError("fixture did not exercise the intended distant motion vector")
    # compare must reject the propagated distant shortcut when local has lower
    # SAD. local-good should make the same choice while avoiding the test.
    if compare_mean > local_mean + 0.25:
        raise RuntimeError(f"compare selected worse propagated motion: compare={compare_mean}, local-good={local_mean}")
    # Historical distant-first mode should reproduce the propagation chain and
    # therefore have dramatically larger mean motion on this fixture.
    if legacy_mean < compare_mean + 100.0:
        raise RuntimeError(f"legacy mode did not preserve distant-first short-circuit: compare={compare_mean}, legacy={legacy_mean}")

print(f"long-range motion policy: PASS (compare={compare_mean:.3f}px, local-good={local_mean:.3f}px, legacy={legacy_mean:.3f}px)")
