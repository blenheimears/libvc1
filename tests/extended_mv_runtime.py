#!/usr/bin/env python3
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile

if len(sys.argv) != 3:
    raise SystemExit("usage: extended_mv_runtime.py VC1ENC FFMPEG")
enc = os.path.abspath(sys.argv[1])
ffmpeg = os.path.abspath(sys.argv[2])

W, H = 1056, 64
CW, CH = W // 2, H // 2
FRAME_BYTES = W * H + 2 * CW * CH


def make_frame(object_x):
    y = bytearray([37]) * (W * H)
    u = bytearray([101]) * (CW * CH)
    v = bytearray([151]) * (CW * CH)
    for yy in range(16):
        for xx in range(16):
            q = (xx // 8) + (yy // 8) * 2
            y[yy * W + object_x + xx] = 70 + q * 37 + ((xx * 19 + yy * 23 + (xx ^ yy) * 11) % 29)
    for yy in range(8):
        for xx in range(8):
            u[yy * CW + object_x // 2 + xx] = 175 + ((xx * 3 + yy * 7) % 23)
            v[yy * CW + object_x // 2 + xx] = 51 + ((xx * 5 + yy * 11) % 17)
    return y, u, v


def write_y4m(path, displacement):
    frames = (make_frame(0), make_frame(displacement))
    with open(path, "wb") as f:
        f.write(f"YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n".encode())
        for planes in frames:
            f.write(b"FRAME\n")
            for plane in planes:
                f.write(plane)


def check_stats(path, profile, expected_search, expected_mv):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 2 or rows[1]["type"] != "P":
        raise RuntimeError(f"unexpected {profile} debug statistics rows: {rows}")
    if int(rows[1]["search_range"]) != expected_search:
        raise RuntimeError(f"{profile} effective search range {rows[1]['search_range']} != {expected_search}")
    max_mv = float(rows[1]["max_mv_pixels"])
    if max_mv < expected_mv - 0.001 or max_mv > expected_mv + 0.001:
        raise RuntimeError(f"{profile} fixture expected {expected_mv}-pixel MV, got {max_mv}")


def common_args(input_path, threads):
    return [
        "-i", str(input_path), "--cq", "6", "--bframes", "0",
        "--keyint", "24", "--search-range", "1024", "--no-scene-cut",
        "--threads", str(threads), "--simd", "scalar",
    ]


def run_advanced(root, threads):
    stem = root / f"adv-t{threads}"
    out = stem.with_suffix(".m2ts")
    es = stem.with_suffix(".vc1")
    recon = stem.with_suffix(".yuv")
    stats = stem.with_suffix(".csv")
    cmd = [enc, *common_args(root / "advanced-long.y4m", threads), "-o", str(out),
           "--es-out", str(es), "--recon-out", str(recon), "--debug-stats", str(stats)]
    cp = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if cp.returncode:
        raise RuntimeError(f"Advanced encoder failed for {threads} threads:\n{cp.stderr}")
    check_stats(stats, "Advanced", 1024, 1024.0)
    return out, es, recon


def run_main(root, threads):
    stem = root / f"main-t{threads}"
    out = stem.with_suffix(".wmv")
    recon = stem.with_suffix(".yuv")
    stats = stem.with_suffix(".csv")
    cmd = [enc, *common_args(root / "main-long.y4m", threads), "-o", str(out), "--format", "wmv",
           "--recon-out", str(recon), "--debug-stats", str(stats)]
    cp = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if cp.returncode:
        raise RuntimeError(f"Main encoder failed for {threads} threads:\n{cp.stderr}")
    check_stats(stats, "Main", 255, 255.0)
    return out, recon


def same(a, b, label):
    if a.read_bytes() != b.read_bytes():
        raise RuntimeError(f"1-thread/4-thread {label} differs")


with tempfile.TemporaryDirectory(prefix="libvc1-extmv-") as td:
    root = Path(td)
    # Advanced keeps the full 1024-pixel requested range and exercises the
    # most-negative horizontal endpoint of ST 421 MVRANGE=111.
    write_y4m(root / "advanced-long.y4m", 1024)
    # WMV3/Main is invoked with the same --search-range 1024 request, but the
    # public one-radius interface is capped to 255 so a symmetric integer-pixel radius cannot exceed
    # Main Profile High Level's positive vertical +255.75 limit.
    write_y4m(root / "main-long.y4m", 255)

    # Advanced/WVC1: prove maximum MVRANGE syntax, long-MVDATA widths,
    # reconstruction, and deterministic transport/ES output.
    ao1, aes1, ar1 = run_advanced(root, 1)
    ao4, aes4, ar4 = run_advanced(root, 4)
    same(ao1, ao4, "Advanced M2TS")
    same(aes1, aes4, "Advanced VC-1 ES")
    same(ar1, ar4, "Advanced reconstruction")

    adv_decoded = root / "advanced-ffmpeg.yuv"
    cp = subprocess.run([
        ffmpeg, "-hide_banner", "-loglevel", "error", "-f", "vc1", "-i", str(aes1),
        "-f", "rawvideo", "-pix_fmt", "yuv420p", "-y", str(adv_decoded),
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if cp.returncode:
        raise RuntimeError(f"FFmpeg rejected maximum-range Advanced VC-1 stream:\n{cp.stderr}")
    same(adv_decoded, ar1, "Advanced FFmpeg/libvc1 reconstruction")

    # WMV9/Main carries its sequence metadata in ASF. The caller still requests
    # 1024, but vc1_encoder_open() clamps Main to an effective 255-pixel scalar
    # radius while Advanced remains 1024. Decode the ASF directly; passthrough
    # timing avoids FFmpeg duplicating frames to the ASF millisecond timebase.
    mo1, mr1 = run_main(root, 1)
    mo4, mr4 = run_main(root, 4)
    same(mo1, mo4, "Main ASF")
    same(mr1, mr4, "Main reconstruction")
    main_decoded = root / "main-ffmpeg.yuv"
    cp = subprocess.run([
        ffmpeg, "-hide_banner", "-loglevel", "error", "-i", str(mo1),
        "-fps_mode", "passthrough", "-f", "rawvideo", "-pix_fmt", "yuv420p",
        "-y", str(main_decoded),
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if cp.returncode:
        raise RuntimeError(f"FFmpeg rejected maximum-range Main/WMV3 stream:\n{cp.stderr}")
    same(main_decoded, mr1, "Main FFmpeg/libvc1 reconstruction")

    for decoded, profile in ((adv_decoded, "Advanced"), (main_decoded, "Main")):
        if len(decoded.read_bytes()) != 2 * FRAME_BYTES:
            raise RuntimeError(f"{profile} FFmpeg decode produced an unexpected byte count")

    # The public range is deliberately capped at the largest progressive
    # MVRANGE defined by ST 421.
    cp = subprocess.run([
        enc, "-i", str(root / "advanced-long.y4m"), "-o", str(root / "invalid.m2ts"),
        "--cq", "6", "--bframes", "0", "--search-range", "1025", "--max-frames", "1",
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if cp.returncode == 0 or "0..1024" not in cp.stderr:
        raise RuntimeError("--search-range 1025 was not rejected at the VC-1 maximum")

print("extended-MV runtime: PASS (Advanced 1024 px; WMV3/Main request capped to 255 px; FFmpeg exact; 1T/4T deterministic)")
