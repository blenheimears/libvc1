#!/usr/bin/env python3
import pathlib
import subprocess
import sys
import tempfile

enc = pathlib.Path(sys.argv[1])
ffmpeg = pathlib.Path(sys.argv[2])
W, H = 1280, 720


def run(cmd, **kwargs):
    return subprocess.run([str(x) for x in cmd], check=True, **kwargs)


with tempfile.TemporaryDirectory(prefix="libvc1-bluray-container-") as td:
    td = pathlib.Path(td)
    y4m = td / "frame.y4m"
    plane_y = bytes([96]) * (W * H)
    plane_c = bytes([128]) * ((W // 2) * (H // 2))
    with y4m.open("wb") as f:
        f.write(f"YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\nFRAME\n".encode())
        f.write(plane_y); f.write(plane_c); f.write(plane_c)

    outputs = [
        (td / "disc-temp.m2ts", []),
        # A .wmv extension would ordinarily imply WMV3/Main in auto mode;
        # explicit --format wvc1 keeps Advanced Profile in ASF.  Blu-ray
        # compatibility must validate codec parameters only and must not reject
        # this temporary/remux-oriented container choice.
        (td / "disc-temp.wmv", ["--format", "wvc1"]),
    ]
    for out, extra in outputs:
        run([
            enc, "-i", y4m, "-o", out, "--bluray-compat", "--cq", "20",
            "--simd", "none", "--threads", "1", "--intra-only",
            "--search-range", "0", "--max-frames", "1", *extra,
        ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        run([
            ffmpeg, "-v", "error", "-i", out, "-frames:v", "1", "-f", "null", "-",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    print("--bluray-compat is codec-only: M2TS and WVC1/ASF both accepted and decoded")
