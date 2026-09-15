#!/usr/bin/env python3
import pathlib
import subprocess
import sys
import tempfile

enc = pathlib.Path(sys.argv[1])
ffmpeg = pathlib.Path(sys.argv[2])
W, H = 2048, 1088  # Above 1920x1080 and above AP@L3's 8192-MB/frame ceiling.
CW, CH = W // 2, H // 2


def run(cmd, **kwargs):
    return subprocess.run([str(x) for x in cmd], check=True, **kwargs)


with tempfile.TemporaryDirectory(prefix="libvc1-high-dim-") as td:
    td = pathlib.Path(td)
    y4m = td / "high.y4m"
    out = td / "high.m2ts"
    recon = td / "high-recon.yuv"
    decoded = td / "high-decoded.yuv"

    with y4m.open("wb") as f:
        f.write(f"YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\nFRAME\n".encode())
        f.write(bytes([96]) * (W * H))
        f.write(bytes([128]) * (CW * CH))
        f.write(bytes([128]) * (CW * CH))

    run([
        enc, "-i", y4m, "-o", out, "--cq", "20", "--simd", "none",
        "--threads", "1", "--intra-only", "--recon-out", recon,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    run([
        ffmpeg, "-v", "error", "-i", out, "-frames:v", "1", "-fps_mode", "passthrough",
        "-pix_fmt", "yuv420p", "-f", "rawvideo", "-y", decoded,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    if decoded.read_bytes() != recon.read_bytes():
        a, b = decoded.read_bytes(), recon.read_bytes()
        diffs = sum(x != y for x, y in zip(a, b)) + abs(len(a) - len(b))
        raise RuntimeError(f">1080 decoder reconstruction differs ({diffs} bytes)")

    print("Advanced Profile Level 4 >1080 decode/reconstruction exact: 2048x1088")
