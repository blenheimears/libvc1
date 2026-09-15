#!/usr/bin/env python3
import pathlib
import subprocess
import sys
import tempfile

enc = pathlib.Path(sys.argv[1])
ffmpeg = pathlib.Path(sys.argv[2])
W, H = 65, 49
CW, CH = (W + 1) // 2, (H + 1) // 2
CODED_W, CODED_H = W + 1, H + 1
CODED_CW, CODED_CH = CODED_W // 2, CODED_H // 2
FRAMES = 3


def run(cmd, **kwargs):
    return subprocess.run([str(x) for x in cmd], check=True, **kwargs)


with tempfile.TemporaryDirectory(prefix="libvc1-odd-dim-") as td:
    td = pathlib.Path(td)
    y4m = td / "odd.y4m"
    out = td / "odd.m2ts"
    recon = td / "odd-recon.yuv"
    decoded = td / "odd-decoded.yuv"

    with y4m.open("wb") as f:
        f.write(f"YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n".encode())
        for n in range(FRAMES):
            f.write(b"FRAME\n")
            f.write(bytes(((x * 3 + y * 5 + n * 11) & 255) for y in range(H) for x in range(W)))
            f.write(bytes((96 + ((x + y + n) & 31)) for y in range(CH) for x in range(CW)))
            f.write(bytes((160 - ((x * 2 + y + n) & 31)) for y in range(CH) for x in range(CW)))

    run([
        enc, "-i", y4m, "-o", out, "--cq", "8", "--simd", "none",
        "--threads", "1", "--bframes", "0", "--search-range", "0",
        "--no-scene-cut", "--keyint", "999", "--recon-out", recon,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    # -fps_mode passthrough prevents ffmpeg from duplicating frames to the MPEG-TS
    # stream's 90 kHz nominal rate when writing headerless rawvideo and remains
    # supported by FFmpeg 9+, where the legacy global -vsync option was removed.
    run([
        ffmpeg, "-v", "error", "-i", out, "-frames:v", str(FRAMES),
        "-fps_mode", "passthrough", "-pix_fmt", "yuv420p", "-f", "rawvideo", "-y", decoded,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    raw = decoded.read_bytes()
    wanted = recon.read_bytes()
    coded_frame = CODED_W * CODED_H + 2 * CODED_CW * CODED_CH
    visible_frame = W * H + 2 * CW * CH
    if len(raw) != FRAMES * coded_frame:
        raise RuntimeError(f"decoder produced {len(raw)} bytes, expected {FRAMES*coded_frame}")
    if len(wanted) != FRAMES * visible_frame:
        raise RuntimeError(f"reconstruction produced {len(wanted)} bytes, expected {FRAMES*visible_frame}")

    cropped = bytearray()
    for fi in range(FRAMES):
        frame = raw[fi * coded_frame:(fi + 1) * coded_frame]
        for y in range(H):
            cropped += frame[y * CODED_W:y * CODED_W + W]
        chroma = CODED_W * CODED_H
        # ceil(display/2) == coded/2 for the one-sample even padding, so both
        # chroma planes are already exactly the user-visible 4:2:0 geometry.
        cropped += frame[chroma:chroma + 2 * CW * CH]

    if bytes(cropped) != wanted:
        diffs = sum(a != b for a, b in zip(cropped, wanted))
        raise RuntimeError(f"odd-dimension Advanced decoder reconstruction differs ({diffs} bytes)")

    # Main/WMV3 has no in-band size syntax. The odd visible dimensions are
    # conveyed out-of-band by ASF while libvc1 keeps the same even internal
    # 4:2:0 working raster. FFmpeg must therefore decode directly to 65x49.
    wmv = td / "odd.wmv"
    main_recon = td / "odd-main-recon.yuv"
    main_decoded = td / "odd-main-decoded.yuv"
    run([
        enc, "-i", y4m, "-o", wmv, "--format", "wmv", "--cq", "8",
        "--simd", "none", "--threads", "1", "--bframes", "0",
        "--search-range", "0", "--no-scene-cut", "--keyint", "999",
        "--recon-out", main_recon,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    run([
        ffmpeg, "-v", "error", "-i", wmv, "-frames:v", str(FRAMES),
        "-fps_mode", "passthrough", "-pix_fmt", "yuv420p", "-f", "rawvideo", "-y", main_decoded,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    if main_decoded.read_bytes() != main_recon.read_bytes():
        a, b = main_decoded.read_bytes(), main_recon.read_bytes()
        diffs = sum(x != y for x, y in zip(a, b)) + abs(len(a) - len(b))
        raise RuntimeError(f"odd-dimension Main decoder reconstruction differs ({diffs} bytes)")

    print("odd-dimension decode/reconstruction exact: Advanced 65x49 display/66x50 coded and Main 65x49 out-of-band")
