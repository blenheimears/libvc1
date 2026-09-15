#!/usr/bin/env python3
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

if len(sys.argv) < 2:
    raise SystemExit('usage: aq_dim_chroma_runtime.py VC1ENC')
enc = sys.argv[1]
W, H = 128, 96
CW, CH = W // 2, H // 2
FB = W * H * 3 // 2


def write_y4m(path, frames):
    with open(path, 'wb') as f:
        f.write(f'YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for y, u, v in frames:
            f.write(b'FRAME\n')
            f.write(np.asarray(y, np.uint8).tobytes())
            f.write(np.asarray(u, np.uint8).tobytes())
            f.write(np.asarray(v, np.uint8).tobytes())


def encode(inp, out, recon, *extra):
    cmd = [enc, '-i', str(inp), '-o', str(out), '--recon-out', str(recon),
           '--intra-only', '--threads', '1', '--no-overlap', *extra]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def planes(raw, index):
    p = index * FB
    frame = raw[p:p + FB].astype(np.float64)
    return (frame[:W * H],
            frame[W * H:W * H + W * H // 4],
            frame[W * H + W * H // 4:])


def mse(a, b):
    d = np.asarray(a, np.float64).ravel() - np.asarray(b, np.float64).ravel()
    return float(np.mean(d * d))

with tempfile.TemporaryDirectory(prefix='libvc1-aq-dim-chroma-') as td:
    td = Path(td)

    # Saturated/dim material: luminance AQ must depend on actual luma visibility,
    # not on whether the chroma happens to be neutral/gray. The two brighter-dim
    # frames intentionally sit above the old narrow dark-detail window.
    yy, xx = np.indices((H, W))
    dim_frames = []
    for mean in (45, 75, 105, 125):
        y = (mean + 5 * np.sin(xx / 2.7) + 4 * np.cos(yy / 3.1) + ((xx + yy) % 3 - 1)).clip(0, 255).astype(np.uint8)
        u = np.full((CH, CW), 60, np.uint8)
        v = np.full((CH, CW), 195, np.uint8)
        dim_frames.append((y, u, v))
    dim_y4m = td / 'dim-color.y4m'
    write_y4m(dim_y4m, dim_frames)
    encode(dim_y4m, td / 'dim-aq.m2ts', td / 'dim-aq.yuv', '--cq', '12')
    encode(dim_y4m, td / 'dim-noaq.m2ts', td / 'dim-noaq.yuv', '--cq', '12', '--no-aq')
    aq = np.fromfile(td / 'dim-aq.yuv', np.uint8)
    no = np.fromfile(td / 'dim-noaq.yuv', np.uint8)
    for i in (2, 3):
        src_y = dim_frames[i][0].astype(np.float64).ravel()
        aq_y = planes(aq, i)[0]
        no_y = planes(no, i)[0]
        ma, mn = mse(src_y, aq_y), mse(src_y, no_y)
        if not (ma < mn * 0.82):
            raise SystemExit(f'dim luma AQ regression frame {i}: AQ MSE {ma:.4f}, no-AQ {mn:.4f}')

    # Flat luma must not hide detailed chroma. Use dissimilar U/V patterns so
    # both chroma channels independently need coefficient protection.
    y = np.full((H, W), 92, np.uint8)
    cy, cx = np.indices((CH, CW))
    u = np.where(((cx // 2 + cy // 2) & 1) == 0, 72, 184).astype(np.uint8)
    v = np.where(((cx // 3 + cy // 3) & 1) == 0, 192, 64).astype(np.uint8)
    chroma_frame = [(y, u, v)]
    chroma_y4m = td / 'flat-luma-detailed-chroma.y4m'
    write_y4m(chroma_y4m, chroma_frame)
    encode(chroma_y4m, td / 'chroma-aq.m2ts', td / 'chroma-aq.yuv', '--cq', '18')
    encode(chroma_y4m, td / 'chroma-noaq.m2ts', td / 'chroma-noaq.yuv', '--cq', '18', '--no-aq')
    ca = np.fromfile(td / 'chroma-aq.yuv', np.uint8)
    cn = np.fromfile(td / 'chroma-noaq.yuv', np.uint8)
    _, au, av = planes(ca, 0)
    _, nu, nv = planes(cn, 0)
    aq_chroma = mse(u, au) + mse(v, av)
    no_chroma = mse(u, nu) + mse(v, nv)
    if not (aq_chroma < no_chroma * 0.93):
        raise SystemExit(f'flat-luma chroma AQ regression: AQ MSEsum {aq_chroma:.4f}, no-AQ {no_chroma:.4f}')

    # Completely black and completely flat colored material must not acquire
    # extra AQ syntax/coefficients merely because its luma is low.
    flat_frames = [
        (np.zeros((H, W), np.uint8), np.full((CH, CW), 128, np.uint8), np.full((CH, CW), 128, np.uint8)),
        (np.full((H, W), 36, np.uint8), np.full((CH, CW), 70, np.uint8), np.full((CH, CW), 190, np.uint8)),
    ] * 2
    flat_y4m = td / 'flat-dark.y4m'
    write_y4m(flat_y4m, flat_frames)
    aq_es, no_es = td / 'flat-aq.vc1', td / 'flat-noaq.vc1'
    subprocess.run([enc, '-i', str(flat_y4m), '-o', str(td / 'flat-aq.m2ts'), '--es-out', str(aq_es),
                    '--intra-only', '--threads', '1', '--no-overlap', '--cq', '18'], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run([enc, '-i', str(flat_y4m), '-o', str(td / 'flat-noaq.m2ts'), '--es-out', str(no_es),
                    '--intra-only', '--threads', '1', '--no-overlap', '--cq', '18', '--no-aq'], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if aq_es.read_bytes() != no_es.read_bytes():
        raise SystemExit('flat/black AQ guard regression: AQ changed an otherwise flat elementary stream')

print('0.1.78 dim/color-aware + chroma-detail AQ runtime regression: PASS')
