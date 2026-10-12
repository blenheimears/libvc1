#!/usr/bin/env python3
"""Reproduce P-intra darkening next to coded black padding, in both directions."""
import csv
import pathlib
import subprocess
import sys
import tempfile

encoder = pathlib.Path(sys.argv[1])
ffmpeg = sys.argv[2]


def run_case(td, w, h, direction):
    src = td / f'{direction}.y4m'
    images = []
    with src.open('wb') as f:
        f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for n in range(2):
            f.write(b'FRAME\n')
            # Unchanged interior preserves a P frame. Only the right/bottom
            # macroblocks differ enough to choose intra coding in picture 2.
            y = bytes((30+((x+y)&1)*190 if n == 0 or
                       (direction == 'bottom' and y < h-8) or
                       (direction == 'right' and x < w-8)
                       else 185+((3*(x if direction == 'bottom' else y))%40))
                      for y in range(h) for x in range(w))
            images.append(y)
            f.write(y)
            f.write(bytes([128])*(w*h//2))
    for overlap in (True, False):
        label = f'{direction}-{int(overlap)}'
        out = td / f'{label}.m2ts'
        recon = td / f'{label}.yuv'
        decoded = td / f'{label}-decoded.yuv'
        stats = td / f'{label}.csv'
        subprocess.run([str(encoder), '-i', str(src), '-o', str(out),
                        '--format', 'm2ts', '--cq', '12', '--threads', '1',
                        '--simd', 'none', '--search-range', '4',
                        '--local-search-range', '4', '--no-scene-cut',
                        '--keyint', '24', '--fixed-gop-grid', '--no-loop-filter',
                        '--no-aq', '--fixed-8x8', '--trellis', '0',
                        '--recon-out', str(recon), '--debug-stats', str(stats),
                        '--overlap' if overlap else '--no-overlap', '--bframes', '0'],
                       check=True, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        with stats.open(newline='') as f:
            rows = list(csv.DictReader(f))
        pframes = [r for r in rows if r['type'] == 'P']
        if len(pframes) != 1 or int(pframes[0]['p_intra_mb']) < 1:
            raise RuntimeError(f'{label}: test did not produce a P-intra macroblock')
        subprocess.run([ffmpeg, '-hide_banner', '-v', 'error', '-xerror',
                        '-f', 'mpegts', '-i', str(out), '-frames:v', '2', '-fps_mode',
                        'passthrough', '-pix_fmt', 'yuv420p', '-f', 'rawvideo',
                        '-y', str(decoded)], check=True, stdout=subprocess.DEVNULL)
        if recon.read_bytes() != decoded.read_bytes():
            raise RuntimeError(f'{label}: decoder and encoder reconstruction differ')
        rec = recon.read_bytes()[w*h*3//2:w*h*3//2+w*h]
        if direction == 'bottom':
            border = [rec[y*w+x] for y in (h-2, h-1) for x in range(w)]
        else:
            border = [rec[y*w+x] for y in range(h) for x in (w-2, w-1)]
        if min(border) < 145:
            raise RuntimeError(f'{label}: black padding contaminated visible edge: {min(border)}')
        print(f'{label}: P-intra={pframes[0]["p_intra_mb"]}; '
              f'border min={min(border)}; decoder parity PASS')


with tempfile.TemporaryDirectory(prefix='libvc1-overlap-edge-padding-') as tmp:
    td = pathlib.Path(tmp)
    run_case(td, 64, 40, 'bottom')
    run_case(td, 40, 64, 'right')
print('overlap P-intra edge padding: PASS')
