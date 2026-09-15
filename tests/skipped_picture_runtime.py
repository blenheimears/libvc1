#!/usr/bin/env python3
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

if len(sys.argv) != 3:
    raise SystemExit('usage: skipped_picture_runtime.py VC1ENC FFMPEG')
enc = Path(sys.argv[1]).resolve()
ffmpeg = Path(sys.argv[2]).resolve()
W = H = 64
FRAME_BYTES = W * H * 3 // 2


def frame(k: int) -> bytes:
    y = bytearray(W * H)
    for yy in range(H):
        for xx in range(W):
            y[yy * W + xx] = 16 + ((29 + k * 37 + (xx // 8) * 9 + (yy // 8) * 5 + ((xx + k * 3) ^ yy)) % 210)
    u = bytes([90 + k * 7]) * (W // 2 * H // 2)
    v = bytes([170 - k * 8]) * (W // 2 * H // 2)
    return bytes(y) + u + v


def write_y4m(path: Path, frames):
    with path.open('wb') as f:
        f.write(b'YUV4MPEG2 W64 H64 F24:1 Ip A1:1 C420jpeg\n')
        for fr in frames:
            f.write(b'FRAME\n')
            f.write(fr)


def run(cmd):
    cp = subprocess.run([str(x) for x in cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if cp.returncode:
        sys.stderr.buffer.write(cp.stdout)
        sys.stderr.buffer.write(cp.stderr)
        raise RuntimeError('command failed: ' + ' '.join(map(str, cmd)))
    return cp


def frame_payloads(es: bytes):
    start = b'\x00\x00\x01\x0d'
    positions = []
    at = 0
    while True:
        p = es.find(start, at)
        if p < 0:
            break
        positions.append(p)
        at = p + 4
    out = []
    all_starts = (b'\x00\x00\x01\x0d', b'\x00\x00\x01\x0e', b'\x00\x00\x01\x0f')
    for p in positions:
        ends = [es.find(sc, p + 4) for sc in all_starts]
        ends = [e for e in ends if e >= 0]
        end = min(ends) if ends else len(es)
        out.append(es[p + 4:end])
    return out

with tempfile.TemporaryDirectory(prefix='libvc1-skip-') as td:
    td = Path(td)

    # Canonical frame-doubled cadence: every second source frame is exact.
    unique = [frame(i) for i in range(4)]
    doubled = [x for f in unique for x in (f, f)]
    src = td / 'doubled.y4m'
    write_y4m(src, doubled)

    def encode(tag, threads=1, extra=()):
        out = td / f'{tag}.m2ts'
        es = td / f'{tag}.vc1'
        recon = td / f'{tag}.yuv'
        stats = td / f'{tag}.csv'
        run([enc, '-i', src, '-o', out, '--cq', '6', '--bframes', '2', '--threads', str(threads),
             '--es-out', es, '--recon-out', recon, '--debug-stats', stats, *extra])
        return es, recon, stats

    es1, recon1, stats1 = encode('skip-t1', 1)
    rows = list(csv.DictReader(stats1.open(newline='')))
    if len(rows) != 8:
        raise RuntimeError(f'expected 8 debug rows, got {len(rows)}')
    skipped = [r for r in rows if r['skipped_picture'] == '1']
    if [int(r['display_order']) for r in skipped] != [1, 3, 5, 7]:
        raise RuntimeError('exact doubled cadence did not select skipped pictures at 1/3/5/7')
    if any(r['type'] != 'P' or int(r['final_actual_bits']) != 40 for r in skipped):
        raise RuntimeError('skipped pictures were not minimal five-byte Advanced P access units')
    payloads = frame_payloads(es1.read_bytes())
    if len(payloads) != 8 or sum(p == b'\xf8' for p in payloads) != 4:
        raise RuntimeError('PTYPE=Skipped payloads are not the expected single-byte F8 RBDUs')

    decoded = td / 'decoded.yuv'
    run([ffmpeg, '-v', 'error', '-i', es1, '-f', 'rawvideo', '-pix_fmt', 'yuv420p', decoded])
    rb = recon1.read_bytes()
    db = decoded.read_bytes()
    if rb != db or len(db) != 8 * FRAME_BYTES:
        raise RuntimeError('FFmpeg reconstruction does not exactly match libvc1 skipped-picture reconstruction')
    for i in (0, 2, 4, 6):
        a = db[i * FRAME_BYTES:(i + 1) * FRAME_BYTES]
        b = db[(i + 1) * FRAME_BYTES:(i + 2) * FRAME_BYTES]
        if a != b:
            raise RuntimeError(f'decoded repeated pair {i}/{i+1} is not byte-identical')

    es4, _, _ = encode('skip-t4', 4)
    if es1.read_bytes() != es4.read_bytes():
        raise RuntimeError('skipped-picture elementary stream is not deterministic across 1/4 threads')

    es_no, _, stats_no = encode('noskip', 1, ('--no-skip-identical-frames',))
    no_rows = list(csv.DictReader(stats_no.open(newline='')))
    if any(r['skipped_picture'] == '1' for r in no_rows):
        raise RuntimeError('--no-skip-identical-frames still emitted skipped pictures')
    if len(es1.read_bytes()) >= len(es_no.read_bytes()):
        raise RuntimeError('skipped-picture fixture did not reduce elementary-stream size')

    # Mixed scheduling: B pictures on both sides of a duplicate pair must remain
    # valid when the duplicate forces two adjacent reference anchors.
    mixed_frames = [frame(10), frame(11), frame(12), frame(12), frame(13), frame(14), frame(15)]
    mixed = td / 'mixed.y4m'
    write_y4m(mixed, mixed_frames)
    mixed_es = td / 'mixed.vc1'
    mixed_recon = td / 'mixed-recon.yuv'
    mixed_stats = td / 'mixed.csv'
    run([enc, '-i', mixed, '-o', td / 'mixed.m2ts', '--cq', '7', '--bframes', '2', '--threads', '2',
         '--es-out', mixed_es, '--recon-out', mixed_recon, '--debug-stats', mixed_stats])
    mr = list(csv.DictReader(mixed_stats.open(newline='')))
    if sum(r['skipped_picture'] == '1' for r in mr) != 1 or sum(r['type'] == 'B' for r in mr) < 2:
        raise RuntimeError('mixed duplicate fixture did not retain both skipped-P and B-picture scheduling')
    mixed_dec = td / 'mixed-dec.yuv'
    run([ffmpeg, '-v', 'error', '-i', mixed_es, '-f', 'rawvideo', '-pix_fmt', 'yuv420p', mixed_dec])
    if mixed_dec.read_bytes() != mixed_recon.read_bytes():
        raise RuntimeError('mixed skipped/B fixture is not decoder/reconstruction exact')

    # ABR path must account timed skipped pictures without training the ordinary
    # P predictor from their fixed 40-bit syntax size.
    abr_stats = td / 'abr.csv'
    abr_es = td / 'abr.vc1'
    run([enc, '-i', src, '-o', td / 'abr.m2ts', '--bitrate', '300k', '--buffer-size', '600k',
         '--bframes', '2', '--threads', '1', '--es-out', abr_es, '--debug-stats', abr_stats])
    ar = list(csv.DictReader(abr_stats.open(newline='')))
    askip = [r for r in ar if r['skipped_picture'] == '1']
    if len(askip) != 4 or any(float(r['complexity']) != 0.0 or int(r['predicted_bits']) != 40 for r in askip):
        raise RuntimeError('ABR skipped-picture accounting is not using fixed syntax cost/zero source complexity')
    run([ffmpeg, '-v', 'error', '-i', abr_es, '-f', 'null', '-'])

print('libvc1 skipped-picture runtime ok')
