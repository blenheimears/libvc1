#!/usr/bin/env python3
import pathlib, subprocess, sys, tempfile

enc = sys.argv[1]
ffmpeg = sys.argv[2]

with tempfile.TemporaryDirectory(prefix='libvc1-raw-output-') as td:
    d = pathlib.Path(td)
    src = d / 'in.y4m'
    raw = d / 'out.vc1'
    auto_raw = d / 'auto.vc1'
    dec = d / 'decoded.yuv'
    w, h = 64, 48
    ysize = w*h
    csize = (w//2)*(h//2)
    with src.open('wb') as f:
        f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for n in range(4):
            f.write(b'FRAME\n')
            f.write(bytes([64+n*8])*ysize)
            f.write(bytes([128])*csize)
            f.write(bytes([128])*csize)

    cmd = [enc, '-i', str(src), '-o', str(raw), '--format', 'raw', '--cq', '20',
           '--threads', '1', '--simd', 'none', '--bframes', '0', '--keyint', '2',
           '--fixed-gop-grid', '--no-scene-cut', '--search-range', '0', '--local-search-range', '0']
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if p.returncode:
        raise SystemExit('raw primary-output encode failed:\n' + p.stderr)
    if not raw.is_file() or raw.stat().st_size == 0:
        raise SystemExit('raw primary-output encode did not produce out.vc1')
    auto_cmd = [enc, '-i', str(src), '-o', str(auto_raw), '--cq', '20',
                '--threads', '1', '--simd', 'none', '--bframes', '0', '--keyint', '2',
                '--fixed-gop-grid', '--no-scene-cut', '--search-range', '0', '--local-search-range', '0']
    a = subprocess.run(auto_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if a.returncode:
        raise SystemExit('.vc1 auto-detected raw primary-output encode failed:\n' + a.stderr)
    if not auto_raw.is_file() or auto_raw.stat().st_size == 0:
        raise SystemExit('.vc1 auto-detected raw primary output was not created')
    if raw.read_bytes() != auto_raw.read_bytes():
        raise SystemExit('explicit --format raw and .vc1 auto-detection produced different streams')
    if list(d.glob('*.m2ts')) or list(d.glob('*.mts')):
        raise SystemExit('raw primary-output encode unexpectedly created an M2TS file')
    data = raw.read_bytes()
    if b'\x00\x00\x01\x0f' not in data or b'\x00\x00\x01\x0d' not in data:
        raise SystemExit('raw primary output is missing VC-1 sequence/frame start codes')

    q = subprocess.run([ffmpeg, '-v', 'error', '-f', 'vc1', '-i', str(raw),
                        '-pix_fmt', 'yuv420p', '-f', 'rawvideo', '-y', str(dec)],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if q.returncode or not dec.is_file() or dec.stat().st_size == 0:
        raise SystemExit('FFmpeg failed to decode raw primary VC-1 output:\n' + q.stderr)

print('raw VC-1 primary output without M2TS: PASS')
