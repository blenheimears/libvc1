#!/usr/bin/env python3
"""End-to-end Blu-ray 40M defaults in both passes, including piped input."""
from pathlib import Path
import subprocess
import sys
import tempfile

exe = sys.argv[1]
ffmpeg = sys.argv[2] if len(sys.argv) > 2 else None
w, h = 1280, 720
with tempfile.TemporaryDirectory(prefix='libvc1-bd-two-pass-peak-') as tmp:
    base = Path(tmp)
    video = bytearray(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    c = bytes([128]) * (w * h // 4)
    for n in range(6):
        video.extend(b'FRAME\n')
        video.extend(bytes([96 + n]) * (w * h))
        video.extend(c)
        video.extend(c)
    common = ['--bluray-compat', '--bitrate', '20M', '--buffer-size', '30M',
              '--keyint', '2', '--fixed-gop-grid', '--bframes', '0',
              '--threads', '1', '--fastest', '--simd', 'none',
              '--search-range', '0', '--local-search-range', '0', '--no-scene-cut']
    stats = base / 'pass.stats'
    def run(extra):
        res = subprocess.run([exe, '-i', '-', *common, *map(str, extra)],
                             input=bytes(video), stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, timeout=120)
        if res.returncode:
            raise AssertionError(res.stderr.decode(errors='replace'))
        return res.stderr.decode(errors='replace')
    log1 = run(['--pass', '1', '--pass-stats', stats])
    if not stats.exists() or 'END 6 ' not in stats.read_text():
        raise AssertionError('Blu-ray first pass did not finish statistics')
    if ('rate-mode=2pass, pass=1/2, target=20000000 bps, peak=399' not in log1
            or 'buffer=299' not in log1 or 'hrd-underflows=0' not in log1):
        raise AssertionError('first-pass target/peak must be independent 20M/40M with enforced VBV: ' + log1)
    # The previous release supported 40M via an explicit override on pass 1.
    # Implicit default must give identical first-pass statistics, not just a matching log.
    stats_explicit = base / 'pass-explicit.stats'
    log1_explicit = run(['--pass', '1', '--pass-stats', stats_explicit, '--max-bitrate', '40M'])
    if 'peak=399' not in log1_explicit or stats.read_bytes() != stats_explicit.read_bytes():
        raise AssertionError('first-pass default does not match explicitly requested 40M peak')
    # A 20M average with no --pass must retain the old single-pass peak default.
    single_log = run(['-o', base / 'single.vc1'])
    if 'rate-mode=abr, target=20000000 bps, peak=199' not in single_log:
        raise AssertionError('single-pass peak default changed unexpectedly: ' + single_log)
    output = base / 'second.vc1'
    log2 = run(['--pass', '2', '--pass-stats', stats, '--max-bitrate', '40M', '-o', output])
    if not output.exists() or not output.stat().st_size:
        raise AssertionError('Blu-ray second pass did not produce video')
    if 'rate-mode=2pass, pass=2/2, target=20000000 bps, peak=' not in log2:
        raise AssertionError('Blu-ray second pass did not distinguish average from peak')
    if 'peak=399' not in log2 or 'VBV=enforced' not in log2:
        raise AssertionError('Blu-ray peak/buffer not enforced at requested rate')
    if 'hrd-underflows=0' not in log2:
        raise AssertionError('Blu-ray second pass has HRD underflows')
    # Same target with no explicit max must adopt the 40 Mb/s BD default.
    default = base / 'default.vc1'
    default_log = run(['--pass', '2', '--pass-stats', stats, '-o', default])
    if 'peak=399' not in default_log or default.read_bytes() != output.read_bytes():
        raise AssertionError('Blu-ray second pass default peak differs from explicit 40M')
    if ffmpeg:
        decoded = subprocess.run([ffmpeg, '-v', 'error', '-i', str(output), '-f', 'null', '-'],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
        if decoded.returncode or decoded.stderr:
            raise AssertionError('Blu-ray second pass decoder errors: ' + decoded.stderr.decode(errors='replace'))
print('Blu-ray 20M average, implicit 40M peak in both passes, enforced HRD and decoder: PASS')
