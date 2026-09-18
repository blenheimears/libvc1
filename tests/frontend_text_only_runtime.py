#!/usr/bin/env python3
"""Check help/2-pass labels and optionally byte-compare 0.2.41 video outputs."""
import csv
from pathlib import Path
import subprocess
import sys
import tempfile

if len(sys.argv) not in (2, 3):
    raise SystemExit('usage: frontend_text_only_runtime.py NEW_VC1ENC [OLD_0_2_41_VC1ENC]')
new, old = sys.argv[1], sys.argv[2] if len(sys.argv) == 3 else None

def run(exe, args, stdin=None):
    p = subprocess.run([exe, *map(str, args)], input=stdin, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, check=False)
    if p.returncode:
        raise AssertionError(f'{exe}: exit {p.returncode}: {p.stderr.decode(errors="replace")}')
    return p.stderr.decode(errors='replace')

help_text = run(new, ['--help'])
for heading in ('INPUT AND OUTPUT', 'RATE CONTROL', 'GOP AND MOTION ESTIMATION',
                'CODING TOOLS', 'PERFORMANCE AND COMPUTE', 'DIAGNOSTICS', 'EXAMPLES'):
    assert '\n' + heading + '\n' in help_text, f'missing section: {heading}'
for option in ('--pass 1|2 --pass-stats FILE', '--no-dynamic-ipb-weights',
               '--simd-primitive', '--macroblock-stats', '--ac-mode',
               '--long-range-search', '--residual-priority-threshold'):
    assert option in help_text, f'missing help option: {option}'
assert '\\n' not in help_text, 'help contains escaped newlines'
assert max(map(len, help_text.splitlines()[3:])) < 112, 'help lines are excessively long'

W, H, N = 48, 32, 12
video = bytearray(f'YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n'.encode())
for t in range(N):
    video += b'FRAME\n'
    video += bytes(24 + ((x * 5 + y * 7 + t * 17 + (t // 6) * 37) % 205)
                   for y in range(H) for x in range(W))
    video += bytes((90 + x * 3 + t * 2) % 256 for y in range(H // 2) for x in range(W // 2))
    video += bytes((130 + y * 5 + t * 3) % 256 for y in range(H // 2) for x in range(W // 2))
common = ['-i', '-', '--bitrate', '450k', '--buffer-size', '800k', '--keyint', '6',
          '--threads', '1', '--fastest', '--no-scene-cut', '--simd', 'none']

with tempfile.TemporaryDirectory(prefix='libvc1-format-only-') as directory:
    root = Path(directory)
    single = root / 'single-new.vc1'
    log = run(new, [*common, '-o', single], video)
    assert 'rate-mode=abr, target=' in log and 'rate-mode=2pass' not in log
    for pass_number in (1, 2):
        stats = root / 'new.stats'
        args = [*common, '--pass', str(pass_number), '--pass-stats', stats]
        if pass_number == 2:
            args += ['-o', root / 'new.vc1']
        log = run(new, args, video)
        if pass_number == 1:
            assert 'rate-mode=2pass, pass=1/2, target=' in log, log
        else:
            assert 'rate-mode=2pass, pass=2/2, target=' in log and 'peak=unrestricted, VBV=disabled' in log, log
        assert 'rate-mode=abr' not in log
    assert (root / 'new.vc1').stat().st_size > 0
    # Diagnostic changes are text only: record the selected mode in both reports.
    debug = root / 'debug.csv'
    mb = root / 'macroblocks.csv'
    run(new, [*common, '--pass', '2', '--pass-stats', root / 'new.stats',
              '-o', root / 'diagnostics.vc1', '--debug-stats', debug,
              '--macroblock-stats', mb], video)
    with debug.open(newline='') as fp:
        assert {row['rc_mode'] for row in csv.DictReader(fp)} == {'2pass'}
    assert '# rc_mode=2pass\n' in mb.read_text()

    if old:
        for suffix in ('vc1', 'm2ts', 'wmv'):
            old_out = root / f'old-single.{suffix}'
            new_out = root / f'new-single.{suffix}'
            run(old, [*common, '-o', old_out], video)
            run(new, [*common, '-o', new_out], video)
            assert old_out.read_bytes() == new_out.read_bytes(), f'{suffix}: single-pass bytes changed'
        old_stats = root / 'old.stats'
        run(old, [*common, '--pass', '1', '--pass-stats', old_stats], video)
        assert old_stats.read_bytes() == (root / 'new.stats').read_bytes(), 'pass-1 stats format changed'
        for suffix in ('vc1', 'm2ts', 'wmv'):
            # WMV3 is a different codec profile: its pass-1 stats must also
            # select WMV, even though first-pass video is suppressed.
            stats_for_codec = old_stats
            if suffix == 'wmv':
                stats_for_codec = root / 'old-wmv.stats'
                new_wmv_stats = root / 'new-wmv.stats'
                run(old, [*common, '--format', 'wmv', '--pass', '1',
                          '--pass-stats', stats_for_codec], video)
                run(new, [*common, '--format', 'wmv', '--pass', '1',
                          '--pass-stats', new_wmv_stats], video)
                assert stats_for_codec.read_bytes() == new_wmv_stats.read_bytes(), 'WMV first-pass stats changed'
            old_out = root / f'old-twopass.{suffix}'
            new_out = root / f'new-twopass.{suffix}'
            run(old, [*common, '--pass', '2', '--pass-stats', stats_for_codec, '-o', old_out], video)
            run(new, [*common, '--pass', '2', '--pass-stats', stats_for_codec, '-o', new_out], video)
            assert old_out.read_bytes() == new_out.read_bytes(), f'{suffix}: two-pass bytes changed'
        print('help, rate labels, debug labels, and 0.2.41 byte-for-byte video parity passed')
    else:
        print('help, rate labels and debug labels passed (no 0.2.41 binary supplied)')
