#!/usr/bin/env python3
"""Check I/P/B content presets, overrides, first-/second-pass aliases, CQP policy."""
from pathlib import Path
import subprocess
import sys
import tempfile

exe = sys.argv[1]
w, h = 48, 32
frames = 9
source = bytearray(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
for t in range(frames):
    source.extend(b'FRAME\n')
    source.extend(bytes(25 + (x * 3 + y * 5 + t * 19) % 200
                        for y in range(h) for x in range(w)))
    source.extend(bytes([96 + t]) * (w * h // 4))
    source.extend(bytes([144 - t]) * (w * h // 4))
common = ['-i', '-', '--bitrate', '450k', '--buffer-size', '800k',
          '--threads', '1', '--no-trellis', '--no-aq', '--no-dquant', '--no-fade-comp', '--fixed-8x8', '--search-range', '0', '--local-search-range', '0', '--simd', 'none',
          '--no-scene-cut', '--keyint', '3']

def invoke(options, out=None):
    args = [exe, *common, *map(str, options)]
    if out is not None:
        args += ['-o', str(out)]
    p = subprocess.run(args, input=bytes(source), capture_output=True, timeout=120)
    return p.returncode, p.stderr.decode(errors='replace')

def encode(root, name, options):
    path = root / (name + '.vc1')
    status, log = invoke(options, path)
    assert status == 0, (name, log)
    assert path.stat().st_size, name
    return path.read_bytes(), log

status, help_text = invoke(['--help'])
assert status == 0
for option in ('--animation', '--film', '--grain'):
    assert option in help_text
assert 'default: 4.0' in help_text

with tempfile.TemporaryDirectory(prefix='libvc1-content-preset-') as tmp:
    root = Path(tmp)
    data = {}
    for name, options, weight in (
        ('default', [], 4), ('default-explicit', ['--i-block-weight', '4'], 4),
        ('animation', ['--animation'], 5), ('animation-explicit', ['--i-block-weight', '5'], 5),
        ('film', ['--film'], 3), ('film-explicit', ['--i-block-weight', '3'], 3),
        ('grain', ['--grain'], 3),
        ('film-then-animation', ['--film', '--animation'], 5),
        ('animation-then-grain', ['--animation', '--grain'], 3),
        ('film-then-explicit', ['--film', '--i-block-weight', '7'], 7),
        ('explicit-seven', ['--i-block-weight', '7'], 7),
        ('explicit-then-film', ['--i-block-weight', '7', '--film'], 3),
        ('film-fast', ['--film', '--fast'], 3),
    ):
        payload, log = encode(root, name, options)
        assert f'weights=I:{weight}/P:1/B:0.7' in log, (name, log)
        data[name] = payload
    assert data['default'] == data['default-explicit']
    assert data['animation'] == data['animation-explicit'] == data['film-then-animation']
    assert data['film'] == data['film-explicit'] == data['grain']
    assert data['film'] == data['animation-then-grain'] == data['explicit-then-film']
    assert data['film-then-explicit'] == data['explicit-seven']

    # Both passes accept content presets; --grain must be an exact --film alias.
    for name, preset in [('film', '--film'), ('grain', '--grain'),
                         ('explicit', '--i-block-weight')]:
        stats = root / f'{name}.stats'
        flags = [preset, '3'] if name == 'explicit' else [preset]
        status, log = invoke([*flags, '--pass', '1', '--pass-stats', stats])
        assert status == 0 and 'pass=1/2' in log, (name, log)
        assert stats.stat().st_size, name
        status, log = invoke([*flags, '--pass', '2', '--pass-stats', stats], root / f'{name}-pass2.vc1')
        assert status == 0 and 'pass=2/2' in log, (name, log)
    assert (root/'film.stats').read_bytes() == (root/'grain.stats').read_bytes() == (root/'explicit.stats').read_bytes()
    assert (root/'film-pass2.vc1').read_bytes() == (root/'grain-pass2.vc1').read_bytes() == (root/'explicit-pass2.vc1').read_bytes()

    # Content presets set ABR weights; reject their use in constant-quantizer mode.
    result = subprocess.run([exe, '-i', '-', '-o', str(root/'invalid.vc1'), '--threads', '1',
                             '--simd', 'none', '--cq', '6', '--grain'],
                            input=bytes(source), capture_output=True, timeout=120)
    assert result.returncode != 0 and b'weights apply only to bitrate-controlled mode' in result.stderr, result.stderr

print('content preset regression: PASS (default 4; animation 5; film/grain 3; order; two-pass alias)')
