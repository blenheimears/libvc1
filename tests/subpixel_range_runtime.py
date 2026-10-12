#!/usr/bin/env python3
"""Exercise independent fractional-search radius without changing integer ME."""
from pathlib import Path
import subprocess
import sys
import tempfile

encoder = str(Path(sys.argv[1]).resolve())
w, h, frames = 96, 64, 9
source = bytearray(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
for t in range(frames):
    source.extend(b'FRAME\n')
    for y in range(h):
        for x in range(w):
            xx, yy = (x + 11*t) % w, (y + 3*t) % h
            source.append((xx*7 + yy*13 + (xx*yy) % 191) % 256)
    for y in range(h//2):
        for x in range(w//2):
            source.append((120 + (x+t*3+y) % 31) % 256)
    for y in range(h//2):
        for x in range(w//2):
            source.append((100 + (x*3+y+t*5) % 73) % 256)

with tempfile.TemporaryDirectory(prefix='libvc1-subpixel-limit-') as td:
    root = Path(td)
    def encode(name, *options, valid=True):
        output = root / (name + '.vc1')
        cmd = [encoder, '-i', '-', '-o', str(output), '--threads', '1', '--simd', 'none',
               '--cq', '6', '--no-scene-cut', *options]
        result = subprocess.run(cmd, input=source, capture_output=True, timeout=120)
        log = result.stderr.decode(errors='replace')
        if not valid:
            assert result.returncode != 0 and 'subpixel refinement range must be 0..1024' in log, (name, log)
            return None, log
        assert result.returncode == 0 and output.stat().st_size > 100, (name, log)
        return output.read_bytes(), log

    for preset in [()]:
        a, _ = encode('default-' + str(preset), *preset)
        b, log = encode('max-' + str(preset), *preset, '--subpixel-range', '1024')
        assert a == b and 'subpixel-range=1024' in log, preset

    standard, _ = encode('standard', '--fast')
    explicit_default, _ = encode('default16', '--fast', '--subpixel-range', '16')
    assert standard == explicit_default, 'fast preset must default to 16'
    restricted, log = encode('restricted', '--fast', '--subpixel-range', '8')
    assert 'subpixel-range=8' in log and restricted, 'range 8 was not accepted'
    zero, _ = encode('zero', '--fast', '--subpixel-range', '0')
    none, _ = encode('none', '--fast', '--no-subpixel-refinement')
    assert zero == none, 'zero cutoff differs from integer-only search'
    for opts in [('--fast', '--subpixel-range', '8')]:
        result, _ = encode('order' + str(opts), *opts)
        assert result == restricted, opts
    for value in ('-1', '1025'):
        encode('bad-' + value, '--fast', '--subpixel-range', value, valid=False)
    ext, log = encode('extended', '--fast', '--local-search-range', '4', '--search-range', '1024',
                      '--subpixel-range', '128')
    assert 'search=1024 px (local 4)' in log and 'subpixel-range=128' in log and len(ext)>0
    same, log = encode('alias', '--fast', '--subpixel-refinement-range', '8')
    assert same == restricted and 'subpixel-range=8' in log
print('subpixel limit: default parity, 0/8/128/1024, preset order, alias, extended range and invalid values passed')
