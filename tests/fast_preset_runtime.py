#!/usr/bin/env python3
"""Check single throughput preset, option order, and removed presets."""
from pathlib import Path
import subprocess
import sys
import tempfile

if len(sys.argv) != 2:
    raise SystemExit('usage: fast_preset_runtime.py /path/to/vc1enc')
encoder = str(Path(sys.argv[1]).resolve())
w, h, count = 64, 48, 7
source = bytearray(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
for t in range(count):
    source.extend(b'FRAME\n')
    source.extend(bytes((19 + x * 5 + y * 7 + (x * y) % 43 + t * (x % 4 + 2)) % 235
                        for y in range(h) for x in range(w)))
    source.extend(bytes((83 + x * 3 + y + t * 2) % 240
                        for y in range(h // 2) for x in range(w // 2)))
    source.extend(bytes((138 + x + y * 3 + t * 4) % 240
                        for y in range(h // 2) for x in range(w // 2)))

with tempfile.TemporaryDirectory(prefix='libvc1-fast-preset-') as tmp:
    root = Path(tmp)
    def encode(label, options, payload=source):
        output = root / (label + '.vc1')
        proc = subprocess.run([encoder, '-i', '-', '-o', str(output), '--cq', '6',
                               '--threads', '1', '--simd', 'none', *options],
                              input=payload, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=120)
        log = proc.stderr.decode(errors='replace')
        assert proc.returncode == 0, (label, log)
        data = output.read_bytes()
        assert len(data) > 100, label
        assert 'I=' in log and 'P=' in log and 'B=' in log, (label, log)
        return data, log

    normal, norm_log = encode('normal', [])
    fast, fast_log = encode('fast', ['--fast'])
    # Old --faster + the user's exact overrides, expressed without removed presets.
    manual_flags = ['--no-fade-comp', '--no-trellis', '--no-aq', '--no-dquant',
                    '--ac-mode', 'vlc', '--ac-y-table', '1', '--ac-c-table', '1',
                    '--bframes', '2', '--no-scene-cut', '--fast-transform-rdo',
                    '--variable-transforms', '--search-range', '1024',
                    '--local-search-range', '8', '--me-algorithm', 'diamond',
                    '--long-range-search', 'local-good', '--me-quality', 'sad',
                    '--fast-subpixel-refinement', '--subpixel-range', '16']
    manual, _ = encode('manual-fast', manual_flags)
    assert fast == manual, 'fast differs from the explicitly specified B-enabled throughput preset'
    for token in ('preset=fast', 'search=1024 px (local 8)', 'me=sad',
                  'me-algorithm=diamond', 'long-range=local-good',
                  'subpixel-mode=fast', 'subpixel-range=16', 'scene-cut=off',
                  'fade-comp=off', 'variable-transforms=on', 'fast-transform-rdo=on',
                  'trellis=0', 'aq=off', 'dquant=off', 'overlap=on',
                  'loop-filter=on', 'bframes=2'):
        assert token in fast_log, (token, fast_log)
    import re
    match = re.search(r'\bB=(\d+)', fast_log)
    assert match and int(match.group(1)) > 0, 'fast must actually encode B pictures'
    for token in ('preset=normal', 'search=1024 px (local 32)', 'me=satd',
                  'me-algorithm=umh', 'long-range=compare', 'subpixel-mode=full',
                  'subpixel-range=1024', 'trellis=1'):
        assert token in norm_log, (token, norm_log)
    # The new preset is a complete option bundle, and subsequent flags win.
    assert encode('normal-then-fast', ['--me-quality', 'rd', '--bframes', '2',
                                      '--local-search-range', '32', '--fast'])[0] == fast
    assert encode('fast-explicit', ['--fast', '--subpixel-range', '8'])[0] == \
           encode('manual-explicit', [*manual_flags, '--subpixel-range', '8'])[0]
    assert encode('fast-no-b', ['--fast', '--bframes', '0'])[0] == \
           encode('manual-no-b', [*manual_flags, '--bframes', '0'])[0], 'explicit --bframes 0 must win'
    fast_full, full_log = encode('fast-full', ['--fast', '--full-subpixel-refinement'])
    assert 'subpixel-mode=full' in full_log and fast_full
    fast_off, off_log = encode('fast-off', ['--fast', '--no-subpixel-refinement'])
    assert 'subpixel-mode=off' in off_log and fast_off
    for removed in ('--faster','--fastest'):
        proc = subprocess.run([encoder, removed], capture_output=True)
        assert proc.returncode != 0 and b'was removed' in proc.stderr, (removed, proc.stderr)
    cropped_w, cropped_h, n = 80, 50, 8
    cropped = bytearray(f'YUV4MPEG2 W{cropped_w} H{cropped_h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for t in range(n):
        cropped += b'FRAME\n'
        cropped += bytes((19+x*5+y*7+(x*y)%43+t*(x%4+2))%235
                         for y in range(cropped_h) for x in range(cropped_w))
        cropped += bytes((83+x*3+y+t*2)%240
                         for y in range(cropped_h//2) for x in range(cropped_w//2))
        cropped += bytes((138+x+y*3+t*4)%240
                         for y in range(cropped_h//2) for x in range(cropped_w//2))
    cropped_data, cropped_log = encode('cropped', ['--fast'], payload=cropped)
    shape = re.search(r'TT=8x8:(\d+)/8x4:(\d+)/4x8:(\d+)/4x4:(\d+)', cropped_log)
    assert shape is not None and all(int(v)>0 for v in shape.groups()[1:]), cropped_log
    cropped_b = re.search(r'\bB=(\d+)', cropped_log)
    assert cropped_b and int(cropped_b.group(1)) > 0, cropped_log
    debug_file=root/'debug.csv'
    debug_data, _=encode('cropped-debug', ['--fast','--debug-stats',str(debug_file)], payload=cropped)
    assert cropped_data == debug_data, 'debug output changed preset bitstream'
    def simd_encode(tier):
        out=root/f'{tier}.vc1'
        cmd=[encoder,'-i','-','-o',str(out),'--threads','1','--cq','6',
             '--simd',tier,'--fast']
        proc=subprocess.run(cmd,input=cropped,capture_output=True,timeout=120)
        assert proc.returncode==0,proc.stderr.decode(errors='replace')
        return out.read_bytes()
    assert cropped_data == simd_encode('auto'), 'SIMD changed cropped bitstream'
    if sys.platform.startswith('linux'):
        import platform
        if platform.machine().lower() in ('x86_64','amd64'):
            assert cropped_data == simd_encode('x86-64-v1'), 'SSE2 changed cropped bitstream'
print('fast preset: PASS (B pictures enabled, exact explicit flags, zero-B override, transforms, cropped SIMD)')
