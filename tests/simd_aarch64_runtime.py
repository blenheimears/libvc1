#!/usr/bin/env python3
"""Native AArch64 test: scalar/forced-NEON/AUTO output and unsupported override."""
import os
import pathlib
import subprocess
import sys
import tempfile

enc = sys.argv[1]
with tempfile.TemporaryDirectory() as tmp:
    tmp = pathlib.Path(tmp)
    y4m = tmp / 'source.y4m'
    width, height = 32, 32
    header = f'YUV4MPEG2 W{width} H{height} F24:1 Ip A1:1 C420jpeg\n'.encode()
    frames = []
    for t in range(3):
        luma = bytes(((x * 7 + y * 11 + t * 19 + (x ^ (y + t)) * 5) & 255)
                     for y in range(height) for x in range(width))
        chroma = bytes([96 + t * 13]) * (width * height // 4) + bytes([155 - t * 11]) * (width * height // 4)
        frames.append(b'FRAME\n' + luma + chroma)
    y4m.write_bytes(header + b''.join(frames))
    env = os.environ.copy()
    env['LIBVC1_SIMD_BENCHMARK_MS'] = '100'
    encoded = {}
    base = [enc, '-i', str(y4m), '--max-frames', '3', '--threads', '1',
            '--cq', '8', '--format', 'raw']

    def run(mode, extra=()):
        path = tmp / f'{mode}.vc1'
        return subprocess.run(base + ['-o', str(path), '--simd', mode] + list(extra),
                              text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              env=env), path

    for mode in ('none', 'aarch64-neon', 'auto'):
        result, path = run(mode)
        if result.returncode or not path.exists() or path.stat().st_size == 0:
            raise SystemExit(f'{mode} failed: {result.stderr}')
        encoded[mode] = path.read_bytes()
        if mode == 'auto' and 'AARCH64-NEON' not in result.stderr:
            raise SystemExit('AUTO did not benchmark AArch64 NEON')
    for extension in ('aarch64-dotprod', 'aarch64-rdm'):
        result, path = run(extension)
        if result.returncode:
            if any(text in result.stderr for text in ('unavailable on this CPU/build',
                                                         'reserved for a future kernel implementation')):
                print(f'{extension}: unavailable on CPU/build; forced bitstream test skipped')
                continue
            raise SystemExit(f'{extension}: unexpected failure: {result.stderr}')
        encoded[extension] = path.read_bytes()
        if encoded[extension] != encoded['none']:
            raise SystemExit(f'{extension}: forced output differs from scalar output')
        primitive = 'luma-mc' if extension == 'aarch64-dotprod' else 'intensity-luma'
        result, path = run('none', ['--simd-primitive', f'{primitive}={extension}'])
        if result.returncode or path.read_bytes() != encoded['none']:
            raise SystemExit(f'{extension}: forced primitive differs from scalar output: {result.stderr}')
    if encoded['none'] != encoded['aarch64-neon']:
        raise SystemExit('forced AArch64 NEON bitstream differs from scalar output')
    if encoded['none'] != encoded['auto']:
        raise SystemExit('AArch64 AUTO bitstream differs from scalar output')
    result, path = run('none', ['--simd-primitive', 'inverse8x8=aarch64-neon'])
    if result.returncode or path.read_bytes() != encoded['none']:
        raise SystemExit('forced NEON inverse8x8 override failed or changed bitstream: ' + result.stderr)
    auto_log = run('auto')[0].stderr
    if any(column in auto_log for column in ('X86-64-V1', 'X86-64-V2', 'X86-64-V3', 'X86-64-V4')):
        raise SystemExit('x86 benchmark columns incorrectly displayed on AArch64')
    print('AArch64 CLI: scalar/NEON/AUTO/available dot-product/RDM streams identical; x86 columns hidden')
