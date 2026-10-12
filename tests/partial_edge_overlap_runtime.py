#!/usr/bin/env python3
"""Keep decoder-signaled OVERLAP on partial edges; encode edge-friendly intra padding."""
import pathlib
import subprocess
import sys
import tempfile

encoder = pathlib.Path(sys.argv[1])
ffmpeg = sys.argv[2]


def fixture(path, w, h):
    cw, ch = (w + 1)//2, (h + 1)//2
    with path.open('wb') as f:
        f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for n in range(5):
            f.write(b'FRAME\n')
            # Construct unrelated frames with nonblack bottom/right visible
            # data to exercise P intra as well as prediction/overlap decisions.
            f.write(bytes(45 + ((x*13 + y*17 + n*37 + ((x//8) ^ (y//8))*23) % 175)
                          for y in range(h) for x in range(w)))
            f.write(bytes(40 + ((x*19+y*7+n*11) % 175) for y in range(ch) for x in range(cw)))
            f.write(bytes(40 + ((x*3+y*23+n*31) % 175) for y in range(ch) for x in range(cw)))


def encode(td, src, w, h, profile, mode, enabled):
    label = f'{w}x{h}-{profile}-{mode}-{int(enabled)}'
    suffix = 'wmv' if profile == 'main' else 'm2ts'
    out, recon, log = td / f'{label}.{suffix}', td / f'{label}.yuv', td / f'{label}.log'
    cmd = [str(encoder), '-i', str(src), '-o', str(out), '--format', 'wmv' if profile == 'main' else 'm2ts',
           '--cq', '12', '--threads', '1', '--simd', 'none', '--search-range', '0',
           '--local-search-range', '0', '--no-scene-cut', '--keyint', '999',
           '--no-loop-filter', '--no-aq', '--fixed-8x8', '--trellis', '0',
           '--recon-out', str(recon), '--overlap' if enabled else '--no-overlap']
    if mode == 'i':
        cmd += ['--intra-only']
    elif mode == 'p':
        cmd += ['--bframes', '0']
    else:
        cmd += ['--bframes', '1']
    with log.open('wb') as f:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=f)
    text = log.read_text(errors='replace')
    if ('overlap=on' in text) != enabled or ('overlap=off' in text) == enabled:
        raise RuntimeError(f'{label}: requested sequence OVERLAP was not honored')
    if 'overlap disabled for non-16-aligned picture dimensions' in text:
        raise RuntimeError(f'{label}: legacy sequence-level disable still active')
    return out.read_bytes(), recon.read_bytes(), out


with tempfile.TemporaryDirectory(prefix='libvc1-overlap-edge-') as tmp:
    td = pathlib.Path(tmp)
    scenarios = [
        (64, 48, 'advanced', 'i'),      # aligned: overlap MUST remain effective
        (64, 40, 'advanced', 'p'),      # partial bottom macroblock; P intra possible
        (56, 48, 'advanced', 'b'),      # partial right macroblock; B frames
        (56, 40, 'advanced', 'b'),      # both partial
        (65, 49, 'advanced', 'p'),      # odd display / even coded raster
        (64, 40, 'main', 'p'),          # Main cannot express CONDOVER per-edge
        (56, 48, 'main', 'i'),
        (56, 40, 'main', 'b'),
    ]
    for w, h, profile, mode in scenarios:
        src = td / f'{w}x{h}.y4m'
        if not src.exists():
            fixture(src, w, h)
        stream_on, recon_on, output = encode(td, src, w, h, profile, mode, True)
        stream_off, recon_off, _ = encode(td, src, w, h, profile, mode, False)
        # The filtered stream must remain different from --no-overlap even when
        # a visible dimension is not 16-aligned: the decoder has to see OVERLAP.
        if stream_on == stream_off or recon_on == recon_off:
            raise RuntimeError(f'{w}x{h}-{profile}-{mode}: overlap was unexpectedly disabled')
        # The decoder must produce the same display pixels as encoder recon.
        # For odd dimensions the decoder emits an even-sized coded raster;
        # that crop-equivalence case is covered by odd_dimension_runtime.py.
        if w % 2 == 0 and h % 2 == 0:
            decoded = td / 'decoded.yuv'
            subprocess.run([ffmpeg, '-hide_banner', '-v', 'error', '-xerror', '-i', str(output),
                            '-frames:v', '5', '-fps_mode', 'passthrough', '-pix_fmt', 'yuv420p',
                            '-f', 'rawvideo', '-y', str(decoded)], check=True, stdout=subprocess.DEVNULL)
            if decoded.read_bytes() != recon_on:
                raise RuntimeError(f'{w}x{h}-{profile}-{mode}: FFmpeg decoded output differs from libvc1 reconstruction')
        print(f'{w}x{h} {profile} {mode}: overlap retained and decoder parity PASS')
print('partial-edge OVERLAP coded-padding regression: PASS')
