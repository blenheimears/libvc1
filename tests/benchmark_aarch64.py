#!/usr/bin/env python3
"""Manual, opt-in end-to-end CPU-time comparison of AArch64 NEON releases.

Example:
  python3 tests/benchmark_aarch64.py film.y4m --baseline /path/to/0.2.52/vc1enc \
      --candidate ./build/vc1enc --frames 240 --runs 3 --scratch /dev/shm

The script is not part of CTest: elapsed time, scheduling, clock speeds and
core count are machine-specific. Forced and AUTO output must match scalar.
"""
import argparse
import hashlib
import pathlib
import resource
import statistics
import subprocess
import tempfile
import time


def measure(executable, input_path, output, mode, frames, extra):
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    start = time.monotonic()
    cmd = [str(executable), '-i', str(input_path), '-o', str(output),
           '--max-frames', str(frames), '--threads', '1', '--cq', '8',
           '--format', 'raw', '--simd', mode] + extra
    p = subprocess.run(cmd, capture_output=True, text=True)
    wall = time.monotonic() - start
    after = resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu = (after.ru_utime + after.ru_stime) - (before.ru_utime + before.ru_stime)
    if p.returncode != 0 or not output.is_file() or output.stat().st_size == 0:
        raise RuntimeError(f'{executable} {mode} exited {p.returncode}:\n{p.stderr[-4000:]}')
    with output.open('rb') as encoded:
        digest = hashlib.file_digest(encoded, 'sha256').hexdigest()
    output.unlink()
    return cpu, wall, digest


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('input', type=pathlib.Path, help='Existing Y4M clip; same input for both versions')
    ap.add_argument('--baseline', type=pathlib.Path, help='Optional older vc1enc (e.g. 0.2.52)')
    ap.add_argument('--candidate', type=pathlib.Path, default=pathlib.Path('build/vc1enc'))
    ap.add_argument('--frames', type=int, default=240)
    ap.add_argument('--runs', type=int, default=3)
    ap.add_argument('--scratch', type=pathlib.Path, default=None,
                    help='Output scratch directory; /dev/shm can eliminate disk output effects')
    opts, extra = ap.parse_known_args()
    if opts.frames < 1 or opts.runs < 1 or not opts.input.is_file():
        ap.error('Specify a readable input and positive --frames / --runs')
    if extra and extra[0] == '--':
        extra = extra[1:]
    input_path = opts.input.resolve()
    executables = [('baseline', opts.baseline), ('candidate', opts.candidate)]
    outcomes = {}
    with tempfile.TemporaryDirectory(prefix='libvc1-arm-bench-', dir=opts.scratch) as folder:
        output = pathlib.Path(folder) / 'out.vc1'
        for label, executable in executables:
            if executable is None:
                continue
            executable = executable.resolve()
            if not executable.is_file():
                ap.error(f'Executable missing: {executable}')
            results = {}
            hashes = {}
            for mode in ('none', 'aarch64-neon', 'auto'):
                # One untimed warm-up per mode; benchmark initialization is
                # still part of each timed AUTO sample, not silently excluded.
                _, _, warm_hash = measure(executable, input_path, output, mode, opts.frames, extra)
                data = [measure(executable, input_path, output, mode, opts.frames, extra)
                        for _ in range(opts.runs)]
                if any(digest != warm_hash for _, _, digest in data):
                    raise RuntimeError(f'{label}: non-deterministic bitstream in {mode}')
                hashes[mode] = warm_hash
                results[mode] = (statistics.median(row[0] for row in data),
                                 statistics.median(row[1] for row in data))
            if len(set(hashes.values())) != 1:
                raise RuntimeError(f'{label}: scalar, forced NEON and AUTO output differs: {hashes}')
            print(f'{label}: {executable} (scalar/NEON/AUTO bitstreams identical)')
            for mode, (cpu, wall) in results.items():
                print(f'  {mode:13s} CPU {cpu:8.3f}s   wall {wall:8.3f}s   '
                      f'scalar/this CPU-speedup {results["none"][0]/cpu:.3f}x')
            outcomes[label] = (results, hashes['none'])
    if len(outcomes) == 2:
        old, old_hash = outcomes['baseline']
        new, new_hash = outcomes['candidate']
        if old_hash != new_hash:
            raise RuntimeError('Baseline and candidate produce different raw VC-1 bitstreams')
        print('Cross-release raw bitstreams identical.')
        for mode in ('none', 'aarch64-neon', 'auto'):
            print(f'  Candidate vs baseline {mode:13s} CPU-speedup '
                  f'{old[mode][0]/new[mode][0]:.3f}x')
    print('CPU time is aggregate user+system time; AUTO timings include its startup benchmark.')


if __name__ == '__main__':
    main()
