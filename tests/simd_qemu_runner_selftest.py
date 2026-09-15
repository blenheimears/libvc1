#!/usr/bin/env python3
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile

HERE=Path(__file__).resolve().parent
RUNNER=HERE/'run_simd_isa_test.py'
spec=importlib.util.spec_from_file_location('simd_runner',RUNNER)
mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)

assert 'xop=on' in mod.CPU_PROFILES['bulldozer'][0] and 'fma4=on' in mod.CPU_PROFILES['bulldozer'][0]
assert 'xop=on' in mod.CPU_PROFILES['piledriver'][0] and 'fma4=on' in mod.CPU_PROFILES['piledriver'][0]
assert 'fma=off' in mod.CPU_PROFILES['avx2-partial'][0] and 'bmi1=off' in mod.CPU_PROFILES['avx2-partial'][0]
assert mod.CPU_PROFILES['v2'][0] == 'Nehalem'
assert mod.CPU_PROFILES['v3'][0] == 'Haswell'
assert mod.CPU_PROFILES['v4'][0] == 'Skylake-Server'

with tempfile.TemporaryDirectory() as td:
    td=Path(td); log=td/'qemu.log'
    def script(name, body):
        p=td/name; p.write_text('#!/bin/sh\nset -eu\n'+body); p.chmod(0o755); return p
    qemu=script('qemu-x86_64', f'''echo "$*" >> "{log}"
[ "$1" = -cpu ]
shift 2
UNDER_QEMU=1 exec "$@"
''')
    probe=script('probe', 'test "${UNDER_QEMU:-}" = 1\nexit 0\n')
    native_ok=script('native-ok', 'exit 0\n')
    gated=script('gated', '[ "${UNDER_QEMU:-}" = 1 ] && exit 0\nexit 77\n')

    # Fallback leaves a natively compatible test completely native.
    p=subprocess.run([sys.executable,str(RUNNER),'--mode','fallback','--target','v2','--qemu',str(qemu),'--probe',str(probe),str(native_ok)])
    assert p.returncode == 0 and not log.exists()

    # Unsupported native ISA is retried under QEMU after the detection probe.
    p=subprocess.run([sys.executable,str(RUNNER),'--mode','fallback','--target','v4','--qemu',str(qemu),'--probe',str(probe),str(gated)])
    assert p.returncode == 0 and log.read_text().count('-cpu Skylake-Server') == 2
    log.unlink()

    # Force mode must not execute the target test natively.
    p=subprocess.run([sys.executable,str(RUNNER),'--mode','force','--target','v3','--qemu',str(qemu),'--probe',str(probe),str(gated)])
    assert p.returncode == 0 and log.read_text().count('-cpu Haswell') == 2

print('qemu SIMD runner self-test OK')
