#!/usr/bin/env python3
"""Run an x86 SIMD test natively and/or under a constrained qemu-x86_64 CPU.

Modes:
  fallback: run natively first; only native exit 77 is retried under QEMU.
  force:    always run under QEMU, useful for detecting ISA leakage.

Exit 77 means CTest skip.  QEMU model/feature combinations which are not
available in the installed QEMU are also reported as skips.  Once QEMU has
successfully exposed the requested target, a SIGILL/test failure is a real
failure: that is how force mode detects accidental higher-ISA instructions.
"""
import argparse
import os
import subprocess
import sys

# These profiles intentionally model the *target ISA*, not the build host.
# Named models provide natural upper bounds.  AMD G4/G5 get explicit XOP/FMA4
# because those flags are the reason these targets need emulation on many hosts.
CPU_PROFILES = {
    # qemu64 is not guaranteed to be a strict psABI-v1 mask, so explicitly
    # remove every feature which can make one of libvc1's higher targets match.
    "v1": ["qemu64,pni=off,ssse3=off,sse4.1=off,sse4.2=off,popcnt=off,sse4a=off,abm=off,avx=off,avx2=off,fma=off,fma4=off,xop=off,f16c=off,bmi1=off,bmi2=off,movbe=off,avx512f=off,avx512bw=off,avx512cd=off,avx512dq=off,avx512vl=off"],
    "prescott": ["qemu64,pni=on,ssse3=off,sse4.1=off,sse4.2=off,popcnt=off,sse4a=off,abm=off,avx=off,avx2=off,fma=off,fma4=off,xop=off,f16c=off,bmi1=off,bmi2=off,movbe=off,avx512f=off"],
    "k10": [
        "Opteron_G3,pni=on,sse4a=on,popcnt=on,abm=on,ssse3=off,avx=off,avx2=off,xop=off,fma4=off",
        "max,pni=on,sse4a=on,popcnt=on,abm=on,ssse3=off,sse4.1=off,sse4.2=off,avx=off,avx2=off,xop=off,fma4=off,fma=off,f16c=off,bmi1=off,bmi2=off,avx512f=off",
    ],
    "conroe": ["qemu64,pni=on,ssse3=on,sse4.1=off,sse4.2=off,popcnt=off,avx=off,avx2=off,fma=off,f16c=off,bmi1=off,bmi2=off,avx512f=off"],
    "penryn": ["Penryn"],
    "v2": ["Nehalem"],
    "sandybridge": ["SandyBridge"],
    # Opteron_G4/G5 are the natural Bulldozer/Piledriver QEMU models, but XOP
    # and FMA4 are requested explicitly so their presence is never accidental
    # or dependent on a QEMU model-version default.
    "bulldozer": [
        "Opteron_G4,xop=on,fma4=on,abm=on,avx=on,fma=off,f16c=off,bmi1=off,avx2=off",
        "max,pni=on,ssse3=on,sse4.1=on,sse4.2=on,popcnt=on,abm=on,avx=on,xop=on,fma4=on,fma=off,f16c=off,bmi1=off,bmi2=off,avx2=off,avx512f=off",
    ],
    "piledriver": [
        "Opteron_G5,xop=on,fma4=on,fma=on,f16c=on,bmi1=on,abm=on,avx=on,avx2=off,bmi2=off",
        "max,pni=on,ssse3=on,sse4.1=on,sse4.2=on,popcnt=on,abm=on,avx=on,xop=on,fma4=on,fma=on,f16c=on,bmi1=on,bmi2=off,avx2=off,avx512f=off",
    ],
    # Deliberately remove the v3-only companions. AVX2 must work without an
    # accidental FMA/F16C/BMI dependency in this partial target.
    "avx2-partial": ["Haswell,fma=off,f16c=off,bmi1=off,bmi2=off,avx2=on"],
    "v3": ["Haswell"],
    "v4": ["Skylake-Server"],
}

QEMU_CONFIG_FAILURE_MARKERS = (
    "unable to find cpu model",
    "unable to find cpu definition",
    "property '.cpu",
    "property 'x86_64-cpu",
    "can't apply global",
    "could not find",
    "invalid parameter",
    "invalid cpu",
    "requested feature",
    "tcg doesn't support requested feature",
    "warning: host doesn't support requested feature",
)


def run(cmd):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def emit(p):
    if p.stdout:
        sys.stdout.write(p.stdout)
    if p.stderr:
        sys.stderr.write(p.stderr)


def qemu_config_failure(p):
    text = (p.stdout + "\n" + p.stderr).lower()
    return any(marker in text for marker in QEMU_CONFIG_FAILURE_MARKERS)


def qemu_command(qemu, cpu, exe, rest):
    return [qemu, "-cpu", cpu, exe, *rest]


def try_profile(qemu, cpu, probe, target):
    p = run(qemu_command(qemu, cpu, probe, [target]))
    if qemu_config_failure(p):
        return False, p
    if p.returncode == 0:
        return True, p
    # 78 is emitted by the probe when this QEMU profile does not expose all
    # required target features.  It is eligible for a different profile or,
    # after all profiles are exhausted, a normal CTest skip.
    if p.returncode == 78 or qemu_config_failure(p):
        return False, p
    emit(p)
    sys.stderr.write(f"libvc1: emulated CPU detection failed for {target} with -cpu {cpu} (exit {p.returncode})\n")
    raise SystemExit(p.returncode if p.returncode > 0 else 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("fallback", "force"), required=True)
    ap.add_argument("--target", choices=tuple(CPU_PROFILES), required=True)
    ap.add_argument("--qemu", required=True)
    ap.add_argument("--probe", required=True)
    ap.add_argument("executable")
    ap.add_argument("args", nargs=argparse.REMAINDER)
    ns = ap.parse_args()

    if ns.mode == "fallback":
        native = subprocess.run([ns.executable, *ns.args])
        if native.returncode != 77:
            return native.returncode
        sys.stderr.write(f"libvc1: {ns.target} unavailable natively; retrying with qemu-user\n")

    last = None
    selected = None
    for cpu in CPU_PROFILES[ns.target]:
        ok, p = try_profile(ns.qemu, cpu, ns.probe, ns.target)
        last = p
        if ok:
            selected = cpu
            break
    if selected is None:
        if last is not None:
            emit(last)
        sys.stderr.write(f"libvc1: SKIP: installed QEMU cannot expose the {ns.target} test CPU\n")
        return 77

    if ns.mode == "force":
        sys.stderr.write(f"libvc1: force-emulating {ns.target} with qemu-user -cpu {selected}\n")
    else:
        sys.stderr.write(f"libvc1: emulating unsupported host target {ns.target} with -cpu {selected}\n")

    p = subprocess.run(qemu_command(ns.qemu, selected, ns.executable, ns.args))
    if p.returncode == 77:
        # The independent probe just verified that this target is visible.  A
        # target test now claiming otherwise means its own gate is inconsistent.
        sys.stderr.write(f"libvc1: ERROR: {ns.target} probe passed but the SIMD test self-gate returned 77\n")
        return 79
    return p.returncode


if __name__ == "__main__":
    raise SystemExit(main())
