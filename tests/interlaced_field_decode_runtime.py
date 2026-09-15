#!/usr/bin/env python3
"""Decoder-facing regression for VC-1 interlace-field references.

The 0.1.96 implementation had two independent failures that ordinary parser
smoke tests missed:

* a backward B field wrote first-MB MVDATA before BMVTYPE, eventually causing
  FFmpeg "Bits overconsumption" near the end of the field; and
* opposite-polarity P prediction from a woven FCM=0 anchor used field-coordinate
  edge clamping. FFmpeg performs edge emulation on the progressive/woven full
  frame first, so encoder and decoder reference pictures diverged at the top
  and bottom edges even when the P bitstream parsed successfully.

The fixture forces all P/B reference classes, includes the exact 1920x1080
backward-B packet shape, checks 1920x1080 same/opposite P reconstruction byte for
byte, and checks a P->P field-reference chain. These are decoder-equivalence
checks for reference-state syntax plus a 1080i coded-residual case, not a lossy quality threshold.
"""
import subprocess
import sys
import tempfile
from pathlib import Path

if len(sys.argv) != 3:
    raise SystemExit("usage: interlaced_field_decode_runtime.py FIXTURE FFMPEG")
FIXTURE, FFMPEG = sys.argv[1:]


def decode_raw(path: Path, width: int, height: int) -> bytes:
    proc = subprocess.run(
        [FFMPEG, "-v", "error", "-xerror", "-f", "vc1", "-i", str(path),
         "-pix_fmt", "yuv420p", "-f", "rawvideo", "-"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if proc.returncode:
        raise RuntimeError(f"FFmpeg rejected {path.name}:\n{proc.stderr.decode(errors='replace')}")
    frame_size = width * height * 3 // 2
    if len(proc.stdout) % frame_size:
        raise RuntimeError(f"decoded {path.name} has a partial raw frame: {len(proc.stdout)} bytes")
    return proc.stdout


with tempfile.TemporaryDirectory(prefix="libvc1-field-refs-") as d:
    root = Path(d)
    subprocess.run([FIXTURE, str(root)], check=True)
    expected = {
        "i-frame-interlace.vc1",
        "i-frame-interlace-flat.vc1",
        "p-same-opposite.vc1",
        "p-field-chain.vc1",
        "p-sparse-opposite-chroma.vc1",
        "p-1080i-same.vc1",
        "p-1080i-opposite.vc1",
        "p-1080i-residual.vc1",
        "b-forward-same.vc1",
        "b-forward-opposite.vc1",
        "b-backward-same.vc1",
        "b-backward-opposite.vc1",
        "b-mixed-forward-backward-raw.vc1",
        "b-1080i-backward-same.vc1",
    }
    got = {p.name for p in root.glob("*.vc1")}
    if got != expected:
        raise RuntimeError(f"field fixture set mismatch: expected {sorted(expected)}, got {sorted(got)}")

    # Every syntax class must be accepted completely by the decoder.  Cache
    # the raw output because the exact-equivalence checks below consume the
    # same frames; the former two-pass harness unnecessarily decoded all
    # 1080i fixtures twice.
    decoded = {}
    for name in sorted(expected):
        p = root / name
        if "1080i" in name:
            dims = (1920, 1080)
        elif name == "p-sparse-opposite-chroma.vc1":
            dims = (256, 128)
        elif name == "b-mixed-forward-backward-raw.vc1":
            dims = (128, 128)
        else:
            dims = (64, 96)
        decoded[name] = decode_raw(p, *dims)

    # The FCM=10 I anchor must itself be byte-identical to the encoder's
    # retained reconstruction before it is permitted to seed P/B references.
    for i_name in ("i-frame-interlace.vc1", "i-frame-interlace-flat.vc1"):
        i_raw = decoded[i_name]
        i_wanted = (root / (i_name + ".expected.yuv")).read_bytes()
        if i_raw != i_wanted:
            diffs = sum(a != b for a, b in zip(i_raw, i_wanted)) + abs(len(i_raw) - len(i_wanted))
            raise RuntimeError(f"{i_name} decoder reconstruction differs from encoder reference ({diffs} bytes)")

    # All four B reference classes are also zero-residual predictions.  The
    # coded stream order is past-I, future-I, B; decoder presentation order is
    # past-I, B, future-I, so the middle frame must match the encoder reference.
    small_frame = 64 * 96 * 3 // 2
    for name in ("b-forward-same.vc1", "b-forward-opposite.vc1",
                 "b-backward-same.vc1", "b-backward-opposite.vc1"):
        raw = decoded[name]
        if len(raw) != 3 * small_frame:
            raise RuntimeError(f"{name} decoded {len(raw) // small_frame} frames, expected 3")
        actual = raw[small_frame:2 * small_frame]
        wanted = (root / (name + ".expected.yuv")).read_bytes()
        if actual != wanted:
            diffs = sum(a != b for a, b in zip(actual, wanted)) + abs(len(actual) - len(wanted))
            raise RuntimeError(f"{name} decoder reconstruction differs from encoder reference ({diffs} bytes)")

    # A mixed forward/backward B-field map must also be exact when the
    # FORWARDMB bitplane chooses IMODE_RAW.  The 0.1.104 encoder wrote the RAW
    # header but omitted the per-MB direction bits, so MBMODE/BMVTYPE parsing
    # desynchronized only on sufficiently mixed B fields/GOPs.
    mixed_name = "b-mixed-forward-backward-raw.vc1"
    mixed_raw = decoded[mixed_name]
    mixed_frame = 128 * 128 * 3 // 2
    if len(mixed_raw) != 3 * mixed_frame:
        raise RuntimeError(f"{mixed_name} decoded {len(mixed_raw) // mixed_frame} frames, expected 3")
    mixed_actual = mixed_raw[mixed_frame:2 * mixed_frame]
    mixed_wanted = (root / (mixed_name + ".expected.yuv")).read_bytes()
    if mixed_actual != mixed_wanted:
        diffs = sum(a != b for a, b in zip(mixed_actual, mixed_wanted))
        raise RuntimeError(f"{mixed_name} RAW FORWARDMB reconstruction drifted ({diffs} bytes)")

    # Sparse second-field opposite-reference chroma must be decoder-exact.
    # This catches FASTUVMC/parity-order drift that leaves luma correct but
    # produces field-height brown/DCT-like chroma blocks on real material.
    sparse_name = "p-sparse-opposite-chroma.vc1"
    sparse_raw = decoded[sparse_name]
    sparse_frame = 256 * 128 * 3 // 2
    if len(sparse_raw) != 2 * sparse_frame:
        raise RuntimeError(f"{sparse_name} decoded {len(sparse_raw) // sparse_frame} frames, expected 2")
    sparse_actual = sparse_raw[sparse_frame:]
    sparse_wanted = (root / (sparse_name + ".expected.yuv")).read_bytes()
    if sparse_actual != sparse_wanted:
        diffs = sum(a != b for a, b in zip(sparse_actual, sparse_wanted))
        raise RuntimeError(f"{sparse_name} opposite-field chroma reconstruction drifted ({diffs} bytes)")

    # The 1080i backward/same B reproducer is also an exact reference test.
    # Keeping a full-size case here proves the edge-safe search policy is not
    # merely a workaround for the compact syntax fixtures.
    b1080_name = "b-1080i-backward-same.vc1"
    b1080_raw = decoded[b1080_name]
    hd_frame = 1920 * 1080 * 3 // 2
    if len(b1080_raw) != 3 * hd_frame:
        raise RuntimeError(f"{b1080_name} decoded {len(b1080_raw) // hd_frame} frames, expected 3")
    b1080_actual = b1080_raw[hd_frame:2 * hd_frame]
    b1080_wanted = (root / (b1080_name + ".expected.yuv")).read_bytes()
    if b1080_actual != b1080_wanted:
        diffs = sum(a != b for a, b in zip(b1080_actual, b1080_wanted))
        raise RuntimeError(f"{b1080_name} decoder reconstruction differs from encoder reference ({diffs} bytes)")

    # Zero-residual P prediction has an exact expected reconstruction.  Compare
    # against FFmpeg rather than against the source so this checks codec
    # semantics/reference state without imposing a quality target.
    for name in ("p-1080i-same.vc1", "p-1080i-opposite.vc1", "p-1080i-residual.vc1"):
        raw = decoded[name]
        if len(raw) != 2 * hd_frame:
            raise RuntimeError(f"{name} decoded {len(raw) // hd_frame} frames, expected 2")
        actual = raw[hd_frame:]
        wanted = (root / (name + ".expected.yuv")).read_bytes()
        if actual != wanted:
            diffs = sum(a != b for a, b in zip(actual, wanted)) + abs(len(actual) - len(wanted))
            raise RuntimeError(f"{name} decoder reconstruction differs from encoder reference ({diffs} bytes)")

    chain_name = "p-field-chain.vc1"
    chain_frame = 64 * 96 * 3 // 2
    raw = decoded[chain_name]
    if len(raw) != 3 * chain_frame:
        raise RuntimeError(f"{chain_name} decoded {len(raw) // chain_frame} frames, expected 3")
    actual = raw[chain_frame:]
    wanted = (root / (chain_name + ".expected.yuv")).read_bytes()
    if actual != wanted:
        diffs = sum(a != b for a, b in zip(actual, wanted)) + abs(len(actual) - len(wanted))
        raise RuntimeError(f"{chain_name} P->P decoder reference chain drifted ({diffs} bytes)")

    print("interlaced field decoder regression ok: P same/opposite + sparse chroma + P chain exact, all four B references, 1080i backward-B")
