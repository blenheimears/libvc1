#!/usr/bin/env python3
"""VC-1 Advanced Profile interlace-field functional regression.

This test intentionally does not establish a quality threshold.  It verifies
that interlaced input reaches true field-picture P/B syntax, P fields enable
both reference fields with NUMREF=1, FRAME/FIELD BDUs stay paired, TFF/BFF is
preserved, and FFmpeg decodes every source frame without bitstream errors.
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

if len(sys.argv) != 4:
    raise SystemExit("usage: interlaced_runtime.py ENCODER FFMPEG FFPROBE")
ENCODER, FFMPEG, FFPROBE = sys.argv[1:]
W, H = 64, 64
FRAME_SIZE = W * H * 3 // 2


def make_frames(nframes: int, duplicate_last: bool = False):
    frames = []
    for n in range(nframes):
        if duplicate_last and n == nframes - 1:
            frames.append(frames[-1])
            continue
        y = bytearray(W * H)
        for yy in range(H):
            field = yy & 1
            for x in range(W):
                base = 48 if field == 0 else 184
                stripe = ((x // 8 + n + field * 2) % 4) * 10
                y[yy * W + x] = max(16, min(235, base + stripe))
        u = bytes([96 + n * 2]) * (W * H // 4)
        v = bytes([160 - n * 2]) * (W * H // 4)
        frames.append(bytes(y) + u + v)
    return frames


def write_y4m(path: Path, scan: str, frames):
    with path.open("wb") as f:
        f.write(f"YUV4MPEG2 W{W} H{H} F30000:1001 I{scan} A1:1 C420mpeg2\n".encode())
        for frame in frames:
            f.write(b"FRAME\n")
            f.write(frame)


def ebdu_payload(data: bytes, start: int, end: int):
    src = data[start:end]
    out = bytearray()
    zeros = 0
    i = 0
    while i < len(src):
        b = src[i]
        if zeros >= 2 and b == 0x03 and i + 1 < len(src) and src[i + 1] <= 0x03:
            zeros = 0
            i += 1
            continue
        out.append(b)
        zeros = zeros + 1 if b == 0 else 0
        i += 1
    return bytes(out)


def units(data: bytes):
    found = []
    p = 0
    while True:
        q = data.find(b"\x00\x00\x01", p)
        if q < 0 or q + 3 >= len(data):
            break
        found.append((q, data[q + 3]))
        p = q + 4
    out = []
    for i, (off, code) in enumerate(found):
        end = found[i + 1][0] if i + 1 < len(found) else len(data)
        out.append((code, ebdu_payload(data, off + 4, end)))
    return out


class Bits:
    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0

    def get(self, n=1):
        if self.pos + n > len(self.data) * 8:
            raise RuntimeError("truncated VC-1 header while checking interlace syntax")
        v = 0
        for _ in range(n):
            v = (v << 1) | ((self.data[self.pos >> 3] >> (7 - (self.pos & 7))) & 1)
            self.pos += 1
        return v

    def decode012(self):
        if self.get() == 0:
            return 0
        return 2 if self.get() else 1


def parse_ptype(bits: Bits):
    if bits.get() == 0:
        return "P"
    if bits.get() == 0:
        return "B"
    if bits.get() == 0:
        return "I"
    if bits.get() == 0:
        return "BI"
    return "S"


def verify_sequence(seq_payload: bytes):
    sb = Bits(seq_payload)
    sb.get(2 + 3 + 2 + 3 + 5 + 1 + 12 + 12)
    pulldown = sb.get()
    interlace = sb.get()
    if (pulldown, interlace) != (1, 1):
        raise RuntimeError(f"sequence did not signal PULLDOWN=1/INTERLACE=1: {(pulldown, interlace)}")


def verify_first_p_field(payload: bytes, top_first: bool):
    b = Bits(payload)
    if b.decode012() != 2 or b.get(3) != 0b011:
        raise RuntimeError("P picture did not use FCM=11 / P-P field-picture syntax")
    tff, rff, rnd, uvsamp = b.get(), b.get(), b.get(), b.get()
    if tff != int(top_first) or rff or rnd or uvsamp:
        raise RuntimeError(f"bad P-field prefix: TFF={tff} RFF={rff} RND={rnd} UVSAMP={uvsamp}")
    pq = b.get(5)
    if pq <= 8:
        b.get()  # HALFQP
    b.get()      # PQUANTIZER
    numref = b.get()
    if numref != 1:
        raise RuntimeError("interlaced P field did not signal NUMREF=1 (two reference fields)")


def verify_bitstream(es: Path, top_first: bool, want_b: bool, want_skipped: bool):
    us = units(es.read_bytes())
    seqs = [p for c, p in us if c == 0x0F]
    if not seqs:
        raise RuntimeError("VC-1 sequence header not found")
    verify_sequence(seqs[0])

    kinds = []
    field_pairs = 0
    p_pairs = 0
    b_pairs = 0
    for i, (code, payload) in enumerate(us):
        if code != 0x0D:
            continue
        b = Bits(payload)
        fcm = b.decode012()
        if fcm == 2:
            fptype = b.get(3)
            if fptype == 0b011:
                kind = "P"
                verify_first_p_field(payload, top_first)
                p_pairs += 1
            elif fptype == 0b100:
                kind = "B"
                # Prefix check is enough here; the dedicated reference-selection
                # test forces all four B reference classes independently.
                tff, rff = b.get(), b.get()
                if tff != int(top_first) or rff:
                    raise RuntimeError(f"bad B-field display flags: TFF={tff} RFF={rff}")
                b_pairs += 1
            else:
                raise RuntimeError(f"unexpected interlace-field FPTYPE {fptype:03b}")
            if i + 1 >= len(us) or us[i + 1][0] != 0x0C:
                raise RuntimeError(f"{kind} first field was not immediately followed by a FIELD BDU")
            field_pairs += 1
            kinds.append(kind)
        elif fcm == 1:
            kind = parse_ptype(b)
            kinds.append(kind)
            if kind not in ("I", "BI"):
                raise RuntimeError(f"FCM=10 picture was not an interlaced I/BI anchor: {kind}")
            tff, rff = b.get(), b.get()
            if tff != int(top_first) or rff:
                raise RuntimeError(f"bad frame-interlaced anchor flags for {kind}: TFF={tff} RFF={rff}")
        elif fcm == 0:
            kind = parse_ptype(b)
            kinds.append(kind)
            if kind != "S":
                raise RuntimeError(f"interlaced coded picture unexpectedly fell back to FCM=0: {kind}")
            tff, rff = b.get(), b.get()
            if tff != int(top_first) or rff:
                raise RuntimeError(f"bad skipped-picture flags: TFF={tff} RFF={rff}")
        else:
            raise RuntimeError(f"unexpected FCM={fcm} in interlace regression")

    if "I" not in kinds or "P" not in kinds or p_pairs == 0 or field_pairs == 0:
        raise RuntimeError(f"interlace regression did not exercise FCM=10 I plus FCM=11 field P pictures: {kinds}")
    if want_b and ("B" not in kinds or b_pairs == 0):
        raise RuntimeError(f"interlace regression did not exercise B/B field pictures: {kinds}")
    if want_skipped and "S" not in kinds:
        raise RuntimeError(f"interlace regression did not exercise a skipped picture: {kinds}")
    return kinds


def run_case(tmp: Path, scan: str, top_first: bool, bframes: int, duplicate_last: bool):
    tag = "tff" if top_first else "bff"
    nframes = 6
    src_frames = make_frames(nframes, duplicate_last=duplicate_last)
    src = tmp / f"{tag}.y4m"
    out = tmp / f"{tag}.m2ts"
    es = tmp / f"{tag}.vc1"
    dec = tmp / f"{tag}.yuv"
    write_y4m(src, scan, src_frames)

    # Keep syntax parsing deterministic: no extended-MV picture field, no
    # DQUANT or variable-transform picture fields. Those are independently
    # covered elsewhere and are orthogonal to field reference semantics.
    cmd = [ENCODER, "-i", str(src), "-o", str(out), "--es-out", str(es),
           "--cq", "5", "--threads", "1", "--no-simd", "--no-aq",
           "--bframes", str(bframes), "--keyint", str(nframes), "--no-scene-cut",
           "--search-range", "16", "--local-search-range", "16", "--no-dquant", "--fixed-8x8"]
    subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    kinds = verify_bitstream(es, top_first, want_b=bframes > 0, want_skipped=duplicate_last and bframes == 0)
    # With scene detection disabled and keyint equal to the complete fixture,
    # interlaced motion analysis must not turn combing into repeated I pictures.
    if kinds.count("I") != 1:
        raise RuntimeError(f"interlaced GOP unexpectedly contains {kinds.count('I')} I pictures: {kinds}")

    probe = subprocess.check_output([
        FFPROBE, "-v", "error", "-select_streams", "v:0", "-count_frames",
        "-show_entries", "stream=width,height,field_order,nb_read_frames", "-of", "json", str(out)
    ], text=True)
    streams = json.loads(probe).get("streams", [])
    if len(streams) != 1:
        raise RuntimeError(f"ffprobe found {len(streams)} video streams")
    st = streams[0]
    if int(st.get("width", 0)) != W or int(st.get("height", 0)) != H:
        raise RuntimeError(f"decoded dimensions changed: {st}")
    if int(st.get("nb_read_frames", -1)) != nframes:
        raise RuntimeError(f"decoded frame count changed: {st}")
    expected_order = "tt" if top_first else "bb"
    if st.get("field_order") != expected_order:
        raise RuntimeError(f"field order was not preserved: expected {expected_order}, got {st.get('field_order')}")

    # -xerror turns decoder complaints into a test failure. We deliberately do
    # not compare reconstruction quality in this release; the test is about
    # standards-valid field syntax/reference operation and complete decode.
    subprocess.run([FFMPEG, "-v", "error", "-xerror", "-i", str(out), "-map", "0:v:0",
                    "-pix_fmt", "yuv420p", "-f", "rawvideo", "-y", str(dec)], check=True)
    raw = dec.read_bytes()
    if len(raw) != nframes * FRAME_SIZE:
        raise RuntimeError(f"decoded raw size implies a wrong frame count: {len(raw)}")

    # This is a functionality/correspondence guard, not a quality benchmark.
    # Every non-duplicate source frame must cause a decoded presentation-frame
    # change, and decoded frames must remain broadly associated with the same
    # source index. The deliberately loose MAE ceiling catches frozen/reordered
    # fields without establishing a codec-quality target.
    decoded = [raw[i * FRAME_SIZE:(i + 1) * FRAME_SIZE] for i in range(nframes)]
    for i, (got, want) in enumerate(zip(decoded, src_frames)):
        mae = sum(abs(a - b) for a, b in zip(got, want)) / FRAME_SIZE
        if mae >= 48.0:
            raise RuntimeError(f"decoded frame {i} lost source correspondence (MAE={mae:.2f})")
        if i and src_frames[i] != src_frames[i - 1] and got == decoded[i - 1]:
            raise RuntimeError(f"moving interlaced frame {i} froze instead of changing on its P/B picture")


with tempfile.TemporaryDirectory(prefix="libvc1-interlace-") as d:
    tmp = Path(d)
    run_case(tmp, "t", True, bframes=1, duplicate_last=False)
    run_case(tmp, "b", False, bframes=0, duplicate_last=True)
    print("interlaced field runtime ok: TFF/BFF, one-I GOP, moving P/B residuals, P NUMREF=1, field pairs, skipped picture, full FFmpeg decode")
