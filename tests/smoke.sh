#!/usr/bin/env bash
# Do not use `set -e`: the smoke suite is intentionally failure-accumulating.
# A failed check is reported immediately and recorded, but the remaining tests
# continue so one Nix check run exposes every regression it can reach.
set -E -o pipefail

ENCODER=${1:-./build/vc1enc}
FFMPEG=${FFMPEG:-ffmpeg}
FFPROBE=${FFPROBE:-ffprobe}
TMP=${TMPDIR:-/tmp}/libvc1-smoke-$$
# Production auto-dispatch benchmarks scalar plus available AVX2/AVX-512 tiers for about two seconds.
# Keep the regression suite fast while still exercising the same selector.
LIBVC1_SIMD_BENCHMARK_MS=${LIBVC1_SIMD_BENCHMARK_MS:-40}
export LIBVC1_SIMD_BENCHMARK_MS

SMOKE_SECTION="startup / base Advanced Profile"
SMOKE_FAILURES="$TMP/failures.log"
SMOKE_FAILURE_KEYS="$TMP/failure-keys.log"
mkdir -p "$TMP"
: >"$SMOKE_FAILURES"
: >"$SMOKE_FAILURE_KEYS"

smoke_section() {
  SMOKE_SECTION=$1
}

smoke_record_failure() {
  local line=$1 rc=$2 detail=$3 key
  key="$line|$SMOKE_SECTION|$detail"
  # ERR can fire once in a command-substitution subshell and again for the
  # containing assignment. Deduplicate by source line/section/text.
  if ! grep -Fqx -- "$key" "$SMOKE_FAILURE_KEYS" 2>/dev/null; then
    printf '%s\n' "$key" >>"$SMOKE_FAILURE_KEYS"
    printf 'FAIL [%s] line %s (exit %s): %s\n' \
      "$SMOKE_SECTION" "$line" "$rc" "$detail" >&2
    printf '[%s] line %s (exit %s): %s\n' \
      "$SMOKE_SECTION" "$line" "$rc" "$detail" >>"$SMOKE_FAILURES"
  fi
}

smoke_err_trap() {
  local rc=$? line detail
  line=${BASH_LINENO[0]:-$LINENO}
  trap - ERR
  detail=$(sed -n "${line}p" "$0" 2>/dev/null)
  detail=${detail#${detail%%[![:space:]]*}}
  [ -n "$detail" ] || detail=${BASH_COMMAND:-unknown command}
  smoke_record_failure "$line" "$rc" "$detail"
  trap 'smoke_err_trap' ERR
  return 0
}

smoke_fail() {
  local rc=${2:-1} line=${BASH_LINENO[0]:-$LINENO}
  smoke_record_failure "$line" "$rc" "$1"
  return 0
}

smoke_finish() {
  trap - ERR
  if [ -s "$SMOKE_FAILURES" ]; then
    local count
    count=$(wc -l <"$SMOKE_FAILURES")
    printf '\nSmoke test suite: FAIL (%s failure%s)\n' "$count" \
      "$([ "$count" -eq 1 ] && printf '' || printf 's')" >&2
    printf '%s\n' '---- failure summary ----' >&2
    sed 's/^/  - /' "$SMOKE_FAILURES" >&2
    return 1
  fi
  printf '\nSmoke test suite: PASS (no failures)\n'
  return 0
}

cleanup_smoke() { rm -rf "$TMP"; }
trap cleanup_smoke EXIT
trap 'smoke_err_trap' ERR

# Fast behavioral self-test for the failure-accumulating harness itself.
# The worker deliberately returns failure after proving execution continued;
# the parent verifies the complete diagnostics and then returns success to CTest.
if [ "${1:-}" = "--self-test-harness-worker" ]; then
  smoke_section "harness self-test"
  false
  false | true
  smoke_fail "intentional explicit self-test failure"
  printf 'self-test continued after intentional failures\n'
  smoke_finish
  exit $?
fi
if [ "${1:-}" = "--self-test-harness" ]; then
  trap - ERR
  if grep -Eq '^set[[:space:]]+-[^[:space:]]*[eu]' "$0"; then
    printf 'smoke harness self-test found fail-fast set -e/-u in smoke.sh\n' >&2
    exit 1
  fi
  normal_body=$(sed -n '/^CQ9=/,$p' "$0")
  if printf '%s\n' "$normal_body" | grep -Eq '^[[:space:]]*exit[[:space:]]+[1-9]|;[[:space:]]*exit[[:space:]]+[1-9]'; then
    printf 'smoke harness self-test found direct nonzero shell exit in normal test body\n' >&2
    exit 1
  fi
  if output=$("${BASH:-bash}" "$0" --self-test-harness-worker 2>&1); then
    printf 'smoke harness self-test worker unexpectedly succeeded\n' >&2
    exit 1
  else
    rc=$?
  fi
  [ "$rc" -eq 1 ] || { printf 'smoke harness self-test expected exit 1, got %s\n' "$rc" >&2; exit 1; }
  printf '%s\n' "$output" | grep -q 'FAIL \[harness self-test\].*false' || { printf 'smoke harness self-test missed ERR-trap diagnostic\n' >&2; exit 1; }
  printf '%s\n' "$output" | grep -Fq 'false | true' || { printf 'smoke harness self-test missed pipefail diagnostic\n' >&2; exit 1; }
  printf '%s\n' "$output" | grep -q 'intentional explicit self-test failure' || { printf 'smoke harness self-test missed explicit diagnostic\n' >&2; exit 1; }
  printf '%s\n' "$output" | grep -q 'self-test continued after intentional failures' || { printf 'smoke harness self-test did not continue\n' >&2; exit 1; }
  printf '%s\n' "$output" | grep -q 'Smoke test suite: FAIL (3 failures)' || { printf 'smoke harness self-test missed final aggregate summary\n' >&2; exit 1; }
  printf 'smoke harness self-test: PASS\n'
  exit 0
fi

CQ9="--cq 9"

"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=320x240:rate=24 -frames:v 3 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/in.y4m"
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i color=c=gray:size=320x240:rate=24 -frames:v 3 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/flat.y4m"

# 0.1.5 keeps the 0.1.3 all-I entropy path available with --intra-only.
# Score all legal PQINDEX=9 I-picture luma/chroma AC tables per picture.
# These historical entropy/PSNR baselines predate OVERLAP and intentionally
# disable it so the test continues to isolate coefficient coding itself; the
# dedicated OVERLAP/CONDOVER runtime regression covers filtered reconstruction.
"$ENCODER" -i "$TMP/in.y4m" -o "$TMP/auto.m2ts" --es-out "$TMP/auto.vc1" --intra-only --no-overlap $CQ9 2>/dev/null
# 0.1.2 compatibility: fixed table indices 0/0, normal VLC + escapes.
"$ENCODER" -i "$TMP/in.y4m" -o "$TMP/fixed00.m2ts" --es-out "$TMP/fixed00.vc1" --intra-only --no-overlap --ac-mode vlc --ac-y-table 0 --ac-c-table 0 $CQ9 2>/dev/null
# 0.1.1 and 0.1.0 compatibility paths.
"$ENCODER" -i "$TMP/in.y4m" -o "$TMP/esc3.m2ts" --intra-only --no-overlap --ac-esc3-only $CQ9 2>/dev/null
"$ENCODER" -i "$TMP/in.y4m" -o "$TMP/dc.m2ts" --intra-only --no-overlap --dc-only $CQ9 2>/dev/null
"$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/flat.m2ts" --es-out "$TMP/flat.vc1" --intra-only --no-overlap $CQ9 2>/dev/null

codec=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "$TMP/auto.m2ts" | head -n 1)
profile=$($FFPROBE -v error -select_streams v:0 -show_entries stream=profile -of default=nw=1:nk=1 "$TMP/auto.m2ts" | head -n 1)
[ "$codec" = vc1 ]
[ "$profile" = Advanced ]

# ---- 0.1.15 WMV9/WMV3 Main Profile + ASF regressions ----
smoke_section '0.1.15 WMV9/WMV3 Main Profile + ASF regressions'
# A .wmv suffix must switch both codec syntax and container automatically.
# 160x120 intentionally leaves an 8-line coded-MB pad at the bottom, so this
# also exercises the Main-profile padded-reference edge that first exposed the
# WMV motion-compensation mismatch. The 24-frame pattern uses B reordering and
# should select all four supported B-MB prediction modes.
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=160x120:rate=24 \
  -frames:v 24 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/wmv.y4m"
"$ENCODER" -i "$TMP/wmv.y4m" -o "$TMP/auto.wmv" --cq 9 --threads 1 \
  --recon-out "$TMP/wmv-recon.yuv" 2>"$TMP/wmv.log"
wmv_codec=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "$TMP/auto.wmv" | head -n 1)
wmv_profile=$($FFPROBE -v error -select_streams v:0 -show_entries stream=profile -of default=nw=1:nk=1 "$TMP/auto.wmv" | head -n 1)
wmv_format=$($FFPROBE -v error -show_entries format=format_name -of default=nw=1:nk=1 "$TMP/auto.wmv" | head -n 1)
[ "$wmv_codec" = wmv3 ]
[ "$wmv_profile" = Main ]
[ "$wmv_format" = asf ]
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/auto.wmv" \
  -frames:v 24 -pix_fmt yuv420p -f rawvideo -y "$TMP/wmv-dec.yuv"
# 0.1.107 intentionally models SMPTE 421M Main-profile P-loop compatibility
# exceptions that FFmpeg's generic P-loop path does not.  Strict FFmpeg decode
# remains valuable, but byte-exact reconstruction is only a valid independent
# decoder invariant when Main LOOPFILTER is disabled.
"$ENCODER" -i "$TMP/wmv.y4m" -o "$TMP/auto-nolf.wmv" --cq 9 --threads 1 --no-loop-filter \
  --recon-out "$TMP/wmv-nolf-recon.yuv" 2>/dev/null
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/auto-nolf.wmv" \
  -frames:v 24 -pix_fmt yuv420p -f rawvideo -y "$TMP/wmv-nolf-dec.yuv"
cmp "$TMP/wmv-nolf-recon.yuv" "$TMP/wmv-nolf-dec.yuv"
grep -Eq '^encoded .*I=1, P=8, B=15.*B-MB=[1-9][0-9]*F/[1-9][0-9]*B/[1-9][0-9]*I/[1-9][0-9]*D.*TT=8x8:[1-9][0-9]*/8x4:[1-9][0-9]*/4x8:[1-9][0-9]*/4x4:[1-9][0-9]*' "$TMP/wmv.log"
grep -Fq 'vc1enc: options: 160x120 @ 24/1 fps, profile=main' "$TMP/wmv.log"
grep -Fq 'output=WMV9/WMV3 Main Profile ASF' "$TMP/wmv.log"

# ASF/WMV3 uses decode timestamps for reordered samples. They must be
# monotonically increasing or ordinary players may drop/duplicate B pictures.
$FFPROBE -v error -select_streams v:0 -show_entries packet=dts_time -of csv=p=0 "$TMP/auto.wmv" > "$TMP/wmv-dts.txt"
python3 - "$TMP/wmv-dts.txt" <<'PYWMVDTS'
import sys
vals=[]
for line in open(sys.argv[1], encoding='utf-8'):
    x=line.strip().split(',')[0]
    if x and x != 'N/A': vals.append(float(x))
if len(vals) != 24:
    raise SystemExit(f'expected 24 WMV packets, got {len(vals)}')
if any(b <= a for a,b in zip(vals, vals[1:])):
    raise SystemExit('WMV packet DTS is not strictly monotonic')
PYWMVDTS

# Explicit format selection overrides the filename extension in both directions.
"$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/forced-m2ts.wmv" --format m2ts --intra-only $CQ9 2>/dev/null
forced_codec=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "$TMP/forced-m2ts.wmv" | head -n 1)
forced_format=$($FFPROBE -v error -show_entries format=format_name -of default=nw=1:nk=1 "$TMP/forced-m2ts.wmv" | head -n 1)
[ "$forced_codec" = vc1 ]
[ "$forced_format" = mpegts ]
"$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/forced-wmv.asf" --format wmv --intra-only $CQ9 2>/dev/null
forced_wmv_codec=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "$TMP/forced-wmv.asf" | head -n 1)
[ "$forced_wmv_codec" = wmv3 ]
if "$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/bad-es.wmv" --es-out "$TMP/bad-es.wmv3" --intra-only $CQ9 >/dev/null 2>&1; then
  smoke_fail "--es-out unexpectedly accepted for WMV9/WMV3 output"
fi

# ---- 0.1.26 Microsoft WVC1 / ASF binding regressions ----
smoke_section '0.1.26 Microsoft WVC1 / ASF binding regressions'
# Advanced Profile in Microsoft's ASF binding uses FourCC WVC1 and begins
# CodecPrivateData with the VC-1 binding byte followed by sequence and entry-
# point EBDUs. This is deliberately explicit-only so ordinary .wmv keeps the
# long-standing WMV3 Main Profile behavior.
"$ENCODER" -i "$TMP/wmv.y4m" -o "$TMP/advanced-wvc1.wmv" --format wvc1 --cq 9 --threads 1 \
  --es-out "$TMP/advanced-wvc1.vc1" --recon-out "$TMP/advanced-wvc1-recon.yuv" 2>"$TMP/advanced-wvc1.log"
wvc1_codec=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "$TMP/advanced-wvc1.wmv" | head -n 1)
wvc1_profile=$($FFPROBE -v error -select_streams v:0 -show_entries stream=profile -of default=nw=1:nk=1 "$TMP/advanced-wvc1.wmv" | head -n 1)
wvc1_tag=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_tag_string -of default=nw=1:nk=1 "$TMP/advanced-wvc1.wmv" | head -n 1)
wvc1_format=$($FFPROBE -v error -show_entries format=format_name -of default=nw=1:nk=1 "$TMP/advanced-wvc1.wmv" | head -n 1)
[ "$wvc1_codec" = vc1 ]
[ "$wvc1_profile" = Advanced ]
[ "$wvc1_tag" = WVC1 ]
[ "$wvc1_format" = asf ]
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/advanced-wvc1.wmv" \
  -frames:v 24 -pix_fmt yuv420p -f rawvideo -y "$TMP/advanced-wvc1-dec.yuv"
cmp "$TMP/advanced-wvc1-recon.yuv" "$TMP/advanced-wvc1-dec.yuv"
grep -q 'output=VC-1 Advanced Profile WVC1 ASF' "$TMP/advanced-wvc1.log"
python3 - "$TMP/advanced-wvc1.wmv" <<'PYWVC1'
from pathlib import Path
import sys
p=Path(sys.argv[1]).read_bytes()
i=p.find(b'WVC1')
if i < 16:
    raise SystemExit('WVC1 FourCC missing from ASF video format data')
format_size=int.from_bytes(p[i-16:i-12], 'little')
private_start=i+24
private=p[private_start:private_start+format_size-40]
if not private or private[0] != 0x35:
    raise SystemExit(f'wrong WVC1 binding byte: {private[:1].hex() if private else "missing"}')
if private[1:5] != b'\x00\x00\x01\x0f':
    raise SystemExit('WVC1 private data does not begin with a sequence EBDU after the binding byte')
if b'\x00\x00\x01\x0e' not in private[5:]:
    raise SystemExit('WVC1 private data lacks an entry-point EBDU')
PYWVC1
# No-B binding advertises NO_BFRAME as well.
"$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/advanced-wvc1-nob.wmv" --format wvc1 --intra-only $CQ9 --threads 1 2>/dev/null
python3 - "$TMP/advanced-wvc1-nob.wmv" <<'PYWVC1NOB'
from pathlib import Path
import sys
p=Path(sys.argv[1]).read_bytes(); i=p.find(b'WVC1')
format_size=int.from_bytes(p[i-16:i-12], 'little'); private=p[i+24:i+24+format_size-40]
if not private or private[0] != 0x37:
    raise SystemExit('WVC1 no-B binding byte is not 0x37')
PYWVC1NOB
# ---- 0.1.29 Matroska/libmatroska regression ----
smoke_section '0.1.29 Matroska/libmatroska regression'
# MKV support is optional at build time.  When available, write enough frames
# to force repeated Cluster/BlockGroup construction.  0.1.27 could crash while
# rendering the second block and also left a zero-sized Segment header.
if "$ENCODER" -i "$TMP/wmv.y4m" -o "$TMP/advanced-wvc1.mkv" --format mkv --cq 9 --threads 1 \
    2>"$TMP/mkv.log"; then
  mkv_format=$($FFPROBE -v error -show_entries format=format_name -of default=nw=1:nk=1 "$TMP/advanced-wvc1.mkv" | head -n 1)
  mkv_codec=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "$TMP/advanced-wvc1.mkv" | head -n 1)
  mkv_tag=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_tag_string -of default=nw=1:nk=1 "$TMP/advanced-wvc1.mkv" | head -n 1)
  mkv_packets=$($FFPROBE -v error -select_streams v:0 -count_packets -show_entries stream=nb_read_packets -of default=nw=1:nk=1 "$TMP/advanced-wvc1.mkv" | head -n 1)
  case "$mkv_format" in matroska*|*matroska*) ;; *) smoke_fail "unexpected MKV format: $mkv_format";; esac
  [ "$mkv_codec" = vc1 ]
  [ "$mkv_tag" = WVC1 ]
  [ "$mkv_packets" = 24 ]
  python3 - "$TMP/advanced-wvc1.mkv" <<'PYMKVSTRUCT'
from pathlib import Path
import sys
p=Path(sys.argv[1]).read_bytes()
id=b'\x18\x53\x80\x67'
i=p.find(id)
if i < 0: raise SystemExit('Matroska Segment ID missing')
pos=i+len(id)
first=p[pos]
mask=0x80; n=1
while n<=8 and not (first & mask):
    mask >>= 1; n += 1
if n>8 or pos+n>len(p): raise SystemExit('invalid Segment size VINT')
value=first & (mask-1)
all_ones=(value == mask-1)
for b in p[pos+1:pos+n]:
    value=(value<<8)|b
    all_ones=all_ones and b==0xff
if all_ones: raise SystemExit('Segment size was not finalized')
payload=pos+n
if value == 0: raise SystemExit('Segment has a zero payload size')
if payload+value != len(p):
    raise SystemExit(f'Segment size mismatch: header ends at {payload+value}, file ends at {len(p)}')
PYMKVSTRUCT
  grep -q 'output=VC-1 Advanced Profile WVC1 MKV' "$TMP/mkv.log"

  "$ENCODER" -i "$TMP/wmv.y4m" -o "$TMP/wmv3.mkv" --format mkv --codec wmv3 --cq 9 --threads 1 \
      2>"$TMP/mkv-wmv3.log"
  mkv_wmv3_codec=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_name -of default=nw=1:nk=1 "$TMP/wmv3.mkv" | head -n 1)
  mkv_wmv3_tag=$($FFPROBE -v error -select_streams v:0 -show_entries stream=codec_tag_string -of default=nw=1:nk=1 "$TMP/wmv3.mkv" | head -n 1)
  [ "$mkv_wmv3_codec" = wmv3 ]
  [ "$mkv_wmv3_tag" = WMV3 ]
  grep -q 'output=WMV9/WMV3 Main Profile MKV' "$TMP/mkv-wmv3.log"
else
  if ! grep -q 'Matroska output was not built' "$TMP/mkv.log"; then
    cat "$TMP/mkv.log" >&2
    smoke_fail 'Matroska regression failed unexpectedly'
  fi
fi

# MPEG-TS uses the standardized VC-1 registration descriptor; WVC1 is an
# ASF/Windows media subtype and must not replace the TS registration string.
python3 - "$TMP/auto.m2ts" <<'PYTSVC1'
from pathlib import Path
import sys
p=Path(sys.argv[1]).read_bytes()[:16384]
if b'VC-1' not in p:
    raise SystemExit('M2TS PMT lacks VC-1 registration descriptor')
PYTSVC1

for f in auto fixed00 esc3 dc flat; do
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/$f.m2ts" -frames:v 3 -f null -
done
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/flat.vc1" -frames:v 3 -f null -

# ---- 0.1.25 speed-preset regressions ----
smoke_section '0.1.25 speed-preset regressions'
# All speed presets must remain decodable predictive encodes.  In particular,
# --fastest must preserve P/B picture generation while taking the dedicated
# zero-motion path rather than silently falling back to all-I coding.
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=160x120:rate=24 \
  -frames:v 12 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/speed.y4m"
for preset in fast faster fastest; do
  "$ENCODER" -i "$TMP/speed.y4m" -o "$TMP/$preset.m2ts" --cq 9 --threads 1 --"$preset" \
    2>"$TMP/$preset.log"
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/$preset.m2ts" \
    -frames:v 12 -f null -
  grep -q "preset=$preset" "$TMP/$preset.log"
done
grep -q 'search=8 px' "$TMP/fast.log"
grep -q 'search=4 px' "$TMP/faster.log"
grep -Eq '^encoded .*I=[1-9][0-9]*, P=[1-9][0-9]*, B=[1-9][0-9]*.*B-MB=[1-9][0-9]*F/0B/0I/0D.*moved-MB=0, halfchroma-MB=0' "$TMP/fastest.log"
grep -Fq 'search=0 px (local 0), distant-max-mae=' "$TMP/fastest.log"
grep -Fq 'me=sad' "$TMP/fastest.log"

# Every one of the 3x3 legal table-index pairs must decode to the exact same
# pixels.  Auto mode must be no larger in elementary-stream bytes than any
# fixed pair on this deterministic regression sequence.
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/auto.m2ts" -frames:v 3 -pix_fmt yuv420p -f rawvideo -y "$TMP/ref.yuv"
auto_es_size=$(wc -c < "$TMP/auto.vc1")
min_fixed=999999999
for y in 0 1 2; do
  for c in 0 1 2; do
    name="fixed${y}${c}"
    if [ "$name" != fixed00 ]; then
      "$ENCODER" -i "$TMP/in.y4m" -o "$TMP/$name.m2ts" --es-out "$TMP/$name.vc1" --intra-only --no-overlap --ac-mode vlc --ac-y-table "$y" --ac-c-table "$c" $CQ9 2>/dev/null
    fi
    "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/$name.m2ts" -frames:v 3 -pix_fmt yuv420p -f rawvideo -y "$TMP/$name.yuv"
    cmp "$TMP/ref.yuv" "$TMP/$name.yuv"
    n=$(wc -c < "$TMP/$name.vc1")
    [ "$n" -lt "$min_fixed" ] && min_fixed=$n
    [ "$auto_es_size" -le "$n" ]
  done
done
# Auto is allowed to choose a different pair per picture, so its aggregate
# syntax cost must not exceed the best fixed pair on this regression.
[ "$auto_es_size" -le "$min_fixed" ]

# Auto/VLC and ESC3-only carry the same quantized coefficients.
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/esc3.m2ts" -frames:v 3 -pix_fmt yuv420p -f rawvideo -y "$TMP/esc3.yuv"
cmp "$TMP/ref.yuv" "$TMP/esc3.yuv"
auto_size=$(wc -c < "$TMP/auto.m2ts")
esc3_size=$(wc -c < "$TMP/esc3.m2ts")
awk -v auto="$auto_size" -v esc3="$esc3_size" 'BEGIN { exit !(auto < esc3 * 0.5) }'

psnr_of() {
  "$FFMPEG" -hide_banner -loglevel info -i "$1" -i "$TMP/in.y4m" -lavfi '[0:v][1:v]psnr' -frames:v 3 -f null - 2>&1 | sed -n 's/.*average:\([0-9.][0-9.]*\).*/\1/p' | tail -n 1
  return 0
}
ac_psnr=$(psnr_of "$TMP/auto.m2ts")
dc_psnr=$(psnr_of "$TMP/dc.m2ts")
[[ -n "$ac_psnr" && -n "$dc_psnr" ]]
awk -v ac="$ac_psnr" -v dc="$dc_psnr" 'BEGIN { exit !(ac > 35.0 && ac > dc + 10.0) }'

# ---- 0.1.20 tiered x86 SIMD regressions ----
smoke_section '0.1.20 tiered x86 SIMD regressions'
# SIMD implementations may legitimately reorder floating-point work or make a
# different but visually equivalent RDO choice. Require decoder-exact
# reconstruction for each SIMD stream and high visual similarity to scalar
# instead of requiring the encoded bytes themselves to be identical.
simd_visual_check() {
  local ref=$1 cand=$2
  local ssim
  ssim=$("$FFMPEG" -hide_banner -loglevel info \
    -f rawvideo -pix_fmt yuv420p -video_size 320x240 -framerate 24 -i "$ref" \
    -f rawvideo -pix_fmt yuv420p -video_size 320x240 -framerate 24 -i "$cand" \
    -lavfi '[0:v][1:v]ssim' -frames:v 8 -f null - 2>&1 | \
    sed -n 's/.*All:\([0-9.][0-9.]*\).*/\1/p' | tail -n 1)
  [[ -n "$ssim" ]] || return 1
  awk -v s="$ssim" 'BEGIN { exit !(s >= 0.95) }'
}
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=320x240:rate=24 \
  -frames:v 8 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/simd.y4m"
"$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-auto.m2ts" --es-out "$TMP/simd-auto.vc1" \
  --recon-out "$TMP/simd-auto.yuv" --cq 9 --threads 1 --keyint 8 --simd auto 2>"$TMP/simd-auto.log"
"$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-scalar.m2ts" --es-out "$TMP/simd-scalar.vc1" \
  --recon-out "$TMP/simd-scalar.yuv" --cq 9 --threads 1 --keyint 8 --simd scalar 2>"$TMP/simd-scalar.log"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/simd-auto.m2ts" \
  -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/simd-auto-dec.yuv"
cmp "$TMP/simd-auto.yuv" "$TMP/simd-auto-dec.yuv"
simd_visual_check "$TMP/simd-scalar.yuv" "$TMP/simd-auto.yuv"
grep -Eq 'simd=(none|mixed|x86-64-v1|prescott|k10|conroe|penryn|x86-64-v2|sandybridge|bulldozer|piledriver|avx2-partial|x86-64-v3|x86-64-v4)' "$TMP/simd-auto.log"
grep -q 'simd=none' "$TMP/simd-scalar.log"

V1_OK=0
if "$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-v1.m2ts" --es-out "$TMP/simd-v1.vc1" \
    --recon-out "$TMP/simd-v1.yuv" --cq 9 --threads 1 --keyint 8 --simd x86-64-v1 2>"$TMP/simd-v1.log"; then
  V1_OK=1
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/simd-v1.m2ts" \
    -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/simd-v1-dec.yuv"
  cmp "$TMP/simd-v1.yuv" "$TMP/simd-v1-dec.yuv"
  cmp "$TMP/simd-scalar.yuv" "$TMP/simd-v1.yuv"
  grep -q 'simd=x86-64-v1, fma3=inactive' "$TMP/simd-v1.log"
fi
V2_OK=0
if "$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-v2.m2ts" --es-out "$TMP/simd-v2.vc1" \
    --recon-out "$TMP/simd-v2.yuv" --cq 9 --threads 1 --keyint 8 --simd x86-64-v2 2>"$TMP/simd-v2.log"; then
  V2_OK=1
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/simd-v2.m2ts" \
    -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/simd-v2-dec.yuv"
  cmp "$TMP/simd-v2.yuv" "$TMP/simd-v2-dec.yuv"
  cmp "$TMP/simd-scalar.yuv" "$TMP/simd-v2.yuv"
  grep -q 'simd=x86-64-v2, fma3=inactive' "$TMP/simd-v2.log"
fi
for MID in prescott k10 conroe penryn sandybridge bulldozer piledriver avx2-partial; do
  if "$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-$MID.m2ts" --es-out "$TMP/simd-$MID.vc1" \
      --recon-out "$TMP/simd-$MID.yuv" --cq 9 --threads 1 --keyint 8 --simd "$MID" 2>"$TMP/simd-$MID.log"; then
    "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/simd-$MID.m2ts" \
      -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/simd-$MID-dec.yuv"
    cmp "$TMP/simd-$MID.yuv" "$TMP/simd-$MID-dec.yuv"
    simd_visual_check "$TMP/simd-scalar.yuv" "$TMP/simd-$MID.yuv"
    grep -q "simd=$MID" "$TMP/simd-$MID.log"
  else
    grep -q 'unavailable on this CPU/build' "$TMP/simd-$MID.log"
  fi
done
V3_OK=0
if "$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-v3.m2ts" --es-out "$TMP/simd-v3.vc1" \
    --recon-out "$TMP/simd-v3.yuv" --cq 9 --threads 1 --keyint 8 --simd x86-64-v3 2>"$TMP/simd-v3.log"; then
  V3_OK=1
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/simd-v3.m2ts" \
    -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/simd-v3-dec.yuv"
  cmp "$TMP/simd-v3.yuv" "$TMP/simd-v3-dec.yuv"
  simd_visual_check "$TMP/simd-scalar.yuv" "$TMP/simd-v3.yuv"
  grep -q 'simd=x86-64-v3' "$TMP/simd-v3.log"
fi
V4_OK=0
if "$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-v4.m2ts" --es-out "$TMP/simd-v4.vc1" \
    --recon-out "$TMP/simd-v4.yuv" --cq 9 --threads 1 --keyint 8 --simd x86-64-v4 2>"$TMP/simd-v4.log"; then
  V4_OK=1
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/simd-v4.m2ts" \
    -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/simd-v4-dec.yuv"
  cmp "$TMP/simd-v4.yuv" "$TMP/simd-v4-dec.yuv"
  simd_visual_check "$TMP/simd-scalar.yuv" "$TMP/simd-v4.yuv"
  grep -q 'simd=x86-64-v4' "$TMP/simd-v4.log"
fi
if [ "$V3_OK" -eq 1 ] && [ "$V4_OK" -eq 1 ]; then
  grep -q 'SIMD auto benchmark per primitive (none=units/s; SIMD columns=% of none):' "$TMP/simd-auto.log"
grep -Eq '^MODE +PRIMITIVE +NONE .*X86-64-V1 .*X86-64-V2 .*X86-64-V3 .*X86-64-V4 .*SELECTED +SELECTED% *$' "$TMP/simd-auto.log"
grep -Eq '^\[auto\] +frame-sad .* +(none|x86-64-v1|prescott|k10|conroe|penryn|x86-64-v2|sandybridge|bulldozer|piledriver|avx2-partial|x86-64-v3|x86-64-v4)(\+fma(3|4))? +(n/a|[0-9]+(\.[0-9]+)?%) *$' "$TMP/simd-auto.log"
grep -Eq '^\[auto\] +geometric-mean .* +- +(n/a|[0-9]+(\.[0-9]+)?%) *$' "$TMP/simd-auto.log"
# Header, primitive rows and total must remain exactly aligned even when an
# intermediate feature-band column is inserted dynamically.
awk '/^(MODE|\[auto\]|\[forced\])/{if(!w)w=length($0);else if(length($0)!=w)exit 1} END{if(!w)exit 1}' "$TMP/simd-auto.log"
fi

# AUTO deliberately keeps coding-affecting forward transforms on the non-fused
# arithmetic law so startup benchmark noise cannot change coefficient rounding or
# the encoded bitstream. Explicit/forced FMA remains available for diagnostics and
# benchmarking; the decoder must reconstruct exactly what the encoder reconstructed.
if [ "$V3_OK" -eq 1 ]; then
  "$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-fma.m2ts" --recon-out "$TMP/simd-fma.yuv" \
    --cq 9 --threads 1 --keyint 8 --simd x86-64-v3 --simd-fma 2>"$TMP/simd-fma.log"
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/simd-fma.m2ts" \
    -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/simd-fma-dec.yuv"
  cmp "$TMP/simd-fma.yuv" "$TMP/simd-fma-dec.yuv"
  grep -q 'simd=x86-64-v3, fma3=allowed' "$TMP/simd-fma.log"
fi
if "$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/bad-simd.m2ts" --simd nonsense --cq 9 >/dev/null 2>&1; then
  smoke_fail "invalid SIMD mode unexpectedly accepted"
fi
"$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-none-fma.m2ts" --simd scalar --simd-fma --cq 9 --threads 1 2>"$TMP/simd-none-fma.log"
grep -q 'simd=none, fma3=inactive' "$TMP/simd-none-fma.log"

# Container choice must not affect SIMD determinism.
"$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-auto.wmv" --recon-out "$TMP/simd-auto-wmv.yuv" \
  --cq 9 --threads 1 --keyint 8 --simd auto --no-loop-filter 2>/dev/null
"$ENCODER" -i "$TMP/simd.y4m" -o "$TMP/simd-scalar.wmv" --recon-out "$TMP/simd-scalar-wmv.yuv" \
  --cq 9 --threads 1 --keyint 8 --simd scalar --no-loop-filter 2>/dev/null
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/simd-auto.wmv" \
  -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/simd-auto-wmv-dec.yuv"
cmp "$TMP/simd-auto-wmv.yuv" "$TMP/simd-auto-wmv-dec.yuv"
simd_visual_check "$TMP/simd-scalar-wmv.yuv" "$TMP/simd-auto-wmv.yuv"

# ---- 0.1.14 trellis-quantization regressions ----
smoke_section '0.1.14 trellis-quantization regressions'
# Default level 1 must be exactly the same as requesting it explicitly.  The
# scalar path is retained for compatibility, while both trellis search widths
# must reconstruct byte-for-byte like an independent VC-1 decoder.
for t in 0 1 2; do
  "$ENCODER" -i "$TMP/in.y4m" -o "$TMP/trellis$t.m2ts" --es-out "$TMP/trellis$t.vc1" \
    --recon-out "$TMP/trellis$t-recon.yuv" --intra-only --no-overlap $CQ9 --trellis "$t" --threads 1 2>/dev/null
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/trellis$t.vc1" \
    -frames:v 3 -pix_fmt yuv420p -f rawvideo -y "$TMP/trellis$t-dec.yuv"
  cmp "$TMP/trellis$t-recon.yuv" "$TMP/trellis$t-dec.yuv"
done
cmp "$TMP/auto.vc1" "$TMP/trellis1.vc1"
cmp "$TMP/auto.m2ts" "$TMP/trellis1.m2ts"
# The same default-vs-explicit level-1 invariant must also hold with the new
# default OVERLAP path enabled; keep this separate from the historical
# no-overlap entropy fixture above.
"$ENCODER" -i "$TMP/in.y4m" -o "$TMP/trellis-auto-overlap.m2ts" --es-out "$TMP/trellis-auto-overlap.vc1" \
  --intra-only $CQ9 --threads 1 2>/dev/null
"$ENCODER" -i "$TMP/in.y4m" -o "$TMP/trellis1-overlap.m2ts" --es-out "$TMP/trellis1-overlap.vc1" \
  --intra-only $CQ9 --trellis 1 --threads 1 2>/dev/null
cmp "$TMP/trellis-auto-overlap.vc1" "$TMP/trellis1-overlap.vc1"
cmp "$TMP/trellis-auto-overlap.m2ts" "$TMP/trellis1-overlap.m2ts"
trellis0_size=$(wc -c < "$TMP/trellis0.vc1")
trellis1_size=$(wc -c < "$TMP/trellis1.vc1")
[ "$trellis1_size" -lt "$trellis0_size" ]
if cmp -s "$TMP/trellis0.vc1" "$TMP/trellis1.vc1"; then
  smoke_fail "trellis level 1 unexpectedly produced scalar output"
fi
if "$ENCODER" -i "$TMP/in.y4m" -o "$TMP/bad-trellis.m2ts" --trellis 3 $CQ9 >/dev/null 2>&1; then
  smoke_fail "invalid trellis level unexpectedly accepted"
fi

# Table overrides are meaningful only in fixed VLC mode.
if "$ENCODER" -i "$TMP/in.y4m" -o "$TMP/bad.m2ts" --ac-y-table 1 $CQ9 >/dev/null 2>&1; then
  smoke_fail "table override unexpectedly accepted in auto mode"
fi

# ---- 0.1.5 P-picture / residual / reconstructed-reference regressions ----
smoke_section '0.1.5 P-picture / residual / reconstructed-reference regressions'
# A static synthetic texture is cropped two pixels farther right every frame.
# The current crop is therefore explained by a +2 px reference MV (except at
# boundaries). A post-motion scene detector must keep these frames as P.
"$FFMPEG" -hide_banner -loglevel error -f lavfi \
  -i "nullsrc=s=96x48:r=24,geq=lum='mod(X*13+Y*17+mod(X*Y,31)*5,256)':cb=128:cr=128,crop=64:48:x='2*n':y=0" \
  -frames:v 8 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/pan.y4m"
"$ENCODER" -i "$TMP/pan.y4m" -o "$TMP/pan.m2ts" --es-out "$TMP/pan.vc1" --recon-out "$TMP/pan-recon.yuv" --bframes 0 --scene-cut --search-range 16 --local-search-range 16 $CQ9 2>"$TMP/pan.log"
"$FFMPEG" -hide_banner -f mpegts -v error -xerror -err_detect explode -i "$TMP/pan.m2ts" -frames:v 8 -f null -
pan_types=$($FFPROBE -f mpegts -v error -select_streams v:0 -show_entries frame=pict_type -of csv=p=0 "$TMP/pan.m2ts" | tr -d '\r' | paste -sd, -)
[ "$pan_types" = "I,P,P,P,P,P,P,P" ]
grep -Eq 'P=7, B=0, scene-I=0.*moved-MB=[1-9][0-9]*.*explicit-MB=[1-9][0-9]*.*coded-MB=[1-9][0-9]*.*coded-blocks=[1-9][0-9]*' "$TMP/pan.log"

# The encoder's reference buffer must be byte-identical to an independent
# decoder's reconstruction. This turns the 0.1.4 source-vs-decoder drift bug
# into a hard regression failure.
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/pan.vc1" \
  -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/pan-dec.yuv"
cmp "$TMP/pan-recon.yuv" "$TMP/pan-dec.yuv"

# 0.1.49: a smooth camera zoom is not a scene cut.  A single global
# translation cannot represent scale change, so the detector must use its
# candidate-only local-motion rescue instead of inserting I pictures.
python3 - "$TMP/zoom-scene.y4m" <<'PYZOOMSCENE'
import numpy as np,sys
out=sys.argv[1]; w,h,n=160,120,24
yy,xx=np.mgrid[0:h,0:w]
base=(40+((xx*13+yy*17+(xx*yy)%31*5)%180)).astype(np.uint8)
cx=(w-1)/2.0; cy=(h-1)/2.0
uv=bytes([128])*(w*h//4)
with open(out,'wb') as f:
    f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for t in range(n):
        scale=1.0+0.40*t/(n-1)
        sx=(xx-cx)/scale+cx; sy=(yy-cy)/scale+cy
        x0=np.floor(sx).astype(int); y0=np.floor(sy).astype(int)
        x1=np.clip(x0+1,0,w-1); y1=np.clip(y0+1,0,h-1)
        x0=np.clip(x0,0,w-1); y0=np.clip(y0,0,h-1)
        fx=sx-x0; fy=sy-y0
        z=((1-fx)*(1-fy)*base[y0,x0]+fx*(1-fy)*base[y0,x1]+
           (1-fx)*fy*base[y1,x0]+fx*fy*base[y1,x1])
        f.write(b'FRAME\n'); f.write(np.clip(np.rint(z),0,255).astype(np.uint8).tobytes()); f.write(uv); f.write(uv)
PYZOOMSCENE
"$ENCODER" -i "$TMP/zoom-scene.y4m" -o "$TMP/zoom-scene.m2ts" --bframes 0 --keyint 999 --scene-cut --search-range 16 --local-search-range 16 $CQ9 2>"$TMP/zoom-scene.log"
grep -Eq 'I=1, P=23, B=0, scene-I=0' "$TMP/zoom-scene.log"


# 0.1.50: keyint is a maximum interval from the most recent I picture by
# default. A scene I at display 10 resets the 24-frame counter, so the next
# scheduled I is display 34 rather than the old absolute-grid display 24.
# --fixed-gop-grid retains the historical cadence for compatibility.
python3 - "$TMP/scene-reset-grid.y4m" <<'PYSCENERESET'
import sys
w,h,n=64,48,48
uv=bytes([128])*(w*h//4)
with open(sys.argv[1],'wb') as f:
    f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for i in range(n):
        y=(bytes([32])*(w*h) if i<10 else
           bytes(220 if ((x//4+y//4)&1) else 80 for y in range(h) for x in range(w)))
        f.write(b'FRAME\n'); f.write(y); f.write(uv); f.write(uv)
PYSCENERESET
for mode in reset fixed; do
  extra=""
  [ "$mode" = fixed ] && extra="--fixed-gop-grid"
  "$ENCODER" -i "$TMP/scene-reset-grid.y4m" -o "$TMP/scene-$mode.m2ts" \
    --es-out "$TMP/scene-$mode.vc1" --bframes 0 --keyint 24 --cq 9 --threads 1 \
    --rc-stats "$TMP/scene-$mode.csv" --scene-cut --search-range 16 --local-search-range 16 $extra 2>"$TMP/scene-$mode.log"
done
python3 - "$TMP/scene-reset.csv" "$TMP/scene-fixed.csv" <<'PYSCENEGRID'
import csv,sys
def ipos(path):
    return [int(r['display_order']) for r in csv.DictReader(open(path,newline='')) if r['type']=='I']
reset=ipos(sys.argv[1]); fixed=ipos(sys.argv[2])
if reset != [0,10,34]: raise SystemExit(f'scene-reset keyint cadence mismatch: {reset}')
if fixed != [0,10,24]: raise SystemExit(f'fixed-grid compatibility cadence mismatch: {fixed}')
PYSCENEGRID
grep -Eq '^encoded .*I=3, P=45, B=0, scene-I=1' "$TMP/scene-reset.log"
grep -Fq 'keyint=24, gop-grid=scene-reset, bframes=0' "$TMP/scene-reset.log"
grep -Eq '^encoded .*I=3, P=45, B=0, scene-I=1' "$TMP/scene-fixed.log"
grep -Fq 'keyint=24, gop-grid=fixed, bframes=0' "$TMP/scene-fixed.log"

# Exercise odd integer luma MVs, which produce half-sample chroma motion in
# 4:2:0, and require exact reconstruction there too.
"$FFMPEG" -hide_banner -loglevel error -f lavfi \
  -i "testsrc2=s=96x48:r=24,crop=64:48:x='n':y=0:exact=1" \
  -frames:v 6 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/oddpan.y4m"
"$ENCODER" -i "$TMP/oddpan.y4m" -o "$TMP/oddpan.m2ts" --es-out "$TMP/oddpan.vc1" $CQ9 \
  --recon-out "$TMP/oddpan-recon.yuv" --bframes 0 --no-scene-cut 2>"$TMP/oddpan.log"
grep -Eq 'halfchroma-MB=[1-9][0-9]*' "$TMP/oddpan.log"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/oddpan.vc1" \
  -frames:v 6 -pix_fmt yuv420p -f rawvideo -y "$TMP/oddpan-dec.yuv"
cmp "$TMP/oddpan-recon.yuv" "$TMP/oddpan-dec.yuv"

# ---- 0.1.13 in-loop deblocking / padded-edge regression ----
smoke_section '0.1.13 in-loop deblocking / padded-edge regression'
# 160x120 occupies a 160x128 coded luma grid.  The hidden block beginning at
# y=120 can affect visible row 119 through the normative loop filter, so an
# implementation that filters only the display crop will fail this comparison.
python3 - "$TMP/deblock-edge.y4m" <<'PYDEBLOCK'
import random, sys
out=sys.argv[1]
w,h=160,120
r=random.Random(12345)
y=bytes(r.randrange(16,236) for _ in range(w*h))
u=bytes(r.randrange(32,225) for _ in range(w*h//4))
v=bytes(r.randrange(32,225) for _ in range(w*h//4))
with open(out,'wb') as f:
    f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    f.write(b'FRAME\n'); f.write(y); f.write(u); f.write(v)
PYDEBLOCK
"$ENCODER" -i "$TMP/deblock-edge.y4m" -o "$TMP/deblock.m2ts" --es-out "$TMP/deblock.vc1" \
  --recon-out "$TMP/deblock-recon.yuv" --intra-only --cq 9 --threads 1 2>/dev/null
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/deblock.vc1" \
  -frames:v 1 -pix_fmt yuv420p -f rawvideo -y "$TMP/deblock-dec.yuv"
cmp "$TMP/deblock-recon.yuv" "$TMP/deblock-dec.yuv"
"$ENCODER" -i "$TMP/deblock-edge.y4m" -o "$TMP/deblock-off.m2ts" --es-out "$TMP/deblock-off.vc1" \
  --recon-out "$TMP/deblock-off-recon.yuv" --intra-only --cq 9 --threads 1 --no-loop-filter 2>/dev/null
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/deblock-off.vc1" \
  -frames:v 1 -pix_fmt yuv420p -f rawvideo -y "$TMP/deblock-off-dec.yuv"
cmp "$TMP/deblock-off-recon.yuv" "$TMP/deblock-off-dec.yuv"
if cmp -s "$TMP/deblock-recon.yuv" "$TMP/deblock-off-recon.yuv"; then
  smoke_fail "loop-filter enabled/disabled reconstructions unexpectedly match"
fi
python3 - "$TMP/deblock.vc1" "$TMP/deblock-off.vc1" <<'PYLOOPBIT'
import sys
def loopbit(path):
    d=open(path,'rb').read(); tag=b'\x00\x00\x01\x0e'; p=d.find(tag)
    if p < 0 or p+4 >= len(d): raise SystemExit('missing VC-1 entry point')
    return (d[p+4] >> 3) & 1
if loopbit(sys.argv[1]) != 1 or loopbit(sys.argv[2]) != 0:
    raise SystemExit('wrong LOOPFILTER entry-point signaling')
PYLOOPBIT

# ---- 0.1.11 intensity-compensation / fade regressions ----
smoke_section '0.1.11 intensity-compensation / fade regressions'
# Build a deterministic textured global fade whose successive pictures are
# well described by VC-1's legal LUMSCALE/LUMSHIFT affine mapping.  Keep the
# sequence small so this remains a cheap regression while still exercising
# motion-aligned estimation, syntax signaling, and decoder-equivalent mapping.
python3 - "$TMP/fade.y4m" <<'PYFADE'
import random, sys
out=sys.argv[1]
w,h,n=160,120,18
r=random.Random(12345)
y=bytes(r.randrange(16,236) for _ in range(w*h))
u=bytes(r.randrange(32,225) for _ in range(w*h//4))
v=bytes(r.randrange(32,225) for _ in range(w*h//4))
with open(out,'wb') as f:
    f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for t in range(n):
        a=1.0-0.022*t
        f.write(b'FRAME\n')
        f.write(bytes(max(0,min(255,round(z*a))) for z in y))
        f.write(bytes(max(0,min(255,round(128+(z-128)*a))) for z in u))
        f.write(bytes(max(0,min(255,round(128+(z-128)*a))) for z in v))
PYFADE

# P-only: every eligible frame should use intensity compensation, the fade
# must not become a false scene cut, and encoder reconstruction must match an
# independent decoder byte-for-byte. This historical IC-selection baseline
# predates OVERLAP; disable overlap here so the initial filtered I reference
# does not intentionally perturb the affine-fit eligibility test.
"$ENCODER" -i "$TMP/fade.y4m" -o "$TMP/fade-ic.m2ts" --es-out "$TMP/fade-ic.vc1" \
  --recon-out "$TMP/fade-ic-recon.yuv" --cq 9 --bframes 0 --keyint 999 --scene-cut --search-range 16 --local-search-range 16 --no-overlap 2>"$TMP/fade-ic.log"
grep -Eq 'I=1, P=17, B=0, scene-I=0, motion-failure-I=0, IC-P=17' "$TMP/fade-ic.log"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/fade-ic.vc1" \
  -frames:v 18 -pix_fmt yuv420p -f rawvideo -y "$TMP/fade-ic-dec.yuv"
cmp "$TMP/fade-ic-recon.yuv" "$TMP/fade-ic-dec.yuv"

# At identical quantization the weighted path must materially reduce syntax
# cost on this known fade.  The disabled run also verifies the CLI control.
"$ENCODER" -i "$TMP/fade.y4m" -o "$TMP/fade-noic.m2ts" --es-out "$TMP/fade-noic.vc1" \
  --cq 9 --bframes 0 --keyint 999 --scene-cut --search-range 16 --local-search-range 16 --no-overlap --no-fade-comp 2>"$TMP/fade-noic.log"
grep -Eq 'IC-P=0' "$TMP/fade-noic.log"
fade_ic_size=$(wc -c < "$TMP/fade-ic.vc1")
fade_noic_size=$(wc -c < "$TMP/fade-noic.vc1")
awk -v ic="$fade_ic_size" -v no="$fade_noic_size" 'BEGIN { exit !(ic < no * 0.75) }'

# A separate brightness-offset ramp stresses signed LUMSHIFT rather than a
# scale-dominated fade.  Exact reconstruction proves the shift code and luma
# mapping agree with the independent decoder.
python3 - "$TMP/shift.y4m" <<'PYSHIFT'
import random, sys
out=sys.argv[1]
w,h,n=128,96,8
r=random.Random(9)
y=bytes(r.randrange(20,220) for _ in range(w*h))
u=bytes(r.randrange(50,205) for _ in range(w*h//4))
v=bytes(r.randrange(50,205) for _ in range(w*h//4))
with open(out,'wb') as f:
    f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for t in range(n):
        f.write(b'FRAME\n')
        f.write(bytes(min(255,z+3*t) for z in y))
        f.write(u); f.write(v)
PYSHIFT
"$ENCODER" -i "$TMP/shift.y4m" -o "$TMP/shift.m2ts" --es-out "$TMP/shift.vc1" \
  --recon-out "$TMP/shift-recon.yuv" --cq 9 --bframes 0 --keyint 999 --no-scene-cut 2>"$TMP/shift.log"
grep -Eq 'I=1, P=7, B=0, scene-I=0, motion-failure-I=0, IC-P=7' "$TMP/shift.log"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/shift.vc1" \
  -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/shift-dec.yuv"
cmp "$TMP/shift-recon.yuv" "$TMP/shift-dec.yuv"

# B pictures cannot signal intensity compensation themselves.  They inherit
# the future P anchor's LUT on their past/forward reference.  Require that
# reordered I/B/P reconstruction to match FFmpeg exactly.
"$ENCODER" -i "$TMP/fade.y4m" -o "$TMP/fade-b.m2ts" --es-out "$TMP/fade-b.vc1" \
  --recon-out "$TMP/fade-b-recon.yuv" --cq 9 --keyint 999 --scene-cut --search-range 16 --local-search-range 16 2>"$TMP/fade-b.log"
grep -Eq 'I=1, P=6, B=11, scene-I=0, motion-failure-I=0, IC-P=6' "$TMP/fade-b.log"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/fade-b.vc1" \
  -frames:v 18 -pix_fmt yuv420p -f rawvideo -y "$TMP/fade-b-dec.yuv"
cmp "$TMP/fade-b-recon.yuv" "$TMP/fade-b-dec.yuv"

# An abrupt black-to-white cut remains high-residual after any motion search,
# so it should trigger an immediate I picture rather than waiting for keyint.
"$FFMPEG" -hide_banner -loglevel error \
  -f lavfi -i color=c=black:s=64x48:r=24:d=0.0833334 \
  -f lavfi -i color=c=white:s=64x48:r=24:d=0.0833334 \
  -filter_complex '[0:v][1:v]concat=n=2:v=1:a=0,format=yuv420p' \
  -frames:v 4 -f yuv4mpegpipe -y "$TMP/cut.y4m"
"$ENCODER" -i "$TMP/cut.y4m" -o "$TMP/cut.m2ts" --bframes 0 --scene-cut --search-range 16 --local-search-range 16 $CQ9 2>"$TMP/cut.log"
"$FFMPEG" -hide_banner -f mpegts -v error -xerror -err_detect explode -i "$TMP/cut.m2ts" -frames:v 12 -f null -
cut_types=$($FFPROBE -f mpegts -v error -select_streams v:0 -show_entries frame=pict_type -of csv=p=0 "$TMP/cut.m2ts" | tr -d '\r' | paste -sd, -)
[ "$cut_types" = "I,P,I,P" ]
grep -Eq 'I=2, P=2, B=0, scene-I=1' "$TMP/cut.log"

# With the threshold detector disabled (the 0.1.74 default), the independent
# motion-failure safety net must still turn a truly unrelated boundary into an
# I picture after every macroblock fails to find a useful temporal match.
"$ENCODER" -i "$TMP/cut.y4m" -o "$TMP/cut-motion-failure.m2ts" --bframes 0 --no-scene-cut \
  --search-range 1024 --local-search-range 32 $CQ9 2>"$TMP/cut-motion-failure.log"
cut_mf_types=$($FFPROBE -f mpegts -v error -select_streams v:0 -show_entries frame=pict_type -of csv=p=0 "$TMP/cut-motion-failure.m2ts" | tr -d '\r' | paste -sd, -)
[ "$cut_mf_types" = "I,P,I,P" ]
grep -Eq 'I=2, P=2, B=0, scene-I=0, motion-failure-I=1' "$TMP/cut-motion-failure.log"

# Generic VC-1 is not bound to Blu-ray's one-second random-access rule.  The
# automatic generic cadence is 120 frames, so this short 24 fps fixture has no
# scheduled second I picture.
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i color=c=gray:s=64x48:r=24 \
  -frames:v 26 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/keyint.y4m"
"$ENCODER" -i "$TMP/keyint.y4m" -o "$TMP/keyint.m2ts" --bframes 0 --no-scene-cut $CQ9 2>"$TMP/keyint.log"
"$FFMPEG" -hide_banner -f mpegts -v error -xerror -err_detect explode -i "$TMP/keyint.m2ts" -frames:v 26 -f null -
key_types=$($FFPROBE -f mpegts -v error -select_streams v:0 -show_entries frame=pict_type -of csv=p=0 "$TMP/keyint.m2ts" | tr -d '\r')
[ "$(printf '%s\n' "$key_types" | sed -n '1p')" = I ]
[ "$(printf '%s\n' "$key_types" | sed -n '2,26p' | sort -u)" = P ]
grep -Eq '^encoded .*I=1, P=25, B=0, scene-I=0.*B-MB=0F/0B/0I/0D' "$TMP/keyint.log"
grep -Fq 'keyint=120, gop-grid=scene-reset, bframes=0' "$TMP/keyint.log"

# ---- 0.1.8 GOP-level multithreading regressions ----
smoke_section '0.1.8 GOP-level multithreading regressions'
# Scheduled GOPs are independent worker jobs.  Thread count must not alter any
# coding decision or output ordering: 1-thread and 4-thread runs must be byte
# identical in transport, elementary stream, and encoder reconstruction.
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=160x120:rate=24 \
  -frames:v 32 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/thread.y4m"
"$ENCODER" -i "$TMP/thread.y4m" -o "$TMP/thread1.m2ts" --es-out "$TMP/thread1.vc1" \
  --recon-out "$TMP/thread1-recon.yuv" --threads 1 --keyint 8 --no-scene-cut $CQ9 2>"$TMP/thread1.log"
"$ENCODER" -i "$TMP/thread.y4m" -o "$TMP/thread4.m2ts" --es-out "$TMP/thread4.vc1" \
  --recon-out "$TMP/thread4-recon.yuv" --threads 4 --keyint 8 --no-scene-cut $CQ9 2>"$TMP/thread4.log"
cmp "$TMP/thread1.m2ts" "$TMP/thread4.m2ts"
cmp "$TMP/thread1.vc1" "$TMP/thread4.vc1"
cmp "$TMP/thread1-recon.yuv" "$TMP/thread4-recon.yuv"
# The new ASF mux path is also owned only by the drain thread, so GOP worker
# count must not alter WMV3 payloads, packetization, timestamps, or recon.
"$ENCODER" -i "$TMP/thread.y4m" -o "$TMP/thread1.wmv" --recon-out "$TMP/thread1-wmv-recon.yuv" \
  --threads 1 --keyint 8 --no-scene-cut --no-loop-filter $CQ9 2>/dev/null
"$ENCODER" -i "$TMP/thread.y4m" -o "$TMP/thread4.wmv" --recon-out "$TMP/thread4-wmv-recon.yuv" \
  --threads 4 --keyint 8 --no-scene-cut --no-loop-filter $CQ9 2>/dev/null
cmp "$TMP/thread1.wmv" "$TMP/thread4.wmv"
cmp "$TMP/thread1-wmv-recon.yuv" "$TMP/thread4-wmv-recon.yuv"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/thread4.wmv" \
  -frames:v 32 -pix_fmt yuv420p -f rawvideo -y "$TMP/thread4-wmv-dec.yuv"
cmp "$TMP/thread4-wmv-recon.yuv" "$TMP/thread4-wmv-dec.yuv"
grep -Eq '^encoded .*I=4, P=12, B=16, scene-I=0.*B-MB=[1-9][0-9]*F/[1-9][0-9]*B/[1-9][0-9]*I/[1-9][0-9]*D.*gops=4$' "$TMP/thread1.log"
grep -Fq 'keyint=8, gop-grid=scene-reset, bframes=2' "$TMP/thread1.log"
grep -Eq '^vc1enc: execution: threads=1, intra-gop=' "$TMP/thread1.log"
grep -Eq '^encoded .*I=4, P=12, B=16, scene-I=0.*B-MB=[1-9][0-9]*F/[1-9][0-9]*B/[1-9][0-9]*I/[1-9][0-9]*D.*gops=4$' "$TMP/thread4.log"
grep -Fq 'keyint=8, gop-grid=scene-reset, bframes=2' "$TMP/thread4.log"
grep -Eq '^vc1enc: execution: threads=4, intra-gop=' "$TMP/thread4.log"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/thread4.vc1" \
  -frames:v 32 -f null -
# Default two-B structure must be visible in presentation order and the
# encoder's display-order reconstruction must exactly match an independent
# decoder despite I/P/B coded-order reordering.
b_types=$($FFPROBE -v error -select_streams v:0 -show_entries frame=pict_type -of csv=p=0 "$TMP/thread4.m2ts" | tr -d '\r' | head -n 8 | paste -sd, -)
[ "$b_types" = "I,B,B,P,B,B,P,P" ]
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/thread4.vc1" \
  -frames:v 32 -pix_fmt yuv420p -f rawvideo -y "$TMP/thread4-dec.yuv"
cmp "$TMP/thread4-recon.yuv" "$TMP/thread4-dec.yuv"
# PES DTS must be monotonic and never exceed PTS after the one-frame reorder delay.
$FFPROBE -v error -select_streams v -show_entries packet=pts,dts -of csv=p=0 "$TMP/thread4.m2ts" | \
  awk -F, 'BEGIN{last=-1} NF>=2 { if ($2<last || $1<$2) exit 1; last=$2 }'
if "$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/bad-threads.m2ts" --threads 0 >/dev/null 2>&1; then
  smoke_fail "zero threads unexpectedly accepted"
fi

# ---- 0.1.36 intra-GOP diagnostic switch ----
smoke_section '0.1.36 intra-GOP diagnostic switch'
# The new parallel scene/B analysis must not alter any coding decision.  The
# disable switch intentionally exercises only the historical GOP-worker model
# so scheduler behavior can be diagnosed independently of intra-GOP work.
"$ENCODER" -i "$TMP/wmv.y4m" -o "$TMP/intragop-on.m2ts" --es-out "$TMP/intragop-on.vc1" \
  --cq 9 --no-scene-cut --threads 4 --simd scalar --intra-gop-parallelism 2>"$TMP/intragop-on.log"
"$ENCODER" -i "$TMP/wmv.y4m" -o "$TMP/intragop-off.m2ts" --es-out "$TMP/intragop-off.vc1" \
  --cq 9 --no-scene-cut --threads 4 --simd scalar --no-intra-gop-parallelism 2>"$TMP/intragop-off.log"
cmp "$TMP/intragop-on.vc1" "$TMP/intragop-off.vc1"
grep -q 'intra-gop=on' "$TMP/intragop-on.log"
grep -q 'intra-gop=off' "$TMP/intragop-off.log"

# ---- 0.1.12 variable-transform / visualization regressions ----
smoke_section '0.1.12 variable-transform / visualization regressions'
# A normal moving texture must exercise every progressive inter transform size.
# The encoder-side reconstruction must remain byte-identical to FFmpeg with
# rectangular transforms and mixed B modes active simultaneously.
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=160x120:rate=24 \
  -frames:v 8 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/xform.y4m"
"$ENCODER" -i "$TMP/xform.y4m" -o "$TMP/xform.m2ts" --es-out "$TMP/xform.vc1" \
  --recon-out "$TMP/xform-recon.yuv" --debug-transforms "$TMP/xform-debug.y4m" \
  --cq 9 --no-scene-cut --threads 2 2>"$TMP/xform.log"
grep -Eq 'TT=8x8:[1-9][0-9]*/8x4:[1-9][0-9]*/4x8:[1-9][0-9]*/4x4:[1-9][0-9]*' "$TMP/xform.log"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/xform.vc1" \
  -frames:v 8 -pix_fmt yuv420p -f rawvideo -y "$TMP/xform-dec.yuv"
cmp "$TMP/xform-recon.yuv" "$TMP/xform-dec.yuv"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -i "$TMP/xform-debug.y4m" -frames:v 8 -f null -

# CQ4 previously exposed a B-direct FASTUVMC chroma-rounding asymmetry after
# variable transforms changed the collocated future-anchor MV.  Exercise that
# exact small reproducer and require byte-exact chroma reconstruction.
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=96x64:rate=24 \
  -frames:v 6 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/xform-q4.y4m"
"$ENCODER" -i "$TMP/xform-q4.y4m" -o "$TMP/xform-q4.m2ts" --es-out "$TMP/xform-q4.vc1" \
  --recon-out "$TMP/xform-q4-recon.yuv" --cq 4 --trellis 2 --threads 1 2>"$TMP/xform-q4.log"
# 0.1.69 TTFRM may legitimately select one picture-wide transform family on
# this low-Q fixture. All-four-size coverage remains enforced by xform.log
# above; this reproducer keeps Direct-B plus 4x4 and reconstruction coverage.
grep -Eq 'B-MB=.*[1-9][0-9]*D.*TT=.*4x4:[1-9][0-9]*' "$TMP/xform-q4.log"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/xform-q4.vc1" \
  -frames:v 6 -pix_fmt yuv420p -f rawvideo -y "$TMP/xform-q4-dec.yuv"
cmp "$TMP/xform-q4-recon.yuv" "$TMP/xform-q4-dec.yuv"

# Enabling visualization must not perturb any coding decision or bitstream.
"$ENCODER" -i "$TMP/xform.y4m" -o "$TMP/xform-nodebug.m2ts" --es-out "$TMP/xform-nodebug.vc1" \
  --cq 9 --no-scene-cut --threads 2 2>/dev/null
cmp "$TMP/xform.m2ts" "$TMP/xform-nodebug.m2ts"
cmp "$TMP/xform.vc1" "$TMP/xform-nodebug.vc1"

# Keep a fixed-8x8 compatibility/benchmark path. On this deterministic content
# variable transforms must save syntax at the same quantizer rather than merely
# exercising extra modes.
"$ENCODER" -i "$TMP/xform.y4m" -o "$TMP/xform-fixed.m2ts" --es-out "$TMP/xform-fixed.vc1" \
  --cq 9 --no-scene-cut --fixed-8x8 --threads 2 2>"$TMP/xform-fixed.log"
grep -Eq 'TT=8x8:[1-9][0-9]*/8x4:0/4x8:0/4x4:0' "$TMP/xform-fixed.log"
xform_var_size=$(wc -c < "$TMP/xform.vc1")
xform_fixed_size=$(wc -c < "$TMP/xform-fixed.vc1")
[ "$xform_var_size" -lt "$xform_fixed_size" ]

# Prediction-only P pictures in 0.1.4 produced paint-like smearing on changing
# content. Require a healthy quality floor across a long P run so residual
# transmission cannot silently disappear again.
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=160x120:rate=24 \
  -frames:v 12 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/pquality.y4m"
"$ENCODER" -i "$TMP/pquality.y4m" -o "$TMP/pquality.m2ts" --bframes 0 --no-scene-cut --keyint 999 $CQ9 2>"$TMP/pquality.log"
p_psnr=$($FFMPEG -hide_banner -loglevel info -i "$TMP/pquality.m2ts" -i "$TMP/pquality.y4m" \
  -lavfi '[0:v][1:v]psnr' -frames:v 12 -f null - 2>&1 | \
  sed -n 's/.*average:\([0-9.][0-9.]*\).*/\1/p' | tail -n 1)
[ -n "$p_psnr" ]
awk -v p="$p_psnr" 'BEGIN { exit !(p > 35.0) }'
grep -Eq 'coded-MB=[1-9][0-9]*.*coded-blocks=[1-9][0-9]*' "$TMP/pquality.log"


# ---- 0.1.7 low-PQ / periodic-I quality regressions ----
smoke_section '0.1.7 low-PQ / periodic-I quality regressions'
# 0.1.6 clipped intra DC to +/-255 even though PQ1 has a 10-bit DC escape,
# causing the first I frame and each one-second refresh to collapse to about
# 25.5 dB while surrounding P pictures remained near 56 dB.  Exercise the
# default-rate 38 Mbit/s path with an explicit historical 24-frame cadence,
# which keeps I/P at PQ1 while the 0.1.41
# reference-priority policy deliberately puts disposable B pictures at PQ2.
# HRD peak and buffer are represented in discrete VC-1 header units, so
# the diagnostic reports the represented 37,999,616/29,999,616 values,
# not the requested 38,000,000/30,000,000 values.
# Require every frame -- especially frames 1 and 25 -- to stay visually clean.
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=320x240:rate=24 \
  -frames:v 26 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/keyquality.y4m"
"$ENCODER" -i "$TMP/keyquality.y4m" -o "$TMP/keyquality.m2ts" --es-out "$TMP/keyquality.vc1" \
  --no-scene-cut --search-range 4 --keyint 24 2>"$TMP/keyquality.log"
grep -Eq '^encoded .*I=2, P=9, B=15, scene-I=0.*hrd-rate=37999616 bps, hrd-buffer=29999616 bits.*hrd-underflows=0.*q=1\.\.2' "$TMP/keyquality.log"
grep -Fq 'keyint=24, gop-grid=scene-reset, bframes=2' "$TMP/keyquality.log"
grep -Fq 'vc1enc: rate control: rate-mode=abr, target=38000000 bps, peak=37999616 bps, buffer=29999616 bits' "$TMP/keyquality.log"
"$FFMPEG" -hide_banner -loglevel error -i "$TMP/keyquality.y4m" -frames:v 26 \
  -pix_fmt yuv420p -f rawvideo -y "$TMP/keyquality-src.yuv"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/keyquality.vc1" \
  -frames:v 26 -pix_fmt yuv420p -f rawvideo -y "$TMP/keyquality-dec.yuv"
keyquality=$(python3 "$(dirname "$0")/check_frame_quality.py" \
  "$TMP/keyquality-src.yuv" "$TMP/keyquality-dec.yuv" --width 320 --height 240 --frames 26 \
  --min-psnr 50 --keyframe 1 --keyframe 25 --keyframe-min 50)

# Directly exercise the widened conservative Escape-3/DC syntax at PQ1/PQ2.
# The encoder reconstruction must remain byte-identical to an independent
# decoder even when transform levels exceed the old eight-bit prototype range.
for q in 1 2; do
  "$ENCODER" -i "$TMP/in.y4m" -o "$TMP/lowpq$q.m2ts" --es-out "$TMP/lowpq$q.vc1" \
    --recon-out "$TMP/lowpq$q-recon.yuv" --intra-only --cq "$q" 2>/dev/null
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/lowpq$q.vc1" \
    -frames:v 3 -pix_fmt yuv420p -f rawvideo -y "$TMP/lowpq$q-dec.yuv"
  cmp "$TMP/lowpq$q-recon.yuv" "$TMP/lowpq$q-dec.yuv"
done

# ---- 0.1.16 content-adaptive quality / difficult-content regression ----
smoke_section '0.1.16 content-adaptive quality / difficult-content regression'
# This synthetic one-GOP sequence intentionally puts dark textured material
# *after* rapid cuts and high-motion detail.  That catches the old behavior
# where burst repayment could starve the following shot, while the final
# credits-like region stresses dense black/white edges and trellis pruning.
python3 - "$TMP/aq-stress.y4m" <<'PYAQGEN'
import sys
import numpy as np
w,h,n=192,112,30
out=bytearray(f"YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n".encode())
Y,X=np.mgrid[0:h,0:w]
for i in range(n):
    if i < 6:
        y=np.full((h,w),96+(i&1),np.uint8)
    elif i < 12:
        m=i%3
        if m == 0:
            y=np.where(((X//3+Y//3)&1),235,16).astype(np.uint8)
        elif m == 1:
            y=np.where(((X//2)&1),225,22).astype(np.uint8)
        else:
            y=np.random.default_rng(1000+i).integers(16,236,(h,w),dtype=np.uint8)
    elif i < 18:
        y=np.where((((X+7*i)//2+(Y+5*i)//2)&1),225,25).astype(np.uint8)
    elif i < 24:
        base=18+((X+2*i)//8%2)*6+((Y+i)//8%2)*4
        y=np.clip(base+3*np.sin((X+i)*.35)+2*np.cos((Y+2*i)*.4),4,55).astype(np.uint8)
    else:
        y=np.full((h,w),16,np.uint8)
        shift=(i-24)*2
        for row in range(5,h-4,8):
            for col in range(-20,w,22):
                x0=(col+shift+(row%17))%(w+20)-10
                length=8+((row+col)%10)
                y[row:row+2,max(0,x0):min(w,x0+length)]=235
                if row+3<h:
                    y[row+3:row+4,max(0,x0+2):min(w,x0+length-1)]=210
    u=np.full((h//2,w//2),128,np.uint8)
    out += b"FRAME\n"+y.tobytes()+u.tobytes()+u.tobytes()
open(sys.argv[1],"wb").write(out)
PYAQGEN

"$ENCODER" -i "$TMP/aq-stress.y4m" -o "$TMP/aq.m2ts" --es-out "$TMP/aq.vc1" \
  --bitrate 500k --buffer-size 1600k --keyint 30 --threads 1 --no-scene-cut --fixed-gop-grid \
  --recon-out "$TMP/aq-recon.yuv" 2>"$TMP/aq.log"
"$ENCODER" -i "$TMP/aq-stress.y4m" -o "$TMP/noaq.m2ts" --es-out "$TMP/noaq.vc1" \
  --bitrate 500k --buffer-size 1600k --keyint 30 --threads 1 --no-scene-cut --fixed-gop-grid --no-aq \
  --recon-out "$TMP/noaq-recon.yuv" 2>"$TMP/noaq.log"
# Strength zero is a compatibility spelling of disabling AQ entirely.
"$ENCODER" -i "$TMP/aq-stress.y4m" -o "$TMP/aq0.m2ts" --es-out "$TMP/aq0.vc1" \
  --bitrate 500k --buffer-size 1600k --keyint 30 --threads 1 --no-scene-cut --fixed-gop-grid --aq-strength 0 \
  --recon-out "$TMP/aq0-recon.yuv" 2>/dev/null
cmp "$TMP/noaq.m2ts" "$TMP/aq0.m2ts"
cmp "$TMP/noaq.vc1" "$TMP/aq0.vc1"
cmp "$TMP/noaq-recon.yuv" "$TMP/aq0-recon.yuv"
grep -Fq 'keyint=30, gop-grid=fixed, bframes=2' "$TMP/aq.log"
grep -Fq 'aq=1.000000' "$TMP/aq.log"
grep -Eq '^vc1enc: rate control: rate-mode=abr, target=' "$TMP/aq.log"
grep -Eq '^encoded .*hrd-underflows=0' "$TMP/aq.log"
grep -Fq 'keyint=30, gop-grid=fixed, bframes=2' "$TMP/noaq.log"
grep -Fq 'aq=off' "$TMP/noaq.log"
grep -Eq '^vc1enc: rate control: rate-mode=abr, target=' "$TMP/noaq.log"
grep -Eq '^encoded .*hrd-underflows=0' "$TMP/noaq.log"

"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/aq.vc1" \
  -frames:v 30 -pix_fmt yuv420p -f rawvideo -y "$TMP/aq-dec.yuv"
cmp "$TMP/aq-recon.yuv" "$TMP/aq-dec.yuv"

python3 - "$TMP/aq-stress.y4m" "$TMP/noaq-recon.yuv" "$TMP/aq-recon.yuv" <<'PYAQCHECK'
import sys
import numpy as np
w,h,n=192,112,30
frame_bytes=w*h*3//2
raw=open(sys.argv[1],'rb').read()
pos=raw.find(b'\n')+1
src=[]
for _ in range(n):
    if not raw.startswith(b'FRAME',pos): raise SystemExit('bad AQ regression Y4M')
    pos=raw.find(b'\n',pos)+1
    src.append(np.frombuffer(raw[pos:pos+w*h],dtype=np.uint8).copy())
    pos += frame_bytes
src=np.stack(src)
def load(path):
    a=np.fromfile(path,dtype=np.uint8)
    if a.size != n*frame_bytes: raise SystemExit('bad AQ reconstruction length')
    return a.reshape(n,frame_bytes)[:,:w*h]
old=load(sys.argv[2]); new=load(sys.argv[3])
regions={'cuts':range(6,12),'motion':range(12,18),'dark':range(18,24),'credits':range(24,30)}
# 0.1.72 materially improved the no-AQ motion/quantizer path. Comparing AQ to
# that moving target makes an encoder improvement look like an AQ regression.
# Preserve the deterministic 0.1.71 no-AQ reconstruction as the absolute floor,
# matching abr_transform_aq_runtime.py; current no-AQ remains diagnostic only.
old_noaq={'cuts':243.102,'motion':24.067,'dark':11.923,'credits':20.527}
for name,ids0 in regions.items():
    ids=list(ids0)
    d0=src[ids].astype(np.int16)-old[ids].astype(np.int16)
    d1=src[ids].astype(np.int16)-new[ids].astype(np.int16)
    mse0=float(np.mean(d0.astype(np.float64)**2))
    mse1=float(np.mean(d1.astype(np.float64)**2))
    ceiling=old_noaq[name]
    if mse1 > ceiling:
        raise SystemExit(f'AQ {name} absolute regression: MSE {mse1:.3f} > 0.1.71-noAQ baseline {ceiling:.3f} (new no-AQ {mse0:.3f})')
    print(f'AQ {name}: new no-AQ {mse0:.3f}, AQ {mse1:.3f}, 0.1.71-noAQ ceiling {ceiling:.3f}')
PYAQCHECK

# ---- 0.1.22 flat-background / detailed-foreground AQ regression ----
smoke_section '0.1.22 flat-background / detailed-foreground AQ regression'
# A large white field must not dilute the frame-complexity estimate for a
# smaller highly detailed foreground.  The second half checks a gray region
# immediately beside very dense detail, where boundary/blocking error is very
# visible even though the gray block itself is inexpensive.
python3 - "$TMP/aq-context.y4m" <<'PYAQCTXGEN'
import sys
import numpy as np
w,h,n=192,112,30
Y,X=np.mgrid[0:h,0:w]
out=bytearray(f"YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n".encode())
for i in range(n):
    if i < 10:
        y=np.full((h,w),230+(i&1),np.uint8)
    elif i < 20:
        y=np.full((h,w),240,np.uint8)
        x0=52+(i%4)*4; x1=x0+64
        xx=X[:,x0:x1]; yy=Y[:,x0:x1]
        y[:,x0:x1]=np.where((((xx+i*3)//2+(yy+i*5)//2)&1),235,12).astype(np.uint8)
        y[::7,x0:x1]=48
    else:
        y=np.full((h,w),210,np.uint8)
        y[:,:112]=(46+((X[:,:112]//8+Y[:,:112]//8+i)&1)*6).astype(np.uint8)
        xx=X[:,112:]; yy=Y[:,112:]
        y[:,112:]=np.where((((xx+i*4)//2+(yy+i*3)//2)&1),238,12).astype(np.uint8)
    u=np.full((h//2,w//2),128,np.uint8)
    out += b"FRAME\n"+y.tobytes()+u.tobytes()+u.tobytes()
open(sys.argv[1],"wb").write(out)
PYAQCTXGEN

"$ENCODER" -i "$TMP/aq-context.y4m" -o "$TMP/aq-context.m2ts" \
  --bitrate 500k --buffer-size 1600k --keyint 30 --threads 1 --search-range 0 --no-scene-cut --fixed-gop-grid \
  --recon-out "$TMP/aq-context.yuv" 2>/dev/null
"$ENCODER" -i "$TMP/aq-context.y4m" -o "$TMP/aq-context-noaq.m2ts" \
  --bitrate 500k --buffer-size 1600k --keyint 30 --threads 1 --search-range 0 --no-scene-cut --fixed-gop-grid --no-aq \
  --recon-out "$TMP/aq-context-noaq.yuv" 2>/dev/null
python3 - "$TMP/aq-context.y4m" "$TMP/aq-context-noaq.yuv" "$TMP/aq-context.yuv" <<'PYAQCTXCHECK'
import sys
import numpy as np
w,h,n=192,112,30
fb=w*h*3//2
raw=open(sys.argv[1],'rb').read(); pos=raw.find(b'\n')+1; src=[]
for _ in range(n):
    pos=raw.find(b'\n',pos)+1
    src.append(np.frombuffer(raw[pos:pos+w*h],dtype=np.uint8).copy().reshape(h,w))
    pos += fb
src=np.stack(src)
def load(path):
    a=np.fromfile(path,dtype=np.uint8)
    if a.size != n*fb: raise SystemExit('bad AQ-context reconstruction length')
    return a.reshape(n,fb)[:,:w*h].reshape(n,h,w)
old=load(sys.argv[2]); new=load(sys.argv[3])
def mse(a,b): return float(np.mean((a.astype(np.float64)-b.astype(np.float64))**2))
checks=[
    ('white-foreground', src[10:20,:,52:132], old[10:20,:,52:132], new[10:20,:,52:132], 2.036),
    ('gray-detail-boundary', src[20:,:,80:112], old[20:,:,80:112], new[20:,:,80:112], 13.764),
]
for name,s,o,nw,old_base in checks:
    m0=mse(s,o); m1=mse(s,nw)
    if m1 > old_base:
        raise SystemExit(f'AQ-context {name} absolute regression: MSE {m1:.3f} > 0.1.71-noAQ baseline {old_base:.3f} (new no-AQ {m0:.3f})')
    print(f'AQ-context {name}: new no-AQ {m0:.3f}, AQ {m1:.3f}, 0.1.71-noAQ ceiling {old_base:.3f}')
PYAQCTXCHECK

if "$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/bad-aq.m2ts" --aq-strength 3.1 >/dev/null 2>&1; then
  smoke_fail "invalid AQ strength unexpectedly accepted"
fi

# ---- 0.1.6 rate-control / HRD regressions ----
smoke_section '0.1.6 rate-control / HRD regressions'
# Use a compact complete 12-picture I/B/P group. The current checks validate
# represented 10/20/38 Mbit/s HRD syntax, buffer/fullness safety and decode;
# they no longer depend on legacy short-clip average-rate behavior.
"$FFMPEG" -hide_banner -loglevel error -f lavfi -i testsrc2=size=640x360:rate=24000/1001 \
  -frames:v 12 -pix_fmt yuv420p -f yuv4mpegpipe -y "$TMP/rc1080.y4m"

check_rc() {
  target="$1"
  tag="$2"
  "$ENCODER" -i "$TMP/rc1080.y4m" -o "$TMP/rc-$tag.m2ts" --es-out "$TMP/rc-$tag.vc1" \
    --bitrate "$target" --search-range 0 --no-scene-cut --keyint 999 2>"$TMP/rc-$tag.log"
  "$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/rc-$tag.vc1" \
    -frames:v 12 -f null -
  grep -Eq '^vc1enc: rate control: rate-mode=abr, target=' "$TMP/rc-$tag.log"
  grep -Eq '^encoded .*hrd-rate=[0-9]+ bps, hrd-buffer=29999616 bits.*hrd-underflows=0' "$TMP/rc-$tag.log"
  maxpre=$(sed -n 's/.*hrd-max-pre=\([0-9][0-9]*\) bits.*/\1/p' "$TMP/rc-$tag.log" | tail -n 1)
  [[ -n "$maxpre" ]]
  case "$tag" in
    10m) represented=9999872; full=131 ;;
    20m) represented=19999744; full=135 ;;
    38m) represented=37999616; full=141 ;;
  esac
  [ "$maxpre" -le 29999616 ]
  python3 "$(dirname "$0")/check_hrd.py" "$TMP/rc-$tag.vc1" \
    --expect-rate "$represented" --expect-buffer 29999616 --expect-full "$full" >/dev/null
  return 0
}
check_rc 10M 10m
check_rc 20M 20m
check_rc 38M 38m

# The no-option default is one-pass ABR+VBV with a 38 Mbit/s nominal target and
# the corresponding representable HRD rate and buffer (37,999,616/29,999,616).
"$ENCODER" -i "$TMP/rc1080.y4m" -o "$TMP/rc-default.m2ts" --max-frames 12 --search-range 0 --no-scene-cut --keyint 999 2>"$TMP/rc-default.log"
grep -Fq 'vc1enc: rate control: rate-mode=abr, target=38000000 bps, peak=37999616 bps, buffer=29999616 bits' "$TMP/rc-default.log"
grep -Eq '^encoded .*hrd-rate=37999616 bps, hrd-buffer=29999616 bits.*hrd-underflows=0' "$TMP/rc-default.log"

"$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/rc-stats.m2ts" --max-frames 4 --search-range 0 --no-scene-cut --rc-stats "$TMP/rc-stats.csv" >/dev/null 2>&1
head -n 1 "$TMP/rc-stats.csv" | grep -Fq 'display_order,coded_order,type,keyframe,complexity,predicted_q,final_q,predicted_bits,first_actual_bits,final_actual_bits,target_bits,allowed_bits,prediction_error_percent,vbv_before_bits,vbv_after_bits,retries,reencoded'
python3 - "$TMP/rc-stats.csv" <<'PYRCSTATS'
import csv,sys
r=list(csv.DictReader(open(sys.argv[1],newline='')))
if len(r)!=3: raise SystemExit(f'rate-control stats row count mismatch: expected 3 input pictures, got {len(r)}')
if not all(int(x['predicted_q'])>=1 and float(x['target_bits'])>0 and float(x['allowed_bits'])>0 for x in r):
    raise SystemExit('rate-control stats missing predictor/VBV data')
PYRCSTATS

"$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/debug-stats.m2ts" --max-frames 4 --search-range 0 --no-scene-cut --debug-stats "$TMP/debug-stats.csv" >/dev/null 2>&1
head -n 1 "$TMP/debug-stats.csv" | grep -Fq 'display_order,coded_order,gop_index,frame_in_gop,gop_frames,type,keyframe,keyframe_reason'
python3 - "$TMP/debug-stats.csv" <<'PYDEBUGSTATS'
import csv,sys
r=list(csv.DictReader(open(sys.argv[1],newline='')))
if len(r)!=3: raise SystemExit(f'debug statistics row count mismatch: expected 3 input pictures, got {len(r)}')
required=['qp','quant_step','qscale','final_actual_bits','gop_budget_scale','encode_trials','moved_mb','skipped_mb','coded_blocks','transform_8x8','ttmbf','ttfrm','ttfrm_exact_checked','acpred_mb','mean_mv_pixels','mse_y','snr_y_db','psnr_y_db','psnr_yuv_db','bits_per_pixel']
missing=[k for k in required if k not in r[0]]
if missing: raise SystemExit('debug statistics missing columns: '+','.join(missing))
if [int(x['display_order']) for x in r] != list(range(3)):
    raise SystemExit('debug statistics are not in display order')
if not all(int(x['encode_trials'])>=1 and int(x['final_actual_bits'])>0 and float(x['qscale'])>0 for x in r):
    raise SystemExit('debug statistics missing coding-decision data')
PYDEBUGSTATS


# Maximum-utilization mode biases the ABR qscale toward fuller nominal bitrate use while keeping the same hard serialized VBV.
"$ENCODER" -i "$TMP/rc1080.y4m" -o "$TMP/rc-maximize.m2ts" --max-frames 12 --search-range 0 --no-scene-cut --keyint 999 --rc-maximize 2>"$TMP/rc-maximize.log"
grep -Fq 'vc1enc: rate control: rate-mode=abr-max, target=38000000 bps, peak=37999616 bps, buffer=29999616 bits' "$TMP/rc-maximize.log"
grep -Eq '^encoded .*hrd-rate=37999616 bps, hrd-buffer=29999616 bits.*hrd-underflows=0' "$TMP/rc-maximize.log"

# One flat GOP followed by two hard checker/motion GOPs provides a severe
# deterministic VBV/recovery fixture for the surviving abr-like controller.
python3 - "$TMP/rc-recovery.y4m" <<'PYRCRECOVERY'
import sys
import numpy as np
w,h,n=320,180,72
Y,X=np.mgrid[0:h,0:w]
out=bytearray(f"YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n".encode())
for i in range(n):
    if i < 24:
        y=np.full((h,w),96,np.uint8)
    else:
        j=i-24
        y=np.where((((X+7*j)//2+(Y+5*j)//2)&1),235,16).astype(np.uint8)
        y[:,::11]=64
        y[::13,:]=192
    uv=np.full((h//2,w//2),128,np.uint8)
    out += b"FRAME\n"+y.tobytes()+uv.tobytes()+uv.tobytes()
open(sys.argv[1],"wb").write(out)
PYRCRECOVERY

# The abr-inspired ABR+VBV controller must remain deterministic on this
# burst-heavy sequence while avoiding unexpected recovery retries.
for th in 1 4; do
  "$ENCODER" -i "$TMP/rc-recovery.y4m" -o "$TMP/abr-recovery-$th.m2ts" \
    --es-out "$TMP/abr-recovery-$th.vc1" --bitrate 1M --buffer-size 8M \
    --keyint 24 --threads "$th" --search-range 0 --no-scene-cut \
    --no-skip-identical-frames --no-intra-gop-parallelism --rc-stats "$TMP/abr-recovery-$th.csv" \
    2>"$TMP/abr-recovery-$th.log"
  grep -Fq 'vc1enc: rate control: rate-mode=abr, target=1000000 bps, peak=1000000 bps, buffer=8000000 bits' "$TMP/abr-recovery-$th.log"
  grep -Eq '^encoded .*hrd-rate=1000000 bps, hrd-buffer=8000000 bits.*hrd-underflows=0' "$TMP/abr-recovery-$th.log"
  python3 - "$TMP/abr-recovery-$th.csv" <<'PYABRRC'
import csv,sys
r=list(csv.DictReader(open(sys.argv[1],newline='')))
if not r:
    raise SystemExit('abr-style severe regression produced no rate-control rows')
# 0.1.46 introduced one narrowly gated *quality* retry for a difficult B
# picture that falls off VC-1's skip/residual cliff.  That retry lowers Q and
# increases payload; it is intentionally different from VBV/recovery retries,
# which raise Q to reduce payload.  This severe fixture legitimately triggers
# the quality repair on its first hard B picture in each hard GOP.
for x in r:
    n=int(x['retries'])
    if not n:
        continue
    pq=int(x['predicted_q']); fq=int(x['final_q'])
    first=int(float(x['first_actual_bits'])); final=int(float(x['final_actual_bits']))
    target=float(x['target_bits'])
    quality_retry=(x['type']=='B' and n==1 and int(x['reencoded'])==1 and
                   fq<pq and final>first and first<target*0.20)
    # 0.1.49 adds one separate GOP-opening I overspend correction. It raises Q
    # and reduces payload when the first entropy result is grossly above the
    # planned I share, but is not a VBV recovery: the first pass must already
    # fit comfortably inside the current authoritative picture allowance.
    allowed=float(x['allowed_bits'])
    i_overspend_retry=(x['type']=='I' and n==1 and int(x['reencoded'])==1 and
                       fq>pq and final<first and first>target*1.65 and
                       first<allowed*0.95)
    if not (quality_retry or i_overspend_retry):
        raise SystemExit('abr-style severe regression used an unexpected retry: '+str(x))
PYABRRC
done
cmp "$TMP/abr-recovery-1.vc1" "$TMP/abr-recovery-4.vc1"
cmp "$TMP/abr-recovery-1.m2ts" "$TMP/abr-recovery-4.m2ts"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/abr-recovery-1.vc1" -frames:v 72 -f null -

# 0.1.41: sustained hard GOPs must not drain VBV until a whole following GOP
# is forced coarse. This historical threshold fixture pins the former 3.5/1/0.70
# block weights so the 0.2.32 default-I-weight change does not redefine its baseline.
# Disposable B pictures must never become finer than
# their P anchors. This compact fixture reproduces the 0.1.40 good/bad-GOP
# flicker without requiring a large HD encode.
python3 - "$TMP/abr-gop-balance.y4m" <<'PYABRBALGEN'
import sys
w,h,n=96,64,144
with open(sys.argv[1],'wb') as f:
    f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    uv=bytes([128])*(w*h//4)
    for i in range(n):
        f.write(b'FRAME\n')
        y=bytearray(w*h)
        for yy in range(h):
            for xx in range(w):
                sx=(xx-(i*3)%w)%w
                base=(sx*9+yy*13)%220+16
                chk=30 if ((xx//2+yy//2+i)&1) else -30
                noise=((xx*17+yy*29+i*31+(xx*yy)%19)%29)-14
                y[yy*w+xx]=max(16,min(235,base+chk+noise))
        f.write(y); f.write(uv); f.write(uv)
PYABRBALGEN
"$ENCODER" -i "$TMP/abr-gop-balance.y4m" -o "$TMP/abr-gop-balance.m2ts" \
  --bitrate 300000 --buffer-size 600000 --keyint 24 --threads 4 --faster \
  --i-block-weight 3.5 --p-block-weight 1.0 --b-block-weight 0.70 \
  --rc-stats "$TMP/abr-gop-balance.csv" 2>"$TMP/abr-gop-balance.log"
python3 - "$TMP/abr-gop-balance.csv" <<'PYABRBAL'
import csv,sys
r=list(csv.DictReader(open(sys.argv[1],newline='')))
if len(r)!=144: raise SystemExit(f'abr GOP-balance row count mismatch: {len(r)}')
for g in range(6):
    rr=[x for x in r if int(x['display_order'])//24==g]
    bits=sum(int(x['final_actual_bits']) for x in rr)
    if not (270000 <= bits <= 330000):
        raise SystemExit(f'abr GOP {g} budget oscillation: {bits} bits')
    ip=[int(x['final_q']) for x in rr if x['type']=='I']
    pp=[int(x['final_q']) for x in rr if x['type']=='P']
    bp=[int(x['final_q']) for x in rr if x['type']=='B']
    if not ip or not pp or not bp: raise SystemExit('abr GOP-balance frame-type plan missing')
    if ip[0] >= 31: raise SystemExit(f'abr GOP {g} refresh I collapsed to PQ31')
    # 0.1.45 repaired the worst B/reference Q gap. 0.1.46 additionally
    # makes hard B allocation follow the current post-prediction residual
    # instead of treating B pictures as the preferred place to repay GOP debt.
    by_display={int(x['display_order']):x for x in rr}
    repaired=[int(by_display[g*24+d]['final_q']) for d in (4,5)]
    payload=[int(by_display[g*24+d]['final_actual_bits']) for d in (4,5)]
    if max(repaired) > 22:
        raise SystemExit(f'abr GOP {g} residual-aware B-Q regression: {repaired} exceeds PQ22')
    if min(payload) < 7800:
        raise SystemExit(f'abr GOP {g} difficult-B underfill regression: {payload} below 7800 bits')
    # Match the encoder's actual B-reference invariant.  The abr-style
    # controller ties B quality to the geometric mean of the two surrounding
    # non-B qscales, not to the coarser integer PQ of either anchor.  Comparing
    # BQ against max(anchor PQ) is therefore too strict once residual-aware B
    # allocation is allowed to recover toward that midpoint (for example,
    # refs PQ22/PQ27 imply a VC-1 reference level of about PQ24).
    import math
    def q_to_scale(q):
        return max(0.5,q/2.0)**0.55
    def scale_to_q(s):
        raw=2.0*max(1e-9,s)**(1.0/0.55)
        return max(1,min(31,int(math.floor(raw+0.5))))
    anchors=sorted((d,int(x['final_q'])) for d,x in by_display.items() if x['type']!='B')
    for x in rr:
        if x['type']!='B': continue
        d=int(x['display_order']); bq=int(x['final_q'])
        past=[q for ad,q in anchors if ad<d]; future=[q for ad,q in anchors if ad>d]
        if not past or not future: continue
        refq=scale_to_q(math.sqrt(q_to_scale(past[-1])*q_to_scale(future[0])))
        if bq < refq:
            raise SystemExit(f'abr GOP {g} B@{d} finer than surrounding qscale midpoint: B={bq}, refs={past[-1]}/{future[0]}, midpoint={refq}')
if sum(int(x['retries']) for x in r): raise SystemExit('abr GOP-balance regression unexpectedly retried')
PYABRBAL
grep -Eq '^vc1enc: rate control: rate-mode=abr, target=' "$TMP/abr-gop-balance.log"
grep -Eq '^encoded .*hrd-underflows=0' "$TMP/abr-gop-balance.log"

# 0.1.46: zooming fine texture exercises the B-picture residual-allocation
# regression directly. 0.1.45 leaves many difficult B pictures at only a few
# kilobits, and one late B picture collapses almost completely at the
# skip/residual threshold. The residual-aware allocator should raise the hard-B
# average and the rare actual-size safety net should repair that cliff with no
# repeated motion analysis.
python3 - "$TMP/abr-b-texture.y4m" <<'PYABRBTEXGEN'
import sys,numpy as np
w,h,n=96,64,48; W,H=192,128
Y,X=np.mgrid[0:H,0:W]
tex=110+34*np.sin(X*.81)+29*np.sin(Y*.57)+22*np.sin((X+Y)*1.07)+(((X*43+Y*67+(X*Y*13)%257)%91)-45)
tex=np.clip(tex,16,235).astype(float); cx,cy=W/2,H/2
uv=bytes([128])*(w*h//4)
with open(sys.argv[1],'wb') as f:
    f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for i in range(n):
        ph=i/(n-1); scale=1.0+.42*ph; dx=.23*np.sin(i*.37); dy=.19*np.cos(i*.29)
        ox=(np.arange(w)-(w-1)/2)/scale+cx+dx
        oy=(np.arange(h)-(h-1)/2)/scale+cy+dy
        xx,yy=np.meshgrid(ox,oy); x0=np.floor(xx).astype(int); y0=np.floor(yy).astype(int)
        fx=xx-x0; fy=yy-y0; x0=np.clip(x0,0,W-2); y0=np.clip(y0,0,H-2)
        z=tex[y0,x0]*(1-fx)*(1-fy)+tex[y0,x0+1]*fx*(1-fy)+tex[y0+1,x0]*(1-fx)*fy+tex[y0+1,x0+1]*fx*fy
        y=np.clip(np.rint(z),16,235).astype(np.uint8)
        f.write(b'FRAME\n'); f.write(y.tobytes()); f.write(uv); f.write(uv)
PYABRBTEXGEN
"$ENCODER" -i "$TMP/abr-b-texture.y4m" -o "$TMP/abr-b-texture.m2ts" --es-out "$TMP/abr-b-texture.vc1" \
  --bitrate 300000 --buffer-size 600000 --keyint 24 --threads 4 --faster \
  --rc-stats "$TMP/abr-b-texture.csv" 2>"$TMP/abr-b-texture.log"
python3 - "$TMP/abr-b-texture.csv" <<'PYABRBTEX'
import csv,sys
r=list(csv.DictReader(open(sys.argv[1],newline='')))
if len(r)!=48: raise SystemExit(f'B-texture regression row count mismatch: {len(r)}')
hard=[x for x in r if x['type']=='B' and float(x['complexity'])>=3.0]
# Group-B qpel/Mixed-MV prediction makes fewer B pictures exceed the historical
# residual-complexity>=3 threshold; keep enough samples to exercise allocation.
if len(hard)<15: raise SystemExit(f'B-texture regression did not exercise enough hard B pictures: {len(hard)}')
avg=sum(int(x['final_actual_bits']) for x in hard)/len(hard)
if avg < 7000:
    raise SystemExit(f'B-texture residual allocation regression: hard-B average {avg:.1f} bits < 7000')
retries=sum(int(x['retries']) for x in r)
# The 0.1.54 forward GOP lookahead can keep this texture above the skip cliff
# proactively, so zero retries is now preferable.  One or two legacy safety
# retries remain acceptable if predictor error still reaches the cliff.
if retries>2:
    raise SystemExit(f'B-texture skip-cliff safety retry count unexpected: {retries}')
if min(int(x['final_actual_bits']) for x in hard) < 2500:
    raise SystemExit('B-texture skip-cliff repair left a difficult B picture below 2500 bits')
total=sum(int(x['final_actual_bits']) for x in r)
if not (570000 <= total <= 630000):
    raise SystemExit(f'B-texture total-rate regression: {total} bits')
PYABRBTEX
grep -Eq '^vc1enc: rate control: rate-mode=abr, target=' "$TMP/abr-b-texture.log"
grep -Eq '^encoded .*hrd-underflows=0' "$TMP/abr-b-texture.log"
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/abr-b-texture.vc1" -frames:v 48 -f null -

# 0.1.49: high-resolution fine detail must not alias to zero I complexity.
# This historical overspend threshold also pins the former 3.5/1/0.70 block
# weights; the current default-I-weight behavior is covered separately.
# The old 2x2 activity sampler can report complexity=0 on this aligned 4x4
# texture, emit an oversized PQ1 refresh I, and then crush the first P anchor
# to roughly PQ30 while the GOP repays that accidental I debt.
python3 - "$TMP/abr-hires-i.y4m" <<'PYABRHII'
import sys,numpy as np
w,h,n=1280,720,4
rng=np.random.default_rng(999)
small=rng.integers(32,224,size=(h//4,w//4),dtype=np.uint8)
y=np.repeat(np.repeat(small,4,axis=0),4,axis=1)
uv=bytes([128])*(w*h//4)
with open(sys.argv[1],'wb') as f:
    f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for _ in range(n):
        f.write(b'FRAME\n'); f.write(y.tobytes()); f.write(uv); f.write(uv)
PYABRHII
"$ENCODER" -i "$TMP/abr-hires-i.y4m" -o "$TMP/abr-hires-i.m2ts" \
  --bitrate 37999616 --buffer-size 29999616 --keyint 24 --threads 1 --search-range 0 \
  --no-scene-cut --no-skip-identical-frames --faster \
  --i-block-weight 3.5 --p-block-weight 1.0 --b-block-weight 0.70 \
  --rc-stats "$TMP/abr-hires-i.csv" 2>"$TMP/abr-hires-i.log"
python3 - "$TMP/abr-hires-i.csv" <<'PYABRHICHECK'
import csv,sys
r=list(csv.DictReader(open(sys.argv[1],newline='')))
if len(r)!=4: raise SystemExit(f'hires-I regression row count mismatch: {len(r)}')
i=next(x for x in r if x['type']=='I')
p=next(x for x in r if x['type']=='P')
if float(i['complexity']) < 8.0:
    raise SystemExit(f'hires-I activity aliased away: complexity={i["complexity"]}')
if int(i['final_actual_bits']) > 5000000:
    raise SystemExit(f'hires-I overspend repair failed: {i["final_actual_bits"]} bits')
if int(p['final_q']) > 12:
    raise SystemExit(f'hires-I debt still starved first P: PQ{p["final_q"]}')
if int(i['retries']) > 1:
    raise SystemExit(f'hires-I overspend repair retried too many times: {i["retries"]}')
PYABRHICHECK
grep -Eq '^vc1enc: rate control: rate-mode=abr, target=' "$TMP/abr-hires-i.log"
grep -Eq '^encoded .*hrd-underflows=0' "$TMP/abr-hires-i.log"

# 0.1.43: a deliberately expensive refresh I must not be charged as if it had
# only an ordinary one-frame allocation. That bug made the first few P/B
# pictures of some GOPs jump to very coarse Q before the local rate-factor
# window recovered. Keep GOP 2 spatially detailed but perfectly stationary so
# the first P/B pictures are cheap to predict; any large post-I Q spike is rate
# control, not motion complexity.
python3 - "$TMP/abr-post-i.y4m" <<'PYABRPOSTIGEN'
import sys
w,h,n=96,64,72
tex=bytes(max(16,min(235,16+((x*37+y*53+(x*y*11))%220))) for y in range(h) for x in range(w))
flat=bytes([96])*(w*h)
checker=bytes(40 if ((x//4+y//4)&1) else 200 for y in range(h) for x in range(w))
with open(sys.argv[1],'wb') as f:
    f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    uv=bytes([128])*(w*h//4)
    for i in range(n):
        f.write(b'FRAME\n')
        g=i//24
        y=flat if g==0 else (tex if g==1 else checker)
        f.write(y); f.write(uv); f.write(uv)
PYABRPOSTIGEN
"$ENCODER" -i "$TMP/abr-post-i.y4m" -o "$TMP/abr-post-i.m2ts" \
  --bitrate 300000 --buffer-size 600000 --keyint 24 --threads 4 --faster \
  --no-skip-identical-frames --rc-stats "$TMP/abr-post-i.csv" 2>"$TMP/abr-post-i.log"
python3 - "$TMP/abr-post-i.csv" <<'PYABRPOSTI'
import csv,sys
r=list(csv.DictReader(open(sys.argv[1],newline='')))
g=[x for x in r if 24 <= int(x['display_order']) < 48]
if len(g)!=24: raise SystemExit(f'post-I regression GOP row count mismatch: {len(g)}')
p=[int(x['final_q']) for x in g if x['type']=='P']
b=[int(x['final_q']) for x in g if x['type']=='B']
if not p or len(b)<2: raise SystemExit('post-I regression frame-type plan missing')
if p[0] > 10: raise SystemExit(f'post-I P starvation regression: first P PQ{p[0]} > 10')
if max(b[:2]) > 20: raise SystemExit(f'post-I B starvation regression: first B pair {b[:2]} exceeds PQ20')
if sum(int(x['retries']) for x in g): raise SystemExit('post-I regression unexpectedly retried')
PYABRPOSTI
grep -Eq '^vc1enc: rate control: rate-mode=abr, target=' "$TMP/abr-post-i.log"
grep -Eq '^encoded .*hrd-underflows=0' "$TMP/abr-post-i.log"

# CQ holds one quantizer and disables HRD signaling.
"$ENCODER" -i "$TMP/rc1080.y4m" -o "$TMP/cq13.m2ts" --es-out "$TMP/cq13.vc1" --cq 13 --max-frames 12 --search-range 0 \
  --no-scene-cut --keyint 999 2>"$TMP/cq13.log"
grep -Eq 'rate-mode=CQ, q=13' "$TMP/cq13.log"
python3 "$(dirname "$0")/check_hrd.py" "$TMP/cq13.vc1" --expect-no-hrd >/dev/null
"$FFMPEG" -hide_banner -v error -xerror -err_detect explode -f vc1 -i "$TMP/cq13.vc1" -frames:v 12 -f null -

# Blu-ray codec guard rails are opt-in. Generic Advanced VC-1 may exceed the
# disc subset while remaining inside AP@L3/L4; --bluray-compat restores the
# 40 Mbit/s / 30 Mbit subset limits. Use the existing 1080p fixture so rejection
# is specifically about rate/buffer rather than a non-Blu-ray picture format.
"$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/generic-rate.m2ts" --bitrate 40000001 --max-frames 1 --search-range 0 >/dev/null 2>&1 || \
  smoke_fail "generic VC-1 unexpectedly retained Blu-ray 40 Mbit/s ceiling"
if "$ENCODER" -i "$TMP/rc1080.y4m" -o "$TMP/bad-rate.m2ts" --bluray-compat --bitrate 40000001 --max-frames 1 >/dev/null 2>&1; then
  smoke_fail "--bluray-compat bitrate above 40 Mbit/s unexpectedly accepted"
fi
if "$ENCODER" -i "$TMP/rc1080.y4m" -o "$TMP/bad-buffer.m2ts" --bluray-compat --buffer-size 30000001 --max-frames 1 >/dev/null 2>&1; then
  smoke_fail "--bluray-compat buffer above 30 Mbit unexpectedly accepted"
fi
if "$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/bad-mix.m2ts" --bitrate 20M --cq 9 >/dev/null 2>&1; then
  smoke_fail "bitrate and CQ unexpectedly accepted together"
fi
if "$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/bad-max-cq.m2ts" --cq 9 --rc-maximize >/dev/null 2>&1; then
  smoke_fail "maximum-utilization rate control unexpectedly accepted in CQ mode"
fi
if "$ENCODER" -i "$TMP/flat.y4m" -o "$TMP/bad-bframes.m2ts" --bframes 3 >/dev/null 2>&1; then
  smoke_fail "unsupported B-frame count unexpectedly accepted"
fi

if [ ! -s "$SMOKE_FAILURES" ]; then
  echo "AC PSNR=$ac_psnr dB; DC-only PSNR=$dc_psnr dB"
  echo "auto ES=$auto_es_size bytes; best fixed ES=$min_fixed bytes; ESC3 M2TS=$esc3_size bytes"
  echo "P-pan types=$pan_types; scene-cut types=$cut_types; generic 120-frame keyint verified"
  echo "Fade compensation: P-only ES=$fade_ic_size bytes vs disabled=$fade_noic_size bytes; P/B reconstruction exact"
  echo "P residual quality=$p_psnr dB; encoder/decoder reconstructed references match exactly"
  echo "Periodic-I quality: $(printf '%s' "$keyquality" | tr '\n' ';')"
  echo "Adaptive quality: high-motion/cut/dark-detail/credits plus flat-background/detailed-foreground stress MSE improved; --no-aq compatibility path verified"
  echo "SIMD: none/v3/v4 decoder reconstruction is exact and visually compatible; auto uses per-primitive runtime selection and FMA3 is checked when available"
  echo "Rate control: ABR+VBV + serialized VBV/recovery + CQ syntax verified"
  echo "B pictures: default I,B,B,P cadence, mixed F/B/interpolated/direct MB modes, PTS/DTS reordering, and reconstruction verified"
  echo "GOP threading: 1-thread/4-thread M2TS, VC-1, WMV/ASF, and reconstruction outputs are byte-identical"
  echo "Variable transforms: 8x8/8x4/4x8/4x4 selection, CQ4 B-direct chroma, overlay determinism, and fixed-8x8 fallback verified"
  echo "Loop filter: entry-point signaling, enabled/disabled paths, padded-edge reconstruction, and decoder equivalence verified"
  echo "libvc1/vc1enc interoperability/WMV9/AQ/SIMD/auto-table/I-P-B/fade-comp/variable-transform/deblocking/trellis/reconstruction/rate-control/threading smoke test: PASS"
fi
smoke_finish
exit $?
