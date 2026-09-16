# libvc1

**libvc1** is an experimental, from-scratch VC-1 encoder library with a stable C API, a small C++ wrapper, and the `vc1enc` command-line frontend.

It encodes **VC-1 Advanced Profile** and **WMV9 / WMV3 Main Profile**, with direct output to M2TS, ASF/WMV, WVC1/ASF, and Matroska. The encoder includes I/P/B/BI pictures, hybrid motion estimation, variable transforms, adaptive quantization, DQUANT, trellis quantization, rate control, multithreading, runtime SIMD dispatch, and an optional Vulkan motion-compute backend.

> [!WARNING]
> libvc1 is still experimental. The 0.2.x series represents broad codec feature coverage while optimization, quality tuning, and hardware-player interoperability work continue. Validate important output with the decoders and hardware players you intend to use.

> [!CAUTION]
> **Interlaced VC-1 coding is experimental.** Advanced Profile supports fixed-order TFF/BFF input and field-picture P/B coding, but the interlaced path is still under compatibility and quality tuning. Progressive encoding is the more mature path. WMV3/Main interlaced coding is not currently supported.


### Raw VC-1 primary output

`vc1enc` can write the Advanced Profile elementary stream directly as its primary output. Use a `.vc1` output name (auto-detected) or select it explicitly with `--format raw` (aliases: `vc1`, `elementary`):

```sh
./build/vc1enc -i input.y4m -o output.vc1 --format raw --bluray-compat
```

This path writes no M2TS/ASF/Matroska container. `--es-out` remains available for workflows that intentionally want a second elementary-stream copy alongside a container.

## Features

- VC-1 **Advanced Profile** encoding
- WMV9 / **WMV3 Main Profile** encoding
- Progressive **I, P, B, and BI pictures**
- Experimental Advanced-Profile **interlaced TFF/BFF coding**
- Quarter-pixel and half-pixel motion compensation
- Progressive Advanced-Profile **Mixed-MV / 4-MV** P macroblocks
- P/B-picture intra macroblocks with AC prediction
- Hybrid **UMH + long-range/content-assisted motion search**
- Extended motion-vector ranges
- 8x8, 8x4, 4x8, and 4x4 inter transforms
- Uniform/nonuniform quantization, HALFQP, and **DQUANT**
- Trellis quantization
- Content-adaptive perceptual AQ
- Intensity compensation / weighted prediction
- OVERLAP / CONDOVER smoothing
- VC-1 in-loop deblocking on supported paths
- Adaptive entropy-table and bitplane decisions
- Skipped-picture coding for exact duplicate input frames
- One-pass **ABR + VBV/HRD** rate control
- Constant-quantizer mode
- Scene-cut I-picture insertion with a configurable cooldown
- GOP-level multithreading
- Per-primitive runtime x86 SIMD benchmarking and dispatch
- Experimental, opt-in **Vulkan** motion-cost acceleration
- Per-frame, per-macroblock, and aggregate performance diagnostics
- CMake, pkg-config, and Nix integration

## Output formats

`vc1enc` chooses the normal output mode from the filename, or it can be overridden explicitly.

| Output | Codec | Notes |
| --- | --- | --- |
| `.m2ts` | VC-1 Advanced Profile | Blu-ray-style MPEG-2 TS carriage |
| `.wmv` | WMV9 / WMV3 Main Profile | Native ASF/WMV |
| `--format wvc1` | VC-1 Advanced Profile | Microsoft `WVC1` in ASF |
| `.mkv` | VC-1 Advanced Profile by default | `WVC1` through libmatroska/libebml |
| `.mkv --codec wmv3` | WMV3 Main Profile | `WMV3` in Matroska |
| `--es-out FILE.vc1` | VC-1 Advanced Profile | Also write the raw elementary stream |

libvc1 itself is the codec library: container parsing and muxing live in the `vc1enc` frontend.

## Build

### Requirements

A normal build requires:

- CMake 3.16 or newer
- a C/C++ compiler
- POSIX threads or the platform equivalent

Optional components:

- **libmatroska + libebml** for MKV output
- **Vulkan development headers/loader + `glslangValidator` + Python 3** for the optional Vulkan backend
- **FFmpeg/ffprobe, Bash, Python 3, and NumPy** for the full regression/smoke suite
- **QEMU user-mode x86-64** for optional cross-ISA SIMD testing

### CMake

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The default build produces the shared library, static library, and `vc1enc`.

Useful switches include:

```text
-DLIBVC1_BUILD_SHARED=ON|OFF
-DLIBVC1_BUILD_STATIC=ON|OFF
-DLIBVC1_BUILD_FRONTEND=ON|OFF
-DLIBVC1_FRONTEND_LINK_STATIC=ON|OFF
-DLIBVC1_BUILD_TESTS=ON|OFF
-DLIBVC1_ENABLE_X86_SIMD=ON|OFF
-DLIBVC1_ENABLE_MATROSKA=ON|OFF
-DLIBVC1_ENABLE_VULKAN=ON|OFF
-DLIBVC1_ENABLE_SANITIZERS=ON|OFF
-DLIBVC1_SIMD_TEST_MODE=native|qemu-fallback|qemu-force
```

Install with:

```sh
cmake --install build --prefix /desired/prefix
```

### Nix / NixOS

A `flake.nix` is included.

```sh
# Build and test.
nix build

# Run vc1enc from the flake.
nix run -- --help

# Development environment.
nix develop
```

On x86-64, the flake also exposes QEMU-assisted ISA-test variants when those are needed.

## Quick start

Encode a YUV4MPEG2 file to Advanced Profile M2TS:

```sh
./build/vc1enc -i input.y4m -o output.m2ts
```

Pipe video from FFmpeg:

```sh
ffmpeg -i input.mp4 -pix_fmt yuv420p -f yuv4mpegpipe - \
  | ./build/vc1enc -o output.m2ts
```

Encode WMV9 / WMV3 Main Profile:

```sh
./build/vc1enc -i input.y4m -o output.wmv
```

Encode Advanced Profile to Matroska:

```sh
./build/vc1enc -i input.y4m -o output.mkv
```

Write an Advanced Profile elementary stream as well:

```sh
./build/vc1enc -i input.y4m -o output.m2ts --es-out output.vc1
```

See all current options with:

```sh
./build/vc1enc --help
```

## Blu-ray-oriented encoding

`--bluray-compat` enables libvc1's codec-level Blu-ray compatibility checks, including the supported Advanced Profile/level, frame-rate, bitrate/VBV, geometry, and GOP restrictions.

```sh
./build/vc1enc -i input.y4m -o output.m2ts --bluray-compat
```

It does **not** author a Blu-ray disc. libvc1 does not generate BDMV structures such as `CLPI` or `MPLS`, and codec-level conformance does not guarantee compatibility with every hardware player.

In Blu-ray compatibility mode, every I-picture random-access access unit repeats the Advanced Profile sequence header immediately before its entry-point header (`SEQUENCE -> ENTRYPOINT -> I`). This makes each GOP start self-initializing for Blu-ray authoring/indexing tools that construct CLPI/CPI seek maps from codec random-access units. Generic Advanced Profile encoding retains the normal single initial sequence header.

For authoring workflows, the raw Advanced Profile stream can also be saved with `--es-out` and remuxed separately.

## Rate control

The normal bounded-rate path uses libvc1's one-pass ABR + VBV/HRD controller. The default macroblock-class allocation weights are **I=5.0, P=1.0, B=0.70**; intra macroblocks inside P/B pictures use the I weight.

```sh
# Target bitrate.
./build/vc1enc -i input.y4m -o output.m2ts --bitrate 20M

# Explicit target and buffer.
./build/vc1enc -i input.y4m -o output.m2ts \
  --bitrate 20M --buffer-size 20M

# Constant quantizer.
./build/vc1enc -i input.y4m -o output.m2ts --cq 9

# Legal low-Q half steps are accepted.
./build/vc1enc -i input.y4m -o output.m2ts --cq 4.5
```

`--quantizer-type auto|uniform|nonuniform` controls picture quantizer selection. DQUANT and perceptual AQ can further redistribute quality within a picture.

## GOPs and scene changes

Generic encoding uses a configurable periodic I-picture cadence:

```sh
./build/vc1enc -i input.y4m -o output.m2ts --keyint 48
```

Scene-cut detection is enabled by default. To avoid rapid hard cuts producing a burst of expensive I pictures, scene-cut I pictures have an independent minimum spacing of **0.25 seconds** by default.

```sh
# Change the scene-cut cooldown.
./build/vc1enc -i input.y4m -o output.m2ts \
  --scene-cut-interval 0.5

# Allow every detected cut to become an I picture.
./build/vc1enc -i input.y4m -o output.m2ts \
  --scene-cut-interval 0

# Disable threshold scene-cut detection.
./build/vc1enc -i input.y4m -o output.m2ts --no-scene-cut
```

Scheduled keyframes do not reset the independent scene-cut cooldown.

## Motion estimation

Advanced Profile defaults to a wide legal search domain while using a much smaller local search for ordinary motion.

```sh
# Overall maximum search radius.
./build/vc1enc -i input.y4m -o output.m2ts --search-range 1024

# Local UMH radius.
./build/vc1enc -i input.y4m -o output.m2ts --local-search-range 32

# Motion-estimation depth.
./build/vc1enc -i input.y4m -o output.m2ts --me-quality satd
```

Available ME depths are:

```text
sad
rate
satd
rd
```
The default in normal mode is satd.
rd is very slow and probably not worth using.

Long-range search can compare local UMH results with propagated/content-assisted distant candidates instead of exhaustively scanning the full extended-MV space.

```sh
./build/vc1enc -i input.y4m -o output.m2ts \
  --long-range-search compare
```

WMV3/Main uses a conservative maximum symmetric search radius of 255 pixels.

## Speed presets

The presets are option bundles; later command-line options override individual preset choices.

```sh
./build/vc1enc -i input.y4m -o output.m2ts --fast
./build/vc1enc -i input.y4m -o output.m2ts --faster
./build/vc1enc -i input.y4m -o output.m2ts --fastest
```

`--fast` is the moderate throughput/quality tradeoff. `--faster` reduces analysis further, while `--fastest` is intended for throughput-sensitive testing and encoding.

## Multithreading

GOP-level parallelism is enabled by default.

```sh
# Use eight GOP workers.
./build/vc1enc -i input.y4m -o output.m2ts --threads 8

# Single-worker mode.
./build/vc1enc -i input.y4m -o output.m2ts --threads 1
```

Memory use rises with thread count, GOP length, and frame size because workers retain independent source/reference state.

Nested intra-GOP helper parallelism remains available but is off by default:

```sh
./build/vc1enc -i input.y4m -o output.m2ts \
  --intra-gop-parallelism
```

## SIMD acceleration

On x86-64, libvc1 can build multiple ISA-specific implementations and benchmark/select them per primitive at startup.

The portable scalar implementation remains a first-class candidate, so a wider SIMD target is not chosen merely because the CPU supports it.

Common controls include:

```sh
# Automatic per-primitive selection.
./build/vc1enc -i input.y4m -o output.m2ts --simd auto

# Force the scalar/reference path.
./build/vc1enc -i input.y4m -o output.m2ts --simd none

# Force an AVX2-class target when supported.
./build/vc1enc -i input.y4m -o output.m2ts --simd x86-64-v3

# Force an AVX-512-class target when supported.
./build/vc1enc -i input.y4m -o output.m2ts --simd x86-64-v4
```

Intermediate x86 feature bands are also available on compatible processors. `--benchmark-all` benchmarks every built target compatible with the current CPU.

## Experimental Vulkan acceleration

Vulkan is **never enabled by default**. Normal encoding uses the CPU/SIMD path.

```sh
./build/vc1enc -i input.y4m -o output.m2ts --compute vulkan
```

This explicitly opts into the experimental Advanced-Profile Vulkan backend. At startup, libvc1 benchmarks eligible GPU motion-cost workloads against the already-selected CPU/SIMD implementation. Vulkan is used only for workload sizes where it wins; otherwise encoding continues on CPU.

To bypass the benchmark decision:

```sh
./build/vc1enc -i input.y4m -o output.m2ts \
  --compute vulkan --vulkan-force
```

Forced mode makes Vulkan initialization/profile failures fatal and is useful for hardware validation.

Additional controls:

```text
--compute-device N
--vulkan-min-batch N
```

The first Vulkan implementation accelerates selected batched motion-estimation SAD/SATD work. Rate control, transforms, entropy coding, bitstream generation, muxing, and WMV3/Main remain CPU-side. No OpenCL backend is currently included.

## Experimental interlaced coding

Fixed-order interlaced Y4M (`It` for top-field-first and `Ib` for bottom-field-first) is accepted for **VC-1 Advanced Profile**.

The implementation includes frame-interlaced I/BI anchors and field-picture predictive coding with field-aware references, motion, residuals, and quantization. It is nevertheless intentionally considered **experimental** while compatibility and quality work continue.

Important limitations:

- use Advanced Profile; WMV3/Main interlaced coding is not implemented;
- interlaced behavior should be validated against the intended decoder/player;
- some coding tools are not yet at parity with the progressive path;
- interlaced in-loop filtering is currently disabled while field-domain reference filtering is being validated.

For production-critical material, progressive encoding is currently the safer path.

## Diagnostics

libvc1 includes several opt-in diagnostics useful for quality tuning and regression work.

```sh
# Per-frame encoder decisions and quality/rate statistics.
./build/vc1enc -i input.y4m -o output.m2ts \
  --debug-stats frames.csv

# Per-macroblock analysis.
./build/vc1enc -i input.y4m -o output.m2ts \
  --macroblock-stats macroblocks.csv

# Compact aggregate performance profile.
./build/vc1enc -i input.y4m -o output.m2ts \
  --speed-profile speed.txt

# Transform visualization.
./build/vc1enc -i input.y4m -o output.m2ts \
  --debug-transforms transforms.y4m

# Encoder-side reconstructed sequence.
./build/vc1enc -i input.y4m -o output.m2ts \
  --recon-out reconstructed.yuv
```

The debug/statistics paths are designed not to change normal coding decisions unless an explicitly diagnostic coding switch is selected.

## Using the library

The public C API is declared in `<libvc1.h>`. A small RAII C++ wrapper is available in `<libvc1.hpp>`.

Typical C lifecycle:

```c
#include <libvc1.h>

vc1_param_t p;
vc1_param_default(&p);

p.i_width   = width;
p.i_height  = height;
p.i_fps_num = 24000;
p.i_fps_den = 1001;

vc1_t *enc = vc1_encoder_open(&p);
if (!enc) {
    /* Inspect vc1_encoder_last_error(NULL). */
    return 1;
}

vc1_picture_t in, out;
vc1_picture_init(&in);
vc1_picture_init(&out);

in.img.i_plane = 3;
in.img.plane[0] = y;
in.img.plane[1] = u;
in.img.plane[2] = v;
in.img.i_stride[0] = y_stride;
in.img.i_stride[1] = u_stride;
in.img.i_stride[2] = v_stride;
in.i_pts = pts;

vc1_au_t *au = NULL;
int nau = 0;

vc1_encoder_encode(enc, &au, &nau, &out, &in);

/* Consume returned access units before the next encode call. */

while (vc1_encoder_delayed_frames(enc) > 0)
    vc1_encoder_encode(enc, &au, &nau, &out, NULL);

vc1_encoder_close(enc);
```

Installed CMake targets:

```cmake
find_package(libvc1 CONFIG REQUIRED)

target_link_libraries(myapp PRIVATE libvc1::vc1)
# or:
target_link_libraries(myapp PRIVATE libvc1::vc1_static)
```

Pkg-config:

```sh
pkg-config --cflags --libs libvc1
pkg-config --cflags --libs libvc1-static
```

## Testing

Run the registered CTest suite:

```sh
ctest --test-dir build --output-on-failure
```

Run the historical end-to-end smoke suite:

```sh
bash ./tests/smoke.sh ./build/vc1enc
```

Useful decoder checks for generated files:

```sh
ffprobe -hide_banner output.m2ts
ffmpeg -v error -err_detect explode -i output.m2ts -f null -

ffprobe -hide_banner output.wmv
ffmpeg -v error -err_detect explode -i output.wmv -f null -
```

A Vulkan-enabled build additionally registers the GPU/CPU parity regression:

```sh
ctest --test-dir build \
  -R libvc1-vulkan-compute-runtime \
  --output-on-failure
```

Decoder acceptance alone does not establish Blu-ray disc or hardware-player compliance.

## Source layout

The codec implementation is split by subsystem rather than concentrated in one encoder file. Major components include:

```text
include/                     Public C/C++ API
src/encoder_core.cpp         API, GOP scheduling, ordered RC/VBV coordination
src/encoder_motion.cpp       Motion analysis and search
src/encoder_motion_compensation.cpp
src/encoder_transform.cpp
src/encoder_aq.cpp           AQ and DQUANT
src/encoder_rate_control.cpp
src/encoder_entropy.cpp
src/encoder_filter.cpp
src/encoder_encode.cpp       Picture/macroblock coding
src/encoder_simd.cpp         SIMD dispatch and benchmarking
src/encoder_compute.cpp      Compute-backend orchestration
src/vulkan_compute.cpp       Experimental Vulkan implementation
tests/                       Unit, runtime, compatibility, and smoke tests
```

## Scope and compatibility

libvc1 implements the codec and several useful container bindings, but it is not a complete media-authoring suite.

In particular:

- Blu-ray compatibility mode is codec-level, not disc authoring.
- Hardware-player behavior can be stricter than software-decoder conformance.
- Experimental interlaced and Vulkan paths require additional target-system validation.
- Some specialized VC-1 tools remain outside the current implementation or are intentionally limited to specific profiles/modes.

The project follows VC-1 syntax defined by **SMPTE ST 421**.

## Known issues
- The encoder uses a lot of memory and is not very fast.
- The fast / faster / fastest modes are poor quality.
- Interlaced coding is poor quality.
- Some scenes trip the encoder up / produce poor quality.
- 2-pass encoding is not supported. It might be added in the future if it's determined to be beneficial.
- Vulkan encoding is probably slower than CPU encoding, even on a fast GPU.
- Sometimes it might not fully utilize the available CPU.
- The SIMD acceleration targeted at specific CPUs is not always faster on that CPU.
- The MKV muxer produces files that don't play in Windows Media Player. Use .wmv for now or remux with mkvtoolnix.
- Large motion vectors currently produce glitchy video in WMV9 mode. Capped at 255 by default to prevent this issue.
- It has only been tested under NixOS.
- There is no SIMD acceleration on ARM.
- The Bulldozer / Piledriver-targeted SIMD acceleration has not been tested on a real CPU.
- It hasn't been tested on many hardware decoders.

## License

libvc1 is released under the **GNU Lesser General Public License v2.1 or later (LGPL-2.1-or-later)**. See [`LICENSE`](LICENSE).
