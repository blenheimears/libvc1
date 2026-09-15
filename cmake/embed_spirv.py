#!/usr/bin/env python3
import pathlib, struct, sys

if len(sys.argv) != 3:
    raise SystemExit("usage: embed_spirv.py input.spv output.h")
src = pathlib.Path(sys.argv[1]).read_bytes()
if len(src) % 4:
    raise SystemExit("SPIR-V bytecode length is not a multiple of four")
words = struct.unpack("<%dI" % (len(src)//4), src)
out = pathlib.Path(sys.argv[2])
out.parent.mkdir(parents=True, exist_ok=True)
with out.open("w", newline="\n") as f:
    f.write("#pragma once\n#include <cstddef>\n#include <cstdint>\n\n")
    f.write("namespace libvc1 {\n")
    f.write("alignas(4) static constexpr uint32_t kVulkanMotionCostSpv[] = {\n")
    for i in range(0, len(words), 8):
        f.write("    " + ", ".join(f"0x{w:08x}u" for w in words[i:i+8]) + ",\n")
    f.write("};\n")
    f.write("static constexpr size_t kVulkanMotionCostSpvSize = sizeof(kVulkanMotionCostSpv);\n")
    f.write("} // namespace libvc1\n")
