#!/usr/bin/env python3
import subprocess, sys, tempfile
from pathlib import Path

if len(sys.argv) != 3:
    raise SystemExit('usage: visible_edge_runtime.py ENCODER FFMPEG')
enc, ffmpeg = sys.argv[1:]


def make_y4m(path: Path, w=50, h=34, frames=7):
    with path.open('wb') as f:
        f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for n in range(frames):
            f.write(b'FRAME\n')
            y = bytearray(w*h)
            for yy in range(h):
                for xx in range(w):
                    sx=(xx-2*n)%w; sy=(yy-n)%h
                    v=31+3*sx+5*sy+(80 if ((sx//5)^(sy//3))&1 else 0)
                    # Exercise the 2-pixel visible slivers at the right/bottom
                    # coded-macroblock edges rather than leaving them flat.
                    if xx>=48 or yy>=32:
                        v=17*xx+29*yy+13*n+(120 if ((xx+yy+n)&1) else 0)
                    y[yy*w+xx]=v&255
            cw,ch=w//2,h//2
            u=bytes(((97+5*x+3*y0+2*n)&255) for y0 in range(ch) for x in range(cw))
            v=bytes(((151+7*x-2*y0-3*n)&255) for y0 in range(ch) for x in range(cw))
            f.write(y); f.write(u); f.write(v)


def run(cmd):
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

with tempfile.TemporaryDirectory(prefix='libvc1-visible-edge-') as td:
    d=Path(td); src=d/'in.y4m'; make_y4m(src)
    for ext in ('m2ts','wmv'):
        out=d/f'out.{ext}'; recon=d/f'{ext}-recon.yuv'; dec=d/f'{ext}-ffmpeg.yuv'
        run([enc,'-i',str(src),'-o',str(out),'--recon-out',str(recon),
             '--cq','6','--threads','1','--simd','none','--bframes','2','--keyint','7',
             '--no-scene-cut','--no-loop-filter'])
        run([ffmpeg,'-v','error','-i',str(out),'-pix_fmt','yuv420p','-f','rawvideo','-y',str(dec)])
        if recon.read_bytes()!=dec.read_bytes():
            raise AssertionError(f'{ext}: decoder reconstruction differs on non-16-aligned edge fixture')


# A half-macroblock bottom crop (120 = 7*16 + 8) used to lose nearly 3 dB
# when crop-safe motion was allowed to bias picture-wide motion-mode RDO.
# The final emitted edge vectors must stay crop-safe, but global mode selection
# must still represent visible-content efficiency.  Keep the historical smoke
# quality floor registered in CTest so this interaction is caught early.
with tempfile.TemporaryDirectory(prefix='libvc1-visible-edge-quality-') as td:
    d=Path(td); src=d/'pquality.y4m'; out=d/'pquality.m2ts'
    subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-f','lavfi','-i',
                    'testsrc2=size=160x120:rate=24','-frames:v','12','-pix_fmt','yuv420p',
                    '-f','yuv4mpegpipe','-y',str(src)],check=True)
    run([enc,'-i',str(src),'-o',str(out),'--bframes','0','--no-scene-cut',
         '--keyint','999','--cq','9'])
    ps=subprocess.run([ffmpeg,'-hide_banner','-loglevel','info','-i',str(out),'-i',str(src),
                       '-lavfi','[0:v][1:v]psnr','-frames:v','12','-f','null','-'],
                      check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True).stderr
    import re
    vals=re.findall(r'average:([0-9.]+)',ps)
    if not vals or float(vals[-1]) <= 35.0:
        raise AssertionError(f'half-macroblock bottom-edge P quality regression: PSNR={vals[-1] if vals else "missing"}')

print('visible edge P/B residual reconstruction checks ok')
