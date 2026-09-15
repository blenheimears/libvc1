#!/usr/bin/env python3
"""P/B temporal macroblocks with large prediction residuals get finer local Q.

The fixtures intentionally run near the bitrate ceiling, disable P/B intra and
motion search, and leave AQ off. Progressive and field-interlaced P/B paths are
covered. Any local MQUANT separation therefore comes from residual-priority
ABR/DQUANT rather than perceptual AQ or intra weighting.
"""
import pathlib
import subprocess
import sys
import tempfile
from mbstats_v2 import read_macroblock_stats, frame_value

if len(sys.argv) != 2:
    raise SystemExit('usage: residual_priority_runtime.py VC1ENC')
enc=sys.argv[1]
W=H=128


def write_y4m(path, n, b_fixture=False, interlaced=False):
    uv=bytes([128])*(W//2)*(H//2)
    scan='It' if interlaced else 'Ip'
    out=bytearray(f'YUV4MPEG2 W{W} H{H} F24:1 {scan} A1:1 C420jpeg\n'.encode())
    for frame in range(n):
        y=bytearray(W*H)
        for yy in range(H):
            for xx in range(W):
                base=32+((xx*7+yy*11)&127)
                if b_fixture:
                    hard=(frame%2==1 and xx>=W//2)
                else:
                    hard=(frame>0 and frame%3==1 and xx>=W//2)
                v=220-((xx*13+yy*17+frame*19)&127) if hard else base
                y[yy*W+xx]=max(16,min(235,v))
        out += b'FRAME\n'+y+uv+uv
    pathlib.Path(path).write_bytes(out)


def run(td, kind, interlaced=False):
    tag=f'{kind}-i' if interlaced else kind
    src=td/f'{tag}.y4m'; mb=td/f'{tag}-mb.csv'; out=td/f'{tag}.m2ts'
    write_y4m(src,13 if kind=='b' else 12,b_fixture=(kind=='b'),interlaced=interlaced)
    cmd=[enc,'-i',str(src),'-o',str(out),'--bitrate','100k','--buffer-size','300k',
         '--keyint','12','--bframes','1' if kind=='b' else '0','--fixed-gop-grid',
         '--no-scene-cut','--search-range','0','--local-search-range','0','--me-quality','sad',
         '--simd','none','--threads','1','--no-aq','--residual-priority-strength','4','--debug-disable-pb-intra',
         '--macroblock-stats',str(mb)]
    p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if p.returncode:
        raise SystemExit(f'{tag} residual-priority encode failed ({p.returncode}):\n{p.stderr}')
    typ='B' if kind=='b' else 'P'
    config, header, frame_rows, mb_rows=read_macroblock_stats(mb)
    frames={}
    for r in mb_rows:
        if frame_value(r,'frame_type')==typ:
            frames.setdefault(frame_value(r,'display_order'),[]).append(r)
    candidates=[]
    for order,rs in frames.items():
        easy=[r for r in rs if int(r['mb_x'])<W//32]
        hard=[r for r in rs if int(r['mb_x'])>=W//32]
        if not easy or not hard: continue
        easy_sad=sum(float(r['prediction_sad_y']) for r in easy)/len(easy)
        hard_sad=sum(float(r['prediction_sad_y']) for r in hard)/len(hard)
        if hard_sad>max(1000.0,easy_sad*2.0):
            easy_q=sum(int(r['mquant']) for r in easy)/len(easy)
            hard_q=sum(int(r['mquant']) for r in hard)/len(hard)
            candidates.append((order,easy_sad,hard_sad,easy_q,hard_q))
    if not candidates:
        raise SystemExit(f'{tag} fixture produced no strongly asymmetric prediction-residual frame')
    if not any(hq+0.5 <= eq for _,_,_,eq,hq in candidates):
        raise SystemExit(f'{tag} high-residual temporal blocks did not receive finer local MQUANT: {candidates}')


with tempfile.TemporaryDirectory(prefix='libvc1-residual-priority-') as d:
    td=pathlib.Path(d)
    run(td,'p')
    run(td,'b')
    run(td,'p',interlaced=True)
    run(td,'b',interlaced=True)

print('progressive + field-interlaced P/B residual-priority ABR/DQUANT runtime regression: PASS')
