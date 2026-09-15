#!/usr/bin/env python3
import pathlib, subprocess, sys, tempfile
from mbstats_v2 import read_macroblock_stats

if len(sys.argv)!=2: raise SystemExit('usage: input_fps_override.py VC1ENC')
enc=sys.argv[1]
with tempfile.TemporaryDirectory(prefix='libvc1-fps-override-') as d:
    td=pathlib.Path(d); src=td/'bad-rate.y4m'; out=td/'out.m2ts'; stats=td/'mb.csv'
    w=h=32; uv=bytes([128])*(w*h//4)
    b=bytearray(b'YUV4MPEG2 W32 H32 F30:1 Ip A1:1 C420jpeg\n')
    for n in range(3):
        y=bytes((48+(x*3+y*5+n*7))&255 for y in range(h) for x in range(w))
        b+=b'FRAME\n'+y+uv+uv
    src.write_bytes(b)
    cmd=[enc,'-i',str(src),'-o',str(out),'--cq','7','--simd','none','--threads','1',
         '--input-fps','24000/1001','--macroblock-stats',str(stats),'--max-frames','3']
    p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if p.returncode: raise SystemExit('override encode failed:\n'+p.stderr)
    cfg,_,frames,_=read_macroblock_stats(stats)
    if cfg.get('y4m_fps')!='30/1' or cfg.get('effective_fps')!='24000/1001' or cfg.get('input_fps_overridden')!='1':
        raise SystemExit(f'wrong override metadata: {cfg.get("y4m_fps")}, {cfg.get("effective_fps")}, {cfg.get("input_fps_overridden")}')
    if len(frames)!=3: raise SystemExit('wrong output frame count')
    # Alias and decimal common-rate parsing should resolve to the exact broadcast rational.
    stats2=td/'mb2.csv'; out2=td/'out2.m2ts'
    p=subprocess.run([enc,'-i',str(src),'-o',str(out2),'--cq','7','--simd','none','--threads','1','--fps','23.976','--macroblock-stats',str(stats2),'--max-frames','1'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if p.returncode: raise SystemExit('decimal alias encode failed:\n'+p.stderr)
    cfg2,_,_,_=read_macroblock_stats(stats2)
    if cfg2.get('effective_fps')!='24000/1001': raise SystemExit('23.976 alias did not resolve to 24000/1001')
print('Y4M input frame-rate override: PASS')
