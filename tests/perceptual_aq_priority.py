#!/usr/bin/env python3
import pathlib, subprocess, sys, tempfile
from mbstats_v2 import read_macroblock_stats

if len(sys.argv)!=2: raise SystemExit('usage: perceptual_aq_priority.py VC1ENC')
enc=sys.argv[1]
W,H=96,16
with tempfile.TemporaryDirectory(prefix='libvc1-perceptual-aq-') as d:
    td=pathlib.Path(d); src=td/'aq.y4m'; out=td/'aq.m2ts'; stats=td/'aq.csv'
    y=bytearray(W*H); u=bytearray(W*H//4); v=bytearray(W*H//4)
    # Six I macroblocks: flat dark, textured dark gray, textured bright green,
    # textured low-luminance blue, dark red, and brown.
    for yy in range(H):
        for xx in range(W):
            mb=xx//16
            if mb==0: val=24
            elif mb==1: val=24+(((xx+yy)&1)*28)
            elif mb==2: val=150+(((xx+yy)&1)*24)
            elif mb==3: val=42+(((xx+yy)&1)*28)
            elif mb==4: val=48+(((xx+yy)&1)*26)
            else: val=58+(((xx+yy)&1)*24)
            y[yy*W+xx]=val
    chroma=[(128,128),(128,128),(82,82),(200,110),(92,210),(94,180)]
    for yy in range(H//2):
        for xx in range(W//2):
            cb,cr=chroma[xx//8]; o=yy*(W//2)+xx; u[o]=cb; v[o]=cr
    src.write_bytes(b'YUV4MPEG2 W96 H16 F24:1 Ip A1:1 C420jpeg\nFRAME\n'+y+u+v)
    p=subprocess.run([enc,'-i',str(src),'-o',str(out),'--cq','12','--intra-only','--threads','1','--simd','none','--macroblock-stats',str(stats),'--max-frames','1'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if p.returncode: raise SystemExit('AQ encode failed:\n'+p.stderr)
    cfg,_,_,mb=read_macroblock_stats(stats)
    byx={int(r['mb_x']):r for r in mb}
    if set(byx)!={0,1,2,3,4,5}: raise SystemExit('fixture did not produce six I macroblocks')
    flat,dark,green,blue,red,brown=[byx[i] for i in range(6)]
    if float(flat['aq_dark_detail'])!=0.0 or float(flat['aq_requested_q_boost'])!=0.0:
        raise SystemExit('flat near-black macroblock incorrectly received explicit dark-detail AQ')
    if not (float(dark['aq_dark_detail'])>0.60 and int(dark['mquant'])+2<=int(flat['mquant'])):
        raise SystemExit('textured dark I macroblock did not receive strong luma-led AQ protection')
    if not (float(green['aq_color_luma_priority'])>0.70 and float(green['aq_color_chroma_priority'])>0.40):
        raise SystemExit('medium/high-luminance green priority was not detected')
    if not (float(blue['aq_color_luma_priority'])>0.25 and float(blue['aq_color_chroma_priority'])>0.25):
        raise SystemExit('low-luminance blue priority was not detected')
    for name,r in (('dark red',red),('brown',brown)):
        if not (float(r['aq_color_luma_priority'])>0.25 and float(r['aq_color_chroma_priority'])>0.25):
            raise SystemExit(name+' priority was not detected')
    for r in (green,blue,red,brown):
        if float(r['aq_color_luma_priority']) < float(r['aq_color_chroma_priority']):
            raise SystemExit('color AQ is not luma-led')
print('dark I-block + luma-led perceptual color AQ (green/blue/red/brown): PASS')
