#!/usr/bin/env python3
import csv, pathlib, subprocess, sys, tempfile

enc=sys.argv[1]
W=H=64
FPS=30
N=50

def write_src(path):
    with open(path,'wb') as f:
        f.write(f'YUV4MPEG2 W{W} H{H} F{FPS}:1 Ip A1:1 C420jpeg\n'.encode())
        uv=bytes([128])*(W//2)*(H//2)
        for i in range(N):
            scene=(i//5)&1
            y=bytearray(W*H)
            for yy in range(H):
                for xx in range(W):
                    base=(xx*7+yy*11)&63
                    y[yy*W+xx]=(30+base) if scene==0 else (225-base)
            f.write(b'FRAME\n'); f.write(y); f.write(uv); f.write(uv)

def run(root, name, extra, keyint='999'):
    out=root/(name+'.m2ts'); stats=root/(name+'.csv')
    p=subprocess.run([enc,'-i',str(root/'src.y4m'),'-o',str(out),'--keyint',str(keyint),'--threads','1','--simd','none','--debug-stats',str(stats),*extra],
                     stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,check=True)
    rows=list(csv.DictReader(open(stats,newline='')))
    scenes=[int(r['display_order']) for r in rows if r['keyframe_reason']=='scene']
    return scenes,p.stderr

with tempfile.TemporaryDirectory(prefix='libvc1-scene-interval-') as td:
    root=pathlib.Path(td); write_src(root/'src.y4m')
    scenes,log=run(root,'default',[])
    if scenes != [5,15,25,35,45]:
        raise SystemExit(f'default 0.25-second cooldown wrong: {scenes}')
    if 'scene-cut=on' not in log or 'scene-cut-interval=0.25s' not in log:
        raise SystemExit('startup summary does not report default enabled scene cut / 0.25-second interval')
    if 'distant-max-mae=255' not in log or 'residual-priority=8+24/0' not in log:
        raise SystemExit('scene-cut mitigation experiments are not neutral by default')

    scenes,_=run(root,'short-gop',[],keyint='12')
    if scenes != [5,15,25,35,45]:
        raise SystemExit(f'scene-cut cooldown incorrectly reset by scheduled GOPs: {scenes}')

    scenes,_=run(root,'half',['--scene-cut-interval','0.5'])
    if scenes != [5,20,35]:
        raise SystemExit(f'0.5-second cooldown wrong: {scenes}')

    scenes,_=run(root,'unlimited',['--scene-cut-interval','0'])
    expected=list(range(5,50,5))
    if scenes != expected:
        raise SystemExit(f'zero cooldown did not allow every cut: {scenes}, expected {expected}')

print('scene-cut default/cooldown runtime regression: PASS')
