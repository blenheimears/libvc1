#!/usr/bin/env python3
import csv
import pathlib
import subprocess
import sys
import tempfile

if len(sys.argv)!=3:
    raise SystemExit('usage: inter_intra_rate_complexity_runtime.py VC1ENC FFMPEG')
enc=pathlib.Path(sys.argv[1]); ffmpeg=sys.argv[2]
W=H=64


def write_fixture(path,kind):
    frames=2 if kind=='p' else 3
    with path.open('wb') as f:
        f.write(f'YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for frame in range(frames):
            f.write(b'FRAME\n')
            y=bytearray(W*H)
            for yy in range(H):
                for xx in range(W):
                    stable=40+(((xx//4)+(yy//4))&1)*160
                    if kind=='p':
                        if frame==0:
                            v=30+(((xx+yy)&1)*190)
                        else:
                            v=50+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                    else:
                        if frame==1 and xx>=W//2:
                            v=48+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                        else:
                            v=stable
                    y[yy*W+xx]=max(16,min(235,v))
            f.write(y)
            f.write(bytes([128])*(W*H//4))
            f.write(bytes([128])*(W*H//4))


def run(td,kind,disable=False):
    src=td/f'{kind}.y4m'
    if not src.exists(): write_fixture(src,kind)
    out=td/f'{kind}-{"nointra" if disable else "intra"}.wmv'
    stats=td/f'{kind}-{"nointra" if disable else "intra"}.csv'
    bframes=0 if kind=='p' else 1
    search=4 if kind=='p' else 0
    cmd=[str(enc),'-i',str(src),'-o',str(out),'--debug-stats',str(stats),
         '--bitrate','300k','--buffer-size','1m','--keyint','24','--bframes',str(bframes),
         '--threads','1','--simd','scalar','--no-scene-cut','--search-range',str(search),'--local-search-range',str(search),
         '--fixed-gop-grid','--no-aq','--no-loop-filter','--fixed-8x8','--trellis','0']
    if disable: cmd.append('--debug-disable-pb-intra')
    p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if p.returncode:
        raise SystemExit(f'{kind} encode failed: {p.stderr}')
    rows=list(csv.DictReader(stats.open(newline='')))
    target='P' if kind=='p' else 'B'
    row=next((r for r in rows if r['type']==target),None)
    if row is None: raise SystemExit(f'{kind}: missing {target} row')
    subprocess.run([ffmpeg,'-hide_banner','-v','error','-xerror','-err_detect','explode','-i',str(out),
                    '-f','null','-'],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    return row


with tempfile.TemporaryDirectory(prefix='libvc1-inter-intra-rc-') as t:
    td=pathlib.Path(t)
    for kind in ('p','b'):
        row=run(td,kind,False)
        intra=int(row['p_intra_mb'])
        if intra<=0:
            raise SystemExit(f'{kind}: fixture failed to select inter-picture intra macroblocks')
        complexity=float(row['complexity']); motion=float(row['motion_residual'])
        if abs(complexity-motion)<0.05:
            raise SystemExit(f'{kind}: RC complexity still aliases the intra decision/motion metric: {complexity} vs {motion}')
        predicted=float(row['predicted_bits']); actual=float(row['first_actual_bits'])
        if actual<=0 or abs(predicted-actual)/actual>0.30:
            raise SystemExit(f'{kind}: reference-Q syntax calibration inaccurate: predicted={predicted}, actual={actual}')

        control=run(td,kind,True)
        if int(control['p_intra_mb'])!=0:
            raise SystemExit(f'{kind}: no-intra control still selected intra macroblocks')
        cc=float(control['complexity']); cm=float(control['motion_residual'])
        if abs(cc-cm)>1e-6:
            raise SystemExit(f'{kind}: no-intra control unexpectedly changed temporal complexity: {cc} vs {cm}')

print('P/B inter-picture intra rate-complexity runtime: PASS')
