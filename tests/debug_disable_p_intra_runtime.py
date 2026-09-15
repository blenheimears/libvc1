#!/usr/bin/env python3
import csv
import pathlib
import subprocess
import sys
import tempfile

enc=pathlib.Path(sys.argv[1])
ffmpeg=sys.argv[2]
W=H=64


def write_p_fixture(path):
    with path.open('wb') as f:
        f.write(f'YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for frame in range(2):
            f.write(b'FRAME\n')
            y=bytearray(W*H)
            for yy in range(H):
                for xx in range(W):
                    if frame==0:
                        v=30+(((xx+yy)&1)*190)
                    else:
                        v=50+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                    y[yy*W+xx]=max(16,min(235,v))
            f.write(y)
            f.write(bytes([128])*((W//2)*(H//2)))
            f.write(bytes([128])*((W//2)*(H//2)))


def run(td,tag,disable):
    src=td/'p-intra.y4m'
    if not src.exists(): write_p_fixture(src)
    out=td/f'{tag}.wmv'
    stats=td/f'{tag}.csv'
    cmd=[str(enc),'-i',str(src),'-o',str(out),'--debug-stats',str(stats),
         '--cq','8','--keyint','24','--bframes','0','--threads','1','--simd','scalar','--no-scene-cut',
         '--search-range','4','--local-search-range','4','--fixed-gop-grid','--no-aq',
         '--no-loop-filter','--fixed-8x8','--trellis','0']
    if disable: cmd.append('--debug-disable-p-intra')
    subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.run([ffmpeg,'-hide_banner','-v','error','-xerror','-err_detect','explode',
                    '-i',str(out),'-f','null','-'],check=True)
    with stats.open(newline='') as f: rows=list(csv.DictReader(f))
    p=next((r for r in rows if r['type']=='P'),None)
    if p is None: raise SystemExit(f'{tag}: no P picture in diagnostic fixture')
    return out,int(p['p_intra_mb'])


with tempfile.TemporaryDirectory(prefix='libvc1-disable-p-intra-') as t:
    td=pathlib.Path(t)
    normal,normal_intra=run(td,'normal',False)
    disabled,disabled_intra=run(td,'disabled',True)
    if normal_intra<=0:
        raise SystemExit('baseline fixture failed to select P-intra')
    if disabled_intra!=0:
        raise SystemExit(f'--debug-disable-p-intra left {disabled_intra} P-intra macroblocks')
    if normal.read_bytes()==disabled.read_bytes():
        raise SystemExit('diagnostic switch did not change the P-picture bitstream')

print('debug disable-P-intra runtime: PASS')
