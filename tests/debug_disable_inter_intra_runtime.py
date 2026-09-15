#!/usr/bin/env python3
import csv
import pathlib
import subprocess
import sys
import tempfile

enc=pathlib.Path(sys.argv[1])
ffmpeg=sys.argv[2]
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
                    if kind=='p':
                        if frame==0:
                            v=30+(((xx+yy)&1)*190)
                        else:
                            v=50+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                    else:
                        stable=40+(((xx//4)+(yy//4))&1)*160
                        if frame==1 and xx>=W//2:
                            v=48+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                        else:
                            v=stable
                    y[yy*W+xx]=max(16,min(235,v))
            f.write(y)
            f.write(bytes([128])*((W//2)*(H//2)))
            f.write(bytes([128])*((W//2)*(H//2)))


def run(td,kind,tag,flags):
    src=td/f'{kind}.y4m'
    if not src.exists(): write_fixture(src,kind)
    out=td/f'{kind}-{tag}.wmv'
    stats=td/f'{kind}-{tag}.csv'
    rng=4 if kind=='p' else 0
    bframes=0 if kind=='p' else 1
    cmd=[str(enc),'-i',str(src),'-o',str(out),'--debug-stats',str(stats),
         '--cq','8','--keyint','24','--bframes',str(bframes),'--threads','1','--simd','scalar','--no-scene-cut',
         '--search-range',str(rng),'--local-search-range',str(rng),'--fixed-gop-grid','--no-aq',
         '--no-loop-filter','--fixed-8x8','--trellis','0'] + list(flags)
    subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.run([ffmpeg,'-hide_banner','-v','error','-xerror','-err_detect','explode',
                    '-i',str(out),'-f','null','-'],check=True)
    with stats.open(newline='') as f: rows=list(csv.DictReader(f))
    wanted='P' if kind=='p' else 'B'
    row=next((r for r in rows if r['type']==wanted),None)
    if row is None: raise SystemExit(f'{kind}-{tag}: no {wanted} picture')
    return out,int(row['p_intra_mb'])


with tempfile.TemporaryDirectory(prefix='libvc1-disable-inter-intra-') as t:
    td=pathlib.Path(t)

    # P control: B-only suppression must not affect P-intra; P and PB must suppress it.
    p_normal,pn=run(td,'p','normal',[])
    p_bonly,pb=run(td,'p','b-only',['--debug-disable-b-intra'])
    p_ponly,pp=run(td,'p','p-only',['--debug-disable-p-intra'])
    p_both,pboth=run(td,'p','both',['--debug-disable-pb-intra'])
    if pn<=0: raise SystemExit('P baseline fixture failed to select P-intra')
    if pb<=0: raise SystemExit('--debug-disable-b-intra unexpectedly suppressed P-intra')
    if pp!=0: raise SystemExit(f'--debug-disable-p-intra left {pp} P-intra macroblocks')
    if pboth!=0: raise SystemExit(f'--debug-disable-pb-intra left {pboth} P-intra macroblocks')
    if p_normal.read_bytes()==p_ponly.read_bytes():
        raise SystemExit('P-only diagnostic switch did not change the P fixture bitstream')

    # B control deliberately uses zero motion search, matching the reported WMV3 case.
    # P-only suppression must leave B-intra available; B and PB must suppress it.
    b_normal,bn=run(td,'b','normal',[])
    b_ponly,bp=run(td,'b','p-only',['--debug-disable-p-intra'])
    b_bonly,bb=run(td,'b','b-only',['--debug-disable-b-intra'])
    b_both,bboth=run(td,'b','both',['--debug-disable-pb-intra'])
    if bn<=0: raise SystemExit('B baseline fixture failed to select B-intra at search-range 0')
    if bp<=0: raise SystemExit('--debug-disable-p-intra unexpectedly suppressed B-intra')
    if bb!=0: raise SystemExit(f'--debug-disable-b-intra left {bb} B-intra macroblocks')
    if bboth!=0: raise SystemExit(f'--debug-disable-pb-intra left {bboth} B-intra macroblocks')
    if b_normal.read_bytes()==b_bonly.read_bytes():
        raise SystemExit('B-only diagnostic switch did not change the B fixture bitstream')

print('debug P/B intra isolation runtime: PASS')
