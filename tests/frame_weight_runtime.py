#!/usr/bin/env python3
"""ABR macroblock-class weighting regression.

The public fields/legacy CLI aliases retain their I/P/B names, but since 0.2.17
those weights classify macroblocks: every intra MB uses I weight, including
P-intra/B-intra, while temporal P/B MBs use P/B weight.
"""
import csv
import pathlib
import subprocess
import sys
import tempfile
from mbstats_v2 import read_macroblock_stats, frame_value

if len(sys.argv) != 2:
    raise SystemExit('usage: frame_weight_runtime.py VC1ENC')
enc = sys.argv[1]
W=H=64


def write_y4m(path, frames):
    out=bytearray(f'YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n'.encode())
    uv=bytes([128])*(W//2)*(H//2)
    for y in frames:
        out += b'FRAME\n'+bytes(y)+uv+uv
    pathlib.Path(path).write_bytes(out)


def pure_frames(n=12):
    frames=[]
    for i in range(n):
        y=bytearray(W*H)
        for yy in range(H):
            for xx in range(W):
                y[yy*W+xx]=(32 + ((xx*3 + yy*5 + i*11) & 127) +
                             (32 if ((xx+i*2)//8)&1 else 0)) & 255
        frames.append(y)
    return frames


def mixed_frames(kind):
    if kind=='p':
        frames=[]
        for frame in range(2):
            y=bytearray(W*H)
            for yy in range(H):
                for xx in range(W):
                    old=30+(((xx+yy)&1)*190)
                    new=50+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                    v=old if frame==0 or xx<W//2 else new
                    y[yy*W+xx]=max(16,min(235,v))
            frames.append(y)
        return frames
    frames=[]
    for frame in range(3):
        y=bytearray(W*H)
        for yy in range(H):
            for xx in range(W):
                stable=40+(((xx//4)+(yy//4))&1)*160
                if frame==1 and xx>=W//2:
                    v=48+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                else:
                    v=stable
                y[yy*W+xx]=max(16,min(235,v))
        frames.append(y)
    return frames


def run(td, src, tag, *, weights=None, bframes=2, search=0,
        disable_pb=False, disable_p=False, disable_b=False, mbstats=False,
        bitrate='800k', buffer='2m'):
    out=td/f'{tag}.m2ts'; stats=td/f'{tag}.csv'; mb=td/f'{tag}-mb.csv'
    cmd=[enc,'-i',str(src),'-o',str(out),'--bitrate',bitrate,'--buffer-size',buffer,
         '--keyint','12','--bframes',str(bframes),'--fixed-gop-grid','--no-scene-cut',
         '--search-range',str(search),'--local-search-range',str(search),'--me-quality','sad',
         '--simd','none','--threads','1','--no-aq','--debug-stats',str(stats)]
    if mbstats:
        cmd += ['--macroblock-stats',str(mb)]
    if disable_pb:
        cmd += ['--debug-disable-pb-intra']
    if disable_p:
        cmd += ['--debug-disable-p-intra']
    if disable_b:
        cmd += ['--debug-disable-b-intra']
    if weights:
        iw,pw,bw=weights
        cmd += ['--i-block-weight',str(iw),'--p-block-weight',str(pw),'--b-block-weight',str(bw)]
    p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if p.returncode:
        raise SystemExit(f'{tag} encode failed ({p.returncode}):\n{p.stderr}')
    rows=list(csv.DictReader(open(stats,newline='')))
    mbrows=read_macroblock_stats(mb)[3] if mbstats else []
    return out.read_bytes(),rows,mbrows,p.stderr


def plans(rows):
    d={}
    for r in rows:
        d.setdefault(r['type'],[]).append(float(r['planned_bits']))
    for typ in ('I','P','B'):
        if typ not in d:
            raise SystemExit(f'missing {typ} picture in pure fixture')
    return {k:sum(v)/len(v) for k,v in d.items()}


def close(a,b,rel=2e-6):
    return abs(a-b) <= rel*max(1.0,abs(a),abs(b))


def frame(rows,typ):
    x=[r for r in rows if r['type']==typ]
    if len(x)!=1:
        raise SystemExit(f'expected one {typ} row, got {len(x)}')
    return x[0]


def mb_for(mbrows,typ):
    return [r for r in mbrows if frame_value(r,'frame_type')==typ]


with tempfile.TemporaryDirectory(prefix='libvc1-block-weights-') as d:
    td=pathlib.Path(d)

    # Pure-picture compatibility: suppress P/B intra so the macroblock model
    # collapses exactly to the old picture-class weight law.
    pure=td/'pure.y4m'; write_y4m(pure,pure_frames())
    default_bs,default_rows,_,default_log=run(td,pure,'pure-default',disable_pb=True)
    doubled_bs,doubled_rows,_,_=run(td,pure,'pure-doubled',weights=(10.0,2.0,1.4),disable_pb=True)
    custom_bs,custom_rows,_,custom_log=run(td,pure,'pure-custom',weights=(5.0,2.0,0.5),disable_pb=True)
    dp=plans(default_rows); xp=plans(doubled_rows); cp=plans(custom_rows)
    if not close(dp['I']/dp['P'],5.0) or not close(dp['B']/dp['P'],0.70):
        raise SystemExit(f'pure default planned-bit ratios wrong: {dp}')
    if not close(cp['I']/cp['P'],2.5) or not close(cp['B']/cp['P'],0.25):
        raise SystemExit(f'pure custom planned-bit ratios wrong: {cp}')
    for typ in ('I','P','B'):
        if not close(dp[typ],xp[typ],1e-9):
            raise SystemExit(f'common scaling changed pure {typ} plan: {dp} vs {xp}')
    if default_bs != doubled_bs:
        raise SystemExit('common scaling of block weights changed pure-picture bitstream')
    if not close(sum(float(r['planned_bits']) for r in default_rows),
                 sum(float(r['planned_bits']) for r in custom_rows),2e-6):
        raise SystemExit('pure-picture block weights changed normalized GOP budget')

    # Mixed P picture: exactly half of the second picture is deliberately a poor
    # temporal match. Those eight P-intra MBs must earn I-class budget and finer
    # local MQUANT than the eight temporal P MBs.
    psrc=td/'mixed-p.y4m'; write_y4m(psrc,mixed_frames('p'))
    pbs,prows,pmb,_=run(td,psrc,'mixed-p',bframes=0,search=4,mbstats=True,
                        bitrate='100k',buffer='500k')
    _,pbase_rows,_,_=run(td,psrc,'mixed-p-no-intra',bframes=0,search=4,disable_p=True,
                         bitrate='100k',buffer='500k')
    pr=frame(prows,'P'); pbase=frame(pbase_rows,'P')
    if int(pr['p_intra_mb'])!=8:
        raise SystemExit(f'mixed P fixture expected 8 P-intra MBs, got {pr["p_intra_mb"]}')
    if float(pr['planned_bits']) <= float(pbase['planned_bits'])*1.5:
        raise SystemExit('P-intra blocks did not increase the mixed P picture block-weight plan')
    pblocks=mb_for(pmb,'P')
    pintra=[int(r['mquant']) for r in pblocks if r['mode']=='P-intra']
    ptemp=[int(r['mquant']) for r in pblocks if r['mode']!='P-intra']
    if len(pintra)!=8 or len(ptemp)!=8 or max(pintra)>=min(ptemp):
        raise SystemExit(f'P block-class MQUANT separation failed: intra={pintra}, temporal={ptemp}')

    # Mixed B picture: same rule, now for B-intra versus temporal B coding.
    bsrc=td/'mixed-b.y4m'; write_y4m(bsrc,mixed_frames('b'))
    bbs,brows,bmb,_=run(td,bsrc,'mixed-b',bframes=1,search=0,mbstats=True,
                        bitrate='100k',buffer='500k')
    _,bbase_rows,_,_=run(td,bsrc,'mixed-b-no-intra',bframes=1,search=0,disable_b=True,
                         bitrate='100k',buffer='500k')
    bscaled_bs,bscaled_rows,bscaled_mb,_=run(td,bsrc,'mixed-b-doubled',weights=(10.0,2.0,1.4),
                                             bframes=1,search=0,mbstats=True,
                                             bitrate='100k',buffer='500k')
    br=frame(brows,'B'); bbase=frame(bbase_rows,'B')
    if int(br['p_intra_mb'])!=8:
        raise SystemExit(f'mixed B fixture expected 8 B-intra MBs, got {br["p_intra_mb"]}')
    if float(br['planned_bits']) <= float(bbase['planned_bits'])*1.5:
        raise SystemExit('B-intra blocks did not increase the mixed B picture block-weight plan')
    bblocks=mb_for(bmb,'B')
    bintra=[int(r['mquant']) for r in bblocks if r['mode'] in ('B-intra','BI-intra')]
    btemp=[int(r['mquant']) for r in bblocks if r['mode'] not in ('B-intra','BI-intra')]
    if len(bintra)!=8 or len(btemp)!=8 or max(bintra)>=min(btemp):
        raise SystemExit(f'B block-class MQUANT separation failed: intra={bintra}, temporal={btemp}')
    if bbs != bscaled_bs:
        raise SystemExit('common scaling of block weights changed mixed-block bitstream')
    if [r['mquant'] for r in bblocks] != [r['mquant'] for r in mb_for(bscaled_mb,'B')]:
        raise SystemExit('common scaling changed mixed-B local MQUANT decisions')

    # Startup configuration is printed before the encode; the completion line
    # contains measured results rather than repeating enabled options. The
    # legacy frame-named switches remain accepted aliases, but the help-facing
    # controls are block weights.
    norm_log=default_log.replace('0.70','0.7')
    if 'weights=I:5/P:1/B:0.7' not in norm_log:
        raise SystemExit('startup options did not report default I/P/B block weights')
    if 'rate-mode=abr, target=' not in norm_log:
        raise SystemExit('startup options did not report ABR ceiling')
    if norm_log.find('weights=I:') > norm_log.find('encoded '):
        raise SystemExit('block weights were reported after rather than before the encode')
    if norm_log.find('hrd-underflows=') < norm_log.find('encoded '):
        raise SystemExit('measured HRD result was reported before the encode completed')
    if 'weights=I:5/P:2/B:0.5' not in custom_log:
        raise SystemExit('startup options did not report custom I/P/B block weights')

    # CQP must reject the ABR-only block controls. Exercise the new spelling.
    bad=subprocess.run([enc,'-i',str(pure),'-o',str(td/'bad.m2ts'),'--cq','4','--i-block-weight','5'],
                       stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if bad.returncode==0 or 'apply only to bitrate-controlled mode' not in bad.stderr:
        raise SystemExit('CQP unexpectedly accepted block-weight control')

print('I/P/B normalized ABR macroblock-weight runtime regression: PASS')
