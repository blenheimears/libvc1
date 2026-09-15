#!/usr/bin/env python3
import csv
import pathlib
import subprocess
import sys
import tempfile

enc=pathlib.Path(sys.argv[1])
ffmpeg=sys.argv[2]
W=H=64
MB_TOTAL=((W+15)//16)*((H+15)//16)


def write_y4m(path,kind):
    frames=2 if kind=='p-acpred' else (3 if kind=='partial-b' else 7)
    with path.open('wb') as f:
        f.write(f'YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for frame in range(frames):
            f.write(b'FRAME\n')
            y=bytearray(W*H)
            for yy in range(H):
                for xx in range(W):
                    stable=40+(((xx//4)+(yy//4))&1)*160
                    if kind=='p-acpred':
                        if frame==0:
                            v=30+(((xx+yy)&1)*190)
                        else:
                            v=50+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                    elif kind=='partial-b':
                        if frame==1 and xx>=W//2:
                            v=48+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                        else:
                            v=stable
                    else: # all-intra B slots -> BI
                        if frame in (1,3,5):
                            v=40+(((xx//8)+(yy//8)*3+frame)&1)*150+((xx&7)*3)
                        else:
                            v=30+(((xx+yy+frame*5)&1)*190)
                    y[yy*W+xx]=max(16,min(235,v))
            f.write(y)
            f.write(bytes([128])*((W//2)*(H//2)))
            f.write(bytes([128])*((W//2)*(H//2)))


def run(kind,profile,threads=1):
    td=current
    src=td/f'{kind}.y4m'
    if not src.exists(): write_y4m(src,kind)
    ext='m2ts' if profile=='advanced' else 'wmv'
    tag=f'{kind}-{profile}-t{threads}'
    out=td/f'{tag}.{ext}'
    stats=td/f'{tag}.csv'
    recon=td/f'{tag}-lib.yuv'
    es=td/f'{tag}.vc1'
    bframes=0 if kind=='p-acpred' else 1
    cmd=[str(enc),'-i',str(src),'-o',str(out),'--debug-stats',str(stats),'--recon-out',str(recon),
         '--cq','8','--keyint','24','--bframes',str(bframes),'--threads',str(threads),'--simd','scalar','--no-scene-cut',
         '--search-range','4','--local-search-range','4','--fixed-gop-grid','--no-aq','--no-loop-filter',
         '--fixed-8x8','--trellis','0']
    if profile=='advanced': cmd += ['--es-out',str(es)]
    subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    decoded=td/f'{tag}-ffmpeg.yuv'
    decode_in=es if profile=='advanced' else out
    decode_cmd=[ffmpeg,'-hide_banner','-v','error','-xerror','-err_detect','explode']
    if profile=='advanced': decode_cmd += ['-f','vc1']
    decode_cmd += ['-i',str(decode_in),'-pix_fmt','yuv420p','-fps_mode','passthrough','-f','rawvideo','-y',str(decoded)]
    subprocess.run(decode_cmd,check=True)
    if recon.read_bytes()!=decoded.read_bytes():
        raise SystemExit(f'{tag}: libvc1 reconstruction differs from FFmpeg')
    with stats.open(newline='') as f: rows=list(csv.DictReader(f))
    return out,es,rows


with tempfile.TemporaryDirectory(prefix='libvc1-step2-') as td:
    current=pathlib.Path(td)

    # P-picture intra AC prediction: the replacement frame is spatially
    # correlated but temporally unrelated, forcing real P-intra + ACPRED.
    for profile in ('advanced','main'):
        _,_,rows=run('p-acpred',profile)
        prow=next((r for r in rows if r['type']=='P'),None)
        if not prow or int(prow['p_intra_mb'])!=MB_TOTAL or int(prow['acpred_mb'])<=0:
            raise SystemExit(f'{profile}: P-intra ACPRED fixture did not exercise the requested path')

    # Mixed B picture: left half remains temporally predictable while right
    # half is new directional texture. This must use actual B-macroblock intra
    # syntax, not promote the whole picture to BI.
    adv1,_,rows=run('partial-b','advanced',1)
    brow=next((r for r in rows if r['type']=='B'),None)
    if not brow:
        raise SystemExit('partial-B fixture produced no B picture')
    intra=int(brow['p_intra_mb']) # historical CSV column now reports current-picture intra MBs
    temporal=sum(int(brow[k]) for k in ('b_forward','b_backward','b_interpolated','b_direct'))
    if not (0<intra<MB_TOTAL and temporal>0 and int(brow['acpred_mb'])>0):
        raise SystemExit(f'partial-B fixture missed B-intra/ACPRED path: intra={intra}, temporal={temporal}')
    adv4,_,rows4=run('partial-b','advanced',4)
    if adv1.read_bytes()!=adv4.read_bytes():
        raise SystemExit('partial B-intra output differs across thread counts')

    # The WMV9/Main macroblock syntax has the same requested intra capability.
    # Exact decoder reconstruction also covers its non-transposed coefficient
    # storage convention.
    _,_,rows=run('partial-b','main',1)
    brow=next((r for r in rows if r['type']=='B'),None)
    if not brow:
        raise SystemExit('Main partial-B fixture produced no B picture')
    intra=int(brow['p_intra_mb'])
    temporal=sum(int(brow[k]) for k in ('b_forward','b_backward','b_interpolated','b_direct'))
    if not (0<intra<MB_TOTAL and temporal>0 and int(brow['acpred_mb'])>0):
        raise SystemExit(f'Main partial-B fixture missed B-intra/ACPRED path: intra={intra}, temporal={temporal}')

    # All-intra B slots are emitted as Advanced BI (PTYPE=1110). Use several
    # groups so the same fixture also crosses multiple anchor/B reorder points.
    _,es,rows=run('all-bi','advanced')
    brows=[r for r in rows if r['type']=='B']
    if not brows or not all(int(r['p_intra_mb'])==MB_TOTAL and int(r['acpred_mb'])>0 for r in brows):
        raise SystemExit('Advanced BI fixture did not select all-intra B slots')
    data=es.read_bytes(); marker=b'\x00\x00\x01\x0d'; first=[]; off=0
    while True:
        at=data.find(marker,off)
        if at<0: break
        if at+4>=len(data): raise SystemExit('truncated frame BDU')
        first.append(data[at+4]); off=at+4
    if sum(1 for v in first if (v>>4)==0xE)!=len(brows):
        raise SystemExit(f'Advanced BI PTYPE not found for every all-intra B slot: {[hex(v) for v in first]}')

    # Main-profile BI uses BFRACTION=BI and, importantly, resets implicit RND
    # state. Multiple BI/P groups plus exact FFmpeg reconstruction check both.
    _,_,rows=run('all-bi','main')
    brows=[r for r in rows if r['type']=='B']
    if not brows or not all(int(r['p_intra_mb'])==MB_TOTAL and int(r['acpred_mb'])>0 for r in brows):
        raise SystemExit('Main BI fixture did not select all-intra B slots')

print('Step 2 B-intra/BI/inter-picture ACPRED runtime: PASS')
