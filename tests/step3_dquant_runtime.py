#!/usr/bin/env python3
import csv
import pathlib
import subprocess
import sys
import tempfile

enc=pathlib.Path(sys.argv[1])
ffmpeg=sys.argv[2]


def write_y4m(path,w,h,frames):
    with path.open('wb') as f:
        f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for yplane in frames:
            f.write(b'FRAME\n')
            f.write(bytes(yplane))
            f.write(bytes([128])*((w//2)*(h//2)))
            f.write(bytes([128])*((w//2)*(h//2)))


def mainp_source(path):
    w,h=96,64
    # Keep the DQUANT mechanics fixture above the 0.1.78 dim-AQ protection window.
    a=bytearray([170])*(w*h)
    b=bytearray(a)
    for my in range((h+15)//16):
        for mx in range((w+15)//16):
            for yy in range(my*16+6,min(my*16+10,h)):
                for xx in range(mx*16+6,min(mx*16+10,w)):
                    b[yy*w+xx]=220
    write_y4m(path,w,h,[a,b])


def pac_source(path):
    w=h=64
    frames=[]
    for frame in range(2):
        y=bytearray(w*h)
        for yy in range(h):
            for xx in range(w):
                if frame==0:
                    v=30+(((xx+yy)&1)*190)
                else:
                    v=50+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                y[yy*w+xx]=max(16,min(235,v))
        frames.append(y)
    write_y4m(path,w,h,frames)


def bi_source(path):
    w=h=64
    frames=[]
    for frame in range(7):
        y=bytearray(w*h)
        for yy in range(h):
            for xx in range(w):
                if frame in (1,3,5):
                    v=40+(((xx//8)+(yy//8)*3+frame)&1)*150+((xx&7)*3)
                else:
                    v=30+(((xx+yy+frame*5)&1)*190)
                y[yy*w+xx]=max(16,min(235,v))
        frames.append(y)
    write_y4m(path,w,h,frames)


def encode(src,tag,profile='advanced',cq='16',bframes=0,threads=1,search=4):
    ext='m2ts' if profile=='advanced' else 'wmv'
    out=td/f'{tag}.{ext}'
    stats=td/f'{tag}.csv'
    recon=td/f'{tag}-lib.yuv'
    es=td/f'{tag}.vc1'
    cmd=[str(enc),'-i',str(src),'-o',str(out),'--debug-stats',str(stats),'--recon-out',str(recon),
         '--cq',str(cq),'--keyint','24','--bframes',str(bframes),'--threads',str(threads),
         '--simd','scalar','--no-scene-cut','--search-range',str(search),'--local-search-range',str(min(search,4)),
         '--fixed-gop-grid','--no-loop-filter','--fixed-8x8','--trellis','0']
    if profile=='advanced': cmd += ['--es-out',str(es)]
    subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    decoded=td/f'{tag}-ffmpeg.yuv'
    decode_in=es if profile=='advanced' else out
    dcmd=[ffmpeg,'-hide_banner','-v','error','-xerror','-err_detect','explode']
    if profile=='advanced': dcmd += ['-f','vc1']
    dcmd += ['-i',str(decode_in),'-pix_fmt','yuv420p','-fps_mode','passthrough','-f','rawvideo','-y',str(decoded)]
    subprocess.run(dcmd,check=True)
    if recon.read_bytes()!=decoded.read_bytes():
        raise SystemExit(f'{tag}: libvc1 reconstruction differs from strict FFmpeg decode')
    with stats.open(newline='') as f: rows=list(csv.DictReader(f))
    return out,es,rows


with tempfile.TemporaryDirectory(prefix='libvc1-step3-') as tmp:
    td=pathlib.Path(tmp)
    mainp=td/'mainp.y4m'; mainp_source(mainp)
    pac=td/'pac.y4m'; pac_source(pac)
    bi=td/'bi.y4m'; bi_source(bi)

    # Main Profile now carries sequence DQUANT and uses legal P/B MQUANT syntax.
    _,_,rows=encode(mainp,'main-p','main','6',0,1,0)
    prow=next((r for r in rows if r['type']=='P'),None)
    if not prow or int(prow['dquant_mb'])<=0 or int(prow['mquant_min'])==int(prow['qp']):
        raise SystemExit('Main-profile P fixture did not exercise DQUANT')
    irow=next(r for r in rows if r['type']=='I')
    if int(irow['dquant_mb'])!=0:
        raise SystemExit('Main-profile I picture unexpectedly consumed macroblock DQUANT')

    # HALFQP and DQUANT must coexist: picture-Q macroblocks retain the half step,
    # while DQUANT-derived macroblocks use integer MQUANT.
    _,_,rows=encode(mainp,'halfqp-p','advanced','4.5',0,1,0)
    prow=next((r for r in rows if r['type']=='P'),None)
    if not prow or int(prow['halfqp'])!=1 or int(prow['dquant_mb'])<=0 or int(prow['mquant_max'])<=int(prow['qp']):
        raise SystemExit('HALFQP + DQUANT fixture did not exercise mixed picture/MQUANT semantics')

    # Intra macroblocks embedded in P pictures must scale both DC and AC
    # predictors when adjacent macroblocks use different quantizers.
    adv1,_,rows=encode(pac,'pac-advanced-t1','advanced','16',0,1,4)
    prow=next((r for r in rows if r['type']=='P'),None)
    if not prow or int(prow['p_intra_mb'])!=16 or int(prow['dquant_mb'])<=0 or int(prow['acpred_mb'])<=0:
        raise SystemExit('Advanced P-intra DQUANT/ACPRED fixture missed requested path')
    adv4,_,rows4=encode(pac,'pac-advanced-t4','advanced','16',0,4,4)
    if adv1.read_bytes()!=adv4.read_bytes():
        raise SystemExit('Advanced DQUANT output differs between one and four GOP workers')
    _,_,rows=encode(pac,'pac-main','main','16',0,1,4)
    prow=next((r for r in rows if r['type']=='P'),None)
    if not prow or int(prow['p_intra_mb'])!=16 or int(prow['dquant_mb'])<=0 or int(prow['acpred_mb'])<=0:
        raise SystemExit('Main P-intra DQUANT/ACPRED fixture missed requested path')

    # Advanced I and BI pictures now consume full DQUANT. Several all-intra B
    # slots ensure BI syntax and q-aware intra prediction cross reorder points.
    _,es,rows=encode(bi,'advanced-bi','advanced','16',1,1,4)
    irow=next((r for r in rows if r['type']=='I'),None)
    brows=[r for r in rows if r['type']=='B']
    if not irow or int(irow['dquant_mb'])<=0:
        raise SystemExit('Advanced I fixture did not exercise DQUANT')
    if not brows or not all(int(r['p_intra_mb'])==16 and int(r['dquant_mb'])>0 and int(r['acpred_mb'])>0 for r in brows):
        raise SystemExit('Advanced BI fixture did not exercise BI intra DQUANT/ACPRED')
    data=es.read_bytes(); marker=b'\x00\x00\x01\x0d'; first=[]; off=0
    while True:
        at=data.find(marker,off)
        if at<0: break
        if at+4>=len(data): raise SystemExit('truncated Advanced frame BDU')
        first.append(data[at+4]); off=at+4
    if sum(1 for v in first if (v>>4)==0xE)!=len(brows):
        raise SystemExit('Advanced BI PTYPE missing from DQUANT BI fixture')

print('Step 3 complete DQUANT runtime: PASS')
