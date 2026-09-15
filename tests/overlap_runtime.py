#!/usr/bin/env python3
import csv
import pathlib
import subprocess
import sys
import tempfile

enc=pathlib.Path(sys.argv[1])
ffmpeg=sys.argv[2]
W,H=64,48


def write_i_fixture(path):
    cw,ch=W//2,H//2
    with path.open('wb') as f:
        f.write(f'YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for n in range(4):
            f.write(b'FRAME\n')
            y=bytearray(W*H); u=bytearray(cw*ch); v=bytearray(cw*ch)
            for yy in range(H):
                for xx in range(W):
                    val=35+((xx//8+yy//8+n)&1)*150+(xx*3+yy*5+n*17)%47
                    y[yy*W+xx]=max(0,min(255,val))
            for yy in range(ch):
                for xx in range(cw):
                    u[yy*cw+xx]=(64+((xx//4+n)&1)*110+yy*3)%256
                    v[yy*cw+xx]=(192-((yy//4+n)&1)*100+xx*2)%256
            f.write(y); f.write(u); f.write(v)


def write_p_fixture(path):
    w=h=64
    with path.open('wb') as f:
        f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for frame in range(2):
            f.write(b'FRAME\n'); y=bytearray(w*h)
            for yy in range(h):
                for xx in range(w):
                    if frame==0: v=30+(((xx+yy)&1)*190)
                    else: v=50+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                    y[yy*w+xx]=max(16,min(235,v))
            f.write(y); f.write(bytes([128])*((w//2)*(h//2))*2)


def encode_decode(td,src,tag,profile,q,extra=()):
    main=profile=='main'; ext='wmv' if main else 'm2ts'
    out=td/f'{tag}.{ext}'; recon=td/f'{tag}-lib.yuv'; dec=td/f'{tag}-ffmpeg.yuv'
    es=td/f'{tag}.vc1'; stats=td/f'{tag}.csv'
    cmd=[str(enc),'-i',str(src),'-o',str(out),'--cq',str(q),'--threads','1','--simd','scalar','--no-scene-cut',
         '--no-loop-filter','--recon-out',str(recon),'--debug-stats',str(stats),*extra]
    if not main: cmd += ['--es-out',str(es)]
    subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    inp=out; dcmd=[ffmpeg,'-hide_banner','-v','error','-xerror','-err_detect','explode']
    if not main: inp=es; dcmd += ['-f','vc1']
    dcmd += ['-i',str(inp),'-pix_fmt','yuv420p','-fps_mode','passthrough','-f','rawvideo','-y',str(dec)]
    subprocess.run(dcmd,check=True)
    if recon.read_bytes()!=dec.read_bytes():
        raise SystemExit(f'{tag}: libvc1 reconstruction differs from FFmpeg')
    with stats.open(newline='') as f: rows=list(csv.DictReader(f))
    return out,recon,rows


with tempfile.TemporaryDirectory(prefix='libvc1-overlap-') as t:
    td=pathlib.Path(t); src=td/'i.y4m'; write_i_fixture(src)

    # Advanced high-Q: unconditional OVERLAP must change reconstruction and
    # agree exactly with an independent decoder.
    _,adv_on,_=encode_decode(td,src,'adv-q12-on','advanced',12,('--intra-only','--overlap'))
    _,adv_off,_=encode_decode(td,src,'adv-q12-off','advanced',12,('--intra-only','--no-overlap'))
    if adv_on.read_bytes()==adv_off.read_bytes():
        raise SystemExit('Advanced high-Q OVERLAP did not change reconstruction')

    # Advanced low-Q consumes CONDOVER (including the SELECT/OVERFLAGS-capable
    # encoder path); whatever RDO mode is selected must reconstruct identically.
    encode_decode(td,src,'adv-q6','advanced',6,('--intra-only','--overlap'))

    # Main high-Q uses sequence OVERLAP with the Main-specific centered intra
    # reconstruction convention. Low-Q must leave the I filter inactive.
    _,main_on,_=encode_decode(td,src,'main-q12-on','main',12,('--intra-only','--overlap'))
    _,main_off,_=encode_decode(td,src,'main-q12-off','main',12,('--intra-only','--no-overlap'))
    if main_on.read_bytes()==main_off.read_bytes():
        raise SystemExit('Main high-Q OVERLAP did not change reconstruction')
    encode_decode(td,src,'main-q6','main',6,('--intra-only','--overlap'))

    # P overlap is restricted to intra macroblocks. Force a temporally unrelated
    # second picture so every P MB is intra, then verify both profile paths.
    psrc=td/'p.y4m'; write_p_fixture(psrc)
    pextra=('--keyint','24','--bframes','0','--search-range','4','--local-search-range','4',
            '--fixed-gop-grid','--no-aq','--fixed-8x8','--trellis','0','--overlap')
    for profile in ('advanced','main'):
        _,_,rows=encode_decode(td,psrc,f'p-{profile}',profile,12,pextra)
        prow=next((r for r in rows if r['type']=='P'),None)
        if not prow or int(prow['p_intra_mb'])!=16:
            raise SystemExit(f'{profile}: P-overlap fixture did not force all 16 intra macroblocks')

print('OVERLAP/CONDOVER runtime decoder equivalence: PASS')
