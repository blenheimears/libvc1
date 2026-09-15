#!/usr/bin/env python3
import csv
import pathlib
import subprocess
import sys
import tempfile

enc=pathlib.Path(sys.argv[1])
ffmpeg=sys.argv[2]
W=H=64
FRAME_BYTES=W*H*3//2
CHROMA_SAMPLES=(W//2)*(H//2)


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
                            # Deliberately poor temporal match: this forces P-intra.
                            v=50+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                    else:
                        stable=40+(((xx//4)+(yy//4))&1)*160
                        if frame==1 and xx>=W//2:
                            # Right half becomes new texture: mixed B inter/intra.
                            v=48+(((xx//8)+(yy//8)*3)&1)*140+((xx&7)*3)
                        else:
                            v=stable
                    y[yy*W+xx]=max(16,min(235,v))
            f.write(y)
            # Neutral chroma makes the Main-profile centering invariant exact:
            # correctly coded inter-picture intra blocks must decode to U=V=128.
            f.write(bytes([128])*CHROMA_SAMPLES)
            f.write(bytes([128])*CHROMA_SAMPLES)


def run(td,kind):
    src=td/f'{kind}.y4m'; write_fixture(src,kind)
    out=td/f'{kind}.wmv'; stats=td/f'{kind}.csv'; recon=td/f'{kind}-recon.yuv'; dec=td/f'{kind}-ffmpeg.yuv'
    bframes=0 if kind=='p' else 1
    search=4 if kind=='p' else 0
    cmd=[str(enc),'-i',str(src),'-o',str(out),'--debug-stats',str(stats),'--recon-out',str(recon),
         '--cq','8','--keyint','24','--bframes',str(bframes),'--threads','1','--simd','scalar','--no-scene-cut',
         '--search-range',str(search),'--local-search-range',str(search),'--fixed-gop-grid','--no-aq',
         '--no-loop-filter','--fixed-8x8','--trellis','0']
    subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.run([ffmpeg,'-hide_banner','-v','error','-xerror','-err_detect','explode','-i',str(out),
                    '-pix_fmt','yuv420p','-fps_mode','passthrough','-f','rawvideo','-y',str(dec)],check=True)
    if recon.read_bytes()!=dec.read_bytes():
        raise SystemExit(f'{kind}: libvc1 reconstruction differs from FFmpeg')
    with stats.open(newline='') as f: rows=list(csv.DictReader(f))
    target='P' if kind=='p' else 'B'
    row=next((r for r in rows if r['type']==target),None)
    if row is None or int(row['p_intra_mb'])<=0:
        raise SystemExit(f'{kind}: fixture failed to exercise {target}-picture intra macroblocks')
    raw=dec.read_bytes()
    if len(raw)%FRAME_BYTES: raise SystemExit(f'{kind}: unexpected decoded frame size')
    frame_index=1
    fr=raw[frame_index*FRAME_BYTES:(frame_index+1)*FRAME_BYTES]
    u=fr[W*H:W*H+CHROMA_SAMPLES]
    v=fr[W*H+CHROMA_SAMPLES:W*H+2*CHROMA_SAMPLES]
    if not u or not v or min(u)!=128 or max(u)!=128 or min(v)!=128 or max(v)!=128:
        raise SystemExit(f'{kind}: neutral chroma was shifted/saturated by inter-picture intra DC centering: '
                         f'U={min(u) if u else -1}..{max(u) if u else -1}, V={min(v) if v else -1}..{max(v) if v else -1}')
    return int(row['p_intra_mb'])


with tempfile.TemporaryDirectory(prefix='libvc1-main-inter-intra-dc-') as t:
    td=pathlib.Path(t)
    p=run(td,'p')
    b=run(td,'b')
    if p<16: raise SystemExit(f'P fixture expected all 16 P-intra macroblocks, got {p}')
    if not (0<b<16): raise SystemExit(f'B fixture expected mixed B-intra/inter coding, got {b} intra macroblocks')

print('WMV9/Main P/B inter-picture intra DC centering runtime: PASS')
