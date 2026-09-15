#!/usr/bin/env python3
import os, re, subprocess, sys, tempfile
from pathlib import Path

if len(sys.argv) != 3:
    raise SystemExit("usage: progressive_transform_runtime.py ENCODER FFMPEG")
enc, ffmpeg = sys.argv[1:]

def run(cmd, **kw):
    return subprocess.run(cmd, check=True, **kw)

def make_y4m(path: Path, w=64, h=64, frames=10):
    with path.open('wb') as f:
        f.write(f"YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n".encode())
        for n in range(frames):
            f.write(b"FRAME\n")
            y=bytearray(w*h)
            # Deterministic moving, textured luma with both smooth and sharp regions.
            for yy in range(h):
                for xx in range(w):
                    sx=(xx+3*n)%w; sy=(yy+2*n)%h
                    checker=52 if ((sx//4) ^ (sy//4)) & 1 else -44
                    grad=(3*sx + 2*sy + 9*n) & 127
                    v=96 + checker + grad//2
                    if 18+n <= xx < 34+n and 20 <= yy < 42:
                        v=220 if ((xx+yy+n)&2) else 28
                    y[yy*w+xx]=max(0,min(255,v))
            cw,ch=w//2,h//2
            u=bytearray(cw*ch); v=bytearray(cw*ch)
            for yy in range(ch):
                for xx in range(cw):
                    u[yy*cw+xx]=(96 + 3*xx + 5*n + (24 if ((xx+yy)//3)&1 else 0)) & 255
                    v[yy*cw+xx]=(160 + 2*yy - 4*n - (20 if ((xx*2+yy)//4)&1 else 0)) & 255
            f.write(y); f.write(u); f.write(v)

def extract_vc1_from_m2ts(path: Path) -> bytes:
    data=path.read_bytes()
    if len(data)%192:
        raise AssertionError("M2TS size is not a multiple of 192 bytes")
    groups=[]; cur=bytearray(); last_cc=None
    for off in range(0,len(data),192):
        ts=data[off+4:off+192]
        if len(ts)!=188 or ts[0]!=0x47:
            raise AssertionError("bad TS sync")
        pusi=bool(ts[1]&0x40); pid=((ts[1]&0x1f)<<8)|ts[2]
        if pid!=0x1011:
            continue
        afc=(ts[3]>>4)&3; cc=ts[3]&15
        if afc&1:
            if last_cc is not None and cc != ((last_cc+1)&15):
                raise AssertionError("video PID continuity-counter discontinuity")
            last_cc=cc
        pos=4
        if afc&2:
            afl=ts[pos]; pos += 1+afl
            if pos>188: raise AssertionError("invalid adaptation field")
        payload=ts[pos:] if afc&1 else b''
        if pusi and cur:
            groups.append(bytes(cur)); cur.clear()
        cur.extend(payload)
    if cur: groups.append(bytes(cur))
    out=bytearray()
    if not groups: raise AssertionError("no VC-1 PES packets found")
    for g in groups:
        if len(g)<9 or g[:4] != b'\x00\x00\x01\xfd':
            raise AssertionError("unexpected VC-1 PES stream_id")
        h=9+g[8]
        if h>len(g): raise AssertionError("truncated PES header")
        out.extend(g[h:])
    return bytes(out)

with tempfile.TemporaryDirectory(prefix='libvc1-transform-') as td:
    d=Path(td); src=d/'in.y4m'; make_y4m(src)
    m2ts=d/'out.m2ts'; es=d/'out.vc1'; recon=d/'recon.yuv'; dec=d/'ff.yuv'
    p=subprocess.run([enc,'-i',str(src),'-o',str(m2ts),'--es-out',str(es),'--recon-out',str(recon),
                      '--cq','6','--threads','1','--simd','scalar','--bframes','2','--keyint','10',
                      '--no-scene-cut','--no-loop-filter'], text=True, stdout=subprocess.PIPE,
                     stderr=subprocess.PIPE, check=True)
    m=re.search(r'TT=8x8:(\d+)/8x4:(\d+)/4x8:(\d+)/4x4:(\d+)',p.stderr)
    if not m or sum(map(int,m.groups()))==0:
        raise AssertionError('transform decisions were not exercised')
    run([ffmpeg,'-v','error','-f','vc1','-i',str(es),'-pix_fmt','yuv420p','-f','rawvideo','-y',str(dec)])
    if recon.read_bytes()!=dec.read_bytes():
        raise AssertionError('raw VC-1 decoder reconstruction differs from encoder reconstruction')
    extracted=extract_vc1_from_m2ts(m2ts)
    if extracted!=es.read_bytes():
        raise AssertionError('M2TS PES payload does not reproduce --es-out byte-for-byte')
print('progressive transform runtime/reconstruction/PES checks ok')
