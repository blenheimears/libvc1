#!/usr/bin/env python3
import csv
import pathlib
import subprocess
import sys
import tempfile

if len(sys.argv) != 3:
    raise SystemExit('usage: abr_coding_law_runtime.py VC1ENC FFMPEG')
enc, ffmpeg = sys.argv[1:]
W,H,N=96,64,48
FB=W*H*3//2

def write_fixture(path):
    out=bytearray(f'YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for i in range(N):
        y=bytearray(W*H)
        for yy in range(H):
            for xx in range(W):
                value=28 if (((xx+3*i)//3+(yy+2*i)//3)&1)==0 else 220
                if 18 <= xx-((i*2)%40) < 42 and 14 <= yy < 50:
                    value=(xx*13+yy*7+i*19)&255
                y[yy*W+xx]=value
        u=bytes([112+((i*3)&15)])*(W//2*H//2)
        v=bytes([144-((i*5)&15)])*(W//2*H//2)
        out += b'FRAME\n'+y+u+v
    pathlib.Path(path).write_bytes(out)

def run(src, td, tag, threads, no_half=False):
    m2ts=td/f'{tag}.m2ts'; es=td/f'{tag}.vc1'; recon=td/f'{tag}.yuv'; stats=td/f'{tag}.csv'
    cmd=[enc,'-i',str(src),'-o',str(m2ts),'--es-out',str(es),'--recon-out',str(recon),
         '--debug-stats',str(stats),'--bitrate','2m','--buffer-size','4m','--keyint','12',
         '--threads',str(threads),'--search-range','8','--no-scene-cut','--simd','scalar']
    if no_half: cmd.append('--no-halfqp')
    p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if p.returncode:
        raise SystemExit(f'{tag} encode failed ({p.returncode}):\n{p.stderr}')
    if 'hrd-underflows=0' not in p.stderr:
        raise SystemExit(f'{tag} did not report zero HRD underflows')
    rows=list(csv.DictReader(open(stats,newline='')))
    if len(rows)!=N:
        raise SystemExit(f'{tag} debug row count mismatch: {len(rows)}')
    return m2ts,es,recon,rows

with tempfile.TemporaryDirectory(prefix='libvc1-072-abr-law-') as d:
    td=pathlib.Path(d); src=td/'fixture.y4m'; write_fixture(src)
    m1,e1,r1,rows=run(src,td,'t1',1)
    if not any(int(row['halfqp']) for row in rows):
        raise SystemExit('ABR AUTO did not exercise a legal HALFQP picture')
    if not any(row['quantizer_type']=='nonuniform' for row in rows):
        raise SystemExit('ABR AUTO did not exercise nonuniform PQUANT')
    if not any(row['type']=='P' for row in rows) or not any(row['type']=='B' for row in rows):
        raise SystemExit('ABR parity fixture did not contain both P and B pictures')
    # The exported effective step must include HALFQP rather than silently
    # reporting 2*PQINDEX as older debug output did.
    for row in rows:
        expected=2*int(row['qp'])+int(row['halfqp'])
        if abs(float(row['quant_step'])-expected)>1e-9:
            raise SystemExit('debug quant_step does not match signaled HALFQP state')

    _,_,_,nohalf=run(src,td,'nohalf',1,True)
    if any(int(row['halfqp']) for row in nohalf):
        raise SystemExit('--no-halfqp did not suppress automatic ABR HALFQP')

    # Independent decoder must reconstruct exactly the reference retained by
    # the encoder for a stream containing automatic HALFQP/nonuniform choices.
    dec=td/'ffmpeg.yuv'
    p=subprocess.run([ffmpeg,'-hide_banner','-v','error','-xerror','-err_detect','explode',
                      '-f','vc1','-i',str(e1),'-pix_fmt','yuv420p','-f','rawvideo','-y',str(dec)],
                     stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    if p.returncode:
        raise SystemExit('FFmpeg rejected ABR parity stream: '+p.stderr.decode(errors='replace'))
    if dec.read_bytes()!=pathlib.Path(r1).read_bytes():
        raise SystemExit('ABR parity retained reconstruction differs from FFmpeg')

    m4,e4,r4,_=run(src,td,'t4',4)
    if pathlib.Path(e1).read_bytes()!=pathlib.Path(e4).read_bytes():
        raise SystemExit('ABR parity elementary stream differs between 1 and 4 threads')
    if pathlib.Path(m1).read_bytes()!=pathlib.Path(m4).read_bytes():
        raise SystemExit('ABR parity M2TS differs between 1 and 4 threads')
    if pathlib.Path(r1).read_bytes()!=pathlib.Path(r4).read_bytes():
        raise SystemExit('ABR parity reconstruction differs between 1 and 4 threads')

print('0.1.72 ABR coding-law parity runtime regression: PASS')
