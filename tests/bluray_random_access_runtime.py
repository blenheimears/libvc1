#!/usr/bin/env python3
import pathlib, subprocess, sys, tempfile

enc = sys.argv[1]


def start_codes(data: bytes):
    out=[]
    i=0
    while True:
        p=data.find(b'\x00\x00\x01', i)
        if p < 0 or p+3 >= len(data):
            return out
        out.append(data[p+3])
        i=p+4

with tempfile.TemporaryDirectory(prefix='libvc1-bd-ra-') as td:
    td=pathlib.Path(td)
    y4m=td/'src.y4m'
    w,h=1280,720
    y=bytes([96])*(w*h)
    c=bytes([128])*((w//2)*(h//2))
    with y4m.open('wb') as f:
        f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for n in range(6):
            f.write(b'FRAME\n')
            # Slight luma changes keep the fixture from being treated as an exact duplicate.
            f.write(bytes([(96+n)&0xff])*(w*h)); f.write(c); f.write(c)

    es=td/'out.vc1'
    out=td/'out.m2ts'
    cmd=[enc,'-i',str(y4m),'-o',str(out),'--es-out',str(es),
         '--bluray-compat','--keyint','2','--bframes','0','--fixed-gop-grid','--cq','31',
         '--threads','1','--simd','none','--search-range','0','--local-search-range','0',
         '--no-aq','--no-loop-filter','--fixed-8x8','--trellis','0','--no-scene-cut']
    p=subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if p.returncode:
        raise SystemExit('Blu-ray random-access encode failed:\n'+p.stderr)

    codes=start_codes(es.read_bytes())
    relevant=[c for c in codes if c in (0x0f,0x0e,0x0d)]
    expected=[0x0f,0x0e,0x0d,0x0d]*3
    if relevant != expected:
        raise SystemExit(f'Blu-ray access units are not repeated SEQUENCE/ENTRYPOINT/FRAME triplets: {relevant!r}')

print('Blu-ray repeated random-access sequence/entry-point signaling: PASS')
