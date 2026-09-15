#!/usr/bin/env python3
import pathlib, subprocess, sys, tempfile

if len(sys.argv)!=2:
    raise SystemExit('usage: speed_profile_runtime.py VC1ENC')
enc=sys.argv[1]
with tempfile.TemporaryDirectory(prefix='libvc1-speed-profile-') as d:
    td=pathlib.Path(d); src=td/'src.y4m'; a=td/'a.m2ts'; b=td/'b.m2ts'; report=td/'speed.txt'
    w,h=64,48; cw,ch=w//2,h//2
    uv=bytes([128])*(cw*ch)
    data=bytearray(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for n in range(12):
        y=bytes((24+((x*5+y*3+n*11)&191)) for y in range(h) for x in range(w))
        data += b'FRAME\n'+y+uv+uv
    src.write_bytes(data)
    common=[enc,'-i',str(src),'--cq','8','--bframes','2','--simd','none','--threads','1','--max-frames','12']
    p=subprocess.run(common+['-o',str(a)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if p.returncode: raise SystemExit('baseline encode failed:\n'+p.stderr)
    p=subprocess.run(common+['-o',str(b),'--speed-profile',str(report)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if p.returncode: raise SystemExit('profiled encode failed:\n'+p.stderr)
    if a.read_bytes()!=b.read_bytes():
        raise SystemExit('enabling speed profiling changed the encoded output')
    text=report.read_text()
    required=['libvc1 aggregate speed profile v1','input_frames=12','No per-frame or per-macroblock timing samples are stored.',
              'transform RDO','trellis quantization','P-picture analysis','GOP encode work total:']
    for token in required:
        if token not in text: raise SystemExit('speed report missing: '+token)
    lines=text.splitlines()
    if len(lines)>80 or report.stat().st_size>16384:
        raise SystemExit(f'speed report unexpectedly large: {len(lines)} lines, {report.stat().st_size} bytes')
print('aggregate speed profiling summary + bitstream invariance: PASS')
