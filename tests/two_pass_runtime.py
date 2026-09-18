#!/usr/bin/env python3
import csv
import pathlib
import subprocess
import sys
import tempfile

if len(sys.argv)!=2:
    raise SystemExit('usage: two_pass_runtime.py VC1ENC')
enc=sys.argv[1]
W,H,N=64,48,36

def write_y4m(path, mutate_frame=None):
    out=bytearray(f'YUV4MPEG2 W{W} H{H} F24:1 Ip A1:1 C420jpeg\n'.encode())
    for i in range(N):
        g=i//12
        y=bytearray(W*H)
        for yy in range(H):
            for xx in range(W):
                if g==0:
                    v=72+((xx*2+yy*3+i*2)&31)
                elif g==1:
                    v=40+(((xx//2+yy//2+i)&1)*150)+((xx*7+yy*5+i*17)&31)
                else:
                    v=32+((xx*29+yy*23+i*41+(xx*yy)%197)&191)
                if mutate_frame is not None and i==mutate_frame and xx==7 and yy==5:
                    v^=127
                y[yy*W+xx]=max(16,min(235,v))
        u=bytes(((x*3+y*5+i*7)&255) for y in range(H//2) for x in range(W//2))
        v=bytes(((x*7+y*3+i*9)&255) for y in range(H//2) for x in range(W//2))
        out += b'FRAME\n'+bytes(y)+u+v
    pathlib.Path(path).write_bytes(out)

def run(cmd, data=None, ok=True):
    p=subprocess.run(cmd,input=data,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    if ok and p.returncode:
        raise SystemExit((b'command failed: '+b' '.join(map(lambda x:str(x).encode(),cmd))+b'\n'+p.stderr).decode('utf-8','replace'))
    if (not ok) and p.returncode==0:
        raise SystemExit('command unexpectedly succeeded: '+' '.join(cmd))
    return p

with tempfile.TemporaryDirectory(prefix='libvc1-two-pass-') as d:
    td=pathlib.Path(d)
    src=td/'in.y4m'; badsrc=td/'bad.y4m'; stats=td/'encode.stats'
    write_y4m(src); write_y4m(badsrc, mutate_frame=5)
    common=['--bitrate','420k','--buffer-size','700k','--keyint','12','--threads','2','--fastest',
            '--search-range','4','--local-search-range','4','--no-scene-cut','--simd','none']
    data=src.read_bytes()
    run([enc,'-i','-','--pass','1','--pass-stats',str(stats),*common], data)
    if (td/'libvc1-pass1-virtual.m2ts').exists():
        raise SystemExit('pass 1 wrote default video output')
    text=stats.read_text()
    if not text.startswith('LIBVC1_TWO_PASS 2 ') or f'END {N} ' not in text:
        raise SystemExit('two-pass stats header/footer missing')
    lines=text.splitlines()
    frame_rows=[line.split() for line in lines if line.startswith('F ')]
    if len(frame_rows)!=N or any(len(row)!=13 for row in frame_rows):
        raise SystemExit('two-pass v2 statistics are missing measured Y/UV distortion')
    if not any(float(row[-2])>0 for row in frame_rows) or not any(float(row[-1])>0 for row in frame_rows):
        raise SystemExit('first-pass reconstruction MSE was not measured')
    if any(not (0<=float(value)<=65025) for row in frame_rows for value in row[-2:]):
        raise SystemExit('first-pass reconstruction MSE out of bounds')

    out=td/'out.vc1'; rc=td/'rc.csv'
    run([enc,'-i','-', '-o',str(out),'--pass','2','--pass-stats',str(stats),*common,'--rc-stats',str(rc)], data)
    if out.stat().st_size==0:
        raise SystemExit('pass 2 produced empty output')
    # Non-Blu-ray pass 2 must not use nominal level Rmax, an arbitrary VBV
    # buffer minimum, or the old per-picture / GOP actual-bit ceiling.
    high=td/'unrestricted.vc1'
    high_log=run([enc,'-i',str(src),'-o',str(high),'--pass','2',
                  '--pass-stats',str(stats),*common,'--bitrate','180M',
                  '--buffer-size','1']).stderr
    if not high.stat().st_size or b'peak=unrestricted, VBV=disabled' not in high_log or b'hrd-rate=0 bps' not in high_log:
        raise SystemExit('non-Blu-ray second-pass peak/HRD limitations remain enabled')
    if b'not be standards-conformant' not in high_log:
        raise SystemExit('nonconformant rate target warning missing')
    rows=list(csv.DictReader(rc.open()))
    if len(rows)!=N:
        raise SystemExit(f'expected {N} rc rows, got {len(rows)}')
    scales={round(float(r['two_pass_gop_scale']),6) for r in rows}
    weights={(round(float(r['effective_i_weight']),6),round(float(r['effective_p_weight']),6),round(float(r['effective_b_weight']),6)) for r in rows}
    if len(scales)<2:
        raise SystemExit('pass 2 did not vary GOP allocation')
    if len(weights)<2:
        raise SystemExit('pass 2 dynamic I/P/B weights did not vary')
    # Disabling class adaptation must keep global/per-picture budgeting active
    # while making the effective weights constant at the configured baseline.
    rc2=td/'rc-nodynamic.csv'
    run([enc,'-i',str(src),'-o',str(td/'out2.vc1'),'--pass','2','--pass-stats',str(stats),*common,'--no-dynamic-ipb-weights','--rc-stats',str(rc2)])
    rows2=list(csv.DictReader(rc2.open()))
    weights2={(round(float(r['effective_i_weight']),6),round(float(r['effective_p_weight']),6),round(float(r['effective_b_weight']),6)) for r in rows2}
    if weights2!={(5.0,1.0,0.7)}:
        raise SystemExit(f'--no-dynamic-ipb-weights not honored: {weights2}')

    p=run([enc,'-i',str(badsrc),'-o',str(td/'bad.vc1'),'--pass','2','--pass-stats',str(stats),*common], ok=False)
    if b'source frame differs' not in p.stderr:
        raise SystemExit('source fingerprint mismatch was not diagnosed')
    p=run([enc,'-i',str(src),'-o',str(td/'badkey.vc1'),'--pass','2','--pass-stats',str(stats),*common,'--keyint','10'], ok=False)
    if b'configuration mismatch' not in p.stderr and b'layout differs' not in p.stderr:
        raise SystemExit('configuration/layout mismatch was not diagnosed')
    broken=td/'broken.stats'; broken.write_text('\n'.join(text.splitlines()[:-1])+'\n')
    p=run([enc,'-i',str(src),'-o',str(td/'broken.vc1'),'--pass','2','--pass-stats',str(broken),*common], ok=False)
    if b'missing END footer' not in p.stderr:
        raise SystemExit('incomplete stats file was not rejected')
    # A legacy first-pass file still works, but uses legacy planning instead of
    # hallucinating missing distortion data.
    legacy=td/'legacy.stats'
    legacy.write_text('\n'.join(
        ('LIBVC1_TWO_PASS 1 '+line.split(' ',2)[2]) if line.startswith('LIBVC1_TWO_PASS ') else
        (' '.join(line.split()[:-2])) if line.startswith('F ') else line
        for line in lines)+'\n')
    run([enc,'-i','-', '-o',str(td/'legacy.vc1'),'--pass','2','--pass-stats',str(legacy),*common], data)
    # Reject missing, non-finite and out-of-range measured distortion rather
    # than allowing NaNs into the budget water-fill.
    for badvalue in ('nan','-1','65026'):
        invalid=td/('invalid-'+badvalue+'.stats')
        altered=lines.copy()
        first=frame_rows[0].copy()
        first[-1]=badvalue
        altered[1]=' '.join(first)
        invalid.write_text('\n'.join(altered)+'\n')
        p=run([enc,'-i',str(src),'-o',str(td/'badstats.vc1'),'--pass','2','--pass-stats',str(invalid),*common],ok=False)
        if b'invalid or out-of-order two-pass picture statistics' not in p.stderr:
            raise SystemExit('invalid distortion not diagnosed: '+badvalue)

print('two-pass runtime passed')
