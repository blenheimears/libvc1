#!/usr/bin/env python3
import csv, pathlib, subprocess, sys, tempfile

enc = pathlib.Path(sys.argv[1])
with tempfile.TemporaryDirectory(prefix='libvc1-debug-stats-') as td:
    td = pathlib.Path(td)
    y4m = td/'in.y4m'
    w=h=32
    with y4m.open('wb') as f:
        f.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
        for frame in range(4):
            f.write(b'FRAME\n')
            y=bytearray(w*h)
            for yy in range(h):
                for xx in range(w):
                    y[yy*w+xx]=48 + (((xx//4)+(frame//2))&1)*144
            f.write(y)
            f.write(bytes([128])*((w//2)*(h//2)))
            f.write(bytes([128])*((w//2)*(h//2)))
    common=[str(enc),'-i',str(y4m),'--cq','8','--intra-only','--threads','1','--simd','scalar']
    plain=td/'plain.m2ts'; dbg=td/'dbg.m2ts'; stats=td/'stats.csv'
    subprocess.run(common+['-o',str(plain)],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.run(common+['-o',str(dbg),'--debug-stats',str(stats)],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    if plain.read_bytes()!=dbg.read_bytes():
        raise SystemExit('debug statistics changed encoded output')
    with stats.open(newline='') as f:
        header=f.readline().strip().split(',')
        stable=['display_order','coded_order','gop_index','frame_in_gop','gop_frames','type','keyframe','keyframe_reason']
        if header[:len(stable)] != stable:
            raise SystemExit('debug statistics stable prefix changed: '+','.join(header[:len(stable)]))
        f.seek(0)
        rows=list(csv.DictReader(f))
    if len(rows)!=4:
        raise SystemExit(f'expected 4 debug rows, got {len(rows)}')
    required={'display_order','gop_index','keyframe_reason','profile','rc_mode','qp','quant_step','qscale',
              'final_actual_bits','encode_trials','p_four_mv_mb','p_intra_mb','dquant_mb','mquant_min','mquant_max','mquant_mean','ttmbf','ttfrm','ttfrm_exact_checked','acpred_mb','mse_y','snr_y_db','psnr_y_db','psnr_yuv_db','bits_per_pixel'}
    missing=required-set(rows[0])
    if missing:
        raise SystemExit('missing debug columns: '+','.join(sorted(missing)))
    if [int(r['display_order']) for r in rows] != list(range(4)):
        raise SystemExit('debug rows not written in display order')
    if not all(r['type']=='I' and int(r['qp'])==8 and int(r['encode_trials'])>=1 for r in rows):
        raise SystemExit('debug coding decisions are incomplete')
    if not all(float(r['psnr_y_db'])>0.0 and int(r['final_actual_bits'])>0 for r in rows):
        raise SystemExit('debug reconstruction-quality fields are incomplete')
print('debug statistics CLI ok')
