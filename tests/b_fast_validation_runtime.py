#!/usr/bin/env python3
"""Regression for B picture search-cache and vectorized prediction refactoring.

An optional 0.2.68 reference executable verifies byte-for-byte preservation.
"""
from pathlib import Path
import subprocess, sys, tempfile, hashlib, re, time

new = str(Path(sys.argv[1]).resolve())
old = str(Path(sys.argv[2]).resolve()) if len(sys.argv)>2 else None

with tempfile.TemporaryDirectory(prefix='libvc1-b-fast-') as td:
    root=Path(td)
    for shape, w, h in [('aligned',64,48), ('odd',80,50)]:
        src=root/(shape+'.y4m')
        with src.open('wb') as out:
            out.write(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\n'.encode())
            for t in range(10):
                out.write(b'FRAME\n')
                out.write(bytes(((17 + 7*((x+t*3)%w) + 3*y + ((x*y+t*47)%83))%230)+12 for y in range(h) for x in range(w)))
                out.write(bytes((41 + x*3 + y*2 + t*7)%240 for y in range(h//2) for x in range(w//2)))
                out.write(bytes((77 + y*7 + x + t*11)%240 for y in range(h//2) for x in range(w//2)))
    tests=[('advanced-aligned-fast','aligned',['--fast']),
           ('advanced-odd-fast','odd',['--fast']),
           ('advanced-odd-normal','odd',['--search-range','32','--local-search-range','8','--no-trellis']),
           ('main-odd','odd',['--search-range','32','--local-search-range','8','--no-trellis'])]
    for name,shape,options in tests:
        inp=root/(shape+'.y4m')
        def run(label,exe,extra):
            dest=root/(name+label+('.wmv' if name.startswith('main-') else '.m2ts'))
            args=[exe,'-i',str(inp),'-o',str(dest),'--cq','8','--threads','1','--simd','none','--no-scene-cut',*extra]
            start=time.perf_counter()
            p=subprocess.run(args,capture_output=True,timeout=150)
            duration=time.perf_counter()-start
            if p.returncode:
                raise AssertionError((name,label,p.stderr.decode(errors='replace')[-2500:]))
            return dest.read_bytes(),p.stderr.decode(errors='replace'),duration
        updated,log,elapsed=run('new',new,options)
        bmatch=re.search(r'\bB=(\d+)',log)
        assert bmatch and int(bmatch.group(1))>=2, (name,log[-800:])
        if old:
            reference,old_log,old_elapsed=run('old',old,[*options,'--bframes','2'] if '--fast' in options else options)
            assert updated==reference, (name,'changed output',hashlib.sha256(updated).hexdigest(),hashlib.sha256(reference).hexdigest())
            print(f'{name}: BIT-EXACT, new={elapsed:.3f}s baseline={old_elapsed:.3f}s',flush=True)
        else:
            print(f'{name}: PASS, B={bmatch.group(1)}, time={elapsed:.3f}s',flush=True)
