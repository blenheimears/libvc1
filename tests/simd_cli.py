#!/usr/bin/env python3
import math, os, pathlib, platform, subprocess, sys, tempfile

enc=sys.argv[1]
GENERIC=('x86-64-v1','x86-64-v2','x86-64-v3','x86-64-v4')
ARM=('aarch64-neon','aarch64-dotprod','aarch64-rdm')
MIDS=('prescott','k10','conroe','penryn','sandybridge','bulldozer','piledriver','avx2-partial')
LABEL={'x86-64-v1':'X86-64-V1','prescott':'PRESCOTT','k10':'K10','conroe':'CONROE','penryn':'PENRYN',
       'x86-64-v2':'X86-64-V2','sandybridge':'SANDYBRIDGE','bulldozer':'BULLDOZER',
       'piledriver':'PILEDRIVER','avx2-partial':'AVX2-PARTIAL','x86-64-v3':'X86-64-V3','x86-64-v4':'X86-64-V4','aarch64-neon':'AARCH64-NEON',
       'aarch64-dotprod':'AARCH64-DOTPROD','aarch64-rdm':'AARCH64-RDM'}

def run(cmd,env):
    return subprocess.run(cmd,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)

def parse_table(log):
    rows=[line for line in log.splitlines() if line.startswith('MODE') or line.startswith('[auto]') or line.startswith('[forced]')]
    if not rows: raise SystemExit('missing SIMD benchmark table')
    w=len(rows[0]); bad=[(r,len(r)) for r in rows if len(r)!=w]
    if bad: raise SystemExit(f'SIMD benchmark rows are not column-aligned: header={w}, mismatches={bad}')
    hdr=rows[0].split()
    if hdr[:3]!=['MODE','PRIMITIVE','NONE'] or hdr[-2:]!=['SELECTED','SELECTED%']:
        raise SystemExit(f'bad SIMD table header: {rows[0]}')
    labels=hdr[3:-2]
    targets=[]
    for label in labels:
        target=label.lower()
        if target not in LABEL or LABEL[target]!=label:
            raise SystemExit(f'unknown SIMD target column {label}')
        if '/NONE' in label: raise SystemExit(f'redundant /NONE suffix in compact header {label}')
        targets.append(target)
    data={}; gm=None
    expected_fields=3+len(targets)+2
    for line in rows[1:]:
        f=line.split()
        if len(f)!=expected_fields: raise SystemExit(f'bad SIMD row field count {len(f)} != {expected_fields}: {line}')
        mode,name,none=f[0],f[1],float(f[2]); off=3; pcts={}
        for target in targets:
            pct=f[off]; off+=1
            if pct!='n/a': pcts[target]=float(pct[:-1])
        selected,selectedpct=f[off],f[off+1]
        if name=='geometric-mean': gm=(none,pcts,selected,selectedpct); continue
        data[name]=(mode,pcts,selected,selectedpct)
        selected_base=selected.removesuffix('+fma3').removesuffix('+fma4')
        sp=pcts.get(selected_base)
        if sp is not None:
            if selectedpct=='n/a' or abs(float(selectedpct[:-1])-sp)>0.11:
                raise SystemExit(f'{name}: selected percentage {selectedpct} != target column {sp:.1f}%')
        elif selectedpct!='n/a' and selected_base!='none':
            raise SystemExit(f'{name}: selected unavailable target has {selectedpct}')
        if selected_base=='none' and selectedpct!='100.0%':
            raise SystemExit(f'{name}: none selection should report 100.0%, got {selectedpct}')
        if mode=='[auto]':
            best='none'; bestpct=100.0
            for t,pctv in pcts.items():
                if pctv>bestpct+0.02: best,bestpct=t,pctv
            if selected_base!=best and abs(bestpct-(pcts.get(selected_base,100.0 if selected_base=='none' else 0.0)))>0.12:
                raise SystemExit(f'{name}: selected {selected_base}, printed fastest {best}: {pcts}')
    if len(data)!=20 or gm is None: raise SystemExit(f'expected 20 primitives plus geometric mean, found {len(data)}')
    # The summary must use paired ratios, including for sparse ARM extensions.
    # Individual rows are printed to 0.1%, so allow accumulated rounding.
    for target in targets:
        ratios=[pcts[target]/100.0 for _,pcts,_,_ in data.values() if target in pcts]
        got=gm[1].get(target)
        if not ratios:
            if got is not None: raise SystemExit(f'{target}: summary must be n/a without coverage')
            continue
        expected=100.0*math.exp(sum(math.log(v) for v in ratios)/len(ratios))
        if got is None or abs(got-expected)>0.2:
            raise SystemExit(f'{target}: geometric mean must use paired ratios: got={got}, expected={expected:.2f} ({len(ratios)} primitives)')
    selected_ratios=[float(pct[:-1])/100.0 for _,_,_,pct in data.values() if pct!='n/a']
    selected_gmean=100.0*math.exp(sum(math.log(v) for v in selected_ratios)/len(selected_ratios))
    if gm[3]=='n/a' or abs(float(gm[3][:-1])-selected_gmean)>0.2:
        raise SystemExit(f'selected geometric mean incorrect: {gm[3]} vs {selected_gmean:.2f}')
    return targets,data,gm,w

with tempfile.TemporaryDirectory() as td:
    td=pathlib.Path(td); src=td/'one.y4m'; out=td/'one.m2ts'; w=h=16
    src.write_bytes(f'YUV4MPEG2 W{w} H{h} F24:1 Ip A1:1 C420jpeg\nFRAME\n'.encode()+bytes([96])*(w*h)+bytes([128])*(w*h//2))
    env=os.environ.copy(); env['LIBVC1_SIMD_BENCHMARK_MS']='100'
    base=[enc,'-i',str(src),'--max-frames','1','--cq','8','--threads','1']

    # Determine compatibility through the public force path.  This does not use CPU names/models.
    supported={}
    for target in GENERIC+MIDS+ARM:
        rr=run(base+['-o',str(td/f'force-{target}.m2ts'),'--simd',target],env)
        supported[target]=(rr.returncode==0)
        if not supported[target] and not any(err in rr.stderr for err in ('unavailable on this CPU/build', 'reserved for a future kernel implementation')):
            raise SystemExit(f'{target}: expected success or clean flag/build rejection: {rr.stderr}')

    cmd=base+['-o',str(out),'--simd','auto','--simd-primitive','frame-sad=v4,block-sad=scalar',
              '--simd-primitive','frame-sad=auto,sum-u8=none']
    r=run(cmd,env)
    if r.returncode: raise SystemExit(r.stderr)
    if 'SIMD auto benchmark per primitive (none=units/s; SIMD columns=% of none):' not in r.stderr:
        raise SystemExit('missing detailed SIMD benchmark header')
    targets,data,gm,width=parse_table(r.stderr)
    # Cross-check the optional ARM-only paired comparison on the exact same
    # supported primitive subset, not on the scalar 20-primitive aggregate.
    for extension in ('aarch64-dotprod','aarch64-rdm'):
        matches=[line for line in r.stderr.splitlines() if line.startswith(
            f'vc1enc: ARM {LABEL[extension]} paired vs NEON:')]
        pairs=[(pcts[extension],pcts['aarch64-neon'])
               for _,pcts,_,_ in data.values()
               if extension in pcts and 'aarch64-neon' in pcts]
        if extension not in targets:
            if matches: raise SystemExit(f'{extension}: unavailable extension comparison printed')
            continue
        if len(matches)!=1: raise SystemExit(f'{extension}: expected exactly one NEON comparison: {matches}')
        if not pairs:
            if not matches[0].endswith('n/a'): raise SystemExit(f'{extension}: missing baseline comparison must be n/a')
            continue
        expected=100.0*math.exp(sum(math.log(a/b) for a,b in pairs)/len(pairs))
        import re
        match=re.search(r': ([0-9]+\.[0-9]+)% \(([0-9]+) primitives\)',matches[0])
        if match is None or abs(float(match.group(1))-expected)>0.2 or int(match.group(2))!=len(pairs):
            raise SystemExit(f'{extension}: incorrect paired NEON summary: {matches[0]}, expected {expected:.2f}% ({len(pairs)} primitives)')
    host_arch=platform.machine().lower()
    x86_host=host_arch in ('x86_64', 'amd64')
    arm_host=host_arch in ('aarch64', 'arm64')
    # Generic x86 columns intentionally remain on x86 even when their SIMD
    # kernels were not compiled; they must not leak into ARM benchmark output.
    if x86_host:
        for g in GENERIC:
            if g not in targets: raise SystemExit(f'default AUTO omitted x86 generic column {g}')
    elif any(g in targets for g in GENERIC):
        raise SystemExit(f'non-x86 host displayed x86 generic columns: {targets}')
    if supported['aarch64-neon']:
        if not arm_host: raise SystemExit('NEON force succeeded on a non-AArch64 host')
        expected_arm=[t for t in ARM if supported[t]]
        if targets!=expected_arm:
            raise SystemExit(f'ARM AUTO columns {targets}, expected {expected_arm}')
        if any('aarch64-neon' not in pcts for _,pcts,_,_ in data.values()):
            raise SystemExit('NEON coverage or benchmark missing for a registered primitive')
    elif any(t in targets for t in ARM):
        raise SystemExit('unavailable ARM target appeared in AUTO columns')
    if arm_host and any(t in targets for t in GENERIC+MIDS):
        raise SystemExit(f'ARM AUTO displayed x86-specific columns: {targets}')
    expected_mids=[t for t in MIDS if supported[t]]
    shown_mids=[t for t in targets if t in MIDS]
    if shown_mids!=expected_mids: raise SystemExit(f'default AUTO intermediate columns {shown_mids}, expected all compatible {expected_mids}')
    if data['frame-sad'][0]!='[auto]': raise SystemExit('later frame-sad=auto did not override earlier assignment')
    if data['block-sad'][0]!='[forced]' or data['block-sad'][2].removesuffix('+fma3').removesuffix('+fma4')!='none': raise SystemExit('scalar override failed')
    if data['sum-u8'][0]!='[forced]' or data['sum-u8'][2].removesuffix('+fma3').removesuffix('+fma4')!='none': raise SystemExit('none override failed')
    if not out.exists() or out.stat().st_size==0: raise SystemExit('encode produced no output')

    # --benchmark-all is now the default and must be output-equivalent in policy.
    allout=td/'all.m2ts'
    rr=run(base+['-o',str(allout),'--simd','auto','--benchmark-all'],env)
    if rr.returncode: raise SystemExit(rr.stderr)
    allt,alldata,allgm,allwidth=parse_table(rr.stderr)
    if allt!=targets:
        raise SystemExit(f'--benchmark-all columns {allt} differ from AUTO {targets}')
    if [t for t in allt if t in MIDS] != expected_mids:
        raise SystemExit('--benchmark-all differs from the default all-compatible policy')
    for fma4_target in ('bulldozer','piledriver'):
        if not supported[fma4_target] and fma4_target in allt:
            raise SystemExit(f'--benchmark-all exposed unavailable FMA4 target {fma4_target}')

    # Opt-out restores the previous gap-only intermediate policy.
    selective=td/'selective.m2ts'
    rr=run(base+['-o',str(selective),'--simd','auto','--benchmark-selective'],env)
    if rr.returncode: raise SystemExit(rr.stderr)
    st,_,_,_=parse_table(rr.stderr)
    eligible=[]
    if supported['prescott'] and not supported['conroe']: eligible.append('prescott')
    if supported['k10'] and not supported['x86-64-v2'] and not supported['bulldozer']: eligible.append('k10')
    if supported['conroe'] and not supported['penryn']: eligible.append('conroe')
    if supported['penryn'] and not supported['x86-64-v2']: eligible.append('penryn')
    if supported['sandybridge'] and not supported['avx2-partial'] and not supported['x86-64-v3']: eligible.append('sandybridge')
    if supported['bulldozer'] and not supported['piledriver']: eligible.append('bulldozer')
    if supported['piledriver']: eligible.append('piledriver')
    if supported['avx2-partial'] and not supported['x86-64-v3']: eligible.append('avx2-partial')
    shown=[t for t in st if t in MIDS]
    if shown!=eligible: raise SystemExit(f'--benchmark-selective intermediate columns {shown}, expected gap-only {eligible}')
    if supported['aarch64-neon'] and st!=[t for t in ARM if supported[t]]:
        raise SystemExit(f'--benchmark-selective ARM targets incorrect or exposed x86 columns: {st}')

    # Named per-primitive overrides are implemented now: success when compatible,
    # otherwise they must fail with the same flag/build availability diagnostic.
    for target in MIDS+('aarch64-neon',):
        rr=run(base+['-o',str(td/f'override-{target}.m2ts'),'--simd','none','--simd-primitive',f'frame-sad={target}'],env)
        if supported[target]:
            if rr.returncode: raise SystemExit(f'{target} override unexpectedly failed: {rr.stderr}')
        elif rr.returncode==0 or 'unavailable for primitive frame-sad' not in rr.stderr:
            raise SystemExit(f'{target}: expected clean unavailable override rejection: {rr.stderr}')
    # Specifically regress 0.2.51's newly supported inverse8x8 and SATD4x4.
    if supported['aarch64-neon']:
        for primitive in ('inverse8x8', 'satd4x4'):
            rr=run(base+['-o',str(td/f'neon-{primitive}.m2ts'),'--simd','none',
                   '--simd-primitive',f'{primitive}=aarch64-neon'],env)
            if rr.returncode:
                raise SystemExit(f'{primitive}: NEON override unexpectedly failed: {rr.stderr}')

    # Partial ARM instruction sets are selectable only for their actual kernels.
    for target, primitive, unsupported in (('aarch64-dotprod','luma-mc','frame-sad'),
                                           ('aarch64-rdm','intensity-luma','block-sad')):
        rr=run(base+['-o',str(td/f'extension-{target}.m2ts'),'--simd','auto',
                    '--simd-primitive',f'{primitive}={target}'],env)
        if supported[target]:
            if rr.returncode: raise SystemExit(f'{target}: supported primitive refused: {rr.stderr}')
        elif rr.returncode==0 or not any(err in rr.stderr for err in ('unavailable for primitive', 'reserved for a future kernel implementation')):
            raise SystemExit(f'{target}: unavailable primitive accepted: {rr.stderr}')
        rr=run(base+['-o',str(td/f'bad-extension-{target}.m2ts'),'--simd','auto',
                    '--simd-primitive',f'{unsupported}={target}'],env)
        if rr.returncode==0 or not any(err in rr.stderr for err in ('unavailable for primitive', 'reserved for a future kernel implementation', 'has no kernel for primitive')):
            raise SystemExit(f'{target}: unsupported primitive accepted: {rr.stderr}')
