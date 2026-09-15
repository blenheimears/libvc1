#!/usr/bin/env python3
import argparse
from pathlib import Path

START=b'\x00\x00\x01'

def payload(data, code):
    marker=START+bytes([code])
    p=data.find(marker)
    if p<0:
        raise SystemExit(f'missing start code 0x{code:02x}')
    p+=4
    q=data.find(START,p)
    if q<0: q=len(data)
    src=data[p:q]
    out=bytearray(); z=0; i=0
    while i<len(src):
        b=src[i]
        if z>=2 and b==3:
            i+=1
            z=0
            continue
        out.append(b)
        z=z+1 if b==0 else 0
        i+=1
    return bytes(out)

class BR:
    def __init__(self,b): self.b=b; self.n=0
    def get(self,n):
        v=0
        for _ in range(n):
            if self.n>=len(self.b)*8: raise SystemExit('truncated VC-1 header')
            x=self.b[self.n>>3]
            v=(v<<1)|((x>>(7-(self.n&7)))&1)
            self.n+=1
        return v

def parse_sequence(data):
    r=BR(payload(data,0x0f))
    profile=r.get(2); level=r.get(3); colordiff=r.get(2)
    r.get(3); r.get(5); r.get(1); r.get(12); r.get(12)
    # PULLDOWN, INTERLACE, TFCNTRFLAG, FINTERPFLAG, reserved, PSF.
    for _ in range(6): r.get(1)
    display_ext=r.get(1)
    if display_ext:
        r.get(14); r.get(14)
        if r.get(1):  # ASPECT_RATIO_FLAG
            ar=r.get(4)
            if ar==15: r.get(8); r.get(8)
        if r.get(1):  # FRAMERATE_FLAG
            if r.get(1): r.get(16)  # FRAMERATEIND=1 -> FRAMERATEEXP
            else: r.get(8); r.get(4) # FRAMERATENR / FRAMERATEDR
        if r.get(1):  # COLOR_FORMAT_FLAG
            r.get(8); r.get(8); r.get(8)
    hrd=r.get(1)
    out={'profile':profile,'level':level,'colordiff':colordiff,'hrd':hrd}
    if hrd:
        n=r.get(5); bre=r.get(4); bse=r.get(4)
        buckets=[]
        for _ in range(n):
            rm=r.get(16); bm=r.get(16)
            buckets.append(((rm+1)<<(bre+6),(bm+1)<<(bse+4),rm,bm))
        out.update(n=n,bre=bre,bse=bse,buckets=buckets)
    return out

def parse_entry_full(data, hrd):
    r=BR(payload(data,0x0e))
    r.get(1); r.get(1); r.get(1); r.get(1); r.get(1); r.get(1); r.get(1)
    r.get(2); r.get(1); r.get(1); r.get(2)
    return r.get(8) if hrd else None

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('vc1')
    ap.add_argument('--expect-rate',type=int)
    ap.add_argument('--expect-buffer',type=int)
    ap.add_argument('--expect-full',type=int)
    ap.add_argument('--expect-no-hrd',action='store_true')
    a=ap.parse_args()
    data=Path(a.vc1).read_bytes()
    s=parse_sequence(data)
    if (s['profile'],s['level'],s['colordiff'])!=(3,3,1):
        raise SystemExit(f'unexpected sequence identity: {s}')
    if a.expect_no_hrd:
        if s['hrd']!=0: raise SystemExit('HRD unexpectedly present')
        print('HRD absent as expected')
        return
    if s['hrd']!=1 or s.get('n')!=1:
        raise SystemExit(f'expected one HRD leaky bucket: {s}')
    rate,buf,_,_=s['buckets'][0]
    if a.expect_rate is not None and rate!=a.expect_rate:
        raise SystemExit(f'HRD rate {rate} != {a.expect_rate}')
    if a.expect_buffer is not None and buf!=a.expect_buffer:
        raise SystemExit(f'HRD buffer {buf} != {a.expect_buffer}')
    full=parse_entry_full(data,True)
    if a.expect_full is not None and full!=a.expect_full:
        raise SystemExit(f'HRD_FULL {full} != {a.expect_full}')
    print(f'HRD rate={rate} buffer={buf} HRD_FULL={full}')

if __name__=='__main__': main()
