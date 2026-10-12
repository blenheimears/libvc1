// Native AArch64 NEON kernel parity tests. Run with CTest on AArch64;
// this file is not compiled on x86 and does not use any x86 intrinsics.
#include "simd.h"
#include "forward_transform_tables.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#if defined(__linux__) && defined(__aarch64__)
#include <sys/auxv.h>
#include <asm/hwcap.h>
#endif

using namespace libvc1;
static uint64_t sad(const uint8_t* a,int as,const uint8_t* b,int bs,int w,int h,int ss) {
    uint64_t sum=0;
    for(int y=0;y<h;y+=ss)for(int x=0;x<w;x+=ss)
        sum+=std::abs(int(a[y*as+x])-int(b[y*bs+x]));
    return ss>1?sum*uint64_t(ss)*ss:sum;
}
static uint64_t fsad(const uint8_t* a,int as,const uint8_t* b,int bs,int w,int h,int dx,int dy,int ss) {
    uint64_t sum=0;
    for(int y=0;y<h;y+=ss)for(int x=0;x<w;x+=ss)
        sum+=std::abs(int(a[y*as+x])-int(b[std::clamp(y+dy,0,h-1)*bs+std::clamp(x+dx,0,w-1)]));
    return ss>1?sum*uint64_t(ss)*ss:sum;
}
static void transform(const uint8_t* a,const uint8_t* b,int stride,int w,int h,double* out,bool intra) {
    double tmp[8][8]{},xf[8][8]{};
    for(int row=0;row<h;++row)for(int x=0;x<w;++x)for(int y=0;y<h;++y)
        tmp[row][x]+=(h==8?forward_transform::k8[row][y]:forward_transform::k4[row][y])*
          double(int(a[y*stride+x])-(intra?128:int(b[y*stride+x])));
    for(int row=0;row<h;++row)for(int col=0;col<w;++col)for(int x=0;x<w;++x)
        xf[row][col]+=tmp[row][x]*(w==8?forward_transform::k8[col][x]:forward_transform::k4[col][x]);
    std::fill(out,out+64,0.0);
    if(w==8 && h==8)for(int y=0;y<8;++y)for(int x=0;x<8;++x)out[y*8+x]=1024.0*xf[x][y];
    else for(int y=0;y<h;++y)for(int x=0;x<w;++x)out[y*8+x]=1024.0*xf[y][x];
}
static uint64_t satd(const uint8_t* a,int as,const uint8_t* b,int bs) {
    int d[4][4],t[4][4];
    for(int y=0;y<4;++y)for(int x=0;x<4;++x)d[y][x]=int(a[y*as+x])-int(b[y*bs+x]);
    for(int y=0;y<4;++y){int u=d[y][0]+d[y][3],v=d[y][1]+d[y][2],
       w=d[y][1]-d[y][2],z=d[y][0]-d[y][3];
       t[y][0]=u+v;t[y][1]=z+w;t[y][2]=u-v;t[y][3]=z-w;}
    uint64_t total=0;
    for(int x=0;x<4;++x){int u=t[0][x]+t[3][x],v=t[1][x]+t[2][x],
       w=t[1][x]-t[2][x],z=t[0][x]-t[3][x];
       total+=std::abs(u+v)+std::abs(z+w)+std::abs(u-v)+std::abs(z-w);}
    return (total+1)>>1;
}
static bool coeff_eq(const double* a,const double* b){
    for(int i=0;i<64;++i)if(std::abs(a[i]-b[i])>1e-7*std::max({1.0,std::abs(a[i]),std::abs(b[i])}))return false;
    return true;
}
// Independent reference for previously unaccelerated primitive families.
static int ref_asr(int v,int shift){return v>=0?v>>shift:-int((uint64_t(-int64_t(v))+((1ull<<shift)-1))>>shift);}
static int ref_filt(int a,int b,int c,int d,int mode){
    switch(mode){case 1:return -4*a+53*b+18*c-3*d;case 2:return -a+9*b+9*c-d;case 3:return -3*a+18*b+53*c-4*d;default:return b;}
}
static int ref_qpel(const uint8_t* p,int s,int hm,int vm,bool rnd){
    if(!hm&&!vm)return p[0];
    if(hm&&vm){const int shift=([&](){const int t[4]={0,5,1,5};return (t[hm]+t[vm])>>1;})();
        const int add=(1<<(shift-1))+(rnd?1:0)-1;int t[4];
        for(int k=0;k<4;++k)t[k]=ref_asr(ref_filt(p[k-1-s],p[k-1],p[k-1+s],p[k-1+2*s],vm)+add,shift);
        return std::clamp(ref_asr(ref_filt(t[0],t[1],t[2],t[3],hm)+64-(rnd?1:0),7),0,255);
    }
    if(vm)return std::clamp(ref_asr(ref_filt(p[-s],p[0],p[s],p[2*s],vm)+(vm==2?7+(rnd?1:0):31+(rnd?1:0)),vm==2?4:6),0,255);
    return std::clamp(ref_asr(ref_filt(p[-1],p[0],p[1],p[2],hm)+(hm==2?8-(rnd?1:0):32-(rnd?1:0)),hm==2?4:6),0,255);
}
static int ref_bilinear(const uint8_t* p,int s,bool fx,bool fy,bool rnd){
    if(!fx&&!fy)return p[0];if(fx&&fy)return (int(p[0])+p[1]+p[s]+p[s+1]+(rnd?1:2))>>2;
    return (int(p[0])+p[fx?1:s]+(rnd?0:1))>>1;
}
static int ref_chroma(const uint8_t* p,int s,int fx,int fy,bool rnd){
    if(!fx&&!fy)return p[0];return (int(p[0])*(8-fx)*(8-fy)+int(p[1])*fx*(8-fy)+int(p[s])*(8-fx)*fy+int(p[s+1])*fx*fy+(rnd?28:32))>>6;
}
static void ref_inverse(int* block){
    int temp[64]{};
    for(int col=0;col<8;++col){
        const int* s=block+col;int* d=temp+col*8;
        int a=12*(s[0]+s[32])+4,b=12*(s[0]-s[32])+4,c=16*s[16]+6*s[48],e=6*s[16]-16*s[48];
        int f=a+c,g=b+e,h=b-e,j=a-c;
        a=16*s[8]+15*s[24]+9*s[40]+4*s[56];b=15*s[8]-4*s[24]-16*s[40]-9*s[56];
        c=9*s[8]-16*s[24]+4*s[40]+15*s[56];e=4*s[8]-9*s[24]+15*s[40]-16*s[56];
        const int v[8]={f+a,g+b,h+c,j+e,j-e,h-c,g-b,f-a};for(int i=0;i<8;++i)d[i]=ref_asr(v[i],3);
    }
    for(int col=0;col<8;++col){
        const int* s=temp+col;
        int a=12*(s[0]+s[32])+64,b=12*(s[0]-s[32])+64,c=16*s[16]+6*s[48],e=6*s[16]-16*s[48];
        int f=a+c,g=b+e,h=b-e,j=a-c;
        a=16*s[8]+15*s[24]+9*s[40]+4*s[56];b=15*s[8]-4*s[24]-16*s[40]-9*s[56];
        c=9*s[8]-16*s[24]+4*s[40]+15*s[56];e=4*s[8]-9*s[24]+15*s[40]-16*s[56];
        const int v[8]={f+a,g+b,h+c,j+e,j-e+1,h-c+1,g-b+1,f-a+1};for(int i=0;i<8;++i)block[i*8+col]=ref_asr(v[i],7);
    }
}
static void ref_stats(const uint8_t* a,const uint8_t* p,int s,int w,int h,uint64_t* su,uint64_t* g,uint64_t* r,int* mn,int* mx,int* edges,int* gn){
    *su=*g=*r=0;*mn=255;*mx=0;*edges=*gn=0;
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){int v=a[y*s+x];*su+=v;*mn=std::min(*mn,v);*mx=std::max(*mx,v);
        if(p)*r+=std::abs(v-int(p[y*s+x]));
        if(x){int d=std::abs(v-int(a[y*s+x-1]));*g+=d;++*gn;*edges+=d>=96;}
        if(y){int d=std::abs(v-int(a[(y-1)*s+x]));*g+=d;++*gn;*edges+=d>=96;}
    }
}

int main(){
    std::mt19937 rng(0x41334364);
    constexpr int stride=79;
    std::vector<uint8_t> a(stride*71),b(stride*71),d(stride*71),ref(stride*71);
    for(int it=0;it<200;++it){
        for(auto& x:a)x=uint8_t(rng());
        for(auto& x:b)x=uint8_t(rng());
        const int w=1+int(rng()%71), h=1+int(rng()%71);
        for(int ss:{1,2,3,4,5,8,16}){
            if(sad(a.data(),stride,b.data(),stride,w,h,ss)!=simd::block_sad_aarch64_neon(a.data(),stride,b.data(),stride,w,h,ss))return 1;
            for(int dx:{-75,-17,-3,0,2,9,87})for(int dy:{-75,-1,0,4,87})
                if(fsad(a.data(),stride,b.data(),stride,w,h,dx,dy,ss)!=
                   simd::frame_sad_aarch64_neon(a.data(),stride,b.data(),stride,w,h,dx,dy,ss))return 2;
        }
        if(satd(a.data(),stride,b.data(),stride)!=simd::satd4x4_aarch64_neon(a.data(),stride,b.data(),stride))return 3;
        for(int width:{4,8})for(int height:{4,8}){
            double expected[64]{},actual[64]{};
            transform(a.data(),b.data(),stride,width,height,expected,false);
            simd::forward_transform_residual_rect_aarch64_neon(a.data(),b.data(),stride,width,height,actual,false);
            if(!coeff_eq(expected,actual))return 4;
            if(width==8&&height==8){
                simd::forward_transform_residual8_aarch64_neon(a.data(),b.data(),stride,actual,false);
                if(!coeff_eq(expected,actual))return 5;
                transform(a.data(),nullptr,stride,8,8,expected,true);
                simd::forward_transform_intra8_aarch64_neon(a.data(),stride,actual,false);
                if(!coeff_eq(expected,actual))return 6;
            }
            int blocks[64];for(int& x:blocks)x=int(rng()%8192)-4096;
            d=a;ref=a;
            simd::add_block_rect_aarch64_neon(d.data(),stride,blocks,width,height);
            for(int y=0;y<height;++y)for(int x=0;x<width;++x)
                ref[size_t(y)*stride+x]=uint8_t(std::clamp(int(ref[size_t(y)*stride+x])+blocks[y*8+x],0,255));
            if(d!=ref)return 7;
            if(width==8&&height==8){
                const int bias=int(rng()%512)-256;
                d=a;ref=a;
                simd::put_block8_aarch64_neon(d.data(),stride,blocks,bias);
                for(int y=0;y<8;++y)for(int x=0;x<8;++x)
                    ref[size_t(y)*stride+x]=uint8_t(std::clamp(blocks[y*8+x]+bias,0,255));
                if(d!=ref)return 8;
            }
        }
        for(size_t n:{size_t(0),size_t(1),size_t(15),size_t(16),size_t(17),size_t(1023),a.size()}){
            uint64_t sum=0;for(size_t i=0;i<n;++i)sum+=a[i];
            if(simd::sum_u8_aarch64_neon(a.data(),n)!=sum)return 9;
        }
    }
    // Exercise every newly registered NEON kernel, including 8-wide and scalar
    // tails, predictor rounding modes, chroma fractions and untouched borders.
    for(int it=0;it<45;++it){
        for(auto& v:a)v=uint8_t(rng());for(auto& v:b)v=uint8_t(rng());
        const uint8_t* ap=a.data()+2*stride+2;
        const uint8_t* bp=b.data()+2*stride+2;
        const int widths[5]={1,7,8,17,32}; const int w=widths[it%5];
        const int h=1+int(rng()%32);
        int inv[64],scalar_inv[64];for(int& v:inv)v=int(rng()%4096)-2048;
        std::copy(inv,inv+64,scalar_inv);ref_inverse(scalar_inv);
        simd::inverse_transform_8x8_aarch64_neon(inv);
        if(!std::equal(inv,inv+64,scalar_inv))return 10;
        for(bool rnd:{false,true})for(int hm=0;hm<4;++hm)for(int vm=0;vm<4;++vm){
            uint64_t qpel_sad=0,bilinear_sad=0;
            for(int y=0;y<h;++y)for(int x=0;x<w;++x){
                qpel_sad+=std::abs(int(ap[y*stride+x])-ref_qpel(bp+y*stride+x,stride,hm,vm,rnd));
                bilinear_sad+=std::abs(int(ap[y*stride+x])-ref_bilinear(bp+y*stride+x,stride,hm!=0,vm!=0,rnd));
            }
            if(simd::block_sad_luma_qpel_aarch64_neon(ap,stride,bp,stride,w,h,hm,vm,rnd)!=qpel_sad)return 11;
            if(simd::block_sad_luma_bilinear_aarch64_neon(ap,stride,bp,stride,w,h,hm!=0,vm!=0,rnd)!=bilinear_sad)return 12;
            for(bool bilinear:{false,true})for(bool avg:{false,true}){
                ref=a;d=a;
                uint8_t* rp=ref.data()+2*stride+2;
                uint8_t* dp=d.data()+2*stride+2;
                for(int y=0;y<h;++y)for(int x=0;x<w;++x){
                    const int pred=bilinear?ref_bilinear(bp+y*stride+x,stride,hm!=0,vm!=0,rnd):ref_qpel(bp+y*stride+x,stride,hm,vm,rnd);
                    rp[y*stride+x]=uint8_t(avg?(int(rp[y*stride+x])+pred+1)>>1:pred);
                }
                if(avg)simd::luma_mc_block_avg_aarch64_neon(dp,stride,bp,stride,w,h,hm,vm,rnd,bilinear);
                else simd::luma_mc_block_aarch64_neon(dp,stride,bp,stride,w,h,hm,vm,rnd,bilinear);
                if(d!=ref)return 13;
            }
        }
        for(bool rnd:{false,true})for(int fx:{0,1,2,3,4,5,7})for(int fy:{0,1,4,7})for(bool avg:{false,true}){
            ref=a;d=a;uint8_t* rp=ref.data()+2*stride+2;uint8_t* dp=d.data()+2*stride+2;
            for(int y=0;y<h;++y)for(int x=0;x<w;++x){const int pred=ref_chroma(bp+y*stride+x,stride,fx,fy,rnd);
                rp[y*stride+x]=uint8_t(avg?(int(rp[y*stride+x])+pred+1)>>1:pred);
            }
            if(avg)simd::chroma_mc_block_avg_aarch64_neon(dp,stride,bp,stride,w,h,fx,fy,rnd);
            else simd::chroma_mc_block_aarch64_neon(dp,stride,bp,stride,w,h,fx,fy,rnd);
            if(d!=ref)return 14;
        }
        for(int scale:{-64,0,1,33,71,95})for(int shift:{-4032,-192,0,8128}){
            const size_t n=(it*79)%a.size();ref=a;d=a;
            simd::intensity_map_plane_aarch64_neon(d.data(),a.data(),n,scale,shift);
            for(size_t i=0;i<n;++i)ref[i]=uint8_t(std::clamp((scale*int(a[i])+shift+32)>>6,0,255));
            if(d!=ref)return 15;
            ref=a;d=a;simd::intensity_map_chroma_plane_aarch64_neon(d.data(),a.data(),n,scale);
            for(size_t i=0;i<n;++i)ref[i]=uint8_t(std::clamp((scale*(int(a[i])-128)+128*64+32)>>6,0,255));
            if(d!=ref)return 16;
        }
        double co[64];int qs[64],qv[64];
        for(double& v:co)v=(int(rng()%65536)-32768)*0.03125;
        for(double scale:{1.0,7.0,18.0,35.0})for(double offset:{0.0,1.0,25.0}){
            for(int i=0;i<64;++i){const double mag=std::abs(co[i]);if(mag==0){qs[i]=0;continue;}
                int q=int(std::ceil((mag-offset)/scale-0.5));q=std::clamp(q,1,2047);
                const double err=mag-(q*scale+offset);qs[i]=(err*err<mag*mag)?(co[i]<0?-q:q):0;
            }
            simd::quantize_coefficients_aarch64_neon(co,qv,64,scale,offset,2047);
            if(!std::equal(qs,qs+64,qv))return 17;
        }
        for(bool has_pred:{false,true}){
            uint64_t su1,g1,r1,su2,g2,r2;int mn1,mx1,e1,n1,mn2,mx2,e2,n2;
            ref_stats(ap,has_pred?bp:nullptr,stride,w,h,&su1,&g1,&r1,&mn1,&mx1,&e1,&n1);
            simd::perceptual_stats_aarch64_neon(ap,has_pred?bp:nullptr,stride,w,h,&su2,&g2,&r2,&mn2,&mx2,&e2,&n2);
            if(su1!=su2||g1!=g2||r1!=r2||mn1!=mn2||mx1!=mx2||e1!=e2||n1!=n2)return 18;
        }
    }
    // Adversarial wide rows exercise accumulator flushes (no 16-bit wrap),
    // non-multiple-of-vector tails, and clamp/sample phase at both edges.
    {
        constexpr int ws=4111;
        std::vector<uint8_t> wide_a(ws*3),wide_b(ws*3);
        for(int pattern=0;pattern<3;++pattern){
            for(size_t i=0;i<wide_a.size();++i){
                wide_a[i]=pattern==0?0:pattern==1?255:uint8_t(rng());
                wide_b[i]=pattern==0?255:pattern==1?0:uint8_t(rng());
            }
            for(int ww:{1,15,16,17,31,32,1024,4097})for(int hh:{1,3})
                for(int ss:{1,2,3,4,5,8,16}){
                    const auto* aa=wide_a.data();const auto* bb=wide_b.data();
                    if(simd::block_sad_aarch64_neon(aa,ws,bb,ws,ww,hh,ss)!=sad(aa,ws,bb,ws,ww,hh,ss))return 19;
                    for(int dx:{-5000,-17,0,3,5000}){
                        if(simd::frame_sad_aarch64_neon(aa,ws,bb,ws,ww,hh,dx,-2,ss)!=
                           fsad(aa,ws,bb,ws,ww,hh,dx,-2,ss))return 20;
                    }
                }
            uint64_t expected=0;
            for(const auto v:wide_a)expected+=v;
            if(simd::sum_u8_aarch64_neon(wide_a.data(),wide_a.size())!=expected)return 21;
        }
    }
    // VC-1 2D filter intermediates must remain signed and unclipped even for
    // maximal edge contrast; test all quarter positions and both rounding flags.
    for(int pattern=0;pattern<4;++pattern){
        for(size_t i=0;i<a.size();++i){
            a[i]=pattern==0?0:pattern==1?255:pattern==2?uint8_t((i%stride)&1?255:0):uint8_t(rng());
            b[i]=pattern==0?255:pattern==1?0:pattern==2?uint8_t((i/stride)&1?255:0):uint8_t(rng());
        }
        const uint8_t* ap=a.data()+2*stride+2;
        const uint8_t* bp=b.data()+2*stride+2;
        for(bool rounding:{false,true})for(int hm=0;hm<4;++hm)for(int vm=0;vm<4;++vm){
            uint64_t expected=0;
            for(int y=0;y<16;++y)for(int x=0;x<16;++x)
                expected+=std::abs(int(ap[y*stride+x])-ref_qpel(bp+y*stride+x,stride,hm,vm,rounding));
            if(simd::block_sad_luma_qpel_aarch64_neon(ap,stride,bp,stride,16,16,hm,vm,rounding)!=expected)return 22;
            ref=a;d=a;
            uint8_t* rp=ref.data()+2*stride+2;uint8_t* dp=d.data()+2*stride+2;
            for(int y=0;y<16;++y)for(int x=0;x<16;++x)
                rp[y*stride+x]=uint8_t(ref_qpel(bp+y*stride+x,stride,hm,vm,rounding));
            simd::luma_mc_block_aarch64_neon(dp,stride,bp,stride,16,16,hm,vm,rounding,false);
            if(d!=ref)return 23;
        }
    }
    // Regression for the 0.2.54 four-byte SATD loads and the vectorized
    // horizontal Hadamard. Check minimum legal row stride and odd strides;
    // no vector read is allowed past the right edge of a 4-pixel row.
    for(int st:{4,5,7,79})for(int offset:{0,1,3})for(int pattern=0;pattern<5;++pattern){
        std::vector<uint8_t> q(4*st+offset+8,0xa5),z(q.size(),0x5a);
        for(int y=0;y<4;++y)for(int x=0;x<4;++x){
            q[offset+y*st+x]=uint8_t(pattern==0?0:pattern==1?255:pattern==2?((x+y)&1?0:255):pattern==3?x*85:rng());
            z[offset+y*st+x]=uint8_t(pattern==0?255:pattern==1?0:pattern==2?((x+y)&1?255:0):pattern==3?y*85:rng());
        }
        if(simd::satd4x4_aarch64_neon(q.data()+offset,st,z.data()+offset,st)!=
           satd(q.data()+offset,st,z.data()+offset,st))return 24;
    }
    // All byte values; short/misaligned/in-place/tail cases for both intensity
    // functions. This catches signed-shift and narrowing mistakes at 0/255.
    std::vector<uint8_t> src(4352),dst(4352),expected(4352);
    for(size_t i=0;i<src.size();++i)src[i]=uint8_t(i);
    for(int scale:{-64,0,1,33,64,71,95,128})for(size_t n:{size_t(0),size_t(1),size_t(7),size_t(8),size_t(15),size_t(16),size_t(17),size_t(31),size_t(32),size_t(33),size_t(255),size_t(256),size_t(257),size_t(4096)}){
        for(int offset:{0,1,3}){
            std::fill(dst.begin(),dst.end(),0xa5);expected=dst;
            for(size_t i=0;i<n;++i)expected[offset+i]=uint8_t(std::clamp((scale*(int(src[offset+i])-128)+8192+32)>>6,0,255));
            simd::intensity_map_chroma_plane_aarch64_neon(dst.data()+offset,src.data()+offset,n,scale);
            if(dst!=expected)return 25;
            dst=src;expected=src;
            for(size_t i=0;i<n;++i)expected[offset+i]=uint8_t(std::clamp((scale*(int(src[offset+i])-128)+8192+32)>>6,0,255));
            simd::intensity_map_chroma_plane_aarch64_neon(dst.data()+offset,dst.data()+offset,n,scale);
            if(dst!=expected)return 26;
        }
    }
    // Dedicated extension tests: execute ISA-specific objects only after Linux
    // HWCAP gates. Baseline NEON and independent scalar references remain legal.
#if defined(__linux__) && defined(__aarch64__)
    const unsigned long arm_hwcap=getauxval(AT_HWCAP);
#else
    const unsigned long arm_hwcap=0;
#endif
#if defined(LIBVC1_HAVE_AARCH64_DOTPROD) && defined(HWCAP_ASIMDDP)
    if ((arm_hwcap & HWCAP_ASIMDDP)!=0) {
        constexpr int stride_ext=79;
        std::vector<uint8_t> pixels(stride_ext*38), cur_ext(pixels.size()),actual_ext(pixels.size()),expected_ext(pixels.size());
        for(int pat=0;pat<5;++pat) {
            for(size_t i=0;i<pixels.size();++i) {
                pixels[i]=pat==0?0:pat==1?255:pat==2?uint8_t((i%stride_ext)&1?255:0):pat==3?uint8_t((i/stride_ext)&1?255:0):uint8_t(rng());
                cur_ext[i]=uint8_t(rng());
            }
            const uint8_t* srcp=pixels.data()+3*stride_ext+3;
            const uint8_t* cp=cur_ext.data()+3*stride_ext+3;
            for(int w:{1,7,8,9,15,16,17,24,32,64}) for(int h:{1,3,8,16})
            for(bool rnd:{false,true}) for(int hm=0;hm<4;++hm) for(int vm=0;vm<4;++vm) {
                uint64_t expected_sad=0;
                for(int y=0;y<h;++y)for(int x=0;x<w;++x)
                    expected_sad+=std::abs(int(cp[y*stride_ext+x])-ref_qpel(srcp+y*stride_ext+x,stride_ext,hm,vm,rnd));
                if(simd::block_sad_luma_qpel_aarch64_dotprod(cp,stride_ext,srcp,stride_ext,w,h,hm,vm,rnd)!=expected_sad) return 101;
                for(bool avg:{false,true}) {
                    expected_ext=cur_ext;actual_ext=cur_ext;
                    uint8_t* expectedp=expected_ext.data()+3*stride_ext+3;
                    uint8_t* actualp=actual_ext.data()+3*stride_ext+3;
                    for(int y=0;y<h;++y)for(int x=0;x<w;++x) {
                        const int pixel=ref_qpel(srcp+y*stride_ext+x,stride_ext,hm,vm,rnd);
                        expectedp[y*stride_ext+x]=uint8_t(avg?(int(expectedp[y*stride_ext+x])+pixel+1)>>1:pixel);
                    }
                    if(avg)simd::luma_mc_block_avg_aarch64_dotprod(actualp,stride_ext,srcp,stride_ext,w,h,hm,vm,rnd,false);
                    else simd::luma_mc_block_aarch64_dotprod(actualp,stride_ext,srcp,stride_ext,w,h,hm,vm,rnd,false);
                    if(expected_ext!=actual_ext) return 102;
                }
            }
            for(int w:{1,7,8,9,15,16,17,24,32,64}) for(int h:{1,3,8,16})
            for(bool rnd:{false,true})for(int fx=0;fx<8;++fx)for(int fy=0;fy<8;++fy)
            for(bool avg:{false,true}) {
                expected_ext=cur_ext;actual_ext=cur_ext;
                uint8_t* expectedp=expected_ext.data()+3*stride_ext+3;
                uint8_t* actualp=actual_ext.data()+3*stride_ext+3;
                for(int y=0;y<h;++y)for(int x=0;x<w;++x) {
                    const int pixel=ref_chroma(srcp+y*stride_ext+x,stride_ext,fx,fy,rnd);
                    expectedp[y*stride_ext+x]=uint8_t(avg?(int(expectedp[y*stride_ext+x])+pixel+1)>>1:pixel);
                }
                if(avg)simd::chroma_mc_block_avg_aarch64_dotprod(actualp,stride_ext,srcp,stride_ext,w,h,fx,fy,rnd);
                else simd::chroma_mc_block_aarch64_dotprod(actualp,stride_ext,srcp,stride_ext,w,h,fx,fy,rnd);
                if(expected_ext!=actual_ext) return 103;
            }
        }
        std::puts("AArch64 ASIMDDP: five primitives match independent scalar pixel references");
    } else std::puts("AArch64 ASIMDDP: not supported by host; native instruction tests skipped");
#endif
#if defined(LIBVC1_HAVE_AARCH64_RDM) && defined(HWCAP_ASIMDRDM)
    if ((arm_hwcap & HWCAP_ASIMDRDM)!=0) {
        std::vector<uint8_t> src(4352),actual(4352),expected(4352);
        for(size_t i=0;i<src.size();++i)src[i]=uint8_t(i);
        for(int scale:{-64,0,1,33,64,71,95,96,128})
        for(int shift:{-5001,-5000,-320,-128,-64,-63,0,64,128,192,8192})
        for(size_t n:{size_t(0),size_t(1),size_t(7),size_t(8),size_t(15),size_t(16),size_t(17),size_t(31),size_t(32),size_t(33),size_t(255),size_t(256),size_t(257),size_t(4096)})
        for(int offset:{0,1,3})for(bool in_place:{false,true}) {
            actual=src;expected=src;
            for(size_t i=0;i<n;++i)
                expected[offset+i]=uint8_t(std::clamp((scale*int(src[offset+i])+shift+32)>>6,0,255));
            simd::intensity_map_plane_aarch64_rdm(actual.data()+offset,in_place?actual.data()+offset:src.data()+offset,n,scale,shift);
            if(actual!=expected) return 104;
        }
        for(int scale:{-64,0,1,33,64,71,95,96,128})
        for(size_t n:{size_t(0),size_t(1),size_t(7),size_t(8),size_t(15),size_t(16),size_t(17),size_t(31),size_t(32),size_t(33),size_t(255),size_t(256),size_t(257),size_t(4096)})
        for(int offset:{0,1,3})for(bool in_place:{false,true}) {
            actual=src;expected=src;
            for(size_t i=0;i<n;++i)
                expected[offset+i]=uint8_t(std::clamp((scale*(int(src[offset+i])-128)+8192+32)>>6,0,255));
            simd::intensity_map_chroma_plane_aarch64_rdm(actual.data()+offset,in_place?actual.data()+offset:src.data()+offset,n,scale);
            if(actual!=expected) return 105;
        }
        std::puts("AArch64 ASIMDRDM: both intensity primitives match exact scalar rounding and saturation");
    } else std::puts("AArch64 ASIMDRDM: not supported by host; native instruction tests skipped");
#endif
    std::puts("AArch64 NEON: all 20 primitive families match scalar references (including boundary SATD and intensity cases)");
    return 0;
}
