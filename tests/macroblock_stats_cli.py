#!/usr/bin/env python3
import os, subprocess, sys, tempfile
from mbstats_v2 import read_macroblock_stats, frame_value

if len(sys.argv) != 2:
    raise SystemExit('usage: macroblock_stats_cli.py /path/to/vc1enc')
enc=sys.argv[1]
W=32; H=32; N=8

def make_y4m(path, interlace='p'):
    with open(path,'wb') as f:
        f.write(f'YUV4MPEG2 W{W} H{H} F24:1 A1:1 C420jpeg I{interlace}\n'.encode())
        for n in range(N):
            f.write(b'FRAME\n')
            y=bytearray(W*H); u=bytearray(W*H//4); v=bytearray(W*H//4)
            for yy in range(H):
                for xx in range(W):
                    # Translation plus independent texture makes temporal and MV metrics nontrivial.
                    y[yy*W+xx]=(40 + ((xx+3*n)*5 + (yy+n)*3 + ((xx//8)^(yy//8))*23)) & 255
            for yy in range(H//2):
                for xx in range(W//2):
                    u[yy*(W//2)+xx]=(96 + xx*3 + n*5) & 255
                    v[yy*(W//2)+xx]=(160 + yy*4 - n*3) & 255
            f.write(y); f.write(u); f.write(v)

def run_case(td, name, interlace):
    inp=os.path.join(td,name+'.y4m'); out=os.path.join(td,name+'.m2ts'); stats=os.path.join(td,name+'.csv')
    make_y4m(inp,interlace)
    cmd=[enc,'-i',inp,'-o',out,'--cq','6','--threads','1','--simd','none','--keyint','3',
         '--macroblock-stats',stats,'--max-frames',str(N)]
    subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
    config, header, frame, mb = read_macroblock_stats(stats)
    if config.get('format') != 'libvc1-macroblock-stats-v2':
        raise RuntimeError(name+': missing v2 format marker')
    required_config={'codec_version','width','height','y4m_fps','effective_fps','input_fps_overridden','profile','rc_mode',
                     'bitrate','vbv_buffer','threads','search_range','local_search_range','bframes','trellis','aq_enabled','aq_strength','dquant',
                     'residual_priority_threshold_mae','residual_priority_width_mae','residual_priority_max_q_boost','inter_intra_threshold',
                     'rc_i_weight','rc_p_weight','rc_b_weight'}
    missing=required_config-set(config)
    if missing: raise RuntimeError(f'{name}: missing config metadata {sorted(missing)}')
    required_mb={'prediction_mae_y','residual_priority_position','residual_priority_requested_q_boost',
                 'residual_priority_applied_q_boost','inter_intra_cost_ratio','aq_dark_detail',
                 'aq_color_luma_priority','aq_color_chroma_priority','aq_requested_q_boost'}
    if not required_mb.issubset(header): raise RuntimeError(f'{name}: missing tuning columns {sorted(required_mb-set(header))}')
    if len(frame)!=N: raise RuntimeError(f'{name}: expected {N} frame rows, got {len(frame)}')
    if len(mb)!=N*4: raise RuntimeError(f'{name}: expected {N*4} macroblock rows, got {len(mb)}')
    # Scoped format deliberately emits frame-level fields once. They must be absent/blank in MB rows.
    for r in mb:
        for k in ('display_order','coded_order','frame_type','frame_bits','shared_frame_bits','target_bits','predicted_bits','vbv_before_bits','vbv_after_bits','frame_q'):
            if r.get(k,'') not in ('',None): raise RuntimeError(f'{name}: frame-level field {k} repeated in macroblock row')
    byframe={i:[] for i in range(N)}
    for r in mb:
        byframe[int(frame_value(r,'display_order'))].append(r)
        if float(r['recon_psnr_y_db'])<=0 or float(r['recon_psnr_yuv_db'])<=0:
            raise RuntimeError(name+': invalid reconstruction quality metric')
    if not any(int(r['local_bits'])>0 for r in mb): raise RuntimeError(name+': no macroblock has local bit accounting')
    for fr in frame:
        d=int(fr['display_order']); local=sum(int(r['local_bits']) for r in byframe[d])
        if int(fr['shared_frame_bits'])+local != int(fr['frame_bits']):
            raise RuntimeError(name+': frame/shared/local bit accounting does not close')
        attributed=sum(float(r['amortized_frame_bits']) for r in byframe[d])
        if abs(attributed-int(fr['frame_bits']))>1e-6:
            raise RuntimeError(name+': amortized macroblock bits do not sum to frame size')
    if any(float(r['previous_mse_y'])!=-1.0 for r in byframe[0]):
        raise RuntimeError(name+': first frame unexpectedly has previous-source metrics')
    if any(float(r['previous_mse_y'])<0.0 for r in byframe[3]):
        raise RuntimeError(name+': previous-source metrics were lost at a GOP boundary')
    return frame+mb,byframe

with tempfile.TemporaryDirectory(prefix='libvc1-mbstats-') as td:
    rows,by=run_case(td,'progressive','p')
    if any(int(r['field_index'])!=-1 or int(r['field_parity'])!=-1 for rs in by.values() for r in rs):
        raise RuntimeError('progressive trace unexpectedly reports field macroblocks')
    predictive=[r for rs in by.values() for r in rs if r['mode'] in ('P-inter','P-4mv','P-skip','B-forward','B-backward','B-bi','B-direct')]
    if not predictive: raise RuntimeError('progressive trace did not exercise inter prediction')
    if not any(int(r['fwd_ref_display'])>=0 or int(r['bwd_ref_display'])>=0 for r in predictive):
        raise RuntimeError('progressive trace did not report reference display indices')
    if not any(float(r['prediction_mse_y'])>=0.0 for r in predictive):
        raise RuntimeError('progressive trace did not report prediction error')

    rows,by=run_case(td,'interlaced','t')
    field=[r for rs in by.values() for r in rs if r['mode'].startswith('field-')]
    if not field: raise RuntimeError('interlaced trace did not report field-coded macroblocks')
    if {int(r['field_index']) for r in field}!={0,1}: raise RuntimeError('interlaced trace did not cover both coded fields')
    if {int(r['field_parity']) for r in field}!={0,1}: raise RuntimeError('interlaced trace did not report both field parities')
    if not any(int(r['opposite_field_ref']) for r in field): raise RuntimeError('interlaced trace did not exercise opposite-field prediction')
    if not any(int(r['fwd_ref_display'])>=0 or int(r['bwd_ref_display'])>=0 for r in field):
        raise RuntimeError('interlaced trace did not report field reference display indices')
print('macroblock statistics CLI/runtime ok')
