#pragma once
#include <cstdint>
#include <cstddef>

namespace libvc1::simd {

// x86-64-v1 baseline acceleration kernels.  These use only SSE2, which is
// mandatory on every x86-64 CPU; the translation unit is compiled with -march=x86-64.
uint64_t frame_sad_x86_64_v1(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_x86_64_v1(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_x86_64_v1(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_x86_64_v1(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_x86_64_v1(int* block64);
void put_block8_x86_64_v1(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_x86_64_v1(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_x86_64_v1(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_x86_64_v1(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_x86_64_v1(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_x86_64_v1(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_x86_64_v1(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_x86_64_v1(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_x86_64_v1(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_x86_64_v1(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_x86_64_v1(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_x86_64_v1(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_x86_64_v1(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_x86_64_v1(const uint8_t* src,size_t count);


// Prescott feature band: x86-64 baseline + SSE3, tuned for Nocona/Prescott.
uint64_t frame_sad_prescott(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_prescott(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_prescott(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_prescott(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_prescott(int* block64);
void put_block8_prescott(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_prescott(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_prescott(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_prescott(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_prescott(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_prescott(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_prescott(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_prescott(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_prescott(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_prescott(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_prescott(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_prescott(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_prescott(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_prescott(const uint8_t* src,size_t count);


// K10 feature band: SSE3 + SSE4a + ABM/POPCNT for AMD Family 10h-class CPUs.
uint64_t frame_sad_k10(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_k10(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_k10(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_k10(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_k10(int* block64);
void put_block8_k10(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_k10(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_k10(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_k10(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_k10(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_k10(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_k10(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_k10(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_k10(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_k10(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_k10(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_k10(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_k10(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_k10(const uint8_t* src,size_t count);


// Conroe feature band: x86-64 baseline + SSE3/SSSE3, tuned for Core 2.
uint64_t frame_sad_conroe(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_conroe(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_conroe(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_conroe(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_conroe(int* block64);
void put_block8_conroe(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_conroe(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_conroe(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_conroe(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_conroe(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_conroe(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_conroe(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_conroe(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_conroe(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_conroe(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_conroe(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_conroe(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_conroe(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_conroe(const uint8_t* src,size_t count);


// Penryn feature band: x86-64 baseline + SSE3/SSSE3/SSE4.1, tuned for Core 2.
uint64_t frame_sad_penryn(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_penryn(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_penryn(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_penryn(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_penryn(int* block64);
void put_block8_penryn(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_penryn(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_penryn(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_penryn(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_penryn(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_penryn(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_penryn(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_penryn(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_penryn(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_penryn(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_penryn(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_penryn(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_penryn(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_penryn(const uint8_t* src,size_t count);


// x86-64-v2 (SSE3/SSSE3/SSE4.x/POPCNT-class) acceleration kernels.
uint64_t frame_sad_x86_64_v2(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_x86_64_v2(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_x86_64_v2(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_x86_64_v2(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_x86_64_v2(int* block64);
void put_block8_x86_64_v2(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_x86_64_v2(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_x86_64_v2(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_x86_64_v2(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_x86_64_v2(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_x86_64_v2(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_x86_64_v2(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_x86_64_v2(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_x86_64_v2(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_x86_64_v2(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_x86_64_v2(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_x86_64_v2(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_x86_64_v2(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_x86_64_v2(const uint8_t* src,size_t count);


// Sandy Bridge feature band: v2 SIMD feature set plus AVX, without AVX2/FMA/BMI.
uint64_t frame_sad_sandybridge(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_sandybridge(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_sandybridge(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_sandybridge(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_sandybridge(int* block64);
void put_block8_sandybridge(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_sandybridge(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_sandybridge(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_sandybridge(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_sandybridge(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_sandybridge(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_sandybridge(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_sandybridge(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_sandybridge(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_sandybridge(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_sandybridge(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_sandybridge(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_sandybridge(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_sandybridge(const uint8_t* src,size_t count);


// Bulldozer feature band: AVX + XOP + FMA4; FMA4 execution is CPUID-gated.
uint64_t frame_sad_bulldozer(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_bulldozer(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_bulldozer(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_bulldozer(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_bulldozer(int* block64);
void put_block8_bulldozer(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_bulldozer(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_bulldozer(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_bulldozer(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_bulldozer(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_bulldozer(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_bulldozer(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_bulldozer(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_bulldozer(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_bulldozer(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_bulldozer(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_bulldozer(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_bulldozer(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_bulldozer(const uint8_t* src,size_t count);


// Piledriver feature band: Bulldozer flags plus FMA3/F16C/BMI1; FMA4 remains required.
uint64_t frame_sad_piledriver(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_piledriver(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_piledriver(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_piledriver(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_piledriver(int* block64);
void put_block8_piledriver(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_piledriver(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_piledriver(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_piledriver(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_piledriver(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_piledriver(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_piledriver(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_piledriver(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_piledriver(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_piledriver(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_piledriver(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_piledriver(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_piledriver(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_piledriver(const uint8_t* src,size_t count);


// AVX2-partial feature band: AVX2 kernels without requiring the full x86-64-v3 feature set.
uint64_t frame_sad_avx2_partial(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_avx2_partial(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_avx2_partial(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_avx2_partial(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_avx2_partial(int* block64);
void put_block8_avx2_partial(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_avx2_partial(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_avx2_partial(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_avx2_partial(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_avx2_partial(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_avx2_partial(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_avx2_partial(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_avx2_partial(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_avx2_partial(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_avx2_partial(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_avx2_partial(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_avx2_partial(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_avx2_partial(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_avx2_partial(const uint8_t* src,size_t count);

// x86-64-v3 (AVX2/FMA/BMI/LZCNT-class) acceleration kernels.
uint64_t frame_sad_x86_64_v3(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_x86_64_v3(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_x86_64_v3(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_x86_64_v3(const uint8_t* src,
                                            const uint8_t* pred, int stride,
                                            double* coeff64, bool use_fma);
void inverse_transform_8x8_x86_64_v3(int* block64);
void put_block8_x86_64_v3(uint8_t* dst, int stride, const int* block64, int bias);
void add_block_rect_x86_64_v3(uint8_t* dst, int stride, const int* block64,
                              int width, int height);
// Decoder-exact Advanced-profile luma interpolation for interior 8/16-pixel blocks.
uint64_t block_sad_luma_qpel_x86_64_v3(const uint8_t* cur, int cur_stride,
                                        const uint8_t* ref_center, int ref_stride,
                                        int width, int height, int hm, int vm,
                                        bool rnd);
uint64_t block_sad_luma_bilinear_x86_64_v3(const uint8_t* cur, int cur_stride,
                                            const uint8_t* ref_center, int ref_stride,
                                            int width, int height, bool fx, bool fy,
                                            bool rnd);
void luma_mc_block_x86_64_v3(uint8_t* dst, int dst_stride,
                             const uint8_t* ref_center, int ref_stride,
                             int width, int height, int hm, int vm,
                             bool rnd, bool bilinear);
void luma_mc_block_avg_x86_64_v3(uint8_t* dst, int dst_stride,
                                 const uint8_t* ref_center, int ref_stride,
                                 int width, int height, int hm, int vm,
                                 bool rnd, bool bilinear);
void chroma_mc_block_x86_64_v3(uint8_t* dst, int dst_stride,
                               const uint8_t* ref_center, int ref_stride,
                               int width, int height, int fx, int fy, bool rnd);
void chroma_mc_block_avg_x86_64_v3(uint8_t* dst, int dst_stride,
                                   const uint8_t* ref_center, int ref_stride,
                                   int width, int height, int fx, int fy, bool rnd);
void intensity_map_plane_x86_64_v3(uint8_t* dst, const uint8_t* src, size_t count,
                                   int scale, int shift);
void intensity_map_chroma_plane_x86_64_v3(uint8_t* dst, const uint8_t* src, size_t count,
                                          int scale);
void forward_transform_residual_rect_x86_64_v3(const uint8_t* src,
                                                const uint8_t* pred, int stride,
                                                int width, int height,
                                                double* coeff64, bool use_fma);
void quantize_coefficients_x86_64_v3(const double* coeff,int* out,int count,
                                         double qscale,double offset,int max_level);
// Exact integer statistics used by AQ/perceptual RDO.  These keep the scalar
// decision law unchanged while vectorizing the repeatedly visited 8/16-pixel
// transform and macroblock regions.
void perceptual_stats_x86_64_v3(const uint8_t* src,const uint8_t* pred,int stride,
                                int width,int height,uint64_t* sum,uint64_t* grad,
                                uint64_t* residual,int* min_value,int* max_value,
                                int* edges,int* grad_count);
uint64_t sum_u8_x86_64_v3(const uint8_t* src,size_t count);

// x86-64-v4 routines.  Every primitive has a v4 entry point so auto
// dispatch can benchmark v3 and v4 independently; individual v4 functions may
// deliberately retain narrower vectors when that wins on a particular CPU.
uint64_t frame_sad_x86_64_v4(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int dx, int dy,
                             int sample_stride);
uint64_t block_sad_x86_64_v4(const uint8_t* cur, int cur_stride,
                             const uint8_t* ref, int ref_stride,
                             int width, int height, int sample_stride);
void forward_transform_intra8_x86_64_v4(const uint8_t* src, int stride,
                                         double* coeff64, bool use_fma);
void forward_transform_residual8_x86_64_v4(const uint8_t* src,const uint8_t* pred,int stride,
                                            double* coeff64,bool use_fma);
void inverse_transform_8x8_x86_64_v4(int* block64);
void put_block8_x86_64_v4(uint8_t* dst,int stride,const int* block64,int bias);
void add_block_rect_x86_64_v4(uint8_t* dst,int stride,const int* block64,int width,int height);
uint64_t block_sad_luma_qpel_x86_64_v4(const uint8_t* cur,int cur_stride,const uint8_t* ref_center,int ref_stride,int width,int height,int hm,int vm,bool rnd);
uint64_t block_sad_luma_bilinear_x86_64_v4(const uint8_t* cur,int cur_stride,const uint8_t* ref_center,int ref_stride,int width,int height,bool fx,bool fy,bool rnd);
void luma_mc_block_x86_64_v4(uint8_t* dst,int dst_stride,const uint8_t* ref_center,int ref_stride,int width,int height,int hm,int vm,bool rnd,bool bilinear);
void luma_mc_block_avg_x86_64_v4(uint8_t* dst,int dst_stride,const uint8_t* ref_center,int ref_stride,int width,int height,int hm,int vm,bool rnd,bool bilinear);
void chroma_mc_block_x86_64_v4(uint8_t* dst,int dst_stride,const uint8_t* ref_center,int ref_stride,int width,int height,int fx,int fy,bool rnd);
void chroma_mc_block_avg_x86_64_v4(uint8_t* dst,int dst_stride,const uint8_t* ref_center,int ref_stride,int width,int height,int fx,int fy,bool rnd);
void intensity_map_plane_x86_64_v4(uint8_t* dst,const uint8_t* src,size_t count,int scale,int shift);
void intensity_map_chroma_plane_x86_64_v4(uint8_t* dst,const uint8_t* src,size_t count,int scale);
void forward_transform_residual_rect_x86_64_v4(const uint8_t* src,const uint8_t* pred,int stride,int width,int height,double* coeff64,bool use_fma);
void quantize_coefficients_x86_64_v4(const double* coeff,int* out,int count,double qscale,double offset,int max_level);
void perceptual_stats_x86_64_v4(const uint8_t* src,const uint8_t* pred,int stride,int width,int height,uint64_t* sum,uint64_t* grad,uint64_t* residual,int* min_value,int* max_value,int* edges,int* grad_count);
uint64_t sum_u8_x86_64_v4(const uint8_t* src,size_t count);

// 4x4 Walsh-Hadamard SATD used by staged motion-estimation refinement.
uint64_t satd4x4_x86_64_v1(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_prescott(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_k10(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_conroe(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_penryn(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_x86_64_v2(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_sandybridge(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_bulldozer(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_piledriver(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_avx2_partial(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_x86_64_v3(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);
uint64_t satd4x4_x86_64_v4(const uint8_t* cur,int cur_stride,const uint8_t* pred,int pred_stride);

// Exact reconstruction-side SIMD primitives.
void dequant_coefficients_x86_64_v1(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_x86_64_v1(int* block64,int type);
void overlap_vertical8_x86_64_v1(int* edge,int stride);
void overlap_horizontal8_x86_64_v1(int* edge,int stride);
bool loop_filter4_h_x86_64_v1(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_x86_64_v1(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_prescott(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_prescott(int* block64,int type);
void overlap_vertical8_prescott(int* edge,int stride);
void overlap_horizontal8_prescott(int* edge,int stride);
bool loop_filter4_h_prescott(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_prescott(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_k10(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_k10(int* block64,int type);
void overlap_vertical8_k10(int* edge,int stride);
void overlap_horizontal8_k10(int* edge,int stride);
bool loop_filter4_h_k10(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_k10(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_conroe(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_conroe(int* block64,int type);
void overlap_vertical8_conroe(int* edge,int stride);
void overlap_horizontal8_conroe(int* edge,int stride);
bool loop_filter4_h_conroe(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_conroe(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_penryn(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_penryn(int* block64,int type);
void overlap_vertical8_penryn(int* edge,int stride);
void overlap_horizontal8_penryn(int* edge,int stride);
bool loop_filter4_h_penryn(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_penryn(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_x86_64_v2(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_x86_64_v2(int* block64,int type);
void overlap_vertical8_x86_64_v2(int* edge,int stride);
void overlap_horizontal8_x86_64_v2(int* edge,int stride);
bool loop_filter4_h_x86_64_v2(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_x86_64_v2(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_sandybridge(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_sandybridge(int* block64,int type);
void overlap_vertical8_sandybridge(int* edge,int stride);
void overlap_horizontal8_sandybridge(int* edge,int stride);
bool loop_filter4_h_sandybridge(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_sandybridge(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_bulldozer(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_bulldozer(int* block64,int type);
void overlap_vertical8_bulldozer(int* edge,int stride);
void overlap_horizontal8_bulldozer(int* edge,int stride);
bool loop_filter4_h_bulldozer(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_bulldozer(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_piledriver(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_piledriver(int* block64,int type);
void overlap_vertical8_piledriver(int* edge,int stride);
void overlap_horizontal8_piledriver(int* edge,int stride);
bool loop_filter4_h_piledriver(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_piledriver(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_avx2_partial(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_avx2_partial(int* block64,int type);
void overlap_vertical8_avx2_partial(int* edge,int stride);
void overlap_horizontal8_avx2_partial(int* edge,int stride);
bool loop_filter4_h_avx2_partial(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_avx2_partial(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_x86_64_v3(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_x86_64_v3(int* block64,int type);
void overlap_vertical8_x86_64_v3(int* edge,int stride);
void overlap_horizontal8_x86_64_v3(int* edge,int stride);
bool loop_filter4_h_x86_64_v3(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_x86_64_v3(uint8_t* p,int stride,int x,int y,int pq);
void dequant_coefficients_x86_64_v4(const int* q,int* out,int count,int double_quant,int mquant,bool nonuniform);
void inverse_transform_rect_x86_64_v4(int* block64,int type);
void overlap_vertical8_x86_64_v4(int* edge,int stride);
void overlap_horizontal8_x86_64_v4(int* edge,int stride);
bool loop_filter4_h_x86_64_v4(uint8_t* p,int stride,int x,int y,int pq);
bool loop_filter4_v_x86_64_v4(uint8_t* p,int stride,int x,int y,int pq);

} // namespace libvc1::simd
