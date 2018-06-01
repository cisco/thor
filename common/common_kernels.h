/*
Copyright (c) 2015, Cisco Systems
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef COMMON_SIMDKERNELS_H
#define COMMON_SIMDKERNELS_H

#include <stdint.h>
#include "common_block.h"

void TEMPLATE(block_avg_simd)(SAMPLE *p,SAMPLE *r0, SAMPLE *r1, int sp, int s0, int s1, int width, int height);
int TEMPLATE(sad_calc_simd_unaligned)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int width, int height);
void TEMPLATE(get_inter_prediction_luma_simd)(int width, int height, int xoff, int yoff, SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip, int istride, int bipred, int bitdepth);
void TEMPLATE(get_inter_prediction_chroma_simd)(int width, int height, int xoff, int yoff, SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip, int istride, int bitdepth);
void transform_simd(const int16_t *block, int16_t *coeff, int size, int fast, int bitdepth);
void inverse_transform_simd(const int16_t *coeff, int16_t *block, int size, int bitdepth);
void TEMPLATE(clpf_block4)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizey, boundary_type bt, unsigned int strength, unsigned int dmp);
void TEMPLATE(clpf_block8)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizey, boundary_type bt, unsigned int strength, unsigned int dmp);
void TEMPLATE(clpf_block4_noclip)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizey, unsigned int strength, unsigned int dmp);
void TEMPLATE(clpf_block8_noclip)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizey, unsigned int strength, unsigned int dmp);
void TEMPLATE(scale_frame_down2x2_simd)(yuv_frame_t* sin, yuv_frame_t* sout);

SIMD_INLINE void TEMPLATE(clpf_block_simd)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizex, int sizey, boundary_type bt, unsigned int strength, unsigned int dmp) {
  if ((sizex != 4 && sizex != 8) || ((sizey & 1) && sizex == 4)) {
    // Fallback to C for odd sizes:
    // * block width not 4 or 8
    // * block heights not a multiple of 2 if the block width is 4
    TEMPLATE(clpf_block)(src, dst, sstride, dstride, x0, y0, sizex, sizey, bt, strength, dmp);
  } else {
    if (bt)
      (sizex == 4 ? TEMPLATE(clpf_block4) : TEMPLATE(clpf_block8))(src, dst, sstride, dstride, x0, y0, sizey, bt, strength, dmp);
    else
      (sizex == 4 ? TEMPLATE(clpf_block4_noclip) : TEMPLATE(clpf_block8_noclip))(src, dst, sstride, dstride, x0, y0, sizey, strength, dmp);
  }
#if 0
  if (sizex == 4 && sizey == 4) // chroma 420
    TEMPLATE(clpf_block4)(src, dst, sstride, dstride, x0, y0, 4, bt, strength, dmp);
  else if (sizex == 4) { // chroma 422
    TEMPLATE(clpf_block4)(src, dst, sstride, dstride, x0, y0, 4, bt, strength, dmp);
    TEMPLATE(clpf_block4)(src + 4*sstride, dst + 4*dstride, sstride, dstride, x0, y0, 4, bt, strength, dmp);
  } else // 444 or luma 420
    TEMPLATE(clpf_block8)(src, dst, sstride, dstride, x0, y0, 8, bt, strength, dmp);
#endif
}

#if CDEF
void cdef_filter_block_simd(uint8_t *dst8, uint16_t *dst16, int dstride,
                              const uint16_t *in, int sstride, int pri_strength,
                              int sec_strength, int dir, int pri_damping,
                            int sec_damping, int bsize, int cdef_directions[8][2 + CDEF_FULL], int coeff_shift);
int TEMPLATE(cdef_find_dir_simd)(const SAMPLE *img, int stride, int32_t *var, int coeff_shift);
v128 compute_directions(v128 lines[8], int32_t tmp_cost1[4]);
void array_reverse_transpose_8x8(v128 *in, v128 *res);
int cdef_find_best_dir(v128 *lines, int32_t *cost, int *best_cost);
#endif

#endif
