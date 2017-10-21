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

#if !defined(_COMMON_BLOCK_H_)
#define _COMMON_BLOCK_H_

#include "types.h"
#include "simd.h"

void TEMPLATE(dequantize)(int16_t *coeff, int16_t *rcoeff, int qp, int size, qmtx_t * wt_matrix);
void TEMPLATE(reconstruct_block)(int16_t *block, SAMPLE *pblock, SAMPLE *rec, int size, int pstride, int stride, int bitdepth);

#if CDEF
void cdef_filter_block(uint8_t *dst8, uint16_t *dst16, int dstride,
                       const uint16_t *in, int sstride, int pri_strength, int sec_strength,
                       int dir, int pri_damping, int sec_damping, int bsize, int cdef_directions[8][2 + CDEF_FULL], int coeff_shift);

int TEMPLATE(cdef_find_dir)(const SAMPLE *img, int stride, int32_t *var, int coeff_shift);
#endif

void TEMPLATE(find_block_contexts)(int ypos, int xpos, int height, int width, int size, deblock_data_t *deblock_data, block_context_t *block_context, int enable);

void TEMPLATE(clpf_block)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizex, int sizey, boundary_type bt, unsigned int strength, unsigned int damping);

int clpf_sample(int X, int A, int B, int C, int D, int E, int F, int G, int H, int s, unsigned int dmp);

void TEMPLATE(improve_uv_prediction)(SAMPLE *y, SAMPLE *u, SAMPLE *v, SAMPLE *ry, int n, int cstride, int stride, int sub, int bitdepth);

SIMD_INLINE int get_left_available(int ypos, int xpos, int bwidth, int bheight, int fwidth, int fheight, int sb_size) {
  return xpos > 0;
}

SIMD_INLINE int get_up_available(int ypos, int xpos, int bwidth, int bheight, int fwidth, int fheight, int sb_size) {
  return ypos > 0;
}

SIMD_INLINE int get_upright_available(int ypos, int xpos, int bwidth, int bheight, int fwidth, int fheight, int sb_size) {

  int upright_available;
  int size, size2;

  /* Test for frame boundaries */
  upright_available = (ypos > 0) && (xpos + bwidth < fwidth);

  /* Test for coding block boundaries */
  size = max(bwidth, bheight);
  for (size2 = size; size2 < sb_size; size2 *= 2) {
    if ((ypos % (size2 << 1)) == size2 && (xpos % size2) == (size2 - size)) upright_available = 0;
  }
  return upright_available;
}

SIMD_INLINE int get_downleft_available(int ypos, int xpos, int bwidth, int bheight, int fwidth, int fheight, int sb_size) {

  int downleft_available;
  int size, size2;

  /* Test for frame boundaries */
  downleft_available = (xpos > 0) && (ypos + bheight < fheight);

  size = max(bwidth, bheight);
  /* Test for external super block boundaries */
  if ((ypos % sb_size) == (sb_size - size) && (xpos % sb_size) == 0) downleft_available = 0;

  /* Test for coding block boundaries */
  size = max(bwidth, bheight);
  for (size2 = 2 * size; size2 <= sb_size; size2 *= 2) {
    if ((ypos % size2) == (size2 - size) && (xpos % size2) > 0) downleft_available = 0;
  }

  return downleft_available;
}

#endif
