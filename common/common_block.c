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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <memory.h>
#include <assert.h>

#include "global.h"
#include "common_block.h"

extern const int zigzag16[16];
extern const int zigzag64[64];
extern const int zigzag256[256];
extern const int chroma_qp[52];
extern const uint16_t gquant_table[6];
extern const uint16_t gdequant_table[6];

void TEMPLATE(dequantize)(int16_t *coeff, int16_t *rcoeff, int qp, int size, qmtx_t * wt_matrix)
{
  int tr_log2size = log2i(size);
  const int lshift = qp / 6;
  const int qsize = min(size,MAX_QUANT_SIZE);
  const int rshift = tr_log2size - 1 + (wt_matrix!=NULL ? INV_WEIGHT_SHIFT : 0);
  const int64_t scale = gdequant_table[qp % 6];
  const int64_t add = lshift < rshift ? (1<<(rshift-lshift-1)) : 0;

  if (lshift >= rshift) {
    for (int i = 0; i < qsize ; i++){
      for (int j = 0; j < qsize; j++){
        int c = coeff[i*qsize + j];
        if (wt_matrix)
          c = c*wt_matrix[i*qsize+j];
        rcoeff[i*size+j] = (int16_t)((c * scale) << (lshift-rshift));// needs clipping?
      }
    }
  } else {
    for (int i = 0; i < qsize ; i++){
      for (int j = 0; j < qsize; j++){
        int c = coeff[i*qsize+j];
        if (wt_matrix)
          c = c*wt_matrix[i*qsize+j];
        rcoeff[i*size+j] = (int16_t)((c * scale + add) >> (rshift - lshift));//needs clipping
      }
    }
  }
}

void TEMPLATE(reconstruct_block)(int16_t *block, SAMPLE *pblock, SAMPLE *rec, int size, int pstride, int stride, int bitdepth)
{ 
  int i,j;
  for(i=0;i<size;i++){    
    for (j=0;j<size;j++){
      rec[i*stride+j] = (SAMPLE)saturate(block[i*size+j] + (int16_t)pblock[i*pstride+j], bitdepth);
    }
  }
}

#if CDEF

/* Detect direction. 0 means 45-degree up-right, 2 is horizontal, and so on.
   The search minimizes the weighted variance along all the lines in a
   particular direction, i.e. the squared error between the input and a
   "predicted" block where each pixel is replaced by the average along a line
   in a particular direction. Since each direction have the same sum(x^2) term,
   that term is never computed. See Section 2, step 2, of:
   http://jmvalin.ca/notes/intra_paint.pdf */
int TEMPLATE(cdef_find_dir)(const SAMPLE *img, int stride, int32_t *var, int coeff_shift) {
  int i;
  int32_t cost[8] = { 0 };
  int partial[8][15] = { { 0 } };
  int32_t best_cost = 0;
  int best_dir = 0;
  /* Instead of dividing by n between 2 and 8, we multiply by 3*5*7*8/n.
     The output is then 840 times larger, but we don't care for finding
     the max. */
  static const int div_table[] = { 0, 840, 420, 280, 210, 168, 140, 120, 105 };
  for (i = 0; i < 8; i++) {
    int j;
    for (j = 0; j < 8; j++) {
      int x;
      /* We subtract 128 here to reduce the maximum range of the squared
         partial sums. */
      x = (img[i * stride + j] >> coeff_shift) - 128;
      partial[0][i + j] += x;
      partial[1][i + j / 2] += x;
      partial[2][i] += x;
      partial[3][3 + i - j / 2] += x;
      partial[4][7 + i - j] += x;
      partial[5][3 - i / 2 + j] += x;
      partial[6][j] += x;
      partial[7][i / 2 + j] += x;
    }
  }
  for (i = 0; i < 8; i++) {
    cost[2] += partial[2][i] * partial[2][i];
    cost[6] += partial[6][i] * partial[6][i];
  }
  cost[2] *= div_table[8];
  cost[6] *= div_table[8];
  for (i = 0; i < 7; i++) {
    cost[0] += (partial[0][i] * partial[0][i] +
                partial[0][14 - i] * partial[0][14 - i]) *
               div_table[i + 1];
    cost[4] += (partial[4][i] * partial[4][i] +
                partial[4][14 - i] * partial[4][14 - i]) *
               div_table[i + 1];
  }
  cost[0] += partial[0][7] * partial[0][7] * div_table[8];
  cost[4] += partial[4][7] * partial[4][7] * div_table[8];
  for (i = 1; i < 8; i += 2) {
    int j;
    for (j = 0; j < 4 + 1; j++) {
      cost[i] += partial[i][3 + j] * partial[i][3 + j];
    }
    cost[i] *= div_table[8];
    for (j = 0; j < 4 - 1; j++) {
      cost[i] += (partial[i][j] * partial[i][j] +
                  partial[i][10 - j] * partial[i][10 - j]) *
                 div_table[2 * j + 2];
    }
  }
  for (i = 0; i < 8; i++) {
    if (cost[i] > best_cost) {
      best_cost = cost[i];
      best_dir = i;
    }
  }
  /* Difference between the optimal variance and the variance along the
     orthogonal direction. Again, the sum(x^2) terms cancel out. */
  *var = best_cost - cost[(best_dir + 4) & 7];
  /* We'd normally divide by 840, but dividing by 1024 is close enough
     for what we're going to do with this. */
  *var >>= 10;
  return best_dir;
}
#ifndef HBD

#if CDEF_FULL
const int cdef_directions_x[8][3] = {
  {  1,  2,  3 },
  {  1,  2,  3 },
  {  1,  2,  3 },
  {  1,  2,  3 },
  {  1,  2,  3 },
  {  0,  1,  1 },
  {  0,  0,  0 },
  {  0, -1, -1 }
};
const int cdef_directions_y[8][3] = {
  { -1, -2, -3 },
  {  0, -1, -1 },
  {  0,  0,  0 },
  {  0,  1,  1 },
  {  1,  2,  3 },
  {  1,  2,  3 },
  {  1,  2,  3 },
  {  1,  2,  3 }
};

const int cdef_pri_taps[2][3] = { { 3, 2, 1 }, { 2, 2, 2 } };
const int cdef_sec_taps[2][2] = { { 3, 1 }, { 3, 1 } };
#else
const int cdef_directions_x[8][2] = {
  {  1,  2 },
  {  1,  2 },
  {  1,  2 },
  {  1,  2 },
  {  1,  2 },
  {  0,  1 },
  {  0,  0 },
  {  0, -1 }
};
const int cdef_directions_y[8][2] = {
  { -1, -2 },
  {  0, -1 },
  {  0,  0 },
  {  0,  1 },
  {  1,  2 },
  {  1,  2 },
  {  1,  2 },
  {  1,  2 }
};

const int cdef_pri_taps[2][2] = { { 4, 2 }, { 3, 3 } };
const int cdef_sec_taps[2][2] = { { 2, 1 }, { 2, 1 } };
#endif

SIMD_INLINE int sign(int i) { return i < 0 ? -1 : 1; }

SIMD_INLINE int constrain(int diff, int threshold, unsigned int damping) {
  return threshold
             ? sign(diff) * min(abs(diff), max(0, threshold - (abs(diff) >> (damping - log2i(threshold)))))
             : 0;
}

/* Smooth in the direction detected. */
void cdef_filter_block(uint8_t *dst8, uint16_t *dst16, int dstride,
                       const uint16_t *in, int sstride, int pri_strength, int sec_strength,
                       int dir, int pri_damping, int sec_damping, int bsize, int cdef_directions[8][2 + CDEF_FULL])
{
  int i, j, k;
  const int *pri_taps = cdef_pri_taps[pri_strength & 1];
  const int *sec_taps = cdef_sec_taps[pri_strength & 1];
  for (i = 0; i < bsize; i++) {
    for (j = 0; j < bsize; j++) {
      int16_t sum = 0;
      int16_t y;
      int16_t x = in[i * sstride + j];
      int mx = x;
      int mn = x;
#if CDEF_FULL
      for (k = 0; k < 3; k++)
#else
      for (k = 0; k < 2; k++)
#endif
      {
        int16_t p0 = in[i * sstride + j + cdef_directions[dir][k]];
        int16_t p1 = in[i * sstride + j - cdef_directions[dir][k]];
        sum += pri_taps[k] * constrain(p0 - x, pri_strength, pri_damping);
        sum += pri_taps[k] * constrain(p1 - x, pri_strength, pri_damping);
        if (p0 != CDEF_VERY_LARGE) mx = max(p0, mx);
        if (p1 != CDEF_VERY_LARGE) mx = max(p1, mx);
        mn = min(p0, mn);
        mn = min(p1, mn);
#if CDEF_FULL
        if (k == 2) continue;
#endif
        int16_t s0 = in[i * sstride + j + cdef_directions[(dir + 2) & 7][k]];
        int16_t s1 = in[i * sstride + j - cdef_directions[(dir + 2) & 7][k]];
        int16_t s2 = in[i * sstride + j + cdef_directions[(dir + 6) & 7][k]];
        int16_t s3 = in[i * sstride + j - cdef_directions[(dir + 6) & 7][k]];
        if (s0 != CDEF_VERY_LARGE) mx = max(s0, mx);
        if (s1 != CDEF_VERY_LARGE) mx = max(s1, mx);
        if (s2 != CDEF_VERY_LARGE) mx = max(s2, mx);
        if (s3 != CDEF_VERY_LARGE) mx = max(s3, mx);
        mn = min(s0, mn);
        mn = min(s1, mn);
        mn = min(s2, mn);
        mn = min(s3, mn);
        sum += sec_taps[k] * constrain(s0 - x, sec_strength, sec_damping);
        sum += sec_taps[k] * constrain(s1 - x, sec_strength, sec_damping);
        sum += sec_taps[k] * constrain(s2 - x, sec_strength, sec_damping);
        sum += sec_taps[k] * constrain(s3 - x, sec_strength, sec_damping);
      }
      y = clip((int16_t)x + ((8 + sum - (sum < 0)) >> 4), mn, mx);
      if (dst8)
        dst8[i * dstride + j] = (uint8_t)y;
      else
        dst16[i * dstride + j] = (uint16_t)y;
    }
  }
}
#endif
#endif

void TEMPLATE(find_block_contexts)(int ypos, int xpos, int height, int width, int size, deblock_data_t *deblock_data, block_context_t *block_context, int enable){

  if (ypos >= MIN_BLOCK_SIZE && xpos >= MIN_BLOCK_SIZE && ypos + size < height && xpos + size < width && enable && size <= MAX_TR_SIZE) {
    int by = ypos/MIN_PB_SIZE;
    int bx = xpos/MIN_PB_SIZE;
    int bs = width/MIN_PB_SIZE;
    int bindex = by*bs+bx;
    block_context->split = (deblock_data[bindex-bs].size < size) + (deblock_data[bindex-1].size < size);
    int cbp1;
    cbp1 = (deblock_data[bindex-bs].cbp.y > 0) + (deblock_data[bindex-1].cbp.y > 0);
    block_context->cbp = cbp1;
    int cbp2 = (deblock_data[bindex-bs].cbp.y > 0 || deblock_data[bindex-bs].cbp.u > 0 || deblock_data[bindex-bs].cbp.v > 0) +
           (deblock_data[bindex-1].cbp.y > 0 || deblock_data[bindex-1].cbp.u > 0 || deblock_data[bindex-1].cbp.v > 0);
    block_context->index = 3*block_context->split + cbp2;
  }
  else{
    block_context->split = -1;
    block_context->cbp = -1;
    block_context->index = -1;
  }
}

#ifndef HBD
#if !CDEF
static int sign(int i) { return i < 0 ? -1 : 1; }

static int constrain(int x, int s, unsigned int damping) {
  return sign(x) * max(0, abs(x) - max(0, abs(x) - s +
                                             (abs(x) >> (damping - log2i(s)))));
}
#endif

int clpf_sample(int X, int A, int B, int C, int D, int E, int F, int G, int H, int s, unsigned int dmp) {
  int delta = 1 * constrain(A - X, s, dmp) + 3 * constrain(B - X, s, dmp) +
              1 * constrain(C - X, s, dmp) + 3 * constrain(D - X, s, dmp) +
              3 * constrain(E - X, s, dmp) + 1 * constrain(F - X, s, dmp) +
              3 * constrain(G - X, s, dmp) + 1 * constrain(H - X, s, dmp);
  return (8 + delta - (delta < 0)) >> 4;
}
#endif

void TEMPLATE(clpf_block)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizex, int sizey, boundary_type bt, unsigned int strength, unsigned int damping) {
  const int xmin = x0 - !(bt & TILE_LEFT_BOUNDARY) * 2;
  const int ymin = y0 - !(bt & TILE_ABOVE_BOUNDARY) * 2;
  const int xmax = x0 + sizex + !(bt & TILE_RIGHT_BOUNDARY) * 2 - 1;
  const int ymax = y0 + sizey + !(bt & TILE_BOTTOM_BOUNDARY) * 2 - 1;

  for (int y = y0; y < y0 + sizey; y++) {
    for (int x = x0; x < x0 + sizex; x++) {
      const int X = src[y * sstride + x];
      const int A = src[max(ymin, y - 2) * sstride + x];
      const int B = src[max(ymin, y - 1) * sstride + x];
      const int C = src[y * sstride + max(xmin, x - 2)];
      const int D = src[y * sstride + max(xmin, x - 1)];
      const int E = src[y * sstride + min(xmax, x + 1)];
      const int F = src[y * sstride + min(xmax, x + 2)];
      const int G = src[min(ymax, y + 1) * sstride + x];
      const int H = src[min(ymax, y + 2) * sstride + x];
      const int delta = clpf_sample(X, A, B, C, D, E, F, G, H, strength, damping);
      dst[y * dstride + x] = X + delta;
    }
  }
}

void TEMPLATE(improve_uv_prediction)(SAMPLE *y, SAMPLE *u, SAMPLE *v, SAMPLE *ry, int n, int cstride, int stride, int sub, int bitdepth)
{
  int nc = n >> sub;
  int lognc = log2i(nc);

  // Compute squared residual
  int64_t squared_residual = 0;
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      squared_residual +=
        (ry[i*stride + j] - y[i*n + j]) *
        (ry[i*stride + j] - y[i*n + j]);

  // If the luma prediction is good, we change nothing
  if ((squared_residual >> (log2i(n) + log2i(n))) <= (64 << 2 * (bitdepth - 8)))
    return;

  // Compute linear fit between predicted chroma and predicted luma
  // Could be int32_t when SAMPLE is 8 bit
  int64_t ysum = 0, usum = 0, vsum = 0, yysum = 0, yusum = 0, yvsum = 0, uusum = 0, vvsum = 0;
  for (int i = 0; i < nc; i++)
    for (int j = 0; j < nc; j++) {
      int us = u[i * (cstride >> sub) + j];
      int vs = v[i * (cstride >> sub) + j];
      int ys = sub ?
	(y[(i*2 + 0)*n + j*2 + 0] + y[(i*2 + 0)*n+j*2 + 1] +
	 y[(i*2 + 1)*n + j*2 + 0] + y[(i*2 + 1)*n+j*2 + 1] + 2) >> 2 :
	y[i * cstride + j];
      ysum  += ys;
      usum  += us;
      vsum  += vs;
      yysum += ys * ys;
      yusum += ys * us;
      yvsum += ys * vs;
      uusum += us * us;
      vvsum += vs * vs;
    }

  int64_t ssyy = yysum - (ysum*ysum >> lognc * 2);
  int64_t ssuu = uusum - (usum*usum >> lognc * 2);
  int64_t ssvv = vvsum - (vsum*vsum >> lognc * 2);
  int64_t ssyu = yusum - (ysum*usum >> lognc * 2);
  int64_t ssyv = yvsum - (ysum*vsum >> lognc * 2);

  // Require a correlation above a threshold
  if (ssyy) {
    if (ssyu * ssyu * 2 > ssyy * ssuu) {
      int64_t a64 = (ssyu << 16) / ssyy;
      int64_t b64 = ((usum << 16) - a64 * ysum) >> lognc * 2;
      int32_t a = (int32_t)clip(a64, -(1 << (31 - bitdepth)), 1 << (31 - bitdepth));
      int32_t b = (int32_t)clip(b64 + (1 << 15), -(1LL << 31), (1U << 31) - 1);

      // Map reconstructed luma to new predicted chroma
      for (int i = 0; i < nc; i++)
        for (int j = 0; j < nc; j++) {
          u[i*(cstride >> sub) + j] = sub ?
            (saturate((a*ry[(i*2+0)*stride+j*2+0] + b) >> 16, bitdepth) +
             saturate((a*ry[(i*2+0)*stride+j*2+1] + b) >> 16, bitdepth) +
             saturate((a*ry[(i*2+1)*stride+j*2+0] + b) >> 16, bitdepth) +
             saturate((a*ry[(i*2+1)*stride+j*2+1] + b) >> 16, bitdepth) + 2) >> 2 :
            saturate((a*ry[i*stride+j] + b) >> 16, bitdepth);
        }
    }
    if (ssyv * ssyv * 2 > ssyy * ssvv) {
      int64_t a64 = (ssyv << 16) / ssyy;
      int64_t b64 = ((vsum << 16) - a64 * ysum) >> lognc * 2;
      int32_t a = (int32_t)clip(a64, -(1 << (31 - bitdepth)), 1 << (31 - bitdepth));
      int32_t b = (int32_t)clip(b64 + (1 << 15), -(1LL << 31), (1U << 31) - 1);

      // Map reconstructed luma to new predicted chroma
      for (int i = 0; i < nc; i++)
        for (int j = 0; j < nc; j++) {
          v[i*(cstride >> sub) + j] = sub ?
            (saturate((a*ry[(i*2+0)*stride+j*2+0] + b) >> 16, bitdepth) +
             saturate((a*ry[(i*2+0)*stride+j*2+1] + b) >> 16, bitdepth) +
             saturate((a*ry[(i*2+1)*stride+j*2+0] + b) >> 16, bitdepth) +
             saturate((a*ry[(i*2+1)*stride+j*2+1] + b) >> 16, bitdepth) + 2) >> 2 :
            saturate((a*ry[i*stride+j] + b) >> 16, bitdepth);
        }
    }
  }
}
