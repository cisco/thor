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

int TEMPLATE(clpf_sample)(int X, int A, int B, int C, int D, int E, int F, int b) {
  int delta =
    4*clip(A - X, -b, b) + clip(B - X, -b, b) + 3*clip(C - X, -b, b) +
    3*clip(D - X, -b, b) + clip(E - X, -b, b) + 4*clip(F - X, -b, b);
  return (8 + delta - (delta < 0)) >> 4;
}

void TEMPLATE(clpf_block)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizex, int sizey, boundary_type bt, unsigned int strength) {
  const int xmin = x0 - !(bt & TILE_LEFT_BOUNDARY) * 2;
  const int ymin = y0 - !(bt & TILE_ABOVE_BOUNDARY) * 2;
  const int xmax = x0 + sizex + !(bt & TILE_RIGHT_BOUNDARY) * 2 - 1;
  const int ymax = y0 + sizey + !(bt & TILE_BOTTOM_BOUNDARY) * 2 - 1;

  for (int y = y0; y < y0 + sizey; y++) {
    for (int x = x0; x < x0 + sizex; x++) {
      const int X = src[y * sstride + x];
      const int A = src[max(ymin, y - 1) * sstride + x];
      const int B = src[y * sstride + max(xmin, x - 2)];
      const int C = src[y * sstride + max(xmin, x - 1)];
      const int D = src[y * sstride + min(xmax, x + 1)];
      const int E = src[y * sstride + min(xmax, x + 2)];
      const int F = src[min(ymax, y + 1) * sstride + x];
      const int delta = TEMPLATE(clpf_sample)(X, A, B, C, D, E, F, strength);
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
