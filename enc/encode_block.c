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

/* -*- mode: c; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2; -*- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <memory.h>
#include <assert.h>

#include "global.h"
#include "strings.h"
#include "snr.h"
#include "mainenc.h"
#include "write_bits.h"
#include "putvlc.h"
#include "transform.h"
#include "common_block.h"
#include "inter_prediction.h"
#include "intra_prediction.h"
#include "enc_kernels.h"

int YPOS,XPOS;

extern int chroma_qp[52];
extern int zigzag16[16];
extern int zigzag64[64];
extern int zigzag256[256];
extern uint16_t gquant_table[6];
extern uint16_t gdequant_table[6];
extern double squared_lambda_QP [MAX_QP+1];

static inline uint64_t mv_mask_hash(const mv_t *mv) { return (uint64_t)1 << (((mv->y << 3) ^ mv->x) & 63); }

static inline void add_mvcandidate(const mv_t *mv, mv_t *list, int *list_len, uint64_t *mask)
{
  mv_t imv;
  imv.x = (mv->x + 2) >> 2;
  imv.y = (mv->y + 2) >> 2;
  uint64_t m = mv_mask_hash(&imv);
  if (!(m & *mask)) {
    list[*list_len] = imv;
    *list_len += 1;
  }
  *mask |= m;
}

int quantize (int16_t *coeff, int16_t *coeffq, int qp, int size, int coeff_block_type, qmtx_t* wmatrix, int ws)
{
  int intra_block = (coeff_block_type>>1) & 1;
  int tr_log2size = log2i(size);
  int qsize = min(MAX_QUANT_SIZE,size); //Only quantize 16x16 low frequency coefficients
  int64_t scale = gquant_table[qp%6];
  int scoeff[MAX_QUANT_SIZE*MAX_QUANT_SIZE];
  int scoeffq[MAX_QUANT_SIZE*MAX_QUANT_SIZE];
  int64_t level64,abs_coeff;
  int i,j,c,sign,offset,level,cbp,pos,last_pos,level0,offset0,offset1;
  int shift2 = 21 - tr_log2size + qp/6 + (wmatrix ? WEIGHT_SHIFT : 0);
  int level_mode = 1;

  int *zigzagptr = zigzag64;
  if (qsize==4)
    zigzagptr = zigzag16;
  else if (qsize==8)
    zigzagptr = zigzag64;
  else if (qsize==16)
    zigzagptr = zigzag256;

  /* Initialize 1D array of quantized coefficients to zero */
  memset(scoeffq,0,qsize*qsize*sizeof(int));

  /* Zigzag scan of 8x8 low frequency coefficients */
  for(i=0;i<qsize;i++){
    for (j=0;j<qsize;j++){
      scoeff[zigzagptr[i*qsize+j]] = coeff[i*size+j];
      if (wmatrix!=NULL)
        scoeff[zigzagptr[i*qsize+j]] *= wmatrix[i*ws+j];
    }
  }

  /* Find last_pos */
  offset = intra_block ? 38 : -26; //Scaled by 256 relative to quantization step size
  offset = offset<<(shift2-8);
  level = 0;
  pos = qsize*qsize-1;
  while (level==0 && pos>=0){
    c = scoeff[pos];
    level64 = (abs(c))*scale + offset;
    level = (level64>0 ? level64 : -level64)>>shift2;
    pos--;
  }
  last_pos = level ? pos+1 : pos;

  /* Forward scan up to last_pos */
  cbp = 0;

  offset0 = intra_block ? 102 : 51; //Scaled by 256 relative to quantization step size
  offset1 = intra_block ? 115 : 90; //Scaled by 256 relative to quantization step size
  for (pos=0;pos<=last_pos;pos++){
    c = scoeff[pos];
    sign = c < 0 ? -1 : 1;
    abs_coeff = scale*abs(c);
    level0 = (abs_coeff + 0)>>shift2;
    offset = (level0 > (1 - level_mode)) ? offset1 : offset0;
    offset = offset<<(shift2-8);
    level = (abs_coeff + offset)>>shift2;
    scoeffq[pos] = sign * level;
    cbp = cbp || (level != 0);
    if (level_mode) {
      if (level == 0) level_mode = 0;
    }
    else {
      if (level > 1) level_mode = 1;
    }

  }

  for (i = 0; i < qsize; i++) {
    for (j = 0; j < qsize; j++) {
      coeffq[i*size + j] = scoeffq[zigzagptr[i*qsize + j]];
    }
  }
  return (cbp != 0);
}

void get_residual(int16_t *block,uint8_t *pblock, uint8_t *orig, int size, int orig_stride)
{ 
  int i,j;
  
  for(i=0;i<size;i++){    
    for (j=0;j<size;j++){
      block[i*size+j] = (int16_t)orig[i*orig_stride+j] - (int16_t)pblock[i*size+j];
    }
  }
}


/* Return the best approximated half-pel position around the centre using SIMD friendly averages */
unsigned int sad_calc_fasthalf(const uint8_t *a, const uint8_t *b, int astride, int bstride, int width, int height, int *x, int *y)
{
  unsigned int tl = 0, tr = 0, br = 0, bl = 0, top = 0, right = 0, down = 0, left = 0;
  int bestx = 0, besty = -2;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int ptl, ptr, pbr, pbl;

      int t1, t2, t3, t4, t5, t6, t7, t8;
      t1 = (b[-bstride+j-1] + b[-bstride+j] + 1) >> 1;
      t2 = (b[j-1] + b[j] + 1) >> 1;
      t1 = (t1 + t2) >> 1;
      t3 = (b[-2*bstride+j-1] + b[bstride+j-1] + 1) >> 1;
      t4 = (b[-2*bstride+j] + b[bstride+j] + 1) >> 1;
      t3 = (t3 + t4) >> 1;
      t5 = (b[-bstride+j-2] + b[-bstride+j+1] + 1) >> 1;
      t6 = (b[j-2] + b[j+1] + 1) >> 1;
      t5 = (t5 + t6) >> 1;
      t5 = (t3 + t5) >> 1;
      ptl = (t5 + t1) >> 1;
      left += abs(a[i*astride+j] - t2);

      t1 = (b[-bstride+j] + b[-bstride+j+1] + 1) >> 1;
      t8 = (b[j] + b[j+1] + 1) >> 1;
      t1 = (t1 + t8) >> 1;
      t5 = (b[-2*bstride+j+1] + b[bstride+j+1] + 1) >> 1;
      t3 = (t4 + t5) >> 1;
      t4 = (b[-bstride+j-1] + b[-bstride+j+2] + 1) >> 1;
      t7 = (b[j-1] + b[j+2] + 1) >> 1;
      t5 = (t7 + t4) >> 1;
      t5 = (t3 + t5) >> 1;
      ptr = (t5 + t1) >> 1;
      right += abs(a[i*astride+j] - t8);

      t1 = (b[bstride+j-1] + b[bstride+j] + 1) >> 1;
      t3 = (t1 + t2) >> 1;
      t2 = (b[-bstride+j-1] + b[2*bstride+j-1] + 1) >> 1;
      t4 = (b[-bstride+j] + b[2*bstride+j] + 1) >> 1;
      t5 = (t4 + t2) >> 1;
      t1 = (b[bstride+j-2] + b[bstride+j+1] + 1) >> 1;
      t2 = (t6 + t1) >> 1;
      t2 = (t5 + t2) >> 1;
      pbl = (t2 + t3) >> 1;

      t2 = (b[bstride+j] + b[bstride+j+1] + 1) >> 1;
      t3 = (t8 + t2) >> 1;
      t5 = (b[-bstride+j+1] + b[2*bstride+j+1] + 1) >> 1;
      t6 = (t4 + t5) >> 1;
      t8 = (b[bstride+j-1] + b[bstride+j+2] + 1) >> 1;
      t1 = (t7 + t8) >> 1;
      t2 = (t6 + t1) >> 1;
      pbr = (t2 + t3) >> 1;

      down += abs(a[i*astride+j] - ((b[j] + b[j+bstride] + 1) >> 1));
      top += abs(a[i*astride+j] - ((b[j] + b[j-bstride] + 1) >> 1));
      tl += abs(a[i*astride+j] - ptl);
      tr += abs(a[i*astride+j] - ptr);
      br += abs(a[i*astride+j] - pbr);
      bl += abs(a[i*astride+j] - pbl);
    }
    b += bstride;
  }

  if (down < top) {
    besty = 2;
    top = down;
  }

  if (right < top) {
    bestx = 2;
    besty = 0;
    top = right;
  }

  if (left < top) {
    bestx = -2;
    besty = 0;
    top = left;
  }

  if (tl < top) {
    bestx = -2;
    besty = -2;
    top = tl;
  }

  if (tr < top) {
    bestx = 2;
    besty = -2;
    top = tr;
  }

  if (br < top) {
    bestx = 2;
    besty = 2;
    top = br;
  }

  if (bl < top) {
    bestx = -2;
    besty = 2;
    top = bl;
  }

  *x = bestx;
  *y = besty;
  return top;
}


/* Return the best approximated quarter-pel position around the centre using SIMD friendly averages */
unsigned int sad_calc_fastquarter(const uint8_t *o, const uint8_t *r, int os, int rs, int width, int height, int *x, int *y)
{
  unsigned int tl = 0, tr = 0, br = 0, bl = 0, top = 0, right = 0, down = 0, left = 0;
  int bestx = 0, besty = -1;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {

      if (*x & *y) {
        uint8_t a = r[j];
        uint8_t d = r[j + 1];
        uint8_t e = r[j + rs + 1];
        uint8_t f = r[j + rs];
        uint8_t ad = (a + d + 1) >> 1;
        uint8_t de = (d + e + 1) >> 1;
        uint8_t af = (a + f + 1) >> 1;
        uint8_t fe = (f + e + 1) >> 1;

        tl +=    abs(o[i*os+j] - ((ad + af) >> 1));
        top +=   abs(o[i*os+j] - ((de +  a) >> 1));
        tr +=    abs(o[i*os+j] - ((ad + de) >> 1));
        left +=  abs(o[i*os+j] - ((ad +  f) >> 1));
        right += abs(o[i*os+j] - ((ad +  e) >> 1));
        bl +=    abs(o[i*os+j] - ((af + fe) >> 1));
        down +=  abs(o[i*os+j] - ((de +  f) >> 1));
        br +=    abs(o[i*os+j] - ((de + fe) >> 1));
      } else if (*x) {
        uint8_t a = r[j];
        uint8_t b = r[j - rs];
        uint8_t c = r[j - rs + 1];
        uint8_t d = r[j + 1];
        uint8_t e = r[j + rs + 1];
        uint8_t f = r[j + rs];
        uint8_t ad = (a + d + 1) >> 1;
        uint8_t de = (d + e + 1) >> 1;
        uint8_t dc = (d + c + 1) >> 1;
        uint8_t af = (a + f + 1) >> 1;
        uint8_t ab = (a + b + 1) >> 1;

        tl +=    abs(o[i*os+j] - ((ad + ab) >> 1));
        top +=   abs(o[i*os+j] - ((dc +  a) >> 1));
        tr +=    abs(o[i*os+j] - ((ad + dc) >> 1));
        left +=  abs(o[i*os+j] - ((ad +  a) >> 1));
        right += abs(o[i*os+j] - ((ad +  d) >> 1));
        bl +=    abs(o[i*os+j] - ((ad + af) >> 1));
        down +=  abs(o[i*os+j] - ((af +  d) >> 1));
        br +=    abs(o[i*os+j] - ((ad + de) >> 1));
      } else if (*y) {
        uint8_t a = r[j];
        uint8_t d = r[j + 1];
        uint8_t e = r[j + rs + 1];
        uint8_t f = r[j + rs];
        uint8_t g = r[j + rs - 1];
        uint8_t h = r[j - 1];
        uint8_t ad = (a + d + 1) >> 1;
        uint8_t af = (a + f + 1) >> 1;
        uint8_t fe = (f + e + 1) >> 1;
        uint8_t ah = (a + h + 1) >> 1;
        uint8_t gf = (g + f + 1) >> 1;

        tl +=    abs(o[i*os+j] - ((ah + af) >> 1));
        top +=   abs(o[i*os+j] - ((af +  a) >> 1));
        tr +=    abs(o[i*os+j] - ((ad + af) >> 1));
        left +=  abs(o[i*os+j] - ((gf +  a) >> 1));
        right += abs(o[i*os+j] - ((ad +  f) >> 1));
        bl +=    abs(o[i*os+j] - ((af + gf) >> 1));
        down +=  abs(o[i*os+j] - ((af +  f) >> 1));
        br +=    abs(o[i*os+j] - ((af + fe) >> 1));
      } else {
        uint8_t a = r[j];
        uint8_t b = r[j - rs];
        uint8_t d = r[j + 1];
        uint8_t f = r[j + rs];
        uint8_t h = r[j - 1];
        uint8_t ad = (a + d + 1) >> 1;
        uint8_t af = (a + f + 1) >> 1;
        uint8_t ah = (a + h + 1) >> 1;
        uint8_t ab = (a + b + 1) >> 1;

        tl +=    abs(o[i*os+j] - ((ah + ab) >> 1));
        top +=   abs(o[i*os+j] - ((ab +  a) >> 1));
        tr +=    abs(o[i*os+j] - ((ad + ab) >> 1));
        left +=  abs(o[i*os+j] - ((ah +  a) >> 1));
        right += abs(o[i*os+j] - ((ad +  a) >> 1));
        bl +=    abs(o[i*os+j] - ((ah + af) >> 1));
        down +=  abs(o[i*os+j] - ((af +  a) >> 1));
        br +=    abs(o[i*os+j] - ((af + ad) >> 1));
      }
    }
    r += rs;
  }

  if (tl < top) {
    bestx = -1;
    top = tl;
  }
  if (tr < top) {
    bestx = 1;
    top = tr;
  }
  if (left < top) {
    bestx = -1;
    besty = 0;
    top = left;
  }
  if (right < top) {
    bestx = 1;
    besty = 0;
    top = right;
  }
  if (bl < top) {
    bestx = -1;
    besty = 1;
    top = bl;
  }
  if (down < top) {
    bestx = 0;
    besty = 1;
    top = down;
  }
  if (br < top) {
    bestx = 1;
    besty = 1;
    top = br;
  }

  *x = bestx;
  *y = besty;
  return top;
}

unsigned int sad_calc(uint8_t *a, uint8_t *b, int astride, int bstride, int width, int height)
{
  unsigned int i,j,sad = 0;

  if (use_simd && width > 4){
    return sad_calc_simd(a, b, astride, bstride, width, height);
  }
  else {
    for(i=0;i<height;i++){
      for (j=0;j<width;j++){
        sad += abs(a[i*astride+j] - b[i*bstride+j]);
      }
    }
  }
  return sad;
}

unsigned int widesad_calc(uint8_t *a, uint8_t *b, int astride, int bstride, int width, int height, int *x)
{
  // Calculate the SAD for five positions x.xXx.x and return the best
  if (use_simd && width == 16 && height == 16) {
    return widesad_calc_simd(a, b, astride, bstride, width, height, x);
  }
  else {
    static int off[] = { -3, -1, 0, 1, 3 };
    unsigned int bestsad = 1<<31;
    int bestx = 0;
    for(int k = 0; k < sizeof(off) / sizeof(int); k++) {
      unsigned int sad = 0;
      for(int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
          sad += abs(a[i*astride + j] - b[i*bstride + j + off[k]]);
      if (sad < bestsad) {
        bestsad = sad;
        bestx = off[k];
      }
    }
    *x = bestx;
    return bestsad;
  }
}

int ssd_calc(uint8_t *a, uint8_t *b, int astride, int bstride, int width,int height)
{
  int i,j,ssd = 0;
  if (use_simd && width > 4 && width==height){
    int size = width;
    return ssd_calc_simd(a, b, astride, bstride, size);
  }
  else{
    for(i=0;i<height;i++){
      for (j=0;j<width;j++){
        ssd += (a[i*astride+j] - b[i*bstride+j]) * (a[i*astride+j] - b[i*bstride+j]);
      }
    }
  }
  return ssd;
}

int quote_mv_bits(int mv_diff_y, int mv_diff_x)
{
  int bits = 0;
  int code,mvabs;

  /* mvx */
  mvabs = abs(mv_diff_x);
  int len;
  if (mvabs < 1) {
    len = 2;
  }
  else if (mvabs < (1 + 1)) {
    len = 3 + 1;
  }
  else if (mvabs < (1 + 1 + 2)) {
    len = 4 + 1;
  }
  else if (mvabs < (1 + 1 + 2 + 4 * 8)) {
    code = mvabs - (1 + 1 + 2);
    len = 5 + (code >> 3) + 1;
  }
  else {
    code = mvabs - (1 + 1 + 2 + 4 * 8);
    len = 10 + (code >> 4) + 1;
  }
  bits += len;

  /* mvy */
  mvabs = abs(mv_diff_y);
  if (mvabs < 1) {
    len = 2;
  }
  else if (mvabs < (1 + 1)) {
    len = 3 + 1;
  }
  else if (mvabs < (1 + 1 + 2)) {
    len = 4 + 1;
  }
  else if (mvabs < (1 + 1 + 2 + 4 * 8)) {
    code = mvabs - (1 + 1 + 2);
    len = 5 + (code >> 3) + 1;
  }
  else {
    code = mvabs - (1 + 1 + 2 + 4 * 8);
    len = 10 + (code >> 4) + 1;
  }
  bits += len;
  return bits;
}

int motion_estimate(uint8_t *orig, uint8_t *ref, int size, int stride_r, int width, int height, mv_t *mv, mv_t *mvc, mv_t *mvp, double lambda,enc_params *params, int sign, int fwidth, int fheight, int xpos, int ypos, mv_t *mvcand, int *mvcand_num, int enable_bipred){
  unsigned int sad;
  uint32_t min_sad;
  uint8_t *rf = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  mv_t mv_cand;
  mv_t mv_opt;
  mv_t mv_ref;
  int s = sign ? -1 : 1;

  min_sad = MAX_UINT32;
  mv_ref.y = (((mvc->y) + 2) >> 2) << 2;
  mv_ref.x = (((mvc->x) + 2) >> 2) << 2;

  if ((size == 16 && enable_bipred) || params->encoder_speed == 0) {

    /* Telescope search */
    int step = 32;
    while (step >= 4) {
      int range = 2*step;
      for (int k = -range; k <= range; k += step) {
        for (int l = -range; l <= range; l += step) {
          if (step < 32 && !k && !l)
            continue; //Center position was investigated at previous step

          mv_cand.y = mv_ref.y + k;
          mv_cand.x = mv_ref.x + l;
          clip_mv(&mv_cand, ypos, xpos, fwidth, fheight, size, sign);
          if (step == 32 && size == 16 && params->encoder_speed < 2 && params->encoder_speed > 0) {
            int x = 0;
            sad = widesad_calc(orig,ref + s*(mv_cand.x >> 2) + s*(mv_cand.y >> 2)*stride_r,size,stride_r,width,height,&x);
            mv_cand.x += s*x << 2;
          } else
            sad = sad_calc(orig,ref + s*(mv_cand.x >> 2) + s*(mv_cand.y >> 2)*stride_r,size,stride_r,width,height);
          sad += (unsigned int)(lambda * (double)quote_mv_bits(mv_cand.y - mvp->y, mv_cand.x - mvp->x) + 0.5);
          if (sad < min_sad){
            min_sad = sad;
            mv_opt = mv_cand;
          }
        }
      }

      mv_ref = mv_opt;
      step = step>>1;
    }
  }

  /* Candidate search */
  for (int idx = 0; idx < *mvcand_num; idx++){
    int x = 0;
    mv_cand.y = mvcand[idx].y << 2;
    mv_cand.x = mvcand[idx].x << 2;
    clip_mv(&mv_cand, ypos, xpos, fwidth, fheight, size, sign);
    if (size == 16)
      sad = widesad_calc(orig,ref + s*(mv_cand.x >> 2) + s*(mv_cand.y >> 2)*stride_r,size,stride_r,width,height, &x);
    else
      sad = sad_calc(orig,ref + s*(mv_cand.x >> 2) + s*(mv_cand.y >> 2)*stride_r,size,stride_r,width,height);
    mv_cand.x += s*x << 2;
    sad += (unsigned int)(lambda * (double)quote_mv_bits(mv_cand.y - mvp->y, mv_cand.x - mvp->x) + 0.5);
    if (sad < min_sad){
      min_sad = sad;
      mv_opt = mv_cand;
    }
  }

  mv_ref = mv_opt;

  int maxsteps = size <= 16 || params->encoder_speed == 0 ? 6 : 0;
  int start = 0;
  int end = 5;

  /* Full-pel search */
  for (int step = 1; step < maxsteps; step++) {
    int dir = start-1;
    int best_dir = -1;

    do {
      dir++;
      dir = dir == 6 ? 0 : dir;
      static int diy[] = {  1, 2, 1, -1, -2, -1 };
      static int dix[] = { -1, 0, 1,  1,  0, -1 };
      mv_cand.y = mv_ref.y + dix[dir]*4;
      mv_cand.x = mv_ref.x + diy[dir]*4;

      clip_mv(&mv_cand, ypos, xpos, fwidth, fheight, size, sign);
      sad = sad_calc(orig,ref + s*(mv_cand.x >> 2) + s*(mv_cand.y >> 2)*stride_r,size,stride_r,width,height);
      sad += (unsigned int)(lambda * (double)quote_mv_bits(mv_cand.y - mvp->y, mv_cand.x - mvp->x) + 0.5);
      if (sad < min_sad){
        min_sad = sad;
        mv_opt = mv_cand;
        best_dir = dir;
      }
    } while (dir != end);

    mv_ref = mv_opt;
    start = best_dir ? best_dir - 1 : 5;
    end = start + 2;
    end -= (end >= 6)*6;
    if (best_dir < 0)
      break;
  }


  int ydelta_hp = 0;
  int xdelta_hp = 0;
  int ydelta_qp = 0;
  int xdelta_qp = 0;
  unsigned int cmin = min_sad;

  if (params->encoder_speed == 0) {

    /* Half-pel search */
    for (int i = 1; i <= 8; i++) {
      static const int8_t hmpos[] = {0, 0, -2, 2, 0, -2, -2, 2, 2 };
      static const int8_t hnpos[] = {0, -2, 0, 0, 2, -2, 2, -2, 2 };

      mv_cand.y = mv_ref.y + hmpos[i];
      mv_cand.x = mv_ref.x + hnpos[i];
      get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign,enable_bipred,fwidth,fheight,xpos,ypos);
      sad = sad_calc(orig,rf,size,width,width,height);
      sad += (unsigned int)(lambda * (double)quote_mv_bits(mv_cand.y - mvp->y, mv_cand.x - mvp->x) + 0.5);

      if (sad < cmin) {
        cmin = sad;
        ydelta_hp = hmpos[i];
        xdelta_hp = hnpos[i];
      }
    }

    mv_opt.x += xdelta_hp;
    mv_opt.y += ydelta_hp;

    /* Quarter-pel search */
    static const int8_t qmpos[] = {0, 0, -1, 1, 0, -1, -1, 1, 1 };
    static const int8_t qnpos[] = {0, -1, 0, 0, 1, -1, 1, -1, 1 };

    for (int i = 1; i <= 8; i++) {
      mv_cand.y = mv_opt.y + qmpos[i];
      mv_cand.x = mv_opt.x + qnpos[i];
      get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign,enable_bipred,fwidth,fheight,xpos,ypos);
      sad = sad_calc(orig,rf,size,width,width,height);
      sad += (int)(lambda * (double)quote_mv_bits(mv_cand.y - mvp->y, mv_cand.x - mvp->x) + 0.5);
      if (sad < cmin) {
        cmin = sad;
        ydelta_qp = qmpos[i];
        xdelta_qp = qnpos[i];
      }
    }
  } else { /* Faster bilinear approximation */
    mv_ref.x *= s;
    mv_ref.y *= s;

    /* Half-pel search */
    int spx, spy;
    sad = (use_simd && width > 4 ? sad_calc_fasthalf_simd : sad_calc_fasthalf)(orig, ref + (mv_ref.x >> 2) + (mv_ref.y >> 2)*stride_r, size, stride_r, width, height, &spx, &spy);
    sad += (unsigned int)(lambda * (double)quote_mv_bits(mv_ref.y + s*spy - mvp->y, mv_ref.x + s*spx - mvp->x) + 0.5);

    if (sad < cmin) {
      cmin = sad;
      xdelta_hp = s*spx;
      ydelta_hp = s*spy;
    }
    spx = xdelta_hp;
    spy = ydelta_hp;

    mv_ref.x = mv_opt.x + s*spx;
    mv_ref.y = mv_opt.y + s*spy;
    mv_opt.x += xdelta_hp;
    mv_opt.y += ydelta_hp;

    /* Quarter-pel search */
    sad = (use_simd && width > 4 ? sad_calc_fastquarter_simd : sad_calc_fastquarter)(orig, ref + s*(mv_ref.x >> 2) + s*(mv_ref.y >> 2)*stride_r, size, stride_r, width, height, &spx, &spy);
    sad += (int)(lambda * (double)quote_mv_bits(mv_ref.y + s*spy - mvp->y, mv_ref.x + s*spx - mvp->x) + 0.5);

    if (sad < cmin) {
      cmin = sad;
      xdelta_qp = s*spx;
      ydelta_qp = s*spy;
    }
  }

  mv_opt.x += xdelta_qp;
  mv_opt.y += ydelta_qp;

  *mv = mv_opt;
  thor_free(rf);
  return min(cmin, min_sad);
}

int motion_estimate_sync(uint8_t *orig, uint8_t *ref, int size, int stride_r, int width, int height, mv_t *mv, mv_t *mvc, mv_t *mvp, double lambda,enc_params *params, int sign, int fwidth, int fheight, int xpos, int ypos, mv_t *mvcand, int *mvcand_num, int enable_bipred){
  int k,l,sad,range,step;
  uint32_t min_sad;
  uint8_t *rf = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  mv_t mv_cand;
  mv_t mv_opt;
  mv_t mv_ref;
  int16_t mv_diff_y,mv_diff_x;
  /* Initialize */
  mv_opt.y = mv_opt.x = 0;
  min_sad = MAX_UINT32;

  /* Telecope search from 8x8 grid to 1/4 x 1/4 grid around rounded mvc (=mvp for uni-pred, part=PART_NONE)*/
  mv_ref.y = (((mvc->y) + 2) >> 2) << 2;
  mv_ref.x = (((mvc->x) + 2) >> 2) << 2;
  step = 32;
  while (step > 0){
    range = step;
    for (k=-range;k<=range;k+=step){
      for (l=-range;l<=range;l+=step){
        if (step<32 && k==0 && l==0) continue; //Center position was investigated at previous step
        int exitFlag=0;
        if (step==1){
          int ver_frac,hor_frac;
          ver_frac = mv_ref.y&3;
          hor_frac = mv_ref.x&3;
          if (ver_frac==0 && hor_frac==0){
            /* Integer pixel */
            if (abs(k)!=abs(l)) exitFlag = 1;
          }
          else if(ver_frac==2 && hor_frac==2){
            /* Funny position */
            exitFlag = 1;
          }
          else{
            /* Remaining half pixel */
            if (abs(k)==abs(l)) exitFlag = 1;
          }
        }
        if (exitFlag) continue;

        mv_cand.y = mv_ref.y + k;
        mv_cand.x = mv_ref.x + l;

        clip_mv(&mv_cand, ypos, xpos, fwidth, fheight, size, sign);
        get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign, enable_bipred,fwidth,fheight,xpos,ypos);
        sad = sad_calc(orig,rf,size,width,width,height);
        mv_diff_y = mv_cand.y - mvp->y;
        mv_diff_x = mv_cand.x - mvp->x;
        sad += (int)(lambda * (double)quote_mv_bits(mv_diff_y,mv_diff_x) + 0.5);
        if (sad < min_sad){
          min_sad = sad;
          mv_opt = mv_cand;
        }
      }
    }
    mv_ref = mv_opt;
    step = step>>1;
  }

  /* Extra candidate search */
  mvcand[4] = *mvp;
  mvcand[5].y = 0;
  mvcand[5].x = 0;
  for (int idx=0;idx<ME_CANDIDATES;idx++){
    mv_cand = mvcand[idx];

    clip_mv(&mv_cand, ypos, xpos, fwidth, fheight, size, sign);
    get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign,enable_bipred,fwidth,fheight,xpos,ypos);
    sad = sad_calc(orig,rf,size,width,width,height);
    mv_diff_y = mv_cand.y - mvp->y;
    mv_diff_x = mv_cand.x - mvp->x;
    sad += (int)(lambda * (double)quote_mv_bits(mv_diff_y,mv_diff_x) + 0.5);
    if (sad < min_sad){
      min_sad = sad;
      mv_opt = mv_cand;
    }
  }
  *mv = mv_opt;

  thor_free(rf);
  return min_sad;
}

int motion_estimate_bi(uint8_t *orig, uint8_t *ref0, uint8_t *ref1, int size, int stride_r, int width, int height, mv_t *mv, mv_t *mvc, mv_t *mvp, double lambda, enc_params *params, int sign, int fwidth, int fheight, int xpos, int ypos, mv_t *mvcand, int *mvcand_num, int enable_bipred) {
  int k, l, sad, range, step;
  uint32_t min_sad;
  uint8_t *rf = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *rf0 = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *rf1 = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  mv_t mv_cand;
  mv_t mv_opt;
  mv_t mv_ref;
  int16_t mv_diff_y, mv_diff_x;
  /* Initialize */
  mv_opt.y = mv_opt.x = 0;
  min_sad = MAX_UINT32;

  /* Telecope search from 8x8 grid to 1/4 x 1/4 grid around rounded mvc (=mvp for uni-pred, part=PART_NONE)*/
  mv_ref.y = (((mvc->y) + 2) >> 2) << 2;
  mv_ref.x = (((mvc->x) + 2) >> 2) << 2;
  step = 32;
  while (step > 0) {
    range = step;
    for (k = -range; k <= range; k += step) {
      for (l = -range; l <= range; l += step) {
        if (step<32 && k == 0 && l == 0) continue; //Center position was investigated at previous step
        int exitFlag = 0;
        if (step == 1) {
          int ver_frac, hor_frac;
          ver_frac = mv_ref.y & 3;
          hor_frac = mv_ref.x & 3;
          if (ver_frac == 0 && hor_frac == 0) {
            /* Integer pixel */
            if (abs(k) != abs(l)) exitFlag = 1;
          }
          else if (ver_frac == 2 && hor_frac == 2) {
            /* Funny position */
            exitFlag = 1;
          }
          else {
            /* Remaining half pixel */
            if (abs(k) == abs(l)) exitFlag = 1;
          }
        }
        if (exitFlag) continue;

        mv_cand.y = mv_ref.y + k;
        mv_cand.x = mv_ref.x + l;

        clip_mv(&mv_cand, ypos, xpos, fwidth, fheight, size, sign);
        get_inter_prediction_luma(rf0, ref0, width, height, stride_r, width, &mv_cand, sign, enable_bipred,fwidth,fheight,xpos,ypos);

        clip_mv(&mv_cand, ypos, xpos, fwidth, fheight, size, 1 - sign);
        get_inter_prediction_luma(rf1, ref1, width, height, stride_r, width, &mv_cand, 1 - sign, enable_bipred,fwidth,fheight,xpos,ypos);

        int i, j;
        for (i = 0; i < size; i++) {
          for (j = 0; j < size; j++) {
            rf[i*size + j] = (uint8_t)(((int)rf0[i*size + j] + (int)rf1[i*size + j]) >> 1);
          }
        }

        sad = sad_calc(orig, rf, size, width, width, height);
        mv_diff_y = mv_cand.y - mvp->y;
        mv_diff_x = mv_cand.x - mvp->x;
        sad += (int)(lambda * (double)quote_mv_bits(mv_diff_y, mv_diff_x) + 0.5);
        if (sad < min_sad) {
          min_sad = sad;
          mv_opt = mv_cand;
        }
      }
    }
    mv_ref = mv_opt;
    step = step >> 1;
  }

  for (int idx = *mvcand_num; idx < 4; idx++) { //Temporary workaround
    mvcand[idx].y = 0;
    mvcand[idx].x = 0;
  }

  /* Extra candidate search */
  mvcand[4] = *mvp;
  mvcand[5].y = 0;
  mvcand[5].x = 0;

  for (int idx = 0; idx<ME_CANDIDATES; idx++) {
    mv_cand = mvcand[idx];

    clip_mv(&mv_cand, ypos, xpos, fwidth, fheight, size, sign);
    get_inter_prediction_luma(rf0, ref0, width, height, stride_r, width, &mv_cand, sign, enable_bipred,fwidth,fheight,xpos,ypos);

    clip_mv(&mv_cand, ypos, xpos, fwidth, fheight, size, 1 - sign);
    get_inter_prediction_luma(rf1, ref1, width, height, stride_r, width, &mv_cand, 1 - sign, enable_bipred,fwidth,fheight,xpos,ypos);

    int i, j;
    for (i = 0; i < size; i++) {
      for (j = 0; j < size; j++) {
        rf[i*size + j] = (uint8_t)(((int)rf0[i*size + j] + (int)rf1[i*size + j]) >> 1);
      }
    }
    sad = sad_calc(orig, rf, size, width, width, height);
    mv_diff_y = mv_cand.y - mvp->y;
    mv_diff_x = mv_cand.x - mvp->x;
    sad += (int)(lambda * (double)quote_mv_bits(mv_diff_y, mv_diff_x) + 0.5);
    if (sad < min_sad) {
      min_sad = sad;
      mv_opt = mv_cand;
    }
  }
  *mv = mv_opt;

  thor_free(rf);
  thor_free(rf0);
  thor_free(rf1);
  return min_sad;
}


uint32_t cost_calc(yuv_block_t *org_block,yuv_block_t *rec_block,int stride,int width, int height,int nbits,double lambda)
{
  uint32_t cost;
  int ssd_y,ssd_u,ssd_v;
  ssd_y = ssd_calc(org_block->y,rec_block->y,stride,stride,width,height);
  ssd_u = ssd_calc(org_block->u,rec_block->u,stride/2,stride/2,width/2,height/2);
  ssd_v = ssd_calc(org_block->v,rec_block->v,stride/2,stride/2,width/2,height/2);
  cost = ssd_y + ssd_u + ssd_v + (int32_t)(lambda*nbits + 0.5);
  if (cost > 1 << 30) cost = 1 << 30; //Robustification
  return cost;
}

int search_intra_prediction_params(uint8_t *org_y,yuv_frame_t *rec,block_pos_t *block_pos,int width,int height,int num_intra_modes,intra_mode_t *intra_mode)
{
  int size = block_pos->size;
  int yposY = block_pos->ypos;
  int xposY = block_pos->xpos;
  int sad,min_sad;
  uint8_t *pblock = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t* left = (uint8_t*)thor_alloc(2*MAX_TR_SIZE+2,16)+1;
  uint8_t* top = (uint8_t*)thor_alloc(2*MAX_TR_SIZE+2,16)+1;
  uint8_t top_left;

  int upright_available = get_upright_available(yposY,xposY,size,width);
  int downleft_available = get_downleft_available(yposY,xposY,size,height);
  make_top_and_left(left,top,&top_left,&rec->y[yposY*rec->stride_y+xposY],rec->stride_y,NULL,0,0,0,yposY,xposY,size,upright_available,downleft_available,0);


  /* Search for intra modes */
  min_sad = (1<<30);
  *intra_mode = MODE_DC;

  get_dc_pred(xposY >=0 ? left:top,yposY >= 0 ? top:left,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_DC;
    min_sad = sad;
  }
  get_hor_pred(left,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_HOR;
    min_sad = sad;
  }

  get_ver_pred(top,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_VER;
    min_sad = sad;
  }

  get_planar_pred(left,top,top_left,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_PLANAR;
    min_sad = sad;
  }

  if (num_intra_modes == 4) { //TODO: generalize
    thor_free(left - 1);
    thor_free(top - 1);
    thor_free(pblock);
    return min_sad;
  }

  get_upleft_pred(left,top,top_left,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPLEFT;
    min_sad = sad;
  }

  get_upright_pred(top,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPRIGHT;
    min_sad = sad;
  }

  get_upupright_pred(top,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPUPRIGHT;
    min_sad = sad;
  }

  get_upupleft_pred(left,top,top_left,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPUPLEFT;
    min_sad = sad;
  }

  get_upleftleft_pred(left,top,top_left,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPLEFTLEFT;
    min_sad = sad;
  }

  get_downleftleft_pred(left,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_DOWNLEFTLEFT;
    min_sad = sad;
  }
  thor_free(left - 1);
  thor_free(top - 1);
  thor_free(pblock);
  return min_sad;
}

int search_inter_prediction_params(uint8_t *org_y,yuv_frame_t *ref,block_pos_t *block_pos,mv_t *mvc, mv_t *mvp, mv_t *mv_arr, part_t part, double lambda, enc_params *params, int sign,int fwidth,int fheight, mv_t *mvcand, int *mvcand_num, int enable_bipred)
{
  int size = block_pos->size;
  int yposY = block_pos->ypos;
  int xposY = block_pos->xpos;
  int ref_posY = yposY*ref->stride_y + xposY;
  uint8_t *ref_y = ref->y + ref_posY;
  mv_t mv;
  mv_t mvp2 = *mvp; //mv predictor from outside block
  int rstride = ref->stride_y;
  int ostride = size;
  int index,py,px,offset_r,offset_o,width,height;
  int sad=0;
  if (part==PART_NONE){
    width = size;
    height = size;
    offset_o = 0;
    offset_r = 0;
    sad += (params->sync ? motion_estimate_sync : motion_estimate)(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,mvc,&mvp2,lambda,params,sign,fwidth,fheight,xposY,yposY,mvcand,mvcand_num, enable_bipred);
    mv_arr[0] = mv;
    mv_arr[1] = mv;
    mv_arr[2] = mv;
    mv_arr[3] = mv;
  }
  else if (part==PART_HOR){
    width = size;
    height = size/2;
    for (index=0;index<4;index+=2){
      py = index>>1;
      offset_o = py*(size/2)*ostride;
      offset_r = py*(size/2)*rstride;
      sad += motion_estimate(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,mvc,&mvp2,lambda,params,sign,fwidth,fheight,xposY,yposY,mvcand,mvcand_num, enable_bipred);
      mv_arr[index] = mv;
      mv_arr[index+1] = mv;
      mvp2 = mv_arr[0]; //mv predictor from inside block
    }
  }
  else if (part==PART_VER){
    width = size/2;
    height = size;
    for (index=0;index<2;index++){
      px = index;
      offset_o = px*(size/2);
      offset_r = px*(size/2);
      sad += motion_estimate(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,mvc,&mvp2,lambda,params,sign,fwidth,fheight,xposY,yposY,mvcand,mvcand_num,enable_bipred);
      mv_arr[index] = mv;
      mv_arr[index+2] = mv;
      mvp2 = mv_arr[0]; //mv predictor from inside block
    }
  }
  else if (part==PART_QUAD){
    width = size/2;
    height = size/2;
    for (index=0;index<4;index++){
      px = index&1;
      py = (index&2)>>1;
      offset_o = py*(size/2)*ostride + px*(size/2);
      offset_r = py*(size/2)*rstride + px*(size/2);
      sad += motion_estimate(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,mvc,&mvp2,lambda,params,sign,fwidth,fheight,xposY,yposY,mvcand,mvcand_num,enable_bipred);
      mv_arr[index] = mv;
      mvp2 = mv_arr[0]; //mv predictor from inside block
    }
  }

  return sad;
}

int encode_and_reconstruct_block_intra (encoder_info_t *encoder_info, uint8_t *orig, int orig_stride, uint8_t* rec, int rec_stride, int ypos, int xpos, int size, int qp,
    uint8_t *pblock, int16_t *coeffq, uint8_t *rec_block, int coeff_type, int tb_split,int rdoq, int width, intra_mode_t intra_mode, int upright_available,int downleft_available,
    qmtx_t ** wmatrix, qmtx_t ** iwmatrix)
{
    int cbp,cbpbit;
    int16_t *block = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *block2 = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *coeff = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *rcoeff = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *rblock = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *rblock2 = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

    uint8_t* left_data = (uint8_t*)thor_alloc(2*MAX_TR_SIZE+2,16)+1;
    uint8_t* top_data = (uint8_t*)thor_alloc(2*MAX_TR_SIZE+2,16)+1;
    uint8_t top_left;

    if (tb_split){
      int size2 = size/2;
      cbp = 0;
      int i,j,index=0;
      for (i=0;i<size;i+=size2){
        for (j=0;j<size;j+=size2){

          make_top_and_left(left_data,top_data,&top_left,rec,rec_stride,&rec_block[i*size+j],size,i,j,ypos,xpos,size2,upright_available,downleft_available,1);

          get_intra_prediction(left_data,top_data,top_left,ypos+i,xpos+j,size2,pblock,intra_mode);
          get_residual (block2, pblock, &orig[i*orig_stride+j], size2, orig_stride);
          transform (block2, coeff, size2, encoder_info->params->encoder_speed > 1);
          cbpbit = quantize (coeff, coeffq+index, qp, size2, coeff_type, encoder_info->params->qmtx ? wmatrix[log2i(size2/4)] : NULL, MAX_QUANT_SIZE);
          if (cbpbit){
            dequantize (coeffq+index, rcoeff, qp, size2, encoder_info->params->qmtx ? iwmatrix[log2i(size2/4)] : NULL, MAX_QUANT_SIZE);
            inverse_transform (rcoeff, rblock2, size2);
          }
          else{
            memset(rblock2,0,size2*size2*sizeof(int16_t));
          }

          cbp = (cbp<<1) + cbpbit;
          index += size2 * size2;

          reconstruct_block (rblock2, pblock, &rec_block[i*size+j], size2, size);
        }
      }
    }
    else{
      make_top_and_left(left_data,top_data,&top_left,rec,rec_stride,NULL,0,0,0,ypos,xpos,size,upright_available,downleft_available,0);
      get_intra_prediction(left_data,top_data,top_left,ypos,xpos,size,pblock,intra_mode);
      get_residual (block, pblock, orig, size, orig_stride);

      transform (block, coeff, size, encoder_info->params->encoder_speed > 1);
      cbp = quantize (coeff, coeffq, qp, size, coeff_type, encoder_info->params->qmtx ? wmatrix[log2i(size/4)] : NULL, MAX_QUANT_SIZE);
      if (cbp){
        dequantize (coeffq, rcoeff, qp, size, encoder_info->params->qmtx ? iwmatrix[log2i(size/4)] : NULL, MAX_QUANT_SIZE);
        inverse_transform (rcoeff, rblock, size);
        reconstruct_block (rblock, pblock, rec_block, size, size);
      }
      else{
        memcpy(rec_block,pblock,size*size*sizeof(uint8_t));
      }
    }

    thor_free(top_data - 1);
    thor_free(left_data - 1);
    thor_free(block);
    thor_free(block2);
    thor_free(coeff);
    thor_free(rcoeff);
    thor_free(rblock);
    thor_free(rblock2);
    return cbp;
}

int encode_and_reconstruct_block_inter (encoder_info_t *encoder_info, uint8_t *orig, int orig_stride, int size, int qp, uint8_t *pblock, int16_t *coeffq, uint8_t *rec, int coeff_type, int tb_split,int rdoq, qmtx_t ** wmatrix, qmtx_t ** iwmatrix)
{
    int cbp,cbpbit;
    int16_t *block = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *block2 = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *coeff = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *rcoeff = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *rblock = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
    int16_t *rblock2 = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

    get_residual (block, pblock, orig, size, orig_stride);

    if (tb_split){
      int size2 = size/2;
      cbp = 0;
      int i,j,k,index=0;
      for (i=0;i<size;i+=size2){
        for (j=0;j<size;j+=size2){

          /* Copy from full size to compact block of quarter size */
          for (k=0;k<size2;k++){
            memcpy(&block2[k*size2],&block[(i+k)*size+j],size2*sizeof(int16_t));
          }
          transform (block2, coeff, size2, size == 64 || encoder_info->params->encoder_speed > 1);
          cbpbit = quantize (coeff, coeffq+index, qp, size2, coeff_type,encoder_info->params->qmtx ? wmatrix[log2i(size2/4)] : NULL,MAX_QUANT_SIZE);
          if (cbpbit){
            dequantize (coeffq+index, rcoeff, qp, size2, encoder_info->params->qmtx ? iwmatrix[log2i(size2/4)] : NULL, MAX_QUANT_SIZE);
            inverse_transform (rcoeff, rblock2, size2);
          }
          else{
            memset(rblock2,0,size2*size2*sizeof(int16_t));
          }

          /* Copy from compact block of quarter size to full size */
          for (k=0;k<size2;k++){
            memcpy(&rblock[(i+k)*size+j],&rblock2[k*size2],size2*sizeof(int16_t));
          }
          cbp = (cbp<<1) + cbpbit;
          index += size2 * size2;
        }
      }
      reconstruct_block (rblock, pblock, rec, size, size);
    }
    else{
      transform (block, coeff, size, (size == 64 && encoder_info->params->encoder_speed > 0) || encoder_info->params->encoder_speed > 1);
      cbp = quantize (coeff, coeffq, qp, size, coeff_type, encoder_info->params->qmtx ? wmatrix[log2i(size/4)] : NULL, MAX_QUANT_SIZE);
      if (cbp){
        dequantize (coeffq, rcoeff, qp, size, encoder_info->params->qmtx ? iwmatrix[log2i(size/4)] : NULL, MAX_QUANT_SIZE);
        inverse_transform (rcoeff, rblock, size);
        reconstruct_block (rblock, pblock, rec, size, size);
      }
      else{
        memcpy(rec,pblock,size*size*sizeof(uint8_t));
      }
    }

    thor_free(block);
    thor_free(block2);
    thor_free(coeff);
    thor_free(rcoeff);
    thor_free(rblock);
    thor_free(rblock2);
    return cbp;
}

void get_inter_prediction_yuv(yuv_frame_t *ref, uint8_t *pblock_y, uint8_t *pblock_u, uint8_t *pblock_v, block_info_t *block_info, mv_t *mv_arr, int sign, int width, int height, int enable_bipred, int split) {
  mv_t mv;
  int div = split + 1;
  int bwidth = block_info->block_pos.bwidth/div;
  int bheight = block_info->block_pos.bheight/div;
  int pstride = block_info->block_pos.size;
  int rstride_y = ref->stride_y;
  int rstride_c = ref->stride_c;
  int index;
  int yposY = block_info->block_pos.ypos;
  int xposY = block_info->block_pos.xpos;
  int yposC = yposY / 2;
  int xposC = xposY / 2;
  int size = block_info->block_pos.size;

  int ref_posY = yposY*ref->stride_y + xposY;
  int ref_posC = yposC*ref->stride_c + xposC;
  uint8_t *ref_y = ref->y + ref_posY;
  uint8_t *ref_u = ref->u + ref_posC;
  uint8_t *ref_v = ref->v + ref_posC;
  for (index = 0; index<div*div; index++) {
    int idx = (index >> 0) & 1;
    int idy = (index >> 1) & 1;
    int offsetpY = idy*bheight*pstride + idx*bwidth;
    int offsetpC = idy*bheight*pstride/4 + idx*bwidth/2;
    int offsetrY = idy*bheight*rstride_y + idx*bwidth;
    int offsetrC = idy*bheight*rstride_c/2 + idx*bwidth/2;
    mv = mv_arr[index];
    clip_mv(&mv, yposY, xposY, width, height, size, sign);
    get_inter_prediction_luma(pblock_y + offsetpY, ref_y + offsetrY, bwidth, bheight, rstride_y, pstride, &mv, sign, enable_bipred, width, height, xposY, yposY);
    get_inter_prediction_chroma(pblock_u + offsetpC, ref_u + offsetrC, bwidth/2, bheight/2, rstride_c, pstride/2, &mv, sign, width/2, height/2, xposC, yposC);
    get_inter_prediction_chroma(pblock_v + offsetpC, ref_v + offsetrC, bwidth/2, bheight/2, rstride_c, pstride/2, &mv, sign, width/2, height/2, xposC, yposC);
  }
}

void average_blocks_all(uint8_t *rec_y, uint8_t *rec_u, uint8_t *rec_v, uint8_t *pblock0_y, uint8_t *pblock0_u, uint8_t *pblock0_v, uint8_t *pblock1_y, uint8_t *pblock1_u, uint8_t *pblock1_v, block_info_t *block_info) {
  int bwidth = block_info->block_pos.bwidth;
  int bheight = block_info->block_pos.bheight;
  int size = block_info->block_pos.size;
  int sizeY = size;
  int sizeC = size / 2;
  int i, j;

  for (i = 0; i < bheight; i++) {
    for (j = 0; j < bwidth; j++) {
      rec_y[i*sizeY + j] = (uint8_t)(((int)pblock0_y[i*sizeY + j] + (int)pblock1_y[i*sizeY + j]) >> 1);
    }
  }
  for (i = 0; i < bheight / 2; i++) {
    for (j = 0; j < bwidth / 2; j++) {
      rec_u[i*sizeC + j] = (uint8_t)(((int)pblock0_u[i*sizeC + j] + (int)pblock1_u[i*sizeC + j]) >> 1);
      rec_v[i*sizeC + j] = (uint8_t)(((int)pblock0_v[i*sizeC + j] + (int)pblock1_v[i*sizeC + j]) >> 1);
    }
  }
}

int encode_block(encoder_info_t *encoder_info, stream_t *stream, block_info_t *block_info,block_param_t *block_param)
{
  int width = encoder_info->width;
  int height = encoder_info->height;
  int size = block_info->block_pos.size;
  int yposY = block_info->block_pos.ypos;
  int xposY = block_info->block_pos.xpos;
  int yposC = yposY/2;
  int xposC = xposY/2;
  int sizeY = size;
  int sizeC = size/2;
  block_mode_t mode = block_param->mode;

  int nbits;
  cbp_t cbp;
  intra_mode_t intra_mode;

  frame_type_t frame_type = encoder_info->frame_info.frame_type;
  int enable_bipred = encoder_info->params->enable_bipred;
  int qpY = block_info->qp;
  int qpC = chroma_qp[qpY];

  /* Intermediate block variables */
  int re_use = (block_info->final_encode & 1) && !(encoder_info->params->enable_tb_split);

  if (re_use) {
    memcpy(block_info->rec_block->y, block_info->rec_block_best->y, size*size*sizeof(uint8_t));
    memcpy(block_info->rec_block->u, block_info->rec_block_best->u, size*size / 4 * sizeof(uint8_t));
    memcpy(block_info->rec_block->v, block_info->rec_block_best->v, size*size / 4 * sizeof(uint8_t));
    nbits = write_block(stream, encoder_info, block_info, block_param);
    return nbits;
  }

  uint8_t *pblock_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);

  int ref_idx = (frame_type==I_FRAME) ? 0 : block_param->ref_idx0;
  int r = encoder_info->frame_info.ref_array[ref_idx];
  yuv_frame_t *rec = encoder_info->rec;
  yuv_frame_t *ref;

  ref = NULL;

  /* Variables for bipred */
  uint8_t *pblock0_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock0_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock0_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock1_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock1_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock1_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  int r0,r1;
  yuv_frame_t *ref0;
  yuv_frame_t *ref1;

  /* Pointers to block of original pixels */
  uint8_t *org_y = block_info->org_block->y;
  uint8_t *org_u = block_info->org_block->u;
  uint8_t *org_v = block_info->org_block->v;

  /* Pointers to block of reconstructed pixels */
  uint8_t *rec_y = block_info->rec_block->y;
  uint8_t *rec_u = block_info->rec_block->u;
  uint8_t *rec_v = block_info->rec_block->v;

  int tb_split = max(0, block_param->tb_param);
  int zero_block = block_param->tb_param == -1 ? 1 : 0;
  block_param->tb_split = tb_split;
  block_param->mode = mode;

  if (mode!=MODE_INTRA) {
    ref = r>=0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
  }
  int16_t *coeffq_y = thor_alloc(2 * MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeffq_u = thor_alloc(2 * MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeffq_v = thor_alloc(2 * MAX_TR_SIZE*MAX_TR_SIZE, 16);

  if (mode==MODE_INTRA){
    intra_mode = block_param->intra_mode;
    int upright_available = get_upright_available(yposY,xposY,sizeY,width);
    int downleft_available = get_downleft_available(yposY,xposY,sizeY,height);
    uint8_t* yrec = &rec->y[yposY*rec->stride_y+xposY];
    uint8_t* urec = &rec->u[yposC*rec->stride_c+xposC];
    uint8_t* vrec = &rec->v[yposC*rec->stride_c+xposC];

    /* Predict, create residual, transform, quantize, and reconstruct.*/
    cbp.y = encode_and_reconstruct_block_intra (encoder_info, org_y,sizeY,yrec,rec->stride_y,yposY,xposY,sizeY,qpY,pblock_y,coeffq_y,rec_y,((frame_type==I_FRAME)<<1)|0,
        tb_split,encoder_info->params->rdoq,width,intra_mode,upright_available,downleft_available,encoder_info->wmatrix[qpY][0][1],encoder_info->iwmatrix[qpY][0][1]);
    cbp.u = encode_and_reconstruct_block_intra (encoder_info, org_u,sizeC,urec,rec->stride_c,yposC,xposC,sizeC,qpC,pblock_u,coeffq_u,rec_u,((frame_type==I_FRAME)<<1)|1,
        tb_split&&(size>8),encoder_info->params->rdoq,width/2,intra_mode,upright_available,downleft_available,encoder_info->wmatrix[qpY][1][1],encoder_info->iwmatrix[qpY][1][1]);
    cbp.v = encode_and_reconstruct_block_intra (encoder_info, org_v,sizeC,vrec,rec->stride_c,yposC,xposC,sizeC,qpC,pblock_v,coeffq_v,rec_v,((frame_type==I_FRAME)<<1)|1,
        tb_split&&(size>8),encoder_info->params->rdoq,width/2,intra_mode,upright_available,downleft_available,encoder_info->wmatrix[qpY][2][1],encoder_info->iwmatrix[qpY][2][1]);

    if (cbp.y) memcpy(block_param->coeff_y, coeffq_y, size*size*sizeof(uint16_t));
    if (cbp.u) memcpy(block_param->coeff_u, coeffq_u, size*size / 4 * sizeof(uint16_t));
    if (cbp.v) memcpy(block_param->coeff_v, coeffq_v, size*size / 4 * sizeof(uint16_t));

  }
  else {
    if (mode==MODE_SKIP){
      int sign;
      if (block_param->dir==2){
        r0 = encoder_info->frame_info.ref_array[block_param->ref_idx0];
        ref0 = r0>=0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
        sign = ref0->frame_num > rec->frame_num;
        get_inter_prediction_yuv(ref0, pblock0_y, pblock0_u, pblock0_v, block_info, block_param->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred,0);

        r1 = encoder_info->frame_info.ref_array[block_param->ref_idx1];
        ref1 = r1 >= 0 ? encoder_info->ref[r1] : encoder_info->interp_frames[0];
        sign = ref1->frame_num > rec->frame_num;
        get_inter_prediction_yuv(ref1, pblock1_y, pblock1_u, pblock1_v, block_info, block_param->mv_arr1, sign, encoder_info->width, encoder_info->height, enable_bipred, 0);

        average_blocks_all(rec_y, rec_u, rec_v, pblock0_y, pblock0_u, pblock0_v, pblock1_y, pblock1_u, pblock1_v, block_info);
      }
      else{
        r0 = encoder_info->frame_info.ref_array[block_param->ref_idx0];
        ref0 = r0>=0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
        sign = ref0->frame_num > rec->frame_num;
        get_inter_prediction_yuv(ref0, rec_y, rec_u, rec_v, block_info, block_param->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred,0);
      }
    }
    else if (mode==MODE_MERGE){
      int sign;
      if (block_param->dir == 2) {
        r0 = encoder_info->frame_info.ref_array[block_param->ref_idx0];
        ref0 = r0 >= 0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
        sign = ref0->frame_num > rec->frame_num;
        get_inter_prediction_yuv(ref0, pblock0_y, pblock0_u, pblock0_v, block_info, block_param->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred, 0);

        r1 = encoder_info->frame_info.ref_array[block_param->ref_idx1];
        ref1 = r1 >= 0 ? encoder_info->ref[r1] : encoder_info->interp_frames[0];
        sign = ref1->frame_num > rec->frame_num;
        get_inter_prediction_yuv(ref1, pblock1_y, pblock1_u, pblock1_v, block_info, block_param->mv_arr1, sign, encoder_info->width, encoder_info->height, enable_bipred, 0);

        average_blocks_all(pblock_y, pblock_u, pblock_v, pblock0_y, pblock0_u, pblock0_v, pblock1_y, pblock1_u, pblock1_v, block_info);
      }
      else {
        r0 = encoder_info->frame_info.ref_array[block_param->ref_idx0];
        ref0 = r0 >= 0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
        sign = ref0->frame_num > rec->frame_num;
        get_inter_prediction_yuv(ref0, pblock_y, pblock_u, pblock_v, block_info, block_param->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred, 0);
      }
    }

    else if (mode==MODE_INTER){
      r0 = encoder_info->frame_info.ref_array[block_param->ref_idx0];
      ref0 = r0 >= 0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
      int sign = ref0->frame_num > rec->frame_num;
      int split = encoder_info->params->enable_pb_split;
      get_inter_prediction_yuv(ref,pblock_y,pblock_u,pblock_v,block_info, block_param->mv_arr0,sign, encoder_info->width, encoder_info->height,enable_bipred,split);
    }
    else if (mode==MODE_BIPRED){
      int sign;
      int split = encoder_info->params->enable_pb_split;
      r0 = encoder_info->frame_info.ref_array[block_param->ref_idx0];
      ref0 = r0 >= 0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
      sign = ref0->frame_num > rec->frame_num;
      get_inter_prediction_yuv(ref0, pblock0_y, pblock0_u, pblock0_v, block_info, block_param->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred, split);
      r1 = encoder_info->frame_info.ref_array[block_param->ref_idx1];
      ref1 = r1 >= 0 ? encoder_info->ref[r1] : encoder_info->interp_frames[0];
      sign = ref1->frame_num > rec->frame_num;
      get_inter_prediction_yuv(ref1, pblock1_y, pblock1_u, pblock1_v, block_info, block_param->mv_arr1, sign, encoder_info->width, encoder_info->height, enable_bipred, split);
      average_blocks_all(pblock_y, pblock_u, pblock_v, pblock0_y, pblock0_u, pblock0_v, pblock1_y, pblock1_u, pblock1_v, block_info);

    }
    if (mode!=MODE_SKIP){
      if (zero_block){
        memcpy(rec_y,pblock_y,sizeY*sizeY*sizeof(uint8_t));
        memcpy(rec_u,pblock_u,sizeC*sizeC*sizeof(uint8_t));
        memcpy(rec_v,pblock_v,sizeC*sizeC*sizeof(uint8_t));
        cbp.y = cbp.u = cbp.v = 0;
      }
      else{
        /* Create residual, transform, quantize, and reconstruct.
        NB: coeff block type is here determined by the frame type not the mode. This is only used for quantisation optimisation */
        cbp.y = encode_and_reconstruct_block_inter (encoder_info, org_y,sizeY,sizeY,qpY,pblock_y,coeffq_y,rec_y,((frame_type==I_FRAME)<<1)|0,tb_split,encoder_info->params->rdoq,
                                                    encoder_info->wmatrix[qpY][0][0],encoder_info->iwmatrix[qpY][0][0]);
        cbp.u = encode_and_reconstruct_block_inter (encoder_info, org_u,sizeC,sizeC,qpC,pblock_u,coeffq_u,rec_u,((frame_type==I_FRAME)<<1)|1,tb_split&&(size>8),
            encoder_info->params->rdoq, encoder_info->wmatrix[qpY][1][0],encoder_info->iwmatrix[qpY][1][0]);
        cbp.v = encode_and_reconstruct_block_inter (encoder_info, org_v,sizeC,sizeC,qpC,pblock_v,coeffq_v,rec_v,((frame_type==I_FRAME)<<1)|1,tb_split&&(size>8),
            encoder_info->params->rdoq, encoder_info->wmatrix[qpY][2][0],encoder_info->iwmatrix[qpY][2][0]);

        if (cbp.y) memcpy(block_param->coeff_y, coeffq_y, size*size*sizeof(uint16_t));
        if (cbp.u) memcpy(block_param->coeff_u, coeffq_u, size*size / 4 * sizeof(uint16_t));
        if (cbp.v) memcpy(block_param->coeff_v, coeffq_v, size*size / 4 * sizeof(uint16_t));
      }
    }
    else if (mode==MODE_SKIP){
      cbp.y = cbp.u = cbp.v = 0;
    }

  }
  block_param->cbp = cbp;
  nbits = write_block(stream,encoder_info,block_info,block_param);

  if (tb_split) {
    /* Used for deblocking only (i.e. not for bitstream generation) */
    block_param->cbp.y = block_param->cbp.u = block_param->cbp.v = 1; //TODO: Do properly with respect to deblocking filter
  }

  thor_free(pblock0_y);
  thor_free(pblock0_u);
  thor_free(pblock0_v);
  thor_free(pblock1_y);
  thor_free(pblock1_u);
  thor_free(pblock1_v);
  thor_free(pblock_y);
  thor_free(pblock_u);
  thor_free(pblock_v);
  thor_free(coeffq_y);
  thor_free(coeffq_u);
  thor_free(coeffq_v);

  return nbits;
}

void copy_block_to_frame(yuv_frame_t *frame, yuv_block_t *block, block_pos_t *block_pos)
{
  int size = block_pos->size;
  int ypos = block_pos->ypos;
  int xpos = block_pos->xpos;
  int bwidth = block_pos->bwidth;
  int bheight = block_pos->bheight;
  int i,pos;
  for (i=0;i<bheight;i++){
    pos = (ypos+i) * frame->stride_y + xpos;
    memcpy(&frame->y[pos],&block->y[i*size],bwidth*sizeof(uint8_t));
  }
  for (i=0;i<bheight/2;i++){
    pos = (ypos/2+i) * frame->stride_c + xpos/2;
    memcpy(&frame->u[pos],&block->u[i*size/2],bwidth/2*sizeof(uint8_t));
    memcpy(&frame->v[pos],&block->v[i*size/2],bwidth/2*sizeof(uint8_t));
  }
}

void copy_frame_to_block(yuv_block_t *block, yuv_frame_t *frame, block_pos_t *block_pos)
{
  int size = block_pos->size;
  int ypos = block_pos->ypos;
  int xpos = block_pos->xpos;
  int bwidth = block_pos->bwidth;
  int bheight = block_pos->bheight;
  int i,pos;
  for (i=0;i<bheight;i++){
    pos = (ypos+i) * frame->stride_y + xpos;
    memcpy(&block->y[i*size],&frame->y[pos],bwidth*sizeof(uint8_t));
  }
  for (i=0;i<bheight/2;i++){
    pos = (ypos/2+i) * frame->stride_c + xpos/2;
    memcpy(&block->u[i*size/2],&frame->u[pos],bwidth/2*sizeof(uint8_t));
    memcpy(&block->v[i*size/2],&frame->v[pos],bwidth/2*sizeof(uint8_t));
  }
}

void get_mv_cand(int ypos,int xpos,int width,int height,int size,int ref_idx,deblock_data_t *deblock_data, mv_t *mvcand){

  mv_t zerovec;
  zerovec.x = 0;
  zerovec.y = 0;

  /* Parameters values measured in units of 4 pixels */
  int block_size = size/MIN_PB_SIZE;
  int block_stride = width/MIN_PB_SIZE;
  int block_posy = ypos/MIN_PB_SIZE;
  int block_posx = xpos/MIN_PB_SIZE;
  int block_index = block_posy * block_stride + block_posx;

  /* Block positions in units of 8x8 pixels */
  int up_index0 = block_index - block_stride;
  int up_index1 = block_index - block_stride + (block_size - 1)/2;
  int up_index2 = block_index - block_stride + block_size - 1;
  int left_index0 = block_index - 1;
  int left_index1 = block_index + block_stride*((block_size-1)/2) - 1;
  int left_index2 = block_index + block_stride*(block_size-1) - 1;
  int upright_index = block_index - block_stride + block_size;
  int upleft_index = block_index - block_stride - 1;
  int downleft_index = block_index + block_stride*block_size - 1;
  
  int up_available = get_up_available(ypos,xpos,size,width);
  int left_available = get_left_available(ypos,xpos,size,width);
  int upright_available = get_upright_available(ypos,xpos,size,width);
  int downleft_available = get_downleft_available(ypos,xpos,size,height);

  int U = up_available;
  int UR = upright_available;
  int L = left_available;
  int DL = downleft_available;

  mvcand[0] = zerovec;
  mvcand[1] = zerovec;
  mvcand[2] = zerovec;
  mvcand[3] = zerovec;

  if (U==0 && UR==0 && L==0 && DL==0){
    mvcand[0] = zerovec;
    mvcand[1] = zerovec;
    mvcand[2] = zerovec;
    mvcand[3] = zerovec;
  }
  else if (U==1 && UR==0 && L==0 && DL==0){
    mvcand[0] = deblock_data[up_index0].inter_pred.mv0;
    mvcand[1] = deblock_data[up_index1].inter_pred.mv0;
    mvcand[2] = deblock_data[up_index2].inter_pred.mv0;
    mvcand[3] = deblock_data[up_index2].inter_pred.mv0;
  }
  else if (U==1 && UR==1 && L==0 && DL==0){
    mvcand[0] = deblock_data[up_index0].inter_pred.mv0;
    mvcand[1] = deblock_data[up_index2].inter_pred.mv0;
    mvcand[2] = deblock_data[upright_index].inter_pred.mv0;
    mvcand[3] = deblock_data[upright_index].inter_pred.mv0;
  }
  else if (U==0 && UR==0 && L==1 && DL==0){
    mvcand[0] = deblock_data[left_index0].inter_pred.mv0;
    mvcand[1] = deblock_data[left_index1].inter_pred.mv0;
    mvcand[2] = deblock_data[left_index2].inter_pred.mv0;
    mvcand[3] = deblock_data[left_index2].inter_pred.mv0;
  }
  else if (U==1 && UR==0 && L==1 && DL==0){
    mvcand[0] = deblock_data[upleft_index].inter_pred.mv0;
    mvcand[1] = deblock_data[up_index2].inter_pred.mv0;
    mvcand[2] = deblock_data[left_index2].inter_pred.mv0;
    mvcand[3] = deblock_data[up_index0].inter_pred.mv0;
  }
 
  else if (U==1 && UR==1 && L==1 && DL==0){
    mvcand[0] = deblock_data[up_index0].inter_pred.mv0;
    mvcand[1] = deblock_data[upright_index].inter_pred.mv0;
    mvcand[2] = deblock_data[left_index2].inter_pred.mv0;
    mvcand[3] = deblock_data[left_index0].inter_pred.mv0;
  }
  else if (U==0 && UR==0 && L==1 && DL==1){
    mvcand[0] = deblock_data[left_index0].inter_pred.mv0;
    mvcand[1] = deblock_data[left_index2].inter_pred.mv0;
    mvcand[2] = deblock_data[downleft_index].inter_pred.mv0;
    mvcand[3] = deblock_data[downleft_index].inter_pred.mv0;
  }
  else if (U==1 && UR==0 && L==1 && DL==1){
    mvcand[0] = deblock_data[up_index2].inter_pred.mv0;
    mvcand[1] = deblock_data[left_index0].inter_pred.mv0;
    mvcand[2] = deblock_data[downleft_index].inter_pred.mv0;
    mvcand[3] = deblock_data[up_index0].inter_pred.mv0;
  }
  else if (U==1 && UR==1 && L==1 && DL==1){
    mvcand[0] = deblock_data[up_index0].inter_pred.mv0;
    mvcand[1] = deblock_data[upright_index].inter_pred.mv0;
    mvcand[2] = deblock_data[left_index0].inter_pred.mv0;
    mvcand[3] = deblock_data[downleft_index].inter_pred.mv0;
  }
  else{
    printf("Error in ME candidate definition\n");
  }

  /* Make sure neighbor has the same reference as the current */
  /*
  if (ref_idx != mvc[0].ref_idx) mvcand[0] = zerovec;
  if (ref_idx != mvc[1].ref_idx) mvcand[1] = zerovec;
  if (ref_idx != mvc[2].ref_idx) mvcand[2] = zerovec;
  if (ref_idx != mvc[3].ref_idx) mvcand[3] = zerovec;
  */
}

void copy_deblock_data(encoder_info_t *encoder_info, block_info_t *block_info){

  int size = block_info->block_pos.size;
  int block_posy = block_info->block_pos.ypos/MIN_PB_SIZE;
  int block_posx = block_info->block_pos.xpos/MIN_PB_SIZE;
  int block_stride = encoder_info->width/MIN_PB_SIZE;
  int block_index;
  int m,n,m0,n0,index;
  int div = size/(2*MIN_PB_SIZE);
  int bwidth =  block_info->block_pos.bwidth;
  int bheight =  block_info->block_pos.bheight;

  uint8_t tb_split = max(0,block_info->block_param.tb_param);
  part_t pb_part = block_info->block_param.mode == MODE_INTER ? block_info->block_param.pb_part : PART_NONE; //TODO: Set pb_part properly for SKIP and BIPRED

  for (m=0;m<bheight/MIN_PB_SIZE;m++){
    for (n=0;n<bwidth/MIN_PB_SIZE;n++){
      block_index = (block_posy+m)*block_stride + block_posx+n;
      m0 = div > 0 ? m/div : 0;
      n0 = div > 0 ? n/div : 0;
      index = 2*m0+n0;
      if (index > 3) printf("error: index=%4d\n",index);
      encoder_info->deblock_data[block_index].cbp = block_info->block_param.cbp;
      encoder_info->deblock_data[block_index].tb_split = tb_split;
      encoder_info->deblock_data[block_index].pb_part = pb_part;
      encoder_info->deblock_data[block_index].size = block_info->block_pos.size;
      encoder_info->deblock_data[block_index].mode = block_info->block_param.mode;
      encoder_info->deblock_data[block_index].inter_pred.mv0 = block_info->block_param.mv_arr0[index];
      encoder_info->deblock_data[block_index].inter_pred.ref_idx0 = block_info->block_param.ref_idx0;
      encoder_info->deblock_data[block_index].inter_pred.mv1 = block_info->block_param.mv_arr1[index];
      encoder_info->deblock_data[block_index].inter_pred.ref_idx1 = block_info->block_param.ref_idx1;
      encoder_info->deblock_data[block_index].inter_pred.bipred_flag = block_info->block_param.dir;
    }
  }
}

void copy_best_parameters(int size,block_info_t *block_info, block_param_t block_param){

  yuv_block_t *rec_block = block_info->rec_block;
  memcpy(block_info->rec_block_best->y, rec_block->y, size*size*sizeof(uint8_t));
  memcpy(block_info->rec_block_best->u, rec_block->u, size*size / 4 * sizeof(uint8_t));
  memcpy(block_info->rec_block_best->v, rec_block->v, size*size / 4 * sizeof(uint8_t));
  if (block_param.cbp.y) memcpy(block_info->block_param.coeff_y, block_param.coeff_y, size*size*sizeof(uint16_t));
  if (block_param.cbp.u) memcpy(block_info->block_param.coeff_u, block_param.coeff_u, size*size/4*sizeof(uint16_t));
  if (block_param.cbp.v) memcpy(block_info->block_param.coeff_v, block_param.coeff_v, size*size/4*sizeof(uint16_t));
  block_info->block_param.pb_part = block_param.pb_part;
  block_info->block_param.skip_idx = block_param.skip_idx;
  block_info->block_param.mode = block_param.mode;
  block_info->block_param.cbp = block_param.cbp;
  block_info->block_param.tb_param = block_param.tb_param;
  block_info->block_param.tb_split = block_param.tb_split;

  int mode = block_param.mode;
  int skip_or_merge_idx = block_param.skip_idx;
  if (mode == MODE_SKIP) {
    block_info->block_param.ref_idx0 = block_info->skip_candidates[skip_or_merge_idx].ref_idx0;
    block_info->block_param.ref_idx1 = block_info->skip_candidates[skip_or_merge_idx].ref_idx1;
    for (int i = 0; i < 4; i++) {
      block_info->block_param.mv_arr0[i] = block_info->skip_candidates[skip_or_merge_idx].mv0;
      block_info->block_param.mv_arr1[i] = block_info->skip_candidates[skip_or_merge_idx].mv1;
    }
    block_info->block_param.dir = block_info->skip_candidates[skip_or_merge_idx].bipred_flag;
  }
  else if (mode == MODE_MERGE) {
    block_info->block_param.ref_idx0 = block_info->merge_candidates[skip_or_merge_idx].ref_idx0;
    block_info->block_param.ref_idx1 = block_info->merge_candidates[skip_or_merge_idx].ref_idx1;
    for (int i = 0; i < 4; i++) {
      block_info->block_param.mv_arr0[i] = block_info->merge_candidates[skip_or_merge_idx].mv0;
      block_info->block_param.mv_arr1[i] = block_info->merge_candidates[skip_or_merge_idx].mv1;
    }
    block_info->block_param.dir = block_info->merge_candidates[skip_or_merge_idx].bipred_flag;
  }
  else if (mode == MODE_INTRA) {
    block_info->block_param.ref_idx0 = 0; //Note: This is necessary for derivation of mvp, mv_cand and mv_skip
    block_info->block_param.ref_idx1 = 0; //Note: This is necessary for derivation of mvp, mv_cand and mv_skip
    for (int i = 0; i < 4; i++) {
      block_info->block_param.mv_arr0[i].x = 0;
      block_info->block_param.mv_arr0[i].y = 0;
      block_info->block_param.mv_arr1[i].x = 0;
      block_info->block_param.mv_arr1[i].y = 0;
    }
    block_info->block_param.dir = -1;
    block_info->block_param.intra_mode = block_param.intra_mode;
  }
  else if (mode == MODE_INTER) {
    block_info->block_param.ref_idx0 = block_param.ref_idx0;
    block_info->block_param.ref_idx1 = block_param.ref_idx1;
    memcpy(block_info->block_param.mv_arr0, block_param.mv_arr0, 4 * sizeof(mv_t));
    memcpy(block_info->block_param.mv_arr1, block_param.mv_arr1, 4 * sizeof(mv_t));
    block_info->block_param.dir = 0;
  }
  else if (mode == MODE_BIPRED) {
    block_info->block_param.ref_idx0 = block_param.ref_idx0;
    block_info->block_param.ref_idx1 = block_param.ref_idx1;
    memcpy(block_info->block_param.mv_arr0, block_param.mv_arr0, 4 * sizeof(mv_t));
    memcpy(block_info->block_param.mv_arr1, block_param.mv_arr1, 4 * sizeof(mv_t));
    block_info->block_param.dir = 2;
  }
}

int search_bipred_prediction_params (encoder_info_t *encoder_info, block_info_t *block_info, int part, mv_t *mv_center, mv_t *mvp, int *ref_idx0, int *ref_idx1, mv_t *mv_arr0, mv_t *mv_arr1, int me_mode){

  int min_sad, sad, ref_idx;
  int r, i;
  mv_t mv;
  mv_t *mvc;

  frame_type_t frame_type = encoder_info->frame_info.frame_type;
  frame_info_t *frame_info = &encoder_info->frame_info;
  yuv_frame_t *ref;
  yuv_frame_t *rec = encoder_info->rec;
  yuv_block_t *org_block = block_info->org_block;
  uint8_t *pblock_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *org8 = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);

  int width = encoder_info->width;
  int height = encoder_info->height;
  double lambda = block_info->lambda;
  mv_t mv_all[4][4];
  int enable_bipred = 1;
  int size = block_info->block_pos.size;
  int n, num_iter, list;
#if BIPRED_PART
  num_iter = encoder_info->params->encoder_speed < 0 ? 2 : 1;
#else
  num_iter = encoder_info->params->encoder_speed == 0 ? 2 : 1;
#endif
  int min_ref_idx0 = 0, min_ref_idx1 = 0;
  mv_t min_mv_arr0[4], min_mv_arr1[4];

  if (me_mode) {
    /* Simultaneous bipred search with mv0 = -mv1 */
    int r_idx0, r_idx1;
    int ypos = block_info->block_pos.ypos;
    int xpos = block_info->block_pos.xpos;

    yuv_frame_t *ref0, *ref1;
    int r, sign = 0;
    mv_t mv;

    r_idx0 = encoder_info->frame_info.interp_ref ? 1 : 0;
    r = encoder_info->frame_info.ref_array[r_idx0];
    ref0 = r >= 0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
    assert(r>=0);

    r_idx1 = encoder_info->frame_info.interp_ref ? 2 : 1;
    r = encoder_info->frame_info.ref_array[r_idx1];
    ref1 = r >= 0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];

    int rstride = ref0->stride_y;
    int ostride = size;
    int ref_posY = ypos*rstride + xpos;
    uint8_t *ref0_y = ref0->y + ref_posY;
    uint8_t *ref1_y = ref1->y + ref_posY;

    sad = motion_estimate_bi(org_block->y, ref0_y, ref1_y, ostride, rstride, size, size, &mv, &mv_center[r_idx0], mvp, sqrt(lambda), encoder_info->params, sign, encoder_info->width, encoder_info->height, xpos, ypos, frame_info->mvcand[r_idx0], frame_info->mvcand_num + r_idx0, 2);

    *ref_idx0 = r_idx0;
    *ref_idx1 = r_idx1;
    mv_arr0[0] = mv_arr0[1] = mv_arr0[2] = mv_arr0[3] = mv;
    memcpy(mv_arr1, mv_arr0, 4 * sizeof(mv_t));

    return sad;
  }

  /* Iterative unipred search for mv0 and mv1 */

  /* Initialize using ME for ref_idx=0 */
  if (encoder_info->frame_info.frame_type == B_FRAME && encoder_info->frame_info.interp_ref == 1)
    ref_idx = 1;
  else
    ref_idx = 0;
  min_ref_idx0 = ref_idx;

  min_mv_arr0[0] = *mvp; //TODO: use actual uni-pred mv instead?
  min_mv_arr0[1] = *mvp;
  min_mv_arr0[2] = *mvp;
  min_mv_arr0[3] = *mvp;
  memcpy(min_mv_arr1, min_mv_arr0, 4 * sizeof(mv_t));

  min_sad = (1 << 30);

  for (n = 0; n < num_iter; n++) {
#if 1
    int stop = part == 0 ? 0 : 1;
    for (list = 1; list >= stop; list--) {
#else
    for (list = 1; list >= 0; list--) {
#endif
      /* Determine mv and ref_idx for the other list */
      mv = list ? min_mv_arr0[0] : min_mv_arr1[0];
      ref_idx = list ? min_ref_idx0 : min_ref_idx1;

      /* Find prediction for the other list */
      r = encoder_info->frame_info.ref_array[ref_idx];
      ref = r >= 0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];

      int sign = ref->frame_num > rec->frame_num;
      get_inter_prediction_yuv(ref, pblock_y, pblock_u, pblock_v, block_info, list ? min_mv_arr0 : min_mv_arr1, sign, encoder_info->width, encoder_info->height, enable_bipred, 1);
      /* Modify the target block based on that predition */
      for (i = 0; i < size*size; i++) {
        org8[i] = (uint8_t)clip255(2 * (int16_t)org_block->y[i] - (int16_t)pblock_y[i]);
      }

      /* Find MV and ref_idx for the current list */
      int ref_start, ref_end;
      if (encoder_info->frame_info.frame_type == P_FRAME) {
        ref_start = 0;
        ref_end = frame_info->num_ref - 1;
      }
      else {
        ref_start = list ? 1 : 0;
        ref_end = list ? 1 : 0;
        if (encoder_info->frame_info.interp_ref) {
          ref_start += 1;
          ref_end += 1;
        }
      }

      for (ref_idx = ref_start; ref_idx <= ref_end; ref_idx++) {
        r = encoder_info->frame_info.ref_array[ref_idx];
        ref = r >= 0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
        int sign = ref->frame_num > rec->frame_num;
        mv_t mvp2 = (frame_type == B_FRAME && list == 1) ? mv : *mvp;
        mvc = &mv_center[ref_idx];
        sad = (uint32_t)search_inter_prediction_params(org8, ref, &block_info->block_pos, mvc, &mvp2, mv_all[part], part, sqrt(lambda), encoder_info->params, sign, width, height, frame_info->mvcand[ref_idx], frame_info->mvcand_num + ref_idx, enable_bipred);
        for (int i = 0; i < 4; i++)
          add_mvcandidate(mv_all[part] + i, frame_info->mvcand[ref_idx], frame_info->mvcand_num + ref_idx, frame_info->mvcand_mask + ref_idx);
        if (sad < min_sad) {
          min_sad = sad;
          if (list) {
            min_ref_idx1 = ref_idx;
            memcpy(min_mv_arr1, mv_all[part], 4 * sizeof(mv_t));
          }
          else {
            min_ref_idx0 = ref_idx;
            memcpy(min_mv_arr0, mv_all[part], 4 * sizeof(mv_t));
          }
        } //if sad < min_sad
      } //for (ref_idx ..
    } //for iter ..
  } //for n=0...

  *ref_idx0 = min_ref_idx0;
  *ref_idx1 = min_ref_idx1;
  memcpy(mv_arr0, min_mv_arr0, 4 * sizeof(mv_t));
  memcpy(mv_arr1, min_mv_arr1, 4 * sizeof(mv_t));

  thor_free(pblock_y);
  thor_free(pblock_u);
  thor_free(pblock_v);
  thor_free(org8);
  return (min_sad/2); //Divide due to the way org8 is calculated
}

int mode_decision_rdo(encoder_info_t *encoder_info,block_info_t *block_info)
{
  int size = block_info->block_pos.size;
  int ypos = block_info->block_pos.ypos;
  int xpos = block_info->block_pos.xpos;
  int width = encoder_info->width;
  int height = encoder_info->height;

  stream_t *stream = encoder_info->stream;
  int enable_bipred = encoder_info->params->enable_bipred;

  yuv_frame_t *rec = encoder_info->rec;
  yuv_frame_t *ref;
  yuv_block_t *org_block = block_info->org_block;
  yuv_block_t *rec_block = block_info->rec_block;

  frame_info_t *frame_info = &encoder_info->frame_info;
  frame_type_t frame_type = frame_info->frame_type;
  double lambda = block_info->lambda;
  int nbits,tb_param;
  int min_tb_param,max_tb_param;
  mv_t mvp;
  mv_t mv_all[4][4];
  mv_t mv_center[MAX_REF_FRAMES];
  block_mode_t mode;
  intra_mode_t intra_mode;
  block_param_t tmp_block_param;

  int do_inter = 1;
  int do_intra = 1;
  int rectangular_flag = block_info->block_pos.bwidth != size || block_info->block_pos.bheight != size; //Only skip evaluation if this is true

  /* Initialize parameters that are determined using RDO */
  int best_ref_idx = 0;

  int intra_inter_sad = encoder_info->params->encoder_speed > 0 && !encoder_info->params->sync;
  
  /* Initialize cost values */
  uint32_t min_cost = MAX_UINT32;
  uint32_t sad_intra = MAX_UINT32;
  uint32_t sad_inter = MAX_UINT32;
  uint32_t cost,sad;

  /* Set reference bitstream position before doing anything at this block size */
  stream_pos_t stream_pos_ref;
  read_stream_pos(&stream_pos_ref,stream);

  /* FIND BEST MODE */

  /* Evaluate skip candidates */
  if (frame_type != I_FRAME){
    tb_param = 0;
    tmp_block_param.tb_param = 0;
    tmp_block_param.pb_part = PART_NONE;
    mode = MODE_SKIP;
    int num_skip_vec = block_info->num_skip_vec;
    int skip_idx;
    int bwidth = block_info->block_pos.bwidth;
    int bheight = block_info->block_pos.bheight;
    for (skip_idx=0;skip_idx<num_skip_vec;skip_idx++){
      tmp_block_param.skip_idx = skip_idx;
      tmp_block_param.ref_idx0 = block_info->skip_candidates[skip_idx].ref_idx0;
      tmp_block_param.ref_idx1 = block_info->skip_candidates[skip_idx].ref_idx1;
      tmp_block_param.mv_arr0[0] = block_info->skip_candidates[skip_idx].mv0;
      tmp_block_param.mv_arr1[0] = block_info->skip_candidates[skip_idx].mv1;
      tmp_block_param.dir = block_info->skip_candidates[skip_idx].bipred_flag;
      tmp_block_param.mode = mode;
      nbits = encode_block(encoder_info,stream,block_info,&tmp_block_param);
      cost = cost_calc(org_block,rec_block,size,bwidth,bheight,nbits,lambda);
      if (cost < min_cost){
        min_cost = cost;
        copy_best_parameters(size, block_info, tmp_block_param);
      }
    }
  }

  /* Evaluate inter mode */
  if (!rectangular_flag && size <= MAX_TR_SIZE) { //Only evaluate intra or inter mode if the block is square
    if (frame_type != I_FRAME){
      tb_param = 0;
      tmp_block_param.tb_param = 0;
      int part = PART_NONE;
      mode = MODE_MERGE;
      int merge_idx;
      int num_merge_vec = block_info->num_merge_vec;
      for (merge_idx=0;merge_idx<num_merge_vec;merge_idx++){
        tmp_block_param.skip_idx = merge_idx;
        tmp_block_param.ref_idx0 = block_info->merge_candidates[merge_idx].ref_idx0;
        tmp_block_param.ref_idx1 = block_info->merge_candidates[merge_idx].ref_idx1;
        tmp_block_param.mv_arr0[0] = block_info->merge_candidates[merge_idx].mv0;
        tmp_block_param.mv_arr1[0] = block_info->merge_candidates[merge_idx].mv1;
        tmp_block_param.dir = block_info->merge_candidates[merge_idx].bipred_flag;
        tmp_block_param.mode = mode;
        nbits = encode_block(encoder_info,stream,block_info,&tmp_block_param);
        cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
        if (cost < min_cost){
          min_cost = cost;
          copy_best_parameters(size, block_info, tmp_block_param);
        }
      }

      if (intra_inter_sad){
        sad_intra = search_intra_prediction_params(org_block->y,rec,&block_info->block_pos,encoder_info->width,encoder_info->height,encoder_info->frame_info.num_intra_modes,&intra_mode);      
        nbits = 2;
        sad_intra += (int)(sqrt(lambda)*(double)nbits + 0.5);
      }

      mode = MODE_INTER;
      /* Find candidate vectors to be used in ME */
      int ref_idx;

      int min_idx,max_idx;
      if (frame_info->best_ref < 0 || encoder_info->params->encoder_speed < 2 || encoder_info->params->enable_bipred || encoder_info->params->sync) {
        min_idx = 0;
        max_idx = frame_info->num_ref - 1;
      } else
        min_idx = max_idx = frame_info->best_ref;

      int worst_cost = 0, best_cost = MAX_UINT32;
      for (ref_idx=min_idx;ref_idx<=max_idx;ref_idx++){
        int r = encoder_info->frame_info.ref_array[ref_idx];
        ref = r>=0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
        tmp_block_param.ref_idx0 = ref_idx;
        tmp_block_param.ref_idx1 = ref_idx;
        mvp = get_mv_pred(ypos,xpos,width,height,size,ref_idx,encoder_info->deblock_data);
        add_mvcandidate(&mvp, frame_info->mvcand[ref_idx], frame_info->mvcand_num + ref_idx, frame_info->mvcand_mask + ref_idx);
        block_info->mvp = mvp;

        int sign = ref->frame_num > rec->frame_num;

        /* Loop over all PU partitions to do ME */
        mv_center[ref_idx] = mvp; //Center integer ME search to mvp for uni-pred, part=PART_NONE;
        sad_inter = MAX_UINT32;
        for (part=0;part<block_info->max_num_pb_part;part++){
          sad = (uint32_t)search_inter_prediction_params(org_block->y,ref,&block_info->block_pos,&mv_center[ref_idx],&mvp,mv_all[part],part,sqrt(lambda),encoder_info->params,sign,width,height,frame_info->mvcand[ref_idx],frame_info->mvcand_num + ref_idx,enable_bipred);
          for (int i = 0; i < 4; i++)
            add_mvcandidate(mv_all[part] + i, frame_info->mvcand[ref_idx], frame_info->mvcand_num + ref_idx, frame_info->mvcand_mask + ref_idx);
          mv_center[ref_idx] = mv_all[0][0];
          sad_inter = min(sad_inter,sad);
        }

        if (intra_inter_sad){
          do_inter = sad_inter < sad_intra; //TODO: encode only if sad_inter is smaller than that of previous ref_idx
          if (sad_inter < sad_intra) do_intra = 0;
        }

        if (do_inter){
          /* Loop over all PB partitions to do RDO */
          for (part=0;part<block_info->max_num_pb_part;part++){
            tmp_block_param.pb_part = part;
            memcpy(tmp_block_param.mv_arr0,mv_all[part],4*sizeof(mv_t));
            memcpy(tmp_block_param.mv_arr1,mv_all[part],4*sizeof(mv_t));
            min_tb_param = encoder_info->params->encoder_speed<1 ? -1 : 0; //tb_split == -1 means force residual to zero.
            max_tb_param = block_info->max_num_tb_part - 1;
            tmp_block_param.mode = mode;
            for (tb_param=min_tb_param; tb_param<=max_tb_param; tb_param++){
              tmp_block_param.tb_param = tb_param;
              nbits = encode_block(encoder_info,stream,block_info,&tmp_block_param);
              cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
              worst_cost = max(worst_cost, cost);
              best_cost = min(best_cost, cost);
              if (cost < min_cost){
                min_cost = cost;
                copy_best_parameters(size, block_info, tmp_block_param);
              }
            }
          } //for part=
        }
      } //for ref_idx=

      // If one ref frame was convincingly better than the other, store that ref index for later use.
      if (worst_cost && worst_cost * 3 > best_cost * 4)
        frame_info->best_ref = best_ref_idx;

      /* Start bipred ME */
      if (frame_info->num_ref > 1 && encoder_info->params->enable_bipred && do_inter) { //TODO: Should work with only one reference also
        mode = MODE_BIPRED;
#if BIPRED_PART
        int num_bi_part = block_info->max_num_pb_part;
#else
        int num_bi_part = 1;
#endif
        int ref_idx0, ref_idx1;
        mv_t mv_arr0[4], mv_arr1[4];
        min_tb_param = 0;
        max_tb_param = 0; //TODO: Support tb-split
        for (part = 0; part < num_bi_part; part++) {
          search_bipred_prediction_params(encoder_info, block_info, part, mv_center, &mvp, &ref_idx0, &ref_idx1, mv_arr0, mv_arr1,0);
          tmp_block_param.pb_part = part;
          tmp_block_param.ref_idx0 = ref_idx0;
          memcpy(tmp_block_param.mv_arr0, mv_arr0, 4 * sizeof(mv_t));
          tmp_block_param.ref_idx1 = ref_idx1;
          memcpy(tmp_block_param.mv_arr1, mv_arr1, 4 * sizeof(mv_t));
          tmp_block_param.mode = mode;
          for (tb_param = min_tb_param; tb_param <= max_tb_param; tb_param++) {
            tmp_block_param.tb_param = tb_param;
            nbits = encode_block(encoder_info, stream, block_info, &tmp_block_param);
            cost = cost_calc(org_block, rec_block, size, size, size, nbits, lambda);
            if (cost < min_cost) {
              min_cost = cost;
              copy_best_parameters(size, block_info, tmp_block_param);
            }
          } //for tb_param..
        } //for part..

        if (encoder_info->frame_info.frame_type == B_FRAME && encoder_info->params->encoder_speed == 0) {
          search_bipred_prediction_params(encoder_info, block_info, part, mv_center, &mvp, &ref_idx0, &ref_idx1, mv_arr0, mv_arr1, 1);
          tmp_block_param.pb_part = PART_NONE;
          tmp_block_param.ref_idx0 = ref_idx0;
          memcpy(tmp_block_param.mv_arr0, mv_arr0, 4 * sizeof(mv_t));
          tmp_block_param.ref_idx1 = ref_idx1;
          memcpy(tmp_block_param.mv_arr1, mv_arr1, 4 * sizeof(mv_t));
          tb_param = 0;
          tmp_block_param.tb_param = 0;
          tmp_block_param.mode = mode;
          nbits = encode_block(encoder_info, stream, block_info, &tmp_block_param);
          cost = cost_calc(org_block, rec_block, size, size, size, nbits, lambda);
          if (cost < min_cost) {
            min_cost = cost;
            copy_best_parameters(size, block_info, tmp_block_param);
          }
        }
      } //if enable_bipred
    } //if frame_type != I_FRAME

    /* Evaluate intra mode */
    mode = MODE_INTRA;
    if (do_intra) {
      min_tb_param = 0;
      max_tb_param = block_info->max_num_tb_part - 1;

      /* Find intra mode (by RDO or SAD) */
      if (encoder_info->params->intra_rdo) {
        uint32_t min_intra_cost = MAX_UINT32;
        intra_mode_t best_intra_mode = MODE_DC;
        int num_intra_modes = frame_info->num_intra_modes;
        for (intra_mode = MODE_DC; intra_mode < num_intra_modes; intra_mode++) {
          tmp_block_param.intra_mode = intra_mode;
          for (tb_param = 0; tb_param <= max_tb_param; tb_param++) {
            tmp_block_param.tb_param = tb_param;
            tmp_block_param.mode = mode;
            nbits = encode_block(encoder_info, stream, block_info, &tmp_block_param);
            cost = cost_calc(org_block, rec_block, size, size, size, nbits, lambda);
            if (cost < min_intra_cost) {
              min_intra_cost = cost;
              best_intra_mode = intra_mode;
            }
          }
        }
        intra_mode = best_intra_mode;
      }
      else {
        search_intra_prediction_params(org_block->y, rec, &block_info->block_pos, encoder_info->width, encoder_info->height, frame_info->num_intra_modes, &intra_mode);
      }

      /* Do final encoding with selected intra mode */
      tmp_block_param.intra_mode = intra_mode;
      for (tb_param = min_tb_param; tb_param <= max_tb_param; tb_param++) {
        tmp_block_param.tb_param = tb_param;
        tmp_block_param.mode = mode;
        nbits = encode_block(encoder_info, stream, block_info, &tmp_block_param);
        cost = cost_calc(org_block, rec_block, size, size, size, nbits, lambda);
        if (cost < min_cost) {
          min_cost = cost;
          copy_best_parameters(size, block_info, tmp_block_param);
        }
      }
    } //if do_intra
  } //if !rectangular_flag

  /* Rewind bitstream to reference position */
  write_stream_pos(stream,&stream_pos_ref);

  return min_cost;
}

int check_early_skip_transform_coeff (int16_t *coeff, int qp, int size, double relative_threshold)
{
  int tr_log2size = log2i(size);
  const int qsize = size;
  int cstride = size;
  int scale = gquant_table[qp%6];
  int c,flag = 0;
  int shift2 = 21 - tr_log2size + qp/6;

  /* Derive threshold from first quantizer level */
  double first_quantizer_level = (double)(1<<shift2)/(double)scale;
  int threshold = (int)(relative_threshold * first_quantizer_level);

  /* Compare each coefficient with threshold */
  for (int i = 0; i < qsize; i++){
    for (int j = 0; j < qsize; j++){
      c = (int)coeff[i*cstride+j];
      if (abs(c) > threshold)
        flag = 1;
    }
  }
  return (flag != 0);
}

int check_early_skip_sub_block (encoder_info_t *encoder_info, uint8_t *orig, int orig_stride, int size, int qp, uint8_t *pblock, float early_skip_threshold)
{
  int cbp;
  int16_t *block = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeff = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

  get_residual(block, pblock, orig, size, orig_stride);

  int fast = 0; //Core transform is never larger than 16x16 when EARLY_SKIP_BLOCK_SIZE=16
  if (size > 4){
    int16_t *tmp = thor_alloc(2*EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE/4,16);
    int i,j,i2,j2;
    int size2 = size/2;
    /* Instead of NxN transform, do a 2x2 average followed by (N/2)x(N/2) transform */
    for (i=0;i<size2;i++){
      i2 = 2*i;
      for (j=0;j<size2;j++){
        j2 = 2*j;
        tmp[i*size2+j] = (block[(i2+0)*size+(j2+0)] + block[(i2+0)*size+(j2+1)] + block[(i2+1)*size+(j2+0)] + block[(i2+1)*size+(j2+1)] + 2)>>2;
      }
    }
    transform(tmp, coeff, size2, fast);
    cbp = check_early_skip_transform_coeff(coeff, qp, size2, 0.5*early_skip_threshold);
    thor_free(tmp);
  }
  else{
    transform (block, coeff, size, fast);
    cbp = check_early_skip_transform_coeff(coeff, qp, size, early_skip_threshold);
  }

  thor_free(block);
  thor_free(coeff);
  return cbp;
}

int check_early_skip_sub_blockC (encoder_info_t *encoder_info, uint8_t *orig, int orig_stride, int size, int qp, uint8_t *pblock, float early_skip_threshold)
{
  int cbp;
  int scale = gquant_table[qp%6];
  int shift2 = 21 - 5 + qp/6;
  double first_quantizer_level = (double)(1<<shift2)/(double)scale;
  int threshold = (int)(early_skip_threshold * first_quantizer_level);
  int16_t *block = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeff = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

  get_residual(block, pblock, orig, size, orig_stride);

  cbp = 0;
  if (size==8){
#if 1
    v128 thr = v128_dup_16(threshold);
    v128 sum = v128_add_16(v128_load_aligned(block+0*size),
                           v128_load_aligned(block+1*size));
    sum = v128_add_16(sum, v128_load_aligned(block+2*size));
    sum = v128_add_16(sum, v128_load_aligned(block+3*size));
    sum = v128_add_16(sum, v128_load_aligned(block+4*size));
    sum = v128_add_16(sum, v128_load_aligned(block+5*size));
    sum = v128_add_16(sum, v128_load_aligned(block+6*size));
    sum = v128_add_16(sum, v128_load_aligned(block+7*size));
    cbp = !!v128_hadd_u8(v128_cmpgt_s16(sum, thr));
#else
    int sum,i,j;
    for (j=0;j<8;j++){
      sum = 0;
      for (i=0;i<8;i++){
        sum += block[i*size+j];
      }
      sum = sum;
      if (sum > threshold){
        cbp = 1;
      }
      if (cbp) break;
    }
#endif
  }
  else{
#if 1
    v64 sum = v64_add_16(v64_load_aligned(block+0*size),
                         v64_load_aligned(block+1*size));
    sum = v64_add_16(sum, v64_load_aligned(block+2*size));
    sum = v64_add_16(sum, v64_load_aligned(block+3*size));
    sum = v64_add_32(v64_shr_n_s32(sum, 16),
                     v64_shr_n_s32(v64_shl_n_32(sum, 16), 16));
    cbp = v64_high_s32(sum) > threshold || v64_low_s32(sum) > threshold;
#else
    int sum,i,j;
    for (j=0;j<4;j+=2){
      sum = 0;
      for (i=0;i<4;i++){
        sum += block[i*size+j];
      }
      for (i=0;i<4;i++){
        sum += block[i*size+j+1];
      }
      sum = sum;
      if (sum > threshold){
        cbp = 1;
      }
      if (cbp) break;
    }
#endif
  }

  thor_free(block);
  thor_free(coeff);
  return cbp;
}

int check_early_skip_block(encoder_info_t *encoder_info,block_info_t *block_info, block_param_t *block_param){

  int size = block_info->block_pos.size;
  int ypos = block_info->block_pos.ypos;
  int xpos = block_info->block_pos.xpos;
  int i,j;
  int significant_flag = 0;
  int size0 = min(size,EARLY_SKIP_BLOCK_SIZE);
  int qpY = block_info->qp;
  int qpC = chroma_qp[qpY];
  uint8_t *pblock = thor_alloc(EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE, 16);
  uint8_t *pblock0 = thor_alloc(EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE, 16);
  uint8_t *pblock1 = thor_alloc(EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE, 16);
  mv_t mv = block_param->mv_arr0[0];
  int ref_idx = block_param->ref_idx0;
  int r = encoder_info->frame_info.ref_array[ref_idx];
  yuv_frame_t *ref = r>=0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
  yuv_block_t *org_block = block_info->org_block; //Compact (size x size) block of original pixels

  float early_skip_threshold = encoder_info->params->early_skip_thr;
  int enable_bipred = encoder_info->params->enable_bipred;

  if (encoder_info->params->encoder_speed > 1 && size == MAX_BLOCK_SIZE)
    early_skip_threshold = 1.3*early_skip_threshold;

  if (block_param->dir==2){
    /* Loop over 8x8 (4x4) sub-blocks */
    for (i=0;i<size;i+=size0){
      for (j=0;j<size;j+=size0){
        int k,l;
        ref_idx = block_param->ref_idx0;
        r = encoder_info->frame_info.ref_array[ref_idx];
        yuv_frame_t *ref0 = r>=0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
        /* Pointer to colocated sub-block in reference frame */
        uint8_t *ref0_y = ref0->y + (ypos + i)*ref->stride_y + (xpos + j);
        uint8_t *ref0_u = ref0->u + (ypos + i)/2*ref->stride_c + (xpos + j)/2;
        uint8_t *ref0_v = ref0->v + (ypos + i)/2*ref->stride_c + (xpos + j)/2;

        ref_idx = block_param->ref_idx1;
        r = encoder_info->frame_info.ref_array[ref_idx];
        yuv_frame_t *ref1 = r>=0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
        /* Pointer to colocated sub-block in reference frame */
        uint8_t *ref1_y = ref1->y + (ypos + i)*ref->stride_y + (xpos + j);
        uint8_t *ref1_u = ref1->u + (ypos + i)/2*ref->stride_c + (xpos + j)/2;
        uint8_t *ref1_v = ref1->v + (ypos + i)/2*ref->stride_c + (xpos + j)/2;

        int sign0 = ref0->frame_num >= encoder_info->frame_info.frame_num;
        int sign1 = ref1->frame_num >= encoder_info->frame_info.frame_num;

        /* Offset for 8x8 (4x4) sub-block within compact block of original pixels */
        int block_offset_y = size*i + j;
        int block_offset_c = (size/2)*(i/2) + j/2;

        /* Y */
        mv = block_param->mv_arr0[0];
        clip_mv(&mv, ypos, xpos, encoder_info->width, encoder_info->height, size0, sign0);
        get_inter_prediction_luma (pblock0,ref0_y,size0,size0,ref->stride_y,size0,&mv,sign0,enable_bipred, encoder_info->width, encoder_info->height, xpos, ypos);
        mv = block_param->mv_arr1[0];
        clip_mv(&mv, ypos, xpos, encoder_info->width, encoder_info->height, size0, sign1);
        get_inter_prediction_luma (pblock1,ref1_y,size0,size0,ref->stride_y,size0,&mv,sign1,enable_bipred, encoder_info->width, encoder_info->height, xpos, ypos);
        for (k=0;k<size0;k++){
          for (l=0;l<size0;l++){
            pblock[k*size0+l] = (uint8_t)(((int)pblock0[k*size0+l] + (int)pblock1[k*size0+l])>>1);
          }
        }
        significant_flag = significant_flag || check_early_skip_sub_block(encoder_info, org_block->y + block_offset_y,size,size0,qpY,pblock,early_skip_threshold);

        /* U */
        mv = block_param->mv_arr0[0];
        get_inter_prediction_chroma(pblock0,ref0_u,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign0,encoder_info->width/2, encoder_info->height/2, xpos/2, ypos/2);
        mv = block_param->mv_arr1[0];
        get_inter_prediction_chroma(pblock1,ref1_u,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign1,encoder_info->width/2, encoder_info->height/2, xpos/2, ypos/2);
        for (k=0;k<size0/2;k++){
          for (l=0;l<size0/2;l++){
            pblock[k*size0/2+l] = (uint8_t)(((int)pblock0[k*size0/2+l] + (int)pblock1[k*size0/2+l])>>1);
          }
        }
        significant_flag = significant_flag || check_early_skip_sub_blockC(encoder_info, org_block->u + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);

        /* V */
        mv = block_param->mv_arr0[0];
        get_inter_prediction_chroma(pblock0,ref0_v,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign0,encoder_info->width/2, encoder_info->height/2, xpos/2, ypos/2);
        mv = block_param->mv_arr1[0];
        get_inter_prediction_chroma(pblock1,ref1_v,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign1,encoder_info->width/2, encoder_info->height/2, xpos/2, ypos/2);
        for (k=0;k<size0/2;k++){
          for (l=0;l<size0/2;l++){
            pblock[k*size0/2+l] = (uint8_t)(((int)pblock0[k*size0/2+l] + (int)pblock1[k*size0/2+l])>>1);
          }
        }
        significant_flag = significant_flag || check_early_skip_sub_blockC(encoder_info, org_block->v + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);
      } //for j
    } //for i
  }
  else{
    int sign = ref->frame_num > encoder_info->frame_info.frame_num;
    /* Loop over 8x8 (4x4) sub-blocks */
    for (i=0;i<size;i+=size0){
      for (j=0;j<size;j+=size0){

        /* Pointer to colocated sub-block in reference frame */
        uint8_t *ref_y = ref->y + (ypos + i)*ref->stride_y + (xpos + j);
        uint8_t *ref_u = ref->u + (ypos + i)/2*ref->stride_c + (xpos + j)/2;
        uint8_t *ref_v = ref->v + (ypos + i)/2*ref->stride_c + (xpos + j)/2;

        /* Offset for 8x8 (4x4) sub-block within compact block of original pixels */
        int block_offset_y = size*i + j;
        int block_offset_c = (size/2)*(i/2) + j/2;

        /* Y */
        clip_mv(&mv, ypos, xpos, encoder_info->width, encoder_info->height, size0, sign);
        get_inter_prediction_luma  (pblock,ref_y,size0,size0,ref->stride_y,size0,&mv,sign,enable_bipred, encoder_info->width, encoder_info->height, xpos, ypos);
        significant_flag = significant_flag || check_early_skip_sub_block(encoder_info, org_block->y + block_offset_y,size,size0,qpY,pblock,early_skip_threshold);

        /* U */
        get_inter_prediction_chroma(pblock,ref_u,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign, encoder_info->width/2, encoder_info->height/2, xpos/2, ypos/2);
        significant_flag = significant_flag || check_early_skip_sub_blockC(encoder_info, org_block->u + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);

        /* V */
        get_inter_prediction_chroma(pblock,ref_v,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign,encoder_info->width/2, encoder_info->height/2, xpos/2, ypos/2);
        significant_flag = significant_flag || check_early_skip_sub_blockC(encoder_info, org_block->v + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);
      }
    }
  }

  thor_free(pblock);
  thor_free(pblock0);
  thor_free(pblock1);
  return (!significant_flag);
}

int search_early_skip_candidates(encoder_info_t *encoder_info,block_info_t *block_info){

  uint32_t cost,min_cost;
  int skip_idx,nbit,tmp_early_skip_flag;
  int early_skip_flag = 0;
  //int tb_split = 0;
  int size = block_info->block_pos.size;
  int num_skip_vec = block_info->num_skip_vec;

  min_cost = MAX_UINT32;

  yuv_block_t *org_block = block_info->org_block;
  yuv_block_t *rec_block = block_info->rec_block;
  double lambda = encoder_info->frame_info.lambda;
  block_param_t tmp_block_param;

  /* Loop over all skip vector candidates */
  for (skip_idx=0; skip_idx<num_skip_vec; skip_idx++){
    /* Early skip check for this vector */
    tmp_block_param.tb_param = 0;
    tmp_block_param.skip_idx = skip_idx;
    tmp_block_param.ref_idx0 = block_info->skip_candidates[skip_idx].ref_idx0;
    tmp_block_param.ref_idx1 = block_info->skip_candidates[skip_idx].ref_idx1;
    tmp_block_param.mv_arr0[0] = block_info->skip_candidates[skip_idx].mv0;
    tmp_block_param.mv_arr1[0] = block_info->skip_candidates[skip_idx].mv1;
    tmp_block_param.dir = block_info->skip_candidates[skip_idx].bipred_flag;
    tmp_early_skip_flag = check_early_skip_block(encoder_info,block_info,&tmp_block_param);
    if (tmp_early_skip_flag){
      /* Calculate RD cost for this skip vector */
      early_skip_flag = 1;
      tmp_block_param.mode = MODE_SKIP;
      nbit = encode_block(encoder_info,encoder_info->stream,block_info,&tmp_block_param);
      cost = cost_calc(org_block,rec_block,size,size,size,nbit,lambda);
      if (cost < min_cost){
        min_cost = cost;
        copy_best_parameters(size, block_info, tmp_block_param);
      }
    }
  }
  return early_skip_flag;
}



int process_block(encoder_info_t *encoder_info,int size,int ypos,int xpos,int qp){

  int height = encoder_info->height;
  int width = encoder_info->width;
  uint32_t cost,cost_small;
  stream_t *stream = encoder_info->stream;
  double lambda = encoder_info->frame_info.lambda;
  int nbit,early_skip_flag;
  frame_type_t frame_type = encoder_info->frame_info.frame_type;

  if (ypos >= height || xpos >= width)
    return 0;

  int encode_this_size = ypos + size <= height && xpos + size <= width;
  int encode_smaller_size = size > MIN_BLOCK_SIZE*(encode_this_size && frame_type != I_FRAME && !encoder_info->params->sync && encoder_info->params->encoder_speed > 0 ? 2 : 1);
  int top_down = !encode_smaller_size && size > MIN_BLOCK_SIZE;
  int encode_rectangular_size = !encode_this_size && frame_type != I_FRAME;
  if (encode_this_size==0 && encode_smaller_size==0)
    return 0;
  cost_small = 1<<28;
  cost = 1<<28;

  /* Store bitstream state before doing anything at this block size */
  stream_pos_t stream_pos_ref;
  read_stream_pos(&stream_pos_ref,stream);

  /* Initialize some block-level parameters */
  block_info_t block_info;
  yuv_block_t *org_block = thor_alloc(sizeof(yuv_block_t),16);
  yuv_block_t *rec_block = thor_alloc(sizeof(yuv_block_t),16);
  yuv_block_t *rec_block_best = thor_alloc(sizeof(yuv_block_t),16);
  block_context_t block_context;
  find_block_contexts(ypos, xpos, height, width, size, encoder_info->deblock_data, &block_context,encoder_info->params->use_block_contexts);

  if (ypos < height && xpos < width){
    block_info.org_block = org_block;
    block_info.rec_block = rec_block;
    block_info.rec_block_best = rec_block_best;
    block_info.block_pos.size = size;
    block_info.block_pos.bwidth = min(size,width-xpos);
    block_info.block_pos.bheight = min(size,height-ypos);
    block_info.block_pos.ypos = ypos;
    block_info.block_pos.xpos = xpos;
    block_info.max_num_tb_part = (encoder_info->params->enable_tb_split==1) ? 2 : 1;
    block_info.max_num_pb_part = encoder_info->params->enable_pb_split ? 4 : 1;
    block_info.qp = qp;
    block_info.delta_qp = qp - encoder_info->frame_info.prev_qp; //TODO: clip qp to 0,51
    block_info.block_context = &block_context;
    if (encoder_info->params->max_delta_qp > 0)
      block_info.lambda = encoder_info->frame_info.lambda_coeff*squared_lambda_QP[encoder_info->frame_info.qp];
    else
      block_info.lambda = encoder_info->frame_info.lambda_coeff*squared_lambda_QP[qp];

    /* Copy original data to smaller compact block */
    copy_frame_to_block(block_info.org_block,encoder_info->orig,&block_info.block_pos);

    if (encoder_info->frame_info.frame_type != I_FRAME) {
      /* Find motion vector predictor (mvp) and skip vector candidates (mv-skip) */
      int bipred_copy = encoder_info->frame_info.interp_ref || frame_type == P_FRAME ? 0 : 1;
      inter_pred_t skip_candidates[MAX_NUM_SKIP];
      block_info.num_skip_vec = get_mv_skip(ypos, xpos, width, height, size, encoder_info->deblock_data, skip_candidates, bipred_copy);
      for (int idx = 0; idx < block_info.num_skip_vec; idx++) {
        memcpy(&block_info.skip_candidates[idx], &skip_candidates[idx], sizeof(inter_pred_t));
      }
      inter_pred_t merge_candidates[MAX_NUM_SKIP];
      block_info.num_merge_vec = get_mv_merge(ypos, xpos, width, height, size, encoder_info->deblock_data, merge_candidates);
      for (int idx = 0; idx < block_info.num_merge_vec; idx++) {
        memcpy(&block_info.merge_candidates[idx], &merge_candidates[idx], sizeof(inter_pred_t));
      }
    }
  }

  if (encode_this_size){
    YPOS = ypos;
    XPOS = xpos;

    if (encoder_info->frame_info.frame_type != I_FRAME && encoder_info->params->early_skip_thr > 0.0){

      /* Search through all skip candidates for early skip */
      block_info.final_encode = 2;

      early_skip_flag = search_early_skip_candidates(encoder_info,&block_info);

      /* Rewind stream to start position of this block size */
      write_stream_pos(stream,&stream_pos_ref);
      if (early_skip_flag){

        /* Encode block with final choice of skip_idx */
        block_info.final_encode = 3;

        block_info.block_param.mode = MODE_SKIP;
        block_info.block_param.tb_param = 0;
        nbit = encode_block(encoder_info,stream,&block_info,&block_info.block_param);

        cost = cost_calc(org_block,rec_block,size,size,size,nbit,lambda);

        /* Copy reconstructed data from smaller compact block to frame array */
        copy_block_to_frame(encoder_info->rec,rec_block,&block_info.block_pos);

        /* Store deblock information for this block to frame array */
        copy_deblock_data(encoder_info,&block_info);

        thor_free(org_block);
        thor_free(rec_block);
        thor_free(rec_block_best);
        return cost;
      }
    }
  }

  if (encode_smaller_size){
    int new_size = size/2;
    int split_flag = 1;
    block_param_t block_param;
    write_super_mode(stream, encoder_info, &block_info, &block_param, split_flag, encode_this_size);
    if (size == MAX_BLOCK_SIZE && (encoder_info->params->max_delta_qp || encoder_info->params->bitrate)) {
      write_delta_qp(stream,block_info.delta_qp);
    }
    cost_small = 0; //TODO: Why not nbit * lambda?
    cost_small += process_block(encoder_info,new_size,ypos+0*new_size,xpos+0*new_size,qp);
    cost_small += process_block(encoder_info,new_size,ypos+1*new_size,xpos+0*new_size,qp);
    cost_small += process_block(encoder_info,new_size,ypos+0*new_size,xpos+1*new_size,qp);
    cost_small += process_block(encoder_info,new_size,ypos+1*new_size,xpos+1*new_size,qp);
  }

  if (encode_this_size){
    YPOS = ypos;
    XPOS = xpos;

    /* RDO-based mode decision */
    block_info.final_encode = 0;

    cost = mode_decision_rdo(encoder_info,&block_info);

    static const uint16_t iq_8x8[52] =
      {6, 7, 8, 8, 10, 11, 12, 13, 15, 17, 19, 21, 24, 27, 30, 34,
       38, 43, 48, 54, 60, 68, 76, 86, 96, 108, 121, 136, 152, 171,
       192, 216, 242, 272, 305, 342, 384, 431, 484, 543, 610, 684,
       768, 862, 968, 1086, 1219, 1368, 1536, 1724, 1935, 2172};

    int me_threshold = size * size * iq_8x8[qp] / 8;

    if (top_down && cost > me_threshold) {
      int new_size = size/2;
      int split_flag = 1;
      block_param_t block_param;
      write_super_mode(stream, encoder_info, &block_info, &block_param, split_flag, encode_this_size);
      cost_small = 0; //TODO: Why not nbit * lambda?
      cost_small += process_block(encoder_info,new_size,ypos+0*new_size,xpos+0*new_size,qp);
      cost_small += process_block(encoder_info,new_size,ypos+1*new_size,xpos+0*new_size,qp);
      cost_small += process_block(encoder_info,new_size,ypos+0*new_size,xpos+1*new_size,qp);
      cost_small += process_block(encoder_info,new_size,ypos+1*new_size,xpos+1*new_size,qp);
    }

    if (cost <= cost_small){

      /* Rewind bitstream to reference position of this block size */
      write_stream_pos(stream,&stream_pos_ref);

      block_info.final_encode = 1;

      encode_block(encoder_info,stream,&block_info,&block_info.block_param);

      /* Copy reconstructed data from smaller compact block to frame array */
      copy_block_to_frame(encoder_info->rec, block_info.rec_block, &block_info.block_pos);

      /* Store deblock information for this block to frame array */
      copy_deblock_data(encoder_info,&block_info);
    }    
  }
  else if (encode_rectangular_size){

    YPOS = ypos;
    XPOS = xpos;

    /* Find best skip_idx */
    block_info.final_encode = 0;
    cost = mode_decision_rdo(encoder_info,&block_info);

    if (cost <= cost_small){
      /* Rewind bitstream to reference position of this block size */
      write_stream_pos(stream,&stream_pos_ref);
      block_info.final_encode = 1;
      block_info.block_param.mode = MODE_SKIP;
      block_info.block_param.tb_param = 0;
      encode_block(encoder_info,stream,&block_info,&block_info.block_param);

      /* Copy reconstructed data from smaller compact block to frame array */
      copy_block_to_frame(encoder_info->rec, block_info.rec_block, &block_info.block_pos);

      /* Store deblock information for this block to frame array */
      copy_deblock_data(encoder_info,&block_info);
    }
  }

  if (size == MAX_BLOCK_SIZE) {
    if (cost > cost_small || block_info.block_param.mode != MODE_SKIP) {
      encoder_info->frame_info.prev_qp = qp;
    }
  }

  thor_free(org_block);
  thor_free(rec_block);
  thor_free(rec_block_best);

  return min(cost,cost_small);
}


void detect_clpf(const uint8_t *rec,const uint8_t *org,int x0, int y0, int width, int height, int so,int stride, int *sum0, int *sum1)
{
  int left = x0 & ~(MAX_BLOCK_SIZE-1);
  int top = y0 & ~(MAX_BLOCK_SIZE-1);
  int right = min(width-1, left + MAX_BLOCK_SIZE-1);
  int bottom = min(height-1, top + MAX_BLOCK_SIZE-1);

  for (int y=y0;y<y0+8;y++){
    for (int x=x0;x<x0+8;x++) {
      int O = org[y*so + x];
      int X = rec[(y+0)*stride + x+0];
      int A = y == top ? X : rec[(y-1)*stride + x+0];
      int B = x == left ? X : rec[(y+0)*stride + x-1];
      int C = x == right ? X : rec[(y+0)*stride + x+1];
      int D = y == bottom ? X : rec[(y+1)*stride + x+0];
      int delta = clpf_sample(X, A, B, C, D);
      int F = X + delta;
      *sum0 += (O-X)*(O-X);
      *sum1 += (O-F)*(O-F);
    }
  }
}
