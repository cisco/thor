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
#include "putbits.h"
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

int quantize (int16_t *coeff, int16_t *coeffq, int qp, int size, int coeff_block_type, int rdoq)
{
  int intra_block = (coeff_block_type>>1) & 1;
  int chroma_flag = coeff_block_type & 1;
  int tr_log2size = log2i(size);
  int qsize = min(MAX_QUANT_SIZE,size); //Only quantize 16x16 low frequency coefficients
  int scale = gquant_table[qp%6];
  int scoeff[MAX_QUANT_SIZE*MAX_QUANT_SIZE];
  int scoeffq[MAX_QUANT_SIZE*MAX_QUANT_SIZE];
  int i,j,c,sign,offset,level,cbp,pos,last_pos,level0,abs_coeff,offset0,offset1;
  int shift2 = 21 - tr_log2size + qp/6;

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
    }
  }

  /* Find last_pos */
  offset = intra_block ? 38 : -26; //Scaled by 256 relative to quantization step size
  offset = offset<<(shift2-8);
  level = 0;
  pos = qsize*qsize-1;
  while (level==0 && pos>=0){
    c = scoeff[pos];
    level = abs((abs(c)*scale + offset))>>shift2;
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
    offset = ((level0==0 || chroma_flag) ? offset0 : offset1);
    offset = offset<<(shift2-8);
    level = (abs_coeff + offset)>>shift2;
    scoeffq[pos] = sign * level;
    cbp = cbp || (level != 0);
  }

  /* RDOQ light - adapted to coefficient encoding */
  if (cbp){
    int pos;
    int N = chroma_flag ? last_pos+1 : qsize*qsize;
    int K1,K2,K3,K4;

    for (pos=2;pos<N;pos++){
      int flag = 1;
      if (pos>2){
        if (scoeffq[pos-3] > 1) flag = 0;
      }
      if (pos>3){
        if (scoeffq[pos-4] > 1 && scoeffq[pos-3] > 0) flag = 0;
      }
      if (pos==2 && (chroma_flag==0 || last_pos >= 6)){
        flag = 0;
      }
      if (flag && scoeffq[pos-2]==0 && scoeffq[pos-1]==0 && abs(scoeffq[pos])>1){
        K1 = abs(scoeff[pos]);
        K2 = abs(scoeff[pos-1]);
        K3 = abs(scoeff[pos-2]);
        K4 = max(K2,K3);
        int threshold = (73*gdequant_table[qp%6]<<(qp/6))>>(4+tr_log2size);
        if (K1+K4 < threshold){
          scoeffq[pos] = scoeff[pos] < 0 ? -1 : 1;
        }
        else{
          if (K2 > K3)
            scoeffq[pos-1] = scoeff[pos-1] < 0 ? -1 : 1;
          else
            scoeffq[pos-2] = scoeff[pos-2] < 0 ? -1 : 1;
        }
      }
    }
  }

  for(i=0;i<qsize;i++){
    for (j=0;j<qsize;j++){
      coeffq[i*size+j] = scoeffq[zigzagptr[i*qsize+j]];
    }
  }

  if (rdoq==0)
    return (cbp != 0);

  /* RDOQ */
  int nbit = 0;
  if (cbp){
    int N = qsize*qsize;
    int bit,vlc,cn,run,maxrun,pos1;
    int vlc_adaptive=0;

    /* Decoder parameters */
    int lshift = qp / 6;
    int rshift = tr_log2size - 1;
    int scale_dec = gdequant_table[qp % 6];
    int add_dec = 1<<(rshift-1);
    int rec,org;

    /* RDOQ parameters */
    int err,min_pos=0;
    double lambda = 1.0 * squared_lambda_QP[qp] * (double)(1 << 2*(7-tr_log2size));
    uint32_t cost0=0,cost1;
    uint32_t min_cost = MAX_UINT32;

    int level_mode = 1;
    level = 1;
    pos = 0;
    while (pos <= last_pos){ //Outer loop for forward scan
      if (level_mode){
        /* Level-mode */
        vlc_adaptive = (level > 3 && chroma_flag==0) ? 1 : 0;
        while (pos <= last_pos && level > 0){
          c = scoeffq[pos];
          level = abs(c);
          bit = quote_vlc(vlc_adaptive,level);
          if (level > 0){
            bit += 1;
          }
          nbit += bit;
          if (chroma_flag==0)
            vlc_adaptive = level > 3;

          org = scoeff[pos];
          rec = ((c * scale_dec << lshift) + add_dec) >> rshift;
          err = (rec-org)*(rec-org);
          if (chroma_flag==1 && pos==0 && level==1)
            bit = 1;
          cost0 += (err + (int)(lambda * (double)bit + 0.5));
          cost1 = cost0;
          for (pos1=pos+1;pos1<N;pos1++){
            err = scoeff[pos1]*scoeff[pos1];
            cost1 += err;
          }
          /* Bit usage for EOB */
          bit = 0;
          if (pos < N-1){
            if (level > 1){
              int tmp_vlc = (level > 3 && chroma_flag==0) ? 1 : 0;
              bit += quote_vlc(tmp_vlc,0);
              if (pos < N-2){
                cn = find_code(0, 0, 0, chroma_flag, 1);
                if (chroma_flag && size <= 8){
                  vlc = 0;
                  bit += quote_vlc(vlc,cn);
                }
                else{
                  vlc = 2;
                  if (cn == 0)
                    bit += 2;
                  else
                    bit += quote_vlc(vlc,cn+1);
                }
              }
            }
            else{
              cn = find_code(0, 0, 0, chroma_flag, 1);
              if (chroma_flag && size <= 8){
                vlc = 0;
                bit += quote_vlc(vlc,cn);
              }
              else{
                vlc = 2;
                if (cn == 0)
                  bit += 2;
                else
                  bit += quote_vlc(vlc,cn+1);
              }
            }  
          }
          cost1 += (int)(lambda * (double)bit + 0.5);
          if (cost1 < min_cost){
            min_cost = cost1;
            min_pos = pos;
          }
          pos++;
        }
      }

      /* Run-mode (run-level coding) */
      maxrun = N - pos - 1;
      run = 0;
      c = 0;
      while (c==0 && pos <= last_pos){
        c = scoeffq[pos];
        if (c==0){
          run++;

          org = scoeff[pos];
          rec = 0;
          bit = 0;
          err = (rec-org)*(rec-org);
          cost0 += (err + (int)(lambda * (double)bit + 0.5));
        }
        else{
          level = abs(c);
          sign = (c < 0) ? 1 : 0;

          /* Code combined event of run and (level>1) */
          cn = find_code(run, level, maxrun, chroma_flag, 0);
          bit = 0;
          if (chroma_flag && size <= 8){
            vlc = 10;
            bit += quote_vlc(vlc,cn);
          }
          else{
            vlc = 2;
            if (cn == 0)
              bit += 2;
            else
              bit += quote_vlc(vlc,cn+1);
          }
          /* Code level and sign */
          if (level > 1){
            bit += quote_vlc(0,2*(level-2)+sign);
          }
          else{
            bit += 1;
          }
          nbit += bit;
          run = 0;

          org = scoeff[pos];
          rec = ((c * scale_dec << lshift) + add_dec) >> rshift;
          err = (rec-org)*(rec-org);
          cost0 += (err + (int)(lambda * (double)bit + 0.5));
          cost1 = cost0;
          for (pos1=pos+1;pos1<N;pos1++){
            err = scoeff[pos1]*scoeff[pos1];
            cost1 += err;
          }
          /* Bit usage for EOB */
          bit = 0;
          if (pos < N-1){
            if (level > 1){
              int tmp_vlc = (level > 3 && chroma_flag==0) ? 1 : 0;
              bit += quote_vlc(tmp_vlc,0);
              if (pos < N-2){
                cn = find_code(0, 0, 0, chroma_flag, 1);
                if (chroma_flag && size <= 8){
                  vlc = 0;
                  bit += quote_vlc(vlc,cn);
                }
                else{
                  vlc = 2;
                  if (cn == 0)
                    bit += 2;
                  else
                    bit += quote_vlc(vlc,cn+1);
                }
              }
            }
            else{
              cn = find_code(0, 0, 0, chroma_flag, 1);
              if (chroma_flag && size <= 8){
                vlc = 0;
                bit += quote_vlc(vlc,cn);
              }
              else{
                vlc = 2;
                if (cn == 0)
                  bit += 2;
                else
                  bit += quote_vlc(vlc,cn+1);
              }
            }
          }
          cost1 += (int)(lambda * (double)bit + 0.5);
          if (cost1 < min_cost){
            min_cost = cost1;
            min_pos = pos;
          }
        }
        pos++;
        vlc_adaptive = (level > 3 && chroma_flag==0) ? 1 : 0;
        level_mode = level > 1; //Set level_mode
      } //while (c==0 && pos < last_pos)
    } //while (pos <= last_pos){

#if 0
    /* Sanity check that bitcount() and write_coeff() are 100% in sync */
    if (pos < N){
      /* If terminated in level mode, code one extra zero before an EOB can be sent */
      if (level_mode){
        c = scoeffq[pos];
        level = abs(c);
        nbit += quote_vlc(vlc_adaptive,level);
        if (level > 0){
          nbit += 1;
        }
        pos++;
      }
    }

    /* EOB */
    if (pos < N){
      cn = find_code(0, 0, 0, chroma_flag, 1);
      if (chroma_flag && size <= 8){
        vlc = 0;
        nbit += quote_vlc(vlc,cn);
      }
      else{
        vlc = 2;
        if (cn == 0)
          nbit += 2;
        else
          nbit += quote_vlc(vlc,cn+1);
      }
    }

    if (chroma_flag){
      if (last_pos==0 && abs(scoeffq[0])==1)
        nbit = 2;
      else
        nbit += 1;
    }

    int start_bits,end_bits,write_bits;
    stream_t tmp_stream;
    tmp_stream.bitstream = (uint8_t *)malloc(2*MAX_QUANT_SIZE*MAX_QUANT_SIZE * sizeof(uint8_t));
    tmp_stream.bitbuf = 0;
    tmp_stream.bitrest = 32;
    tmp_stream.bytepos = 0;
    tmp_stream.bytesize = 2*MAX_QUANT_SIZE*MAX_QUANT_SIZE;
    stream_t *stream = &tmp_stream;

    start_bits = get_bit_pos(stream);
    write_coeff(stream,coeffq,size,coeff_block_type);
    end_bits = get_bit_pos(stream);
    write_bits = end_bits-start_bits;
    if (write_bits != nbit){
      printf("write_bits=%8d nbits=%8d\n",write_bits,nbit);
    }

    free (tmp_stream.bitstream);

#endif

    /* Evaluate cbp = 0 */
    cost1 = 0;
    for (pos1=0;pos1<N;pos1++){
      err = scoeff[pos1]*scoeff[pos1];
      cost1 += err;
    }
    if (cost1 < min_cost){
      min_pos = -1;
      min_cost = cost1;
    }

    if (chroma_flag){
      /* Evaluate special DC case */
      cost1 = 0;
      sign = (scoeff[0] < 0) ? 1 : 0;
      rec = ((sign * scale_dec << lshift) + add_dec) >> rshift;
      err = (scoeff[0]-rec)*(scoeff[0]-rec);
      bit = 1;
      cost1 += (err + (int)(lambda * (double)bit + 0.5));
      for (pos1=1;pos1<N;pos1++){
        err = scoeff[pos1]*scoeff[pos1];
        cost1 += err;
      }
      if (cost1 < min_cost){
        min_pos = 0;
        scoeffq[0] = sign;
      }
    }

    /* Re-evaluate CBP */
    for (pos=min_pos+1;pos<N;pos++){
      scoeffq[pos] = 0;
    }
    int flag=0;
    for (pos=0;pos<N;pos++){
      if (scoeffq[pos] != 0)
        flag = 1;
    }
    if (flag==0)
      cbp = 0;
  } //if (cbp)

  /* Inverse zigzag scan */
  for(i=0;i<qsize;i++){
    for (j=0;j<qsize;j++){
      coeffq[i*size+j] = scoeffq[zigzagptr[i*qsize+j]];
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
  int code,mvabs,mvsign;

  mvabs = abs(mv_diff_x);
  mvsign = mv_diff_x < 0 ? 1 : 0;
  code = 2*mvabs - mvsign;
  bits += quote_vlc(10,code);

  mvabs = abs(mv_diff_y);
  mvsign = mv_diff_y < 0 ? 1 : 0;
  code = 2*mvabs - mvsign;
  bits += quote_vlc(10,code);
  return bits;
}

void clip_mv(mv_t *mv_cand, int ypos, int xpos, int fwidth, int fheight, int size, int sign) {

  int max_mv_ext = PADDING_Y - 16; //max MV extension outside frame boundaries in integer pixel resolution
  int mvy, mvx;
  mvy = sign ? -mv_cand->y : mv_cand->y;
  mvx = sign ? -mv_cand->x : mv_cand->x;
  if (ypos + mvy / 4 < -max_mv_ext) mvy = 4 * (-max_mv_ext - ypos);
  if (ypos + mvy / 4 + size > fheight + max_mv_ext) mvy = 4 * (fheight + max_mv_ext - ypos - size);
  if (xpos + mvx / 4 < -max_mv_ext) mvx = 4 * (-max_mv_ext - xpos);
  if (xpos + mvx / 4 > fwidth + max_mv_ext) mvx = 4 * (fwidth + max_mv_ext - xpos - size);
  mv_cand->y = sign ? -mvy : mvy;
  mv_cand->x = sign ? -mvx : mvx;
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
      get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign,enable_bipred);
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
      get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign,enable_bipred);
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
        get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign, enable_bipred);
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
    get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign,enable_bipred);
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

#if LIMIT_INTRA_MODES
  if (num_intra_modes<8){
#else
  if (1){
#endif
    get_planar_pred(left,top,top_left,size,pblock);
    sad = sad_calc(org_y,pblock,size,size,size,size);
    if (sad < min_sad){
      *intra_mode = MODE_PLANAR;
      min_sad = sad;
    }
  }

  if (num_intra_modes == 4) { //TODO: generalize
    thor_free(pblock);
    return min_sad;
  }

  get_upleft_pred(left,top,top_left,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPLEFT;
    min_sad = sad;
  }

#if LIMIT_INTRA_MODES
#else
  get_upright_pred(top,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPRIGHT;
    min_sad = sad;
  }
#endif

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
      sad += (params->sync ? motion_estimate_sync : motion_estimate)(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,mvc,&mvp2,lambda,params,sign,fwidth,fheight,xposY,yposY,mvcand,mvcand_num, enable_bipred);
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
      sad += (params->sync ? motion_estimate_sync : motion_estimate)(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,mvc,&mvp2,lambda,params,sign,fwidth,fheight,xposY,yposY,mvcand,mvcand_num,enable_bipred);
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
      sad += (params->sync ? motion_estimate_sync : motion_estimate)(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,mvc,&mvp2,lambda,params,sign,fwidth,fheight,xposY,yposY,mvcand,mvcand_num,enable_bipred);
      mv_arr[index] = mv;
      mvp2 = mv_arr[0]; //mv predictor from inside block
    }
  }

  return sad;
}

int encode_and_reconstruct_block_intra (encoder_info_t *encoder_info, uint8_t *orig, int orig_stride, uint8_t* rec, int rec_stride, int ypos, int xpos, int size, int qp,
    uint8_t *pblock, int16_t *coeffq, uint8_t *rec_block, int coeff_type, int tb_split,int rdoq, int width, intra_mode_t intra_mode, int upright_available,int downleft_available)
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
          transform (block2, coeff, size2, encoder_info->params->encoder_speed > 1 || encoder_info->params->sync);
          cbpbit = quantize (coeff, coeffq+index, qp, size2, coeff_type, rdoq);
          if (cbpbit){
            dequantize (coeffq+index, rcoeff, qp, size2);
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

      transform (block, coeff, size, encoder_info->params->encoder_speed > 1 || encoder_info->params->sync);
      cbp = quantize (coeff, coeffq, qp, size, coeff_type, rdoq);
      if (cbp){
        dequantize (coeffq, rcoeff, qp, size);
        inverse_transform (rcoeff, rblock, size);
        reconstruct_block (rblock, pblock, rec_block, size, size);
      }
      else{
        memcpy(rec_block,pblock,size*size*sizeof(uint8_t));
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

int encode_and_reconstruct_block_inter (encoder_info_t *encoder_info, uint8_t *orig, int orig_stride, int size, int qp, uint8_t *pblock, int16_t *coeffq, uint8_t *rec, int coeff_type, int tb_split,int rdoq)
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
          transform (block2, coeff, size2, size == 64 || encoder_info->params->encoder_speed > 1 || encoder_info->params->sync);
          cbpbit = quantize (coeff, coeffq+index, qp, size2, coeff_type, rdoq);
          if (cbpbit){
            dequantize (coeffq+index, rcoeff, qp, size2);
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
      transform (block, coeff, size, (size == 64 && encoder_info->params->encoder_speed > 0) || encoder_info->params->encoder_speed > 1 || encoder_info->params->sync);
      cbp = quantize (coeff, coeffq, qp, size, coeff_type, rdoq);
      if (cbp){
        dequantize (coeffq, rcoeff, qp, size);
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
    get_inter_prediction_luma(pblock_y + offsetpY, ref_y + offsetrY, bwidth, bheight, rstride_y, pstride, &mv, sign, enable_bipred);
    get_inter_prediction_chroma(pblock_u + offsetpC, ref_u + offsetrC, bwidth/2, bheight/2, rstride_c, pstride/2, &mv, sign);
    get_inter_prediction_chroma(pblock_v + offsetpC, ref_v + offsetrC, bwidth/2, bheight/2, rstride_c, pstride/2, &mv, sign);
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

int encode_block(encoder_info_t *encoder_info, stream_t *stream, block_info_t *block_info,pred_data_t *pred_data, block_mode_t mode,int tb_param)
{
  int size = block_info->block_pos.size;
  int yposY = block_info->block_pos.ypos;
  int xposY = block_info->block_pos.xpos;
  int yposC = yposY/2;
  int xposC = xposY/2;
  int sizeY = size;
  int sizeC = size/2;

  int nbits;
  cbp_t cbp;
  intra_mode_t intra_mode;

  frame_type_t frame_type = encoder_info->frame_info.frame_type;
  int enable_bipred = encoder_info->params->enable_bipred;
  int qpY = encoder_info->frame_info.qp + block_info->delta_qp;
  int qpC = chroma_qp[qpY];

  /* Intermediate block variables */
  int re_use = (block_info->final_encode & 1) && !(encoder_info->params->enable_tb_split);
  uint8_t *pblock_y = NULL;
  uint8_t *pblock_u = NULL;
  uint8_t *pblock_v = NULL;
  if (!re_use){
    pblock_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
    pblock_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
    pblock_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  }
  int16_t *coeffq_y = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeffq_u = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeffq_v = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

  int ref_idx = (frame_type==I_FRAME) ? 0 : pred_data->ref_idx0;
  int r = encoder_info->frame_info.ref_array[ref_idx];
  yuv_frame_t *rec = encoder_info->rec;
  yuv_frame_t *ref;

  ref = NULL;

  /* Variables for bipred */
  uint8_t *pblock0_y = NULL;
  uint8_t *pblock0_u = NULL;
  uint8_t *pblock0_v = NULL;
  uint8_t *pblock1_y = NULL;
  uint8_t *pblock1_u = NULL;
  uint8_t *pblock1_v = NULL;
  if (!re_use){
    pblock0_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
    pblock0_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
    pblock0_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
    pblock1_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
    pblock1_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
    pblock1_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  }
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

  /* Store all information that is needed to produce bitstream for this block */
  write_data_t write_data;

  int zero_block = tb_param == -1 ? 1 : 0;
  int tb_split = max(0,tb_param);

  if (mode!=MODE_INTRA) {
    ref = r>=0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
  }

  write_data.mode = mode;
  write_data.size = size;
  write_data.max_num_pb_part = block_info->max_num_pb_part;
  write_data.max_num_tb_part = block_info->max_num_tb_part;
  write_data.tb_part = tb_split;
  write_data.frame_type = frame_type;
  write_data.ref_idx = pred_data->ref_idx0;
  write_data.enable_bipred = encoder_info->params->enable_bipred;
  write_data.num_ref = encoder_info->frame_info.num_ref;
  write_data.cbp = &cbp;
  write_data.coeffq_y = coeffq_y;
  write_data.coeffq_u = coeffq_u;
  write_data.coeffq_v = coeffq_v;
  write_data.max_delta_qp = encoder_info->params->max_delta_qp;
  write_data.delta_qp = block_info->delta_qp;
  write_data.block_context = block_info->block_context;
  write_data.encode_rectangular_size = yposY + size > encoder_info->height || xposY + size > encoder_info->width;
  write_data.interp_ref = encoder_info->frame_info.interp_ref;
  if (mode==MODE_SKIP){
    write_data.skip_idx = pred_data->skip_idx;
    write_data.num_skip_vec = block_info->num_skip_vec;
  }
  else if (mode==MODE_MERGE){
    write_data.skip_idx = pred_data->skip_idx;
    write_data.num_skip_vec = block_info->num_merge_vec;
    write_data.max_num_tb_part = 1; ////TODO: Support merge mode and tb-split combination
  }
  else if (mode==MODE_INTER){
    write_data.mvp = block_info->mvp;
    memcpy(write_data.mv_arr,pred_data->mv_arr0,4*sizeof(mv_t));
    write_data.pb_part = pred_data->PBpart;
    write_data.max_num_tb_part = block_info->max_num_tb_part>1 && pred_data->PBpart==PART_NONE ? 2 : 1; //Can't have PU-split and TU-split at the same time
  }
  else if (mode==MODE_INTRA){
    write_data.intra_mode = pred_data->intra_mode;
    write_data.num_intra_modes = encoder_info->frame_info.num_intra_modes;
    write_data.max_num_tb_part = block_info->max_num_tb_part;
  }
  else if (mode==MODE_BIPRED){
    write_data.mvp = block_info->mvp;
    memcpy(write_data.mv_arr0,pred_data->mv_arr0,4*sizeof(mv_t));
    memcpy(write_data.mv_arr1,pred_data->mv_arr1,4*sizeof(mv_t));
    write_data.ref_idx0 = pred_data->ref_idx0;
    write_data.ref_idx1 = pred_data->ref_idx1;
    write_data.pb_part = pred_data->PBpart;
    write_data.max_num_tb_part = 1; //TODO: Support bipred and tb-split combination
  }

  if (re_use){
    memcpy(block_info->rec_block->y,block_info->rec_block_best->y,size*size*sizeof(uint8_t));
    memcpy(block_info->rec_block->u,block_info->rec_block_best->u,size*size/4*sizeof(uint8_t));
    memcpy(block_info->rec_block->v,block_info->rec_block_best->v,size*size/4*sizeof(uint8_t));
    if (mode==MODE_SKIP || zero_block){
      cbp.y = 0;
      cbp.u = 0;
      cbp.v = 0;
    }
    else{
      cbp.y = block_info->cbp_best.y;
      cbp.u = block_info->cbp_best.u;
      cbp.v = block_info->cbp_best.v;
    }
    if (mode != MODE_SKIP) {
      memcpy(coeffq_y, block_info->coeff_y_best, size*size*sizeof(uint16_t));
      memcpy(coeffq_u, block_info->coeff_u_best, size*size / 4 * sizeof(uint16_t));
      memcpy(coeffq_v, block_info->coeff_v_best, size*size / 4 * sizeof(uint16_t));
    }
    nbits = write_block(stream,&write_data);

    if (tb_split){
      cbp.y = cbp.u = cbp.v = 1; //TODO: Do properly with respect to deblocking filter
    }

    /* Needed for array containing deblocking filter parameters */
    block_info->cbp = cbp;

    thor_free(coeffq_y);
    thor_free(coeffq_u);
    thor_free(coeffq_v);

    return nbits;
  }

  if (mode==MODE_INTRA){
    intra_mode = pred_data->intra_mode;
    int width = encoder_info->width;
    int height = encoder_info->height;
    int upright_available = get_upright_available(yposY,xposY,sizeY,width);
    int downleft_available = get_downleft_available(yposY,xposY,sizeY,height);
    uint8_t* yrec = &rec->y[yposY*rec->stride_y+xposY];
    uint8_t* urec = &rec->u[yposC*rec->stride_c+xposC];
    uint8_t* vrec = &rec->v[yposC*rec->stride_c+xposC];

    /* Predict, create residual, transform, quantize, and reconstruct.*/
    cbp.y = encode_and_reconstruct_block_intra (encoder_info, org_y,sizeY,yrec,rec->stride_y,yposY,xposY,sizeY,qpY,pblock_y,coeffq_y,rec_y,((frame_type==I_FRAME)<<1)|0,
        tb_split,encoder_info->params->rdoq,width,intra_mode,upright_available,downleft_available);
    cbp.u = encode_and_reconstruct_block_intra (encoder_info, org_u,sizeC,urec,rec->stride_c,yposC,xposC,sizeC,qpC,pblock_u,coeffq_u,rec_u,((frame_type==I_FRAME)<<1)|1,
        tb_split&&(size>8),encoder_info->params->rdoq,width/2,intra_mode,upright_available,downleft_available);
    cbp.v = encode_and_reconstruct_block_intra (encoder_info, org_v,sizeC,vrec,rec->stride_c,yposC,xposC,sizeC,qpC,pblock_v,coeffq_v,rec_v,((frame_type==I_FRAME)<<1)|1,
        tb_split&&(size>8),encoder_info->params->rdoq,width/2,intra_mode,upright_available,downleft_available);

    if (cbp.y) memcpy(block_info->coeff_y,coeffq_y,size*size*sizeof(uint16_t));
    if (cbp.u) memcpy(block_info->coeff_u,coeffq_u,size*size/4*sizeof(uint16_t));
    if (cbp.v) memcpy(block_info->coeff_v,coeffq_v,size*size/4*sizeof(uint16_t));

  }
  else {
    if (mode==MODE_SKIP){
      int sign;
      if (pred_data->dir==2){
        r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
        ref0 = r0>=0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
        sign = ref0->frame_num >= rec->frame_num;
        get_inter_prediction_yuv(ref0, pblock0_y, pblock0_u, pblock0_v, block_info, pred_data->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred,0);

        r1 = encoder_info->frame_info.ref_array[pred_data->ref_idx1];
        ref1 = r1 >= 0 ? encoder_info->ref[r1] : encoder_info->interp_frames[0];
        sign = ref1->frame_num >= rec->frame_num;
        get_inter_prediction_yuv(ref1, pblock1_y, pblock1_u, pblock1_v, block_info, pred_data->mv_arr1, sign, encoder_info->width, encoder_info->height, enable_bipred, 0);

        average_blocks_all(rec_y, rec_u, rec_v, pblock0_y, pblock0_u, pblock0_v, pblock1_y, pblock1_u, pblock1_v, block_info);
      }
      else{
        r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
        ref0 = r0>=0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
        sign = ref0->frame_num > rec->frame_num;
        get_inter_prediction_yuv(ref0, rec_y, rec_u, rec_v, block_info, pred_data->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred,0);
      }
    }
    else if (mode==MODE_MERGE){
      int sign;
      if (pred_data->dir == 2) {
        r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
        ref0 = r0 >= 0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
        sign = ref0->frame_num >= rec->frame_num;
        get_inter_prediction_yuv(ref0, pblock0_y, pblock0_u, pblock0_v, block_info, pred_data->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred, 0);

        r1 = encoder_info->frame_info.ref_array[pred_data->ref_idx1];
        ref1 = r1 >= 0 ? encoder_info->ref[r1] : encoder_info->interp_frames[0];
        sign = ref1->frame_num >= rec->frame_num;
        get_inter_prediction_yuv(ref1, pblock1_y, pblock1_u, pblock1_v, block_info, pred_data->mv_arr1, sign, encoder_info->width, encoder_info->height, enable_bipred, 0);

        average_blocks_all(pblock_y, pblock_u, pblock_v, pblock0_y, pblock0_u, pblock0_v, pblock1_y, pblock1_u, pblock1_v, block_info);
      }
      else {
        r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
        ref0 = r0 >= 0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
        sign = ref0->frame_num > rec->frame_num;
        get_inter_prediction_yuv(ref0, pblock_y, pblock_u, pblock_v, block_info, pred_data->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred, 0);
      }
    }

    else if (mode==MODE_INTER){
      r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
      ref0 = r0 >= 0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
      int sign = ref0->frame_num > rec->frame_num;
      int split = encoder_info->params->enable_pb_split;
      get_inter_prediction_yuv(ref,pblock_y,pblock_u,pblock_v,block_info, pred_data->mv_arr0,sign, encoder_info->width, encoder_info->height,enable_bipred,split);
    }
    else if (mode==MODE_BIPRED){
      int sign;
      int split = encoder_info->params->enable_pb_split;
      r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
      ref0 = r0 >= 0 ? encoder_info->ref[r0] : encoder_info->interp_frames[0];
      sign = ref0->frame_num >= rec->frame_num;
      get_inter_prediction_yuv(ref0, pblock0_y, pblock0_u, pblock0_v, block_info, pred_data->mv_arr0, sign, encoder_info->width, encoder_info->height, enable_bipred, split);
      r1 = encoder_info->frame_info.ref_array[pred_data->ref_idx1];
      ref1 = r1 >= 0 ? encoder_info->ref[r1] : encoder_info->interp_frames[0];
      sign = ref1->frame_num >= rec->frame_num;
      get_inter_prediction_yuv(ref1, pblock1_y, pblock1_u, pblock1_v, block_info, pred_data->mv_arr1, sign, encoder_info->width, encoder_info->height, enable_bipred, split);
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
        cbp.y = encode_and_reconstruct_block_inter (encoder_info, org_y,sizeY,sizeY,qpY,pblock_y,coeffq_y,rec_y,((frame_type==I_FRAME)<<1)|0,tb_split,encoder_info->params->rdoq);
        cbp.u = encode_and_reconstruct_block_inter (encoder_info, org_u,sizeC,sizeC,qpC,pblock_u,coeffq_u,rec_u,((frame_type==I_FRAME)<<1)|1,tb_split&&(size>8),encoder_info->params->rdoq);
        cbp.v = encode_and_reconstruct_block_inter (encoder_info, org_v,sizeC,sizeC,qpC,pblock_v,coeffq_v,rec_v,((frame_type==I_FRAME)<<1)|1,tb_split&&(size>8),encoder_info->params->rdoq);
        if (cbp.y) memcpy(block_info->coeff_y,coeffq_y,size*size*sizeof(uint16_t));
        if (cbp.u) memcpy(block_info->coeff_u,coeffq_u,size*size/4*sizeof(uint16_t));
        if (cbp.v) memcpy(block_info->coeff_v,coeffq_v,size*size/4*sizeof(uint16_t));
      }
    }
    else if (mode==MODE_SKIP){
      cbp.y = cbp.u = cbp.v = 0;
    }

  }


  nbits = write_block(stream,&write_data);

  if (tb_split){
    cbp.y = cbp.u = cbp.v = 1; //TODO: Do properly with respect to deblocking filter
  }

  /* Needed for array containing deblocking filter parameters */
  block_info->cbp = cbp;

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

  uint8_t tb_split = block_info->tb_param > 0;
  part_t pb_part = block_info->pred_data.mode == MODE_INTER ? block_info->pred_data.PBpart : PART_NONE; //TODO: Set PBpart properly for SKIP and BIPRED

  for (m=0;m<bheight/MIN_PB_SIZE;m++){
    for (n=0;n<bwidth/MIN_PB_SIZE;n++){
      block_index = (block_posy+m)*block_stride + block_posx+n;
      m0 = div > 0 ? m/div : 0;
      n0 = div > 0 ? n/div : 0;
      index = 2*m0+n0;
      if (index > 3) printf("error: index=%4d\n",index);
      encoder_info->deblock_data[block_index].cbp = block_info->cbp;
      encoder_info->deblock_data[block_index].tb_split = tb_split;
      encoder_info->deblock_data[block_index].pb_part = pb_part;
      encoder_info->deblock_data[block_index].size = block_info->block_pos.size;
      encoder_info->deblock_data[block_index].mode = block_info->pred_data.mode;
      encoder_info->deblock_data[block_index].inter_pred.mv0 = block_info->pred_data.mv_arr0[index];
      encoder_info->deblock_data[block_index].inter_pred.ref_idx0 = block_info->pred_data.ref_idx0;
      encoder_info->deblock_data[block_index].inter_pred.mv1 = block_info->pred_data.mv_arr1[index];
      encoder_info->deblock_data[block_index].inter_pred.ref_idx1 = block_info->pred_data.ref_idx1;
      encoder_info->deblock_data[block_index].inter_pred.bipred_flag = block_info->pred_data.dir;
    }
  }
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
  double lambda = frame_info->lambda;

  int nbits,part,tb_param;
  int min_tb_param,max_tb_param;
  mv_t mvp,zerovec;
  mv_t mv_all[4][4];
  mv_t mv_center[MAX_REF_FRAMES];
  block_mode_t mode;
  intra_mode_t intra_mode;
  pred_data_t pred_data;

  int do_inter = 1;
  int do_intra = 1;
  int rectangular_flag = block_info->block_pos.bwidth != size || block_info->block_pos.bheight != size; //Only skip evaluation if this is true

  /* Initialize parameters that are determined using RDO */
  int best_mode = MODE_SKIP;
  int best_skip_idx = 0;
  int best_tb_param = 0;
  int best_pb_part = 0;
  int best_ref_idx = 0;
  int best_skip_dir = 0; //TODO: don't call it "dir"
  mv_t best_mv_arr[4];

  /* Initialize cost values */
  uint32_t min_cost = MAX_UINT32;
  uint32_t sad_intra = MAX_UINT32;
  uint32_t sad_inter = MAX_UINT32;
  uint32_t cost,sad;

  /* Set reference bitstream position before doing anything at this block size */
  stream_pos_t stream_pos_ref;
  read_stream_pos(&stream_pos_ref,stream);

  zerovec.x = 0;
  zerovec.y = 0;

  /* FIND BEST MODE */

  /* Evaluate skip candidates */
  if (frame_type != I_FRAME){
    tb_param = 0;
    mode = MODE_SKIP;
    int num_skip_vec = block_info->num_skip_vec;
    int skip_idx;
    int bwidth = block_info->block_pos.bwidth;
    int bheight = block_info->block_pos.bheight;
    for (skip_idx=0;skip_idx<num_skip_vec;skip_idx++){
      pred_data.skip_idx = skip_idx;
      pred_data.mv_arr0[0].x = block_info->mvb_skip[skip_idx].x0;
      pred_data.mv_arr0[0].y = block_info->mvb_skip[skip_idx].y0;
      pred_data.ref_idx0 = block_info->mvb_skip[skip_idx].ref_idx0;
      pred_data.mv_arr1[0].x = block_info->mvb_skip[skip_idx].x1;
      pred_data.mv_arr1[0].y = block_info->mvb_skip[skip_idx].y1;
      pred_data.ref_idx1 = block_info->mvb_skip[skip_idx].ref_idx1;
      pred_data.dir = block_info->mvb_skip[skip_idx].dir;
      nbits = encode_block(encoder_info,stream,block_info,&pred_data,mode,tb_param);
      cost = cost_calc(org_block,rec_block,size,bwidth,bheight,nbits,lambda);
      if (cost < min_cost){
        min_cost = cost;
        best_mode = MODE_SKIP;
        best_tb_param = tb_param;
        best_skip_idx = skip_idx;
        best_skip_dir = pred_data.dir;
        memcpy(block_info->rec_block_best->y,rec_block->y,size*size*sizeof(uint8_t));
        memcpy(block_info->rec_block_best->u,rec_block->u,size*size/4*sizeof(uint8_t));
        memcpy(block_info->rec_block_best->v,rec_block->v,size*size/4*sizeof(uint8_t));
        block_info->cbp_best.y = block_info->cbp.y;
        block_info->cbp_best.u = block_info->cbp.u;
        block_info->cbp_best.v = block_info->cbp.v;
      }
    }
  }

  /* Evaluate inter mode */
  if (!rectangular_flag && size <= MAX_TR_SIZE) { //Only evaluate intra or inter mode if the block is square
    if (frame_type != I_FRAME){
      tb_param = 0;
      mode = MODE_MERGE;
      int merge_idx;
      int num_merge_vec = block_info->num_merge_vec;
      for (merge_idx=0;merge_idx<num_merge_vec;merge_idx++){
        pred_data.skip_idx = merge_idx;
        pred_data.mv_arr0[0].x = block_info->mvb_merge[merge_idx].x0;
        pred_data.mv_arr0[0].y = block_info->mvb_merge[merge_idx].y0;
        pred_data.ref_idx0 = block_info->mvb_merge[merge_idx].ref_idx0;
        pred_data.mv_arr1[0].x = block_info->mvb_merge[merge_idx].x1;
        pred_data.mv_arr1[0].y = block_info->mvb_merge[merge_idx].y1;
        pred_data.ref_idx1 = block_info->mvb_merge[merge_idx].ref_idx1;
        pred_data.dir = block_info->mvb_merge[merge_idx].dir;
        nbits = encode_block(encoder_info,stream,block_info,&pred_data,mode,tb_param);
        cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
        if (cost < min_cost){
          min_cost = cost;
          best_mode = MODE_MERGE;
          best_tb_param = tb_param;
          best_skip_idx = merge_idx;
          best_skip_dir = pred_data.dir;
          memcpy(block_info->rec_block_best->y,rec_block->y,size*size*sizeof(uint8_t));
          memcpy(block_info->rec_block_best->u,rec_block->u,size*size/4*sizeof(uint8_t));
          memcpy(block_info->rec_block_best->v,rec_block->v,size*size/4*sizeof(uint8_t));
          block_info->cbp_best.y = block_info->cbp.y;
          block_info->cbp_best.u = block_info->cbp.u;
          block_info->cbp_best.v = block_info->cbp.v;
          if (block_info->cbp.y) memcpy(block_info->coeff_y_best,block_info->coeff_y,size*size*sizeof(uint16_t));
          if (block_info->cbp.u) memcpy(block_info->coeff_u_best,block_info->coeff_u,size*size/4*sizeof(uint16_t));
          if (block_info->cbp.v) memcpy(block_info->coeff_v_best,block_info->coeff_v,size*size/4*sizeof(uint16_t));
        }
      }

      if (encoder_info->params->encoder_speed > 0 && !encoder_info->params->sync){
        sad_intra = search_intra_prediction_params(org_block->y,rec,&block_info->block_pos,encoder_info->width,encoder_info->height,encoder_info->frame_info.num_intra_modes,&intra_mode);      
        nbits = 2;
        sad_intra += (int)(sqrt(lambda)*(double)nbits + 0.5);
      }

      mode = MODE_INTER;
      /* Find candidate vectors to be used in ME */
      int ref_idx;

      int min_idx,max_idx;
      if (frame_info->best_ref < 0 || encoder_info->params->encoder_speed < 2 || encoder_info->params->sync) {
        min_idx = 0;
        max_idx = frame_info->num_ref - 1;
      } else
        min_idx = max_idx = frame_info->best_ref;

      int worst_cost = 0, best_cost = MAX_UINT32;
      for (ref_idx=min_idx;ref_idx<=max_idx;ref_idx++){
        int r = encoder_info->frame_info.ref_array[ref_idx];
        ref = r>=0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
        pred_data.ref_idx0 = ref_idx;
        mvp = get_mv_pred(ypos,xpos,width,height,size,ref_idx,encoder_info->deblock_data);
        add_mvcandidate(&mvp, frame_info->mvcand[ref_idx], frame_info->mvcand_num + ref_idx, frame_info->mvcand_mask + ref_idx);
        block_info->mvp = mvp;

        int sign = ref->frame_num >= rec->frame_num;

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

        /* Loop over all PU partitions to do RDO */
        if (encoder_info->params->encoder_speed > 0 && !encoder_info->params->sync){
          do_inter = sad_inter < sad_intra; //TODO: encode only if sad_inter is smaller than that of previous ref_idx
          if (sad_inter < sad_intra) do_intra = 0;
        }
        if (do_inter){
          for (part=0;part<block_info->max_num_pb_part;part++){
            pred_data.PBpart = part;
            memcpy(pred_data.mv_arr0,mv_all[part],4*sizeof(mv_t));
            min_tb_param = encoder_info->params->encoder_speed<1 && !encoder_info->params->sync ? -1 : 0; //tb_split == -1 means force residual to zero.
            max_tb_param = part>0 ? 0 : block_info->max_num_tb_part-1;  //Can't have TU-split and PU-split at the same time
            for (tb_param=min_tb_param; tb_param<=max_tb_param; tb_param++){
              nbits = encode_block(encoder_info,stream,block_info,&pred_data,mode,tb_param);
              cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
              worst_cost = max(worst_cost, cost);
              best_cost = min(best_cost, cost);
              if (cost < min_cost){
                min_cost = cost;
                best_mode = MODE_INTER;
                best_tb_param = tb_param;
                best_pb_part = part;
                best_ref_idx = ref_idx;
                memcpy(best_mv_arr,mv_all[part],4*sizeof(mv_t));
                memcpy(block_info->rec_block_best->y,rec_block->y,size*size*sizeof(uint8_t));
                memcpy(block_info->rec_block_best->u,rec_block->u,size*size/4*sizeof(uint8_t));
                memcpy(block_info->rec_block_best->v,rec_block->v,size*size/4*sizeof(uint8_t));
                block_info->cbp_best.y = block_info->cbp.y;
                block_info->cbp_best.u = block_info->cbp.u;
                block_info->cbp_best.v = block_info->cbp.v;
                if (block_info->cbp.y) memcpy(block_info->coeff_y_best,block_info->coeff_y,size*size*sizeof(uint16_t));
                if (block_info->cbp.u) memcpy(block_info->coeff_u_best,block_info->coeff_u,size*size/4*sizeof(uint16_t));
                if (block_info->cbp.v) memcpy(block_info->coeff_v_best,block_info->coeff_v,size*size/4*sizeof(uint16_t));
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
        int min_sad, sad;
        int r, i;
        mv_t mv;
        uint8_t *pblock_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
        uint8_t *pblock_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
        uint8_t *pblock_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
        uint8_t *org8 = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);

        int n, num_iter, list;
#if BIPRED_PART
        int num_bi_part = block_info->max_num_pb_part;
        num_iter = encoder_info->params->encoder_speed < 0 ? 2 : 1;
#else
        int num_bi_part = 1;
        num_iter = encoder_info->params->encoder_speed == 0 && !encoder_info->params->sync ? 2 : 1;
#endif
        int min_ref_idx0 = 0, min_ref_idx1 = 0;
        mv_t min_mv_arr0[4], min_mv_arr1[4];
        int opt_ref_idx0[4];
        int opt_ref_idx1[4];
        mv_t opt_mv_arr0[4][4];
        mv_t opt_mv_arr1[4][4];

        for (part = 0; part < num_bi_part; part++) {

          /* Initialize using ME for ref_idx=0 */
          if (encoder_info->frame_info.frame_type == B_FRAME && encoder_info->frame_info.interp_ref == 1)
            ref_idx = 1;
          else
            ref_idx = 0;
          min_ref_idx0 = ref_idx;

          min_mv_arr0[0] = mvp; //TODO: use actual uni-pred mv instead?
          min_mv_arr0[1] = mvp;
          min_mv_arr0[2] = mvp;
          min_mv_arr0[3] = mvp;
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
                mv_t mvp2 = (frame_type == B_FRAME && list == 1) ? mv : mvp;
                sad = (uint32_t)search_inter_prediction_params(org8, ref, &block_info->block_pos, &mv_center[ref_idx], &mvp2, mv_all[part], part, sqrt(lambda), encoder_info->params, sign, width, height, frame_info->mvcand[ref_idx], frame_info->mvcand_num + ref_idx, enable_bipred);
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
          opt_ref_idx0[part] = min_ref_idx0;
          opt_ref_idx1[part] = min_ref_idx1;
          memcpy(opt_mv_arr0[part], min_mv_arr0, 4 * sizeof(mv_t));
          memcpy(opt_mv_arr1[part], min_mv_arr1, 4 * sizeof(mv_t));
        } //for part...
        thor_free(pblock_y);
        thor_free(pblock_u);
        thor_free(pblock_v);
        thor_free(org8);

        mode = MODE_BIPRED;
        min_tb_param = 0;
        max_tb_param = 0; //TODO: Support tb-split
        int best_part = 0;

        for (part = 0; part < num_bi_part; part++) {
          pred_data.PBpart = part;
          pred_data.ref_idx0 = opt_ref_idx0[part];
          memcpy(pred_data.mv_arr0, opt_mv_arr0[part], 4 * sizeof(mv_t));
          pred_data.ref_idx1 = opt_ref_idx1[part];
          memcpy(pred_data.mv_arr1, opt_mv_arr1[part], 4 * sizeof(mv_t));
          for (tb_param = min_tb_param; tb_param <= max_tb_param; tb_param++) {
            nbits = encode_block(encoder_info, stream, block_info, &pred_data, mode, tb_param);
            cost = cost_calc(org_block, rec_block, size, size, size, nbits, lambda);
            if (cost < min_cost) {
              best_part = part;
              min_cost = cost;
              best_mode = MODE_BIPRED;
              best_tb_param = tb_param;
              memcpy(block_info->rec_block_best->y, rec_block->y, size*size*sizeof(uint8_t));
              memcpy(block_info->rec_block_best->u, rec_block->u, size*size / 4 * sizeof(uint8_t));
              memcpy(block_info->rec_block_best->v, rec_block->v, size*size / 4 * sizeof(uint8_t));
              block_info->cbp_best.y = block_info->cbp.y;
              block_info->cbp_best.u = block_info->cbp.u;
              block_info->cbp_best.v = block_info->cbp.v;
              if (block_info->cbp.y) memcpy(block_info->coeff_y_best, block_info->coeff_y, size*size*sizeof(uint16_t));
              if (block_info->cbp.u) memcpy(block_info->coeff_u_best, block_info->coeff_u, size*size / 4 * sizeof(uint16_t));
              if (block_info->cbp.v) memcpy(block_info->coeff_v_best, block_info->coeff_v, size*size / 4 * sizeof(uint16_t));
            }
          } //for tb_param..
        } //for part..
        pred_data.ref_idx0 = opt_ref_idx0[best_part];
        memcpy(pred_data.mv_arr0, opt_mv_arr0[best_part], 4 * sizeof(mv_t));
        pred_data.ref_idx1 = opt_ref_idx1[best_part];
        memcpy(pred_data.mv_arr1, opt_mv_arr1[best_part], 4 * sizeof(mv_t));
        pred_data.PBpart = best_part;
      } //if enable_bipred
    } //if frame_type != I_FRAME

    /* Evaluate intra mode */
    mode = MODE_INTRA;
    if (do_intra && encoder_info->params->intra_rdo){
      uint8_t *pblock = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
      uint32_t min_intra_cost = MAX_UINT32;
      intra_mode_t best_intra_mode = MODE_DC;
      for (intra_mode = MODE_DC; intra_mode<MAX_NUM_INTRA_MODES; intra_mode++) {

        if (encoder_info->frame_info.num_intra_modes==4 && intra_mode >= 4)
          continue;
#if LIMIT_INTRA_MODES
        if (intra_mode == MODE_PLANAR || intra_mode == MODE_UPRIGHT)
          continue;
#endif
        pred_data.intra_mode = intra_mode;
        min_tb_param = 0;
        max_tb_param = block_info->max_num_tb_part-1;
        for (tb_param=0; tb_param<=max_tb_param; tb_param++){
          nbits = encode_block(encoder_info,stream,block_info,&pred_data,mode,tb_param);
          cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
          if (cost < min_intra_cost){
            min_intra_cost = cost;
            best_intra_mode = intra_mode;
          }
        }
      }
      intra_mode = best_intra_mode;
      uint8_t* left_data = (uint8_t*)thor_alloc(2*MAX_TR_SIZE+2,16)+1;
      uint8_t* top_data = (uint8_t*)thor_alloc(2*MAX_TR_SIZE+2,16)+1;
      uint8_t top_left;

      int upright_available = get_upright_available(ypos,xpos,size,width);
      int downleft_available = get_downleft_available(ypos,xpos,size,height);

      // FIXME: ignoring TB split when calculating SAD
      make_top_and_left(left_data,top_data,&top_left,&rec->y[ypos*rec->stride_y+xpos],rec->stride_y,NULL,0,0,0,ypos,xpos,size,upright_available,downleft_available,0);
      get_intra_prediction(left_data,top_data,top_left,ypos,xpos,size,pblock,intra_mode);

      sad_intra = sad_calc(org_block->y,pblock,size,size,size,size);
      thor_free(pblock);
    }
    else{
      sad_intra = search_intra_prediction_params(org_block->y,rec,&block_info->block_pos,encoder_info->width,encoder_info->height,encoder_info->frame_info.num_intra_modes,&intra_mode);
    }
    nbits = 2;
    sad_intra += (int)(sqrt(lambda)*(double)nbits + 0.5);
    pred_data.intra_mode = intra_mode;

    min_tb_param = 0;
    max_tb_param = block_info->max_num_tb_part-1;
    if (do_intra){
      for (tb_param=min_tb_param; tb_param<=max_tb_param; tb_param++){
        nbits = encode_block(encoder_info,stream,block_info,&pred_data,mode,tb_param);
        cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
        if (cost < min_cost){
          min_cost = cost;
          best_mode = MODE_INTRA;
          best_tb_param = tb_param;
          memcpy(block_info->rec_block_best->y,rec_block->y,size*size*sizeof(uint8_t));
          memcpy(block_info->rec_block_best->u,rec_block->u,size*size/4*sizeof(uint8_t));
          memcpy(block_info->rec_block_best->v,rec_block->v,size*size/4*sizeof(uint8_t));
          block_info->cbp_best.y = block_info->cbp.y;
          block_info->cbp_best.u = block_info->cbp.u;
          block_info->cbp_best.v = block_info->cbp.v;
          if (block_info->cbp.y) memcpy(block_info->coeff_y_best,block_info->coeff_y,size*size*sizeof(uint16_t));
          if (block_info->cbp.u) memcpy(block_info->coeff_u_best,block_info->coeff_u,size*size/4*sizeof(uint16_t));
          if (block_info->cbp.v) memcpy(block_info->coeff_v_best,block_info->coeff_v,size*size/4*sizeof(uint16_t));
        }
      }
    }
  } //if !rectangular_flag


  /* Rewind bitstream to reference position */
  write_stream_pos(stream,&stream_pos_ref);

  /* Store prediction data derived through RDO */
  block_info->pred_data.mode = best_mode;
  if (best_mode == MODE_SKIP){
    block_info->pred_data.skip_idx = best_skip_idx;
    block_info->pred_data.ref_idx0 = block_info->mvb_skip[best_skip_idx].ref_idx0;
    block_info->pred_data.ref_idx1 = block_info->mvb_skip[best_skip_idx].ref_idx1;
    for (int i=0;i<4;i++){
      block_info->pred_data.mv_arr0[i].x = block_info->mvb_skip[best_skip_idx].x0;
      block_info->pred_data.mv_arr0[i].y = block_info->mvb_skip[best_skip_idx].y0;
      block_info->pred_data.mv_arr1[i].x = block_info->mvb_skip[best_skip_idx].x1;
      block_info->pred_data.mv_arr1[i].y = block_info->mvb_skip[best_skip_idx].y1;
    }
    block_info->pred_data.dir = best_skip_dir;
  }
  else if (best_mode == MODE_MERGE){
    block_info->pred_data.PBpart = PART_NONE;
    block_info->pred_data.skip_idx = best_skip_idx;
    block_info->pred_data.ref_idx0 = block_info->mvb_merge[best_skip_idx].ref_idx0;
    block_info->pred_data.ref_idx1 = block_info->mvb_merge[best_skip_idx].ref_idx1;
    for (int i=0;i<4;i++){
      block_info->pred_data.mv_arr0[i].x = block_info->mvb_merge[best_skip_idx].x0;
      block_info->pred_data.mv_arr0[i].y = block_info->mvb_merge[best_skip_idx].y0;
      block_info->pred_data.mv_arr1[i].x = block_info->mvb_merge[best_skip_idx].x1;
      block_info->pred_data.mv_arr1[i].y = block_info->mvb_merge[best_skip_idx].y1;
    }
    block_info->pred_data.dir = best_skip_dir;
  }
  else if (best_mode == MODE_INTER){
    block_info->pred_data.PBpart = best_pb_part;
    block_info->mvp = get_mv_pred(ypos,xpos,width,height,size,best_ref_idx,encoder_info->deblock_data);
    memcpy(block_info->pred_data.mv_arr0,best_mv_arr,4*sizeof(mv_t));
    memcpy(block_info->pred_data.mv_arr1,best_mv_arr,4*sizeof(mv_t));
    block_info->pred_data.ref_idx0 = best_ref_idx;
    block_info->pred_data.ref_idx1 = best_ref_idx;
    block_info->pred_data.dir = 0;
  }
  else if (best_mode == MODE_INTRA){
    block_info->pred_data.intra_mode = intra_mode;
    for (int i=0;i<4;i++){
      block_info->pred_data.mv_arr0[i] = zerovec;
      block_info->pred_data.mv_arr1[i] = zerovec;
    }
    block_info->pred_data.ref_idx0 = 0; //Note: This is necessary for derivation of mvp, mv_cand and mv_skip
    block_info->pred_data.ref_idx1 = 0; //Note: This is necessary for derivation of mvp, mv_cand and mv_skip
    block_info->pred_data.dir = -1;
  }
  else if (best_mode == MODE_BIPRED){
#if 2
    block_info->pred_data.PBpart = pred_data.PBpart;
#else
    block_info->pred_data.PBpart = PART_NONE;
#endif
    memcpy(block_info->pred_data.mv_arr0,pred_data.mv_arr0,4*sizeof(mv_t)); //Used for writing to bitstream
    memcpy(block_info->pred_data.mv_arr1,pred_data.mv_arr1,4*sizeof(mv_t)); //Used for writing to bitstream
    block_info->pred_data.ref_idx0 = pred_data.ref_idx0;
    block_info->pred_data.ref_idx1 = pred_data.ref_idx1;
    best_ref_idx = 0;                                                       //TODO: remove if ref_idx is removed as input parameter to get_mv_pred
    block_info->mvp = get_mv_pred(ypos,xpos,width,height,size,best_ref_idx,encoder_info->deblock_data);
    block_info->pred_data.dir = 2;
  }
  block_info->tb_param = best_tb_param;

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
    transform(tmp, coeff, size2, encoder_info->params->encoder_speed > 1 || encoder_info->params->sync);
    cbp = check_early_skip_transform_coeff(coeff, qp, size2, 0.5*early_skip_threshold);
    thor_free(tmp);
  }
  else{
    transform (block, coeff, size, encoder_info->params->encoder_speed > 1 || encoder_info->params->sync);
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

int check_early_skip_block(encoder_info_t *encoder_info,block_info_t *block_info, pred_data_t *pred_data){

  int size = block_info->block_pos.size;
  int ypos = block_info->block_pos.ypos;
  int xpos = block_info->block_pos.xpos;
  int i,j;
  int significant_flag = 0;
  int size0 = min(size,EARLY_SKIP_BLOCK_SIZE);
  int qpY = encoder_info->frame_info.qp + block_info->delta_qp;
  int qpC = chroma_qp[qpY];
  uint8_t *pblock = thor_alloc(EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE, 16);
  uint8_t *pblock0 = thor_alloc(EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE, 16);
  uint8_t *pblock1 = thor_alloc(EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE, 16);
  mv_t mv = pred_data->mv_arr0[0];
  int ref_idx = pred_data->ref_idx0;
  int r = encoder_info->frame_info.ref_array[ref_idx];
  yuv_frame_t *ref = r>=0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
  yuv_block_t *org_block = block_info->org_block; //Compact (size x size) block of original pixels

  float early_skip_threshold = encoder_info->params->early_skip_thr;
  int enable_bipred = encoder_info->params->enable_bipred;

  if ((encoder_info->params->encoder_speed > 1 || encoder_info->params->sync) && size == MAX_BLOCK_SIZE)
    early_skip_threshold = 1.3*early_skip_threshold;

  if (pred_data->dir==2){
    /* Loop over 8x8 (4x4) sub-blocks */
    for (i=0;i<size;i+=size0){
      for (j=0;j<size;j+=size0){
        int k,l;
        ref_idx = pred_data->ref_idx0;
        r = encoder_info->frame_info.ref_array[ref_idx];
        yuv_frame_t *ref0 = r>=0 ? encoder_info->ref[r] : encoder_info->interp_frames[0];
        /* Pointer to colocated sub-block in reference frame */
        uint8_t *ref0_y = ref0->y + (ypos + i)*ref->stride_y + (xpos + j);
        uint8_t *ref0_u = ref0->u + (ypos + i)/2*ref->stride_c + (xpos + j)/2;
        uint8_t *ref0_v = ref0->v + (ypos + i)/2*ref->stride_c + (xpos + j)/2;

        ref_idx = pred_data->ref_idx1;
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
        mv = pred_data->mv_arr0[0];
        clip_mv(&mv, ypos, xpos, encoder_info->width, encoder_info->height, size0, sign0);
        get_inter_prediction_luma (pblock0,ref0_y,size0,size0,ref->stride_y,size0,&mv,sign0,enable_bipred);
        mv = pred_data->mv_arr1[0];
        clip_mv(&mv, ypos, xpos, encoder_info->width, encoder_info->height, size0, sign1);
        get_inter_prediction_luma (pblock1,ref1_y,size0,size0,ref->stride_y,size0,&mv,sign1,enable_bipred);
        for (k=0;k<size0;k++){
          for (l=0;l<size0;l++){
            pblock[k*size0+l] = (uint8_t)(((int)pblock0[k*size0+l] + (int)pblock1[k*size0+l])>>1);
          }
        }
        significant_flag = significant_flag || check_early_skip_sub_block(encoder_info, org_block->y + block_offset_y,size,size0,qpY,pblock,early_skip_threshold);

        /* U */
        mv = pred_data->mv_arr0[0];
        get_inter_prediction_chroma(pblock0,ref0_u,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign0);
        mv = pred_data->mv_arr1[0];
        get_inter_prediction_chroma(pblock1,ref1_u,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign1);
        for (k=0;k<size0/2;k++){
          for (l=0;l<size0/2;l++){
            pblock[k*size0/2+l] = (uint8_t)(((int)pblock0[k*size0/2+l] + (int)pblock1[k*size0/2+l])>>1);
          }
        }
        significant_flag = significant_flag || check_early_skip_sub_blockC(encoder_info, org_block->u + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);

        /* V */
        mv = pred_data->mv_arr0[0];
        get_inter_prediction_chroma(pblock0,ref0_v,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign0);
        mv = pred_data->mv_arr1[0];
        get_inter_prediction_chroma(pblock1,ref1_v,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign1);
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
        get_inter_prediction_luma  (pblock,ref_y,size0,size0,ref->stride_y,size0,&mv,sign,enable_bipred);
        significant_flag = significant_flag || check_early_skip_sub_block(encoder_info, org_block->y + block_offset_y,size,size0,qpY,pblock,early_skip_threshold);

        /* U */
        get_inter_prediction_chroma(pblock,ref_u,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign);
        significant_flag = significant_flag || check_early_skip_sub_blockC(encoder_info, org_block->u + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);

        /* V */
        get_inter_prediction_chroma(pblock,ref_v,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign);
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
  int best_skip_idx = 0;
  int early_skip_flag = 0;
  int tb_split = 0;
  int size = block_info->block_pos.size;
  int num_skip_vec = block_info->num_skip_vec;
  int best_skip_dir = 0;

  min_cost = MAX_UINT32;

  yuv_block_t *org_block = block_info->org_block;
  yuv_block_t *rec_block = block_info->rec_block;
  double lambda = encoder_info->frame_info.lambda;
  pred_data_t tmp_pred_data;

  /* Loop over all skip vector candidates */
  for (skip_idx=0; skip_idx<num_skip_vec; skip_idx++){
    /* Early skip check for this vector */
    tmp_pred_data.skip_idx = skip_idx;
    tmp_pred_data.mv_arr0[0].x = block_info->mvb_skip[skip_idx].x0;
    tmp_pred_data.mv_arr0[0].y = block_info->mvb_skip[skip_idx].y0;
    tmp_pred_data.ref_idx0 = block_info->mvb_skip[skip_idx].ref_idx0;
    tmp_pred_data.mv_arr1[0].x = block_info->mvb_skip[skip_idx].x1;
    tmp_pred_data.mv_arr1[0].y = block_info->mvb_skip[skip_idx].y1;
    tmp_pred_data.ref_idx1 = block_info->mvb_skip[skip_idx].ref_idx1;
    tmp_pred_data.dir = block_info->mvb_skip[skip_idx].dir;

    tmp_early_skip_flag = check_early_skip_block(encoder_info,block_info,&tmp_pred_data);
    if (tmp_early_skip_flag){
      /* Calculate RD cost for this skip vector */
      early_skip_flag = 1;
      nbit = encode_block(encoder_info,encoder_info->stream,block_info,&tmp_pred_data,MODE_SKIP,tb_split);
      cost = cost_calc(org_block,rec_block,size,size,size,nbit,lambda);
      if (cost < min_cost){
        min_cost = cost;
        best_skip_idx = skip_idx;
        best_skip_dir = tmp_pred_data.dir;
        memcpy(block_info->rec_block_best->y,rec_block->y,size*size*sizeof(uint8_t));
        memcpy(block_info->rec_block_best->u,rec_block->u,size*size/4*sizeof(uint8_t));
        memcpy(block_info->rec_block_best->v,rec_block->v,size*size/4*sizeof(uint8_t));
        block_info->cbp_best.y = block_info->cbp.y;
        block_info->cbp_best.u = block_info->cbp.u;
        block_info->cbp_best.v = block_info->cbp.v;
      }
    }
  }

  if (early_skip_flag){
    /* Store relevant parameters to block_info */
    block_info->pred_data.skip_idx = best_skip_idx;
    block_info->pred_data.mode = MODE_SKIP;
    {
      block_info->pred_data.skip_idx = best_skip_idx;
      block_info->pred_data.mode = MODE_SKIP;
      for (int i=0;i<4;i++){
        block_info->pred_data.mv_arr0[i].x = block_info->mvb_skip[best_skip_idx].x0;
        block_info->pred_data.mv_arr0[i].y = block_info->mvb_skip[best_skip_idx].y0;
        block_info->pred_data.mv_arr1[i].x = block_info->mvb_skip[best_skip_idx].x1;
        block_info->pred_data.mv_arr1[i].y = block_info->mvb_skip[best_skip_idx].y1;
      }
      block_info->pred_data.ref_idx0 = block_info->mvb_skip[best_skip_idx].ref_idx0;
      block_info->pred_data.ref_idx1 = block_info->mvb_skip[best_skip_idx].ref_idx1;
      block_info->pred_data.dir = best_skip_dir;
      block_info->tb_param = 0;
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
  int nbit,early_skip_flag,tb_split;
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

#if TEST_AVAILABILITY
  frame_info_t *frame_info =  &encoder_info->frame_info;
  int by = (ypos%64)/8;
  int bx = (xpos%64)/8;
  int bs = size/8;
  int k,l;
  if (size==64){
    int k,l;
    for (l=0;l<9;l++){
      frame_info->ur[0][l] = (ypos > 0 && xpos + l*8 < width) ? 1 : 0;        
    }  
    for (k=1;k<9;k++){
      for (l=0;l<9;l++){
        frame_info->ur[k][l] = 0;      
      }
    }
    for (k=0;k<8;k++){
      frame_info->dl[k][0] = (xpos > 0 && ypos + k*8 < height) ? 1 : 0;
    }
    frame_info->dl[8][0] = 0;
    for (k=0;k<9;k++){
      for (l=1;l<9;l++){
        frame_info->dl[k][l] = 0;      
      }
    }
  }
#endif

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
    block_info.delta_qp = qp - encoder_info->frame_info.qp; //TODO: clip qp to 0,51
    block_info.block_context = &block_context;

    /* Copy original data to smaller compact block */
    copy_frame_to_block(block_info.org_block,encoder_info->orig,&block_info.block_pos);

    if (encoder_info->frame_info.frame_type != I_FRAME) {
      /* Find motion vector predictor (mvp) and skip vector candidates (mv-skip) */
      int bipred_copy = encoder_info->frame_info.interp_ref || frame_type == P_FRAME ? 0 : 1;
      inter_pred_t skip_candidates[MAX_NUM_SKIP];
      block_info.num_skip_vec = get_mv_skip(ypos, xpos, width, height, size, encoder_info->deblock_data, skip_candidates, bipred_copy);
      for (int idx = 0; idx < block_info.num_skip_vec; idx++) {
        inter_pred_to_mvb(&skip_candidates[idx], &block_info.mvb_skip[idx]);
      }
      inter_pred_t merge_candidates[MAX_NUM_SKIP];
      block_info.num_merge_vec = get_mv_merge(ypos, xpos, width, height, size, encoder_info->deblock_data, merge_candidates);
      for (int idx = 0; idx < block_info.num_merge_vec; idx++) {
        inter_pred_to_mvb(&merge_candidates[idx], &block_info.mvb_merge[idx]);
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

      /* Revind stream to start position of this block size */
      write_stream_pos(stream,&stream_pos_ref);
      if (early_skip_flag){

        /* Encode block with final choice of skip_idx */
        tb_split = 0;
        block_info.final_encode = 3;

        nbit = encode_block(encoder_info,stream,&block_info,&block_info.pred_data,MODE_SKIP,tb_split);

        cost = cost_calc(org_block,rec_block,size,size,size,nbit,lambda);

        /* Copy reconstructed data from smaller compact block to frame array */
        copy_block_to_frame(encoder_info->rec,rec_block,&block_info.block_pos);

        /* Store deblock information for this block to frame array */
        copy_deblock_data(encoder_info,&block_info);

#if TEST_AVAILABILITY
        for (k=by;k<by+bs;k++){
          for (l=bx;l<bx+bs;l++){
            frame_info->ur[k+1][l] = 1;
            frame_info->dl[k][l+1] = 1;
          }
        }        
#endif
        thor_free(org_block);
        thor_free(rec_block);
        thor_free(rec_block_best);
        return cost;
      }
    }
  }

  if (encode_smaller_size){
    int new_size = size/2;
    if (frame_type == I_FRAME || encode_this_size){
      int split_flag = 1;
      write_data_t write_data;
      write_data.size = size;
      write_data.block_context = block_info.block_context;
      write_data.frame_type = frame_type;
      // We can't use encode_rectangular_size here directly, because it is
      // never true for I frames. !encode_this_size is what we actually want.
      write_data.encode_rectangular_size = !encode_this_size;
      write_super_mode(stream, &write_data, split_flag);
    }
    else{
      putbits(1,0,stream); //Flag to signal either split or rectangular skip
    }
    if (size==MAX_BLOCK_SIZE && encoder_info->params->max_delta_qp){
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
#if TEST_AVAILABILITY
    int ur = get_upright_available(ypos,xpos,size,width);
    int dl = get_downleft_available(ypos,xpos,size,height);
    if (ur != frame_info->ur[by][bx+bs])
      printf("Error in upright availability: ypos=%4d xpos=%4d size=%4d bur=%4d ur=%4d\n",ypos,xpos,size,frame_info->ur[by][bx+bs],ur);
    if (dl != frame_info->dl[by+bs][bx])
      printf("Error in upright availability: ypos=%4d xpos=%4d size=%4d bur=%4d ur=%4d\n",ypos,xpos,size,frame_info->dl[by+bs][bx],dl);
    frame_info->ur[by+1][bx] = 1;
    frame_info->dl[by][bx+1] = 1; 
#endif

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
      write_data_t write_data;
      write_data.size = size;
      write_data.block_context = block_info.block_context;
      write_data.frame_type = frame_type;
      write_super_mode(stream, &write_data, split_flag);
      cost_small = 0; //TODO: Why not nbit * lambda?
      cost_small += process_block(encoder_info,new_size,ypos+0*new_size,xpos+0*new_size,qp);
      cost_small += process_block(encoder_info,new_size,ypos+1*new_size,xpos+0*new_size,qp);
      cost_small += process_block(encoder_info,new_size,ypos+0*new_size,xpos+1*new_size,qp);
      cost_small += process_block(encoder_info,new_size,ypos+1*new_size,xpos+1*new_size,qp);
    }

    if (cost <= cost_small){

      /* Revind bitstream to reference position of this block size */
      write_stream_pos(stream,&stream_pos_ref);

      block_info.final_encode = 1;

      encode_block(encoder_info,stream,&block_info,&block_info.pred_data,block_info.pred_data.mode,block_info.tb_param);

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
      /* Revind bitstream to reference position of this block size */
      write_stream_pos(stream,&stream_pos_ref);
      block_info.final_encode = 1;
      encode_block(encoder_info,stream,&block_info,&block_info.pred_data,MODE_SKIP,0);

      /* Copy reconstructed data from smaller compact block to frame array */
      copy_block_to_frame(encoder_info->rec, block_info.rec_block, &block_info.block_pos);

      /* Store deblock information for this block to frame array */
      copy_deblock_data(encoder_info,&block_info);
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
      int delta = ((A>X)+(B>X)+(C>X)+(D>X) > 2) - ((A<X)+(B<X)+(C<X)+(D<X) > 2);
      int F = X + delta;
      *sum0 += (O-X)*(O-X);
      *sum1 += (O-F)*(O-F);
    }
  }
}
