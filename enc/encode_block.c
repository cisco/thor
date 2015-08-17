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
#include "encode_config.h"
#include "snr.h"
#include "mainenc.h"
#include "write_bits.h"
#include "putbits.h"
#include "putvlc.h"
#include "transform.h"
#include "common_block.h"
#include "inter_prediction.h"
#include "intra_prediction.h"
#include "simd.h"
#include "enc_kernels.h"

int YPOS,XPOS;

extern int chroma_qp[52];
extern int zigzag16[16];
extern int zigzag64[64];
extern int zigzag256[256];
extern uint16_t gquant_table[6];
extern uint16_t gdequant_table[6];
extern double squared_lambda_QP [52];

int quantize (int16_t *coeff, int16_t *coeffq, int qp, int size, int frame_type, int chroma_flag, int rdoq)
{
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
  offset = frame_type==I_FRAME ? 38 : -26; //Scaled by 256 relative to quantization step size
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

  offset0 = frame_type==I_FRAME ? 102 : 51; //Scaled by 256 relative to quantization step size
  offset1 = frame_type==I_FRAME ? 115 : 90; //Scaled by 256 relative to quantization step size
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
    write_coeff(stream,coeffq,size,chroma_flag);
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

int sad_calc(uint8_t *a, uint8_t *b, int astride, int bstride, int width, int height)
{
  int i,j,sad = 0;

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

int motion_estimate(uint8_t *orig, uint8_t *ref, int size, int stride_r, int width, int height, mv_t *mv, mv_t *mvp, mv_t *mvcand, double lambda,int encoder_speed, int sign){
  int k,l,sad,range,step;
  uint32_t min_sad;
  uint8_t *rf = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  mv_t mv_cand;
  mv_t mv_opt;
  mv_t mv_ref;
#if TWO_MVP
#else
  int16_t mv_diff_y,mv_diff_x;
#endif
  /* Initialize */
  mv_opt.y = mv_opt.x = 0;
  min_sad = MAX_UINT32;

  /* Telsecope search from 8x8 grid to 1/4 x 1/4 grid around rounded mvp */
  mv_ref.y = (((mvp->y) + 2)>>2)<<2;
  mv_ref.x = (((mvp->x) + 2)>>2)<<2;
  step = 32;

  while (step > 0){
    range = encoder_speed < 2 ? 2*step : step;
    for (k=-range;k<=range;k+=step){
      for (l=-range;l<=range;l+=step){

#if 3
        if (step<32 && k==0 && l==0) continue; //Center position was investigated at previous step
#endif

#if 4
        int exitFlag=0;
        if (encoder_speed > 1 && step==1){
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
#endif

        mv_cand.y = mv_ref.y + k;
        mv_cand.x = mv_ref.x + l;
        get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign);
        sad = sad_calc(orig,rf,size,width,width,height);
#if TWO_MVP
        int bit0 = quote_mv_bits(mv_cand.y - mvcand[6].y,mv_cand.x - mvcand[6].x);
        int bit1 = quote_mv_bits(mv_cand.y - mvcand[7].y,mv_cand.x - mvcand[7].x);
        int bit = min(bit0,bit1);
        sad += (int)(lambda * (double)bit + 0.5);
#else
        mv_diff_y = mv_cand.y - mvp->y;
        mv_diff_x = mv_cand.x - mvp->x;
        sad += (int)(lambda * (double)quote_mv_bits(mv_diff_y,mv_diff_x) + 0.5);
#endif
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
  for (int idx=0;idx<6;idx++){
    mv_cand = mvcand[idx];
    get_inter_prediction_luma(rf,ref,width,height,stride_r,width,&mv_cand, sign);
    sad = sad_calc(orig,rf,size,width,width,height);
#if TWO_MVP
    int bit0 = quote_mv_bits(mv_cand.y - mvcand[6].y,mv_cand.x - mvcand[6].x);
    int bit1 = quote_mv_bits(mv_cand.y - mvcand[7].y,mv_cand.x - mvcand[7].x);
    int bit = min(bit0,bit1);
    sad += (int)(lambda * (double)bit + 0.5);
#else
    mv_diff_y = mv_cand.y - mvp->y;
    mv_diff_x = mv_cand.x - mvp->x;
    sad += (int)(lambda * (double)quote_mv_bits(mv_diff_y,mv_diff_x) + 0.5);
#endif
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
  return cost;
}

int search_intra_prediction_params(uint8_t *org_y,yuv_frame_t *rec,block_pos_t *block_pos,int width,int height,int num_intra_modes,intra_mode_t *intra_mode)
{
  int size = block_pos->size;
  int yposY = block_pos->ypos;
  int xposY = block_pos->xpos;
  int sad,min_sad;
  uint8_t pblock[MAX_BLOCK_SIZE*MAX_BLOCK_SIZE];

  /* Search for intra modes */
  min_sad = (1<<30);
  *intra_mode = MODE_DC;

  get_dc_pred(rec->y,yposY,xposY,rec->stride_y,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_DC;
    min_sad = sad;
  }
  get_hor_pred(rec->y,yposY,xposY,rec->stride_y,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_HOR;
    min_sad = sad;
  }

  get_ver_pred(rec->y,yposY,xposY,rec->stride_y,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_VER;
    min_sad = sad;
  }

#if LIMIT_INTRA_MODES
  if (num_intra_modes<8){
    get_planar_pred(rec->y,yposY,xposY,rec->stride_y,size,pblock);
    sad = sad_calc(org_y,pblock,size,size,size,size);
    if (sad < min_sad){
      *intra_mode = MODE_PLANAR;
      min_sad = sad;
    }
  }
#else
  get_planar_pred(rec->y,yposY,xposY,rec->stride_y,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_PLANAR;
    min_sad = sad;
  }
#endif

  int upright_available = get_upright_available(yposY,xposY,size,width);

  if (num_intra_modes == 4) //TODO: generalize
    return min_sad;

  get_upleft_pred(rec->y,yposY,xposY,rec->stride_y,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPLEFT;
    min_sad = sad;
  }

#if LIMIT_INTRA_MODES
#else
  get_upright_pred(rec->y,yposY,xposY,rec->stride_y,size,width,pblock,upright_available);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPRIGHT;
    min_sad = sad;
  }
#endif

  get_upupright_pred(rec->y,yposY,xposY,rec->stride_y,size,width,pblock,upright_available);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPUPRIGHT;
    min_sad = sad;
  }

  get_upupleft_pred(rec->y,yposY,xposY,rec->stride_y,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPUPLEFT;
    min_sad = sad;
  }

  get_upleftleft_pred(rec->y,yposY,xposY,rec->stride_y,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_UPLEFTLEFT;
    min_sad = sad;
  }

  get_downleftleft_pred(rec->y,yposY,xposY,rec->stride_y,size,pblock);
  sad = sad_calc(org_y,pblock,size,size,size,size);
  if (sad < min_sad){
    *intra_mode = MODE_DOWNLEFTLEFT;
    min_sad = sad;
  }
  return min_sad;
}

int search_inter_prediction_params(uint8_t *org_y,yuv_frame_t *ref,block_pos_t *block_pos,mv_t *mvp, mv_t *mvcand, mv_t *mv_arr, part_t part, double lambda, int encoder_speed, int sign)
{
  int size = block_pos->size;
  int yposY = block_pos->ypos;
  int xposY = block_pos->xpos;
  int ref_posY = ref->offset_y + yposY*ref->stride_y + xposY;
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
    sad +=  motion_estimate(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,&mvp2,mvcand,lambda,encoder_speed,sign);
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
      sad +=  motion_estimate(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,&mvp2,mvcand,lambda,encoder_speed,sign);
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
      sad +=  motion_estimate(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,&mvp2,mvcand,lambda,encoder_speed,sign);
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
      sad +=  motion_estimate(org_y+offset_o,ref_y+offset_r,ostride,rstride,width,height,&mv,&mvp2,mvcand,lambda,encoder_speed,sign);
      mv_arr[index] = mv;
      mvp2 = mv_arr[0]; //mv predictor from inside block
    }
  }

  return sad;
}

int encode_and_reconstruct_block (encoder_info_t *encoder_info, uint8_t *orig, int orig_stride, int size, int qp, uint8_t *pblock, int16_t *coeffq, uint8_t *rec, int frame_type, int chroma_flag,int tb_split,int rdoq)
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
          transform (block2, coeff, size2, encoder_info->params->encoder_speed > 1);
          cbpbit = quantize (coeff, coeffq+index, qp, size2, frame_type, chroma_flag, rdoq);
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
      transform (block, coeff, size, encoder_info->params->encoder_speed > 1);
      cbp = quantize (coeff, coeffq, qp, size, frame_type, chroma_flag, rdoq);
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
  mv_t mv;
  intra_mode_t intra_mode;

  frame_type_t frame_type = encoder_info->frame_info.frame_type;
  int qpY = encoder_info->frame_info.qp + block_info->delta_qp;
  int qpC = chroma_qp[qpY];

  /* Intermediate block variables */
  uint8_t *pblock_y = thor_alloc(MAX_TR_SIZE*MAX_TR_SIZE, 16);
  uint8_t *pblock_u = thor_alloc(MAX_TR_SIZE*MAX_TR_SIZE, 16);
  uint8_t *pblock_v = thor_alloc(MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeffq_y = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeffq_u = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeffq_v = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

  int ref_idx = (frame_type==I_FRAME) ? 0 : pred_data->ref_idx0;
  int r = encoder_info->frame_info.ref_array[ref_idx];
  yuv_frame_t *rec = encoder_info->rec;
  yuv_frame_t *ref;
  uint8_t *ref_y, *ref_u, *ref_v;
  int ref_posY, ref_posC;

  /* Variables for bipred */
  uint8_t *pblock0_y = thor_alloc(MAX_TR_SIZE*MAX_TR_SIZE, 16);
  uint8_t *pblock0_u = thor_alloc(MAX_TR_SIZE*MAX_TR_SIZE, 16);
  uint8_t *pblock0_v = thor_alloc(MAX_TR_SIZE*MAX_TR_SIZE, 16);
  uint8_t *pblock1_y = thor_alloc(MAX_TR_SIZE*MAX_TR_SIZE, 16);
  uint8_t *pblock1_u = thor_alloc(MAX_TR_SIZE*MAX_TR_SIZE, 16);
  uint8_t *pblock1_v = thor_alloc(MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int r0,r1;
  yuv_frame_t *ref0;
  yuv_frame_t *ref1;
  uint8_t *ref0_y, *ref0_u, *ref0_v;
  uint8_t *ref1_y, *ref1_u, *ref1_v;

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
    ref = encoder_info->ref[r];
    /* Calculate position in padded reference frame */
    ref_posY = ref->offset_y + yposY*ref->stride_y + xposY;
    ref_posC = ref->offset_c + yposC*ref->stride_c + xposC;

    /* Pointers to colocated block position in reference frame */
    ref_y = ref->y + ref_posY; //TODO: Collapse with ref0
    ref_u = ref->u + ref_posC;
    ref_v = ref->v + ref_posC;
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
#if NEW_BLOCK_STRUCTURE
    write_data.max_num_tb_part = block_info->max_num_tb_part;
#else
    write_data.max_num_tb_part = block_info->max_num_tb_part>1 && pred_data->PBpart==PART_NONE ? 2 : 1; //Can't have PU-split and TU-split at the same time
#endif
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

  if (mode==MODE_SKIP){
    if (pred_data->dir==2){
      int bwidth = block_info->block_pos.bwidth;
      int bheight = block_info->block_pos.bheight;

      r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
      ref0 = encoder_info->ref[r0];
      ref0_y = ref0->y + ref_posY;
      ref0_u = ref0->u + ref_posC;
      ref0_v = ref0->v + ref_posC;

      r1 = encoder_info->frame_info.ref_array[pred_data->ref_idx1];
      ref1 = encoder_info->ref[r1];
      ref1_y = ref1->y + ref_posY;
      ref1_u = ref1->u + ref_posC;
      ref1_v = ref1->v + ref_posC;

      int sign0 = ref0->frame_num > rec->frame_num;
      int sign1 = ref1->frame_num > rec->frame_num;

      mv = pred_data->mv_arr0[0];
      get_inter_prediction_luma  (pblock0_y, ref0_y, bwidth,   bheight,   ref->stride_y, sizeY, &mv, sign0);
      get_inter_prediction_chroma(pblock0_u, ref0_u, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign0);
      get_inter_prediction_chroma(pblock0_v, ref0_v, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign0);
      mv = pred_data->mv_arr1[0];
      get_inter_prediction_luma  (pblock1_y, ref1_y, bwidth,   bheight,   ref->stride_y, sizeY, &mv, sign1);
      get_inter_prediction_chroma(pblock1_u, ref1_u, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign1);
      get_inter_prediction_chroma(pblock1_v, ref1_v, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign1);
      int i,j;
      for (i=0;i<bheight;i++){
        for (j=0;j<bwidth;j++){
          rec_y[i*sizeY+j] = (uint8_t)(((int)pblock0_y[i*sizeY+j] + (int)pblock1_y[i*sizeY+j])>>1);
        }
      }
      for (i=0;i<bheight/2;i++){
        for (j=0;j<bwidth/2;j++){
          rec_u[i*sizeC+j] = (uint8_t)(((int)pblock0_u[i*sizeC+j] + (int)pblock1_u[i*sizeC+j])>>1);
          rec_v[i*sizeC+j] = (uint8_t)(((int)pblock0_v[i*sizeC+j] + (int)pblock1_v[i*sizeC+j])>>1);
        }
      }
    }
    else{
      mv = pred_data->mv_arr0[0];
      int bwidth = block_info->block_pos.bwidth;
      int bheight = block_info->block_pos.bheight;
      r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
      ref0 = encoder_info->ref[r0];
      int sign = ref0->frame_num > rec->frame_num;
      get_inter_prediction_luma  (rec_y,ref_y,bwidth,bheight,ref->stride_y,sizeY,&mv,sign);
      get_inter_prediction_chroma(rec_u,ref_u,bwidth/2,bheight/2,ref->stride_c,sizeC,&mv,sign);
      get_inter_prediction_chroma(rec_v,ref_v,bwidth/2,bheight/2,ref->stride_c,sizeC,&mv,sign);
    }
  }
  else if (mode==MODE_MERGE){
    if (pred_data->dir==2){
      r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
      ref0 = encoder_info->ref[r0];
      ref0_y = ref0->y + ref_posY;
      ref0_u = ref0->u + ref_posC;
      ref0_v = ref0->v + ref_posC;

      r1 = encoder_info->frame_info.ref_array[pred_data->ref_idx1];
      ref1 = encoder_info->ref[r1];
      ref1_y = ref1->y + ref_posY;
      ref1_u = ref1->u + ref_posC;
      ref1_v = ref1->v + ref_posC;

      int sign0 = ref0->frame_num > rec->frame_num;
      int sign1 = ref1->frame_num > rec->frame_num;

      mv = pred_data->mv_arr0[0];
      get_inter_prediction_luma  (pblock0_y,ref0_y,sizeY,sizeY,ref->stride_y,sizeY,&mv,sign0);
      get_inter_prediction_chroma(pblock0_u,ref0_u,sizeC,sizeC,ref->stride_c,sizeC,&mv,sign0);
      get_inter_prediction_chroma(pblock0_v,ref0_v,sizeC,sizeC,ref->stride_c,sizeC,&mv,sign0);
      mv = pred_data->mv_arr1[0];
      get_inter_prediction_luma  (pblock1_y,ref1_y,sizeY,sizeY,ref->stride_y,sizeY,&mv,sign1);
      get_inter_prediction_chroma(pblock1_u,ref1_u,sizeC,sizeC,ref->stride_c,sizeC,&mv,sign1);
      get_inter_prediction_chroma(pblock1_v,ref1_v,sizeC,sizeC,ref->stride_c,sizeC,&mv,sign1);
      int i,j;
      for (i=0;i<sizeY;i++){
        for (j=0;j<sizeY;j++){
          pblock_y[i*sizeY+j] = (uint8_t)(((int)pblock0_y[i*sizeY+j] + (int)pblock1_y[i*sizeY+j])>>1);
        }
      }
      for (i=0;i<sizeC;i++){
        for (j=0;j<sizeC;j++){
          pblock_u[i*sizeC+j] = (uint8_t)(((int)pblock0_u[i*sizeC+j] + (int)pblock1_u[i*sizeC+j])>>1);
          pblock_v[i*sizeC+j] = (uint8_t)(((int)pblock0_v[i*sizeC+j] + (int)pblock1_v[i*sizeC+j])>>1);
        }
      }
    }
    else{
      r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
      ref0 = encoder_info->ref[r0];
      int sign = ref0->frame_num > rec->frame_num;
      mv = pred_data->mv_arr0[0];
      get_inter_prediction_luma  (pblock_y,ref_y,sizeY,sizeY,ref->stride_y,sizeY,&mv,sign);
      get_inter_prediction_chroma(pblock_u,ref_u,sizeC,sizeC,ref->stride_c,sizeC,&mv,sign);
      get_inter_prediction_chroma(pblock_v,ref_v,sizeC,sizeC,ref->stride_c,sizeC,&mv,sign);
    }
  }
  else if (mode==MODE_INTRA){
    intra_mode = pred_data->intra_mode;
    int width = encoder_info->width;
    int upright_available = get_upright_available(yposY,xposY,sizeY,width);
    get_intra_prediction(rec->y,yposY,xposY,rec->stride_y,sizeY,width,pblock_y,intra_mode,upright_available);
    get_intra_prediction(rec->u,yposC,xposC,rec->stride_c,sizeC,width/2,pblock_u,intra_mode,upright_available);
    get_intra_prediction(rec->v,yposC,xposC,rec->stride_c,sizeC,width/2,pblock_v,intra_mode,upright_available);    
  }
  else if (mode==MODE_INTER){
    int index;
    int split = 2 - !encoder_info->params->enable_pb_split;
    int psizeY = sizeY/split;
    int psizeC = sizeC/split;
    int pstrideY = sizeY;
    int pstrideC = sizeC;
    int rstrideY = ref->stride_y;
    int rstrideC = ref->stride_c;
    r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
    ref0 = encoder_info->ref[r0];
    int sign = ref0->frame_num > rec->frame_num;
    for (index=0;index<split*split;index++){
      int idx = (index>>0)&1;
      int idy = (index>>1)&1;
      int offsetpY = idy*psizeY*pstrideY + idx*psizeY;
      int offsetpC = idy*psizeC*pstrideC + idx*psizeC;
      int offsetrY = idy*psizeY*rstrideY + idx*psizeY;
      int offsetrC = idy*psizeC*rstrideC + idx*psizeC;
      mv = pred_data->mv_arr0[index];
      get_inter_prediction_luma  (pblock_y + offsetpY, ref_y + offsetrY, psizeY, psizeY, rstrideY, pstrideY, &mv,sign);
      get_inter_prediction_chroma(pblock_u + offsetpC, ref_u + offsetrC, psizeC, psizeC, rstrideC, pstrideC, &mv,sign);
      get_inter_prediction_chroma(pblock_v + offsetpC, ref_v + offsetrC, psizeC, psizeC, rstrideC, pstrideC, &mv,sign);
    }
  }
  else if (mode==MODE_BIPRED){
    r0 = encoder_info->frame_info.ref_array[pred_data->ref_idx0];
    ref0 = encoder_info->ref[r0];
    ref0_y = ref0->y + ref_posY;
    ref0_u = ref0->u + ref_posC;
    ref0_v = ref0->v + ref_posC;

    r1 = encoder_info->frame_info.ref_array[pred_data->ref_idx1];
    ref1 = encoder_info->ref[r1];
    ref1_y = ref1->y + ref_posY;
    ref1_u = ref1->u + ref_posC;
    ref1_v = ref1->v + ref_posC;

    int index;
    int split = 1; //TODO: Make bipred and pb-split combination work
    int psizeY = sizeY/split;
    int psizeC = sizeC/split;
    int pstrideY = sizeY;
    int pstrideC = sizeC;
    int rstrideY = ref->stride_y;
    int rstrideC = ref->stride_c;
    int sign0 = ref0->frame_num > rec->frame_num;
    int sign1 = ref1->frame_num > rec->frame_num;
    for (index=0;index<split*split;index++){
      int idx = (index>>0)&1;
      int idy = (index>>1)&1;
      int offsetpY = idy*psizeY*pstrideY + idx*psizeY;
      int offsetpC = idy*psizeC*pstrideC + idx*psizeC;
      int offsetrY = idy*psizeY*rstrideY + idx*psizeY;
      int offsetrC = idy*psizeC*rstrideC + idx*psizeC;
      mv = pred_data->mv_arr0[index];
      get_inter_prediction_luma  (pblock0_y + offsetpY, ref0_y + offsetrY, psizeY, psizeY, rstrideY, pstrideY, &mv,sign0);
      get_inter_prediction_chroma(pblock0_u + offsetpC, ref0_u + offsetrC, psizeC, psizeC, rstrideC, pstrideC, &mv,sign0);
      get_inter_prediction_chroma(pblock0_v + offsetpC, ref0_v + offsetrC, psizeC, psizeC, rstrideC, pstrideC, &mv,sign0);
      mv = pred_data->mv_arr1[index];
      get_inter_prediction_luma  (pblock1_y + offsetpY, ref1_y + offsetrY, psizeY, psizeY, rstrideY, pstrideY, &mv,sign1);
      get_inter_prediction_chroma(pblock1_u + offsetpC, ref1_u + offsetrC, psizeC, psizeC, rstrideC, pstrideC, &mv,sign1);
      get_inter_prediction_chroma(pblock1_v + offsetpC, ref1_v + offsetrC, psizeC, psizeC, rstrideC, pstrideC, &mv,sign1);
    }
    int i,j;
    for (i=0;i<sizeY;i++){
      for (j=0;j<sizeY;j++){
        pblock_y[i*pstrideY+j] = (uint8_t)(((int)pblock0_y[i*pstrideY+j] + (int)pblock1_y[i*pstrideY+j])>>1);
      }
    }
    for (i=0;i<sizeC;i++){
      for (j=0;j<sizeC;j++){
        pblock_u[i*pstrideC+j] = (uint8_t)(((int)pblock0_u[i*pstrideC+j] + (int)pblock1_u[i*pstrideC+j])>>1);
        pblock_v[i*pstrideC+j] = (uint8_t)(((int)pblock0_v[i*pstrideC+j] + (int)pblock1_v[i*pstrideC+j])>>1);
      }
    }
  }

  if (mode!=MODE_SKIP){
    if (zero_block){
      memcpy(rec_y,pblock_y,sizeY*sizeY*sizeof(uint8_t));
      memcpy(rec_u,pblock_u,sizeC*sizeC*sizeof(uint8_t));
      memcpy(rec_v,pblock_v,sizeC*sizeC*sizeof(uint8_t));
      cbp.y = cbp.u = cbp.v = 0;
    }
    else{
      /* Create residual, transform, quantize, and reconstruct */
      cbp.y = encode_and_reconstruct_block (encoder_info, org_y,sizeY,sizeY,qpY,pblock_y,coeffq_y,rec_y,frame_type,0,tb_split,encoder_info->params->rdoq);
      cbp.u = encode_and_reconstruct_block (encoder_info, org_u,sizeC,sizeC,qpC,pblock_u,coeffq_u,rec_u,frame_type,1,tb_split&&(size>8),encoder_info->params->rdoq);
      cbp.v = encode_and_reconstruct_block (encoder_info, org_v,sizeC,sizeC,qpC,pblock_v,coeffq_v,rec_v,frame_type,1,tb_split&&(size>8),encoder_info->params->rdoq);
    }
  }
  else if (mode==MODE_SKIP){
    cbp.y = cbp.u = cbp.v = 0;
  }

#if TWO_MVP
  if (mode==MODE_INTER || mode==MODE_BIPRED){
    int min_idx = -1;
    if (block_info->num_skip_vec > 1){
      int bit0,bit1;
      bit0 = quote_mv_bits(block_info->mvb_skip[0].y0 - pred_data->mv_arr0[0].y, block_info->mvb_skip[0].x0 - pred_data->mv_arr0[0].x);
      bit1 = quote_mv_bits(block_info->mvb_skip[1].y0 - pred_data->mv_arr0[0].y, block_info->mvb_skip[1].x0 - pred_data->mv_arr0[0].x);
      if (mode==MODE_BIPRED){
        bit0 += quote_mv_bits(block_info->mvb_skip[0].y0 - pred_data->mv_arr1[0].y, block_info->mvb_skip[0].x0 - pred_data->mv_arr1[0].x);
        bit1 += quote_mv_bits(block_info->mvb_skip[1].y0 - pred_data->mv_arr1[0].y, block_info->mvb_skip[1].x0 - pred_data->mv_arr1[0].x);
      }
      min_idx = bit1 < bit0;
    }
    write_data.mvp.x = block_info->mvb_skip[max(0,min_idx)].x0;
    write_data.mvp.y = block_info->mvb_skip[max(0,min_idx)].y0;
    write_data.mv_idx = min_idx;
  }
#endif

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

  /*
  mv_t zerovec;
  zerovec.x = 0;
  zerovec.y = 0;
  */

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
  int left_index1 = block_index + block_stride*(block_size-1)/2 - 1;
  int left_index2 = block_index + block_stride*(block_size-1) - 1;
  int upright_index = block_index - block_stride + block_size;
  int upleft_index = block_index - block_stride - 1;
  int downleft_index = block_index + block_stride*block_size - 1;
  
  int up_available = get_up_available(ypos,xpos,size,width);
  int left_available = get_left_available(ypos,xpos,size,width);
  int upright_available = get_upright_available(ypos,xpos,size,width);
  int downleft_available = get_downleft_available(ypos,xpos,size,height);

  mvb_t mvc[4],zerovecr;
  zerovecr.ref_idx0 = 0;
  zerovecr.x0 = 0;
  zerovecr.y0 = 0;
  zerovecr.ref_idx1 = 0;
  zerovecr.x1 = 0;
  zerovecr.y1 = 0;
  zerovecr.dir = 0;

  /* Note: It is assumed that MV for intra blocks are stored as zero in deblock_data */
  mvc[0] = zerovecr;
  mvc[1] = zerovecr;
  mvc[2] = zerovecr;
  mvc[3] = zerovecr;

  int U = up_available;
  int UR = upright_available;
  int L = left_available;
  int DL = downleft_available;
  if (U==0 && UR==0 && L==0 && DL==0){
    mvc[0] = zerovecr;
    mvc[1] = zerovecr;
    mvc[2] = zerovecr;
    mvc[3] = zerovecr;
  }
  else if (U==1 && UR==0 && L==0 && DL==0){
    mvc[0] = deblock_data[up_index0].mvb;
    mvc[1] = deblock_data[up_index1].mvb;
    mvc[2] = deblock_data[up_index2].mvb;
    mvc[3] = deblock_data[up_index2].mvb;
  }
  else if (U==1 && UR==1 && L==0 && DL==0){
    mvc[0] = deblock_data[up_index0].mvb;
    mvc[1] = deblock_data[up_index2].mvb;
    mvc[2] = deblock_data[upright_index].mvb;
    mvc[3] = deblock_data[upright_index].mvb;
  }
  else if (U==0 && UR==0 && L==1 && DL==0){
    mvc[0] = deblock_data[left_index0].mvb;
    mvc[1] = deblock_data[left_index1].mvb;
    mvc[2] = deblock_data[left_index2].mvb;
    mvc[3] = deblock_data[left_index2].mvb;
  }
  else if (U==1 && UR==0 && L==1 && DL==0){
    mvc[0] = deblock_data[upleft_index].mvb;
    mvc[1] = deblock_data[up_index2].mvb;
    mvc[2] = deblock_data[left_index2].mvb;
    mvc[3] = deblock_data[up_index0].mvb;
  }
 
  else if (U==1 && UR==1 && L==1 && DL==0){
    mvc[0] = deblock_data[up_index0].mvb;
    mvc[1] = deblock_data[upright_index].mvb;
    mvc[2] = deblock_data[left_index2].mvb;
    mvc[3] = deblock_data[left_index0].mvb;
  }
  else if (U==0 && UR==0 && L==1 && DL==1){
    mvc[0] = deblock_data[left_index0].mvb;
    mvc[1] = deblock_data[left_index2].mvb;
    mvc[2] = deblock_data[downleft_index].mvb;
    mvc[3] = deblock_data[downleft_index].mvb;
  }
  else if (U==1 && UR==0 && L==1 && DL==1){
    mvc[0] = deblock_data[up_index2].mvb;
    mvc[1] = deblock_data[left_index0].mvb;
    mvc[2] = deblock_data[downleft_index].mvb;
    mvc[3] = deblock_data[up_index0].mvb;
  }
  else if (U==1 && UR==1 && L==1 && DL==1){
    mvc[0] = deblock_data[up_index0].mvb;
    mvc[1] = deblock_data[upright_index].mvb;
    mvc[2] = deblock_data[left_index0].mvb;
    mvc[3] = deblock_data[downleft_index].mvb;
  }
  else{
    printf("Error in ME candidate definition\n");
  }
  mvcand[0].y = mvc[0].y0;
  mvcand[0].x = mvc[0].x0;
  mvcand[1].y = mvc[1].y0;
  mvcand[1].x = mvc[1].x0;
  mvcand[2].y = mvc[2].y0;
  mvcand[2].x = mvc[2].x0;
  mvcand[3].y = mvc[3].y0;
  mvcand[3].x = mvc[3].x0;

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
      encoder_info->deblock_data[block_index].mvb.x0 = block_info->pred_data.mv_arr0[index].x;
      encoder_info->deblock_data[block_index].mvb.y0 = block_info->pred_data.mv_arr0[index].y;
      encoder_info->deblock_data[block_index].mvb.ref_idx0 = block_info->pred_data.ref_idx0;
      encoder_info->deblock_data[block_index].mvb.x1 = block_info->pred_data.mv_arr1[index].x;
      encoder_info->deblock_data[block_index].mvb.y1 = block_info->pred_data.mv_arr1[index].y;
      encoder_info->deblock_data[block_index].mvb.ref_idx1 = block_info->pred_data.ref_idx1;
      encoder_info->deblock_data[block_index].mvb.dir = block_info->pred_data.dir;
      encoder_info->deblock_data[block_index].mode = block_info->pred_data.mode;
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
#if TWO_MVP
  mv_t mvcand[8];
#else
  mv_t mvcand[6];
#endif
  block_mode_t mode;
  intra_mode_t intra_mode;
  pred_data_t pred_data;

#if 2
  int do_inter = 1;
  int do_intra = 1;
#endif
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
      }
    }
  }

  /* Evaluate inter mode */
  if (!rectangular_flag){ //Only evaluate intra or inter mode if the block is square
    if (frame_type != I_FRAME){
      tb_param = 0;
      mode = MODE_MERGE;
      int merge_idx;
#if 1//NO_SUBBLOCK_SKIP
      int num_merge_vec = block_info->num_merge_vec;
      for (merge_idx=0;merge_idx<num_merge_vec;merge_idx++){
#else
      for (merge_idx=best_skip_idx;merge_idx<=best_skip_idx;merge_idx++){
#endif
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
        }
      }

#if 2
      if (encoder_info->params->encoder_speed>1){
        sad_intra = search_intra_prediction_params(org_block->y,rec,&block_info->block_pos,encoder_info->width,encoder_info->height,encoder_info->frame_info.num_intra_modes,&intra_mode);      
        nbits = 2;
        sad_intra += (int)(sqrt(lambda)*(double)nbits + 0.5);
      }
#endif

      mode = MODE_INTER;
      /* Find candidate vectors to be used in ME */
      int ref_idx;

      int min_idx,max_idx;
      min_idx = 0;
      max_idx = frame_info->num_ref - 1;

      for (ref_idx=min_idx;ref_idx<=max_idx;ref_idx++){
        int r = encoder_info->frame_info.ref_array[ref_idx];
        ref = encoder_info->ref[r];
        pred_data.ref_idx0 = ref_idx;
        get_mv_cand(ypos,xpos,width,height,size,ref_idx,encoder_info->deblock_data,mvcand);
        mvp = get_mv_pred(ypos,xpos,width,height,size,ref_idx,encoder_info->deblock_data);
        block_info->mvp = mvp;
#if TWO_MVP
        int idx;
        int max_mvp_cand = min(2,block_info->num_skip_vec);
        mvcand[6+1].x = mvcand[6+1].y = 1<<14;
        for (idx=0;idx<max_mvp_cand;idx++){
          mvcand[6+idx].x = block_info->mvb_skip[idx].x0;
          mvcand[6+idx].y = block_info->mvb_skip[idx].y0;
        }
#endif
        int sign = ref->frame_num > rec->frame_num;

        /* Loop over all PU partitions to do ME */
        for (part=0;part<block_info->max_num_pb_part;part++){
          sad = (uint32_t)search_inter_prediction_params(org_block->y,ref,&block_info->block_pos,&mvp,mvcand,mv_all[part],part,sqrt(lambda),encoder_info->params->encoder_speed,sign);
          sad_inter = min(sad_inter,sad);
        }

        /* Loop over all PU partitions to do RDO */


#if 2
        if (encoder_info->params->encoder_speed>1){
          if (sad_intra < sad_inter)
            do_inter = 0;
          else
            do_intra = 0;
        }
        if (do_inter){
#endif
        for (part=0;part<block_info->max_num_pb_part;part++){
          pred_data.PBpart = part;
          memcpy(pred_data.mv_arr0,mv_all[part],4*sizeof(mv_t));
          min_tb_param = encoder_info->params->encoder_speed==0 ? -1 : 0; //tb_split == -1 means force residual to zero.
#if NEW_BLOCK_STRUCTURE
          max_tb_param = block_info->max_num_tb_part-1;
#else
          max_tb_param = part>0 ? 0 : block_info->max_num_tb_part-1;  //Can't have TU-split and PU-split at the same time
#endif
          for (tb_param=min_tb_param; tb_param<=max_tb_param; tb_param++){
            nbits = encode_block(encoder_info,stream,block_info,&pred_data,mode,tb_param);
            cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
            if (cost < min_cost){
              min_cost = cost;
              best_mode = MODE_INTER;
              best_tb_param = tb_param;
              best_pb_part = part;
              best_ref_idx = ref_idx;
              memcpy(best_mv_arr,mv_all[part],4*sizeof(mv_t));
            }
          }
        } //for part=
#if 2
        }
#endif
      } //for ref_idx=

      /* Start bipred ME */
#if 2
      if (frame_info->num_ref>1 && encoder_info->params->enable_bipred && do_inter){ //TODO: Should work with only one reference also
#else
      if (frame_info->num_ref>1 && encoder_info->params->enable_bipred){ //TODO: Should work with only one reference also
#endif
        int min_sad,sad;
        int r,i,ref_posY;
        mv_t mv;
        uint8_t *ref_y;
        uint8_t pblock_y[MAX_BLOCK_SIZE*MAX_BLOCK_SIZE];
        uint8_t *org8 = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);

        /* Initialize using ME for ref_idx=0 */
        ref_idx = 0;
        r = encoder_info->frame_info.ref_array[ref_idx];
        ref = encoder_info->ref[r];
        ref_posY = ref->offset_y + ypos*ref->stride_y + xpos;
        part = 0;
        pred_data.mv_arr0[0] = mvp;
        pred_data.mv_arr0[1] = mvp;
        pred_data.mv_arr0[2] = mvp;
        pred_data.mv_arr0[3] = mvp;
        pred_data.ref_idx0 = 0;

        min_sad = (1<<30);
        int n,num_iter,list;
        num_iter = encoder_info->params->encoder_speed == 0 ? 2 : 1;
        for (n=0;n<num_iter;n++){
          for (list=1;list>=0;list--){

            /* Determine mv and ref_idx for the other list */
            mv = list ? pred_data.mv_arr0[0] : pred_data.mv_arr1[0];
            ref_idx = list ? pred_data.ref_idx0 : pred_data.ref_idx1;

            /* Find prediction for the other list */
            r = encoder_info->frame_info.ref_array[ref_idx];
            ref = encoder_info->ref[r];
            ref_y = ref->y + ref_posY;

            int sign = ref->frame_num > rec->frame_num;
            get_inter_prediction_luma  (pblock_y, ref_y, size, size, ref->stride_y, size, &mv, sign);

            /* Modify the target block based on that predition */
            for (i=0;i<size*size;i++){
              org8[i] = (uint8_t)clip255(2*(int16_t)org_block->y[i] - (int16_t)pblock_y[i]);
            }

            /* Find MV and ref_idx for the current list */
            for (ref_idx=0;ref_idx<frame_info->num_ref;ref_idx++){
              r = encoder_info->frame_info.ref_array[ref_idx];
              ref = encoder_info->ref[r];
              int sign = ref->frame_num > rec->frame_num;
              sad = (uint32_t)search_inter_prediction_params(org8,ref,&block_info->block_pos,&mvp,mvcand,mv_all[part],0,sqrt(lambda),encoder_info->params->encoder_speed,sign);
              if (sad < min_sad){
                min_sad = sad;
                if (list){
                  pred_data.ref_idx1 = ref_idx;
                  memcpy(pred_data.mv_arr1,mv_all[part],4*sizeof(mv_t));
                }
                else{
                  pred_data.ref_idx0 = ref_idx;
                  memcpy(pred_data.mv_arr0,mv_all[part],4*sizeof(mv_t));
                }
              }
            }
          }
        }
        thor_free(org8);
        mode = MODE_BIPRED;
        min_tb_param = 0;
        max_tb_param = 0; //TODO: Support tb-split
        for (tb_param=min_tb_param; tb_param<=max_tb_param; tb_param++){
          nbits = encode_block(encoder_info,stream,block_info,&pred_data,mode,tb_param);
          cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
          if (cost < min_cost){
            min_cost = cost;
            best_mode = MODE_BIPRED;
            best_tb_param = tb_param;
          }
        }
      } //if enable_bipred
    } //if frame_type != I_FRAME

    /* Evaluate intra mode */
    mode = MODE_INTRA;
#if 2
    if (do_intra && encoder_info->params->intra_rdo){
#else
    if (encoder_info->params->intra_rdo){
#endif
      uint8_t pblock[MAX_BLOCK_SIZE*MAX_BLOCK_SIZE];
      uint32_t min_intra_cost = MAX_UINT32;
      intra_mode_t best_intra_mode = MODE_DC;
      int upright_available = get_upright_available(ypos,xpos,size,width);
      for (intra_mode=MODE_DC;intra_mode<encoder_info->frame_info.num_intra_modes;intra_mode++){

        pred_data.intra_mode = intra_mode;
        min_tb_param = 0;
        max_tb_param = block_info->max_num_tb_part-1;
        for (tb_param=0; tb_param<=max_tb_param; tb_param++){
#if LIMIT_INTRA_MODES
          if (intra_mode==MODE_PLANAR || intra_mode==MODE_UPRIGHT)
           continue;
#endif
          nbits = encode_block(encoder_info,stream,block_info,&pred_data,mode,tb_param);
          cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
          if (cost < min_intra_cost){
            min_intra_cost = cost;
            best_intra_mode = intra_mode;
          }
        }
      }
      intra_mode = best_intra_mode;
      get_intra_prediction(rec->y,ypos,xpos,rec->stride_y,size,width,pblock,intra_mode,upright_available);
      sad_intra = sad_calc(org_block->y,pblock,size,size,size,size);
    }
    else{
      sad_intra = search_intra_prediction_params(org_block->y,rec,&block_info->block_pos,encoder_info->width,encoder_info->height,encoder_info->frame_info.num_intra_modes,&intra_mode);
    }
    nbits = 2;
    sad_intra += (int)(sqrt(lambda)*(double)nbits + 0.5);
    pred_data.intra_mode = intra_mode;

    min_tb_param = 0;
    max_tb_param = block_info->max_num_tb_part-1;
#if 2
    if (do_intra){
#endif
    for (tb_param=min_tb_param; tb_param<=max_tb_param; tb_param++){
      nbits = encode_block(encoder_info,stream,block_info,&pred_data,mode,tb_param);
      cost = cost_calc(org_block,rec_block,size,size,size,nbits,lambda);
      if (cost < min_cost){
        min_cost = cost;
        best_mode = MODE_INTRA;
        best_tb_param = tb_param;
      }
    }
#if 2
    }
#endif
  } //if !rectangular_flag


  /* Rewind bitstream to reference position */
  write_stream_pos(stream,&stream_pos_ref);

  /* Store prediction data derived through RDO */
  block_info->pred_data.mode = best_mode;
  if (best_mode == MODE_SKIP){
    block_info->pred_data.skip_idx = best_skip_idx;
    for (int i=0;i<4;i++){
      block_info->pred_data.mv_arr0[i].x = block_info->mvb_skip[best_skip_idx].x0;
      block_info->pred_data.mv_arr0[i].y = block_info->mvb_skip[best_skip_idx].y0;
      block_info->pred_data.mv_arr1[i].x = block_info->mvb_skip[best_skip_idx].x1;
      block_info->pred_data.mv_arr1[i].y = block_info->mvb_skip[best_skip_idx].y1;
    }
    block_info->pred_data.ref_idx0 = block_info->mvb_skip[best_skip_idx].ref_idx0;
    block_info->pred_data.ref_idx1 = block_info->mvb_skip[best_skip_idx].ref_idx1;
    block_info->pred_data.dir = best_skip_dir;
  }
  else if (best_mode == MODE_MERGE){
    block_info->pred_data.PBpart = PART_NONE;
    block_info->pred_data.skip_idx = best_skip_idx;
    for (int i=0;i<4;i++){
      block_info->pred_data.mv_arr0[i].x = block_info->mvb_merge[best_skip_idx].x0;
      block_info->pred_data.mv_arr0[i].y = block_info->mvb_merge[best_skip_idx].y0;
      block_info->pred_data.mv_arr1[i].x = block_info->mvb_merge[best_skip_idx].x1;
      block_info->pred_data.mv_arr1[i].y = block_info->mvb_merge[best_skip_idx].y1;
    }
    block_info->pred_data.ref_idx0 = block_info->mvb_merge[best_skip_idx].ref_idx0;
    block_info->pred_data.ref_idx1 = block_info->mvb_merge[best_skip_idx].ref_idx1;
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
    block_info->pred_data.PBpart = PART_NONE;
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
  const int qsize = min(MAX_QUANT_SIZE,size);
  int cstride = size;
  int scale = gquant_table[qp%6];
  int c,flag = 0;
  int shift2 = 21 - tr_log2size + qp/6;

  /* Derive threshold from first quantizer level */
  double first_quantizer_level = (double)(1<<shift2)/(double)scale;
  double threshold = relative_threshold * first_quantizer_level;

  /* Compare each coefficient with threshold */
  for (int i = 0; i < qsize; i++){
    for (int j = 0; j < qsize; j++){
      c = (int)coeff[i*cstride+j];
      if ((double)abs(c) > threshold)
        flag = 1;
    }
  }
  return (flag != 0);
}

int check_early_skip_8x8_block (encoder_info_t *encoder_info, uint8_t *orig, int orig_stride, int size, int qp, uint8_t *pblock, float early_skip_threshold)
{
  int cbp;
  int16_t *block = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeff = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

  get_residual(block, pblock, orig, size, orig_stride);

  if (size==8){
    int16_t tmp[4*4];
    int i,j;
    /* Instead of 8x8 transform, do a 2x2 average followed by 4x4 transform */
    for (i=0;i<8;i+=2){
      for (j=0;j<8;j+=2){
        tmp[(i/2)*4+(j/2)] = (block[(i+0)*size+(j+0)] + block[(i+0)*size+(j+1)] + block[(i+1)*size+(j+0)] + block[(i+1)*size+(j+1)] + 2)>>2;
      }
    }
    transform(tmp, coeff, size/2, encoder_info->params->encoder_speed > 1);
    cbp = check_early_skip_transform_coeff(coeff, qp, size/2, 0.5*early_skip_threshold);
  }
  else{
    transform (block, coeff, size, encoder_info->params->encoder_speed > 1);
    cbp = check_early_skip_transform_coeff(coeff, qp, size, early_skip_threshold);
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
  int size0 = EARLY_SKIP_BLOCK_SIZE;
  int qpY = encoder_info->frame_info.qp + block_info->delta_qp;
  int qpC = chroma_qp[qpY];
  uint8_t *pblock = thor_alloc(EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE, 16);
  uint8_t *pblock0 = thor_alloc(EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE, 16);
  uint8_t *pblock1 = thor_alloc(EARLY_SKIP_BLOCK_SIZE*EARLY_SKIP_BLOCK_SIZE, 16);
  mv_t mv = pred_data->mv_arr0[0];
  int ref_idx = pred_data->ref_idx0;
  int r = encoder_info->frame_info.ref_array[ref_idx];
  yuv_frame_t *ref = encoder_info->ref[r];
  yuv_block_t *org_block = block_info->org_block; //Compact (size x size) block of original pixels

  float early_skip_threshold = encoder_info->params->early_skip_thr;

#if 5
  if (encoder_info->params->encoder_speed > 1 && size == MAX_BLOCK_SIZE)
    early_skip_threshold = 1.3*early_skip_threshold;
#endif

  if (pred_data->dir==2){
    /* Loop over 8x8 (4x4) sub-blocks */
    for (i=0;i<size;i+=size0){
      for (j=0;j<size;j+=size0){
        int k,l;
        ref_idx = pred_data->ref_idx0;
        r = encoder_info->frame_info.ref_array[ref_idx];
        yuv_frame_t *ref0 = encoder_info->ref[r];
        /* Pointer to colocated sub-block in reference frame */
        uint8_t *ref0_y = ref0->y + ref->offset_y + (ypos + i)*ref->stride_y + (xpos + j);
        uint8_t *ref0_u = ref0->u + ref->offset_c + (ypos + i)/2*ref->stride_c + (xpos + j)/2;
        uint8_t *ref0_v = ref0->v + ref->offset_c + (ypos + i)/2*ref->stride_c + (xpos + j)/2;

        ref_idx = pred_data->ref_idx1;
        r = encoder_info->frame_info.ref_array[ref_idx];
        yuv_frame_t *ref1 = encoder_info->ref[r];
        /* Pointer to colocated sub-block in reference frame */
        uint8_t *ref1_y = ref1->y + ref->offset_y + (ypos + i)*ref->stride_y + (xpos + j);
        uint8_t *ref1_u = ref1->u + ref->offset_c + (ypos + i)/2*ref->stride_c + (xpos + j)/2;
        uint8_t *ref1_v = ref1->v + ref->offset_c + (ypos + i)/2*ref->stride_c + (xpos + j)/2;

        int sign0 = ref0->frame_num > encoder_info->frame_info.frame_num;
        int sign1 = ref1->frame_num > encoder_info->frame_info.frame_num;

        /* Offset for 8x8 (4x4) sub-block within compact block of original pixels */
        int block_offset_y = size*i + j;
        int block_offset_c = (size/2)*(i/2) + j/2;

        /* Y */
        mv = pred_data->mv_arr0[0];
        get_inter_prediction_luma  (pblock0,ref0_y,size0,size0,ref->stride_y,size0,&mv,sign0);
        mv = pred_data->mv_arr1[0];
        get_inter_prediction_luma  (pblock1,ref1_y,size0,size0,ref->stride_y,size0,&mv,sign1);
        for (k=0;k<size0;k++){
          for (l=0;l<size0;l++){
            pblock[k*size0+l] = (uint8_t)(((int)pblock0[k*size0+l] + (int)pblock1[k*size0+l])>>1);
          }
        }
        significant_flag = significant_flag || check_early_skip_8x8_block(encoder_info, org_block->y + block_offset_y,size,size0,qpY,pblock,early_skip_threshold);

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
        significant_flag = significant_flag || check_early_skip_8x8_block(encoder_info, org_block->u + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);

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
        significant_flag = significant_flag || check_early_skip_8x8_block(encoder_info, org_block->v + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);
      } //for j
    } //for i
  }
  else{
    int sign = ref->frame_num > encoder_info->frame_info.frame_num;
    /* Loop over 8x8 (4x4) sub-blocks */
    for (i=0;i<size;i+=size0){
      for (j=0;j<size;j+=size0){

        /* Pointer to colocated sub-block in reference frame */
        uint8_t *ref_y = ref->y + ref->offset_y + (ypos + i)*ref->stride_y + (xpos + j);
        uint8_t *ref_u = ref->u + ref->offset_c + (ypos + i)/2*ref->stride_c + (xpos + j)/2;
        uint8_t *ref_v = ref->v + ref->offset_c + (ypos + i)/2*ref->stride_c + (xpos + j)/2;

        /* Offset for 8x8 (4x4) sub-block within compact block of original pixels */
        int block_offset_y = size*i + j;
        int block_offset_c = (size/2)*(i/2) + j/2;

        /* Y */
        get_inter_prediction_luma  (pblock,ref_y,size0,size0,ref->stride_y,size0,&mv,sign);
        significant_flag = significant_flag || check_early_skip_8x8_block(encoder_info, org_block->y + block_offset_y,size,size0,qpY,pblock,early_skip_threshold);

        /* U */
        get_inter_prediction_chroma(pblock,ref_u,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign);
        significant_flag = significant_flag || check_early_skip_8x8_block(encoder_info, org_block->u + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);

        /* V */
        get_inter_prediction_chroma(pblock,ref_v,size0/2,size0/2,ref->stride_c,size0/2,&mv,sign);
        significant_flag = significant_flag || check_early_skip_8x8_block(encoder_info, org_block->v + block_offset_c,size/2,size0/2,qpC,pblock,early_skip_threshold);
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
  int encode_smaller_size = size > MIN_BLOCK_SIZE;
  int encode_rectangular_size = !encode_this_size && frame_type != I_FRAME;

#if NEW_BLOCK_STRUCTURE
  int depth = log2i(MAX_BLOCK_SIZE) - log2i(size);
  if (size==MAX_BLOCK_SIZE){
    if (frame_type==I_FRAME || encode_rectangular_size==1)
      putbits(2,encoder_info->depth,stream);
    else if (encoder_info->depth>0)
      putbits(encoder_info->depth+1,1,stream);
  }

  encode_this_size = encode_this_size && depth == encoder_info->depth;
  encode_rectangular_size = encode_rectangular_size && depth == encoder_info->depth;
  encode_smaller_size = encode_smaller_size && depth != encoder_info->depth;
  if (encode_this_size==0 && encode_smaller_size==0 && encode_rectangular_size==0)
    return 1<<28;
#else
  if (encode_this_size==0 && encode_smaller_size==0)
    return 0;
#endif
  cost_small = 1<<28;
  cost = 1<<28;


  /* Store bitstream state before doing anything at this block size */
  stream_pos_t stream_pos_ref;
  read_stream_pos(&stream_pos_ref,stream);

  /* Initialize some block-level parameters */
  block_info_t block_info;
  yuv_block_t *org_block = thor_alloc(sizeof(yuv_block_t),16);
  yuv_block_t *rec_block = thor_alloc(sizeof(yuv_block_t),16);
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

    if (encoder_info->frame_info.frame_type != I_FRAME){
      /* Find motion vector predictor (mvp) and skip vector candidates (mv-skip) */
      block_info.num_skip_vec = get_mv_skip(ypos,xpos,width,height,size,encoder_info->deblock_data,block_info.mvb_skip);
      block_info.num_merge_vec = get_mv_merge(ypos,xpos,width,height,size,encoder_info->deblock_data,block_info.mvb_merge);
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
        return cost;
      }
    }
  }

  if (encode_smaller_size){
    int new_size = size/2;
#if NEW_BLOCK_STRUCTURE
#else
    int code;
    if (frame_type == I_FRAME || encode_this_size){
      if (frame_type==I_FRAME){
        putbits(1,1,stream);
      }
      else{
        code = 1; //Split
        if(block_context.index==2 || block_context.index>3)
          code = (code+3)%4;
        putbits(code+1,1,stream);
      }
    }
    else{
      putbits(1,0,stream); //Flag to signal either split or rectangular skip
    }
#endif
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

  return min(cost,cost_small);
}

int detect_clpf(uint8_t *org, uint8_t *rec,int x0, int x1, int y0, int y1,int stride){

  int y,x,O,A,B,C,D,E,F,delta,sum0,sum1,sum,sign;
  sum0 = 0;
  sum1 = 0;
  for (y=y0;y<y1;y++){
    for (x=x0;x<x1;x++){
      O = org[(y+0)*stride + x+0];
      A = rec[(y-1)*stride + x+0];
      B = rec[(y+0)*stride + x-1];
      C = rec[(y+0)*stride + x+0];
      D = rec[(y+0)*stride + x+1];
      E = rec[(y+1)*stride + x+0];
      sum = A+B+D+E-4*C;
      sign = sum < 0 ? -1 : 1;
      delta = sign*min(1,((abs(sum)+2)>>2));
      F = clip255(C + delta);
      sum0 += (O-C)*(O-C);
      sum1 += (O-F)*(O-F);
    }
  }
  return (100*sum1 < CLPF_BIAS*sum0);
}
