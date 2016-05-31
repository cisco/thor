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
#include "common_block.h"
#include "simd.h"
#include "common_kernels.h"


#define NTAPY 6
#define OFFY (NTAPY/2)
#define OFFYM1 (OFFY-1)

static const int16_t filter_coeffsYbi[4][8] = {
  { 0,  0, 64,  0,   0, 0 },
  { 2,-10, 59, 17,  -5, 1 },
  { 1, -8, 39, 39,  -8, 1 },
  { 1, -5, 17, 59, -10, 2 }
};

static const int16_t filter_coeffsYuni[4][8] = {
  { 0,  0, 64,  0,  0, 0},
  { 1, -7, 55, 19, -5, 1},
  { 1, -7, 38, 38, -7, 1},
  { 1, -5, 19, 55,- 7, 1}
};

static const int8_t filter_coeffsC[8][4] = {
    { 0, 64,  0,  0},
    {-2, 58,  10,-2},
    {-4, 54, 16, -2},
    {-4, 44, 28, -4},
    {-4, 36, 36, -4},
    {-4, 28, 44, -4},
    {-2, 16, 54, -4},
    {-2, 10, 58, -2}
};

void clip_mv(mv_t *mv_cand, int ypos, int xpos, int fwidth, int fheight, int bwidth, int bheight, int sign) {

  int max_mv_ext = PADDING_Y - 16; //max MV extension outside frame boundaries in integer pixel resolution
  int mvy, mvx;
  mvy = sign ? -mv_cand->y : mv_cand->y;
  mvx = sign ? -mv_cand->x : mv_cand->x;
  if (ypos + mvy / 4 < -max_mv_ext) mvy = 4 * (-max_mv_ext - ypos);
  if (ypos + mvy / 4 + bheight > fheight + max_mv_ext) mvy = 4 * (fheight + max_mv_ext - ypos - bheight);
  if (xpos + mvx / 4 < -max_mv_ext) mvx = 4 * (-max_mv_ext - xpos);
  if (xpos + mvx / 4 + bwidth > fwidth + max_mv_ext) mvx = 4 * (fwidth + max_mv_ext - xpos - bwidth);
  mv_cand->y = sign ? -mvy : mvy;
  mv_cand->x = sign ? -mvx : mvx;
}

static void get_inter_prediction_chroma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign, int pic_width2, int pic_height2, int xpos, int ypos, int sub)
{
  int i,j;

  int m,i_off,j_off;
  mv_t mvtemp;
  mvtemp.x = sign ? -mv->x : mv->x;
  mvtemp.y = sign ? -mv->y : mv->y;
  int ver_frac = sub ? (mvtemp.y)&7 : 2*((mvtemp.y)&3);
  int hor_frac = sub ? (mvtemp.x)&7 : 2*((mvtemp.x)&3);
  int ver_int = sub ? (mvtemp.y)>>3 : (mvtemp.y)>>2;
  int hor_int = sub ? (mvtemp.x)>>3 : (mvtemp.x)>>2;
  ver_int = min(ver_int,pic_height2-ypos);
  ver_int = max(ver_int,-xpos-height);
  hor_int = min(hor_int,pic_width2-xpos);
  hor_int = max(hor_int,-xpos-width);
  int16_t tmp[MAX_SB_SIZE / 2 + 16][MAX_SB_SIZE / 2 + 16];

  if (ver_frac==0 && hor_frac==0){
    j_off = 0 + hor_int;
    for(i=0;i<height;i++){
      i_off = i + ver_int;
      memcpy(pblock + i*pstride,ref + i_off*stride + j_off, width*sizeof(uint8_t));
    }
    return;
  }

  if (use_simd && width > 2)
    get_inter_prediction_chroma_simd(width, height, hor_frac, ver_frac, pblock, pstride, ref + ver_int*stride + hor_int, stride);
  else {
    /* Horizontal filtering */
    for(i=-1;i<height+2;i++){
      for (j=0;j<width;j++){
        int sum = 0;
        i_off = i + ver_int;
        j_off = j + hor_int;
        for (m=0;m<4;m++) sum += filter_coeffsC[hor_frac][m] * ref[i_off * stride + j_off + m - 1];
        tmp[i+1][j] = sum;
      }
    }

    /* Vertical filtering */
    for(i=0;i<height;i++){
      for (j=0;j<width;j++){
        int sum = 0;
        for (m=0;m<4;m++) sum += filter_coeffsC[ver_frac][m] * tmp[i+m][j];
        pblock[i*pstride+j] = clip255((sum + 2048)>>12);
      }
    }
  }
}

void get_inter_prediction_luma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign, int bipred, int pic_width, int pic_height, int xpos, int ypos) 
{
  int i,j;
  int m,i_off,j_off;
  mv_t mvtemp;
  mvtemp.x = sign ? -mv->x : mv->x;
  mvtemp.y = sign ? -mv->y : mv->y;
  int ver_frac = (mvtemp.y)&3;
  int hor_frac = (mvtemp.x)&3;
  int ver_int = (mvtemp.y)>>2;
  int hor_int = (mvtemp.x)>>2;
  ver_int = min(ver_int,pic_height-ypos);
  ver_int = max(ver_int,-xpos-height);
  hor_int = min(hor_int,pic_width-xpos);
  hor_int = max(hor_int,-xpos-width);
  int32_t tmp[MAX_SB_SIZE+16][MAX_SB_SIZE + 16]; //7-bit filter exceeds 16 bit temporary storage
  /* Integer position */
  if (ver_frac==0 && hor_frac==0){
    j_off = 0 + hor_int;
    for(i=0;i<height;i++){
      i_off = i + ver_int;
      memcpy(pblock + i*pstride,ref + i_off*stride+j_off, width*sizeof(uint8_t));
    }
    return;
  }

  if (use_simd)
    get_inter_prediction_luma_simd(width, height, hor_frac, ver_frac, pblock, pstride, ref + ver_int*stride + hor_int, stride, bipred);
  /* Special lowpass filter at center position */
  else if (ver_frac == 2 && hor_frac == 2 && bipred < 2) {
    for(i=0;i<height;i++){
      for (j=0;j<width;j++){
        int sum = 0;
        i_off = i + ver_int;
        j_off = j + hor_int;
        sum += 0*ref[(i_off-1)*stride+j_off-1]+1*ref[(i_off-1)*stride+j_off+0]+1*ref[(i_off-1)*stride+j_off+1]+0*ref[(i_off-1)*stride+j_off+2];
        sum += 1*ref[(i_off+0)*stride+j_off-1]+2*ref[(i_off+0)*stride+j_off+0]+2*ref[(i_off+0)*stride+j_off+1]+1*ref[(i_off+0)*stride+j_off+2];
        sum += 1*ref[(i_off+1)*stride+j_off-1]+2*ref[(i_off+1)*stride+j_off+0]+2*ref[(i_off+1)*stride+j_off+1]+1*ref[(i_off+1)*stride+j_off+2];
        sum += 0*ref[(i_off+2)*stride+j_off-1]+1*ref[(i_off+2)*stride+j_off+0]+1*ref[(i_off+2)*stride+j_off+1]+0*ref[(i_off+2)*stride+j_off+2];
        pblock[i*pstride+j] = clip255((sum + 8)>>4);
      }
    }
  } else {
    /* Vertical filtering */
    const int16_t *filterV = (bipred ? filter_coeffsYbi : filter_coeffsYuni)[ver_frac];
    for(i=-OFFYM1;i<width+OFFY;i++){
      for (j=0;j<height;j++){
        int sum = 0;
        i_off = i + hor_int;
        j_off = j + ver_int;
        for (m=0;m<NTAPY;m++) sum += filterV[m] * ref[(j_off + m - OFFYM1) * stride + i_off];
        tmp[j][i+OFFYM1] = sum;
      }
    }
    /* Horizontal filtering */
    const int16_t *filterH = (bipred ? filter_coeffsYbi : filter_coeffsYuni)[hor_frac];
    for(i=0;i<width;i++){
      for (j=0;j<height;j++){
        int sum = 0;
        for (m=0;m<NTAPY;m++) sum += filterH[m] * tmp[j][i+m];
        pblock[j*pstride+i] = clip255((sum + 2048)>>12);
      }
    }
  }
}



void get_inter_prediction_yuv(yuv_frame_t *ref, uint8_t *pblock_y, uint8_t *pblock_u, uint8_t *pblock_v, block_pos_t *block_pos, mv_t *mv_arr, int sign, int width, int height, int enable_bipred, int split) {
  mv_t mv;
  int div = split + 1;

  int bwidth = block_pos->bwidth / div;
  int bheight = block_pos->bheight / div;
  int pstride = block_pos->size;
  int rstride_y = ref->stride_y;
  int rstride_c = ref->stride_c;
  int index;
  int yposY = block_pos->ypos;
  int xposY = block_pos->xpos;
  int yposC = yposY >> ref->sub;
  int xposC = xposY >> ref->sub;

  int ref_posY = yposY*ref->stride_y + xposY;
  int ref_posC = yposC*ref->stride_c + xposC;
  uint8_t *ref_y = ref->y + ref_posY;
  uint8_t *ref_u = ref->u + ref_posC;
  uint8_t *ref_v = ref->v + ref_posC;
  for (index = 0; index<div*div; index++) {
    int idx = (index >> 0) & 1;
    int idy = (index >> 1) & 1;
    int offsetpY = idy*bheight*pstride + idx*bwidth;
    int offsetpC = (idy*bheight*pstride >> (ref->sub + ref->sub)) + (idx*bwidth >> ref->sub);
    int offsetrY = idy*bheight*rstride_y + idx*bwidth;
    int offsetrC = (idy*bheight*rstride_c >> ref->sub) + (idx*bwidth >> ref->sub);
    mv = mv_arr[index];
    clip_mv(&mv, yposY, xposY, width, height, bwidth, bheight, sign);
    get_inter_prediction_luma(pblock_y + offsetpY, ref_y + offsetrY, bwidth, bheight, rstride_y, pstride, &mv, sign, enable_bipred, width, height, xposY, yposY); //get_inter_prediction_yuv()
    get_inter_prediction_chroma(pblock_u + offsetpC, ref_u + offsetrC, bwidth >> ref->sub, bheight >> ref->sub, rstride_c, pstride >> ref->sub, &mv, sign, width >> ref->sub, height >> ref->sub, xposC, yposC, ref->sub);
    get_inter_prediction_chroma(pblock_v + offsetpC, ref_v + offsetrC, bwidth >> ref->sub, bheight >> ref->sub, rstride_c, pstride >> ref->sub, &mv, sign, width >> ref->sub, height >> ref->sub, xposC, yposC, ref->sub);
  }
}

void average_blocks_all(uint8_t *rec_y, uint8_t *rec_u, uint8_t *rec_v, uint8_t *pblock0_y, uint8_t *pblock0_u, uint8_t *pblock0_v, uint8_t *pblock1_y, uint8_t *pblock1_u, uint8_t *pblock1_v, block_pos_t *block_pos, int sub) {
  int bwidth = block_pos->bwidth;
  int bheight = block_pos->bheight;
  int size = block_pos->size;
  int sizeY = size;
  int sizeC = size >> sub;
  int i, j;

  for (i = 0; i < bheight; i++) {
    for (j = 0; j < bwidth; j++) {
      rec_y[i*sizeY + j] = (uint8_t)(((int)pblock0_y[i*sizeY + j] + (int)pblock1_y[i*sizeY + j]) >> 1);
    }
  }
  for (i = 0; i < bheight >> sub; i++) {
    for (j = 0; j < bwidth >> sub; j++) {
      rec_u[i*sizeC + j] = (uint8_t)(((int)pblock0_u[i*sizeC + j] + (int)pblock1_u[i*sizeC + j]) >> 1);
      rec_v[i*sizeC + j] = (uint8_t)(((int)pblock0_v[i*sizeC + j] + (int)pblock1_v[i*sizeC + j]) >> 1);
    }
  }
}

void scale_mv(mv_t *mv_in, mv_t *mv_out, double scale, double offset) {
  double scalef = 1.0 / scale;
  int absx = abs(mv_in->x);
  int absy = abs(mv_in->y);
  int signx = mv_in->x >= 0 ? 1 : -1;
  int signy = mv_in->y >= 0 ? 1 : -1;
  mv_out->x = signx * (int)floor(scalef * absx + offset);
  mv_out->y = signy * (int)floor(scalef * absy + offset);
}
void store_mv(int width, int height, int b_level, int frame_type, int frame_num, int gop_size, deblock_data_t *deblock_data) {
  int i, j, block_index, block_posy, block_posx;
  int block_stride = width / MIN_PB_SIZE;
  int ref_idx0, bipred_flag;
  int phase = frame_num % gop_size;
  inter_pred_t *inter_pred;
  mv_t mvin, mvout;
  double offset = 0.125;
  static double scale_array[4] = { 8.0 / 4.0, 16.0 / 4.0, 9.0 / 4.0, 11.0 / 4.0 };

  int scale, p, lev, inc, delta;
  int num_lev = log2i(gop_size);

  if (gop_size == 3) { //Support for 2B
    static double scale_array2[3] = { 3.0 / 3.0, 6.0 / 3.0, 5.0 / 3.0 };
    for (i = 0; i < height; i += MIN_PB_SIZE) {
      for (j = 0; j < width; j += MIN_PB_SIZE) {
        block_posy = i / MIN_PB_SIZE;
        block_posx = j / MIN_PB_SIZE;
        block_index = block_posy*block_stride + block_posx;
        ref_idx0 = deblock_data[block_index].inter_pred.ref_idx0;
        bipred_flag = deblock_data[block_index].inter_pred.bipred_flag;
        inter_pred = &deblock_data[block_index].inter_pred;

        if (frame_type == P_FRAME) {
          mvin = inter_pred->mv0;
          scale_mv(&mvin, &mvout, (3.0 / 1.0)*scale_array2[ref_idx0], offset);
          deblock_data[block_index].inter_pred_arr[1].mv0 = mvout;
          deblock_data[block_index].inter_pred_arr[2].mv0 = mvout;
        }
        else if (frame_type == B_FRAME && phase == 1 && deblock_data[block_index].mode != MODE_INTRA) {
          if (bipred_flag || ref_idx0 == 1) {
            mvin = bipred_flag ? inter_pred->mv1 : inter_pred->mv0;
            scale_mv(&mvin, &mvout, 2.0, offset);
            deblock_data[block_index].inter_pred_arr[2].mv0 = mvout;
          }
        }
      }
    }
    return;
  }

  for (i = 0; i < height; i += MIN_PB_SIZE) {
    for (j = 0; j < width; j += MIN_PB_SIZE) {
      block_posy = i / MIN_PB_SIZE;
      block_posx = j / MIN_PB_SIZE;
      block_index = block_posy*block_stride + block_posx;
      ref_idx0 = deblock_data[block_index].inter_pred.ref_idx0;
      bipred_flag = deblock_data[block_index].inter_pred.bipred_flag;
      inter_pred = &deblock_data[block_index].inter_pred;

      if (frame_type == P_FRAME) {
        mvin = inter_pred->mv0;
        for (lev = 0; lev < num_lev; lev++) {
          scale = (1 << lev);
          scale_mv(&mvin, &mvout, (double)scale*scale_array[ref_idx0], offset);
          inc = gop_size >> lev;
          delta = (inc >> 1);
          for (p = delta; p < gop_size; p += inc) {
            deblock_data[block_index].inter_pred_arr[p].mv0 = mvout;
          }
        }
      }
      else if (frame_type == B_FRAME && b_level < num_lev-1 && deblock_data[block_index].mode != MODE_INTRA) {
        if (bipred_flag || ref_idx0 == 0) {
          mvin = inter_pred->mv0;
          for (lev = b_level + 1; lev < num_lev; lev++) {
            scale = (1 << (lev - b_level));
            scale_mv(&mvin, &mvout, (double)scale, offset);
            inc = gop_size >> lev;
            delta = (scale - 1)*(inc >> 1);
            for (p = phase - delta; p < phase; p += inc) {
              deblock_data[block_index].inter_pred_arr[p].mv0 = mvout;
            }
          }
        }
        if (bipred_flag || ref_idx0 == 1) {
          mvin = bipred_flag ? inter_pred->mv1 : inter_pred->mv0;
          for (lev = b_level + 1; lev < num_lev; lev++) {
            scale = (1 << (lev - b_level));
            scale_mv(&mvin, &mvout, (double)scale, offset);
            inc = gop_size >> lev;
            delta = (scale - 1)*(inc >> 1);
            for (p = phase + delta; p > phase; p -= inc) {
              deblock_data[block_index].inter_pred_arr[p].mv0 = mvout;
            }
          }
        }
      }
    }
  }
}

void get_inter_prediction_temp(int width, int height, yuv_frame_t *ref0, yuv_frame_t *ref1, block_pos_t *block_pos, deblock_data_t *deblock_data, int gop_size, int phase, uint8_t *pblock_y, uint8_t *pblock_u, uint8_t *pblock_v) {
  int i, m, n;
  block_pos_t tmp_block_pos;
  int ypos0, xpos0;
  int block_stride, block_posy, block_posx, block_index;
  mv_t mv_arr[4];
  uint8_t pblock0_y[MIN_PB_SIZE*MIN_PB_SIZE];
  uint8_t pblock0_u[MIN_PB_SIZE*MIN_PB_SIZE];
  uint8_t pblock0_v[MIN_PB_SIZE*MIN_PB_SIZE];
  uint8_t pblock1_y[MIN_PB_SIZE*MIN_PB_SIZE];
  uint8_t pblock1_u[MIN_PB_SIZE*MIN_PB_SIZE];
  uint8_t pblock1_v[MIN_PB_SIZE*MIN_PB_SIZE];
  uint8_t pblock2_y[MIN_PB_SIZE*MIN_PB_SIZE];
  uint8_t pblock2_u[MIN_PB_SIZE*MIN_PB_SIZE];
  uint8_t pblock2_v[MIN_PB_SIZE*MIN_PB_SIZE];
  int sign;// , r0, r1;
  int yposY = block_pos->ypos;
  int xposY = block_pos->xpos;
  int size = block_pos->size;
  for (ypos0 = yposY; ypos0 < yposY + block_pos->bheight; ypos0 += MIN_PB_SIZE) {
    for (xpos0 = xposY; xpos0 < xposY + block_pos->bwidth; xpos0 += MIN_PB_SIZE) {

      m = ypos0 - yposY;
      n = xpos0 - xposY;
      tmp_block_pos.size = MIN_PB_SIZE;
      tmp_block_pos.bwidth = MIN_PB_SIZE;
      tmp_block_pos.bheight = MIN_PB_SIZE;
      tmp_block_pos.ypos = ypos0;
      tmp_block_pos.xpos = xpos0;

      block_stride = width / MIN_PB_SIZE;
      block_posy = ypos0 / MIN_PB_SIZE;
      block_posx = xpos0 / MIN_PB_SIZE;
      block_index = block_posy * block_stride + block_posx;

      mv_arr[0] = deblock_data[block_index].inter_pred_arr[phase].mv0;

      sign = 0;
      get_inter_prediction_yuv(ref0, pblock0_y, pblock0_u, pblock0_v, &tmp_block_pos, mv_arr, sign, width, height, 2, 0);

      sign = 1;
      if (gop_size == 3 && phase == 1) { //Support for 2B
        mv_arr[0].x = 2 * mv_arr[0].x;
        mv_arr[0].y = 2 * mv_arr[0].y;
      }
      get_inter_prediction_yuv(ref1, pblock1_y, pblock1_u, pblock1_v, &tmp_block_pos, mv_arr, sign, width, height, 2, 0);

      average_blocks_all(pblock2_y, pblock2_u, pblock2_v, pblock0_y, pblock0_u, pblock0_v, pblock1_y, pblock1_u, pblock1_v, &tmp_block_pos, ref0->sub);
      for (i = 0; i < MIN_PB_SIZE; i++) {
        memcpy(pblock_y + (m + i)*size + n, pblock2_y + i*MIN_PB_SIZE, MIN_PB_SIZE*sizeof(uint8_t));
      }
      for (i = 0; i < MIN_PB_SIZE / 2; i++) {
        memcpy(pblock_u + (m / 2 + i)*size / 2 + n / 2, pblock2_u + i*MIN_PB_SIZE / 2, MIN_PB_SIZE*sizeof(uint8_t) / 2);
        memcpy(pblock_v + (m / 2 + i)*size / 2 + n / 2, pblock2_v + i*MIN_PB_SIZE / 2, MIN_PB_SIZE*sizeof(uint8_t) / 2);
      }
    }
  }
}

mv_t get_mv_pred(int ypos,int xpos,int width,int height,int bwidth, int bheight, int sb_size,int ref_idx,deblock_data_t *deblock_data) //TODO: Remove ref_idx as argument if not needed
{
  mv_t mvp, mva, mvb, mvc;
  inter_pred_t zero_pred, inter_predA, inter_predB, inter_predC;
  int size = max(bwidth, bheight);

  /* Initialize zero unipred structure */
  zero_pred.mv0.x = 0;
  zero_pred.mv0.y = 0;
  zero_pred.ref_idx0 = 0;
  zero_pred.mv1.x = 0;
  zero_pred.mv1.y = 0;
  zero_pred.ref_idx1 = 0;
  zero_pred.bipred_flag = 0;

  inter_predA = zero_pred;
  inter_predB = zero_pred;
  inter_predC = zero_pred;

  /* Parameters values measured in units of 8 pixels */
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
  int downleft_index = block_index + block_stride*block_size - 1;
  int upright_index = block_index - block_stride + block_size;
  int upleft_index = block_index - block_stride - 1;

  /* Determine availability */
  int up_available = get_up_available(ypos, xpos, bwidth, bheight, width, height, sb_size);
  int left_available = get_left_available(ypos, xpos, bwidth, bheight, width, height, sb_size);
  int upright_available = get_upright_available(ypos, xpos, bwidth, bheight, width, height, sb_size);
  int downleft_available = get_downleft_available(ypos, xpos, bwidth, bheight, width, height, sb_size);

  int U = up_available;
  int UR = upright_available;
  int L = left_available;
  int DL = downleft_available;

  if (U==0 && UR==0 && L==0 && DL==0){
     inter_predA = zero_pred;
     inter_predB = zero_pred;
     inter_predC = zero_pred;
  }
  else if (U==1 && UR==0 && L==0 && DL==0){
     inter_predA = deblock_data[up_index0].inter_pred;
     inter_predB = deblock_data[up_index1].inter_pred;
     inter_predC = deblock_data[up_index2].inter_pred;
  }
  else if (U==1 && UR==1 && L==0 && DL==0){
     inter_predA = deblock_data[up_index0].inter_pred;
     inter_predB = deblock_data[up_index2].inter_pred;
     inter_predC = deblock_data[upright_index].inter_pred;
  }
  else if (U==0 && UR==0 && L==1 && DL==0){
     inter_predA = deblock_data[left_index0].inter_pred;
     inter_predB = deblock_data[left_index1].inter_pred;
     inter_predC = deblock_data[left_index2].inter_pred;
  }
  else if (U==1 && UR==0 && L==1 && DL==0){
     inter_predA = deblock_data[upleft_index].inter_pred;
     inter_predB = deblock_data[up_index2].inter_pred;
     inter_predC = deblock_data[left_index2].inter_pred;
  }
  else if (U==1 && UR==1 && L==1 && DL==0){
     inter_predA = deblock_data[up_index0].inter_pred;
     inter_predB = deblock_data[upright_index].inter_pred;
     inter_predC = deblock_data[left_index2].inter_pred;
  }
 else if (U==0 && UR==0 && L==1 && DL==1){
     inter_predA = deblock_data[left_index0].inter_pred;
     inter_predB = deblock_data[left_index2].inter_pred;
     inter_predC = deblock_data[downleft_index].inter_pred;
  }
 else if (U==1 && UR==0 && L==1 && DL==1){
     inter_predA = deblock_data[up_index2].inter_pred;
     inter_predB = deblock_data[left_index0].inter_pred;
     inter_predC = deblock_data[downleft_index].inter_pred;
  }
 else if (U==1 && UR==1 && L==1 && DL==1){
     inter_predA = deblock_data[up_index0].inter_pred;
     inter_predB = deblock_data[upright_index].inter_pred;
     inter_predC = deblock_data[left_index0].inter_pred;
  }
  else{
    printf("Error in mvp definition\n");
  }

  mva = inter_predA.mv0;
  mvb = inter_predB.mv0;
  mvc = inter_predC.mv0;

  /* Median */
  if (mva.x < mvb.x)
    mvp.x = min(mvb.x, max(mva.x, mvc.x));
  else
    mvp.x = min(mva.x, max(mvb.x, mvc.x));

  if (mva.y < mvb.y)
    mvp.y = min(mvb.y, max(mva.y, mvc.y));
  else
    mvp.y = min(mva.y, max(mvb.y, mvc.y));

  return mvp;
}

int get_mv_merge(int yposY, int xposY, int width, int height, int bwidth, int bheight, int sb_size, deblock_data_t *deblock_data, inter_pred_t *merge_candidates)
{
  int num_merge_vec = 0;
  int i, idx, duplicate;
  inter_pred_t zero_pred;
  inter_pred_t tmp_merge_candidates[MAX_NUM_SKIP];
  int size = max(bwidth, bheight);

  /* Initialize zero unipred structure */
  zero_pred.mv0.x = 0;
  zero_pred.mv0.y = 0;
  zero_pred.ref_idx0 = 0;
  zero_pred.mv1.x = 0;
  zero_pred.mv1.y = 0;
  zero_pred.ref_idx1 = 0;
  zero_pred.bipred_flag = 0;

  /* Parameters values measured in units of 4 pixels */
  int block_size = size / MIN_PB_SIZE;
  int block_stride = width / MIN_PB_SIZE;
  int block_posy = yposY / MIN_PB_SIZE;
  int block_posx = xposY / MIN_PB_SIZE;
  int block_index = block_posy * block_stride + block_posx;

  /* Block positions in units of 8x8 pixels */
  int up_index0 = block_index - block_stride;
  int up_index2 = block_index - block_stride + block_size - 1;
  int left_index0 = block_index - 1;
  int left_index2 = block_index + block_stride*(block_size - 1) - 1;
  int upright_index = block_index - block_stride + block_size;

  /* Determine availability */
  int up_available = get_up_available(yposY, xposY, bwidth, bheight, width, height, sb_size);
  int left_available = get_left_available(yposY, xposY, bwidth, bheight, width, height, sb_size);
  int upright_available = get_upright_available(yposY, xposY, bwidth, bheight, width, height, sb_size);
  //int downleft_available = get_downleft_available(yposY, xposY, bwidth, bheight, width, height, sb_size);

#if LIMITED_SKIP
  /* Special case for rectangular skip blocks at frame boundaries */
  if (yposY + size > height) {
    left_index2 = left_index0;
  }
  if (xposY + size > width) {
    up_index2 = up_index0;
  }
  if (left_available)
    tmp_merge_candidates[0] = deblock_data[left_index2].inter_pred;
  else
    tmp_merge_candidates[0] = zero_pred;
  if (upright_available)
    tmp_merge_candidates[1] = deblock_data[upright_index].inter_pred;
  else if (up_available)
    tmp_merge_candidates[1] = deblock_data[up_index2].inter_pred;
  else
    tmp_merge_candidates[1] = zero_pred;
#else
  int up_index1 = block_index - block_stride + (block_size - 1) / 2;
  int left_index1 = block_index + block_stride*((block_size - 1) / 2) - 1;
  int upleft_index = block_index - block_stride - 1;
  int downleft_index = block_index + block_stride*block_size - 1;
  //int downleft_available = get_downleft_available(yposY, xposY, size, height, sb_size);
  /* Special case for rectangular skip blocks at frame boundaries */
  if (yposY + size > height) {
    left_index1 = left_index2 = left_index0;
  }
  if (xposY + size > width) {
    up_index1 = up_index2 = up_index0;
  }

  int U = up_available;
  int UR = upright_available;
  int L = left_available;
  int DL = downleft_available;

  if (U == 0 && UR == 0 && L == 0 && DL == 0) {
    tmp_merge_candidates[0] = zero_pred;
    tmp_merge_candidates[1] = zero_pred;
    tmp_merge_candidates[2] = zero_pred;
    tmp_merge_candidates[3] = zero_pred;
  }
  else if (U == 1 && UR == 0 && L == 0 && DL == 0) {
    tmp_merge_candidates[0] = deblock_data[up_index0].inter_pred;
    tmp_merge_candidates[1] = deblock_data[up_index1].inter_pred;
    tmp_merge_candidates[2] = deblock_data[up_index2].inter_pred;
    tmp_merge_candidates[3] = deblock_data[up_index2].inter_pred;
  }
  else if (U == 1 && UR == 1 && L == 0 && DL == 0) {
    tmp_merge_candidates[0] = deblock_data[up_index0].inter_pred;
    tmp_merge_candidates[1] = deblock_data[up_index2].inter_pred;
    tmp_merge_candidates[2] = deblock_data[upright_index].inter_pred;
    tmp_merge_candidates[3] = deblock_data[upright_index].inter_pred;
  }
  else if (U == 0 && UR == 0 && L == 1 && DL == 0) {
    tmp_merge_candidates[0] = deblock_data[left_index0].inter_pred;
    tmp_merge_candidates[1] = deblock_data[left_index1].inter_pred;
    tmp_merge_candidates[2] = deblock_data[left_index2].inter_pred;
    tmp_merge_candidates[3] = deblock_data[left_index2].inter_pred;
  }
  else if (U == 1 && UR == 0 && L == 1 && DL == 0) {
    tmp_merge_candidates[0] = deblock_data[upleft_index].inter_pred;
    tmp_merge_candidates[1] = deblock_data[up_index2].inter_pred;
    tmp_merge_candidates[2] = deblock_data[left_index2].inter_pred;
    tmp_merge_candidates[3] = deblock_data[up_index0].inter_pred;
  }
  else if (U == 1 && UR == 1 && L == 1 && DL == 0) {
    tmp_merge_candidates[0] = deblock_data[up_index0].inter_pred;
    tmp_merge_candidates[1] = deblock_data[upright_index].inter_pred;
    tmp_merge_candidates[2] = deblock_data[left_index2].inter_pred;
    tmp_merge_candidates[3] = deblock_data[left_index0].inter_pred;
  }
  else if (U == 0 && UR == 0 && L == 1 && DL == 1) {
    tmp_merge_candidates[0] = deblock_data[left_index0].inter_pred;
    tmp_merge_candidates[1] = deblock_data[left_index2].inter_pred;
    tmp_merge_candidates[2] = deblock_data[downleft_index].inter_pred;
    tmp_merge_candidates[3] = deblock_data[downleft_index].inter_pred;
  }
  else if (U == 1 && UR == 0 && L == 1 && DL == 1) {
    tmp_merge_candidates[0] = deblock_data[up_index2].inter_pred;
    tmp_merge_candidates[1] = deblock_data[left_index0].inter_pred;
    tmp_merge_candidates[2] = deblock_data[downleft_index].inter_pred;
    tmp_merge_candidates[3] = deblock_data[up_index0].inter_pred;
  }
  else if (U == 1 && UR == 1 && L == 1 && DL == 1) {
    tmp_merge_candidates[0] = deblock_data[up_index0].inter_pred;
    tmp_merge_candidates[1] = deblock_data[upright_index].inter_pred;
    tmp_merge_candidates[2] = deblock_data[left_index0].inter_pred;
    tmp_merge_candidates[3] = deblock_data[downleft_index].inter_pred;
  }
  else {
    printf("Error in merge vector definition\n");
  }
#endif

  /* Remove duplicates */
  num_merge_vec = 1;
  merge_candidates[0] = tmp_merge_candidates[0];
  for (i = 1; i<MAX_NUM_SKIP; i++) {
    duplicate = 0;
    for (idx = 0; idx<num_merge_vec; idx++) {
      if (tmp_merge_candidates[i].mv0.x == merge_candidates[idx].mv0.x && tmp_merge_candidates[i].mv0.y == merge_candidates[idx].mv0.y &&
        tmp_merge_candidates[i].ref_idx0 == merge_candidates[idx].ref_idx0 &&
        tmp_merge_candidates[i].mv1.x == merge_candidates[idx].mv1.x && tmp_merge_candidates[i].mv1.y == merge_candidates[idx].mv1.y &&
        tmp_merge_candidates[i].ref_idx1 == merge_candidates[idx].ref_idx1 &&
        (tmp_merge_candidates[i].bipred_flag == merge_candidates[idx].bipred_flag || tmp_merge_candidates[i].bipred_flag == -1)) duplicate = 1; //TODO: proper handling fo dir for intra

    }
    if (duplicate == 0) {
      merge_candidates[num_merge_vec] = tmp_merge_candidates[i];
      num_merge_vec++;
    }
  }
  return num_merge_vec;
}

int get_mv_skip(int yposY, int xposY, int width, int height, int bwidth, int bheight, int sb_size, deblock_data_t *deblock_data, inter_pred_t *skip_candidates)
{
  int num_skip_vec=0;
  int i,idx,duplicate;
  inter_pred_t zero_pred;
  inter_pred_t tmp_skip_candidates[MAX_NUM_SKIP];
  int size = max(bwidth, bheight);

  /* Initialize zero unipred structure */
  zero_pred.mv0.x = 0;
  zero_pred.mv0.y = 0;
  zero_pred.ref_idx0 = 0;
  zero_pred.mv1.x = 0;
  zero_pred.mv1.y = 0;
  zero_pred.ref_idx1 = 0;
  zero_pred.bipred_flag = 0;

  /* Parameters values measured in units of 4 pixels */
  int block_size = size/MIN_PB_SIZE;
  int block_stride = width/MIN_PB_SIZE;
  int block_posy = yposY/MIN_PB_SIZE;
  int block_posx = xposY/MIN_PB_SIZE;
  int block_index = block_posy * block_stride + block_posx;

  /* Block positions in units of 8x8 pixels */
  int up_index0 = block_index - block_stride;
  int up_index2 = block_index - block_stride + block_size - 1;
  int left_index0 = block_index - 1;
  int left_index2 = block_index + block_stride*(block_size-1) - 1;
  int upright_index = block_index - block_stride + block_size;

  /* Determine availability */
  int up_available = get_up_available(yposY, xposY, bwidth, bheight, width, height, sb_size);
  int left_available = get_left_available(yposY, xposY, bwidth, bheight, width, height, sb_size);
  int upright_available = get_upright_available(yposY, xposY, bwidth, bheight, width, height, sb_size);
  //int downleft_available = get_downleft_available(yposY, xposY, bwidth, bheight, width, height, sb_size);

#if LIMITED_SKIP
  /* Special case for rectangular skip blocks at frame boundaries */
  if (yposY + size > height) {
    left_index2 = left_index0;
  }
  if (xposY + size > width) {
    up_index2 = up_index0;
  }
  if (left_available)
    tmp_skip_candidates[0] = deblock_data[left_index2].inter_pred;
  else
    tmp_skip_candidates[0] = zero_pred;
  if (upright_available)
     tmp_skip_candidates[1] = deblock_data[upright_index].inter_pred;
  else if (up_available)
    tmp_skip_candidates[1] = deblock_data[up_index2].inter_pred;
  else
    tmp_skip_candidates[1] = zero_pred;
#else
  int up_index1 = block_index - block_stride + (block_size - 1) / 2;
  int left_index1 = block_index + block_stride*((block_size - 1) / 2) - 1;
  int upleft_index = block_index - block_stride - 1;
  int downleft_index = block_index + block_stride*block_size - 1;
  //int downleft_available = get_downleft_available(yposY, xposY, size, height, sb_size);
  /* Special case for rectangular skip blocks at frame boundaries */
  if (yposY + size > height) {
    left_index1 = left_index2 = left_index0;
  }
  if (xposY + size > width) {
    up_index1 = up_index2 = up_index0;
  }

  int U = up_available;
  int UR = upright_available;
  int L = left_available;
  int DL = downleft_available;

  if (U==0 && UR==0 && L==0 && DL==0){
    tmp_skip_candidates[0] = zero_pred;
    tmp_skip_candidates[1] = zero_pred;
    tmp_skip_candidates[2] = zero_pred;
    tmp_skip_candidates[3] = zero_pred;
  }
  else if (U == 1 && UR == 0 && L == 0 && DL == 0) {
    tmp_skip_candidates[0] = deblock_data[up_index0].inter_pred;
    tmp_skip_candidates[1] = deblock_data[up_index1].inter_pred;
    tmp_skip_candidates[2] = deblock_data[up_index2].inter_pred;
    tmp_skip_candidates[3] = deblock_data[up_index2].inter_pred;
  }
  else if (U==1 && UR==1 && L==0 && DL==0){
    tmp_skip_candidates[0] = deblock_data[up_index0].inter_pred;
    tmp_skip_candidates[1] = deblock_data[up_index2].inter_pred;
    tmp_skip_candidates[2] = deblock_data[upright_index].inter_pred;
    tmp_skip_candidates[3] = deblock_data[upright_index].inter_pred;
  }
  else if (U==0 && UR==0 && L==1 && DL==0){
    tmp_skip_candidates[0] = deblock_data[left_index0].inter_pred;
    tmp_skip_candidates[1] = deblock_data[left_index1].inter_pred;
    tmp_skip_candidates[2] = deblock_data[left_index2].inter_pred;
    tmp_skip_candidates[3] = deblock_data[left_index2].inter_pred;
  }
  else if (U==1 && UR==0 && L==1 && DL==0){
    tmp_skip_candidates[0] = deblock_data[upleft_index].inter_pred;
    tmp_skip_candidates[1] = deblock_data[up_index2].inter_pred;
    tmp_skip_candidates[2] = deblock_data[left_index2].inter_pred;
    tmp_skip_candidates[3] = deblock_data[up_index0].inter_pred;
  }
  else if (U==1 && UR==1 && L==1 && DL==0){
    tmp_skip_candidates[0] = deblock_data[up_index0].inter_pred;
    tmp_skip_candidates[1] = deblock_data[upright_index].inter_pred;
    tmp_skip_candidates[2] = deblock_data[left_index2].inter_pred;
    tmp_skip_candidates[3] = deblock_data[left_index0].inter_pred;
  }
  else if (U==0 && UR==0 && L==1 && DL==1){
   tmp_skip_candidates[0] = deblock_data[left_index0].inter_pred;
    tmp_skip_candidates[1] = deblock_data[left_index2].inter_pred;
    tmp_skip_candidates[2] = deblock_data[downleft_index].inter_pred;
    tmp_skip_candidates[3] = deblock_data[downleft_index].inter_pred;
  }
  else if (U==1 && UR==0 && L==1 && DL==1){
    tmp_skip_candidates[0] = deblock_data[up_index2].inter_pred;
    tmp_skip_candidates[1] = deblock_data[left_index0].inter_pred;
    tmp_skip_candidates[2] = deblock_data[downleft_index].inter_pred;
    tmp_skip_candidates[3] = deblock_data[up_index0].inter_pred;
  }
  else if (U==1 && UR==1 && L==1 && DL==1){
    tmp_skip_candidates[0] = deblock_data[up_index0].inter_pred;
    tmp_skip_candidates[1] = deblock_data[upright_index].inter_pred;
    tmp_skip_candidates[2] = deblock_data[left_index0].inter_pred;
    tmp_skip_candidates[3] = deblock_data[downleft_index].inter_pred;
  }
  else{
    printf("Error in skip vector definition\n");
  }
#endif

  /* Remove duplicates */
  num_skip_vec = 1;
  skip_candidates[0] = tmp_skip_candidates[0];
  for (i=1;i<MAX_NUM_SKIP;i++){
    duplicate = 0;
    for (idx=0; idx<num_skip_vec; idx++){
      if (tmp_skip_candidates[i].mv0.x == skip_candidates[idx].mv0.x && tmp_skip_candidates[i].mv0.y == skip_candidates[idx].mv0.y &&
          tmp_skip_candidates[i].ref_idx0 == skip_candidates[idx].ref_idx0 &&
          tmp_skip_candidates[i].mv1.x == skip_candidates[idx].mv1.x && tmp_skip_candidates[i].mv1.y == skip_candidates[idx].mv1.y &&
          tmp_skip_candidates[i].ref_idx1 == skip_candidates[idx].ref_idx1 &&
          (tmp_skip_candidates[i].bipred_flag == skip_candidates[idx].bipred_flag || tmp_skip_candidates[i].bipred_flag == -1)) duplicate = 1; //TODO: proper handling fo dir for intra

    }
    if (duplicate==0){
      skip_candidates[num_skip_vec] = tmp_skip_candidates[i];
      num_skip_vec++;
    }
  }
  return num_skip_vec;
}

int get_mv_skip_temp(int width, int phase, int gop_size, block_pos_t *block_pos, deblock_data_t *deblock_data, inter_pred_t *skip_candidates)
{
  int m, n;
  int num_skip_vec;
  int block_posy = block_pos->ypos / MIN_PB_SIZE;
  int block_posx = block_pos->xpos / MIN_PB_SIZE;
  int block_stride = width / MIN_PB_SIZE;
  int block_index;
  int bwidth = block_pos->bwidth;
  int bheight = block_pos->bheight;
  mv_t mv0, mv1;
  int ref_idx0, ref_idx1, bipred_flag;
  int duplicate = 1;
  for (m = 0; m < bheight / MIN_PB_SIZE; m++) {
    for (n = 0; n < bwidth / MIN_PB_SIZE; n++) {
      block_index = (block_posy + m)*block_stride + block_posx + n;
      mv0 = deblock_data[block_index].inter_pred_arr[phase].mv0;
      mv1 = deblock_data[block_index].inter_pred_arr[phase].mv0;
      if (gop_size == 3 && phase == 1) {
        mv1.x *= 2;
        mv1.y *= 2;
      }
      ref_idx0 = 0;
      ref_idx1 = 1;
      bipred_flag = 2;
      if (mv0.x != skip_candidates[0].mv0.x) duplicate = 0;
      if (mv0.y != skip_candidates[0].mv0.y) duplicate = 0;
      if (mv1.x != skip_candidates[0].mv1.x) duplicate = 0;
      if (mv1.y != skip_candidates[0].mv1.y) duplicate = 0;
      if (ref_idx0 != skip_candidates[0].ref_idx0) duplicate = 0;
      if (ref_idx1 != skip_candidates[0].ref_idx1) duplicate = 0;
      if (bipred_flag != skip_candidates[0].bipred_flag) duplicate = 0;
    }
  }
  if (!duplicate) {
    num_skip_vec = 2;
    skip_candidates[1] = skip_candidates[0];
  }
  else {
   num_skip_vec = 1;
  }
  skip_candidates[0].ref_idx0 = 0;
  skip_candidates[0].ref_idx1 = 1;
  skip_candidates[0].bipred_flag = 2;
  return num_skip_vec;
}
