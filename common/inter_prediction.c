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

void get_inter_prediction_chroma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign, int pic_width2, int pic_height2, int xpos, int ypos)
{
  int i,j;

  int m,i_off,j_off;
  mv_t mvtemp;
  mvtemp.x = sign ? -mv->x : mv->x;
  mvtemp.y = sign ? -mv->y : mv->y;
  int ver_frac = (mvtemp.y)&7;
  int hor_frac = (mvtemp.x)&7;
  int ver_int = (mvtemp.y)>>3;
  int hor_int = (mvtemp.x)>>3;
  ver_int = min(ver_int,pic_height2-ypos);
  ver_int = max(ver_int,-xpos-height);
  hor_int = min(hor_int,pic_width2-xpos);
  hor_int = max(hor_int,-xpos-width);
  int16_t tmp[80][80];

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
  else if (ver_frac == 2 && hor_frac == 2) {
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

mv_t get_mv_pred(int ypos,int xpos,int width,int height,int size,int ref_idx,deblock_data_t *deblock_data) //TODO: Remove ref_idx as argument if not needed
{
  mv_t mvp, mva, mvb, mvc;
  inter_pred_t zero_pred, inter_predA, inter_predB, inter_predC;

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
  int up_available = get_up_available(ypos,xpos,size,width);
  int left_available = get_left_available(ypos,xpos,size,width);
  int upright_available = get_upright_available(ypos,xpos,size,width);
  int downleft_available = get_downleft_available(ypos,xpos,size,height);

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

int get_mv_merge(int yposY, int xposY, int width, int height, int size, deblock_data_t *deblock_data, inter_pred_t *merge_candidates)
{
  int num_merge_vec = 0;
  int i, idx, duplicate;
  inter_pred_t zero_pred;
  inter_pred_t tmp_merge_candidates[MAX_NUM_SKIP];

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
  int up_available = get_up_available(yposY, xposY, size, width);
  int left_available = get_left_available(yposY, xposY, size, width);
  int upright_available = get_upright_available(yposY, xposY, size, width);

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
  int downleft_available = get_downleft_available(yposY, xposY, size, height);

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

int get_mv_skip(int yposY, int xposY, int width, int height, int size, deblock_data_t *deblock_data, inter_pred_t *skip_candidates)
{
  int num_skip_vec=0;
  int i,idx,duplicate;
  inter_pred_t zero_pred;
  inter_pred_t tmp_skip_candidates[MAX_NUM_SKIP];

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
  int up_available = get_up_available(yposY,xposY,size,width);
  int left_available = get_left_available(yposY,xposY,size,width);
  int upright_available = get_upright_available(yposY,xposY,size,width);

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
  int downleft_available = get_downleft_available(yposY, xposY, size, height);

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
