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

#if HEVC_INTERPOLATION
#define NTAPY 8
#define OFFY (NTAPY/2)
#define OFFYM1 (OFFY-1)
static const int16_t filter_coeffsY[4][8] = {
    { 0,  0,  0, 64,  0,  0, 0,  0},
    {-1,  4,-10, 58, 17, -5, 1,  0},
    {-1,  4,-11, 40, 40,-11, 4, -1},
    { 0,  1, -5, 17, 58,-10, 4, -1}
};
#else
#define NTAPY 6
#define OFFY (NTAPY/2)
#define OFFYM1 (OFFY-1)
static const int16_t filter_coeffsY[4][8] = {
    { 0,  0,128,  0,  0,  0},
    { 3,-15,111, 37,-10,  2},
    { 3,-17, 78, 78,-17,  3},
    { 2,-10, 37,111,-15,  3}
};
#endif

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

void get_inter_prediction_chroma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign)
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

void get_inter_prediction_luma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign)
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
  int32_t tmp[80][80]; //7-bit filter exceeds 16 bit temporary storage

  /* Integer position */
  if (ver_frac==0 && hor_frac==0){
    j_off = 0 + hor_int;
    for(i=0;i<height;i++){
      i_off = i + ver_int;
      memcpy(pblock + i*pstride,ref + i_off*stride+j_off, width*sizeof(uint8_t));
    }
    return;
  }

#if HEVC_INTERPOLATION
  /* Vertical filtering */
  for(i=-OFFYM1;i<width+OFFY;i++){
    for (j=0;j<height;j++){
      int sum = 0;
      i_off = i + hor_int;
      j_off = j + ver_int;
      for (m=0;m<NTAPY;m++) sum += filter_coeffsY[ver_frac][m] * ref[(j_off + m - OFFYM1) * stride + i_off];
      tmp[j][i+OFFYM1] = sum;
    }
  }
  /* Horizontal filtering */
  for(i=0;i<width;i++){
    for (j=0;j<height;j++){
      int sum = 0;
      for (m=0;m<NTAPY;m++) sum += filter_coeffsY[hor_frac][m] * tmp[j][i+m];
      pblock[j*pstride+i] = clip255((sum + 2048)>>12);
    }
  }
  return;
#endif

  if (use_simd) {
    get_inter_prediction_luma_simd(width, height, hor_frac, ver_frac, pblock, pstride, ref + ver_int*stride + hor_int, stride);
  }
  else {
    /* Special lowpass filter at center position */
    if (ver_frac == 2 && hor_frac == 2) {
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
      for(i=-OFFYM1;i<width+OFFY;i++){
        for (j=0;j<height;j++){
          int sum = 0;
          i_off = i + hor_int;
          j_off = j + ver_int;
          for (m=0;m<NTAPY;m++) sum += filter_coeffsY[ver_frac][m] * ref[(j_off + m - OFFYM1) * stride + i_off]; //7-bit version
          tmp[j][i+OFFYM1] = sum;
        }
      }
      /* Horizontal filtering */
      for(i=0;i<width;i++){
        for (j=0;j<height;j++){
          int sum = 0;
          for (m=0;m<NTAPY;m++) sum += filter_coeffsY[hor_frac][m] * tmp[j][i+m]; //7-bit version
          pblock[j*pstride+i] = clip255((sum + 8192)>>14); //7-bit version
        }
      }
    }
  }
}

mv_t get_mv_pred(int ypos,int xpos,int width,int height,int size,int ref_idx,deblock_data_t *deblock_data) //TODO: Remove ref_idx as argument if not needed
{
  mv_t mvp;
  mvr_t zerovec;
  mvr_t mva,mvb,mvc;
  zerovec.x = 0;
  zerovec.y = 0;
  zerovec.ref_idx = 0;

  mva = zerovec;
  mvb = zerovec;
  mvc = zerovec;


  mvb_t zerovecb;
  mvb_t mvba,mvbb,mvbc;
  zerovecb.x0 = 0;
  zerovecb.y0 = 0;
  zerovecb.ref_idx0 = 0;
  zerovecb.x1 = 0;
  zerovecb.y1 = 0;
  zerovecb.ref_idx1 = 0;
  zerovecb.dir = 0;

   mvba = zerovecb;
   mvbb = zerovecb;
   mvbc = zerovecb;

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
  int left_index1 = block_index + block_stride*((block_size - 1)/2) - 1;
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
     mvba = zerovecb;
     mvbb = zerovecb;
     mvbc = zerovecb;
  }
  else if (U==1 && UR==0 && L==0 && DL==0){
     mvba = deblock_data[up_index0].mvb;
     mvbb = deblock_data[up_index1].mvb;
     mvbc = deblock_data[up_index2].mvb;
  }
  else if (U==1 && UR==1 && L==0 && DL==0){
     mvba = deblock_data[up_index0].mvb;
     mvbb = deblock_data[up_index2].mvb;
     mvbc = deblock_data[upright_index].mvb;
  }
  else if (U==0 && UR==0 && L==1 && DL==0){
     mvba = deblock_data[left_index0].mvb;
     mvbb = deblock_data[left_index1].mvb;
     mvbc = deblock_data[left_index2].mvb;
  }
  else if (U==1 && UR==0 && L==1 && DL==0){
     mvba = deblock_data[upleft_index].mvb;
     mvbb = deblock_data[up_index2].mvb;
     mvbc = deblock_data[left_index2].mvb;
  }
  else if (U==1 && UR==1 && L==1 && DL==0){
     mvba = deblock_data[up_index0].mvb;
     mvbb = deblock_data[upright_index].mvb;
     mvbc = deblock_data[left_index2].mvb;
  }
 else if (U==0 && UR==0 && L==1 && DL==1){
     mvba = deblock_data[left_index0].mvb;
     mvbb = deblock_data[left_index2].mvb;
     mvbc = deblock_data[downleft_index].mvb;
  }
 else if (U==1 && UR==0 && L==1 && DL==1){
     mvba = deblock_data[up_index2].mvb;
     mvbb = deblock_data[left_index0].mvb;
     mvbc = deblock_data[downleft_index].mvb;
  }
 else if (U==1 && UR==1 && L==1 && DL==1){
     mvba = deblock_data[up_index0].mvb;
     mvbb = deblock_data[upright_index].mvb;
     mvbc = deblock_data[left_index0].mvb;
  }
  else{
    printf("Error in mvp definition\n");
  }
  mva.x = mvba.x0;
  mva.y = mvba.y0;
  mvb.x = mvbb.x0;
  mvb.y = mvbb.y0;
  mvc.x = mvbc.x0;
  mvc.y = mvbc.y0;


  /* Median */
  if (mva.x < mvb.x)
    mvp.x = min(mvb.x,max(mva.x,mvc.x));
  else
    mvp.x = min(mva.x,max(mvb.x,mvc.x));

  if (mva.y < mvb.y)
    mvp.y = min(mvb.y,max(mva.y,mvc.y));
  else
    mvp.y = min(mva.y,max(mvb.y,mvc.y));
  return mvp;
}

int get_mv_merge(int yposY,int xposY,int width,int height,int size,deblock_data_t *deblock_data, mvb_t *mvb_skip)
{
  int num_skip_vec=0;
  int idx,duplicate;
#if LIMITED_SKIP
  mvb_t tmb_skip[4];
#else
  mvb_t tmb_skip[MAX_NUM_SKIP];
#endif
  mvb_t zerovecb;
  zerovecb.dir = 0;
  zerovecb.x0 = 0;
  zerovecb.y0 = 0;
  zerovecb.ref_idx0 = 0;
  zerovecb.x1 = 0;
  zerovecb.y1 = 0;
  zerovecb.ref_idx1 = 0;

  /* Parameters values measured in units of 4 pixels */
  int block_size = size/MIN_PB_SIZE;
  int block_stride = width/MIN_PB_SIZE;
  int block_posy = yposY/MIN_PB_SIZE;
  int block_posx = xposY/MIN_PB_SIZE;
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


  /* Special case for rectangular skip blocks at frame boundaries */
  if (yposY+size > height){
    left_index1 = left_index2 = left_index0;
  }
  if (xposY+size > width){
    up_index1 = up_index2 = up_index0;
  }

  /* Determine availability */
  int up_available = get_up_available(yposY,xposY,size,width);
  int left_available = get_left_available(yposY,xposY,size,width);
  int upright_available = get_upright_available(yposY,xposY,size,width);
  int downleft_available = get_downleft_available(yposY,xposY,size,height);

  int U = up_available;
  int UR = upright_available;
  int L = left_available;
  int DL = downleft_available;

  if (U==0 && UR==0 && L==0 && DL==0){
    tmb_skip[0] = zerovecb;
    tmb_skip[1] = zerovecb;
    tmb_skip[2] = zerovecb;
    tmb_skip[3] = zerovecb;
  }
  else if (U==1 && UR==0 && L==0 && DL==0){
    tmb_skip[0] = deblock_data[up_index0].mvb;
    tmb_skip[1] = deblock_data[up_index1].mvb;
    tmb_skip[2] = deblock_data[up_index2].mvb;
    tmb_skip[3] = deblock_data[up_index2].mvb;
  }
  else if (U==1 && UR==1 && L==0 && DL==0){
    tmb_skip[0] = deblock_data[up_index0].mvb;
    tmb_skip[1] = deblock_data[up_index2].mvb;
    tmb_skip[2] = deblock_data[upright_index].mvb;
    tmb_skip[3] = deblock_data[upright_index].mvb;
  }
  else if (U==0 && UR==0 && L==1 && DL==0){
    tmb_skip[0] = deblock_data[left_index0].mvb;
    tmb_skip[1] = deblock_data[left_index1].mvb;
    tmb_skip[2] = deblock_data[left_index2].mvb;
    tmb_skip[3] = deblock_data[left_index2].mvb;
  }
  else if (U==1 && UR==0 && L==1 && DL==0){
    tmb_skip[0] = deblock_data[upleft_index].mvb;
    tmb_skip[1] = deblock_data[up_index2].mvb;
    tmb_skip[2] = deblock_data[left_index2].mvb;
    tmb_skip[3] = deblock_data[up_index0].mvb;
  }
  else if (U==1 && UR==1 && L==1 && DL==0){
    tmb_skip[0] = deblock_data[up_index0].mvb;
    tmb_skip[1] = deblock_data[upright_index].mvb;
    tmb_skip[2] = deblock_data[left_index2].mvb;
    tmb_skip[3] = deblock_data[left_index0].mvb;
  }
  else if (U==0 && UR==0 && L==1 && DL==1){
    tmb_skip[0] = deblock_data[left_index0].mvb;
    tmb_skip[1] = deblock_data[left_index2].mvb;
    tmb_skip[2] = deblock_data[downleft_index].mvb;
    tmb_skip[3] = deblock_data[downleft_index].mvb;
  }
  else if (U==1 && UR==0 && L==1 && DL==1){
    tmb_skip[0] = deblock_data[up_index2].mvb;
    tmb_skip[1] = deblock_data[left_index0].mvb;
    tmb_skip[2] = deblock_data[downleft_index].mvb;
    tmb_skip[3] = deblock_data[up_index0].mvb;
  }
  else if (U==1 && UR==1 && L==1 && DL==1){
    tmb_skip[0] = deblock_data[up_index0].mvb;
    tmb_skip[1] = deblock_data[upright_index].mvb;
    tmb_skip[2] = deblock_data[left_index0].mvb;
    tmb_skip[3] = deblock_data[downleft_index].mvb;
  }
  else{
    printf("Error in skip vector definition\n");
  }

#if LIMITED_SKIP
  if (left_available)
    tmb_skip[0] = deblock_data[left_index2].mvb;
  else
    tmb_skip[0] = zerovecb;
  if (upright_available)
    tmb_skip[1] = deblock_data[upright_index].mvb;
  else if (up_available)
    tmb_skip[1] = deblock_data[up_index2].mvb;
  else
    tmb_skip[1] = zerovecb;
#endif

  int i;
  /* Remove duplicates */
  num_skip_vec = 1;
#if ENABLE_SKIP_BIPRED
#else
  for (i=0;i<MAX_NUM_SKIP;i++){
    tmb_skip[i].dir = 0;
    tmb_skip[i].x1 =  tmb_skip[i].x0;
    tmb_skip[i].y1 =  tmb_skip[i].y0;
    tmb_skip[i].ref_idx1 = tmb_skip[i].ref_idx0;
  }
#endif
  mvb_skip[0] = tmb_skip[0];

  for (i=1;i<MAX_NUM_SKIP;i++){
    duplicate = 0;
    for (idx=0; idx<num_skip_vec; idx++){
      if (tmb_skip[i].x0 == mvb_skip[idx].x0 && tmb_skip[i].y0 == mvb_skip[idx].y0 && tmb_skip[i].ref_idx0 == mvb_skip[idx].ref_idx0 &&
          tmb_skip[i].x1 == mvb_skip[idx].x1 && tmb_skip[i].y1 == mvb_skip[idx].y1 && tmb_skip[i].ref_idx1 == mvb_skip[idx].ref_idx1 &&
          (tmb_skip[i].dir == mvb_skip[idx].dir || tmb_skip[i].dir == -1)) duplicate = 1; //TODO: proper handling fo dir for intra
    }
    if (duplicate==0){
      mvb_skip[num_skip_vec++] = tmb_skip[i];
    }
  }

  return num_skip_vec;
}

int get_mv_skip(int yposY,int xposY,int width,int height,int size,deblock_data_t *deblock_data, mvb_t *mvb_skip)
{
  int num_skip_vec=0;
  int idx,duplicate;
#if LIMITED_SKIP
  mvb_t tmb_skip[4];
#else
  mvb_t tmb_skip[MAX_NUM_SKIP];
#endif
  mvb_t zerovecb;
  zerovecb.dir = 0;
  zerovecb.x0 = 0;
  zerovecb.y0 = 0;
  zerovecb.ref_idx0 = 0;
  zerovecb.x1 = 0;
  zerovecb.y1 = 0;
  zerovecb.ref_idx1 = 0;

  /* Parameters values measured in units of 4 pixels */
  int block_size = size/MIN_PB_SIZE;
  int block_stride = width/MIN_PB_SIZE;
  int block_posy = yposY/MIN_PB_SIZE;
  int block_posx = xposY/MIN_PB_SIZE;
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


  /* Special case for rectangular skip blocks at frame boundaries */
  if (yposY+size > height){
    left_index1 = left_index2 = left_index0;
  }
  if (xposY+size > width){
    up_index1 = up_index2 = up_index0;
  }

  /* Determine availability */
  int up_available = get_up_available(yposY,xposY,size,width);
  int left_available = get_left_available(yposY,xposY,size,width);
  int upright_available = get_upright_available(yposY,xposY,size,width);
  int downleft_available = get_downleft_available(yposY,xposY,size,height);

  int U = up_available;
  int UR = upright_available;
  int L = left_available;
  int DL = downleft_available;

  if (U==0 && UR==0 && L==0 && DL==0){
    tmb_skip[0] = zerovecb;
    tmb_skip[1] = zerovecb;
    tmb_skip[2] = zerovecb;
    tmb_skip[3] = zerovecb;
  }
  else if (U==1 && UR==0 && L==0 && DL==0){
    tmb_skip[0] = deblock_data[up_index0].mvb;
    tmb_skip[1] = deblock_data[up_index1].mvb;
    tmb_skip[2] = deblock_data[up_index2].mvb;
    tmb_skip[3] = deblock_data[up_index2].mvb;
  }
  else if (U==1 && UR==1 && L==0 && DL==0){
    tmb_skip[0] = deblock_data[up_index0].mvb;
    tmb_skip[1] = deblock_data[up_index2].mvb;
    tmb_skip[2] = deblock_data[upright_index].mvb;
    tmb_skip[3] = deblock_data[upright_index].mvb;
  }
  else if (U==0 && UR==0 && L==1 && DL==0){
    tmb_skip[0] = deblock_data[left_index0].mvb;
    tmb_skip[1] = deblock_data[left_index1].mvb;
    tmb_skip[2] = deblock_data[left_index2].mvb;
    tmb_skip[3] = deblock_data[left_index2].mvb;
  }
  else if (U==1 && UR==0 && L==1 && DL==0){
    tmb_skip[0] = deblock_data[upleft_index].mvb;
    tmb_skip[1] = deblock_data[up_index2].mvb;
    tmb_skip[2] = deblock_data[left_index2].mvb;
    tmb_skip[3] = deblock_data[up_index0].mvb;
  }
  else if (U==1 && UR==1 && L==1 && DL==0){
    tmb_skip[0] = deblock_data[up_index0].mvb;
    tmb_skip[1] = deblock_data[upright_index].mvb;
    tmb_skip[2] = deblock_data[left_index2].mvb;
    tmb_skip[3] = deblock_data[left_index0].mvb;
  }
  else if (U==0 && UR==0 && L==1 && DL==1){
    tmb_skip[0] = deblock_data[left_index0].mvb;
    tmb_skip[1] = deblock_data[left_index2].mvb;
    tmb_skip[2] = deblock_data[downleft_index].mvb;
    tmb_skip[3] = deblock_data[downleft_index].mvb;
  }
  else if (U==1 && UR==0 && L==1 && DL==1){
    tmb_skip[0] = deblock_data[up_index2].mvb;
    tmb_skip[1] = deblock_data[left_index0].mvb;
    tmb_skip[2] = deblock_data[downleft_index].mvb;
    tmb_skip[3] = deblock_data[up_index0].mvb;
  }
  else if (U==1 && UR==1 && L==1 && DL==1){
    tmb_skip[0] = deblock_data[up_index0].mvb;
    tmb_skip[1] = deblock_data[upright_index].mvb;
    tmb_skip[2] = deblock_data[left_index0].mvb;
    tmb_skip[3] = deblock_data[downleft_index].mvb;
  }
  else{
    printf("Error in skip vector definition\n");
  }

#if LIMITED_SKIP
  if (left_available)
    tmb_skip[0] = deblock_data[left_index2].mvb;
  else
    tmb_skip[0] = zerovecb;
  if (upright_available)
    tmb_skip[1] = deblock_data[upright_index].mvb;
  else if (up_available)
    tmb_skip[1] = deblock_data[up_index2].mvb;
  else
    tmb_skip[1] = zerovecb;
#endif

  int i;
  /* Remove duplicates */
  num_skip_vec = 1;
#if ENABLE_SKIP_BIPRED
#else
  for (i=0;i<MAX_NUM_SKIP;i++){
    tmb_skip[i].dir = 0;
    tmb_skip[i].x1 =  tmb_skip[i].x0;
    tmb_skip[i].y1 =  tmb_skip[i].y0;
    tmb_skip[i].ref_idx1 = tmb_skip[i].ref_idx0;
  }
#endif
  mvb_skip[0] = tmb_skip[0];

  for (i=1;i<MAX_NUM_SKIP;i++){
    duplicate = 0;
    for (idx=0; idx<num_skip_vec; idx++){
      if (tmb_skip[i].x0 == mvb_skip[idx].x0 && tmb_skip[i].y0 == mvb_skip[idx].y0 && tmb_skip[i].ref_idx0 == mvb_skip[idx].ref_idx0 &&
          tmb_skip[i].x1 == mvb_skip[idx].x1 && tmb_skip[i].y1 == mvb_skip[idx].y1 && tmb_skip[i].ref_idx1 == mvb_skip[idx].ref_idx1 &&
          (tmb_skip[i].dir == mvb_skip[idx].dir || tmb_skip[i].dir == -1)) duplicate = 1; //TODO: proper handling fo dir for intra
    }
    if (duplicate==0){
      mvb_skip[num_skip_vec++] = tmb_skip[i];
    }
  }

#if NO_SUBBLOCK_SKIP
  if (size<MAX_BLOCK_SIZE){
    tmb_skip[0].dir = 0;
    tmb_skip[0].x0 = 0;
    tmb_skip[0].y0 = 0;
    tmb_skip[0].ref_idx0 = 0;
    tmb_skip[0].x1 = 0;
    tmb_skip[0].y1 = 0;
    tmb_skip[0].ref_idx1 = 0;
    mvb_skip[0] = tmb_skip[0];
    num_skip_vec = 1;
  }
#endif

  return num_skip_vec;
}
