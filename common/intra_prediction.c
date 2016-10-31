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
#include "intra_prediction.h"

static void filter_121(SAMPLE* in, SAMPLE* out, int len)
{
  int j;
  /* Calculate filtered 1D arrays */
  out[0] = (SAMPLE)((in[0] + 2*in[0] + in[1] + 2)>>2);
  for (j=1;j<len-1;j++){
    out[j] = (SAMPLE)((in[j-1] + 2*in[j] + in[j+1] + 2)>>2);
  }
  out[len-1] = (SAMPLE)((in[len-2] + 2*in[len-1] + in[len-1] + 2)>>2);
}

static void filter_121_all(SAMPLE* left_in, SAMPLE* left_out, SAMPLE* top_in, SAMPLE* top_out, int len, SAMPLE tl_in, SAMPLE* tl_out)
{
  filter_121(left_in,left_out,len);
  filter_121(top_in,top_out,len);
  *tl_out = (2*tl_in+left_in[0]+top_in[0]+2)>>2;
}

void TEMPLATE(make_top_and_left)(SAMPLE* left, SAMPLE* top, SAMPLE* top_left, SAMPLE* rec_frame, int fstride, SAMPLE* rblock, int rbstride, int i, int j,
                                 int ypos, int xpos, int size, int cb_upright_available, int cb_downleft_available, int tb_split, int bitdepth)
{
  // xpos, ypos are CB coords; j,i are coords within the CB
  // Assuming *raster* scan of TUs
  // Padding by a single extra pixel in case up-right or down-left are available
  SAMPLE val;
  int len = 2*size;
  int toplen, leftlen;
  int downleft_available,upright_available;

  if (!tb_split) {
    assert(i==0 && j==0);
    downleft_available = cb_downleft_available;
    leftlen = downleft_available ? (size+1) : size;

    upright_available = cb_upright_available;
    toplen = upright_available ? (size+1) : size;

    if (ypos==0) {
      if (sizeof(SAMPLE) == 1)
        memset(&top[0], 128, len);
      else
        for (int i = 0; i < len; i++)
          top[i] = 128 << (bitdepth-8);
      *top_left = 128 << (bitdepth-8);
    } else {
      memcpy(&top[0],&rec_frame[-fstride+j],toplen*sizeof(SAMPLE));
      val = top[toplen-1];
      if (sizeof(SAMPLE) == 1)
        memset(&top[size],val,size);
      else
        for (int i = 0; i < size; i++)
          top[size+i] = val;

      *top_left = xpos > 0 ? rec_frame[-fstride+j-1] : top[0];
    }

    if (xpos==0) {
      if (sizeof(SAMPLE) == 1)
        memset(&left[0], 128, len);
      else
        for (int i = 0; i < len; i++)
          left[i] = 128 << (bitdepth-8);
    } else {
      for (int k=0; k<leftlen; ++k){
        left[k] = rec_frame[k*fstride-1];
      }
      val = left[leftlen-1];
      if (sizeof(SAMPLE) == 1)
        memset(&left[size],val,size);
      else
        for (int i = 0; i < size; i++)
          left[size+i] = val;
    }

    if (ypos==0)
      *top_left = left[0];
  }
  else
  {
    downleft_available = (j==0 && (i==0 || cb_downleft_available)) ? 1 : 0;
    leftlen = downleft_available ? (size+1) : size;

    upright_available = (j==0 || (i==0 && cb_upright_available)) ? 1 : 0;
    toplen = upright_available ? (size+1) : size;

    if (ypos+i==0) {
      if (sizeof(SAMPLE) == 1)
        memset(&top[0], 128, len);
      else
        for (int i = 0; i < len; i++)
          top[i] = 128 << (bitdepth-8);
      *top_left = 128 << (bitdepth-8);
    } else if (i==0){
      memcpy(&top[0],&rec_frame[-fstride+j],toplen*sizeof(SAMPLE));
      val = top[toplen-1];
      if (sizeof(SAMPLE) == 1)
        memset(&top[size],val,size);
      else
        for (int i = 0; i < size; i++)
          top[size+i] = val;
      *top_left = xpos > 0 ? rec_frame[-fstride+j-1] : top[0];
    } else {
      memcpy(&top[0],&rblock[-rbstride],toplen*sizeof(SAMPLE));
      val = top[toplen-1];
      if (sizeof(SAMPLE) == 1)
        memset(&top[size],val,size);
      else
        for (int i = 0; i < size; i++)
          top[size+i] = val;
      *top_left = xpos > 0 ? (j>0 ? rblock[-rbstride-1] : rec_frame[(i-1)*fstride-1]) : top[0];
    }

    if (xpos+j==0) {
      if (sizeof(SAMPLE) == 1)
        memset(&left[0], 128, len);
      else
        for (int i = 0; i < len; i++)
          left[i] = 128 << (bitdepth-8);
    } else if (j==0){
      for (int k=0; k<leftlen; ++k){
        left[k] = rec_frame[(i+k)*fstride-1];
      }
      val = left[leftlen-1];
      if (sizeof(SAMPLE) == 1)
        memset(&left[size],val,size);
      else
        for (int i = 0; i < size; i++)
          left[size+i] = val;

    } else {
      for (int k=0; k<leftlen; ++k){
        left[k] = rblock[k*rbstride-1];
      }
      val = left[leftlen-1];
      if (sizeof(SAMPLE) == 1)
        memset(&left[size],val,size);
      else
        for (int i = 0; i < size; i++)
          left[size+i] = val;
    }

    if (ypos+i==0)
      *top_left = left[0];
  }
}

void TEMPLATE(get_dc_pred)(SAMPLE* left, SAMPLE* top, int size, SAMPLE *pblock, int pstride, int bitdepth){
  unsigned int i,j,dc=128 << (bitdepth-8),sum;

  sum = 0;
  for (j=0;j<size;j++) sum += top[j];
  for (i=0;i<size;i++) sum += left[i];
  dc = (sum + size)/(2*size);

  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      pblock[i*pstride+j] = dc;
    }
  }
}


void TEMPLATE(get_hor_pred)(SAMPLE* left, int size, SAMPLE *pblock, int pstride) {
  int i,j;

  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      pblock[i*pstride+j] = left[i];
    }
  }
}


void TEMPLATE(get_ver_pred)(SAMPLE* top, int size, SAMPLE *pblock, int pstride) {
  int i,j;

  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      pblock[i*pstride+j] = top[j];
    }
  }
}

void TEMPLATE(get_planar_pred)(SAMPLE* left, SAMPLE* top, SAMPLE top_left, int size, SAMPLE *pblock, int pstride, int bitdepth) {
  int i,j;

  int16_t topF[MAX_TR_SIZE];
  int16_t leftF[MAX_TR_SIZE];
  int16_t top_leftF;

  /* Calculate filtered 1D arrays */
  topF[0] = top[0] + 2* top[0] + 2*top[0] + 2* top[1] +top[2];
  topF[1] = top[0] + 2* top[0] + 2*top[1] + 2* top[2] +top[3];
  for (j=2;j<size-2;j++){
    topF[j] = top[j-2] + 2*top[j-1] + 2*top[j] + 2*top[j+1] + top[j+2];
  }
  topF[size-2] = top[size-4]+ 2*top[size-3] + 2*top[size-2] + 2* top[size-1] + top[size-1];
  topF[size-1] = top[size-3]+ 2*top[size-2] + 2*top[size-1] + 2* top[size-1] + top[size-1];

  leftF[0] = left[0] + 2* left[0] + 2*left[0] + 2* left[1] +left[2];
  leftF[1] = left[0] + 2* left[0] + 2*left[1] + 2* left[2] +left[3];
  for (j=2;j<size-2;j++){
    leftF[j] = left[j-2] + 2*left[j-1] + 2*left[j] + 2*left[j+1] + left[j+2];
  }
  leftF[size-2] = left[size-4]+ 2*left[size-3] + 2*left[size-2] + 2* left[size-1] + left[size-1];
  leftF[size-1] = left[size-3]+ 2*left[size-2] + 2*left[size-1] + 2* left[size-1] + left[size-1];


  top_leftF = left[1] + 2*left[0] + 2*top_left + 2*top[0]+top[1];

  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      pblock[i*pstride+j] = saturate((leftF[i] + topF[j] - top_leftF + 4) / 8, bitdepth);
    }
  }
}

void TEMPLATE(get_upleft_pred)(SAMPLE* left, SAMPLE* top, SAMPLE top_left, int size, SAMPLE *pblock, int pstride) {
  int i,j,diag;

  SAMPLE topF[MAX_TR_SIZE];
  SAMPLE leftF[MAX_TR_SIZE];
  SAMPLE top_leftF;

  filter_121_all(left,leftF,top,topF,size,top_left,&top_leftF);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = i-j;
      if (diag > 0){
        pblock[i*pstride+j] = leftF[diag-1];
        if (diag-1 < 0) printf("error\n");
      }
      else if (diag==0)
        pblock[i*pstride+j] = top_leftF;
      else{
        pblock[i*pstride+j] = topF[-diag-1];
        if (-diag-1 < 0) printf("error\n");
      }
    }
  }
}

void TEMPLATE(get_upright_pred)(SAMPLE *top, int size, SAMPLE *pblock, int pstride) {
  int i,j,diag;

  //int upright_available;
  SAMPLE topF[2*MAX_TR_SIZE];

  filter_121(top,topF,2*size);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = i+j;
      pblock[i*pstride+j] = topF[diag+1];
    }
  }
}

void TEMPLATE(get_upupright_pred)(SAMPLE *top, int size, SAMPLE *pblock, int pstride) {
  int i,j,diag;

  SAMPLE topF[2*MAX_TR_SIZE];

  filter_121(top,topF,2*size);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = i+2*j;
      if (diag&1){
        pblock[i*pstride+j] = topF[(diag+1)/2];
      }
      else{
        pblock[i*pstride+j] = (topF[diag/2] + topF[diag/2 + 1])>>1;
      }
    }
  }
}

void TEMPLATE(get_upupleft_pred)(SAMPLE *left, SAMPLE * top, SAMPLE top_left, int size, SAMPLE *pblock, int pstride) {
  int i,j,diag;

  SAMPLE topF[MAX_TR_SIZE];
  SAMPLE leftF[MAX_TR_SIZE];
  SAMPLE top_leftF;

  filter_121_all(left,leftF,top,topF,size,top_left,&top_leftF);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = i-2*j;
      if (diag > 1){
        pblock[i*pstride+j] = leftF[diag-2];
      }
      else if (diag == 1)
        pblock[i*pstride+j] = top_leftF;
      else if (diag == 0)
        pblock[i*pstride+j] = (top_leftF + topF[0])>>1;
      else{
        assert((-diag)/2 < size);
        if (diag&1)
          pblock[i*pstride+j] = topF[(-diag)/2];
        else
          pblock[i*pstride+j] = (topF[(-diag)/2] + topF[((-diag)/2) - 1])>>1;
      }
    }
  }
}

void TEMPLATE(get_upleftleft_pred)(SAMPLE* left, SAMPLE* top, SAMPLE top_left, int size, SAMPLE *pblock, int pstride) {
  int i,j,diag;

  SAMPLE topF[MAX_TR_SIZE];
  SAMPLE leftF[MAX_TR_SIZE];
  SAMPLE top_leftF;

  filter_121_all(left,leftF,top,topF,size,top_left,&top_leftF);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = 2*i-j;
      if (diag < -1){
        pblock[i*pstride+j] = topF[-diag-2];
      }
      else if (diag == -1)
        pblock[i*pstride+j] = top_leftF;
      else if (diag == 0)
        pblock[i*pstride+j] = (top_leftF + leftF[0])>>1;
      else{
        assert(diag/2 < size);
        if (diag&1)
          pblock[i*pstride+j] = leftF[diag/2];
        else
          pblock[i*pstride+j] = (leftF[diag/2] + leftF[diag/2 - 1])>>1;
      }
    }
  }
}

void TEMPLATE(get_downleftleft_pred)(SAMPLE *left, int size, SAMPLE *pblock, int pstride) {
  int i,j,diag;

  SAMPLE leftF[2*MAX_TR_SIZE];

  filter_121(left,leftF,2*size);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = 2*i+j;
      if (diag&1) {
        pblock[i*pstride+j] = leftF[(diag+1)/2];
      } else {
        pblock[i*pstride+j] = (leftF[diag/2] + leftF[diag/2 + 1])>>1;
      }
    }
  }
}

void TEMPLATE(get_intra_prediction)(SAMPLE* left, SAMPLE* top, SAMPLE top_left, int ypos,int xpos,
                                    int size, SAMPLE *pblock, int pstride, intra_mode_t intra_mode, int bitdepth)
{
  if (intra_mode == MODE_DC)
    TEMPLATE(get_dc_pred)(xpos!=0 ? left:top, ypos!=0 ? top:left, size, pblock, pstride, bitdepth);
  else if (intra_mode == MODE_HOR)
    TEMPLATE(get_hor_pred)(left, size, pblock, pstride);
  else if (intra_mode == MODE_VER)
    TEMPLATE(get_ver_pred)(top, size, pblock, pstride);
  else if (intra_mode == MODE_PLANAR)
    TEMPLATE(get_planar_pred)(left, top, top_left, size, pblock, pstride, bitdepth);
  else if (intra_mode == MODE_UPLEFT)
    TEMPLATE(get_upleft_pred)(left, top, top_left, size, pblock, pstride);
  else if (intra_mode == MODE_UPRIGHT)
    TEMPLATE(get_upright_pred)(top, size, pblock, pstride);
  else if (intra_mode == MODE_UPUPRIGHT)
    TEMPLATE(get_upupright_pred)(top, size, pblock, pstride);
  else if (intra_mode == MODE_UPUPLEFT)
    TEMPLATE(get_upupleft_pred)(left, top, top_left, size, pblock, pstride);
  else if (intra_mode == MODE_UPLEFTLEFT)
    TEMPLATE(get_upleftleft_pred)(left, top, top_left, size, pblock, pstride);
  else if (intra_mode == MODE_DOWNLEFTLEFT)
    TEMPLATE(get_downleftleft_pred)(left, size, pblock, pstride);
  else
    TEMPLATE(get_dc_pred)(xpos!=0 ? left:top, ypos!=0 ? top:left, size, pblock, pstride, bitdepth);
}
