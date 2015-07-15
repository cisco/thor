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


void get_dc_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,uint8_t *pblock){
  int i,j,dc=128,sum;

  if (ypos>0 && xpos>0){
    sum = 0;
    for (j=0;j<size;j++) sum += rec[(ypos-1)*stride+xpos+j];
    for (i=0;i<size;i++) sum += rec[(ypos+i)*stride+xpos-1];
    dc = (sum + size)/(2*size);
  }
  else if(ypos>0 && xpos==0){
    sum = 0;
    for (j=0;j<size;j++) sum += rec[(ypos-1)*stride+xpos+j];
    dc = (sum + size/2)/size;
  }
  else if(ypos==0 && xpos>0){
    sum = 0;
    for (i=0;i<size;i++) sum += rec[(ypos+i)*stride+xpos-1];
    dc = (sum + size/2)/size;
  }
  else if(ypos==0 && xpos==0){
    dc = 128;
  }
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      pblock[i*size+j] = dc;
    }
  }
}

void get_hor_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,uint8_t *pblock){
  int i,j,val;


#if FILTER_HOR_AND_VER
  if (xpos>0){
    uint16_t hor[MAX_TR_SIZE];
    uint8_t horF[MAX_TR_SIZE];
    for (i=0;i<size;i++){
      hor[i] = rec[(ypos+i)*stride+xpos-1];
    }
    horF[0] = (uint8_t)((hor[0] + 14*hor[0] + hor[1] + 8)>>4);
    for (j=1;j<size-1;j++){
      horF[j] = (uint8_t)((hor[j-1] + 14*hor[j] + hor[j+1] + 8)>>4);
    }
    horF[size-1] = (uint8_t)((hor[size-2] + 14*hor[size-1] + hor[size-1] + 8)>>4);
    for (i=0;i<size;i++){
      val = horF[i];
      for (j=0;j<size;j++){
        pblock[i*size+j] = val;
      }
    }
  }
  else{
    for (i=0;i<size;i++){
      for (j=0;j<size;j++){
        pblock[i*size+j] = 128;
      }
    }
  }
#else
  if (xpos>0){
    for (i=0;i<size;i++){
      val = rec[(ypos+i)*stride+xpos-1];
      for (j=0;j<size;j++){
        pblock[i*size+j] = val;
      }
    }
  }
  else{
    for (i=0;i<size;i++){
      for (j=0;j<size;j++){
        pblock[i*size+j] = 128;
      }
    }
  }
#endif
}

void get_ver_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,uint8_t *pblock){
  int i,j;

#if FILTER_HOR_AND_VER
  if (ypos>0){
    uint16_t ver[MAX_TR_SIZE];
    uint8_t verF[MAX_TR_SIZE];
    for (j=0;j<size;j++){
      ver[j] = rec[(ypos-1)*stride+xpos+j];
    }
    verF[0] = (uint8_t)((ver[0] + 14*ver[0] + ver[1] + 8)>>4);
    for (j=1;j<size-1;j++){
      verF[j] = (uint8_t)((ver[j-1] + 14*ver[j] + ver[j+1] + 8)>>4);
    }
    verF[size-1] = (uint8_t)((ver[size-2] + 14*ver[size-1] + ver[size-1] + 8)>>4);
    for (i=0;i<size;i++){
      for (j=0;j<size;j++){
        pblock[i*size+j] = verF[j];
      }
    }
  }
  else{
    for (i=0;i<size;i++){
      for (j=0;j<size;j++){
        pblock[i*size+j] = 128;
      }
    }
  }
#else

  if (ypos>0){
    for (i=0;i<size;i++){
      for (j=0;j<size;j++){
        pblock[i*size+j] = rec[(ypos-1)*stride+xpos+j];
      }
    }
  }
  else{
    for (i=0;i<size;i++){
      for (j=0;j<size;j++){
        pblock[i*size+j] = 128;
      }
    }
  }

#endif

}

void get_planar_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,uint8_t *pblock){
  int i,j;

  uint8_t ver[MAX_TR_SIZE];
  uint8_t hor[MAX_TR_SIZE];
  uint8_t up_left;

  if (ypos>0){
    for (j=0;j<size;j++){
      ver[j] = rec[(ypos-1)*stride+xpos+j];
    }
  }
  else{
    for (j=0;j<size;j++){
      ver[j] = 128;
    }
  }

  if (xpos>0){
    for (i=0;i<size;i++){
      hor[i] = rec[(ypos+i)*stride+xpos-1];
    }
  }
  else{
    for (i=0;i<size;i++){
      hor[i] = 128;
    }
  }

  if (xpos>0 && ypos>0){
    up_left = rec[(ypos-1)*stride+xpos-1];
  }
  else{
    up_left = 128;
  }

  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      pblock[i*size+j] = clip255(hor[i] + ver[j] - up_left);
    }
  }
}

void get_upleft_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,uint8_t *pblock){
  int i,j,diag;

  uint16_t ver[MAX_TR_SIZE];
  uint16_t hor[MAX_TR_SIZE];
  uint16_t up_left;

  uint8_t verF[MAX_TR_SIZE];
  uint8_t horF[MAX_TR_SIZE];
  uint8_t up_leftF;

  /* Define unfiltered arrays */
  if (ypos>0){
    for (j=0;j<size;j++){
      ver[j] = rec[(ypos-1)*stride+xpos+j];
    }
  }
  else{
    for (j=0;j<size;j++){
      ver[j] = 128;
    }
  }

  if (xpos>0){
    for (i=0;i<size;i++){
      hor[i] = rec[(ypos+i)*stride+xpos-1];
    }
  }
  else{
    for (i=0;i<size;i++){
      hor[i] = 128;
    }
  }

  if (xpos>0 && ypos>0){
    up_left = rec[(ypos-1)*stride+xpos-1];
  }
  else{
    up_left = 128;
  }

  /* Calculate filtered 1D arrays */
  verF[0] = (uint8_t)((ver[0] + 2*ver[0] + ver[1] + 2)>>2);
  for (j=1;j<size-1;j++){
    verF[j] = (uint8_t)((ver[j-1] + 2*ver[j] + ver[j+1] + 2)>>2);
  }
  verF[size-1] = (uint8_t)((ver[size-2] + 2*ver[size-1] + ver[size-1] + 2)>>2);

  horF[0] = (uint8_t)((hor[0] + 2*hor[0] + hor[1] + 2)>>2);
  for (j=1;j<size-1;j++){
    horF[j] = (uint8_t)((hor[j-1] + 2*hor[j] + hor[j+1] + 2)>>2);
  }
  horF[size-1] = (uint8_t)((hor[size-2] + 2*hor[size-1] + hor[size-1] + 2)>>2);

  up_leftF = (uint8_t)((hor[0] + 2*up_left + ver[0] + 2)>>2);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = i-j;
      if (diag > 0){
        pblock[i*size+j] = horF[diag-1];
        if (diag-1 < 0) printf("error\n");
      }
      else if (diag==0)
        pblock[i*size+j] = up_leftF;
      else{
        pblock[i*size+j] = verF[-diag-1];
        if (-diag-1 < 0) printf("error\n");
      }
    }
  }
}

void get_upright_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,int width,uint8_t *pblock,int upright_available){
  int i,j,diag;

  //int upright_available;
  uint16_t ver[2*MAX_TR_SIZE];
  uint8_t verF[2*MAX_TR_SIZE];

  //upright_available = get_upright_available(ypos,xpos,size,width);

  /* Generate unfiltered 1D array */
  if (ypos>0){
    for (j=0;j<size;j++){
      ver[j] = rec[(ypos-1)*stride+xpos+j];
    }
    if (upright_available){
      for (j=size;j<2*size;j++){
        ver[j] = rec[(ypos-1)*stride+xpos+j];
      }
    }
    else{
      for (j=size;j<2*size;j++){
        ver[j] = rec[(ypos-1)*stride+xpos+size-1];
      }
    }
  }
  else{
    for (j=0;j<2*size;j++){
      ver[j] = 128;
    }
  }

  /* Generate filtered 1D array */
  for (j=1;j<2*size-1;j++){
    verF[j] = (uint8_t)((ver[j-1] + 2*ver[j] + ver[j+1] + 2)>>2);
  }
  verF[2*size-1] = (uint8_t)((ver[2*size-2] + 2*ver[2*size-1] + ver[2*size-1] + 2)>>2);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = i+j;
      pblock[i*size+j] = verF[diag+1];
    }
  }
}

void get_upupright_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,int width,uint8_t *pblock,int upright_available){
  int i,j,diag;

  //int upright_available;
  uint16_t ver[2*MAX_TR_SIZE];
  uint8_t verF[2*MAX_TR_SIZE];

  //upright_available = get_upright_available(ypos,xpos,size,width);

  /* Generate unfiltered 1D array */
  if (ypos>0){
    for (j=0;j<size;j++){
      ver[j] = rec[(ypos-1)*stride+xpos+j];
    }
    if (upright_available){
      for (j=size;j<2*size;j++){
        ver[j] = rec[(ypos-1)*stride+xpos+j];
      }
    }
    else{
      for (j=size;j<2*size;j++){
        ver[j] = rec[(ypos-1)*stride+xpos+size-1];
      }
    }
  }
  else{
    for (j=0;j<2*size;j++){
      ver[j] = 128;
    }
  }

  /* Generate filtered 1D array */
  verF[0] = (uint8_t)((ver[0] + 2*ver[0] + ver[1] + 2)>>2);
  for (j=1;j<2*size-1;j++){
    verF[j] = (uint8_t)((ver[j-1] + 2*ver[j] + ver[j+1] + 2)>>2);
  }
  verF[2*size-1] = (uint8_t)((ver[2*size-2] + 2*ver[2*size-1] + ver[2*size-1] + 2)>>2);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = i+2*j;
      if (diag&1){
        pblock[i*size+j] = verF[(diag+1)/2];
      }
      else{
        pblock[i*size+j] = (verF[diag/2] + verF[diag/2 + 1])>>1;
      }
    }
  }
}

void get_upupleft_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,uint8_t *pblock){
  int i,j,diag;

  uint16_t ver[MAX_TR_SIZE];
  uint16_t hor[MAX_TR_SIZE];
  uint16_t up_left;

  uint8_t verF[MAX_TR_SIZE];
  uint8_t horF[MAX_TR_SIZE];
  uint8_t up_leftF;

  /* Define unfiltered arrays */
  if (ypos>0){
    for (j=0;j<size;j++){
      ver[j] = rec[(ypos-1)*stride+xpos+j];
    }
  }
  else{
    for (j=0;j<size;j++){
      ver[j] = 128;
    }
  }

  if (xpos>0){
    for (i=0;i<size;i++){
      hor[i] = rec[(ypos+i)*stride+xpos-1];
    }
  }
  else{
    for (i=0;i<size;i++){
      hor[i] = 128;
    }
  }

  if (xpos>0 && ypos>0){
    up_left = rec[(ypos-1)*stride+xpos-1];
  }
  else{
    up_left = 128;
  }

  /* Calculate filtered 1D arrays */
  verF[0] = (uint8_t)((ver[0] + 2*ver[0] + ver[1] + 2)>>2);
  for (j=1;j<size-1;j++){
    verF[j] = (uint8_t)((ver[j-1] + 2*ver[j] + ver[j+1] + 2)>>2);
  }
  verF[size-1] = (uint8_t)((ver[size-2] + 2*ver[size-1] + ver[size-1] + 2)>>2);

  horF[0] = (uint8_t)((hor[0] + 2*hor[0] + hor[1] + 2)>>2);
  for (j=1;j<size-1;j++){
    horF[j] = (uint8_t)((hor[j-1] + 2*hor[j] + hor[j+1] + 2)>>2);
  }
  horF[size-1] = (uint8_t)((hor[size-2] + 2*hor[size-1] + hor[size-1] + 2)>>2);

  up_leftF = (uint8_t)((hor[0] + 2*up_left + ver[0] + 2)>>2);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = i-2*j;
      if (diag > 1){
        pblock[i*size+j] = horF[diag-2];
      }
      else if (diag == 1)
        pblock[i*size+j] = up_leftF;
      else if (diag == 0)
        pblock[i*size+j] = (up_leftF + verF[0])>>1;
      else{
        if (diag&1)
          pblock[i*size+j] = verF[(-diag)/2];
        else
          pblock[i*size+j] = (verF[(-diag)/2] + verF[((-diag)/2) - 1])>>1;
      }
    }
  }
}

void get_upleftleft_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,uint8_t *pblock){
  int i,j,diag;

  uint16_t ver[MAX_TR_SIZE];
  uint16_t hor[MAX_TR_SIZE];
  uint16_t up_left;

  uint8_t verF[MAX_TR_SIZE];
  uint8_t horF[MAX_TR_SIZE];
  uint8_t up_leftF;

  /* Define unfiltered arrays */
  if (ypos>0){
    for (j=0;j<size;j++){
      ver[j] = rec[(ypos-1)*stride+xpos+j];
    }
  }
  else{
    for (j=0;j<size;j++){
      ver[j] = 128;
    }
  }

  if (xpos>0){
    for (i=0;i<size;i++){
      hor[i] = rec[(ypos+i)*stride+xpos-1];
    }
  }
  else{
    for (i=0;i<size;i++){
      hor[i] = 128;
    }
  }

  if (xpos>0 && ypos>0){
    up_left = rec[(ypos-1)*stride+xpos-1];
  }
  else{
    up_left = 128;
  }

  /* Calculate filtered 1D arrays */
  verF[0] = (uint8_t)((ver[0] + 2*ver[0] + ver[1] + 2)>>2);
  for (j=1;j<size-1;j++){
    verF[j] = (uint8_t)((ver[j-1] + 2*ver[j] + ver[j+1] + 2)>>2);
  }
  verF[size-1] = (uint8_t)((ver[size-2] + 2*ver[size-1] + ver[size-1] + 2)>>2);

  horF[0] = (uint8_t)((hor[0] + 2*hor[0] + hor[1] + 2)>>2);
  for (j=1;j<size-1;j++){
    horF[j] = (uint8_t)((hor[j-1] + 2*hor[j] + hor[j+1] + 2)>>2);
  }
  horF[size-1] = (uint8_t)((hor[size-2] + 2*hor[size-1] + hor[size-1] + 2)>>2);

  up_leftF = (uint8_t)((hor[0] + 2*up_left + ver[0] + 2)>>2);

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = 2*i-j;
      if (diag < -1){
        pblock[i*size+j] = verF[-diag-2];
      }
      else if (diag == -1)
        pblock[i*size+j] = up_leftF;
      else if (diag == 0)
        pblock[i*size+j] = (up_leftF + horF[0])>>1;
      else{
        if (diag&1)
          pblock[i*size+j] = horF[diag/2];
        else
          pblock[i*size+j] = (horF[diag/2] + horF[diag/2 - 1])>>1;
      }
    }
  }
}

void get_downleftleft_pred(uint8_t *rec,int ypos,int xpos,int stride,int size,uint8_t *pblock){
  int i,j,diag;

  uint16_t hor[MAX_TR_SIZE];
  uint8_t horF[2*MAX_TR_SIZE];
  if (xpos>0){
    for (i=0;i<size;i++){
      hor[i] = rec[(ypos+i)*stride+xpos-1];
    }
  }
  else{
    for (i=0;i<size;i++){
      hor[i] = 128;
    }
  }
  horF[0] = (uint8_t)((hor[0] + 2*hor[0] + hor[1] + 2)>>2);
  for (j=1;j<size-1;j++){
    horF[j] = (uint8_t)((hor[j-1] + 2*hor[j] + hor[j+1] + 2)>>2);
  }
  horF[size-1] = (uint8_t)((hor[size-2] + 2*hor[size-1] + hor[size-1] + 2)>>2);
  for (j=size;j<2*size-1;j++){
    horF[j] = horF[size-1];
  }

  /* Perform prediction */
  for (i=0;i<size;i++){
    for (j=0;j<size;j++){
      diag = 2*i+j;
      if (diag&1)
        pblock[i*size+j] = horF[(diag+1)/2];
      else
        pblock[i*size+j] = (horF[diag/2] + horF[diag/2 + 1])>>1;
    }
  }
}

void get_intra_prediction(uint8_t *rec,int ypos,int xpos,int stride,int size,int width,uint8_t *pblock,intra_mode_t intra_mode,int upright_available)
{
  if (intra_mode == MODE_DC)
    get_dc_pred(rec,ypos,xpos,stride,size,pblock);
  else if (intra_mode == MODE_HOR)
    get_hor_pred(rec,ypos,xpos,stride,size,pblock);
  else if (intra_mode == MODE_VER)
    get_ver_pred(rec,ypos,xpos,stride,size,pblock);
  else if (intra_mode == MODE_PLANAR)
    get_planar_pred(rec,ypos,xpos,stride,size,pblock);
  else if (intra_mode == MODE_UPLEFT)
    get_upleft_pred(rec,ypos,xpos,stride,size,pblock);
  else if (intra_mode == MODE_UPRIGHT)
    get_upright_pred(rec,ypos,xpos,stride,size,width,pblock,upright_available);
  else if (intra_mode == MODE_UPUPRIGHT)
    get_upupright_pred(rec,ypos,xpos,stride,size,width,pblock,upright_available);
  else if (intra_mode == MODE_UPUPLEFT)
    get_upupleft_pred(rec,ypos,xpos,stride,size,pblock);
  else if (intra_mode == MODE_UPLEFTLEFT)
    get_upleftleft_pred(rec,ypos,xpos,stride,size,pblock);
  else if (intra_mode == MODE_DOWNLEFTLEFT)
    get_downleftleft_pred(rec,ypos,xpos,stride,size,pblock);
}
