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

int zigzag16[16] = {
    0, 1, 5, 6, 
    2, 4, 7, 12, 
    3, 8, 11, 13, 
    9, 10, 14, 15
};

int zigzag64[64] = {
     0,  1,  5,  6, 14, 15, 27, 28,
     2,  4,  7, 13, 16, 26, 29, 42,
     3,  8, 12, 17, 25, 30, 41, 43,
     9, 11, 18, 24, 31, 40, 44, 53,
    10, 19, 23, 32, 39, 45, 52, 54,
    20, 22, 33, 38, 46, 51, 55, 60,
    21, 34, 37, 47, 50, 56, 59, 61,
    35, 36, 48, 49, 57, 58, 62, 63
};

int zigzag256[256] = {
    0,  1,  5,  6, 14, 15, 27, 28, 44, 45, 65, 66, 90, 91,119,120,
    2,  4,  7, 13, 16, 26, 29, 43, 46, 64, 67, 89, 92,118,121,150,
    3,  8, 12, 17, 25, 30, 42, 47, 63, 68, 88, 93,117,122,149,151,
    9, 11, 18, 24, 31, 41, 48, 62, 69, 87, 94,116,123,148,152,177,
   10, 19, 23, 32, 40, 49, 61, 70, 86, 95,115,124,147,153,176,178,
   20, 22, 33, 39, 50, 60, 71, 85, 96,114,125,146,154,175,179,200,
   21, 34, 38, 51, 59, 72, 84, 97,113,126,145,155,174,180,199,201,
   35, 37, 52, 58, 73, 83, 98,112,127,144,156,173,181,198,202,219,
   36, 53, 57, 74, 82, 99,111,128,143,157,172,182,197,203,218,220,
   54, 56, 75, 81,100,110,129,142,158,171,183,196,204,217,221,234,
   55, 76, 80,101,109,130,141,159,170,184,195,205,216,222,233,235,
   77, 79,102,108,131,140,160,169,185,194,206,215,223,232,236,245,
   78,103,107,132,139,161,168,186,193,207,214,224,231,237,244,246,
  104,106,133,138,162,167,187,192,208,213,225,230,238,243,247,252,
  105,134,137,163,166,188,191,209,212,226,229,239,242,248,251,253,
  135,136,164,165,189,190,210,211,227,228,240,241,249,250,254,255
};




int chroma_qp[52] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 29,
        30, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38,
        39, 40, 41, 42, 43, 44, 45
};

int super_table[8][20] = {
  {-1,-1, -1,-1,-1,-1,-1,-1,-1,-1,  1, 0, 5, 2, 6, 3, 7, 4, 8,-1},
  {-1, 0, -1,-1,-1,-1,-1,-1,-1,-1,  2, 1, 6, 3, 7, 5, 8, 4, 9,-1},
  {-1, 0, -1,-1,-1,-1,-1,-1,-1,-1,  2, 1, 6, 3, 7, 5, 8, 4, 9,-1},
  {-1, 0, -1,-1,-1,-1,-1,-1,-1,-1,  2, 1, 6, 3, 7, 5, 8, 4, 9,-1},

  { 0,-1,  2, 1,12, 7,13, 5,16,11,  3, 4,14, 8, 9, 6,15,10,17,18},
  { 0, 1,  3, 2,10, 7,11, 6,16, 9,  5, 4,15,13,14, 8,17,12,18,19},
  { 0, 1,  3, 2,10, 4,12, 5,14, 6,  8, 7,15,13,16,11,17, 9,18,19},
  { 0, 1,  3, 2, 7, 4, 8, 5, 9, 6, 11,10,15,14,16,13,17,12,18,19}
};

const uint16_t gquant_table[6] = {26214,23302,20560,18396,16384,14564};
const uint16_t gdequant_table[6] = {40,45,51,57,64,72};

int get_left_available(int ypos, int xpos, int size, int width){
  int left_available = xpos > 0;
  return left_available;
}

int get_up_available(int ypos, int xpos, int size, int width){
  int up_available = ypos > 0;
  return up_available;
}

int get_upright_available(int ypos, int xpos, int size, int width){

  int upright_available = (ypos > 0) && (xpos + size < width);
  if (size==32 && (ypos%64)==32) upright_available = 0;
  if (size==16 && ((ypos%32)==16 || ((ypos%64)==32 && (xpos%32)==16))) upright_available = 0;
  if (size== 8 && ((ypos%16)==8 || ((ypos%32)==16 && (xpos%16)==8) || ((ypos%64)==32 && (xpos%32)==24))) upright_available = 0;

  return upright_available;
}

int get_downleft_available(int ypos, int xpos, int size, int height){

  int downleft_available = (xpos > 0) && (ypos + size < height);
  if (size==64) downleft_available = 0;
  if (size==32 && (ypos%64)==32) downleft_available = 0;
  if (size==16 && ((ypos%64)==48 || ((ypos%64)==16 && (xpos%32)==16))) downleft_available = 0;
  if (size== 8 && ((ypos%64)==56 || ((ypos%16)==8 && (xpos%16)==8) || ((ypos%64)==24 && (xpos%32)==16))) downleft_available = 0;

  return downleft_available;
}


void dequantize (int16_t *coeff, int16_t *rcoeff, int qp, int size)
{
  int tr_log2size = log2i(size);
  const int lshift = qp / 6;
  const int rshift = tr_log2size - 1;
  const int scale = gdequant_table[qp % 6];
  const int add = 1<<(rshift-1);

  for (int i = 0; i < size ; i++){
    for (int j = 0; j < size; j++){
      int c = coeff[i*size+j];
      rcoeff[i*size+j] = ((c * scale << lshift) + add) >> rshift;
    }
  }
}

void reconstruct_block(int16_t *block, uint8_t *pblock, uint8_t *rec, int size, int stride)
{ 
  int i,j;
  for(i=0;i<size;i++){    
    for (j=0;j<size;j++){
      rec[i*stride+j] = (uint8_t)clip255(block[i*size+j] + (int16_t)pblock[i*size+j]);      
    }
  }
}

void find_block_contexts(int ypos, int xpos, int height, int width, int size, deblock_data_t *deblock_data, block_context_t *block_context, int enable){

  if (ypos >= MIN_BLOCK_SIZE && xpos >= MIN_BLOCK_SIZE && ypos+size < height && xpos+size < width && enable){
    int by = ypos/MIN_PB_SIZE;
    int bx = xpos/MIN_PB_SIZE;
    int bs = width/MIN_PB_SIZE;
    int bindex = by*bs+bx;
    block_context->split = (deblock_data[bindex-bs].size < size) + (deblock_data[bindex-1].size < size);
    int cbp1;
    cbp1 = (deblock_data[bindex-bs].cbp.y > 0) + (deblock_data[bindex-1].cbp.y > 0);
    block_context->cbp = cbp1;
#if NEW_BLOCK_STRUCTURE
    block_context->index = -1;
#else
    int cbp2 = (deblock_data[bindex-bs].cbp.y > 0 || deblock_data[bindex-bs].cbp.u > 0 || deblock_data[bindex-bs].cbp.v > 0) +
           (deblock_data[bindex-1].cbp.y > 0 || deblock_data[bindex-1].cbp.u > 0 || deblock_data[bindex-1].cbp.v > 0);
    block_context->index = 3*block_context->split + cbp2;
#endif
  }
  else{
    block_context->split = -1;
    block_context->cbp = -1;
    block_context->index = -1;
  }
}

void clpf_block(uint8_t *rec,int x0, int x1, int y0, int y1,int stride){

  int y,x,A,B,C,D,E,F,delta,sum,sign;
  uint8_t tmp_block[MAX_BLOCK_SIZE*MAX_BLOCK_SIZE];

  for (y=y0;y<y1;y++){
    for (x=x0;x<x1;x++){
      A = rec[(y-1)*stride + x+0];
      B = rec[(y+0)*stride + x-1];
      C = rec[(y+0)*stride + x+0];
      D = rec[(y+0)*stride + x+1];
      E = rec[(y+1)*stride + x+0];
      sum = A+B+D+E-4*C;
      sign = sum < 0 ? -1 : 1;
      delta = sign*min(1,((abs(sum)+2)>>2));
      F = clip255(C + delta);
      tmp_block[(y-y0)*MAX_BLOCK_SIZE + x-x0] = F;
    }
  }
  for (y=y0;y<y1;y++){
    for (x=x0;x<x1;x++){
      rec[y*stride + x] = tmp_block[(y-y0)*MAX_BLOCK_SIZE + x-x0];
    }
  }
}
