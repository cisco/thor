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
#include <memory.h>
#include "types.h"
#include "global.h"
#include "assert.h"
#include "common_block.h"
#include "common_kernels.h"

static const SAMPLE beta_table[52] = {
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
     16, 17, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64
};

static const SAMPLE tc_table[56] =
{
  0,0,1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,36,40,44,48,52,56,60,64,68,72,80,88,96,104,112,128,144,152,160,168,176,184,192,200,208,216,224,232
};

void TEMPLATE(deblock_frame_y)(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp, int bitdepth)
{
  int i,j,k,l,d;
  int stride = rec->stride_y;
#if MODIFIED_DEBLOCK_TEST
  int d_15, d_26;
  int p11, p01, q01, q11;
  int p12, p02, q02, q12;
  int p15, p05, q05, q15;
  int p16, p06, q06, q16;
#else
#if NEW_DEBLOCK_TEST
  int p12,p02,q02,q12;
  int p15,p05,q05,q15;
#else
  int p22,p12,p02,q02,q12,q22;
  int p25,p15,p05,q05,q15,q25;
#endif
#endif
  int p2,p1,p0,q0,q1,q2;
  SAMPLE *recY = rec->y;
  uint8_t do_filter;
  SAMPLE beta = beta_table[qp] << (bitdepth-8);
  SAMPLE tc = bitdepth > 12 ? tc_table[qp] << (bitdepth-12) : tc_table[qp] >> (12-bitdepth); //TODO: increment with 4 for intra

  int p_index,q_index;
  mv_t p_mv0,q_mv0;
  mv_t p_mv1,q_mv1;
  block_mode_t p_mode,q_mode;
  int p_cbp,q_cbp;
  int q_size;
  int mv,mode,cbp,interior;
  int delta;

  /* Vertical filtering */
  for (i=0;i<height;i+=MIN_BLOCK_SIZE){
    for (j=MIN_BLOCK_SIZE;j<width;j+=MIN_BLOCK_SIZE){

#if MODIFIED_DEBLOCK_TEST
      p11 = recY[(i + 1)*stride + j - 2];
      p01 = recY[(i + 1)*stride + j - 1];
      q01 = recY[(i + 1)*stride + j + 0];
      q11 = recY[(i + 1)*stride + j + 1];

      p15 = recY[(i + 5)*stride + j - 2];
      p05 = recY[(i + 5)*stride + j - 1];
      q05 = recY[(i + 5)*stride + j + 0];
      q15 = recY[(i + 5)*stride + j + 1];
      d_15 = abs(p11 - p01) + abs(q11 - q01) + abs(p15 - p05) + abs(q15 - q05);

      p12 = recY[(i + 2)*stride + j - 2];
      p02 = recY[(i + 2)*stride + j - 1];
      q02 = recY[(i + 2)*stride + j + 0];
      q12 = recY[(i + 2)*stride + j + 1];

      p16 = recY[(i + 6)*stride + j - 2];
      p06 = recY[(i + 6)*stride + j - 1];
      q06 = recY[(i + 6)*stride + j + 0];
      q16 = recY[(i + 6)*stride + j + 1];
      d_26 = abs(p12 - p02) + abs(q12 - q02) + abs(p16 - p06) + abs(q16 - q06);
#else
#if NEW_DEBLOCK_TEST
      p12 = recY[(i+2)*stride + j - 2];
      p02 = recY[(i+2)*stride + j - 1];
      q02 = recY[(i+2)*stride + j + 0];
      q12 = recY[(i+2)*stride + j + 1];

      p15 = recY[(i+5)*stride + j - 2];
      p05 = recY[(i+5)*stride + j - 1];
      q05 = recY[(i+5)*stride + j + 0];
      q15 = recY[(i+5)*stride + j + 1];
      d = abs(p12-p02) + abs(q12-q02) + abs(p15-p05) + abs(q15-q05);
#else
      p22 = recY[(i+2)*stride + j - 3];
      p12 = recY[(i+2)*stride + j - 2];
      p02 = recY[(i+2)*stride + j - 1];
      q02 = recY[(i+2)*stride + j + 0];
      q12 = recY[(i+2)*stride + j + 1];
      q22 = recY[(i+2)*stride + j + 2];

      p25 = recY[(i+5)*stride + j - 3];
      p15 = recY[(i+5)*stride + j - 2];
      p05 = recY[(i+5)*stride + j - 1];
      q05 = recY[(i+5)*stride + j + 0];
      q15 = recY[(i+5)*stride + j + 1];
      q25 = recY[(i+5)*stride + j + 2];
      d = abs(p22-2*p12+p02) + abs(q22-2*q12+q02) + abs(p25-2*p15+p05) + abs(q25-2*q15+q05);
#endif
#endif
      int m;
      for (m=0;m<MIN_BLOCK_SIZE;m+=MIN_PB_SIZE){
        q_index = ((i+m)/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (j/MIN_PB_SIZE);
        p_index = q_index - 1;
        p_mv0 = deblock_data[p_index].inter_pred.mv0;
        q_mv0 = deblock_data[q_index].inter_pred.mv0;
        p_mv1 = deblock_data[p_index].inter_pred.mv1;
        q_mv1 = deblock_data[q_index].inter_pred.mv1;
        p_mode = deblock_data[p_index].mode;
        q_mode = deblock_data[q_index].mode;
        p_cbp = deblock_data[p_index].cbp.y;
        q_cbp = deblock_data[q_index].cbp.y;
        q_size = deblock_data[q_index].size;
        if ((deblock_data[q_index].tb_split || deblock_data[q_index].pb_part == PART_VER || deblock_data[q_index].pb_part == PART_QUAD) && q_size > MIN_BLOCK_SIZE) q_size = q_size/2;

#if NEW_MV_TEST
        mv = abs(p_mv0.y) >= 4 || abs(q_mv0.y) >= 4 || abs(p_mv0.x) >= 4 || abs(q_mv0.x) >= 4; //TODO: Investigate >=3 instead
        mv = mv || abs(p_mv1.y) >= 4 || abs(q_mv1.y) >= 4 || abs(p_mv1.x) >= 4 || abs(q_mv1.x) >= 4;
#else
        mv = abs(p_mv0.y - q_mv0.y) >= 2 || abs(p_mv0.x - q_mv0.x) >= 2;
        mv = mv || abs(p_mv1.y - q_mv1.y) >= 2 || abs(p_mv1.x - q_mv1.x) >= 2;
#endif
        cbp = p_cbp || q_cbp;
        mode = p_mode == MODE_INTRA || q_mode == MODE_INTRA;
        interior = j%q_size > 0 ? 1 : 0;
#if MODIFIED_DEBLOCK_TEST
        do_filter = !interior && (mv || cbp || mode); //TODO: This logic needs to support 4x4TUs
#else
        do_filter = (d < beta) && !interior && (mv || cbp || mode); //TODO: This logic needs to support 4x4TUs
#endif
        if (do_filter){
          for (k=m;k<m+MIN_PB_SIZE;k++){
#if MODIFIED_DEBLOCK_TEST
            d = k & 1 ? d_26 : d_15;
            if (d < beta) {
              p2 = (int)recY[(i + k)*stride + j - 3];
              p1 = (int)recY[(i + k)*stride + j - 2];
              p0 = (int)recY[(i + k)*stride + j - 1];
              q0 = (int)recY[(i + k)*stride + j + 0];
              q1 = (int)recY[(i + k)*stride + j + 1];
              q2 = (int)recY[(i + k)*stride + j + 2];
#if NEW_DEBLOCK_FILTER
              delta = (18 * (q0 - p0) - 6 * (q1 - p1) + 0 * (q2 - p2) + 16) >> 5;
#else
              delta = (13 * (q0 - p0) + 4 * (q1 - p1) - 5 * (q2 - p2) + 16) >> 5;
#endif
              delta = clip(delta, -tc, tc);

              recY[(i + k)*stride + j - 2] = (SAMPLE)saturate(p1 + delta / 2, bitdepth);
              recY[(i + k)*stride + j - 1] = (SAMPLE)saturate(p0 + delta, bitdepth);
              recY[(i + k)*stride + j + 0] = (SAMPLE)saturate(q0 - delta, bitdepth);
              recY[(i + k)*stride + j + 1] = (SAMPLE)saturate(q1 - delta / 2, bitdepth);
            }
#else
            p2 = (int)recY[(i+k)*stride + j - 3];
            p1 = (int)recY[(i+k)*stride + j - 2];
            p0 = (int)recY[(i+k)*stride + j - 1];
            q0 = (int)recY[(i+k)*stride + j + 0];
            q1 = (int)recY[(i+k)*stride + j + 1];
            q2 = (int)recY[(i+k)*stride + j + 2];
#if NEW_DEBLOCK_FILTER
            delta = (18*(q0-p0) - 6*(q1-p1) + 0*(q2-p2) + 16)>>5;
#else
            delta = (13*(q0-p0) + 4*(q1-p1) - 5*(q2-p2) + 16)>>5;
#endif
            delta = clip(delta,-tc,tc);

            recY[(i+k)*stride + j - 2] = (SAMPLE)saturate(p1 + delta/2, bitdepth);
            recY[(i+k)*stride + j - 1] = (SAMPLE)saturate(p0 + delta, bitdepth);
            recY[(i+k)*stride + j + 0] = (SAMPLE)saturate(q0 - delta, bitdepth);
            recY[(i+k)*stride + j + 1] = (SAMPLE)saturate(q1 - delta/2, bitdepth);
#endif
          }
        }
      }

    }
  }

  /* Horizontal filtering */
  for (i=MIN_BLOCK_SIZE;i<height;i+=MIN_BLOCK_SIZE){
    for (j=0;j<width;j+=MIN_BLOCK_SIZE){

#if MODIFIED_DEBLOCK_TEST
      p11 = recY[(i - 2)*stride + j + 1];
      p01 = recY[(i - 1)*stride + j + 1];
      q01 = recY[(i + 0)*stride + j + 1];
      q11 = recY[(i + 1)*stride + j + 1];

      p15 = recY[(i - 2)*stride + j + 5];
      p05 = recY[(i - 1)*stride + j + 5];
      q05 = recY[(i + 0)*stride + j + 5];
      q15 = recY[(i + 1)*stride + j + 5];
      d_15 = abs(p11 - p01) + abs(q11 - q01) + abs(p15 - p05) + abs(q15 - q05);

      p12 = recY[(i - 2)*stride + j + 2];
      p02 = recY[(i - 1)*stride + j + 2];
      q02 = recY[(i + 0)*stride + j + 2];
      q12 = recY[(i + 1)*stride + j + 2];

      p16 = recY[(i - 2)*stride + j + 6];
      p06 = recY[(i - 1)*stride + j + 6];
      q06 = recY[(i + 0)*stride + j + 6];
      q16 = recY[(i + 1)*stride + j + 6];
      d_26 = abs(p12 - p02) + abs(q12 - q02) + abs(p16 - p06) + abs(q16 - q06);
#else
#if NEW_DEBLOCK_TEST
      p12 = recY[(i-2)*stride + j + 2];
      p02 = recY[(i-1)*stride + j + 2];
      q02 = recY[(i+0)*stride + j + 2];
      q12 = recY[(i+1)*stride + j + 2];

      p15 = recY[(i-2)*stride + j + 5];
      p05 = recY[(i-1)*stride + j + 5];
      q05 = recY[(i+0)*stride + j + 5];
      q15 = recY[(i+1)*stride + j + 5];
      d = abs(p12-p02) + abs(q12-q02) + abs(p15-p05) + abs(q15-q05);
#else
      p22 = recY[(i-3)*stride + j + 2];
      p12 = recY[(i-2)*stride + j + 2];
      p02 = recY[(i-1)*stride + j + 2];
      q02 = recY[(i+0)*stride + j + 2];
      q12 = recY[(i+1)*stride + j + 2];
      q22 = recY[(i+2)*stride + j + 2];

      p25 = recY[(i-3)*stride + j + 5];
      p15 = recY[(i-2)*stride + j + 5];
      p05 = recY[(i-1)*stride + j + 5];
      q05 = recY[(i+0)*stride + j + 5];
      q15 = recY[(i+1)*stride + j + 5];
      q25 = recY[(i+2)*stride + j + 5];
      d = abs(p22-2*p12+p02) + abs(q22-2*q12+q02) + abs(p25-2*p15+p05) + abs(q25-2*q15+q05);
#endif
#endif
      int n;
      for (n=0;n<MIN_BLOCK_SIZE;n+=MIN_PB_SIZE){
        q_index = (i/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + ((j+n)/MIN_PB_SIZE);
        p_index = q_index - (width/MIN_PB_SIZE);
        p_mv0 = deblock_data[p_index].inter_pred.mv0;
        q_mv0 = deblock_data[q_index].inter_pred.mv0;
        p_mv1 = deblock_data[p_index].inter_pred.mv1;
        q_mv1 = deblock_data[q_index].inter_pred.mv1;
        p_mode = deblock_data[p_index].mode;
        q_mode = deblock_data[q_index].mode;
        p_cbp = deblock_data[p_index].cbp.y;
        q_cbp = deblock_data[q_index].cbp.y;
        q_size = deblock_data[q_index].size;

        if ((deblock_data[q_index].tb_split || deblock_data[q_index].pb_part == PART_HOR || deblock_data[q_index].pb_part == PART_QUAD) && q_size > MIN_BLOCK_SIZE) q_size = q_size/2;

#if NEW_MV_TEST
        mv = abs(p_mv0.y) >= 4 || abs(q_mv0.y) >= 4 || abs(p_mv0.x) >= 4 || abs(q_mv0.x) >= 4;
        mv = mv || abs(p_mv1.y) >= 4 || abs(q_mv1.y) >= 4 || abs(p_mv1.x) >= 4 || abs(q_mv1.x) >= 4;
#else
        mv = abs(p_mv0.y - q_mv0.y) >= 2 || abs(p_mv0.x - q_mv0.x) >= 2;
        mv = mv || abs(p_mv1.y - q_mv1.y) >= 2 || abs(p_mv1.x - q_mv1.x) >= 2;
#endif

        cbp = p_cbp || q_cbp;
        mode = p_mode == MODE_INTRA || q_mode == MODE_INTRA;
        interior = i%q_size > 0 ? 1 : 0;
#if MODIFIED_DEBLOCK_TEST
        do_filter = !interior && (mv || cbp || mode); //TODO: This logic needs to support 4x4TUs
#else
        do_filter = (d < beta) && !interior && (mv || cbp || mode); //TODO: This logic needs to support 4x4TUs
#endif
        if (do_filter){
          for (l=n;l<n+MIN_PB_SIZE;l++){
#if MODIFIED_DEBLOCK_TEST
            d = l & 1 ? d_26 : d_15;
            if (d < beta) {
              p2 = (int)recY[(i - 3)*stride + j + l];
              p1 = (int)recY[(i - 2)*stride + j + l];
              p0 = (int)recY[(i - 1)*stride + j + l];
              q0 = (int)recY[(i + 0)*stride + j + l];
              q1 = (int)recY[(i + 1)*stride + j + l];
              q2 = (int)recY[(i + 2)*stride + j + l];

#if NEW_DEBLOCK_FILTER
              delta = (18 * (q0 - p0) - 6 * (q1 - p1) + 0 * (q2 - p2) + 16) >> 5;
#else
              delta = (13 * (q0 - p0) + 4 * (q1 - p1) - 5 * (q2 - p2) + 16) >> 5;
#endif
              delta = clip(delta, -tc, tc);

              recY[(i - 2)*stride + j + l] = (SAMPLE)saturate(p1 + delta / 2, bitdepth);
              recY[(i - 1)*stride + j + l] = (SAMPLE)saturate(p0 + delta, bitdepth);
              recY[(i + 0)*stride + j + l] = (SAMPLE)saturate(q0 - delta, bitdepth);
              recY[(i + 1)*stride + j + l] = (SAMPLE)saturate(q1 - delta / 2, bitdepth);
            }
#else
            p2 = (int)recY[(i-3)*stride + j + l];
            p1 = (int)recY[(i-2)*stride + j + l];
            p0 = (int)recY[(i-1)*stride + j + l];
            q0 = (int)recY[(i+0)*stride + j + l];
            q1 = (int)recY[(i+1)*stride + j + l];
            q2 = (int)recY[(i+2)*stride + j + l];

#if NEW_DEBLOCK_FILTER
            delta = (18*(q0-p0) - 6*(q1-p1) + 0*(q2-p2) + 16)>>5;
#else
            delta = (13*(q0-p0) + 4*(q1-p1) - 5*(q2-p2) + 16)>>5;
#endif
            delta = clip(delta,-tc,tc);

            recY[(i-2)*stride + j + l] = (SAMPLE)saturate(p1 + delta/2, bitdepth);
            recY[(i-1)*stride + j + l] = (SAMPLE)saturate(p0 + delta, bitdepth);
            recY[(i+0)*stride + j + l] = (SAMPLE)saturate(q0 - delta, bitdepth);
            recY[(i+1)*stride + j + l] = (SAMPLE)saturate(q1 - delta/2, bitdepth);
#endif
          }
        }
      }

    }
  }
}

void TEMPLATE(deblock_frame_uv)(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp, int bitdepth)
{
  int i,j,k,l;
  int stride = rec->stride_c;
  int p1,p0,q0,q1;

  uint8_t do_filter;
  SAMPLE tc = bitdepth > 12 ? tc_table[qp] << (bitdepth-12) : tc_table[qp] >> (12-bitdepth);

  int p_index,q_index;
  block_mode_t p_mode,q_mode;
  int q_size;
  int mode,interior;
  int delta;

  for (int uv=0;uv<2;uv++){

    SAMPLE *recC = (uv ? rec->v : rec->u);

    /* Vertical filtering */
    for (i=0;i<height;i+=MIN_BLOCK_SIZE){
      for (j=MIN_BLOCK_SIZE;j<width;j+=MIN_BLOCK_SIZE){
        int i2 = i>>rec->sub;
        int j2 = j>>rec->sub;
        q_index = (i/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (j/MIN_PB_SIZE);
        p_index = q_index - 1;

        p_mode = deblock_data[p_index].mode;
        q_mode = deblock_data[q_index].mode;
        q_size = deblock_data[q_index].size;

        mode = p_mode == MODE_INTRA || q_mode == MODE_INTRA;
        interior = j%q_size > 0 ? 1 : 0;
        do_filter = !interior && mode;
        if (do_filter){
          for (k=0;k<MIN_BLOCK_SIZE>>rec->sub;k++){
            p1 = (int)recC[(i2+k)*stride + j2 - 2];
            p0 = (int)recC[(i2+k)*stride + j2 - 1];
            q0 = (int)recC[(i2+k)*stride + j2 + 0];
            q1 = (int)recC[(i2+k)*stride + j2 + 1];
            delta = (4*(q0-p0) + (p1-q1) + 4)>>3;
            delta = clip(delta,-tc,tc);
            recC[(i2+k)*stride + j2 - 1] = (SAMPLE)saturate(p0 + delta, bitdepth);
            recC[(i2+k)*stride + j2 + 0] = (SAMPLE)saturate(q0 - delta, bitdepth);
          }
        }
      }
    }

    /* Horizontal filtering */
    for (i=MIN_BLOCK_SIZE;i<height;i+=MIN_BLOCK_SIZE){
      for (j=0;j<width;j+=MIN_BLOCK_SIZE){
        int i2 = i>>rec->sub;
        int j2 = j>>rec->sub;
        q_index = (i/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (j/MIN_PB_SIZE);
        p_index = q_index - (width/MIN_PB_SIZE);
        p_mode = deblock_data[p_index].mode;
        q_mode = deblock_data[q_index].mode;
        q_size = deblock_data[q_index].size;

        mode = p_mode == MODE_INTRA || q_mode == MODE_INTRA;
        interior = i%q_size > 0 ? 1 : 0;
        do_filter = !interior && mode;
        if (do_filter){
          for (l=0;l<MIN_BLOCK_SIZE>>rec->sub;l++){
            p1 = (int)recC[(i2-2)*stride + j2 + l];
            p0 = (int)recC[(i2-1)*stride + j2 + l];
            q0 = (int)recC[(i2+0)*stride + j2 + l];
            q1 = (int)recC[(i2+1)*stride + j2 + l];
            delta = (4*(q0-p0) + (p1-q1) + 4)>>3;
            delta = clip(delta,-tc,tc);
            recC[(i2-1)*stride + j2 + l] = (SAMPLE)saturate(p0 + delta, bitdepth);
            recC[(i2+0)*stride + j2 + l] = (SAMPLE)saturate(q0 - delta, bitdepth);
          }
        }
      }
    }
  }
}


void TEMPLATE(create_yuv_frame)(yuv_frame_t  *frame, int width, int height, int sub, int pad_hor, int pad_ver, int bitdepth, int input_bitdepth)
{
  int align;

  frame->sub = sub;
  frame->width = width;
  frame->height = height;
  frame->pad_hor_y = pad_hor;
  frame->pad_ver_y = pad_ver;
  frame->pad_hor_c = pad_hor >> sub;
  frame->pad_ver_c = pad_ver >> sub;
  frame->stride_y = (width + 2*frame->pad_hor_y + 15) & ~15;
  frame->stride_c = ((width >> sub) + 2*frame->pad_hor_c + 15) & ~15;
  frame->offset_y = frame->pad_ver_y * frame->stride_y + frame->pad_hor_y;
  frame->offset_c = frame->pad_ver_c * frame->stride_c + frame->pad_hor_c;
  frame->area_y = ((height + 2*frame->pad_ver_y) * frame->stride_y + 16 + 15) & ~15;
  frame->area_c = (((height >> sub) + 2*frame->pad_ver_c) * frame->stride_c + 16 + 15) & ~15;
  frame->y = (SAMPLE *)malloc(frame->area_y*sizeof(SAMPLE))+frame->offset_y;
  frame->u = (SAMPLE *)malloc(2*frame->area_c*sizeof(SAMPLE))+frame->offset_c;
  frame->v = frame->u + frame->area_c;

  align = (16 - ((int)(uintptr_t)frame->y)) & 15;
  frame->offset_y += align;
  frame->y += align;

  align = (16 - ((int)(uintptr_t)frame->u)) & 15;
  frame->offset_c += align;
  frame->u += align;
  frame->v += align;
  frame->bitdepth = bitdepth;
  frame->input_bitdepth = input_bitdepth;
}

void TEMPLATE(close_yuv_frame)(yuv_frame_t  *frame)
{
  free(frame->y-frame->offset_y);
  free(frame->u-frame->offset_c);
}

void TEMPLATE(read_yuv_frame)(yuv_frame_t *frame, FILE *infile)
{
  int sub = frame->sub;
  int width = frame->width;
  int height = frame->height;
  int frame_bitdepth = sizeof(SAMPLE) << 3;
  int round = frame->bitdepth > frame->input_bitdepth-1 ? 1 << (frame->bitdepth-frame->input_bitdepth-1) : 0;

  for (int i=0; i<height; ++i) {
    if (fread(&frame->y[i*frame->stride_y], 1 + (frame->input_bitdepth > 8), width, infile) != width)
      fatalerror("Error reading Y from file");

    if (frame->input_bitdepth != frame->bitdepth || (frame_bitdepth == 16 && frame->bitdepth == 8)) {
      for (int j = width-1; j >= 0; j--) {
        SAMPLE *y = frame->y + i*frame->stride_y;
        y[j] = frame->input_bitdepth == 8 ?
          ((uint8_t*)y)[j] << (frame->bitdepth-frame->input_bitdepth) :
          (frame->bitdepth > frame->input_bitdepth ? y[j] << (frame->bitdepth-frame->input_bitdepth) :
           (y[j] + round) >> (frame->input_bitdepth-frame->bitdepth));
      }
    }
  }

  for (int i=0; i<height>>sub; ++i) {
    if (fread(&frame->u[i*frame->stride_c], 1 + (frame->input_bitdepth > 8), width>>sub, infile) != width>>sub)
      fatalerror("Error reading U from file");

    if (frame->input_bitdepth != frame->bitdepth || (frame_bitdepth == 16 && frame->bitdepth == 8)) {
      for (int j = (width>>sub)-1; j >= 0; j--) {
        SAMPLE *u = frame->u + i*frame->stride_c;
        u[j] = frame->input_bitdepth == 8 ?
          ((uint8_t*)u)[j] << (frame->bitdepth-frame->input_bitdepth) :
          (frame->bitdepth > frame->input_bitdepth ? u[j] << (frame->bitdepth-frame->input_bitdepth) :
           (u[j] + round) >> (frame->input_bitdepth-frame->bitdepth));
      }
    }
  }

  for (int i=0; i<height>>sub; ++i) {
    if (fread(&frame->v[i*frame->stride_c], 1 + (frame->input_bitdepth > 8), width>>sub, infile) != width>>sub)
      fatalerror("Error reading V from file");

    if (frame->input_bitdepth != frame->bitdepth || (frame_bitdepth == 16 && frame->bitdepth == 8)) {
      for (int j = (width>>sub)-1; j >= 0; j--) {
        SAMPLE *v = frame->v + i*frame->stride_c;
        v[j] = frame->input_bitdepth == 8 ?
          ((uint8_t*)v)[j] << (frame->bitdepth-frame->input_bitdepth) :
          (frame->bitdepth > frame->input_bitdepth ? v[j] << (frame->bitdepth-frame->input_bitdepth) :
           (v[j] + round) >> (frame->input_bitdepth-frame->bitdepth));
      }
    }
  }
}

void TEMPLATE(write_yuv_frame)(yuv_frame_t *frame, FILE *outfile)
{
  int sub = frame->sub;
  int width = frame->width;
  int height = frame->height;
  int round = frame->bitdepth > frame->input_bitdepth ? 1 << (frame->bitdepth - frame->input_bitdepth - 1) : 0;
  int frame_bitdepth = sizeof(SAMPLE) << 3;
  uint8_t *buf8 = thor_alloc(width, 32);
  uint8_t *buf16 = thor_alloc(width * 2, 32);

  for (int i=0; i<height; ++i) {
    if (frame->input_bitdepth == 8) {
      if (frame_bitdepth > 8) {
        for (int j = 0; j < width; j++)
          buf8[j] = saturate((frame->y[i*frame->stride_y + j] + round) >> (frame->bitdepth-8), frame->input_bitdepth);
        if (fwrite(buf8, 1, width, outfile) != width)
          fatalerror("Error writing Y to file");
      } else
        if (fwrite(&frame->y[i*frame->stride_y], 1, width, outfile) != width)
          fatalerror("Error writing Y to file");
    } else {
      if (frame->input_bitdepth == frame->bitdepth) {
        if (fwrite(&frame->y[i*frame->stride_y], 2, width, outfile) != width)
          fatalerror("Error writing Y to file");
      } else {
        for (int j = 0; j < width; j++)
          buf16[j] = frame->input_bitdepth > frame->bitdepth ? frame->y[i*frame->stride_y + j] << (frame->input_bitdepth - frame->bitdepth) :
            saturate((frame->y[i*frame->stride_y + j] + round) >> (frame->bitdepth-frame->input_bitdepth), frame->input_bitdepth);
        if (fwrite(buf16, 2, width, outfile) != width)
          fatalerror("Error writing Y to file");
      }
    }
  }
  for (int i=0; i<height>>sub; ++i) {
    if (frame->input_bitdepth == 8) {
      if (frame_bitdepth > 8) {
        for (int j = 0; j < width>>sub; j++)
          buf8[j] = saturate((frame->u[i*frame->stride_c + j] + round) >> (frame->bitdepth-8), frame->input_bitdepth);
        if (fwrite(buf8, 1, width>>sub, outfile) != width>>sub)
          fatalerror("Error writing U to file");
      } else
        if (fwrite(&frame->u[i*frame->stride_c], 1, width>>sub, outfile) != width>>sub)
          fatalerror("Error writing U to file");
    } else {
      if (frame->input_bitdepth == frame->bitdepth) {
        if (fwrite(&frame->u[i*frame->stride_c], 2, width>>sub, outfile) != width>>sub)
          fatalerror("Error writing U to file");
      } else {
        for (int j = 0; j < width>>sub; j++)
          buf16[j] = frame->input_bitdepth > frame->bitdepth ? frame->u[i*frame->stride_c + j] << (frame->input_bitdepth - frame->bitdepth) :
            saturate((frame->u[i*frame->stride_c + j] + round) >> (frame->bitdepth-frame->input_bitdepth), frame->input_bitdepth);
        if (fwrite(buf16, 2, width>>sub, outfile) != width>>sub)
          fatalerror("Error writing U to file");
      }
    }
  }
  for (int i=0; i<height>>sub; ++i) {
    if (frame->input_bitdepth == 8) {
      if (frame_bitdepth > 8) {
        for (int j = 0; j < width>>sub; j++)
          buf8[j] = saturate((frame->v[i*frame->stride_c + j] + round) >> (frame->bitdepth-8), frame->input_bitdepth);
        if (fwrite(buf8, 1, width>>sub, outfile) != width>>sub)
          fatalerror("Error writing V to file");
      } else
        if (fwrite(&frame->v[i*frame->stride_c], 1, width>>sub, outfile) != width>>sub)
          fatalerror("Error writing V to file");
    } else {
      if (frame->input_bitdepth == frame->bitdepth) {
        if (fwrite(&frame->v[i*frame->stride_c], 2, width>>sub, outfile) != width>>sub)
          fatalerror("Error writing V to file");
      } else {
        for (int j = 0; j < width>>sub; j++)
          buf16[j] = frame->input_bitdepth > frame->bitdepth ? frame->v[i*frame->stride_c + j] << (frame->input_bitdepth - frame->bitdepth) :
            saturate((frame->v[i*frame->stride_c + j] + round) >> (frame->bitdepth-frame->input_bitdepth), frame->input_bitdepth);
        if (fwrite(buf16, 2, width>>sub, outfile) != width>>sub)
          fatalerror("Error writing V to file");
      }
    }
  }

  thor_free(buf8);
  thor_free(buf16);
}


void TEMPLATE(pad_yuv_frame)(yuv_frame_t * f)
{
  int sy = f->stride_y;
  int sc = f->stride_c;
  int w = f->width;
  int h = f->height;
  int i;
  SAMPLE val;
  int frame_bitdepth = sizeof(SAMPLE) << 3;

  /* Y */
  /* Left and right */
  for (i=0;i<h;i++)
  {
    val=f->y[i*sy];
    if (frame_bitdepth == 8)
      memset(&f->y[i*sy-f->pad_hor_y],val,f->pad_hor_y);
    else
      for (int j = 0; j < f->pad_hor_y; j++)
        f->y[i*sy-f->pad_hor_y+j] = val;
    val=f->y[i*sy+w-1];
    if (frame_bitdepth == 8)
      memset(&f->y[i*sy+w],val,f->pad_hor_y);
    else
      for (int j = 0; j < f->pad_hor_y; j++)
        f->y[i*sy+w+j] = val;
  }
  /* Top and bottom */
  for (i=-f->pad_ver_y;i<0;i++)
  {
    memcpy(&f->y[i*sy-f->pad_hor_y], &f->y[-f->pad_hor_y], (w+2*f->pad_hor_y)*frame_bitdepth / 8);
  }
  for (i=h;i<h+f->pad_ver_y;i++)
  {
    memcpy(&f->y[i*sy-f->pad_hor_y], &f->y[(h-1)*sy-f->pad_hor_y], (w+2*f->pad_hor_y)*frame_bitdepth / 8);
  }

  /* UV */

  /* Left and right */
  w >>= f->sub;
  h >>= f->sub;
  for (i=0;i<h;i++)
  {
    val=f->u[i*sc];
    if (frame_bitdepth == 8)
      memset(&f->u[i*sc-f->pad_hor_c],val,f->pad_hor_c);
    else
      for (int j = 0; j < f->pad_hor_c; j++)
        f->u[i*sc-f->pad_hor_c+j] = val;
    val=f->u[i*sc+w-1];
    if (frame_bitdepth == 8)
      memset(&f->u[i*sc+w],val,f->pad_hor_c);
    else
      for (int j = 0; j < f->pad_hor_c; j++)
        f->u[i*sc+w+j] = val;

    val=f->v[i*sc];
    if (frame_bitdepth == 8)
      memset(&f->v[i*sc-f->pad_hor_c],val,f->pad_hor_c);
    else
      for (int j = 0; j < f->pad_hor_c; j++)
        f->v[i*sc-f->pad_hor_c+j] = val;

    val=f->v[i*sc+w-1];
    if (frame_bitdepth == 8)
      memset(&f->v[i*sc+w],val,f->pad_hor_c);
    else
      for (int j = 0; j < f->pad_hor_c; j++)
        f->v[i*sc+w+j] = val;
  }

  /* Top and bottom */
  for (i=-f->pad_ver_c;i<0;i++)
  {
    memcpy(&f->u[i*sc-f->pad_hor_c], &f->u[-f->pad_hor_c], (w+2*f->pad_hor_c)*frame_bitdepth / 8);
    memcpy(&f->v[i*sc-f->pad_hor_c], &f->v[-f->pad_hor_c], (w+2*f->pad_hor_c)*frame_bitdepth / 8);
  }
  for (i=h;i<h+f->pad_ver_c;i++)
  {
    memcpy(&f->u[i*sc-f->pad_hor_c], &f->u[(h-1)*sc-f->pad_hor_c], (w+2*f->pad_hor_c)*frame_bitdepth / 8);
    memcpy(&f->v[i*sc-f->pad_hor_c], &f->v[(h-1)*sc-f->pad_hor_c], (w+2*f->pad_hor_c)*frame_bitdepth / 8);
  }
}

void TEMPLATE(create_reference_frame)(yuv_frame_t  *ref,yuv_frame_t  *rec)
{
  ref->frame_num = rec->frame_num;
  int height = rec->height;
  int width = rec->width;  
  int i;
  SAMPLE *ref_y = ref->y;
  SAMPLE *ref_u = ref->u;
  SAMPLE *ref_v = ref->v;
  for (i=0;i<height;i++){
    memcpy(&ref_y[i*ref->stride_y],&rec->y[i*rec->stride_y],width*sizeof(SAMPLE));
  }
  for (i=0;i<height>>ref->sub;i++){
    memcpy(&ref_u[i*ref->stride_c],&rec->u[i*rec->stride_c],(width>>ref->sub)*sizeof(SAMPLE));
    memcpy(&ref_v[i*ref->stride_c],&rec->v[i*rec->stride_c],(width>>ref->sub)*sizeof(SAMPLE));
  }

  TEMPLATE(pad_yuv_frame)(ref);
}

void TEMPLATE(clpf_frame)(yuv_frame_t *dst, yuv_frame_t *rec, yuv_frame_t *org, const deblock_data_t *deblock_data, void *stream, int enable_sb_flag, unsigned int strength, unsigned int fb_size_log2, int bitdepth,
                          int(*decision)(int, int, yuv_frame_t *, yuv_frame_t *, const deblock_data_t *, int, int, int, void *, unsigned int, unsigned int, unsigned int)) {

  /* Constrained low-pass filter (CLPF) */
  int width = rec->width;
  int height = rec->height;
  int xpos, ypos, index;
  int stride_y = rec->stride_y;
  int stride_c = rec->stride_c;
  const int bs = 8;

  int num_sb_hor = (width+(1<<fb_size_log2)-bs)>>fb_size_log2;
  int num_sb_ver = (height+(1<<fb_size_log2)-bs)>>fb_size_log2;

  strength <<= bitdepth - 8;

  for (int k = 0; k < num_sb_ver; k++) {
    for (int l = 0; l < num_sb_hor; l++) {
      int allskip = 1;
      for (int m = 0; allskip && m < (1<<fb_size_log2)/bs; m++) {
        for (int n = 0; allskip && n < (1<<fb_size_log2)/bs; n++) {
          xpos = (l<<fb_size_log2) + n*bs;
          ypos = (k<<fb_size_log2) + m*bs;
          if (xpos < width && ypos < height) {
            index = (ypos/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (xpos/MIN_PB_SIZE);
            allskip &= deblock_data[index].mode == MODE_SKIP;
          }
        }
      }
      int h = min(height, (k+1)<<fb_size_log2) & ((1<<fb_size_log2)-1);
      int w = min(width, (l+1)<<fb_size_log2) & ((1<<fb_size_log2)-1);
      h += !h << fb_size_log2;
      w += !w << fb_size_log2;
      if (!allskip && (!enable_sb_flag || decision(k, l, rec, org, deblock_data, bs, w/bs, h/bs, stream, strength, fb_size_log2, bitdepth-8))) {
        for (int m = 0; m < h/bs; m++) {
          for (int n = 0; n < w/bs; n++) {
            xpos = (l<<fb_size_log2) + n*bs;
            ypos = (k<<fb_size_log2) + m*bs;
            index = (ypos/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (xpos/MIN_PB_SIZE);
            int filter = deblock_data[index].mode != MODE_SKIP;

            if (filter) {
              if (use_simd) {
                TEMPLATE(clpf_block_simd)(rec->y, dst->y, stride_y, xpos, ypos, bs, bs, width, height, strength);
                TEMPLATE(clpf_block_simd)(rec->u, dst->u, stride_c, xpos >> rec->sub, ypos >> rec->sub, bs >> rec->sub, bs >> rec->sub, width >> rec->sub, height >> rec->sub, strength);
                TEMPLATE(clpf_block_simd)(rec->v, dst->v, stride_c, xpos >> rec->sub, ypos >> rec->sub, bs >> rec->sub, bs >> rec->sub, width >> rec->sub, height >> rec->sub, strength);
              } else {
                TEMPLATE(clpf_block)(rec->y, dst->y, stride_y, xpos, ypos, bs, bs, width, height, strength);
                TEMPLATE(clpf_block)(rec->u, dst->u, stride_c, xpos >> rec->sub, ypos >> rec->sub, bs >> rec->sub, bs >> rec->sub, width >> rec->sub, height >> rec->sub, strength);
                TEMPLATE(clpf_block)(rec->v, dst->v, stride_c, xpos >> rec->sub, ypos >> rec->sub, bs >> rec->sub, bs >> rec->sub, width >> rec->sub, height >> rec->sub, strength);
              }
            } else { // Copy
              for (int c = 0; c < bs; c++)
                for (int d = 0; d < bs; d += 8 / sizeof(SAMPLE))
                  *(uint64_t*)(dst->y + (ypos + c)*stride_y + xpos + d) =
                    *(uint64_t*)(rec->y + (ypos + c)*stride_y + xpos + d);
              if (rec->sub) {
                for (int c = 0; c < (bs >> rec->sub); c++)
                  for (int d = 0; d < bs; d += 4 / sizeof(SAMPLE)) {
                    *(uint32_t*)(dst->u + ((ypos >> dst->sub) + c)*stride_c + (xpos >> 1) + d) =
                      *(uint32_t*)(rec->u + ((ypos >> rec->sub) + c)*stride_c + (xpos >> 1) + d);
                    *(uint32_t*)(dst->v + ((ypos >> dst->sub) + c)*stride_c + (xpos >> 1) + d) =
                      *(uint32_t*)(rec->v + ((ypos >> dst->sub) + c)*stride_c + (xpos >> 1) + d);
                  }
              } else {
                for (int c = 0; c < (bs >> rec->sub); c++)
                  for (int d = 0; d < bs; d += 8 / sizeof(SAMPLE)) {
                    *(uint64_t*)(dst->u + ((ypos >> dst->sub) + c)*stride_c + xpos + d) =
                      *(uint64_t*)(rec->u + ((ypos >> rec->sub) + c)*stride_c + xpos + d);
                    *(uint64_t*)(dst->v + ((ypos >> dst->sub) + c)*stride_c + xpos + d) =
                      *(uint64_t*)(rec->v + ((ypos >> dst->sub) + c)*stride_c + xpos + d);
                  }
              }
            }
          }
        }
      } else { // Copy
        for (int m = 0; m < h; m++)
          memcpy(dst->y + ((k<<fb_size_log2)+m)*stride_y + (l<<fb_size_log2),
                 rec->y + ((k<<fb_size_log2)+m)*stride_y + (l<<fb_size_log2), w*sizeof(SAMPLE));
        for (int m = 0; m < h >> dst->sub; m++) {
          memcpy(dst->u + (((k<<fb_size_log2) >> dst->sub)+m)*stride_c + ((l<<fb_size_log2) >> dst->sub),
                 rec->u + (((k<<fb_size_log2) >> rec->sub)+m)*stride_c + ((l<<fb_size_log2) >> rec->sub), (w >> rec->sub)*sizeof(SAMPLE));
          memcpy(dst->v + (((k<<fb_size_log2) >> dst->sub)+m)*stride_c + ((l<<fb_size_log2) >> dst->sub),
                 rec->v + (((k<<fb_size_log2) >> rec->sub)+m)*stride_c + ((l<<fb_size_log2) >> rec->sub), (w >> rec->sub)*sizeof(SAMPLE));
        }
      }
    }
  }
}
