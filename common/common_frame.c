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

int beta_table[52] = {
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
     16, 17, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64
};

static const int tc_table[56] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,6,7,8,9,9,10,10,11,11,12,12,13,13,14,14
};

void deblock_frame_y(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp)
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
  uint8_t *recY = rec->y;
  uint8_t do_filter;
  uint8_t beta = beta_table[qp];
  uint8_t tc = tc_table[qp]; //TODO: increment with 4 for intra

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

              recY[(i + k)*stride + j - 2] = (uint8_t)clip255(p1 + delta / 2);
              recY[(i + k)*stride + j - 1] = (uint8_t)clip255(p0 + delta);
              recY[(i + k)*stride + j + 0] = (uint8_t)clip255(q0 - delta);
              recY[(i + k)*stride + j + 1] = (uint8_t)clip255(q1 - delta / 2);
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

            recY[(i+k)*stride + j - 2] = (uint8_t)clip255(p1 + delta/2);
            recY[(i+k)*stride + j - 1] = (uint8_t)clip255(p0 + delta);
            recY[(i+k)*stride + j + 0] = (uint8_t)clip255(q0 - delta);
            recY[(i+k)*stride + j + 1] = (uint8_t)clip255(q1 - delta/2);
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

              recY[(i - 2)*stride + j + l] = (uint8_t)clip255(p1 + delta / 2);
              recY[(i - 1)*stride + j + l] = (uint8_t)clip255(p0 + delta);
              recY[(i + 0)*stride + j + l] = (uint8_t)clip255(q0 - delta);
              recY[(i + 1)*stride + j + l] = (uint8_t)clip255(q1 - delta / 2);
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

            recY[(i-2)*stride + j + l] = (uint8_t)clip255(p1 + delta/2);
            recY[(i-1)*stride + j + l] = (uint8_t)clip255(p0 + delta);
            recY[(i+0)*stride + j + l] = (uint8_t)clip255(q0 - delta);
            recY[(i+1)*stride + j + l] = (uint8_t)clip255(q1 - delta/2);
#endif
          }
        }
      }

    }
  }
}

void deblock_frame_uv(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp)
{
  int i,j,k,l;
  int stride = rec->stride_c;
  int p1,p0,q0,q1;

  uint8_t do_filter;
  uint8_t tc = tc_table[qp];

  int p_index,q_index;
  block_mode_t p_mode,q_mode;
  int q_size;
  int mode,interior;
  int delta;

  for (int uv=0;uv<2;uv++){

    uint8_t *recC = (uv ? rec->v : rec->u);

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
            recC[(i2+k)*stride + j2 - 1] = (uint8_t)clip255(p0 + delta);
            recC[(i2+k)*stride + j2 + 0] = (uint8_t)clip255(q0 - delta);
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
            recC[(i2-1)*stride + j2 + l] = (uint8_t)clip255(p0 + delta);
            recC[(i2+0)*stride + j2 + l] = (uint8_t)clip255(q0 - delta);
          }
        }
      }
    }
  }
}


void create_yuv_frame(yuv_frame_t  *frame, int width, int height, int sub, int pad_hor, int pad_ver, int subsample)
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
  frame->y = (uint8_t *)malloc(frame->area_y*sizeof(uint8_t))+frame->offset_y;
  frame->u = (uint8_t *)malloc(2*frame->area_c*sizeof(uint8_t))+frame->offset_c;
  frame->v = frame->u + frame->area_c*sizeof(uint8_t);
  if (!sub && subsample) {
    frame->pad_hor_c2 = pad_hor >> 1;
    frame->pad_ver_c2 = pad_ver >> 1;
    frame->stride_c2 = ((width >> 1) + 2*frame->pad_hor_c2 + 15) & ~15;
    frame->offset_c2 = frame->pad_ver_c2 * frame->stride_c2 + frame->pad_hor_c2;
    int area_c = (((height >> 1) + 2*frame->pad_ver_c2) * frame->stride_c2 + 16 + 15) & ~15;
    frame->u2 = (uint8_t *)malloc(2*area_c*sizeof(uint8_t))+frame->offset_c2;
    frame->v2 = frame->u2 + area_c*sizeof(uint8_t);
    align = (16 - ((int)(uintptr_t)frame->u2)) & 15;
    frame->offset_c2 += align;
    frame->u2 += align;
    frame->v2 += align;
  } else
    frame->u2 = frame->v2 = 0;

  align = (16 - ((int)(uintptr_t)frame->y)) & 15;
  frame->offset_y += align;
  frame->y += align;

  align = (16 - ((int)(uintptr_t)frame->u)) & 15;
  frame->offset_c += align;
  frame->u += align;
  frame->v += align;
}

void close_yuv_frame(yuv_frame_t  *frame)
{
  free(frame->y-frame->offset_y);
  free(frame->u-frame->offset_c);
  if (frame->u2)
    free(frame->u2-frame->offset_c2);
}

void subsample_yuv_frame(yuv_frame_t  *frame)
{
  if (frame->u2)
    for (int i = 0; i < frame->height; i += 2) {
      int pos = i * frame->stride_c;
      int pos2 = pos + frame->stride_c;
      for (int j = 0; j < frame->width; j += 2) {
	frame->u2[i/2*frame->stride_c2+j/2] =
	  (frame->u[pos+j] + frame->u[pos+1+j] + frame->u[pos2+j] + frame->u[pos2+1+j] + 2) >> 2;
	frame->v2[i/2*frame->stride_c2+j/2] =
	  (frame->v[pos+j] + frame->v[pos+1+j] + frame->v[pos2+j] + frame->v[pos2+1+j] + 2) >> 2;
      }
    }
}

void swap_chroma(yuv_frame_t *frame)
{
  uint8_t *tmp;
  int tmp2;

  tmp = frame->u2;
  frame->u2 = frame->u;
  frame->u = tmp;
  tmp = frame->v2;
  frame->v2 = frame->v;
  frame->v = tmp;
  tmp2 = frame->stride_c2;
  frame->stride_c2 = frame->stride_c;
  frame->stride_c = tmp2;
  tmp2 = frame->offset_c2;
  frame->offset_c2 = frame->offset_c;
  frame->offset_c = tmp2;
  frame->sub ^= 1;
}

void read_yuv_frame(yuv_frame_t *frame, FILE *infile)
{
  int sub = frame->sub;
  int width = frame->width;
  int height = frame->height;

  for (int i=0; i<height; ++i) {
    if (fread(&frame->y[i*frame->stride_y], sizeof(unsigned char), width, infile) != width)
    {
      fatalerror("Error reading Y from file");
    }
  }
  for (int i=0; i<height>>sub; ++i) {
    if (fread(&frame->u[i*frame->stride_c], sizeof(unsigned char), width>>sub, infile) != width>>sub)
    {
      fatalerror("Error reading U from file");
    }
  }
  for (int i=0; i<height>>sub; ++i) {
    if (fread(&frame->v[i*frame->stride_c], sizeof(unsigned char), width>>sub, infile) != width>>sub)
    {
      fatalerror("Error reading V from file");
    }
  }

}

void write_yuv_frame(yuv_frame_t *frame, FILE *outfile)
{
  int sub = frame->sub;
  int width = frame->width;
  int height = frame->height;

  for (int i=0; i<height; ++i) {
    if (fwrite(&frame->y[i*frame->stride_y], sizeof(unsigned char), width, outfile) != width)
    {
      fatalerror("Error reading Y from file");
    }
  }
  for (int i=0; i<height>>sub; ++i) {
    if (fwrite(&frame->u[i*frame->stride_c], sizeof(unsigned char), width>>sub, outfile) != width>>sub)
    {
      fatalerror("Error reading U from file");
    }
  }
  for (int i=0; i<height>>sub; ++i) {
    if (fwrite(&frame->v[i*frame->stride_c], sizeof(unsigned char), width>>sub, outfile) != width>>sub)
    {
      fatalerror("Error reading V from file");
    }
  }
}


void pad_yuv_frame(yuv_frame_t * f)
{
  int sy = f->stride_y;
  int sc = f->stride_c;
  int w = f->width;
  int h = f->height;
  int i;
  uint8_t val;
  /* Y */
  /* Left and right */
  for (i=0;i<h;i++)
  {
    val=f->y[i*sy];
    memset(&f->y[i*sy-f->pad_hor_y],val,f->pad_hor_y*sizeof(uint8_t));
    val=f->y[i*sy+w-1];
    memset(&f->y[i*sy+w],val,f->pad_hor_y*sizeof(uint8_t));
  }
  /* Top and bottom */
  for (i=-f->pad_ver_y;i<0;i++)
  {
    memcpy(&f->y[i*sy-f->pad_hor_y], &f->y[-f->pad_hor_y], w+2*f->pad_hor_y);
  }
  for (i=h;i<h+f->pad_ver_y;i++)
  {
    memcpy(&f->y[i*sy-f->pad_hor_y], &f->y[(h-1)*sy-f->pad_hor_y], w+2*f->pad_hor_y);
  }

  /* UV */

  /* Left and right */
  w >>= f->sub;
  h >>= f->sub;
  for (i=0;i<h;i++)
  {
    val=f->u[i*sc];
    memset(&f->u[i*sc-f->pad_hor_c],val,f->pad_hor_c*sizeof(uint8_t));
    val=f->u[i*sc+w-1];
    memset(&f->u[i*sc+w],val,f->pad_hor_c*sizeof(uint8_t));

    val=f->v[i*sc];
    memset(&f->v[i*sc-f->pad_hor_c],val,f->pad_hor_c*sizeof(uint8_t));
    val=f->v[i*sc+w-1];
    memset(&f->v[i*sc+w],val,f->pad_hor_c*sizeof(uint8_t));
  }

  /* Top and bottom */
  for (i=-f->pad_ver_c;i<0;i++)
  {
    memcpy(&f->u[i*sc-f->pad_hor_c], &f->u[-f->pad_hor_c], w+2*f->pad_hor_c);
    memcpy(&f->v[i*sc-f->pad_hor_c], &f->v[-f->pad_hor_c], w+2*f->pad_hor_c);
  }
  for (i=h;i<h+f->pad_ver_c;i++)
  {
    memcpy(&f->u[i*sc-f->pad_hor_c], &f->u[(h-1)*sc-f->pad_hor_c], w+2*f->pad_hor_c);
    memcpy(&f->v[i*sc-f->pad_hor_c], &f->v[(h-1)*sc-f->pad_hor_c], w+2*f->pad_hor_c);
  }

  if (f->u2) {
    sc = f->stride_c2;

    /* Left and right */
    w = f->width >> 1;
    h = f->height >> 1;
    for (i=0;i<h;i++)
    {
      val=f->u2[i*sc];
      memset(&f->u2[i*sc-f->pad_hor_c2],val,f->pad_hor_c2*sizeof(uint8_t));
      val=f->u2[i*sc+w-1];
      memset(&f->u2[i*sc+w],val,f->pad_hor_c2*sizeof(uint8_t));

      val=f->v2[i*sc];
      memset(&f->v2[i*sc-f->pad_hor_c2],val,f->pad_hor_c2*sizeof(uint8_t));
      val=f->v2[i*sc+w-1];
      memset(&f->v2[i*sc+w],val,f->pad_hor_c2*sizeof(uint8_t));
    }

    /* Top and bottom */
    for (i=-f->pad_ver_c2;i<0;i++)
    {
      memcpy(&f->u2[i*sc-f->pad_hor_c2], &f->u2[-f->pad_hor_c2], w+2*f->pad_hor_c2);
      memcpy(&f->v2[i*sc-f->pad_hor_c2], &f->v2[-f->pad_hor_c2], w+2*f->pad_hor_c2);
    }
    for (i=h;i<h+f->pad_ver_c2;i++)
    {
      memcpy(&f->u2[i*sc-f->pad_hor_c2], &f->u2[(h-1)*sc-f->pad_hor_c2], w+2*f->pad_hor_c2);
      memcpy(&f->v2[i*sc-f->pad_hor_c2], &f->v2[(h-1)*sc-f->pad_hor_c2], w+2*f->pad_hor_c2);
    }
  }
}

void create_reference_frame(yuv_frame_t  *ref,yuv_frame_t  *rec)
{
  ref->frame_num = rec->frame_num;
  int height = rec->height;
  int width = rec->width;  
  int i;
  uint8_t *ref_y = ref->y;
  uint8_t *ref_u = ref->u;
  uint8_t *ref_v = ref->v;
  for (i=0;i<height;i++){
    memcpy(&ref_y[i*ref->stride_y],&rec->y[i*rec->stride_y],width*sizeof(uint8_t)); 
  }
  for (i=0;i<height>>ref->sub;i++){
    memcpy(&ref_u[i*ref->stride_c],&rec->u[i*rec->stride_c],(width>>ref->sub)*sizeof(uint8_t));
    memcpy(&ref_v[i*ref->stride_c],&rec->v[i*rec->stride_c],(width>>ref->sub)*sizeof(uint8_t));
  }

  subsample_yuv_frame(ref);
  pad_yuv_frame(ref);
}

void clpf_frame(yuv_frame_t *dst, yuv_frame_t *rec, yuv_frame_t *org, const deblock_data_t *deblock_data, void *stream, int enable_sb_flag, unsigned int strength, unsigned int fb_size_log2,
                int(*decision)(int, int, yuv_frame_t *, yuv_frame_t *, const deblock_data_t *, int, int, int, void *, unsigned int, unsigned int)) {

  /* Constrained low-pass filter (CLPF) */
  int width = rec->width;
  int height = rec->height;
  int xpos, ypos, index;
  int stride_y = rec->stride_y;
  int stride_c = rec->stride_c;
  const int bs = 8;

  int num_sb_hor = (width+(1<<fb_size_log2)-bs)>>fb_size_log2;
  int num_sb_ver = (height+(1<<fb_size_log2)-bs)>>fb_size_log2;

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
      if (!allskip && (!enable_sb_flag || decision(k, l, rec, org, deblock_data, bs, w/bs, h/bs, stream, strength, fb_size_log2))) {
        for (int m = 0; m < h/bs; m++) {
          for (int n = 0; n < w/bs; n++) {
            xpos = (l<<fb_size_log2) + n*bs;
            ypos = (k<<fb_size_log2) + m*bs;
            index = (ypos/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (xpos/MIN_PB_SIZE);
            int filter = deblock_data[index].mode != MODE_SKIP;

            if (filter) {
              (use_simd ? clpf_block_simd : clpf_block)(rec->y, dst->y, stride_y, xpos, ypos, bs, bs, width, height, strength);
              (use_simd ? clpf_block_simd : clpf_block)(rec->u, dst->u, stride_c, xpos >> rec->sub, ypos >> rec->sub, bs >> rec->sub, bs >> rec->sub, width >> rec->sub, height >> rec->sub, strength);
              (use_simd ? clpf_block_simd : clpf_block)(rec->v, dst->v, stride_c, xpos >> rec->sub, ypos >> rec->sub, bs >> rec->sub, bs >> rec->sub, width >> rec->sub, height >> rec->sub, strength);
            } else { // Copy
              for (int c = 0; c < bs; c++)
                *(uint64_t*)(dst->y + (ypos + c)*stride_y + xpos) =
                  *(uint64_t*)(rec->y + (ypos + c)*stride_y + xpos);
              if (rec->sub) {
                for (int c = 0; c < bs >> rec->sub; c++) {
                  *(uint32_t*)(dst->u + ((ypos >> dst->sub) + c)*stride_c + (xpos >> 1)) =
                    *(uint32_t*)(rec->u + ((ypos >> rec->sub) + c)*stride_c + (xpos >> 1));
                  *(uint32_t*)(dst->v + ((ypos >> dst->sub) + c)*stride_c + (xpos >> 1)) =
                    *(uint32_t*)(rec->v + ((ypos >> dst->sub) + c)*stride_c + (xpos >> 1));
                }
              } else {
                for (int c = 0; c < bs >> rec->sub; c++) {
                  *(uint64_t*)(dst->u + ((ypos >> dst->sub) + c)*stride_c + xpos) =
                    *(uint64_t*)(rec->u + ((ypos >> rec->sub) + c)*stride_c + xpos);
                  *(uint64_t*)(dst->v + ((ypos >> dst->sub) + c)*stride_c + xpos) =
                    *(uint64_t*)(rec->v + ((ypos >> dst->sub) + c)*stride_c + xpos);
                }
              }
            }
          }
        }
      } else { // Copy
        for (int m = 0; m < h; m++)
          memcpy(dst->y + ((k<<fb_size_log2)+m)*stride_y + (l<<fb_size_log2),
                 rec->y + ((k<<fb_size_log2)+m)*stride_y + (l<<fb_size_log2), w);
        for (int m = 0; m < h >> dst->sub; m++) {
          memcpy(dst->u + (((k<<fb_size_log2) >> dst->sub)+m)*stride_c + ((l<<fb_size_log2) >> dst->sub),
                 rec->u + (((k<<fb_size_log2) >> rec->sub)+m)*stride_c + ((l<<fb_size_log2) >> rec->sub), w >> rec->sub);
          memcpy(dst->v + (((k<<fb_size_log2) >> dst->sub)+m)*stride_c + ((l<<fb_size_log2) >> dst->sub),
                 rec->v + (((k<<fb_size_log2) >> rec->sub)+m)*stride_c + ((l<<fb_size_log2) >> rec->sub), w >> rec->sub);
        }
      }
    }
  }
}
