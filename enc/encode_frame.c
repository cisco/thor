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

#include "global.h"
#include <string.h>

#include "mainenc.h"
#include "encode_block.h"
#include "common_block.h"
#include "common_frame.h"
#include "putvlc.h"
#include "wt_matrix.h"
#include "enc_kernels.h"
#include "inter_prediction.h"
#include "write_bits.h"

extern int chroma_qp[52];
extern double squared_lambda_QP[52];

static int clpf_decision(int k, int l, yuv_frame_t *rec, yuv_frame_t *org, const deblock_data_t *deblock_data, int block_size, int w, int h, void *stream, unsigned int strength, unsigned int fb_size_log2, unsigned int shift) {
  int sum0 = 0, sum1 = 0;
  for (int m = 0; m < h; m++) {
    for (int n = 0; n < w; n++) {
      int xpos = (l<<fb_size_log2) + n*block_size;
      int ypos = (k<<fb_size_log2) + m*block_size;
      int index = (ypos / MIN_PB_SIZE)*(rec->width / MIN_PB_SIZE) + (xpos / MIN_PB_SIZE);
      if (deblock_data[index].mode != MODE_SKIP) {
        if (use_simd && sizeof(SAMPLE) == 1)
          detect_clpf_simd((uint8_t *)rec->y, (uint8_t *)org->y, xpos, ypos, rec->width, rec->height, org->stride_y, rec->stride_y, &sum0, &sum1, strength);
        else
          TEMPLATE(detect_clpf)(rec->y, org->y, xpos, ypos, rec->width, rec->height, org->stride_y, rec->stride_y, &sum0, &sum1, strength, shift);
      }
    }
  }
  put_flc(1, sum1 < sum0, (stream_t*)stream);
  return sum1 < sum0;
}

// Calculate the square error of all filter settings.  Result:
// res[0][0]   : unfiltered
// res[0][1-3] : strength=1,2,4, no signals
// res[1][0]   : (bit count, fb size = 128)
// res[1][1-3] : strength=1,2,4, fb size = 128
// res[2][0]   : (bit count, fb size = 64)
// res[2][1-3] : strength=1,2,4, fb size = 64
// res[3][0]   : (bit count, fb size = 32)
// res[3][1-3] : strength=1,2,4, fb size = 32
static int clpf_rdo(int y, int x, yuv_frame_t *rec, yuv_frame_t *org, const deblock_data_t *deblock_data, unsigned int block_size, unsigned int fb_size_log2, int w, int h, int64_t res[4][4], int bitdepth) {
  int filtered = 0;
  int sum[4];
  int bslog = log2i(block_size);
  sum[0] = sum[1] = sum[2] = sum[3] = 0;
  if (fb_size_log2 > log2i(MAX_SB_SIZE) - 3) {
    fb_size_log2--;
    int w1 = min(1<<(fb_size_log2-bslog), w);
    int h1 = min(1<<(fb_size_log2-bslog), h);
    int w2 = min(w - (1<<(fb_size_log2-bslog)), w>>1);
    int h2 = min(h - (1<<(fb_size_log2-bslog)), h>>1);
    int i = log2i(MAX_SB_SIZE) - fb_size_log2;
    int64_t sum1 = res[i][1], sum2 = res[i][2], sum3 = res[i][3];
    int64_t oldfiltered = res[i][0];
    res[i][0] = 0;

    filtered = clpf_rdo(y, x, rec, org, deblock_data, block_size, fb_size_log2, w1, h1, res, bitdepth);
    if (1<<(fb_size_log2-bslog) < w)
      filtered |= clpf_rdo(y, x+(1<<fb_size_log2), rec, org, deblock_data, block_size, fb_size_log2, w2, h1, res, bitdepth);
    if (1<<(fb_size_log2-bslog) < h) {
      filtered |= clpf_rdo(y+(1<<fb_size_log2), x, rec, org, deblock_data, block_size, fb_size_log2, w1, h2, res, bitdepth);
      filtered |= clpf_rdo(y+(1<<fb_size_log2), x+(1<<fb_size_log2), rec, org, deblock_data, block_size, fb_size_log2, w2, h2, res, bitdepth);
    }

    res[i][1] = min(sum1 + res[i][0], res[i][1]);
    res[i][2] = min(sum2 + res[i][0], res[i][2]);
    res[i][3] = min(sum3 + res[i][0], res[i][3]);
    res[i][0] = oldfiltered + filtered; // Number of signal bits
    return filtered;
  }

  for (int m = 0; m < h; m++) {
    for (int n = 0; n < w; n++) {
      int xpos = x + n*block_size;
      int ypos = y + m*block_size;
      int index = (ypos / MIN_PB_SIZE)*(rec->width / MIN_PB_SIZE) + (xpos / MIN_PB_SIZE);
      if (deblock_data[index].mode != MODE_SKIP) {
        if (use_simd && sizeof(SAMPLE) == 1)
          detect_multi_clpf_simd((uint8_t *)rec->y, (uint8_t *)org->y, xpos, ypos, rec->width, rec->height, org->stride_y, rec->stride_y, sum);
        else
          TEMPLATE(detect_multi_clpf)(rec->y, org->y, xpos, ypos, rec->width, rec->height, org->stride_y, rec->stride_y, sum, bitdepth - 8);
        filtered = 1;
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    res[i][0] += sum[0];
    res[i][1] += sum[1];
    res[i][2] += sum[2];
    res[i][3] += sum[3];
  }
  return filtered;
}

static void clpf_test_frame(yuv_frame_t *rec, yuv_frame_t *org, const deblock_data_t *deblock_data, const frame_info_t *frame_info, int *best_strength, int *best_bs, int bitdepth) {

  int64_t sums[4][4];
  int width = rec->width, height = rec->height;
  const int bs = 8;
  memset(sums, 0, sizeof(sums));
  int fb_size_log2 = log2i(MAX_SB_SIZE);

  for (int k = 0; k < (height+(1<<fb_size_log2)-bs)>>fb_size_log2; k++) {
    for (int l = 0; l < (width+(1<<fb_size_log2)-bs)>>fb_size_log2; l++) {
      int h = min(height, (k+1)<<fb_size_log2) & ((1<<fb_size_log2)-1);
      int w = min(width, (l+1)<<fb_size_log2) & ((1<<fb_size_log2)-1);
      h += !h << fb_size_log2;
      w += !w << fb_size_log2;
      clpf_rdo((k<<fb_size_log2), (l<<fb_size_log2), rec, org, deblock_data, bs, fb_size_log2, w/bs, h/bs, sums, bitdepth);
    }
  }
  for (int j = 0; j < 4; j++) {
    int cost = (int)((frame_info->lambda * sums[j][0] + 0.5));
    for (int i = 0; i < 4; i++) {
      int i_max = min(frame_info->max_clpf_strength, 3);
      if (i > i_max) sums[j][i] = 1 << 30;
      sums[j][i] = ((sums[j][i] + (i && j) * cost) << 4) + j * 4 + i;
    }
  }

  int64_t best = (int64_t)1 << 62;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      if ((!i || j) && sums[i][j] < best)
        best = sums[i][j];
  best &= 15;
  *best_bs = (best > 3) * (5 + (best < 12) + (best < 8));
  *best_strength = best ? 1<<((best-1) & 3) : 0;  
}

void TEMPLATE(encode_frame)(encoder_info_t *encoder_info)
{
  int k,l;
  int width = encoder_info->width;
  int height = encoder_info->height;  
  int sb_size = 1 << encoder_info->params->log2_sb_size;
  int num_sb_hor = (width + sb_size - 1) / sb_size;
  int num_sb_ver = (height + sb_size - 1) / sb_size;
  stream_t *stream = encoder_info->stream;

  if (encoder_info->frame_info.frame_type == I_FRAME)
    memset(encoder_info->deblock_data, 0, ((height/MIN_PB_SIZE) * (width/MIN_PB_SIZE) * sizeof(deblock_data_t)) );

  frame_info_t *frame_info = &(encoder_info->frame_info);
  uint8_t qp = frame_info->qp;

  double lambda_coeff;
  if (frame_info->frame_type == I_FRAME)
    lambda_coeff = encoder_info->params->lambda_coeffI;
  else if(frame_info->frame_type == P_FRAME)
    lambda_coeff = encoder_info->params->lambda_coeffP;
  else{
    if (frame_info->b_level==0)
      lambda_coeff = encoder_info->params->lambda_coeffB0;
    else if(frame_info->b_level == 1)
      lambda_coeff = encoder_info->params->lambda_coeffB1;
    else if (frame_info->b_level == 2)
      lambda_coeff = encoder_info->params->lambda_coeffB2;
    else if (frame_info->b_level == 3)
      lambda_coeff = encoder_info->params->lambda_coeffB3;
    else
      lambda_coeff = encoder_info->params->lambda_coeffB;
  }
  frame_info->lambda_coeff = lambda_coeff;
  frame_info->lambda = lambda_coeff*squared_lambda_QP[frame_info->qp];

  int sb_idx = 0;
  int start_bits_sb, end_bits_sb, num_bits_sb;
  int start_bits_frame=0, end_bits_frame, num_bits_frame;
  if (encoder_info->params->bitrate > 0) {
    start_bits_frame = get_bit_pos(stream);
    int max_qp = frame_info->frame_type == I_FRAME ? encoder_info->params->max_qpI : encoder_info->params->max_qp;
    int min_qp = frame_info->frame_type == I_FRAME ? encoder_info->params->min_qpI : encoder_info->params->min_qp;
    init_rate_control_per_frame(encoder_info->rc, min_qp, max_qp);
  }

  write_frame_header(stream, frame_info);

  // Initialize prev_qp to qp used in frame header
  encoder_info->frame_info.prev_qp = encoder_info->frame_info.qp;

  for (k=0;k<num_sb_ver;k++){
    for (l=0;l<num_sb_hor;l++){
      int sub = encoder_info->params->subsample != 444;
      int xposY = l*sb_size;
      int yposY = k*sb_size;
      for (int ref_idx = 0; ref_idx <= frame_info->num_ref - 1; ref_idx++){
        frame_info->mvcand_num[ref_idx] = 0;
        frame_info->mvcand_mask[ref_idx] = 0;
      }
      frame_info->best_ref = -1;

      int max_delta_qp = encoder_info->params->max_delta_qp;
      if (max_delta_qp){
        /* RDO-based search for best QP value */
        int cost,min_cost,best_qp,qp0,max_delta_qp,min_qp,max_qp;
        max_delta_qp = encoder_info->params->max_delta_qp;
        min_cost = 1<<30;
        stream_pos_t stream_pos_ref;
        read_stream_pos(&stream_pos_ref,stream);
        best_qp = qp;
        min_qp = qp-max_delta_qp;
        max_qp = qp+max_delta_qp;
        int pqp = encoder_info->frame_info.prev_qp; // Save prev_qp in local variable
        for (qp0=min_qp;qp0<=max_qp;qp0+=encoder_info->params->delta_qp_step){
          cost = TEMPLATE(process_block)(encoder_info, sb_size, yposY, xposY, qp0, sub);
          if (cost < min_cost){
            min_cost = cost;
            best_qp = qp0;
          }
        }
        encoder_info->frame_info.prev_qp = pqp; // Restore prev_qp from local variable
        write_stream_pos(stream,&stream_pos_ref);
        TEMPLATE(process_block)(encoder_info, sb_size, yposY, xposY, best_qp, sub);
      }
      else{
        if (encoder_info->params->bitrate > 0) {
          start_bits_sb = get_bit_pos(stream);
          TEMPLATE(process_block)(encoder_info, sb_size, yposY, xposY, qp, sub);
          end_bits_sb = get_bit_pos(stream);
          num_bits_sb = end_bits_sb - start_bits_sb;
          qp = update_rate_control_sb(encoder_info->rc, sb_idx, num_bits_sb, qp);
          sb_idx++;
        }
        else {
          TEMPLATE(process_block)(encoder_info, sb_size, yposY, xposY, qp, sub);
        }
      }
    }
  }

  qp = encoder_info->frame_info.qp = encoder_info->frame_info.prev_qp; //TODO: Consider using average QP instead

  //Scale and store MVs in encode_frame()
  if (encoder_info->params->interp_ref > 1) {
    int gop_size = encoder_info->params->num_reorder_pics + 1;
    int b_level = encoder_info->frame_info.b_level;
    int frame_type = encoder_info->frame_info.frame_type;
    int frame_num = encoder_info->frame_info.frame_num;
    TEMPLATE(store_mv)(width, height, b_level, frame_type, frame_num, gop_size, encoder_info->deblock_data);
  }

  if (encoder_info->params->deblocking){
    //TODO: Use QP per SB or average QP
    TEMPLATE(deblock_frame_y)(encoder_info->rec, encoder_info->deblock_data, width, height, qp, encoder_info->params->bitdepth);
    int qpc = chroma_qp[qp];
    TEMPLATE(deblock_frame_uv)(encoder_info->rec, encoder_info->deblock_data, width, height, qpc, encoder_info->params->bitdepth);
  }

  if (encoder_info->params->clpf){
    if (qp <= 16) // CLPF will have no effect if the quality is very high
      put_flc(2, 0, stream);
    else {
      int enable_sb_flag = 1;
      int fb_size_log2;
      int strength;
      // Find the best strength for the entire frame
      clpf_test_frame(encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, frame_info, &strength, &fb_size_log2, encoder_info->params->bitdepth);
      if (!fb_size_log2) { // Disable sb signal
        enable_sb_flag = 0;
        fb_size_log2 = log2i(MAX_SB_SIZE);
     }
      if (!strength)  // Better to disable for the whole frame?
        put_flc(2, 0, stream);
      else {
        // Apply the filter using the chosen strength
        yuv_frame_t tmp = *encoder_info->rec;
        put_flc(2, strength - (strength == 4), stream);
        put_flc(2, (fb_size_log2-log2i(MAX_SB_SIZE)+3)*enable_sb_flag, stream);
        TEMPLATE(clpf_frame)(encoder_info->tmp, encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, stream, enable_sb_flag, strength, fb_size_log2, encoder_info->params->bitdepth, clpf_decision);
        *encoder_info->rec = *encoder_info->tmp;
        *encoder_info->tmp = tmp;
        encoder_info->rec->frame_num = tmp.frame_num;
      }
    }
  }

  if (encoder_info->params->bitrate > 0) {
    end_bits_frame = get_bit_pos(stream);
    num_bits_frame = end_bits_frame - start_bits_frame;
    update_rate_control_per_frame(encoder_info->rc, num_bits_frame);
  }

  /* Sliding window operation for reference frame buffer by circular buffer */

  /* Store pointer to reference frame that is shifted out of reference buffer */
  yuv_frame_t *tmp = encoder_info->ref[MAX_REF_FRAMES-1];

  /* Update remaining pointers to implement sliding window reference buffer operation */
  memmove(encoder_info->ref+1, encoder_info->ref, sizeof(yuv_frame_t*)*(MAX_REF_FRAMES-1));

  /* Set ref[0] to the memory slot where the new current reconstructed frame wil replace reference frame being shifted out */
  encoder_info->ref[0] = tmp;

  /* Pad the reconstructed frame and write into ref[0] */
  TEMPLATE(create_reference_frame)(encoder_info->ref[0],encoder_info->rec);

#if 0
  /* To test sliding window operation */
  int offsetx = 500;
  int offsety = 200;
  int offset_rec = offsety * encoder_info->rec->stride_y +  offsetx;
  int offset_ref = offsety * encoder_info->ref[0]->stride_y +  offsetx;
  printf("rec: %3d ",encoder_info->rec->y[offset_rec]);
  printf("ref: ");
  for (r=0;r<MAX_REF_FRAMES;r++){
    printf("%3d ",encoder_info->ref[r]->y[offset_ref]);
  }
#endif

}


