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

#if !defined(_MAINENC_H_)
#define _MAINENC_H_

#include <stdio.h>
#include "putbits.h"
#include "types.h"

typedef struct
{
  unsigned int width;
  unsigned int height;
  unsigned int qp;
  char *infilestr;
  char *outfilestr;
  char *reconfilestr;
  char *statfilestr;
  unsigned int file_headerlen;
  unsigned int frame_headerlen;
  unsigned int num_frames;
  int skip;
  float frame_rate;
  float lambda_coeffI;
  float lambda_coeffP;
  float lambda_coeffB;
  float lambda_coeffB0;
  float lambda_coeffB1;
  float lambda_coeffB2;
  float lambda_coeffB3;
  float early_skip_thr;
  int enable_tb_split;
  int enable_pb_split;
  int max_num_ref;
  int HQperiod;
  int num_reorder_pics;
  int dyadic_coding;
  int interp_ref;
  int dqpP;
  int dqpB;
  int dqpB0;
  int dqpB1;
  int dqpB2;
  int dqpB3;
  float mqpP;
  float mqpB;
  float mqpB0;
  float mqpB1;
  float mqpB2;
  float mqpB3;
  int dqpI;
  int intra_period;
  int intra_rdo;
  int rdoq;
  int max_delta_qp;
  int encoder_speed;
  int sync;
  int deblocking;
  int clpf;
  int snrcalc;
  int use_block_contexts;
  int enable_bipred;
} enc_params;

typedef struct
{
  uint8_t y[MAX_BLOCK_SIZE*MAX_BLOCK_SIZE];
  uint8_t u[MAX_BLOCK_SIZE/2*MAX_BLOCK_SIZE/2];
  uint8_t v[MAX_BLOCK_SIZE/2*MAX_BLOCK_SIZE/2];
} yuv_block_t;

typedef struct
{
  block_pos_t block_pos;
  yuv_block_t *rec_block;
  yuv_block_t *org_block;
  pred_data_t pred_data;
  inter_pred_t skip_candidates[MAX_NUM_SKIP];
  inter_pred_t merge_candidates[MAX_NUM_MERGE];
  inter_pred_t inter_pred_data[4]; //inter prediction parameters for up to 4 PB in a CB
  int num_skip_vec;
  mvb_t mvb_skip[MAX_NUM_SKIP];
  int num_merge_vec;
  mvb_t mvb_merge[MAX_NUM_SKIP];
  mv_t mvp;
  int tb_param;
  int max_num_pb_part;
  int max_num_tb_part;
  cbp_t cbp;
  int delta_qp;
  block_context_t *block_context;
  int final_encode;
  yuv_block_t *rec_block_best;
  cbp_t cbp_best;
  int16_t coeff_y[MAX_BLOCK_SIZE*MAX_BLOCK_SIZE];
  int16_t coeff_u[MAX_BLOCK_SIZE / 2 * MAX_BLOCK_SIZE / 2];
  int16_t coeff_v[MAX_BLOCK_SIZE / 2 * MAX_BLOCK_SIZE / 2];
  int16_t coeff_y_best[MAX_BLOCK_SIZE*MAX_BLOCK_SIZE];
  int16_t coeff_u_best[MAX_BLOCK_SIZE / 2 * MAX_BLOCK_SIZE / 2];
  int16_t coeff_v_best[MAX_BLOCK_SIZE / 2 * MAX_BLOCK_SIZE / 2];
} block_info_t; //TODO: Consider merging with block_pos_t


typedef struct
{
  frame_type_t frame_type;
  uint8_t qp;
  int num_ref;
  int best_ref;
  int ref_array[MAX_REF_FRAMES];
  mv_t mvcand[MAX_REF_FRAMES][64];
  int mvcand_num[MAX_REF_FRAMES];
  uint64_t mvcand_mask[MAX_REF_FRAMES];
  double lambda;
  int num_intra_modes;
  int frame_num;
#if TEST_AVAILABILITY
  int ur[9][9];
  int dl[9][9];
#endif
  int interp_ref;
  int b_level;
} frame_info_t;

typedef struct 
{
  block_info_t *block_info;
  frame_info_t frame_info;
  enc_params *params;
  yuv_frame_t *orig;
  yuv_frame_t *rec;
  yuv_frame_t *ref[MAX_REF_FRAMES];
  yuv_frame_t *interp_frames[MAX_SKIP_FRAMES];
  stream_t *stream;
  deblock_data_t *deblock_data;
  int width;
  int height;
  int depth;
} encoder_info_t;

#endif
