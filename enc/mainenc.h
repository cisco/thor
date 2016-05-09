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
#include "rc.h"

typedef struct
{
  unsigned int width;
  unsigned int height;
  int log2_sb_size;
  unsigned int qp;
  char *infilestr;
  char *outfilestr;
  char *reconfilestr;
  char *statfilestr;
  unsigned int file_headerlen;
  unsigned int frame_headerlen;
  int num_frames;
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
  int delta_qp_step;
  int encoder_speed;
  int sync;
  int deblocking;
  int clpf;
  int snrcalc;
  int use_block_contexts;
  int enable_bipred;
  int bitrate;
  int max_qp;
  int min_qp;
  int max_qpI;
  int min_qpI;
  int qmtx;
  int qmtx_offset;
  int subsample;
  int aspectnum;
  int aspectden;
} enc_params;

typedef struct
{
  uint8_t y[MAX_SB_SIZE*MAX_SB_SIZE];
  uint8_t u[MAX_SB_SIZE*MAX_SB_SIZE];
  uint8_t v[MAX_SB_SIZE*MAX_SB_SIZE];
} yuv_block_t;

typedef struct
{
  block_pos_t block_pos;
  yuv_block_t *rec_block;
  yuv_block_t *org_block;
  block_param_t block_param;
  inter_pred_t skip_candidates[MAX_NUM_SKIP];
  inter_pred_t merge_candidates[MAX_NUM_MERGE];
  inter_pred_t inter_block_param[4]; //inter prediction parameters for up to 4 PB in a CB
  int num_skip_vec;
  int num_merge_vec;
  mv_t mvp;
  int tb_param;
  int max_num_pb_part;
  int max_num_tb_part;
  int delta_qp;
  block_context_t *block_context;
  int final_encode;
  yuv_block_t *rec_block_best;
  double lambda;
  int qp;
  int sub;
  int adaptive_chroma;
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
  int interp_ref;
  int b_level;
  double lambda_coeff;
  int prev_qp;
  int min_ref_dist;
} frame_info_t;

typedef struct 
{
  block_info_t *block_info;
  frame_info_t frame_info;
  enc_params *params;
  yuv_frame_t *orig;
  yuv_frame_t *rec;
  yuv_frame_t *tmp;
  yuv_frame_t *ref[MAX_REF_FRAMES];
  yuv_frame_t *interp_frames[MAX_SKIP_FRAMES];
  stream_t *stream;
  deblock_data_t *deblock_data;
  rate_control_t *rc;
  int width;
  int height;
  int depth;
  qmtx_t *wmatrix[NUM_QM_LEVELS][3][2][TR_SIZE_RANGE];
  qmtx_t *iwmatrix[NUM_QM_LEVELS][3][2][TR_SIZE_RANGE];
} encoder_info_t;

#endif
