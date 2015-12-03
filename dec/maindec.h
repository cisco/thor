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

#if !defined(_MAINDEC_H_)
#define _MAINDEC_H_

#include <stdio.h>
#include "getbits.h"
#include "types.h"

typedef struct 
{
  frame_type_t frame_type;
  uint8_t qp;
  uint8_t qpb; //TODO: Move to some structure at 64x64 level
  int num_ref;
  int ref_array[MAX_REF_FRAMES];
  int num_intra_modes;
  int decode_order_frame_num;
  int display_frame_num;
  int interp_ref;
} frame_info_t;

typedef struct
{
  block_pos_t block_pos;
  block_param_t block_param;
  int num_skip_vec;
  cbp_t cbp;
  int16_t *coeffq_y;
  int16_t *coeffq_u;
  int16_t *coeffq_v;
  int delta_qp;
} block_info_dec_t;

typedef struct 
{
    frame_info_t frame_info;
    yuv_frame_t *rec;
    yuv_frame_t *ref[MAX_REF_FRAMES];
    yuv_frame_t *interp_frames[MAX_SKIP_FRAMES];
    stream_t *stream;
    deblock_data_t *deblock_data;
    int width;
    int height;
    bit_count_t bit_count;
    int pb_split;
    int max_num_ref;
    int interp_ref;
    int max_delta_qp;
    int deblocking;
    int clpf;
    int tb_split_enable;
    int mode;    //TODO: Move to a block structure
    int ref_idx; //TODO: Move to a block structure
    int super_mode;
    int use_block_contexts;
    block_context_t *block_context;
    int bipred;
    int depth;
    int qmtx;
    unsigned int iwmatrix[52][3][2][TR_SIZE_RANGE][MAX_QUANT_SIZE*MAX_QUANT_SIZE];
} decoder_info_t;

#endif
