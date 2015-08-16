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

#ifndef __MAINTYPES
#define __MAINTYPES

#include <stdint.h>
#include "global.h"

typedef struct
{
    double y;
    double u;
    double v;
} snrvals;

typedef struct
{   
    uint8_t *y;
    uint8_t *u;
    uint8_t *v;
    int width;
    int height;
    int stride_y;
    int stride_c;
    int offset_y;
    int offset_c;
    int frame_num;
} yuv_frame_t;

typedef enum {     // Order matters: log2(size)-2
    TR_4x4 = 0,
    TR_8x8 = 1,
    TR_16x16 = 2,
    TR_32x32 = 3,
    TR_SIZES
} trsizes;

typedef enum {
    I_FRAME,
    P_FRAME,
    B_FRAME
} frame_type_t;

typedef enum {
    MODE_SKIP = 0,
    MODE_INTRA,
    MODE_INTER,
    MODE_BIPRED,
    MODE_MERGE,
    MAX_NUM_MODES
} block_mode_t;

typedef enum {
    PART_NONE = 0,
    PART_HOR,
    PART_VER,
    PART_QUAD
} part_t;

typedef struct
{
  int16_t x;
  int16_t y;
} mv_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int32_t ref_idx;
} mvr_t;

typedef struct
{
  int16_t x0;
  int16_t y0;
  int32_t ref_idx0;
  int16_t x1;
  int16_t y1;
  int32_t ref_idx1;
  int32_t dir;
} mvb_t;

typedef struct
{
  int y;
  int u;
  int v;
} cbp_t;

typedef struct
{
  block_mode_t mode;
  cbp_t cbp;
  uint8_t size;
  mvb_t mvb;
  uint8_t tb_split;
  part_t pb_part;
} deblock_data_t;

typedef enum {
    MODE_DC = 0,
    MODE_PLANAR,
    MODE_HOR,
    MODE_VER,
    MODE_UPLEFT,
    MODE_UPRIGHT,
    MODE_UPUPRIGHT,
    MODE_UPUPLEFT,
    MODE_UPLEFTLEFT,
    MODE_DOWNLEFTLEFT,
    MAX_NUM_INTRA_MODES
} intra_mode_t;



typedef struct
{
  block_mode_t mode;
  intra_mode_t intra_mode;
  int skip_idx;
  int PBpart;
  mv_t mv_arr0[4];
  mv_t mv_arr1[4];
  int ref_idx0;
  int ref_idx1;
  int dir;
} pred_data_t;

typedef struct
{
  int mode;
  int skip_idx;
  int tb_split;
  int pb_part;
  mv_t mv[4];
} best_rdo_block_params_t;

typedef struct
{
  uint16_t ypos;
  uint16_t xpos;
  uint8_t size;
  uint8_t bwidth;
  uint8_t bheight;
} block_pos_t;

typedef struct
{
  int8_t split;
  int8_t cbp;
  int8_t mode;
  int8_t size;
  int8_t index;
} block_context_t;

typedef struct
{
  block_mode_t mode;
  intra_mode_t intra_mode;
  mv_t mvp;
  cbp_t *cbp;
  int16_t *coeffq_y;
  int16_t *coeffq_u;
  int16_t *coeffq_v;
  uint8_t size;
  int skip_idx;
  int num_skip_vec;
  mv_t mv_arr[4]; //TODO: collapse with mv_arr0
  mv_t mv_arr0[4];
  mv_t mv_arr1[4];
  int ref_idx0;
  int ref_idx1;
  int max_num_pb_part;
  int max_num_tb_part;
  int tb_part;
  int pb_part;
  int frame_type;
  int num_ref;
  int ref_idx; //TODO: collapse with ref_idx0
  int num_intra_modes;
  int delta_qp;
  int max_delta_qp;
  block_context_t *block_context;
  int enable_bipred;
  int encode_rectangular_size;
#if TWO_MVP
  int mv_idx;
#endif
} write_data_t;

typedef struct
{
  /* For bit ccount */
  uint32_t sequence_header;
  uint32_t frame_header[2];
  uint32_t super_mode[2];
  uint32_t mv[2];
  uint32_t intra_mode[2];
  uint32_t skip_idx[2];
  uint32_t coeff_y[2];
  uint32_t coeff_u[2];
  uint32_t coeff_v[2];
  uint32_t cbp[2];
  uint32_t clpf[2];

  /* For statistics */
  uint32_t mode[2][4];
  uint32_t size[2][4];
  uint32_t size_and_mode[4][4];
  uint32_t frame_type[2];
  uint32_t super_mode_stat[9][4][MAX_REF_FRAMES+8]; //split_flag,mode and ref_idx for each context(18) and block size(4)
  uint32_t cbp_stat[2][16];
  uint32_t cbp2_stat[3][2][2][4][9];
  uint32_t size_and_intra_mode[2][4][MAX_NUM_INTRA_MODES];
  uint32_t size_and_ref_idx[4][MAX_REF_FRAMES];
  uint32_t bi_ref[MAX_REF_FRAMES*MAX_REF_FRAMES];
} bit_count_t;
#endif
