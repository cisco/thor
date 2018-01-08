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
#include "assert.h"

// Default to low bitdepth (8 bit)
#ifndef SAMPLE
#define SAMPLE uint8_t
#define TEMPLATE(name) name ## _lbd
#endif

typedef enum {
  TILE_LEFT_BOUNDARY = 1,
  TILE_RIGHT_BOUNDARY = 2,
  TILE_ABOVE_BOUNDARY = 4,
  TILE_BOTTOM_BOUNDARY = 8,
  FRAME_LEFT_BOUNDARY = 16,
  FRAME_RIGHT_BOUNDARY = 32,
  FRAME_ABOVE_BOUNDARY = 64,
  FRAME_BOTTOM_BOUNDARY = 128,
} boundary_type;

typedef struct
{
    double y;
    double u;
    double v;
} snrvals;

typedef struct
{   
    SAMPLE *y;
    SAMPLE *u;
    SAMPLE *v;
    int width;
    int height;
    int stride_y;
    int stride_c;
    int offset_y;
    int offset_c;
    int pad_hor_y;
    int pad_hor_c;
    int pad_ver_y;
    int pad_ver_c;
    int area_y;
    int area_c;
    int sub;
    int subsample;
    int frame_num;
    int bitdepth;
    int input_bitdepth;
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
    B_FRAME,
    NUM_FRAME_TYPES
} frame_type_t;

typedef enum {
    PLANE_Y,
    PLANE_U,
    PLANE_V
} plane_t;

typedef enum {
    MODE_SKIP = 0,
    MODE_INTRA,
    MODE_INTER,
    MODE_BIPRED,
    MODE_MERGE,
    NUM_BLOCK_MODES
} block_mode_t;

// Statistics types
typedef enum {
  STAT_SKIP = 0,
  STAT_SPLIT,
  STAT_REF_IDX0,
  STAT_MERGE,
  STAT_BIPRED,
  STAT_INTRA,
  STAT_REF_IDX1,
  STAT_REF_IDX2,
  STAT_REF_IDX3
} stat_mode_t;

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
  mv_t mv0;
  mv_t mv1;
  uint32_t ref_idx0;    //TODO: Maybe use uint8_t?
  uint32_t ref_idx1;    //TODO: Maybe use uint8_t?
  uint32_t bipred_flag; //TODO: Maybe use uint8_t?
} inter_pred_t;

typedef struct
{
  int y;
  int u;
  int v;
} cbp_t;

#if CDEF
typedef struct
{
  int level;  // pri_strength, skip_condition
  int sec_strength;
  int pri_damping;
  int sec_damping;
} cdef_strength;

typedef struct
{
  int pri_strength[2];
  int skip_condition[2];
  int sec_strength[2];
} cdef_preset;

typedef struct
{
  int dir[CDEF_BLOCKSIZE * CDEF_BLOCKSIZE / 64];
  int var[CDEF_BLOCKSIZE * CDEF_BLOCKSIZE / 64];
  cdef_strength plane[2];
} cdef_strengths;
#endif

typedef struct
{
  block_mode_t mode;
  cbp_t cbp;
  uint8_t size;
  uint8_t tb_split;
  part_t pb_part;
  inter_pred_t inter_pred;
  inter_pred_t inter_pred_arr[16]; //TODO: MAX_GOP_SIZE
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
  int pb_part;
  mv_t mv_arr0[4];
  mv_t mv_arr1[4];
  int ref_idx0;
  int ref_idx1;
  int dir;
  cbp_t cbp;
  int tb_param;
  int tb_split;
  int16_t coeff_y[4*MAX_QUANT_SIZE*MAX_QUANT_SIZE]; //TODO: Not needed in decoder
  int16_t coeff_u[4*MAX_QUANT_SIZE*MAX_QUANT_SIZE];
  int16_t coeff_v[4*MAX_QUANT_SIZE*MAX_QUANT_SIZE];
} block_param_t;

typedef struct
{
  uint16_t ypos;
  uint16_t xpos;
  uint8_t size;
  uint8_t bwidth;
  uint8_t bheight;
  uint8_t sb_size;
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
  frame_type_t stat_frame_type;
  /* For bitcount */
  uint32_t sequence_header;
  uint32_t frame_header[NUM_FRAME_TYPES];
  uint32_t super_mode[NUM_FRAME_TYPES];
  uint32_t mv[NUM_FRAME_TYPES];
  uint32_t intra_mode[NUM_FRAME_TYPES];
  uint32_t skip_idx[NUM_FRAME_TYPES];
  uint32_t coeff_y[NUM_FRAME_TYPES];
  uint32_t coeff_u[NUM_FRAME_TYPES];
  uint32_t coeff_v[NUM_FRAME_TYPES];
  uint32_t cbp[NUM_FRAME_TYPES];
  uint32_t clpf[NUM_FRAME_TYPES];

  /* For statistics */
  uint32_t mode[NUM_FRAME_TYPES][NUM_BLOCK_MODES];
  uint32_t size[NUM_FRAME_TYPES][NUM_BLOCK_SIZES];
  uint32_t size_and_mode[NUM_FRAME_TYPES][NUM_BLOCK_SIZES][NUM_BLOCK_MODES];
  uint32_t frame_type[NUM_FRAME_TYPES];
  uint32_t super_mode_stat[NUM_FRAME_TYPES][NUM_BLOCK_SIZES][NUM_BLOCK_MODES+MAX_REF_FRAMES];
  uint32_t cbp_stat[NUM_FRAME_TYPES][8];
  uint32_t cbp2_stat[3][NUM_FRAME_TYPES][NUM_BLOCK_MODES-1][NUM_BLOCK_SIZES][9]; //Three contexts
  uint32_t size_and_intra_mode[NUM_FRAME_TYPES][NUM_BLOCK_SIZES][MAX_NUM_INTRA_MODES];
  uint32_t size_and_ref_idx[NUM_FRAME_TYPES][NUM_BLOCK_MODES][MAX_REF_FRAMES];
  uint32_t bi_ref[NUM_FRAME_TYPES][MAX_REF_FRAMES*MAX_REF_FRAMES];
} bit_count_t;

#endif
