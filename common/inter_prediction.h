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

#if !defined(_INTER_PREDICTION_H_)
#define _INTER_PREDICTION_H_
#include "types.h"

void store_mv_lbd(int width, int height, int b_level, int frame_type, int frame_num, int gop_size, deblock_data_t *deblock_data);
void store_mv_hbd(int width, int height, int b_level, int frame_type, int frame_num, int gop_size, deblock_data_t *deblock_data);
int get_mv_skip_lbd(int yposY, int xposY, int width, int height, int bwidth, int bheight, int sb_size, deblock_data_t *deblock_data, inter_pred_t *skip_candidates);
int get_mv_skip_hbd(int yposY, int xposY, int width, int height, int bwidth, int bheight, int sb_size, deblock_data_t *deblock_data, inter_pred_t *skip_candidates);
int get_mv_skip_temp_lbd(int width, int phase, int gop_size, block_pos_t *block_pos, deblock_data_t *deblock_data, inter_pred_t *skip_candidates);
int get_mv_skip_temp_hbd(int width, int phase, int gop_size, block_pos_t *block_pos, deblock_data_t *deblock_data, inter_pred_t *skip_candidates);
mv_t get_mv_pred_lbd(int yposY,int xposY,int width,int height,int bwidth,int bheight,int sb_size,int ref_idx,deblock_data_t *deblock_data);
mv_t get_mv_pred_hbd(int yposY,int xposY,int width,int height,int bwidth,int bheight,int sb_size,int ref_idx,deblock_data_t *deblock_data);
int get_mv_merge_lbd(int yposY, int xposY, int width, int height, int bwidth, int bheight, int sb_size, deblock_data_t *deblock_data, inter_pred_t *merge_candidates);
int get_mv_merge_hbd(int yposY, int xposY, int width, int height, int bwidth, int bheight, int sb_size, deblock_data_t *deblock_data, inter_pred_t *merge_candidates);

void TEMPLATE(get_inter_prediction_luma)(SAMPLE *pblock, SAMPLE *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign, int bipred, int pic_width, int pic_height, int xpos, int ypos, int bitdepth);
void TEMPLATE(get_inter_prediction_temp)(int width, int height, yuv_frame_t *ref0, yuv_frame_t *ref1, block_pos_t *block_pos, deblock_data_t *deblock_data, int gop_size, int phase, SAMPLE *pblock_y, SAMPLE *pblock_u, SAMPLE *pblock_v);
void TEMPLATE(get_inter_prediction_yuv)(yuv_frame_t *ref, SAMPLE *pblock_y, SAMPLE *pblock_u, SAMPLE *pblock_v, block_pos_t *block_pos, mv_t *mv_arr, int sign, int width, int height, int enable_bipred, int split, int bitdepth);
void TEMPLATE(average_blocks_all)(SAMPLE *rec_y, SAMPLE *rec_u, SAMPLE *rec_v, SAMPLE *pblock0_y, SAMPLE *pblock0_u, SAMPLE *pblock0_v, SAMPLE *pblock1_y, SAMPLE *pblock1_u, SAMPLE *pblock1_v, block_pos_t *block_pos, int sub);

void TEMPLATE(clip_mv)(mv_t *mv_cand, int ypos, int xpos, int fwidth, int fheight, int bwidth, int bheight, int sign);

#endif
