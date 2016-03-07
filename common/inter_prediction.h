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

void get_inter_prediction_chroma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign, int pic_width2, int pic_height2, int xpos, int ypos);
void get_inter_prediction_luma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign, int bipred, int pic_width, int pic_height, int xpos, int ypos);
//void get_inter_prediction_luma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign, int bipred);
//void get_inter_prediction_chroma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride, mv_t *mv, int sign);
mv_t get_mv_pred(int yposY,int xposY,int width,int height,int size,int sb_size,int ref_idx,deblock_data_t *deblock_data);
int get_mv_skip(int yposY, int xposY, int width, int height, int size, int sb_size, deblock_data_t *deblock_data, inter_pred_t *skip_candidates);
int get_mv_merge(int yposY, int xposY, int width, int height, int size, int sb_size, deblock_data_t *deblock_data, inter_pred_t *merge_candidates);
void clip_mv(mv_t *mv_cand, int ypos, int xpos, int fwidth, int fheight, int size, int sign);

#endif
