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
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <memory.h>
#include <assert.h>

#include "global.h"
#include "strings.h"
#include "snr.h"
#include "mainenc.h"
#include "putbits.h"
#include "putvlc.h"
#include "transform.h"
#include "common_block.h"

extern int zigzag16[16];
extern int zigzag64[64];
extern int zigzag256[256];
extern int YPOS,XPOS;

void write_sequence_header(stream_t *stream, enc_params *params) {
  put_flc(16, params->width, stream);
  put_flc(16, params->height, stream);
  put_flc(3, params->log2_sb_size, stream);
  put_flc(1, params->enable_pb_split, stream);
  put_flc(1, params->enable_tb_split, stream);
  put_flc(2, params->max_num_ref - 1, stream); //TODO: Support more than 4 reference frames
  put_flc(2, params->interp_ref, stream);// Use an interpolated reference frame
  put_flc(1, (params->max_delta_qp || params->bitrate), stream);
  put_flc(1, params->deblocking, stream);
  put_flc(1, params->clpf ? 1 : 0, stream);
  put_flc(1, params->use_block_contexts, stream);
  put_flc(2, params->enable_bipred ,stream);
  put_flc(1, params->qmtx, stream);
  if (params->qmtx)
    put_flc(6, params->qmtx_offset + 32, stream);
  put_flc(1, params->subsample == 420, stream);
  put_flc(4, params->num_reorder_pics, stream);
  put_flc(1, params->cfl_intra, stream);
  put_flc(1, params->cfl_inter, stream);
  put_flc(1, params->bitdepth != 8, stream);
  if (params->bitdepth != 8)
    put_flc(1, params->bitdepth == 12, stream);
  put_flc(1, params->input_bitdepth != 8, stream);
  if (params->input_bitdepth != 8)
    put_flc(1, params->input_bitdepth == 12, stream);
}

void write_frame_header(stream_t *stream, frame_info_t *frame_info) {

  put_flc(1, frame_info->frame_type != I_FRAME, stream);
  put_flc(8, (int)frame_info->qp, stream);
  put_flc(4, (int)frame_info->num_intra_modes, stream);

  // Signal actual number of reference frames
  if (frame_info->frame_type != I_FRAME)
    put_flc(2, frame_info->num_ref - 1, stream);

  int r;
  for (r = 0; r < frame_info->num_ref; r++) {
    put_flc(6, frame_info->ref_array[r] + 1, stream);
  }
  // 16 bit frame number for now
  put_flc(16, frame_info->frame_num, stream);
}

void write_mv(stream_t *stream,mv_t *mv,mv_t *mvp)
{
    uint16_t  mvabs,mvsign;
    mv_t mvd;
    mvd.x = mv->x - mvp->x;
    mvd.y = mv->y - mvp->y;

    /* MVX */
    mvabs = abs(mvd.x);
    mvsign = mvd.x < 0 ? 1 : 0;
    put_vlc(7, mvabs, stream);
    if (mvabs>0)
      put_flc(1, mvsign, stream);

    /* MVY */
    mvabs = abs(mvd.y);
    mvsign = mvd.y < 0 ? 1 : 0;
    put_vlc(7, mvabs, stream);
    if (mvabs>0)
      put_flc(1, mvsign, stream);
}

void write_coeff(stream_t *stream,int16_t *coeff,int size,int type)
{
  int16_t scoeff[MAX_QUANT_SIZE*MAX_QUANT_SIZE];
  int i,j,pos,c;
  int qsize = min(MAX_QUANT_SIZE,size);
  unsigned int cn;
  int level,sign,last_pos;
  int run;
  int N = qsize*qsize;
  int level_mode;
  int chroma_flag = type&1;
  int intra_flag = (type>>1)&1;
  int vlc_adaptive = intra_flag && !chroma_flag;
  unsigned int eob_pos = chroma_flag ? 0 : 2;
  static int *zigzag[] = { 0, 0, zigzag16, zigzag64, zigzag256 };
  int *zigzagptr = zigzag[log2i(qsize)];
  int runs = 0;

  /* Zigzag scan */
  for (i = 0; i < qsize; i++)
    for (j = 0; j < qsize; j++)
      scoeff[zigzagptr[i*qsize + j]] = coeff[i*qsize + j];

  /* Find last_pos to determine when to send EOB */
  for (pos = N-1; !scoeff[pos] && pos; pos--);
  if (!pos && !scoeff[0])
    fatalerror("No coeffs even if cbp nonzero. Exiting.");
  last_pos = pos;

  /* Use one bit to signal chroma/last_pos=1/level=1 */
  pos = 0;
  if (chroma_flag) {
    if (last_pos==0 && abs(scoeff[0])==1){
      put_flc(2, 2 + (scoeff[0] < 0), stream);
      pos = N;
    } else
      put_flc(1, 0, stream);
  }

  /* Initiate forward scan */
  level_mode = level = 1;

  while (pos <= last_pos) { //Outer loop for forward scan
    if (level_mode) {
      /* Level-mode */
      while (pos <= last_pos && level > 0){
        c = scoeff[pos++];
        level = abs(c);
        put_vlc(vlc_adaptive,level,stream);
        if (level > 0)
          put_flc(1,c < 0,stream);

        if (chroma_flag==0)
          vlc_adaptive = level > 3;
      }
    }

    /* Run-mode (run-level coding) */
    run = c = 0;
    while (c==0 && pos <= last_pos) {
      c = scoeff[pos++];
      run += !c;
      if (c) {
        int interval = 5;
        level = abs(c);
        sign = c < 0;

        /* Code combined event of run and (level>1) */
        if (level == 1)
          cn = (run * interval) / (interval - 1);
        else
          cn = run * interval + interval - 1;
        put_vlc(chroma_flag && size <= 8 ? 10 : 6, cn + (cn >= eob_pos), stream);
        level_mode = level > 1; //Set level_mode

        /* Code level and sign */
        if (level > 1)
          put_vlc(0, (level-2)*2+sign, stream);
        else
          put_flc(1,sign,stream);
        run = 0;
        runs++;
      }
    } //while (c==0 && pos < last_pos)
  } //while (pos <= last_pos){

  /* If terminated in level mode, code one extra zero before an EOB can be sent */
  if (pos < N && level_mode){
    put_vlc(vlc_adaptive, 0, stream);
    pos++;
  }

  /* EOB */
  if (pos < N)
    put_vlc(chroma_flag && size <= 8 ? 10 : 6, eob_pos, stream);
}

int write_delta_qp(stream_t *stream, int delta_qp){
  int len;
  int abs_delta_qp = abs(delta_qp);
  int sign_delta_qp = delta_qp < 0 ? 1 : 0;
  len = put_vlc(0,abs_delta_qp,stream);
  if (abs_delta_qp > 0){
    put_flc(1,sign_delta_qp,stream);
    len += 1;
  }
  return len;
}


void write_super_mode(stream_t *stream,encoder_info_t *encoder_info, block_info_t *block_info, block_param_t *block_param,int split_flag,int encode_this_size){

  int size = block_info->block_pos.size;
  block_mode_t mode = block_param->mode;
  frame_type_t frame_type = encoder_info->frame_info.frame_type;
  if (frame_type!=I_FRAME){

    if (!encode_this_size) {
      put_flc(1,!split_flag,stream); //Flag to signal either split or rectangular skip
      return;
    }
    int code = 0, maxbit;
    int bipred_possible_flag = encoder_info->frame_info.num_ref > 1 && encoder_info->params->enable_bipred;
    int split_possible_flag = size > MIN_BLOCK_SIZE;
    int interp_ref = encoder_info->frame_info.interp_ref;
    maxbit = 2 + encoder_info->frame_info.num_ref + split_possible_flag + bipred_possible_flag;

    if (interp_ref > 2) {
      maxbit -= 1; //ref_idx = 0 is disallowed
    }

    if (split_flag == 1) {
      if (size > MAX_TR_SIZE) {
        put_flc(1, 0, stream);
      }
      else {
        int code = 1;
        if (block_info->block_context->index == 2 || block_info->block_context->index>3)
          code = (code + 3) % 4;
        put_vlc(10 + maxbit, code, stream);
      }
      return;
    }

    if (interp_ref) {
      if (mode==MODE_SKIP)
        code = 0;
      else if (mode==MODE_MERGE)
        code = 2;
      else if (mode == MODE_BIPRED)
        code = 3;
      else if (mode == MODE_INTRA)
        code = 4;
      else if (mode == MODE_INTER && block_param->ref_idx0 > 0)
        code = 4 + block_param->ref_idx0;
      else {
        assert(mode == MODE_INTER && block_param->ref_idx0 == 0);
        code = 4 + encoder_info->frame_info.num_ref;
      }
      if (!bipred_possible_flag && code > 3) {
        /* Don't need a codeword for bipred so fill the empty slot */
        code = code - 1;
      }

      if (!split_possible_flag && code > 1) {
        /* Don't need a codeword for split so fill the empty slot */
        code = code - 1;
      }
      if ((block_info->block_context->index == 2 || block_info->block_context->index>3) && size>MIN_BLOCK_SIZE) {
        /* Skip is less likely than split, merge and inter-ref_idx=0 so move skip down the list */
        if (code<3)
          code = (code+2)%3;
      }

    } else {
      if (mode==MODE_SKIP)
        code = 0;
      else if (mode == MODE_INTER && block_param->ref_idx0 == 0)
        code = 2;
      else if (mode==MODE_MERGE)
        code = 3;
      else if (mode == MODE_BIPRED)
        code = 4;
      else if (mode == MODE_INTRA)
        code = 5;
      else if (mode == MODE_INTER && block_param->ref_idx0>0)
        code = 5 + block_param->ref_idx0;

      if (!bipred_possible_flag && code > 4) {
        /* Don't need a codeword for bipred so fill the empty slot */
        code = code - 1;
      }

      if (!split_possible_flag && code > 1) {
        /* Don't need a codeword for split so fill the empty slot */
        code = code - 1;
      }
      if ((block_info->block_context->index == 2 || block_info->block_context->index>3) && size>MIN_BLOCK_SIZE) {
        /* Skip is less likely than split, merge and inter-ref_idx=0 so move skip down the list */
        if (code<4)
          code = (code+3)%4;
      }
    }

    put_vlc(10 + maxbit, code, stream);
  }
  else{
    /* Split flag = 0 */
    if (encode_this_size && (size > MIN_BLOCK_SIZE || split_flag==1))
      put_flc(1,split_flag,stream);
  }
}

int write_block(stream_t *stream,encoder_info_t *encoder_info, block_info_t *block_info, block_param_t *block_param){

  int start_bits,end_bits;

  int size = block_info->block_pos.size;
  int tb_split = block_param->tb_split;

  uint8_t cbp_y = block_param->cbp.y;
  uint8_t cbp_u = block_param->cbp.u;
  uint8_t cbp_v = block_param->cbp.v;
  int16_t *coeffq_y = block_param->coeff_y;
  int16_t *coeffq_u = block_param->coeff_u;
  int16_t *coeffq_v = block_param->coeff_v;
  block_mode_t mode = block_param->mode;
  intra_mode_t intra_mode = block_param->intra_mode;
  mv_t mvp = block_info->mvp;
  int coeff_type = (mode == MODE_INTRA) << 1;
  int size_uv = size >> block_info->sub;

  start_bits = get_bit_pos(stream);

  int code,cbp;
  int cbp_table[8] = {1,0,5,2,6,3,7,4};

  /* Write mode and ref_idx */
  int split_flag = 0;
  int encode_this_size =
    block_info->block_pos.ypos + size <= encoder_info->height &&
    block_info->block_pos.xpos + size <= encoder_info->width;
  write_super_mode(stream, encoder_info, block_info, block_param, split_flag, encode_this_size);

  if (size == (1<< encoder_info->params->log2_sb_size) && mode != MODE_SKIP && (encoder_info->params->max_delta_qp || encoder_info->params->bitrate)) {
    write_delta_qp(stream, block_info->delta_qp);
  }
  /* Code intra mode */
  if (mode==MODE_INTRA){
    if (encoder_info->frame_info.num_intra_modes <= 4) {
      put_flc(2,intra_mode,stream);
    }
    else {
      put_vlc(8, intra_mode, stream);
    }
  }
  else if (mode==MODE_INTER){
    /* Code PU partitions */
    if (block_info->max_num_pb_part > 1) {
      put_vlc(13, block_param->pb_part, stream);
    }
    /* Code motion vectors for each prediction block */
    mv_t mvp2 = mvp;
    if (block_param->pb_part == PART_NONE) { //NONE
      write_mv(stream, &block_param->mv_arr0[0], &mvp2);
    }
    else if (block_param->pb_part == PART_HOR) { //HOR
      write_mv(stream, &block_param->mv_arr0[0], &mvp2);
      mvp2 = block_param->mv_arr0[0];
      write_mv(stream, &block_param->mv_arr0[2], &mvp2);
    }
    else if (block_param->pb_part == PART_VER) { //VER
      write_mv(stream, &block_param->mv_arr0[0], &mvp2);
      mvp2 = block_param->mv_arr0[0];
      write_mv(stream, &block_param->mv_arr0[1], &mvp2);
    }
    else {
      write_mv(stream, &block_param->mv_arr0[0], &mvp2);
      mvp2 = block_param->mv_arr0[0];
      write_mv(stream, &block_param->mv_arr0[1], &mvp2);
      write_mv(stream, &block_param->mv_arr0[2], &mvp2);
      write_mv(stream, &block_param->mv_arr0[3], &mvp2);
    }
  }
  else if (mode==MODE_BIPRED){
#if BIPRED_PART
    /* Code PU partitions */
    if (block_info->max_num_pb_part > 1) {
      put_vlc(13, block_param->pb_part, stream);
    }
#endif

    /* Code motion vectors for each prediction block */
    mv_t mvp2 = mvp;
    if (block_param->pb_part == PART_NONE) { //NONE
      write_mv(stream, &block_param->mv_arr0[0], &mvp2);
    }
    if (encoder_info->frame_info.frame_type == B_FRAME)
      mvp2 = block_param->mv_arr0[0];
    if (block_param->pb_part == PART_NONE) { //NONE
      write_mv(stream, &block_param->mv_arr1[0], &mvp2);
    }
    else if (block_param->pb_part == PART_HOR) { //HOR
      write_mv(stream, &block_param->mv_arr1[0], &mvp2);
      mvp2 = block_param->mv_arr1[0];
      write_mv(stream, &block_param->mv_arr1[2], &mvp2);
    }
    else if (block_param->pb_part == PART_VER) { //VER
      write_mv(stream, &block_param->mv_arr1[0], &mvp2);
      mvp2 = block_param->mv_arr1[0];
      write_mv(stream, &block_param->mv_arr1[1], &mvp2);
    }
    else {
      write_mv(stream, &block_param->mv_arr1[0], &mvp2);
      mvp2 = block_param->mv_arr1[0];
      write_mv(stream, &block_param->mv_arr1[1], &mvp2);
      write_mv(stream, &block_param->mv_arr1[2], &mvp2);
      write_mv(stream, &block_param->mv_arr1[3], &mvp2);
    }

    if (encoder_info->frame_info.frame_type == P_FRAME) {
      if (encoder_info->frame_info.num_ref==2) {
        put_vlc(13, 2 * block_param->ref_idx0 + block_param->ref_idx1, stream);
      }
      else {
        int code = 4 * block_param->ref_idx0 + block_param->ref_idx1; //TODO: Optimize for num_ref != 4
        put_vlc(10, code, stream);
      }
    }
  }
  else if (mode==MODE_SKIP){
    /* Code skip_idx */
    if (block_info->num_skip_vec == 4)
      put_flc(2, block_param->skip_idx, stream);
    else if (block_info->num_skip_vec == 3) {
      put_vlc(12, block_param->skip_idx, stream);
    }
    else if (block_info->num_skip_vec == 2) {
      put_flc(1, block_param->skip_idx, stream);
    }
  }
  else if (mode==MODE_MERGE){
    /* Code skip_idx */
    if (block_info->num_merge_vec == 4)
      put_flc(2, block_param->skip_idx, stream);
    else if (block_info->num_merge_vec == 3) {
      put_vlc(12, block_param->skip_idx, stream);
    }
    else if (block_info->num_merge_vec == 2) {
      put_flc(1, block_param->skip_idx, stream);
    }
  }

  if (mode != MODE_SKIP){
    int off = mode == MODE_MERGE ? 1 : 2;
    int max_num_tb_part = block_info->max_num_tb_part;
    if (max_num_tb_part > 1 && tb_split) {
      code = off;
    }
    else {
      cbp = cbp_y + (cbp_u << 1) + (cbp_v << 2);
      code = cbp_table[cbp];
      if (mode == MODE_MERGE) {
        if (code == 1)
          code = 7;
        else if (code>1)
          code = code - 1;
      }
      else {
        if (block_info->block_context->cbp == 0 && code < 2)
          code = 1 - code;
      }
      if (max_num_tb_part > 1 && code >= off) code++;
    }

    put_vlc(0,code,stream);

    if (tb_split==0){
      if (cbp_y){
        write_coeff(stream,coeffq_y,size,coeff_type|0);
      }
      if (cbp_u){
        write_coeff(stream,coeffq_u,size_uv,coeff_type|1);
      }
      if (cbp_v){
        write_coeff(stream,coeffq_v,size_uv,coeff_type|1);
      }
    }
    else{
      if (size_uv > 4){
        int index;
        for (index=0;index<4;index++){
          cbp_y = ((block_param->cbp.y) >> (3 - index)) & 1;
          cbp_u = ((block_param->cbp.u) >> (3 - index)) & 1;
          cbp_v = ((block_param->cbp.v) >> (3 - index)) & 1;
          /* Code cbp separately for each TU */
          int cbp = cbp_y + (cbp_u<<1) + (cbp_v<<2);
          code = cbp_table[cbp];
          if (block_info->block_context->cbp == 0 && code < 2)
            code = 1-code;
          put_vlc(0,code,stream);

          /* Code coefficients for each TU separately */
          coeffq_y = block_param->coeff_y + index*MAX_QUANT_SIZE*MAX_QUANT_SIZE; //TODO: Pack better when tb_split
          coeffq_u = block_param->coeff_u + index*MAX_QUANT_SIZE*MAX_QUANT_SIZE;
          coeffq_v = block_param->coeff_v + index*MAX_QUANT_SIZE*MAX_QUANT_SIZE;

          if (cbp_y){
            write_coeff(stream,coeffq_y,size/2,coeff_type|0);
          }
          if (cbp_u){
            write_coeff(stream,coeffq_u,size_uv/2,coeff_type|1);
          }
          if (cbp_v){
            write_coeff(stream,coeffq_v,size_uv/2,coeff_type|1);
          }
        } //for index=
      } //if (size > 8)
      else{
        int index;
        for (index=0;index<4;index++){
          cbp_y = ((block_param->cbp.y) >> (3 - index)) & 1;
          /* Code cbp_y separately for each TU */
          put_flc(1,cbp_y,stream);

          /* Code coefficients for each TU separately */
          coeffq_y = block_param->coeff_y + index*MAX_QUANT_SIZE*MAX_QUANT_SIZE; //TODO: Pack better when tb_split
          if (cbp_y){
            write_coeff(stream,coeffq_y,size/2,coeff_type|0);
          }
        }
        cbp = cbp_u + 2*cbp_v;
        put_vlc(13, cbp, stream);
        if (cbp_u){
          write_coeff(stream,coeffq_u,size_uv,coeff_type|1);
        }
        if (cbp_v){
          write_coeff(stream,coeffq_v,size_uv,coeff_type|1);
        }
      } //if (size > 8)
    } //if (tb_split==0
  } //if (mode!= MODE_SKIP)

  end_bits = get_bit_pos(stream);

  return (end_bits - start_bits);
}
