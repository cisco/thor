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
extern int super_table[8][20];
extern int YPOS,XPOS;

void write_mv(stream_t *stream,mv_t *mv,mv_t *mvp)
{
    uint16_t  mvabs,mvsign;
    mv_t mvd;
    mvd.x = mv->x - mvp->x;
    mvd.y = mv->y - mvp->y;

    int code;

    mvabs = abs(mvd.x);
    mvsign = mvd.x < 0 ? 1 : 0;
    code = 2*mvabs - mvsign;
    put_vlc(10,code,stream);

    mvabs = abs(mvd.y);
    mvsign = mvd.y < 0 ? 1 : 0;
    code = 2*mvabs - mvsign;
    put_vlc(10,code,stream);

}

int find_code(int run, int level, int maxrun, int chroma_flag,int eob){

  int cn,index;
  int maxrun2 = max(4,maxrun);
  index = run + (level>1)*(maxrun2+1);

  if (chroma_flag){
    if (eob)
      cn = 0;
    else if (index<=4)
      cn = index + 1;
    else if (index<=maxrun2)
      cn = index + 3;
    else if (index==(maxrun2+1))
      cn = 6;
    else if (index==(maxrun2+2))
      cn = 7;
    else
      cn = index+1;
  }
  else{
    if (eob)
      cn = 2;
    else if (index<2)
      cn = index;
    else if (index<=4)
      cn = index + 1;
    else if (index<=maxrun2)
      cn = index + 3;
    else if (index==(maxrun2+1))
      cn = 6;
    else if (index==(maxrun2+2))
      cn = 7;
    else
      cn = index+1;
  }
  return cn;
}

void write_coeff(stream_t *stream,int16_t *coeff,int size,int type)
{
  int16_t scoeff[MAX_QUANT_SIZE*MAX_QUANT_SIZE];
  int i,j,len,pos,c;
  int qsize = min(MAX_QUANT_SIZE,size);
  unsigned int cn;
  int level,vlc,sign,last_pos;
  int maxrun,run;
  int N = qsize*qsize;
  int level_mode;
  int chroma_flag = type&1;
  int intra_flag = (type>>1)&1;
  int vlc_adaptive = intra_flag && !chroma_flag;

  /* Zigzag scan */
  int *zigzagptr = zigzag64;
  if (qsize==4)
    zigzagptr = zigzag16;
  else if (qsize==8)
    zigzagptr = zigzag64;
  else if (qsize==16)
    zigzagptr = zigzag256;

  for(i=0;i<qsize;i++){
    for (j=0;j<qsize;j++){
      scoeff[zigzagptr[i*qsize+j]] = coeff[i*size+j];
    }
  }

  /* Find last_pos to determine when to send EOB */
  pos = N-1;
  while (scoeff[pos]==0 && pos>0) pos--;
  if (pos==0 && scoeff[0]==0)
    fatalerror("No coeffs even if cbp nonzero. Exiting.");
  last_pos = pos;

  /* Use one bit to signal chroma/last_pos=1/level=1 */
  pos = 0;
  if (chroma_flag){
    if (last_pos==0 && abs(scoeff[0])==1){
      putbits(1,1,stream);
      sign = (scoeff[0] < 0) ? 1 : 0;
      putbits(1,sign,stream);
      pos = N;
    }
    else{
      putbits(1,0,stream);
    }
  }

  /* Initiate forward scan */
  level_mode = 1;
  level = 1;
  while (pos <= last_pos){ //Outer loop for forward scan
    if (level_mode){
      /* Level-mode */
      while (pos <= last_pos && level > 0){
        c = scoeff[pos];
        level = abs(c);
        len = put_vlc(vlc_adaptive,level,stream);
        if (level > 0){
          sign = (c < 0) ? 1 : 0;
          putbits(1,sign,stream);
          len += 1;
        }
        if (chroma_flag==0)
          vlc_adaptive = level > 3;
        pos++;
      }
    }

    /* Run-mode (run-level coding) */
    maxrun = N - pos - 1;
    run = 0;
    c = 0;
    while (c==0 && pos <= last_pos){
      c = scoeff[pos];
      if (c==0){
        run++;
      }
      else{
        level = abs(c);
        sign = (c < 0) ? 1 : 0;

        /* Code combined event of run and (level>1) */
        cn = find_code(run, level, maxrun, chroma_flag, 0);

        if (chroma_flag && size <= 8){
          vlc = 10;
          len = put_vlc(vlc,cn,stream);
        }
        else{
          vlc = 2;
          if (cn == 0)
            putbits(2,2,stream);
          else
            put_vlc(vlc,cn+1,stream);
        }
        /* Code level and sign */
        if (level > 1){
          len += put_vlc(0,2*(level-2)+sign,stream);
        }
        else{
          putbits(1,sign,stream);
          len += 1;
        }
        run = 0;
      }
      pos++;
      level_mode = level > 1; //Set level_mode
    } //while (c==0 && pos < last_pos)
  } //while (pos <= last_pos){

  if (pos < N){
    /* If terminated in level mode, code one extra zero before an EOB can be sent */
    if (level_mode){
      c = scoeff[pos];
      level = abs(c);
      len = put_vlc(vlc_adaptive,level,stream);
      if (level > 0){
        sign = (c < 0) ? 1 : 0;
        putbits(1,sign,stream);
        len += 1;
      }
      pos++;
    }
  }

  /* EOB */
  if (pos < N){
    cn = find_code(0, 0, 0, chroma_flag, 1);
    if (chroma_flag && size <= 8){
      vlc = 0;
      put_vlc(vlc,cn,stream);
    }
    else{
      vlc = 2;
      if (cn == 0)
        putbits(2,2,stream);
      else
        put_vlc(vlc,cn+1,stream);
    }
  }
}

int write_delta_qp(stream_t *stream, int delta_qp){
  int len;
  int abs_delta_qp = abs(delta_qp);
  int sign_delta_qp = delta_qp < 0 ? 1 : 0;
  len = put_vlc(0,abs_delta_qp,stream);
  if (abs_delta_qp > 0){
    putbits(1,sign_delta_qp,stream);
    len += 1;
  }
  return len;
}


void write_super_mode(stream_t *stream,encoder_info_t *encoder_info, block_info_t *block_info, pred_data_t *pred_data,int split_flag){

  int size = block_info->block_pos.size;
  block_mode_t mode = pred_data->mode;
  frame_type_t frame_type = encoder_info->frame_info.frame_type;
  if (frame_type!=I_FRAME){
    if (split_flag == 1) {
      if (size > MAX_TR_SIZE) {
        putbits(1, 0, stream);
      }
      else {
        int code = 1;
        if (block_info->block_context->index == 2 || block_info->block_context->index>3)
          code = (code + 3) % 4;
        putbits(code + 1, 1, stream);
      }
      return;
    }

    int code = 0, maxbit;
    int bipred_possible_flag = encoder_info->frame_info.num_ref > 1 && encoder_info->params->enable_bipred;
    int split_possible_flag = size > MIN_BLOCK_SIZE;
    int interp_ref = encoder_info->frame_info.interp_ref;
    maxbit = 2 + encoder_info->frame_info.num_ref + split_possible_flag + bipred_possible_flag;
    if (interp_ref) {
      if (mode==MODE_SKIP)
        code = 0;
      else if (mode==MODE_MERGE)
        code = 2;
      else if (mode == MODE_BIPRED)
        code = 3;
      else if (mode == MODE_INTRA)
        code = 4;
      else if (mode == MODE_INTER && pred_data->ref_idx0 > 0)
        code = 4 + pred_data->ref_idx0;
      else {
        assert(mode == MODE_INTER && pred_data->ref_idx0 == 0);
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
      else if (mode == MODE_INTER && pred_data->ref_idx0 == 0)
        code = 2;
      else if (mode==MODE_MERGE)
        code = 3;
      else if (mode == MODE_BIPRED)
        code = 4;
      else if (mode == MODE_INTRA)
        code = 5;
      else if (mode == MODE_INTER && pred_data->ref_idx0>0)
        code = 5 + pred_data->ref_idx0;

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

    if (code==maxbit)
      putbits(maxbit,0,stream);
    else
      putbits(code+1,1,stream);
  }
  else{
    /* Split flag = 0 */
    if (size > MIN_BLOCK_SIZE || split_flag==1)
      putbits(1,split_flag,stream);
  }
}

int write_block(stream_t *stream,encoder_info_t *encoder_info, block_info_t *block_info, pred_data_t *pred_data){

  int start_bits,end_bits;

  int size = block_info->block_pos.size;
  int tb_split = pred_data->tb_split;

  uint8_t cbp_y = pred_data->cbp.y;
  uint8_t cbp_u = pred_data->cbp.u;
  uint8_t cbp_v = pred_data->cbp.v;
  int16_t *coeffq_y = block_info->coeff_y;
  int16_t *coeffq_u = block_info->coeff_u;
  int16_t *coeffq_v = block_info->coeff_v;
  block_mode_t mode = pred_data->mode;
  intra_mode_t intra_mode = pred_data->intra_mode;
  mv_t mvp = block_info->mvp;
  int coeff_type = (mode == MODE_INTRA) << 1;

  start_bits = get_bit_pos(stream);

  int code,cbp;
  int cbp_table[8] = {1,0,5,2,6,3,7,4};


  /* Write mode and ref_idx */
  int split_flag = 0;
  write_super_mode(stream, encoder_info, block_info, pred_data, split_flag);

  if (size == MAX_BLOCK_SIZE && mode != MODE_SKIP && encoder_info->params->max_delta_qp) {
    write_delta_qp(stream, encoder_info->params->max_delta_qp);
  }
  /* Code intra mode */
  if (mode==MODE_INTRA){
    if (encoder_info->frame_info.num_intra_modes <= 4) {
      putbits(2,intra_mode,stream);
    }
    else if (encoder_info->frame_info.num_intra_modes <= 8) {
      int intra_mode_map[MAX_NUM_INTRA_MODES] = {2,8,1,0,5,9,7,6,4,3};
      int len[8] = {2,2,2,4,4,4,5,5};
      int codeword[8] = {0,1,2,12,13,14,30,31};
      int code = intra_mode_map[intra_mode];
      assert(code<8);
      putbits(len[code], codeword[code], stream);
    }
    else if (encoder_info->frame_info.num_intra_modes <= MAX_NUM_INTRA_MODES) {
      int intra_mode_map[MAX_NUM_INTRA_MODES] = {2,3,1,0,6,9,8,7,5,4};
      int len[MAX_NUM_INTRA_MODES] = {2,2,3,3,4,4,5,5,5,5};
      int codeword[MAX_NUM_INTRA_MODES] = {2,3,2,3,2,3,0,1,2,3};
      int code = intra_mode_map[intra_mode];
      assert(code<MAX_NUM_INTRA_MODES);
      putbits(len[code], codeword[code], stream);
    }
  }
  else if (mode==MODE_INTER){
    /* Code PU partitions */
    if (block_info->max_num_pb_part > 1) {
      if (pred_data->pb_part == 0)
        putbits(1, 1, stream);
      else if (pred_data->pb_part == 1)
        putbits(2, 1, stream);
      else if (pred_data->pb_part == 2)
        putbits(3, 1, stream);
      else if (pred_data->pb_part == 3)
        putbits(3, 0, stream);
    }
    /* Code motion vectors for each prediction block */
    mv_t mvp2 = mvp;
    if (pred_data->pb_part == PART_NONE) { //NONE
      write_mv(stream, &pred_data->mv_arr0[0], &mvp2);
    }
    else if (pred_data->pb_part == PART_HOR) { //HOR
      write_mv(stream, &pred_data->mv_arr0[0], &mvp2);
      mvp2 = pred_data->mv_arr0[0];
      write_mv(stream, &pred_data->mv_arr0[2], &mvp2);
    }
    else if (pred_data->pb_part == PART_VER) { //VER
      write_mv(stream, &pred_data->mv_arr0[0], &mvp2);
      mvp2 = pred_data->mv_arr0[0];
      write_mv(stream, &pred_data->mv_arr0[1], &mvp2);
    }
    else {
      write_mv(stream, &pred_data->mv_arr0[0], &mvp2);
      mvp2 = pred_data->mv_arr0[0];
      write_mv(stream, &pred_data->mv_arr0[1], &mvp2);
      write_mv(stream, &pred_data->mv_arr0[2], &mvp2);
      write_mv(stream, &pred_data->mv_arr0[3], &mvp2);
    }
  }
  else if (mode==MODE_BIPRED){
#if BIPRED_PART
    /* Code PU partitions */
    if (block_info->max_num_pb_part > 1) {
      if (pred_data->pb_part == 0)
        putbits(1, 1, stream);
      else if (pred_data->pb_part == 1)
        putbits(2, 1, stream);
      else if (pred_data->pb_part == 2)
        putbits(3, 1, stream);
      else if (pred_data->pb_part == 3)
        putbits(3, 0, stream);
    }
#endif

    /* Code motion vectors for each prediction block */
    mv_t mvp2 = mvp;
    if (pred_data->pb_part == PART_NONE) { //NONE
      write_mv(stream, &pred_data->mv_arr0[0], &mvp2);
    }
    if (encoder_info->frame_info.frame_type == B_FRAME)
      mvp2 = pred_data->mv_arr0[0];
    if (pred_data->pb_part == PART_NONE) { //NONE
      write_mv(stream, &pred_data->mv_arr1[0], &mvp2);
    }
    else if (pred_data->pb_part == PART_HOR) { //HOR
      write_mv(stream, &pred_data->mv_arr1[0], &mvp2);
      mvp2 = pred_data->mv_arr1[0];
      write_mv(stream, &pred_data->mv_arr1[2], &mvp2);
    }
    else if (pred_data->pb_part == PART_VER) { //VER
      write_mv(stream, &pred_data->mv_arr1[0], &mvp2);
      mvp2 = pred_data->mv_arr1[0];
      write_mv(stream, &pred_data->mv_arr1[1], &mvp2);
    }
    else {
      write_mv(stream, &pred_data->mv_arr1[0], &mvp2);
      mvp2 = pred_data->mv_arr1[0];
      write_mv(stream, &pred_data->mv_arr1[1], &mvp2);
      write_mv(stream, &pred_data->mv_arr1[2], &mvp2);
      write_mv(stream, &pred_data->mv_arr1[3], &mvp2);
    }

    if (encoder_info->frame_info.frame_type == P_FRAME) {
      if (encoder_info->frame_info.num_ref==2) {
        int code = 2 * pred_data->ref_idx0 + pred_data->ref_idx1;
        if (code==3)
          putbits(3,0,stream);
        else
          putbits(code+1,1,stream);
      }
      else {
        int code = 4 * pred_data->ref_idx0 + pred_data->ref_idx1; //TODO: Optimize for num_ref != 4
        put_vlc(10, code, stream);
      }
    }
  }
  else if (mode==MODE_SKIP){
    /* Code skip_idx */
    if (block_info->num_skip_vec == 4)
      putbits(2, pred_data->skip_idx, stream);
    else if (block_info->num_skip_vec == 3) {
      if (pred_data->skip_idx == 0) putbits(1, 1, stream);
      else if (pred_data->skip_idx == 1) putbits(2, 0, stream);
      else putbits(2, 1, stream);
    }
    else if (block_info->num_skip_vec == 2) {
      putbits(1, pred_data->skip_idx, stream);
    }
  }
  else if (mode==MODE_MERGE){
    /* Code skip_idx */
    if (block_info->num_merge_vec == 4)
      putbits(2, pred_data->skip_idx, stream);
    else if (block_info->num_merge_vec == 3) {
      if (pred_data->skip_idx == 0) putbits(1, 1, stream);
      else if (pred_data->skip_idx == 1) putbits(2, 0, stream);
      else putbits(2, 1, stream);
    }
    else if (block_info->num_merge_vec == 2) {
      putbits(1, pred_data->skip_idx, stream);
    }
  }

  if (mode != MODE_SKIP){
    int max_num_tb_part = 1;
    if (mode == MODE_MERGE || mode == MODE_BIPRED)
      max_num_tb_part = 1;
    else if (mode == MODE_INTER)
      max_num_tb_part = block_info->max_num_tb_part > 1 ? 2 : 1;
    else if (mode == MODE_INTRA)
      max_num_tb_part = block_info->max_num_tb_part;
    if (max_num_tb_part>1) {
      if (tb_split) {
        code = 2;
      }
      else {
        cbp = cbp_y + (cbp_u << 1) + (cbp_v << 2);
        code = cbp_table[cbp];
        if (block_info->block_context->cbp == 0 && code < 2)
          code = 1 - code;
        if (code > 1) code++;
      }
    }
    else{
      if (mode==MODE_MERGE){
        cbp = cbp_y + (cbp_u<<1) + (cbp_v<<2);
        code = cbp_table[cbp];
        if (code==1)
          code = 7;
        else if (code>1)
          code = code-1;
      }
      else{
        cbp = cbp_y + (cbp_u<<1) + (cbp_v<<2);
        code = cbp_table[cbp];
        if (block_info->block_context->cbp == 0 && code < 2) {
          code = 1-code;
        }
      }
    }
    put_vlc(0,code,stream);

    if (tb_split==0){
      if (cbp_y){
        write_coeff(stream,coeffq_y,size,coeff_type|0);
      }
      if (cbp_u){
        write_coeff(stream,coeffq_u,size/2,coeff_type|1);
      }
      if (cbp_v){
        write_coeff(stream,coeffq_v,size/2,coeff_type|1);
      }
    }
    else{
      if (size > 8){
        int index;
        for (index=0;index<4;index++){
          cbp_y = ((pred_data->cbp.y) >> (3 - index)) & 1;
          cbp_u = ((pred_data->cbp.u) >> (3 - index)) & 1;
          cbp_v = ((pred_data->cbp.v) >> (3 - index)) & 1;
          /* Code cbp separately for each TU */
          int cbp = cbp_y + (cbp_u<<1) + (cbp_v<<2);
          code = cbp_table[cbp];
          if (block_info->block_context->cbp == 0 && code < 2)
            code = 1-code;
          put_vlc(0,code,stream);

          /* Code coefficients for each TU separately */
          coeffq_y = block_info->coeff_y + index*(size / 2)*(size / 2);
          coeffq_u = block_info->coeff_u + index*(size / 4)*(size / 4);
          coeffq_v = block_info->coeff_v + index*(size / 4)*(size / 4);
          if (cbp_y){
            write_coeff(stream,coeffq_y,size/2,coeff_type|0);
          }
          if (cbp_u){
            write_coeff(stream,coeffq_u,size/4,coeff_type|1);
          }
          if (cbp_v){
            write_coeff(stream,coeffq_v,size/4,coeff_type|1);
          }
        } //for index=
      } //if (size > 8)
      else{
        int index;
        for (index=0;index<4;index++){
          cbp_y = ((pred_data->cbp.y) >> (3 - index)) & 1;
          /* Code cbp_y separately for each TU */
          putbits(1,cbp_y,stream);

          /* Code coefficients for each TU separately */
          coeffq_y = block_info->coeff_y + index*(size / 2)*(size / 2);
          if (cbp_y){
            write_coeff(stream,coeffq_y,size/2,coeff_type|0);
          }
        }
        cbp = cbp_u + 2*cbp_v;
        if (cbp==0)
          putbits(1,1,stream);
        else if(cbp==1)
          putbits(2,1,stream);
        else if(cbp==2)
          putbits(3,1,stream);
        else
          putbits(3,0,stream);
        if (cbp_u){
          write_coeff(stream,coeffq_u,size/2,coeff_type|1);
        }
        if (cbp_v){
          write_coeff(stream,coeffq_v,size/2,coeff_type|1);
        }
      } //if (size > 8)
    } //if (tb_split==0
  } //if (mode!= MODE_SKIP)

  end_bits = get_bit_pos(stream);

  return (end_bits - start_bits);
}
