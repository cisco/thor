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
#include "getvlc.h"
#include "common_block.h"
#include "inter_prediction.h"

extern int zigzag16[16];
extern int zigzag64[64];
extern int zigzag256[256];

int YPOS, XPOS;

void read_mv(stream_t *stream,mv_t *mv,mv_t *mvp)
{
    mv_t mvd;
    int mvabs, mvsign = 0;

    /* MVX */
    if ((mvabs = get_vlc(7, stream)))
      mvsign = get_flc(1, stream);
    mvd.x = mvabs * (mvsign ? -1 : 1);
    mv->x = mvp->x + mvd.x;

    /* MVY */
    if ((mvabs = get_vlc(7, stream)))
      mvsign = get_flc(1, stream);
    mvd.y = mvabs * (mvsign ? -1 : 1);
    mv->y = mvp->y + mvd.y;
}



void read_coeff(stream_t *stream,int16_t *coeff,int size,int type){

  int16_t scoeff[MAX_QUANT_SIZE*MAX_QUANT_SIZE];
  int i,j,levelFlag,sign,level,pos,run,tmp,code;
  int qsize = min(size,MAX_QUANT_SIZE);
  int N = qsize*qsize;
  int level_mode;
  int chroma_flag = type&1;
  int intra_flag = (type>>1)&1;
  int vlc_adaptive = intra_flag && !chroma_flag;

  /* Initialize arrays */
  memset(scoeff,0,N*sizeof(int16_t));
  memset(coeff,0,size*size*sizeof(int16_t));

  pos = 0;
  /* Use one bit to signal chroma/last_pos=1/level=1 */
  if (chroma_flag==1){
    int tmp = get_flc(1, stream);
    if (tmp){
      sign = get_flc(1, stream);
      scoeff[pos] = sign ? -1 : 1;
      pos = N;
    }
  }

  /* Initiate forward scan */
  level_mode = 1;
  level = 1;
  while (pos < N){
    if (level_mode){
      /* Level-mode */
      while (pos < N && level > 0){
        level = get_vlc(vlc_adaptive,stream);
        if (level){
          sign = get_flc(1, stream);
        }
        else{
          sign = 1;
        }
        scoeff[pos] = sign ? -level : level;
        if (chroma_flag==0)
          vlc_adaptive = level > 3;
        pos++;
      }
    }
    if (pos >= N){
      break;
    }

    /* Run-mode */
    int eob;
    int eob_pos = chroma_flag ? 0 : 2;
    if (chroma_flag && size <= 8)
      code = get_vlc(10, stream);
    else
      code = get_vlc(6, stream);

    eob = code == eob_pos;
    if (eob) {
      break;
    }
    if (code > eob_pos) code -= 1;
    levelFlag = (code % 5) == 4;
    if (levelFlag)
      run = code / 5;
    else
      run = 4*(code/5) + code % 5;
    pos += run;

    /* Decode level and sign */
    if (levelFlag){
      tmp = get_vlc(0,stream);
      sign = tmp&1;
      level = (tmp>>1)+2;
    }
    else{
      level = 1;
      sign = get_flc(1, stream);
    }
    scoeff[pos] = sign ? -level : level;

    level_mode = level > 1; //Set level_mode
    pos++;
  } //while pos < N

  /* Perform inverse zigzag scan */
  int *zigzagptr = zigzag64;
  if (qsize==4)
    zigzagptr = zigzag16;
  else if (qsize==8)
    zigzagptr = zigzag64;
  else if (qsize==16)
    zigzagptr = zigzag256;
  for (i=0;i<qsize;i++){
    for (j=0;j<qsize;j++){
      coeff[i*size+j] = scoeff[zigzagptr[i*qsize+j]];
    }
  }
}

int read_delta_qp(stream_t *stream){
  int abs_delta_qp,sign_delta_qp,delta_qp;
  sign_delta_qp = 0;
  abs_delta_qp = get_vlc(0,stream);
  if (abs_delta_qp > 0)
    sign_delta_qp = get_flc(1, stream);
  delta_qp = sign_delta_qp ? -abs_delta_qp : abs_delta_qp;
  return delta_qp;
}
int read_block(decoder_info_t *decoder_info,stream_t *stream,block_info_dec_t *block_info, frame_type_t frame_type)
{
  int width = decoder_info->width;
  int height = decoder_info->height;
  int bit_start;
  int code,tb_split;
  int pb_part=0;
  cbp_t cbp;
  int stat_frame_type = decoder_info->bit_count.stat_frame_type; //TODO: Use only one variable for frame type

  int size = block_info->block_pos.size;
  int ypos = block_info->block_pos.ypos;
  int xpos = block_info->block_pos.xpos;

  YPOS = ypos;
  XPOS = xpos;

  int sizeY = size;
  int sizeC = size>>(decoder_info->subx || decoder_info->suby); // TODO: What about 422?

  mv_t mv,zerovec;
  mv_t mvp;
  mv_t mv_arr[4]; //TODO: Use mv_arr0 instead
  mv_t mv_arr0[4];
  mv_t mv_arr1[4];

  block_mode_t mode;
  intra_mode_t intra_mode = MODE_DC;

  int16_t *coeff_y = block_info->coeffq_y;
  int16_t *coeff_u = block_info->coeffq_u;
  int16_t *coeff_v = block_info->coeffq_v;

  zerovec.y = zerovec.x = 0;
  bit_start = stream->bitcnt;

  mode = decoder_info->mode;
  int coeff_block_type = (mode == MODE_INTRA)<<1;

  /* Initialize bit counter for statistical purposes */
  bit_start = stream->bitcnt;

  if (mode == MODE_SKIP){
    /* Derive skip vector candidates and number of skip vector candidates from neighbour blocks */
    mv_t mv_skip[MAX_NUM_SKIP];
    int num_skip_vec,skip_idx;
    inter_pred_t skip_candidates[MAX_NUM_SKIP];
    num_skip_vec = get_mv_skip(ypos, xpos, width, height, size, size, 1 << decoder_info->log2_sb_size, decoder_info->deblock_data, skip_candidates);
    for (int idx = 0; idx < num_skip_vec; idx++) {
      mv_skip[idx] = skip_candidates[idx].mv0;
    }
    /* Decode skip index */
    if (num_skip_vec == 4)
      skip_idx = get_flc(2, stream);
    else if (num_skip_vec == 3){
      skip_idx = get_vlc(12, stream);
    }
    else if (num_skip_vec == 2){
      skip_idx = get_flc(1, stream);
    }
    else
      skip_idx = 0;
    decoder_info->bit_count.skip_idx[stat_frame_type] += (stream->bitcnt - bit_start);

    block_info->num_skip_vec = num_skip_vec;
    block_info->block_param.skip_idx = skip_idx;

    if (skip_idx == num_skip_vec)
      mv = mv_skip[0];
    else
      mv = mv_skip[skip_idx];
    mv_arr[0] = mv;
    mv_arr[1] = mv;
    mv_arr[2] = mv;
    mv_arr[3] = mv;

    block_info->block_param.ref_idx0 = skip_candidates[skip_idx].ref_idx0;
    block_info->block_param.ref_idx1 = skip_candidates[skip_idx].ref_idx1;
    for (int i = 0; i < 4; i++) {
      mv_arr0[i] = skip_candidates[skip_idx].mv0;
      mv_arr1[i] = skip_candidates[skip_idx].mv1;
    }
    block_info->block_param.dir = skip_candidates[skip_idx].bipred_flag;
  }
  else if (mode == MODE_MERGE){
    /* Derive skip vector candidates and number of skip vector candidates from neighbour blocks */
    mv_t mv_skip[MAX_NUM_SKIP];
    int num_skip_vec,skip_idx;
    inter_pred_t merge_candidates[MAX_NUM_SKIP];
    num_skip_vec = get_mv_merge(ypos, xpos, width, height, size, size, 1 << decoder_info->log2_sb_size, decoder_info->deblock_data, merge_candidates);
    for (int idx = 0; idx < num_skip_vec; idx++) {
      mv_skip[idx] = merge_candidates[idx].mv0;
    }
    /* Decode skip index */
    if (num_skip_vec == 4)
      skip_idx = get_flc(2, stream);
    else if (num_skip_vec == 3){
      skip_idx = get_vlc(12, stream);
    }
    else if (num_skip_vec == 2){
      skip_idx = get_flc(1, stream);
    }
    else
      skip_idx = 0;
    decoder_info->bit_count.skip_idx[stat_frame_type] += (stream->bitcnt - bit_start);

    block_info->num_skip_vec = num_skip_vec;
    block_info->block_param.skip_idx = skip_idx;

    if (skip_idx == num_skip_vec)
      mv = mv_skip[0];
    else
      mv = mv_skip[skip_idx];
    mv_arr[0] = mv;
    mv_arr[1] = mv;
    mv_arr[2] = mv;
    mv_arr[3] = mv;

    block_info->block_param.ref_idx0 = merge_candidates[skip_idx].ref_idx0;
    block_info->block_param.ref_idx1 = merge_candidates[skip_idx].ref_idx1;
    for (int i = 0; i < 4; i++) {
      mv_arr0[i] = merge_candidates[skip_idx].mv0;
      mv_arr1[i] = merge_candidates[skip_idx].mv1;
    }
    block_info->block_param.dir = merge_candidates[skip_idx].bipred_flag;
  }
  else if (mode==MODE_INTER){
    int ref_idx;

    if (decoder_info->pb_split){
      /* Decode PU partition */
      pb_part = get_vlc(13, stream);
    }
    else{
      pb_part = 0;
    }
    block_info->block_param.pb_part = pb_part;
    if (decoder_info->frame_info.num_ref > 1){
      ref_idx = decoder_info->ref_idx;
    }
    else{
      ref_idx = 0;
    }

    //if (mode==MODE_INTER)
    decoder_info->bit_count.size_and_ref_idx[stat_frame_type][log2i(size)-3][ref_idx] += 1;

    mvp = get_mv_pred(ypos,xpos,width,height,size,size,1<<decoder_info->log2_sb_size,ref_idx,decoder_info->deblock_data);

    /* Deode motion vectors for each prediction block */
    mv_t mvp2 = mvp;

    if (pb_part==0){
      read_mv(stream,&mv_arr[0],&mvp2);
      mv_arr[1] = mv_arr[0];
      mv_arr[2] = mv_arr[0];
      mv_arr[3] = mv_arr[0];
    }
    else if(pb_part==1){ //HOR
      read_mv(stream,&mv_arr[0],&mvp2);
      mvp2 = mv_arr[0];
      read_mv(stream,&mv_arr[2],&mvp2);
      mv_arr[1] = mv_arr[0];
      mv_arr[3] = mv_arr[2];
    }
    else if(pb_part==2){ //VER
      read_mv(stream,&mv_arr[0],&mvp2);
      mvp2 = mv_arr[0];
      read_mv(stream,&mv_arr[1],&mvp2);
      mv_arr[2] = mv_arr[0];
      mv_arr[3] = mv_arr[1];
    }
    else{
      read_mv(stream,&mv_arr[0],&mvp2);
      mvp2 = mv_arr[0];
      read_mv(stream,&mv_arr[1],&mvp2);
      read_mv(stream,&mv_arr[2],&mvp2);
      read_mv(stream,&mv_arr[3],&mvp2);
    }
    decoder_info->bit_count.mv[stat_frame_type] += (stream->bitcnt - bit_start);
    block_info->block_param.ref_idx0 = ref_idx;
    block_info->block_param.ref_idx1 = ref_idx;
    block_info->block_param.dir = 0;
  }
  else if (mode==MODE_BIPRED){
    int ref_idx = 0;
    mvp = get_mv_pred(ypos,xpos,width,height,size,size,1 << decoder_info->log2_sb_size, ref_idx,decoder_info->deblock_data);

    /* Deode motion vectors */
    mv_t mvp2 = mvp;

#if BIPRED_PART
    if (decoder_info->pb_split) {
      /* Decode PU partition */
      pb_part = get_vlc(13, stream);
    }
    else {
      pb_part = 0;
    }
#else
    pb_part = 0;
#endif
    block_info->block_param.pb_part = pb_part;

    if (pb_part == 0) {
      read_mv(stream, &mv_arr0[0], &mvp2);
      mv_arr0[1] = mv_arr0[0];
      mv_arr0[2] = mv_arr0[0];
      mv_arr0[3] = mv_arr0[0];
    }
    else {
      mv_arr0[0] = mvp2;
      mv_arr0[1] = mvp2;
      mv_arr0[2] = mvp2;
      mv_arr0[3] = mvp2;
    }
    if (decoder_info->bit_count.stat_frame_type == B_FRAME)
      mvp2 = mv_arr0[0];
    if (pb_part == 0) {
      read_mv(stream, &mv_arr1[0], &mvp2);
      mv_arr1[1] = mv_arr1[0];
      mv_arr1[2] = mv_arr1[0];
      mv_arr1[3] = mv_arr1[0];
    }
    else if (pb_part == 1) { //HOR
      read_mv(stream, &mv_arr1[0], &mvp2);
      mvp2 = mv_arr1[0];
      read_mv(stream, &mv_arr1[2], &mvp2);
      mv_arr1[1] = mv_arr1[0];
      mv_arr1[3] = mv_arr1[2];
    }
    else if (pb_part == 2) { //VER
      read_mv(stream, &mv_arr1[0], &mvp2);
      mvp2 = mv_arr1[0];
      read_mv(stream, &mv_arr1[1], &mvp2);
      mv_arr1[2] = mv_arr1[0];
      mv_arr1[3] = mv_arr1[1];
    }
    else {
      read_mv(stream, &mv_arr1[0], &mvp2);
      mvp2 = mv_arr1[0];
      read_mv(stream, &mv_arr1[1], &mvp2);
      read_mv(stream, &mv_arr1[2], &mvp2);
      read_mv(stream, &mv_arr1[3], &mvp2);
    }

    if (decoder_info->bit_count.stat_frame_type == B_FRAME) {
      block_info->block_param.ref_idx0 = 0;
      block_info->block_param.ref_idx1 = 1;
      if (decoder_info->frame_info.interp_ref == 1) {
        block_info->block_param.ref_idx0 += 1;
        block_info->block_param.ref_idx1 += 1;
      }
    }
    else{
      if (decoder_info->frame_info.num_ref == 2) {
        int code = get_vlc(13, stream);
        block_info->block_param.ref_idx0 = (code >> 1) & 1;
        block_info->block_param.ref_idx1 = (code >> 0) & 1;
      }
      else {
        int code = get_vlc(10, stream);
        block_info->block_param.ref_idx0 = (code >> 2) & 3;
        block_info->block_param.ref_idx1 = (code >> 0) & 3;
      }
    }
    block_info->block_param.dir = 2;
    int combined_ref = block_info->block_param.ref_idx0 * decoder_info->frame_info.num_ref + block_info->block_param.ref_idx1;
    decoder_info->bit_count.bi_ref[stat_frame_type][combined_ref] += 1;
    decoder_info->bit_count.mv[stat_frame_type] += (stream->bitcnt - bit_start);
  }

  else if (mode==MODE_INTRA){
    /* Decode intra prediction mode */
    if (decoder_info->frame_info.num_intra_modes<=4){
      intra_mode = get_flc(2, stream);
    }
    else {
      intra_mode = get_vlc(8, stream);
    }

    decoder_info->bit_count.intra_mode[stat_frame_type] += (stream->bitcnt - bit_start);
    decoder_info->bit_count.size_and_intra_mode[stat_frame_type][log2i(size)-3][intra_mode] += 1;

    block_info->block_param.intra_mode = intra_mode;
    for (int i=0;i<4;i++){
      mv_arr[i] = zerovec; //Note: This is necessary for derivation of mvp and mv_skip
    }
    block_info->block_param.ref_idx0 = 0;
    block_info->block_param.ref_idx1 = 0;
    block_info->block_param.dir = -1;
  }


  if (mode!=MODE_SKIP){
    int tmp;
    int cbp_table[8] = {1,0,5,2,6,3,7,4};

    bit_start = stream->bitcnt;
    code = get_vlc(0,stream);
    int off = (mode == MODE_MERGE) ? 1 : 2;
    if (decoder_info->tb_split_enable) {
      tb_split = code == off;
      if (code > off) code -= 1;
      if (tb_split)
        decoder_info->bit_count.cbp2_stat[0][stat_frame_type][mode-1][log2i(size)-3][8] += 1;
    }
    else{
      tb_split = 0;
    }
    block_info->block_param.tb_split = tb_split;
    decoder_info->bit_count.cbp[stat_frame_type] += (stream->bitcnt - bit_start);

    if (tb_split == 0){
      tmp = 0;
      if (mode==MODE_MERGE){
        if (code==7)
          code = 1;
        else if (code>0)
          code = code+1;
      }
      else {
        if (decoder_info->block_context->cbp == 0 && code < 2) {
          code = 1 - code;
        }
      }
      while (tmp < 8 && code != cbp_table[tmp]) tmp++;
      decoder_info->bit_count.cbp2_stat[max(0,decoder_info->block_context->cbp)][stat_frame_type][mode-1][log2i(size)-3][tmp] += 1;

      cbp.y = ((tmp>>0)&1);
      cbp.u = ((tmp>>1)&1);
      cbp.v = ((tmp>>2)&1);
      block_info->cbp = cbp;

      if (cbp.y){
        bit_start = stream->bitcnt;
        read_coeff(stream,coeff_y,sizeY,coeff_block_type|0);
        decoder_info->bit_count.coeff_y[stat_frame_type] += (stream->bitcnt - bit_start);
      }
      else
        memset(coeff_y,0,sizeY*sizeY*sizeof(int16_t));

      if (cbp.u){
        bit_start = stream->bitcnt;
        read_coeff(stream,coeff_u,sizeC,coeff_block_type|1);
        decoder_info->bit_count.coeff_u[stat_frame_type] += (stream->bitcnt - bit_start);
      }
      else
        memset(coeff_u,0,sizeC*sizeC*sizeof(int16_t));

      if (cbp.v){
        bit_start = stream->bitcnt;
        read_coeff(stream,coeff_v,sizeC,coeff_block_type|1);
        decoder_info->bit_count.coeff_v[stat_frame_type] += (stream->bitcnt - bit_start);
      }
      else
        memset(coeff_v,0,sizeC*sizeC*sizeof(int16_t));
    }
    else{
      if (sizeC > 4){
        int index;
        int16_t *coeff;

        /* Loop over 4 TUs */
        for (index=0;index<4;index++){
          bit_start = stream->bitcnt;
          code = get_vlc(0,stream);
          int tmp = 0;
          while (code != cbp_table[tmp] && tmp < 8) tmp++;
          if (decoder_info->block_context->cbp==0 && tmp < 2)
            tmp = 1-tmp;
          cbp.y = ((tmp>>0)&1);
          cbp.u = ((tmp>>1)&1);
          cbp.v = ((tmp>>2)&1);

          /* Updating statistics for CBP */
          decoder_info->bit_count.cbp[stat_frame_type] += (stream->bitcnt - bit_start);
          decoder_info->bit_count.cbp_stat[stat_frame_type][cbp.y + (cbp.u<<1) + (cbp.v<<2)] += 1;

          /* Decode coefficients for this TU */

          /* Y */
          coeff = coeff_y + index*sizeY/2*sizeY/2;
          if (cbp.y){
            bit_start = stream->bitcnt;
            read_coeff(stream,coeff,sizeY/2,coeff_block_type|0);
            decoder_info->bit_count.coeff_y[stat_frame_type] += (stream->bitcnt - bit_start);
          }
          else{
            memset(coeff,0,sizeY/2*sizeY/2*sizeof(int16_t));
          }

          /* U */
          coeff = coeff_u + index*sizeC/2*sizeC/2;
          if (cbp.u){
            bit_start = stream->bitcnt;
            read_coeff(stream,coeff,sizeC/2,coeff_block_type|1);
            decoder_info->bit_count.coeff_u[stat_frame_type] += (stream->bitcnt - bit_start);
          }
          else{
            memset(coeff,0,sizeC/2*sizeC/2*sizeof(int16_t));
          }

          /* V */
          coeff = coeff_v + index*sizeC/2*sizeC/2;
          if (cbp.v){
            bit_start = stream->bitcnt;
            read_coeff(stream,coeff,sizeC/2,coeff_block_type|1);
            decoder_info->bit_count.coeff_v[stat_frame_type] += (stream->bitcnt - bit_start);
          }
          else{
            memset(coeff,0,sizeC/2*sizeC/2*sizeof(int16_t));
          }
        }
        block_info->cbp.y = 1; //TODO: Do properly with respect to deblocking filter
        block_info->cbp.u = 1;
        block_info->cbp.v = 1;
      }
      else{
        int index;
        int16_t *coeff;

        /* Loop over 4 TUs */
        for (index=0;index<4;index++){
          bit_start = stream->bitcnt;
          cbp.y = get_flc(1, stream);
          decoder_info->bit_count.cbp[stat_frame_type] += (stream->bitcnt - bit_start);

          /* Y */
          coeff = coeff_y + index*sizeY/2*sizeY/2;
          if (cbp.y){
            bit_start = stream->bitcnt;
            read_coeff(stream,coeff,sizeY/2,coeff_block_type|0);
            decoder_info->bit_count.coeff_y[stat_frame_type] += (stream->bitcnt - bit_start);
          }
          else{
            memset(coeff,0,sizeY/2*sizeY/2*sizeof(int16_t));
          }
        }

        bit_start = stream->bitcnt;
        int tmp;
        tmp = get_vlc(13, stream);
        cbp.u = tmp & 1;
        cbp.v = (tmp >> 1) & 1;
        decoder_info->bit_count.cbp[stat_frame_type] += (stream->bitcnt - bit_start);
        if (cbp.u){
          bit_start = stream->bitcnt;
          read_coeff(stream,coeff_u,sizeC,coeff_block_type|1);
          decoder_info->bit_count.coeff_u[stat_frame_type] += (stream->bitcnt - bit_start);
        }
        else
          memset(coeff_u,0,sizeC*sizeC*sizeof(int16_t));
        if (cbp.v){
          bit_start = stream->bitcnt;
          read_coeff(stream,coeff_v,sizeC,coeff_block_type|1);
          decoder_info->bit_count.coeff_v[stat_frame_type] += (stream->bitcnt - bit_start);
        }
        else
          memset(coeff_v,0,sizeC*sizeC*sizeof(int16_t));

        block_info->cbp.y = 1; //TODO: Do properly with respect to deblocking filter
        block_info->cbp.u = 1;
        block_info->cbp.v = 1;
      } //if (size==8)
    } //if (tb_split==0)
  } //if (mode!=MODE_SKIP)
  else{
    tb_split = 0;
    block_info->cbp.y = 0;
    block_info->cbp.u = 0;
    block_info->cbp.v = 0;
  }

  /* Store block data */
  if (mode==MODE_BIPRED){
    memcpy(block_info->block_param.mv_arr0,mv_arr0,4*sizeof(mv_t)); //Used for mv0 coding
    memcpy(block_info->block_param.mv_arr1,mv_arr1,4*sizeof(mv_t)); //Used for mv1 coding
  }
  else if(mode==MODE_SKIP){
    memcpy(block_info->block_param.mv_arr0,mv_arr0,4*sizeof(mv_t)); //Used for mv0 coding
    memcpy(block_info->block_param.mv_arr1,mv_arr1,4*sizeof(mv_t)); //Used for mv1 coding
  }
  else if(mode==MODE_MERGE){
    memcpy(block_info->block_param.mv_arr0,mv_arr0,4*sizeof(mv_t)); //Used for mv0 coding
    memcpy(block_info->block_param.mv_arr1,mv_arr1,4*sizeof(mv_t)); //Used for mv1 coding
  }
  else{
    memcpy(block_info->block_param.mv_arr0,mv_arr,4*sizeof(mv_t)); //Used for mv0 coding
    memcpy(block_info->block_param.mv_arr1,mv_arr,4*sizeof(mv_t)); //Used for mv1 coding
  }
  block_info->block_param.mode = mode;
  block_info->block_param.tb_split = tb_split;

  int bwidth = min(size,width - xpos);
  int bheight = min(size,height - ypos);

  /* Update mode and block size statistics */
  decoder_info->bit_count.mode[stat_frame_type][mode] += (bwidth/MIN_BLOCK_SIZE * bheight/MIN_BLOCK_SIZE);
  decoder_info->bit_count.size[stat_frame_type][log2i(size)-3] += (bwidth/MIN_BLOCK_SIZE * bheight/MIN_BLOCK_SIZE);
  decoder_info->bit_count.size_and_mode[stat_frame_type][log2i(size)-3][mode] += (bwidth/MIN_BLOCK_SIZE * bheight/MIN_BLOCK_SIZE);
  return 0;
}
