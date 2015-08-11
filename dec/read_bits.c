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
#include "getbits.h"
#include "getvlc.h"
#include "common_block.h"
#include "inter_prediction.h"

extern int zigzag16[16];
extern int zigzag64[64];
extern int zigzag256[256];
extern int super_table[8][20];

void read_mv(stream_t *stream,mv_t *mv,mv_t *mvp)
{
    mv_t mvd;
    int code;

    code = get_vlc(10,stream);
    mvd.x = code&1 ? -((code+1)/2) : code/2;
    mv->x = mvp->x + mvd.x;

    code = get_vlc(10,stream);
    mvd.y = code&1 ? -((code+1)/2) : code/2;
    mv->y = mvp->y + mvd.y;
}

int YPOS,XPOS;


int find_index(int code, int maxrun, int type){

  int index;
  int maxrun2 = max(4,maxrun);

  if (type){
    if (code==0)
      index = -1;
    else if (code<=5)
      index = code-1;
    else if (code==6)
      index = maxrun2+1;
    else if (code==7)
      index = maxrun2+2;
    else if (code<=(maxrun2+3))
      index = code-3;
    else
      index = code-1;
  }
  else{
    if (code<=1)
      index = code;
    else if (code==2)
      index = -1;
    else if (code<=5)
      index = code-1;
    else if (code==6)
      index = maxrun2+1;
    else if (code==7)
      index = maxrun2+2;
    else if (code<=(maxrun2+3))
      index = code-3;
    else
      index = code-1;
  }
  return index;
}

void read_coeff(stream_t *stream,int16_t *coeff,int size,int type){

  int16_t scoeff[MAX_QUANT_SIZE*MAX_QUANT_SIZE];
  int i,j,levelFlag,sign,level,pos,index,run,tmp,vlc,maxrun,code,vlc_adaptive;
  int qsize = min(size,MAX_QUANT_SIZE);
  int N = qsize*qsize;
  int level_mode;

  /* Initialize arrays */
  memset(scoeff,0,N*sizeof(int16_t));
  memset(coeff,0,size*size*sizeof(int16_t));

  pos = 0;
  /* Use one bit to signal chroma/last_pos=1/level=1 */
  if (type==1){
    int tmp = getbits1(stream);
    if (tmp){
      sign = getbits1(stream);
      scoeff[pos] = sign ? -1 : 1;
      pos = N;
    }
  }

  /* Initiate forward scan */
  level_mode = 1;
  level = 1;
  vlc_adaptive = 0;
  while (pos < N){
    if (level_mode){
      /* Level-mode */
      //vlc_adaptive = (level > 3 && type==0) ? 1 : 0;
      while (pos < N && level > 0){
        level = get_vlc(vlc_adaptive,stream);
        if (level){
          sign = getbits1(stream);
        }
        else{
          sign = 1;
        }
        scoeff[pos] = sign ? -level : level;
        if (type==0)
          vlc_adaptive = level > 3;
        pos++;
      }
    }
    if (pos >= N){
      break;
    }

    /* Run-mode (run-level coding) */
    maxrun = N - pos - 1;

    /* Decode levelFlag (level > 1) and run */
    if (type && size <= 8){
      vlc = 10;
      code = get_vlc(vlc,stream);
    }
    else{
      vlc = 2;
      if (showbits(stream,2)==2){
        code = getbits(stream,2) - 2;
      }
      else{
        code = get_vlc(vlc,stream) - 1;
      }
    }

    index = find_index(code,maxrun,type);
    if (index == -1){
      break;
    }

    /* Extract levelFlag (level > 1) and run */
    int maxrun2 = max(4,maxrun);
    levelFlag =  index/(maxrun2+1);
    run = index%(maxrun2+1);
    pos += run;

    /* Decode level and sign */
    if (levelFlag){
      tmp = get_vlc(0,stream);
      sign = tmp&1;
      level = (tmp>>1)+2;
    }
    else{
      level = 1;
      sign = getbits1(stream);
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
    sign_delta_qp = getbits(stream,1);
  delta_qp = sign_delta_qp ? -abs_delta_qp : abs_delta_qp;
  return delta_qp;
}

int read_block(decoder_info_t *decoder_info,stream_t *stream,block_info_dec_t *block_info, frame_type_t frame_type)
{
  int width = decoder_info->width;
  int height = decoder_info->height;
  int bit_start;
  int code,tmp,tb_split;
  int PBpart=0;
  cbp_t cbp;

  int size = block_info->block_pos.size;
  int ypos = block_info->block_pos.ypos;
  int xpos = block_info->block_pos.xpos;

  YPOS = ypos;
  XPOS = xpos;

  int sizeY = size;
  int sizeC = size/2;

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

  /* Initialize bit counter for statistical purposes */
  bit_start = stream->bitcnt;

  if (mode == MODE_SKIP){
    /* Derive skip vector candidates and number of skip vector candidates from neighbour blocks */
    mv_t mv_skip[MAX_NUM_SKIP];
    int num_skip_vec,skip_idx;
    mvb_t tmp_mvb_skip[MAX_NUM_SKIP];
    num_skip_vec = get_mv_skip(ypos,xpos,width,height,size,decoder_info->deblock_data,tmp_mvb_skip);
    for (int idx=0;idx<num_skip_vec;idx++){
      mv_skip[idx].x = tmp_mvb_skip[idx].x0;
      mv_skip[idx].y = tmp_mvb_skip[idx].y0;
    }
    /* Decode skip index */
    if (num_skip_vec == 4)
      skip_idx = getbits(stream,2);
    else if (num_skip_vec == 3){
      tmp = getbits(stream,1);
      if (tmp)
        skip_idx = 0;
      else
        skip_idx = 1 + getbits(stream,1);
    }
    else if (num_skip_vec == 2){
      skip_idx = getbits(stream,1);
    }
    else
      skip_idx = 0;
    decoder_info->bit_count.skip_idx[frame_type] += (stream->bitcnt - bit_start);

    block_info->num_skip_vec = num_skip_vec;
    block_info->pred_data.skip_idx = skip_idx;

    if (skip_idx == num_skip_vec)
      mv = mv_skip[0];
    else
      mv = mv_skip[skip_idx];
    mv_arr[0] = mv;
    mv_arr[1] = mv;
    mv_arr[2] = mv;
    mv_arr[3] = mv;

    //int ref_idx = tmp_mvb_skip[skip_idx].ref_idx0;

    block_info->pred_data.ref_idx0 = tmp_mvb_skip[skip_idx].ref_idx0;
    block_info->pred_data.ref_idx1 = tmp_mvb_skip[skip_idx].ref_idx1;
    block_info->pred_data.dir = tmp_mvb_skip[skip_idx].dir;
    mv_arr0[0].x = tmp_mvb_skip[skip_idx].x0;
    mv_arr0[0].y = tmp_mvb_skip[skip_idx].y0;
    mv_arr0[1] = mv_arr0[0];
    mv_arr0[2] = mv_arr0[0];
    mv_arr0[3] = mv_arr0[0];

    mv_arr1[0].x = tmp_mvb_skip[skip_idx].x1;
    mv_arr1[0].y = tmp_mvb_skip[skip_idx].y1;
    mv_arr1[1] = mv_arr1[0];
    mv_arr1[2] = mv_arr1[0];
    mv_arr1[3] = mv_arr1[0];

  }
  else if (mode == MODE_MERGE){
    /* Derive skip vector candidates and number of skip vector candidates from neighbour blocks */
    mv_t mv_skip[MAX_NUM_SKIP];
    int num_skip_vec,skip_idx;
    mvb_t tmp_mvb_skip[MAX_NUM_SKIP];
    num_skip_vec = get_mv_merge(ypos,xpos,width,height,size,decoder_info->deblock_data,tmp_mvb_skip);
    for (int idx=0;idx<num_skip_vec;idx++){
      mv_skip[idx].x = tmp_mvb_skip[idx].x0;
      mv_skip[idx].y = tmp_mvb_skip[idx].y0;
    }
    /* Decode skip index */
    if (num_skip_vec == 4)
      skip_idx = getbits(stream,2);
    else if (num_skip_vec == 3){
      tmp = getbits(stream,1);
      if (tmp)
        skip_idx = 0;
      else
        skip_idx = 1 + getbits(stream,1);
    }
    else if (num_skip_vec == 2){
      skip_idx = getbits(stream,1);
    }
    else
      skip_idx = 0;
    decoder_info->bit_count.skip_idx[frame_type] += (stream->bitcnt - bit_start);

    block_info->num_skip_vec = num_skip_vec;
    block_info->pred_data.skip_idx = skip_idx;

    if (skip_idx == num_skip_vec)
      mv = mv_skip[0];
    else
      mv = mv_skip[skip_idx];
    mv_arr[0] = mv;
    mv_arr[1] = mv;
    mv_arr[2] = mv;
    mv_arr[3] = mv;

    //int ref_idx = tmp_mvb_skip[skip_idx].ref_idx0;

    block_info->pred_data.ref_idx0 = tmp_mvb_skip[skip_idx].ref_idx0;
    block_info->pred_data.ref_idx1 = tmp_mvb_skip[skip_idx].ref_idx1;
    block_info->pred_data.dir = tmp_mvb_skip[skip_idx].dir;
    mv_arr0[0].x = tmp_mvb_skip[skip_idx].x0;
    mv_arr0[0].y = tmp_mvb_skip[skip_idx].y0;
    mv_arr0[1] = mv_arr0[0];
    mv_arr0[2] = mv_arr0[0];
    mv_arr0[3] = mv_arr0[0];

    mv_arr1[0].x = tmp_mvb_skip[skip_idx].x1;
    mv_arr1[0].y = tmp_mvb_skip[skip_idx].y1;
    mv_arr1[1] = mv_arr1[0];
    mv_arr1[2] = mv_arr1[0];
    mv_arr1[3] = mv_arr1[0];
  }
  else if (mode==MODE_INTER){
    int ref_idx;

    if (decoder_info->pb_split){
      /* Decode PU partition */
      tmp = getbits(stream,1);
      if (tmp==1){
        code = 0;
      }
      else{
        tmp = getbits(stream,1);
        if (tmp==1){
          code = 1;
        }
        else{
          tmp = getbits(stream,1);
          code = 3 - tmp;
        }
      }
      PBpart = code;
    }
    else{
      PBpart = 0;
    }
    block_info->pred_data.PBpart = PBpart;
    if (decoder_info->frame_info.num_ref > 1){
      ref_idx = decoder_info->ref_idx;
    }
    else{
      ref_idx = 0;
    }

    if (mode==MODE_INTER)
      decoder_info->bit_count.size_and_ref_idx[log2i(size)-3][ref_idx] += 1;

#if TWO_MVP
    int num_skip_vec;
    mvb_t tmp_mvb_skip[MAX_NUM_SKIP];
    num_skip_vec = get_mv_skip(ypos,xpos,width,height,size,decoder_info->deblock_data,tmp_mvb_skip);
    if (num_skip_vec > 1){
      int idx = getbits(stream,1);
      mvp.y = tmp_mvb_skip[idx].y0;
      mvp.x = tmp_mvb_skip[idx].x0;
    }
    else{
      mvp.y = tmp_mvb_skip[0].y0;
      mvp.x = tmp_mvb_skip[0].x0;
    }
#else
    mvp = get_mv_pred(ypos,xpos,width,height,size,ref_idx,decoder_info->deblock_data);
#endif

    /* Deode motion vectors for each prediction block */
    mv_t mvp2 = mvp;

    if (PBpart==0){
      read_mv(stream,&mv_arr[0],&mvp2);
      mv_arr[1] = mv_arr[0];
      mv_arr[2] = mv_arr[0];
      mv_arr[3] = mv_arr[0];
    }
    else if(PBpart==1){ //HOR
      read_mv(stream,&mv_arr[0],&mvp2);
      mvp2 = mv_arr[0];
      read_mv(stream,&mv_arr[2],&mvp2);
      mv_arr[1] = mv_arr[0];
      mv_arr[3] = mv_arr[2];
    }
    else if(PBpart==2){ //VER
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
    decoder_info->bit_count.mv[frame_type] += (stream->bitcnt - bit_start);
    block_info->pred_data.ref_idx0 = ref_idx;
    block_info->pred_data.ref_idx1 = ref_idx;
    block_info->pred_data.dir = 0;
  }
  else if (mode==MODE_BIPRED){
#if TWO_MVP
    int num_skip_vec;
    mvb_t tmp_mvb_skip[MAX_NUM_SKIP];
    num_skip_vec = get_mv_skip(ypos,xpos,width,height,size,decoder_info->deblock_data,tmp_mvb_skip);
    if (num_skip_vec > 1){
      int idx = getbits(stream,1);
      mvp.y = tmp_mvb_skip[idx].y0;
      mvp.x = tmp_mvb_skip[idx].x0;
    }
    else{
      mvp.y = tmp_mvb_skip[0].y0;
      mvp.x = tmp_mvb_skip[0].x0;
    }
#else
    int ref_idx = 0;
    mvp = get_mv_pred(ypos,xpos,width,height,size,ref_idx,decoder_info->deblock_data);
#endif

    /* Deode motion vectors */
    mv_t mvp2 = mvp;

    read_mv(stream,&mv_arr0[0],&mvp2);
    mv_arr0[1] = mv_arr0[0];
    mv_arr0[2] = mv_arr0[0];
    mv_arr0[3] = mv_arr0[0];
    read_mv(stream,&mv_arr1[0],&mvp2);
    mv_arr1[1] = mv_arr1[0];
    mv_arr1[2] = mv_arr1[0];
    mv_arr1[3] = mv_arr1[0];

    if (decoder_info->frame_info.num_ref==2){
      int code = get_vlc0_limit(3,stream);
      block_info->pred_data.ref_idx0 = (code>>0)&1;
      block_info->pred_data.ref_idx1 = (code>>1)&1;
    }
    else{
      int code = get_vlc(10,stream);
      block_info->pred_data.ref_idx0 = (code>>0)&3;
      block_info->pred_data.ref_idx1 = (code>>2)&3;
    }
    block_info->pred_data.dir = 2;
    decoder_info->bit_count.bi_ref[block_info->pred_data.ref_idx0 * decoder_info->frame_info.num_ref + block_info->pred_data.ref_idx1] += 1;
    decoder_info->bit_count.mv[frame_type] += (stream->bitcnt - bit_start);
  }

  else if (mode==MODE_INTRA){
    /* Decode intra prediction mode */
    if (decoder_info->frame_info.num_intra_modes<=4){
      intra_mode = getbits(stream,2);
    }
    else if (decoder_info->frame_info.num_intra_modes<=8){
      intra_mode = getbits(stream,3);
    }
    else if (decoder_info->frame_info.num_intra_modes<=10){
#if LIMIT_INTRA_MODES
      int intra_mode_map_inv[MAX_NUM_INTRA_MODES] = {3,2,0,9,8,4,7,6,1,5};
      int tmp,code;
      tmp = getbits(stream,2);
      if (tmp<3){
        code = tmp;
      }
      else{
        tmp = getbits(stream,2);
        if (tmp<3){
          code = 3 + tmp;
        }
        else{
          tmp = getbits(stream,1);
          code = 6 + tmp;
        }
      }
      intra_mode = intra_mode_map_inv[code];
#else
      int intra_mode_map_inv[MAX_NUM_INTRA_MODES] = {3,2,0,1,9,8,4,7,6,5};
      int tmp,code;
      tmp = getbits(stream,1);
      if (tmp){
        code = getbits(stream,1);
      }
      else{
        tmp = getbits(stream,1);
        if (tmp){
          code = 2 + getbits(stream,1);
        }
        else{
          tmp = getbits(stream,1);
          if (tmp){
            code = 4 + getbits(stream,1);
          }
          else{
            code = 6 + getbits(stream,2);
          }
        }
      }
      intra_mode = intra_mode_map_inv[code];
#endif
    }

    decoder_info->bit_count.intra_mode[frame_type] += (stream->bitcnt - bit_start);
    decoder_info->bit_count.size_and_intra_mode[frame_type][log2i(size)-3][intra_mode] += 1;

    block_info->pred_data.intra_mode = intra_mode;
    for (int i=0;i<4;i++){
      mv_arr[i] = zerovec; //Note: This is necessary for derivation of mvp and mv_skip
    }
    block_info->pred_data.ref_idx0 = 0;
    block_info->pred_data.ref_idx1 = 0;
    block_info->pred_data.dir = -1;
  }


  if (mode!=MODE_SKIP){
    int tmp,cbp2;
    int cbp_table[8] = {1,0,5,2,6,3,7,4};

    bit_start = stream->bitcnt;
    code = get_vlc(0,stream);

#if NEW_BLOCK_STRUCTURE
    if (decoder_info->tb_split_enable && mode!=MODE_BIPRED){
#else
    if (decoder_info->tb_split_enable && (mode==MODE_INTRA || (mode==MODE_INTER && PBpart==0))){
#endif
      tb_split = code==2;
      if (code > 2) code -= 1;
      if (tb_split)
        decoder_info->bit_count.cbp2_stat[0][frame_type][mode-1][log2i(size)-3][8] += 1;
    }
    else{
      tb_split = 0;
    }
    block_info->tb_split = tb_split;
    decoder_info->bit_count.cbp[frame_type] += (stream->bitcnt - bit_start);

    if (tb_split == 0){
      tmp = 0;
#if NO_SUBBLOCK_SKIP
      if (0){
#else
      if (mode==MODE_MERGE){
#endif
        if (code==7)
          code = 1;
        else if (code>0)
          code = code+1;
      }
      while (tmp < 8 && code != cbp_table[tmp]) tmp++;
#if NO_SUBBLOCK_SKIP
      if (1){
#else
      if (mode!=MODE_MERGE){
#endif

        if (decoder_info->block_context->cbp==0 && tmp < 2){
          tmp = 1-tmp;
        }
      }
      cbp2 = tmp;
      decoder_info->bit_count.cbp2_stat[max(0,decoder_info->block_context->cbp)][frame_type][mode-1][log2i(size)-3][cbp2] += 1;

      cbp.y = ((cbp2>>0)&1);
      cbp.u = ((cbp2>>1)&1);
      cbp.v = ((cbp2>>2)&1);
      block_info->cbp = cbp;

      if (cbp.y){
        bit_start = stream->bitcnt;
        read_coeff(stream,coeff_y,sizeY,0);
        decoder_info->bit_count.coeff_y[frame_type] += (stream->bitcnt - bit_start);
      }
      else
        memset(coeff_y,0,sizeY*sizeY*sizeof(int16_t));

      if (cbp.u){
        bit_start = stream->bitcnt;
        read_coeff(stream,coeff_u,sizeC,1);
        decoder_info->bit_count.coeff_u[frame_type] += (stream->bitcnt - bit_start);
      }
      else
        memset(coeff_u,0,sizeC*sizeC*sizeof(int16_t));

      if (cbp.v){
        bit_start = stream->bitcnt;
        read_coeff(stream,coeff_v,size/2,1);
        decoder_info->bit_count.coeff_v[frame_type] += (stream->bitcnt - bit_start);
      }
      else
        memset(coeff_v,0,sizeC*sizeC*sizeof(int16_t));
    }
    else{
      if (size > 8){
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
          decoder_info->bit_count.cbp[frame_type] += (stream->bitcnt - bit_start);
          decoder_info->bit_count.cbp_stat[frame_type][cbp.y + (cbp.u<<1) + (cbp.v<<2)] += 1;

          /* Decode coefficients for this TU */

          /* Y */
          coeff = coeff_y + index*sizeY/2*sizeY/2;
          if (cbp.y){
            bit_start = stream->bitcnt;
            read_coeff(stream,coeff,sizeY/2,0);
            decoder_info->bit_count.coeff_y[frame_type] += (stream->bitcnt - bit_start);
          }
          else{
            memset(coeff,0,sizeY/2*sizeY/2*sizeof(int16_t));
          }

          /* U */
          coeff = coeff_u + index*sizeC/2*sizeC/2;
          if (cbp.u){
            bit_start = stream->bitcnt;
            read_coeff(stream,coeff,sizeC/2,1);
            decoder_info->bit_count.coeff_u[frame_type] += (stream->bitcnt - bit_start);
          }
          else{
            memset(coeff,0,sizeC/2*sizeC/2*sizeof(int16_t));
          }

          /* V */
          coeff = coeff_v + index*sizeC/2*sizeC/2;
          if (cbp.v){
            bit_start = stream->bitcnt;
            read_coeff(stream,coeff,sizeC/2,1);
            decoder_info->bit_count.coeff_v[frame_type] += (stream->bitcnt - bit_start);
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
          cbp.y = getbits(stream,1);
          decoder_info->bit_count.cbp[frame_type] += (stream->bitcnt - bit_start);

          /* Y */
          coeff = coeff_y + index*sizeY/2*sizeY/2;
          if (cbp.y){
            bit_start = stream->bitcnt;
            read_coeff(stream,coeff,sizeY/2,0);
            decoder_info->bit_count.coeff_y[frame_type] += (stream->bitcnt - bit_start);
          }
          else{
            memset(coeff,0,sizeY/2*sizeY/2*sizeof(int16_t));
          }
        }

        bit_start = stream->bitcnt;
        int tmp;
        tmp = getbits(stream,1);
        if (tmp){
          cbp.u = cbp.v = 0;
        }
        else{
          tmp = getbits(stream,1);
          if (tmp){
            cbp.u = 1;
            cbp.v = 0;
          }
          else{
            tmp = getbits(stream,1);
            if (tmp){
              cbp.u = 0;
              cbp.v = 1;
            }
            else{
              cbp.u = 1;
              cbp.v = 1;
            }
          }
        }
        decoder_info->bit_count.cbp[frame_type] += (stream->bitcnt - bit_start);
        if (cbp.u){
          bit_start = stream->bitcnt;
          read_coeff(stream,coeff_u,sizeC,1);
          decoder_info->bit_count.coeff_u[frame_type] += (stream->bitcnt - bit_start);
        }
        else
          memset(coeff_u,0,sizeC*sizeC*sizeof(int16_t));
        if (cbp.v){
          bit_start = stream->bitcnt;
          read_coeff(stream,coeff_v,size/2,1);
          decoder_info->bit_count.coeff_v[frame_type] += (stream->bitcnt - bit_start);
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
    memcpy(block_info->pred_data.mv_arr0,mv_arr0,4*sizeof(mv_t)); //Used for mv0 coding
    memcpy(block_info->pred_data.mv_arr1,mv_arr1,4*sizeof(mv_t)); //Used for mv1 coding
  }
  else if(mode==MODE_SKIP){
    memcpy(block_info->pred_data.mv_arr0,mv_arr0,4*sizeof(mv_t)); //Used for mv0 coding
    memcpy(block_info->pred_data.mv_arr1,mv_arr1,4*sizeof(mv_t)); //Used for mv1 coding
  }
  else if(mode==MODE_MERGE){
    memcpy(block_info->pred_data.mv_arr0,mv_arr0,4*sizeof(mv_t)); //Used for mv0 coding
    memcpy(block_info->pred_data.mv_arr1,mv_arr1,4*sizeof(mv_t)); //Used for mv1 coding
  }
  else{
    memcpy(block_info->pred_data.mv_arr0,mv_arr,4*sizeof(mv_t)); //Used for mv0 coding
    memcpy(block_info->pred_data.mv_arr1,mv_arr,4*sizeof(mv_t)); //Used for mv1 coding
  }
  block_info->pred_data.mode = mode;
  block_info->tb_split = tb_split;

  int bwidth = min(size,width - xpos);
  int bheight = min(size,height - ypos);

  /* Update mode and block size statistics */
  decoder_info->bit_count.mode[frame_type][mode] += (bwidth/MIN_BLOCK_SIZE * bheight/MIN_BLOCK_SIZE);
  decoder_info->bit_count.size[frame_type][log2i(size)-3] += (bwidth/MIN_BLOCK_SIZE * bheight/MIN_BLOCK_SIZE);
  if (frame_type != I_FRAME){
    decoder_info->bit_count.size_and_mode[log2i(size)-3][mode] += (bwidth/MIN_BLOCK_SIZE * bheight/MIN_BLOCK_SIZE);
  }
  return 0;
}
