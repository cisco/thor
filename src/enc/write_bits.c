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

int find_code(int run, int level, int maxrun, int type,int eob){

  int cn,index;
  int maxrun2 = max(4,maxrun);
  index = run + (level>1)*(maxrun2+1);

  if (type){
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
  int vlc_adaptive=0;
  int N = qsize*qsize;
  int level_mode;

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
  if (type==1){
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
      //vlc_adaptive = (level > 3 && type==0) ? 1 : 0;
      while (pos <= last_pos && level > 0){
        c = scoeff[pos];
        level = abs(c);
        len = put_vlc(vlc_adaptive,level,stream);
        if (level > 0){
          sign = (c < 0) ? 1 : 0;
          putbits(1,sign,stream);
          len += 1;
        }
        if (type==0)
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
        cn = find_code(run, level, maxrun, type, 0);

        if (type && size <= 8){
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
      //vlc_adaptive = (level > 3 && type==0) ? 1 : 0;
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
    cn = find_code(0, 0, 0, type, 1);
    if (type && size <= 8){
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

#if NEW_BLOCK_STRUCTURE
void write_super_mode(stream_t *stream,write_data_t *write_data){

  int size = write_data->size;
  block_mode_t mode = write_data->mode;
  frame_type_t frame_type = write_data->frame_type;
  if (frame_type!=I_FRAME && write_data->encode_rectangular_size==0){
    int num_split_codes = 2*(size==MAX_BLOCK_SIZE);
    int code=0,maxbit,mode_offset,ref_offset;
    mode_offset = 2 + num_split_codes;
    maxbit = write_data->num_ref + 2 + num_split_codes;
    if (write_data->num_ref>1 && write_data->enable_bipred) maxbit += 1;

    if (mode==MODE_SKIP){
      code = 0;
    }
    else if (mode==MODE_MERGE){
      code = mode_offset - 1;
    }
    else if (mode==MODE_INTRA){
      code = mode_offset + 1;
    }
    else if (mode==MODE_INTER){
      ref_offset = write_data->ref_idx==0 ? 0 : write_data->ref_idx + 1;
      code = mode_offset + ref_offset;
    }
    else if (mode==MODE_BIPRED){
      code = maxbit;
    }

    /* Switch MERGE and INTER-R0 */ //TODO: Integrate with code above
    if (code==2)
      code = 3;
    else if(code==3)
      code = 2;

    if (write_data->block_context->index==2 || write_data->block_context->index>3){
      if (size>MIN_BLOCK_SIZE && code<3)
        code = (code+2)%3;
    }

    if (code==maxbit)
      putbits(maxbit,0,stream);
    else
      putbits(code+1,1,stream);

  }
}
#else
void write_super_mode(stream_t *stream,write_data_t *write_data){

  int size = write_data->size;
  block_mode_t mode = write_data->mode;
  frame_type_t frame_type = write_data->frame_type;
  if (frame_type!=I_FRAME){
    int code=0,maxbit;
    maxbit = write_data->num_ref + 2 + (size>MIN_BLOCK_SIZE);
    if (write_data->num_ref>1 && write_data->enable_bipred) maxbit += 1;

    if (size>MIN_BLOCK_SIZE){
      if (mode==MODE_SKIP)
        code = 0;
      else if (mode==MODE_INTER && write_data->ref_idx==0)
        code = 2;
      else if (mode==MODE_MERGE)
        code = 3;
      else if (mode==MODE_INTRA)
        code = 4;
      else if (mode==MODE_INTER && write_data->ref_idx>0)
        code = 4 + write_data->ref_idx;
      else if (mode==MODE_BIPRED)
        code = 4 + write_data->num_ref;
#if NO_SUBBLOCK_SKIP
      if (size < MAX_BLOCK_SIZE){
        if (code==2) code = 3;
        else if (code==3) code = 2;
      }
#endif
    }
    else{
      if (mode==MODE_SKIP)
        code = 0;
#if 9
      else if (mode==MODE_INTER && write_data->ref_idx==0)
        code = 1;
      else if (mode==MODE_MERGE)
        code = 2;
      else if (mode==MODE_INTRA)
        code = 3;
#else
      else if (mode==MODE_INTER && write_data->ref_idx==0)
        code = 3;
      else if (mode==MODE_MERGE)
        code = 1;
      else if (mode==MODE_INTRA)
        code = 2;
#endif
      else if (mode==MODE_INTER && write_data->ref_idx>0)
        code = 3 + write_data->ref_idx;
      else if (mode==MODE_BIPRED)
        code = 3 + write_data->num_ref;
#if NO_SUBBLOCK_SKIP
      if (size < MAX_BLOCK_SIZE){
        if (code==1) code = 2;
        else if (code==2) code = 1;
      }
#endif
    }

    if (write_data->block_context->index==2 || write_data->block_context->index>3){
      if (size>MIN_BLOCK_SIZE && code<4)
        code = (code+3)%4;
    }

    if (code==maxbit)
      putbits(maxbit,0,stream);
    else
      putbits(code+1,1,stream);

  }
  else{
    putbits(1,0,stream); // To signal split_flag = 0
  }
}
#endif

int write_block(stream_t *stream,write_data_t *write_data){

  int start_bits,end_bits;
  int size = write_data->size;

  int tb_split = write_data->tb_part;

  uint8_t cbp_y = write_data->cbp->y;
  uint8_t cbp_u = write_data->cbp->u;
  uint8_t cbp_v = write_data->cbp->v;

  int16_t *coeffq_y = write_data->coeffq_y;
  int16_t *coeffq_u = write_data->coeffq_u;
  int16_t *coeffq_v = write_data->coeffq_v;

  block_mode_t mode = write_data->mode;
  intra_mode_t intra_mode = write_data->intra_mode;

  mv_t mvp = write_data->mvp;

  start_bits = get_bit_pos(stream);

  int code,cbp;
  int cbp_table[8] = {1,0,5,2,6,3,7,4};


  /* Write mode and ref_idx */
  write_super_mode(stream,write_data);

  if (size==MAX_BLOCK_SIZE && mode != MODE_SKIP && write_data->max_delta_qp){
    write_delta_qp(stream,write_data->delta_qp);
  }

  /* Code intra mode */
  if (mode==MODE_INTRA){
    if (write_data->num_intra_modes<=4){
      putbits(2,intra_mode,stream);
    }
    else if (write_data->num_intra_modes <= 8){
      putbits(3,intra_mode,stream);
    }
    else if (write_data->num_intra_modes <= 10){
#if LIMIT_INTRA_MODES
      int intra_mode_map[MAX_NUM_INTRA_MODES] = {2,8,1,0,5,9,7,6,4,3};
      int code = intra_mode_map[intra_mode];
      assert (code<8);
      if (code==0){
        putbits(2,0,stream);
      }
      else if(code==1){
         putbits(2,1,stream);
      }
      else if(code==2){
         putbits(2,2,stream);
      }
      else if(code==3){
         putbits(4,12,stream);
      }
      else if(code==4){
         putbits(4,13,stream);
      }
      else if(code==5){
         putbits(4,14,stream);
      }
      else if(code==6){
         putbits(5,30,stream);
      }
      else if(code==7){
         putbits(5,31,stream);
      }
#else
      int intra_mode_map[MAX_NUM_INTRA_MODES] = {2,3,1,0,6,9,8,7,5,4};
      int code = intra_mode_map[intra_mode];
      if (code==0){
        putbits(2,2,stream);
      }
      else if(code==1){
         putbits(2,3,stream);
      }
      else if(code==2){
         putbits(3,2,stream);
      }
      else if(code==3){
         putbits(3,3,stream);
      }
      else if(code==4){
         putbits(4,2,stream);
      }
      else if(code==5){
         putbits(4,3,stream);
      }
      else if(code==6){
         putbits(5,0,stream);
      }
      else if(code==7){
         putbits(5,1,stream);
      }
      else if(code==8){
         putbits(5,2,stream);
      }
      else if(code==9){
         putbits(5,3,stream);
      }
#endif
    }
  }
  else if (mode==MODE_INTER){
    /* Code PU partitions */
    if (write_data->max_num_pb_part > 1){
      if (write_data->pb_part==0)
        putbits(1,1,stream);
      else if(write_data->pb_part==1)
        putbits(2,1,stream);
      else if(write_data->pb_part==2)
        putbits(3,1,stream);
      else if(write_data->pb_part==3)
        putbits(3,0,stream);
    }
    /* Code motion vectors for each prediction block */
    mv_t mvp2 = mvp;
#if TWO_MVP
    if (write_data->mv_idx >= 0){
      putbits(1,write_data->mv_idx,stream);
    }
#endif
    if (write_data->pb_part==PART_NONE){ //NONE
      write_mv(stream,&write_data->mv_arr[0],&mvp2);
    }
    else if (write_data->pb_part==PART_HOR){ //HOR
      write_mv(stream,&write_data->mv_arr[0],&mvp2);
      mvp2 = write_data->mv_arr[0];
      write_mv(stream,&write_data->mv_arr[2],&mvp2);
    }
    else if (write_data->pb_part==PART_VER){ //VER
      write_mv(stream,&write_data->mv_arr[0],&mvp2);
      mvp2 = write_data->mv_arr[0];
      write_mv(stream,&write_data->mv_arr[1],&mvp2);
    }
    else{
      write_mv(stream,&write_data->mv_arr[0],&mvp2);
      mvp2 = write_data->mv_arr[0];
      write_mv(stream,&write_data->mv_arr[1],&mvp2);
      write_mv(stream,&write_data->mv_arr[2],&mvp2);
      write_mv(stream,&write_data->mv_arr[3],&mvp2);
    }
  }
  else if (mode==MODE_BIPRED){
#if TWO_MVP
    if (write_data->mv_idx >= 0){
      putbits(1,write_data->mv_idx,stream);
    }
#endif
    /* Code motion vectors for each prediction block */
    mv_t mvp2 = mvp; //TODO: Use different predictors for mv0 and mv1
    write_mv(stream,&write_data->mv_arr0[0],&mvp2); //TODO: Make bipred and pb-split combination work
    write_mv(stream,&write_data->mv_arr1[0],&mvp2);
    if (write_data->num_ref==2){
      int code = 2*write_data->ref_idx1 + write_data->ref_idx0;
      if (code==3)
        putbits(3,0,stream);
      else
        putbits(code+1,1,stream);
    }
    else{
      int code = 4*write_data->ref_idx1 + write_data->ref_idx0; //TODO: Optimize for num_ref != 4
      put_vlc(10,code,stream);
    }
  }
  else if (mode==MODE_SKIP){
    /* Code skip_idx */
    if (write_data->num_skip_vec == 4)
      putbits(2,write_data->skip_idx,stream);
    else if (write_data->num_skip_vec == 3){
      if (write_data->skip_idx == 0) putbits(1,1,stream);
      else if (write_data->skip_idx == 1) putbits(2,0,stream);
      else putbits(2,1,stream);
    }
    else if (write_data->num_skip_vec == 2){
      putbits(1,write_data->skip_idx,stream);
    }
  }
  else if (mode==MODE_MERGE){
    /* Code skip_idx */
    if (write_data->num_skip_vec == 4)
      putbits(2,write_data->skip_idx,stream);
    else if (write_data->num_skip_vec == 3){
      if (write_data->skip_idx == 0) putbits(1,1,stream);
      else if (write_data->skip_idx == 1) putbits(2,0,stream);
      else putbits(2,1,stream);
    }
    else if (write_data->num_skip_vec == 2){
      putbits(1,write_data->skip_idx,stream);
    }
  }

  if (mode != MODE_SKIP){
    if (write_data->max_num_tb_part>1){
      if (tb_split){
        code = 2;
      }
      else{
        cbp = cbp_y + (cbp_u<<1) + (cbp_v<<2);
        code = cbp_table[cbp];
        if (write_data->block_context->cbp==0 && code < 2)
          code = 1-code;
        if (code > 1) code++;
      }
    }
    else{
#if NO_SUBBLOCK_SKIP
      if (0){
#else
      if (mode==MODE_MERGE){
#endif
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
        if (write_data->block_context->cbp==0 && code < 2){
          code = 1-code;
        }
      }
    }
    put_vlc(0,code,stream);

    if (tb_split==0){
      if (cbp_y){
        write_coeff(stream,coeffq_y,size,0);
      }
      if (cbp_u){
        write_coeff(stream,coeffq_u,size/2,1);
      }
      if (cbp_v){
        write_coeff(stream,coeffq_v,size/2,1);
      }
    }
    else{
      if (size > 8){
        int index;
        for (index=0;index<4;index++){
          cbp_y = ((write_data->cbp->y)>>(3-index))&1;
          cbp_u = ((write_data->cbp->u)>>(3-index))&1;
          cbp_v = ((write_data->cbp->v)>>(3-index))&1;

          /* Code cbp separately for each TU */
          int cbp = cbp_y + (cbp_u<<1) + (cbp_v<<2);
          code = cbp_table[cbp];
          if (write_data->block_context->cbp==0 && code < 2)
            code = 1-code;
          put_vlc(0,code,stream);

          /* Code coefficients for each TU separately */
          coeffq_y = write_data->coeffq_y + index*(size/2)*(size/2);
          coeffq_u = write_data->coeffq_u + index*(size/4)*(size/4);
          coeffq_v = write_data->coeffq_v + index*(size/4)*(size/4);
          if (cbp_y){
            write_coeff(stream,coeffq_y,size/2,0);
          }
          if (cbp_u){
            write_coeff(stream,coeffq_u,size/4,1);
          }
          if (cbp_v){
            write_coeff(stream,coeffq_v,size/4,1);
          }
        } //for index=
      } //if (size > 8)
      else{
        int index;
        for (index=0;index<4;index++){
          cbp_y = ((write_data->cbp->y)>>(3-index))&1;

          /* Code cbp_y separately for each TU */
          putbits(1,cbp_y,stream);

          /* Code coefficients for each TU separately */
          coeffq_y = write_data->coeffq_y + index*(size/2)*(size/2);
          if (cbp_y){
            write_coeff(stream,coeffq_y,size/2,0);
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
          write_coeff(stream,coeffq_u,size/2,1);
        }
        if (cbp_v){
          write_coeff(stream,coeffq_v,size/2,1);
        }
      } //if (size > 8)
    } //if (tb_split==0
  } //if (mode!= MODE_SKIP)

  end_bits = get_bit_pos(stream);

  return (end_bits - start_bits);
}
