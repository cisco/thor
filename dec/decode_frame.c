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

#include <string.h>

#include "global.h"
#include "decode_block.h"
#include "common_block.h"
#include "common_frame.h"

extern int chroma_qp[52];

void clpf_frame(decoder_info_t *decoder_info){

  /* Constrained low-pass filter (CLPF) */
  stream_t *stream = decoder_info->stream;
  frame_type_t frame_type = decoder_info->frame_info.frame_type;
  int height = decoder_info->height;
  int width = decoder_info->width;
  int xpos,ypos,index,filter_flag,filter;
  int k,l,x0,x1,y0,y1;
  int num_sb_hor = width/MAX_BLOCK_SIZE;
  int num_sb_ver = height/MAX_BLOCK_SIZE;

  uint8_t *recY = decoder_info->rec->y;
  uint8_t *recU = decoder_info->rec->u;
  uint8_t *recV = decoder_info->rec->v;
  int stride_y = decoder_info->rec->stride_y;
  int stride_c = decoder_info->rec->stride_c;

  int bit_start = stream->bitcnt;
  for (k=0;k<num_sb_ver;k++){
    for (l=0;l<num_sb_hor;l++){
      xpos = l*MAX_BLOCK_SIZE;
      ypos = k*MAX_BLOCK_SIZE;
      index = (ypos/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (xpos/MIN_PB_SIZE);
      filter_flag = decoder_info->deblock_data[index].size < 64 ||
                    decoder_info->deblock_data[index].mode != MODE_SKIP ||
                    decoder_info->deblock_data[index].mvb.x0 != 0 ||
                    decoder_info->deblock_data[index].mvb.y0 != 0;
      if (filter_flag){
        filter = getbits(stream,1);
        if (filter){
          /* Y */
          x0 = max(1,xpos);
          x1 = min(width-1,xpos+MAX_BLOCK_SIZE);
          y0 = max(1,ypos);
          y1 = min(height-1,ypos+MAX_BLOCK_SIZE);
          clpf_block(recY,x0,x1,y0,y1,stride_y);

          /* C */
          x0 = max(1,xpos/2);
          x1 = min(width/2-1,(xpos+MAX_BLOCK_SIZE)/2);
          y0 = max(1,ypos/2);
          y1 = min(height/2-1,(ypos+MAX_BLOCK_SIZE)/2);
          clpf_block(recU,x0,x1,y0,y1,stride_c);
          clpf_block(recV,x0,x1,y0,y1,stride_c);
        } //if (filter)
      } //if (filter_flag)
    }
  }
  decoder_info->bit_count.clpf[frame_type] += (stream->bitcnt - bit_start);
}

void decode_frame(decoder_info_t *decoder_info)
{
  int height = decoder_info->height;
  int width = decoder_info->width;
  int k,l;
  int num_sb_hor = (width + MAX_BLOCK_SIZE - 1)/MAX_BLOCK_SIZE;
  int num_sb_ver = (height + MAX_BLOCK_SIZE - 1)/MAX_BLOCK_SIZE;
  stream_t *stream = decoder_info->stream;

  int bit_start = stream->bitcnt;

  decoder_info->frame_info.frame_type = getbits(stream,1);
  int qp = getbits(stream,8);
  decoder_info->frame_info.num_intra_modes = getbits(stream,4);

  int r;
  for (r=0;r<decoder_info->frame_info.num_ref;r++){
    decoder_info->frame_info.ref_array[r] = getbits(stream,4);
  }

  decoder_info->bit_count.frame_header[decoder_info->frame_info.frame_type] += (stream->bitcnt - bit_start);
  decoder_info->bit_count.frame_type[decoder_info->frame_info.frame_type] += 1;
  decoder_info->frame_info.qp = qp;
  decoder_info->frame_info.qpb = qp;

  for (k=0;k<num_sb_ver;k++){
    for (l=0;l<num_sb_hor;l++){
      int xposY = l*MAX_BLOCK_SIZE;
      int yposY = k*MAX_BLOCK_SIZE;
      process_block_dec(decoder_info,MAX_BLOCK_SIZE,yposY,xposY);
    }
  }

  if (decoder_info->deblocking){
    deblock_frame_y(decoder_info->rec, decoder_info->deblock_data, width, height, qp);
    int qpc = chroma_qp[qp];
    deblock_frame_uv(decoder_info->rec, decoder_info->deblock_data, width, height, qpc);
  }

  if (decoder_info->clpf){
    if ((decoder_info->frame_info.display_frame_num%CLPF_PERIOD)==0){
      clpf_frame(decoder_info);
    }
  }

  /* Sliding window operation for reference frame buffer by circular buffer */

  /* Store pointer to reference frame that is shifted out of reference buffer */
  yuv_frame_t *tmp = decoder_info->ref[MAX_REF_FRAMES-1];

  /* Update remaining pointers to implement sliding window reference buffer operation */
  memmove(decoder_info->ref+1, decoder_info->ref, sizeof(yuv_frame_t*)*(MAX_REF_FRAMES-1));

  /* Set ref[0] to the memory slot where the new current reconstructed frame wil replace reference frame being shifted out */
  decoder_info->ref[0] = tmp;

  /* Pad the reconstructed frame and write into ref[0] */
  create_reference_frame(decoder_info->ref[0],decoder_info->rec);
}


