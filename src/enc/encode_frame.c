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

#include "global.h"
#include <string.h>

#include "mainenc.h"
#include "encode_block.h"
#include "common_block.h"
#include "common_frame.h"


extern int chroma_qp[52];
const double squared_lambda_QP [52] = {
    0.0382, 0.0485, 0.0615, 0.0781, 0.0990, 0.1257, 0.1595, 0.2023, 0.2567,
    0.3257, 0.4132, 0.5243, 0.6652, 0.8440, 1.0709, 1.3588, 1.7240, 2.1874,
    2.7754, 3.5214, 4.4679, 5.6688, 7.1926, 9.1259, 11.5789, 14.6912, 18.6402,
    23.6505, 30.0076, 38.0735, 48.3075, 61.2922, 77.7672, 98.6706, 125.1926, 158.8437,
    201.5399, 255.7126, 324.4467, 411.6560, 522.3067, 662.6996, 840.8294, 1066.8393, 1353.5994,
    1717.4389, 2179.0763, 2764.7991, 3507.9607, 4450.8797, 5647.2498, 7165.1970
};

void clpf_frame(encoder_info_t *encoder_info){

  /* Constrained low-pass filter (CLPF) */
  stream_t *stream = encoder_info->stream;
  int width = encoder_info->width;
  int height = encoder_info->height;
  int xpos,ypos,index,filter_flag,filter;
  int k,l,x0,x1,y0,y1;
  uint8_t *recY = encoder_info->rec->y;
  uint8_t *recU = encoder_info->rec->u;
  uint8_t *recV = encoder_info->rec->v;
  uint8_t *orgY = encoder_info->orig->y;
  int stride_y = encoder_info->rec->stride_y;
  int stride_c = encoder_info->rec->stride_c;
  int num_sb_hor = width/MAX_BLOCK_SIZE;
  int num_sb_ver = height/MAX_BLOCK_SIZE;
  for (k=0;k<num_sb_ver;k++){
    for (l=0;l<num_sb_hor;l++){
      xpos = l*MAX_BLOCK_SIZE;
      ypos = k*MAX_BLOCK_SIZE;
      index = (ypos/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (xpos/MIN_PB_SIZE);
      filter_flag = encoder_info->deblock_data[index].size < 64 ||
                    encoder_info->deblock_data[index].mode != MODE_SKIP ||
                    encoder_info->deblock_data[index].mvb.x0 != 0 ||
                    encoder_info->deblock_data[index].mvb.y0 != 0;
      if (filter_flag){
        x0 = max(1,xpos);
        x1 = min(width-1,xpos+MAX_BLOCK_SIZE);
        y0 = max(1,ypos);
        y1 = min(height-1,ypos+MAX_BLOCK_SIZE);
        filter = detect_clpf(orgY,recY,x0,x1,y0,y1,stride_y);
        putbits(1,filter,stream);

        if (filter){

          /* Y */
          clpf_block(recY,x0,x1,y0,y1,stride_y);

          /* C */
          x0 = max(1,xpos/2);
          x1 = min(width/2-1,(xpos+MAX_BLOCK_SIZE)/2);
          y0 = max(1,ypos/2);
          y1 = min(height/2-1,(ypos+MAX_BLOCK_SIZE)/2);
          clpf_block(recU,x0,x1,y0,y1,stride_c);
          clpf_block(recV,x0,x1,y0,y1,stride_c);
        }
      }
    }
  }
}


void encode_frame(encoder_info_t *encoder_info)
{
  int k,l;
  int width = encoder_info->width;
  int height = encoder_info->height;  
  int num_sb_hor = (width + MAX_BLOCK_SIZE - 1)/MAX_BLOCK_SIZE;
  int num_sb_ver = (height + MAX_BLOCK_SIZE - 1)/MAX_BLOCK_SIZE;
  stream_t *stream = encoder_info->stream;

  frame_info_t *frame_info = &(encoder_info->frame_info);
  double lambda_coeff = frame_info->frame_type==I_FRAME ? encoder_info->params->lambda_coeffI :
    (frame_info->frame_type==P_FRAME ? encoder_info->params->lambda_coeffP : encoder_info->params->lambda_coeffB);
  frame_info->lambda = lambda_coeff*squared_lambda_QP[frame_info->qp];

  putbits(1,encoder_info->frame_info.frame_type!=I_FRAME,stream);
  uint8_t qp = encoder_info->frame_info.qp;
  putbits(8,(int)qp,stream);
  putbits(4,(int)encoder_info->frame_info.num_intra_modes,stream);

  int r;
  for (r=0;r<encoder_info->frame_info.num_ref;r++){
    putbits(4,encoder_info->frame_info.ref_array[r],stream);
  }

  for (k=0;k<num_sb_ver;k++){
    for (l=0;l<num_sb_hor;l++){
      int xposY = l*MAX_BLOCK_SIZE;
      int yposY = k*MAX_BLOCK_SIZE;

      int max_delta_qp = encoder_info->params->max_delta_qp;
      if (max_delta_qp){
        /* RDO-based search for best QP value */
        int cost,min_cost,best_qp,qp0,max_delta_qp,min_qp,max_qp;
        max_delta_qp = encoder_info->params->max_delta_qp;
        min_cost = 1<<30;
        stream_pos_t stream_pos_ref;
        read_stream_pos(&stream_pos_ref,stream);
        best_qp = qp;
        min_qp = qp-max_delta_qp;
        max_qp = qp;
        for (qp0=min_qp;qp0<=max_qp;qp0++){
          cost = process_block(encoder_info,MAX_BLOCK_SIZE,yposY,xposY,qp0);
          if (cost < min_cost){
            min_cost = cost;
            best_qp = qp0;
          }
        }
        write_stream_pos(stream,&stream_pos_ref);
        process_block(encoder_info,MAX_BLOCK_SIZE,yposY,xposY,best_qp);
      }
      else{
#if NEW_BLOCK_STRUCTURE
        int depth;
        int best_depth;
        uint32_t cost,min_cost;
        stream_pos_t stream_pos_ref;
        read_stream_pos(&stream_pos_ref,stream);
        best_depth = 0;
        min_cost = (1<<30);
        int max_depth = frame_info->frame_type==I_FRAME || yposY + MAX_BLOCK_SIZE > height || xposY + MAX_BLOCK_SIZE > width ? 4 : 3;
        for (depth=0;depth<max_depth;depth++){
          encoder_info->depth = depth;
          cost = process_block(encoder_info,MAX_BLOCK_SIZE,yposY,xposY,qp);
          if (cost < min_cost){
            min_cost = cost;
            best_depth = depth;
          }
        }
        write_stream_pos(stream,&stream_pos_ref);
        encoder_info->depth = best_depth;
#endif
        process_block(encoder_info,MAX_BLOCK_SIZE,yposY,xposY,qp);
      }
    }
  }

  if (encoder_info->params->deblocking){
    deblock_frame_y(encoder_info->rec, encoder_info->deblock_data, width, height, qp);
    int qpc = chroma_qp[qp];
    deblock_frame_uv(encoder_info->rec, encoder_info->deblock_data, width, height, qpc);
  }

  if (encoder_info->params->clpf){
    if ((frame_info->frame_num%CLPF_PERIOD)==0){
      clpf_frame(encoder_info);
    }
  }

  /* Sliding window operation for reference frame buffer by circular buffer */

  /* Store pointer to reference frame that is shifted out of reference buffer */
  yuv_frame_t *tmp = encoder_info->ref[MAX_REF_FRAMES-1];

  /* Update remaining pointers to implement sliding window reference buffer operation */
  memmove(encoder_info->ref+1, encoder_info->ref, sizeof(yuv_frame_t*)*(MAX_REF_FRAMES-1));

  /* Set ref[0] to the memory slot where the new current reconstructed frame wil replace reference frame being shifted out */
  encoder_info->ref[0] = tmp;

  /* Pad the reconstructed frame and write into ref[0] */
  create_reference_frame(encoder_info->ref[0],encoder_info->rec);

#if 0
  /* To test sliding window operation */
  int offsetx = 500;
  int offsety = 200;
  int offset_rec = encoder_info->rec->offset_y + offsety * encoder_info->rec->stride_y +  offsetx;
  int offset_ref = encoder_info->ref[0]->offset_y + offsety * encoder_info->ref[0]->stride_y +  offsetx;
  printf("rec: %3d ",encoder_info->rec->y[offset_rec]);
  printf("ref: ");
  for (r=0;r<MAX_REF_FRAMES;r++){
    printf("%3d ",encoder_info->ref[r]->y[offset_ref]);
  }
#endif

}


