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
#include "snr.h"
#include "getbits.h"
#include "getvlc.h"
#include "read_bits.h"
#include "transform.h"
#include "common_block.h"
#include "inter_prediction.h"
#include "intra_prediction.h"
#include "simd.h"

extern int chroma_qp[52];

void decode_and_reconstruct_block_intra (uint8_t *rec, int stride, int size, int qp, uint8_t *pblock, int16_t *coeffq,
    int tb_split, int upright_available,int downleft_available, intra_mode_t intra_mode,int ypos,int xpos,int width,int comp){

  int16_t *rcoeff = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *rblock = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *rblock2 = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

  uint8_t* left_data = (uint8_t*)thor_alloc(2*MAX_TR_SIZE+2,16)+1;
  uint8_t* top_data = (uint8_t*)thor_alloc(2*MAX_TR_SIZE+2,16)+1;
  uint8_t top_left;


  if (tb_split){
    int size2 = size/2;
    int i,j,index;
    for (i=0;i<size;i+=size2){
      for (j=0;j<size;j+=size2){
        make_top_and_left(left_data,top_data,&top_left,rec,stride,&rec[i*stride+j],stride,i,j,ypos,xpos,size2,upright_available,downleft_available,1);

        get_intra_prediction(left_data,top_data,top_left,ypos+i,xpos+j,size2,pblock,intra_mode);
        index = 2*(i/size2) + (j/size2);
        dequantize (coeffq+index*size2*size2,rcoeff,qp,size2);
        inverse_transform (rcoeff, rblock2, size2);
        reconstruct_block(rblock2,pblock,&rec[i*stride+j],size2,stride);
      }
    }
  }
  else{
    make_top_and_left(left_data,top_data,&top_left,rec,stride,NULL,0,0,0,ypos,xpos,size,upright_available,downleft_available,0);
    get_intra_prediction(left_data,top_data,top_left,ypos,xpos,size,pblock,intra_mode);
    dequantize (coeffq,rcoeff,qp,size);
    inverse_transform (rcoeff, rblock, size);
    reconstruct_block(rblock,pblock,rec,size,stride);
  }

  thor_free(top_data - 1);
  thor_free(left_data - 1);
  thor_free(rcoeff);
  thor_free(rblock);
  thor_free(rblock2);
}

void decode_and_reconstruct_block_inter (uint8_t *rec, int stride, int size, int qp, uint8_t *pblock, int16_t *coeffq,int tb_split){

  int16_t *rcoeff = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *rblock = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *rblock2 = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

  if (tb_split){
    int size2 = size/2;
    int i,j,k,index;
    for (i=0;i<size;i+=size2){
      for (j=0;j<size;j+=size2){
        index = 2*(i/size2) + (j/size2);
        dequantize (coeffq+index*size2*size2,rcoeff,qp,size2);
        inverse_transform (rcoeff, rblock2, size2);
        /* Copy from compact block of quarter size to full size */
        for (k=0;k<size2;k++){
          memcpy(rblock+(i+k)*size+j,rblock2+k*size2,size2*sizeof(int16_t));
        }
      }
    }
  }
  else{
    dequantize (coeffq,rcoeff,qp,size);
    inverse_transform (rcoeff, rblock, size);
  }
  reconstruct_block(rblock,pblock,rec,size,stride);

  thor_free(rcoeff);
  thor_free(rblock);
  thor_free(rblock2);
}

void copy_deblock_data(decoder_info_t *decoder_info, block_info_dec_t *block_info){

  int size = block_info->block_pos.size;
  int block_posy = block_info->block_pos.ypos/MIN_PB_SIZE;
  int block_posx = block_info->block_pos.xpos/MIN_PB_SIZE;
  int block_stride = decoder_info->width/MIN_PB_SIZE;
  int block_index;
  int m,n,m0,n0,index;
  int div = size/(2*MIN_PB_SIZE);
  int bwidth =  block_info->block_pos.bwidth;
  int bheight =  block_info->block_pos.bheight;
  uint8_t tb_split = block_info->block_param.tb_split > 0;
  part_t pb_part = block_info->block_param.mode == MODE_INTER ? block_info->block_param.pb_part : PART_NONE; //TODO: Set pb_part properly for SKIP and BIPRED

  for (m=0;m<bheight/MIN_PB_SIZE;m++){
    for (n=0;n<bwidth/MIN_PB_SIZE;n++){
      block_index = (block_posy+m)*block_stride + block_posx+n;
      m0 = div > 0 ? m/div : 0;
      n0 = div > 0 ? n/div : 0;
      index = 2*m0+n0;
      if (index > 3) printf("error: index=%4d\n",index);
      decoder_info->deblock_data[block_index].cbp = block_info->cbp;
      decoder_info->deblock_data[block_index].tb_split = tb_split;
      decoder_info->deblock_data[block_index].pb_part = pb_part;
      decoder_info->deblock_data[block_index].size = block_info->block_pos.size;

      decoder_info->deblock_data[block_index].mode = block_info->block_param.mode;
      decoder_info->deblock_data[block_index].inter_pred.mv0 = block_info->block_param.mv_arr0[index];
      decoder_info->deblock_data[block_index].inter_pred.ref_idx0 = block_info->block_param.ref_idx0;
      decoder_info->deblock_data[block_index].inter_pred.mv1 = block_info->block_param.mv_arr1[index];
      decoder_info->deblock_data[block_index].inter_pred.ref_idx1 = block_info->block_param.ref_idx1;
      decoder_info->deblock_data[block_index].inter_pred.bipred_flag = block_info->block_param.dir;
    }
  }
}

void decode_block(decoder_info_t *decoder_info,int size,int ypos,int xpos){

  int width = decoder_info->width;
  int height = decoder_info->height;
  int xposY = xpos;
  int yposY = ypos;
  int xposC = xpos/2;
  int yposC = ypos/2;
  int sizeY = size;
  int sizeC = size/2;

  block_mode_t mode;
  mv_t mv;
  intra_mode_t intra_mode;

  frame_type_t frame_type = decoder_info->frame_info.frame_type;
  int bipred = decoder_info->bipred;

  int qpY = decoder_info->frame_info.qpb;
  int qpC = chroma_qp[qpY];

  /* Intermediate block variables */
  uint8_t *pblock_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  int16_t *coeff_y = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeff_u = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);
  int16_t *coeff_v = thor_alloc(2*MAX_TR_SIZE*MAX_TR_SIZE, 16);

  /* Block variables for bipred */
  uint8_t *pblock0_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock0_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock0_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock1_y = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock1_u = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  uint8_t *pblock1_v = thor_alloc(MAX_BLOCK_SIZE*MAX_BLOCK_SIZE, 16);
  yuv_frame_t *rec = decoder_info->rec;
  yuv_frame_t *ref = decoder_info->ref[0];

  /* Calculate position in padded reference frame */
  int ref_posY = yposY*ref->stride_y+xposY;
  int ref_posC = yposC*ref->stride_c+xposC;

  /* Pointers to current position in reconstructed frame*/
  uint8_t *rec_y = &rec->y[yposY*rec->stride_y+xposY];
  uint8_t *rec_u = &rec->u[yposC*rec->stride_c+xposC];
  uint8_t *rec_v = &rec->v[yposC*rec->stride_c+xposC];

  /* Pointers to colocated block position in reference frame */
  uint8_t *ref_y = ref->y + ref_posY;
  uint8_t *ref_u = ref->u + ref_posC;
  uint8_t *ref_v = ref->v + ref_posC;

  stream_t *stream = decoder_info->stream;

  /* Read data from bitstream */
  block_info_dec_t block_info;
  block_info.block_pos.size = size;
  block_info.block_pos.ypos = ypos;
  block_info.block_pos.xpos = xpos;
  block_info.coeffq_y = coeff_y;
  block_info.coeffq_u = coeff_u;
  block_info.coeffq_v = coeff_v;

  /* Used for rectangular skip blocks */
  int bwidth = min(size,width - xpos);
  int bheight = min(size,height - ypos);
  block_info.block_pos.bwidth = bwidth;
  block_info.block_pos.bheight = bheight;

  read_block(decoder_info,stream,&block_info,frame_type);
  mode = block_info.block_param.mode;

  if (mode == MODE_INTRA){
    /* Dequantize, inverse tranform, predict and reconstruct */
    intra_mode = block_info.block_param.intra_mode;
    int upright_available = get_upright_available(ypos,xpos,size,width);
    int downleft_available = get_downleft_available(ypos,xpos,size,height);
    int tb_split = block_info.block_param.tb_split;
    decode_and_reconstruct_block_intra(rec_y,rec->stride_y,sizeY,qpY,pblock_y,coeff_y,tb_split,upright_available,downleft_available,intra_mode,yposY,xposY,width,0);
    decode_and_reconstruct_block_intra(rec_u,rec->stride_c,sizeC,qpC,pblock_u,coeff_u,tb_split&&size>8,upright_available,downleft_available,intra_mode,yposC,xposC,width/2,1);
    decode_and_reconstruct_block_intra(rec_v,rec->stride_c,sizeC,qpC,pblock_v,coeff_v,tb_split&&size>8,upright_available,downleft_available,intra_mode,yposC,xposC,width/2,2);
  }
  else
  {
    if (mode==MODE_SKIP){
      if (block_info.block_param.dir==2){
        uint8_t *ref0_y,*ref0_u,*ref0_v;
        uint8_t *ref1_y,*ref1_u,*ref1_v;

        int r0 = decoder_info->frame_info.ref_array[block_info.block_param.ref_idx0];
        yuv_frame_t *ref0 = r0>=0 ? decoder_info->ref[r0] : decoder_info->interp_frames[0];
        ref0_y = ref0->y + ref_posY;
        ref0_u = ref0->u + ref_posC;
        ref0_v = ref0->v + ref_posC;

        int r1 = decoder_info->frame_info.ref_array[block_info.block_param.ref_idx1];
        yuv_frame_t *ref1 = r1>=0 ? decoder_info->ref[r1] : decoder_info->interp_frames[0];
        ref1_y = ref1->y + ref_posY;
        ref1_u = ref1->u + ref_posC;
        ref1_v = ref1->v + ref_posC;
        int sign0 = ref0->frame_num >= rec->frame_num;
        int sign1 = ref1->frame_num >= rec->frame_num;

        mv = block_info.block_param.mv_arr0[0];
        get_inter_prediction_luma  (pblock0_y, ref0_y, bwidth,   bheight,   ref->stride_y, sizeY, &mv, sign0, bipred);
        get_inter_prediction_chroma(pblock0_u, ref0_u, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign0);
        get_inter_prediction_chroma(pblock0_v, ref0_v, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign0);
        mv = block_info.block_param.mv_arr1[0];
        get_inter_prediction_luma  (pblock1_y, ref1_y, bwidth,   bheight,   ref->stride_y, sizeY, &mv, sign1, bipred);
        get_inter_prediction_chroma(pblock1_u, ref1_u, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign1);
        get_inter_prediction_chroma(pblock1_v, ref1_v, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign1);

        int i,j;
        for (i=0;i<bheight;i++){
          for (j=0;j<bwidth;j++){
            rec_y[i*rec->stride_y+j] = (uint8_t)(((int)pblock0_y[i*sizeY+j] + (int)pblock1_y[i*sizeY+j])>>1);
          }
        }

        for (i=0;i<bheight/2;i++){
          for (j=0;j<bwidth/2;j++){
            rec_u[i*rec->stride_c+j] = (uint8_t)(((int)pblock0_u[i*sizeC+j] + (int)pblock1_u[i*sizeC+j])>>1);
            rec_v[i*rec->stride_c+j] = (uint8_t)(((int)pblock0_v[i*sizeC+j] + (int)pblock1_v[i*sizeC+j])>>1);
          }
        }
        copy_deblock_data(decoder_info,&block_info);
      }
      else{
        mv = block_info.block_param.mv_arr0[0];
        int ref_idx = block_info.block_param.ref_idx0; //TODO: Move to top
        int r = decoder_info->frame_info.ref_array[ref_idx];
        ref = r>=0 ? decoder_info->ref[r] : decoder_info->interp_frames[0];
        int sign = ref->frame_num > rec->frame_num;
        ref_y = ref->y + ref_posY;
        ref_u = ref->u + ref_posC;
        ref_v = ref->v + ref_posC;
        get_inter_prediction_luma  (pblock_y, ref_y, bwidth, bheight, ref->stride_y, sizeY, &mv, sign, bipred);
        get_inter_prediction_chroma(pblock_u, ref_u, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign);
        get_inter_prediction_chroma(pblock_v, ref_v, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign);

        int j;
        for (j=0;j<bheight;j++){
          memcpy(&rec_y[j*rec->stride_y],&pblock_y[j*sizeY],bwidth*sizeof(uint8_t));
        }
        for (j=0;j<bheight/2;j++){
          memcpy(&rec_u[j*rec->stride_c],&pblock_u[j*sizeC],(bwidth/2)*sizeof(uint8_t));
          memcpy(&rec_v[j*rec->stride_c],&pblock_v[j*sizeC],(bwidth/2)*sizeof(uint8_t));
        }
        copy_deblock_data(decoder_info,&block_info);
      }
      return;
    }
    else if (mode==MODE_MERGE){
      if (block_info.block_param.dir==2){
        uint8_t *ref0_y,*ref0_u,*ref0_v;
        uint8_t *ref1_y,*ref1_u,*ref1_v;

        int r0 = decoder_info->frame_info.ref_array[block_info.block_param.ref_idx0];
        yuv_frame_t *ref0 = r0>=0 ? decoder_info->ref[r0] : decoder_info->interp_frames[0];
        ref0_y = ref0->y + ref_posY;
        ref0_u = ref0->u + ref_posC;
        ref0_v = ref0->v + ref_posC;

        int r1 = decoder_info->frame_info.ref_array[block_info.block_param.ref_idx1];
        yuv_frame_t *ref1 = r1>=0 ? decoder_info->ref[r1] : decoder_info->interp_frames[0];
        ref1_y = ref1->y + ref_posY;
        ref1_u = ref1->u + ref_posC;
        ref1_v = ref1->v + ref_posC;

        int sign0 = ref0->frame_num >= rec->frame_num;
        int sign1 = ref1->frame_num >= rec->frame_num;

        mv = block_info.block_param.mv_arr0[0];
        get_inter_prediction_luma  (pblock0_y, ref0_y, bwidth,   bheight,   ref->stride_y, sizeY, &mv, sign0, bipred);
        get_inter_prediction_chroma(pblock0_u, ref0_u, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign0);
        get_inter_prediction_chroma(pblock0_v, ref0_v, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign0);
        mv = block_info.block_param.mv_arr1[0];
        get_inter_prediction_luma  (pblock1_y, ref1_y, bwidth,   bheight,   ref->stride_y, sizeY, &mv, sign1, bipred);
        get_inter_prediction_chroma(pblock1_u, ref1_u, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign1);
        get_inter_prediction_chroma(pblock1_v, ref1_v, bwidth/2, bheight/2, ref->stride_c, sizeC, &mv, sign1);

        int i,j;
        for (i=0;i<sizeY;i++){
          for (j=0;j<sizeY;j++){
            pblock_y[i*sizeY+j] = (uint8_t)(((int)pblock0_y[i*sizeY+j] + (int)pblock1_y[i*sizeY+j])>>1);
          }
        }
        for (i=0;i<sizeC;i++){
          for (j=0;j<sizeC;j++){
            pblock_u[i*sizeC+j] = (uint8_t)(((int)pblock0_u[i*sizeC+j] + (int)pblock1_u[i*sizeC+j])>>1);
            pblock_v[i*sizeC+j] = (uint8_t)(((int)pblock0_v[i*sizeC+j] + (int)pblock1_v[i*sizeC+j])>>1);
          }
        }
      }
      else{
        mv = block_info.block_param.mv_arr0[0];
        int ref_idx = block_info.block_param.ref_idx0; //TODO: Move to top
        int r = decoder_info->frame_info.ref_array[ref_idx];
        ref = r>=0 ? decoder_info->ref[r] : decoder_info->interp_frames[0];
        int sign = ref->frame_num > rec->frame_num;
        ref_y = ref->y + ref_posY;
        ref_u = ref->u + ref_posC;
        ref_v = ref->v + ref_posC;
        get_inter_prediction_luma  (pblock_y, ref_y, sizeY, sizeY, ref->stride_y, sizeY, &mv, sign, bipred);
        get_inter_prediction_chroma(pblock_u, ref_u, sizeC, sizeC, ref->stride_c, sizeC, &mv, sign);
        get_inter_prediction_chroma(pblock_v, ref_v, sizeC, sizeC, ref->stride_c, sizeC, &mv, sign);

      }
    }
    else if (mode == MODE_INTER){
      int index;
      int psizeY = sizeY/2;
      int psizeC = sizeC/2;
      int pstrideY = sizeY;
      int pstrideC = sizeC;
      int ref_idx = block_info.block_param.ref_idx0;
      int r = decoder_info->frame_info.ref_array[ref_idx];
      ref = r>=0 ? decoder_info->ref[r] : decoder_info->interp_frames[0];
      int sign = ref->frame_num > rec->frame_num;
      ref_y = ref->y + ref_posY;
      ref_u = ref->u + ref_posC;
      ref_v = ref->v + ref_posC;
      for (index=0;index<4;index++){
        int idx = (index>>0)&1;
        int idy = (index>>1)&1;
        int offsetpY = idy*psizeY*pstrideY + idx*psizeY;
        int offsetpC = idy*psizeC*pstrideC + idx*psizeC;
        int offsetrY = idy*psizeY*ref->stride_y + idx*psizeY;
        int offsetrC = idy*psizeC*ref->stride_c + idx*psizeC;
        mv = block_info.block_param.mv_arr0[index];
        get_inter_prediction_luma  (pblock_y + offsetpY, ref_y + offsetrY, psizeY, psizeY, ref->stride_y, pstrideY, &mv, sign, bipred);
        get_inter_prediction_chroma(pblock_u + offsetpC, ref_u + offsetrC, psizeC, psizeC, ref->stride_c, pstrideC, &mv, sign);
        get_inter_prediction_chroma(pblock_v + offsetpC, ref_v + offsetrC, psizeC, psizeC, ref->stride_c, pstrideC, &mv, sign);
      }
    }
    else if (mode == MODE_BIPRED){
      int index;
      int psizeY = sizeY/2;
      int psizeC = sizeC/2;
      int pstrideY = sizeY;
      int pstrideC = sizeC;

      uint8_t *ref0_y,*ref0_u,*ref0_v;
      uint8_t *ref1_y,*ref1_u,*ref1_v;

      int r0 = decoder_info->frame_info.ref_array[block_info.block_param.ref_idx0];
      yuv_frame_t *ref0 = r0>=0 ? decoder_info->ref[r0] : decoder_info->interp_frames[0];
      ref0_y = ref0->y + ref_posY;
      ref0_u = ref0->u + ref_posC;
      ref0_v = ref0->v + ref_posC;

      int r1 = decoder_info->frame_info.ref_array[block_info.block_param.ref_idx1];
      yuv_frame_t *ref1 = r1>=0 ? decoder_info->ref[r1] : decoder_info->interp_frames[0];
      ref1_y = ref1->y + ref_posY;
      ref1_u = ref1->u + ref_posC;
      ref1_v = ref1->v + ref_posC;

      int sign0 = ref0->frame_num >= rec->frame_num;
      int sign1 = ref1->frame_num >= rec->frame_num;

      for (index=0;index<4;index++){
        int idx = (index>>0)&1;
        int idy = (index>>1)&1;
        int offsetpY = idy*psizeY*pstrideY + idx*psizeY;
        int offsetpC = idy*psizeC*pstrideC + idx*psizeC;
        int offsetrY = idy*psizeY*ref->stride_y + idx*psizeY;
        int offsetrC = idy*psizeC*ref->stride_c + idx*psizeC;
        mv = block_info.block_param.mv_arr0[index];
        get_inter_prediction_luma  (pblock0_y + offsetpY, ref0_y + offsetrY, psizeY, psizeY, ref->stride_y, pstrideY, &mv, sign0, bipred);
        get_inter_prediction_chroma(pblock0_u + offsetpC, ref0_u + offsetrC, psizeC, psizeC, ref->stride_c, pstrideC, &mv, sign0);
        get_inter_prediction_chroma(pblock0_v + offsetpC, ref0_v + offsetrC, psizeC, psizeC, ref->stride_c, pstrideC, &mv, sign0);
        mv = block_info.block_param.mv_arr1[index];
        get_inter_prediction_luma  (pblock1_y + offsetpY, ref1_y + offsetrY, psizeY, psizeY, ref->stride_y, pstrideY, &mv, sign1, bipred);
        get_inter_prediction_chroma(pblock1_u + offsetpC, ref1_u + offsetrC, psizeC, psizeC, ref->stride_c, pstrideC, &mv, sign1);
        get_inter_prediction_chroma(pblock1_v + offsetpC, ref1_v + offsetrC, psizeC, psizeC, ref->stride_c, pstrideC, &mv, sign1);
      }
      int i,j;
      for (i=0;i<sizeY;i++){
        for (j=0;j<sizeY;j++){
          pblock_y[i*pstrideY+j] = (uint8_t)(((int)pblock0_y[i*pstrideY+j] + (int)pblock1_y[i*pstrideY+j])>>1);
        }
      }
      for (i=0;i<sizeC;i++){
        for (j=0;j<sizeC;j++){
          pblock_u[i*pstrideC+j] = (uint8_t)(((int)pblock0_u[i*pstrideC+j] + (int)pblock1_u[i*pstrideC+j])>>1);
          pblock_v[i*pstrideC+j] = (uint8_t)(((int)pblock0_v[i*pstrideC+j] + (int)pblock1_v[i*pstrideC+j])>>1);
        }
      }
    }

    /* Dequantize, invere tranform and reconstruct */
    int tb_split = block_info.block_param.tb_split;
    decode_and_reconstruct_block_inter(rec_y,rec->stride_y,sizeY,qpY,pblock_y,coeff_y,tb_split);
    decode_and_reconstruct_block_inter(rec_u,rec->stride_c,sizeC,qpC,pblock_u,coeff_u,tb_split&&size>8);
    decode_and_reconstruct_block_inter(rec_v,rec->stride_c,sizeC,qpC,pblock_v,coeff_v,tb_split&&size>8);
  }

  /* Copy deblock data to frame array */
  copy_deblock_data(decoder_info,&block_info);

  thor_free(pblock0_y);
  thor_free(pblock0_u);
  thor_free(pblock0_v);
  thor_free(pblock1_y);
  thor_free(pblock1_u);
  thor_free(pblock1_v);
  thor_free(pblock_y);
  thor_free(pblock_u);
  thor_free(pblock_v);
  thor_free(coeff_y);
  thor_free(coeff_u);
  thor_free(coeff_v);
}


int decode_super_mode(decoder_info_t *decoder_info, int size, int decode_this_size){
  stream_t *stream = decoder_info->stream;
  block_context_t *block_context = decoder_info->block_context;

  frame_type_t frame_type = decoder_info->frame_info.frame_type;
  int split_flag = 0;
  int mode = MODE_SKIP;
  int stat_mode = STAT_SKIP;
  int num_ref=0,code,maxbit;
  int idx = log2i(size)-3;

  decoder_info->mode = MODE_SKIP; //Default initial value

  if (frame_type==I_FRAME){
    decoder_info->mode = MODE_INTRA;
    if (size > MIN_BLOCK_SIZE && decode_this_size)
      split_flag = getbits(stream, 1);
    else
      split_flag = !decode_this_size;
    return split_flag;
  }
  else{
    if (!decode_this_size) {
      split_flag = !getbits(stream, 1);
      return split_flag;
    }
  }
  if (size > MAX_TR_SIZE) {
    split_flag = !getbits(stream, 1);
    if (!split_flag)  decoder_info->mode = MODE_SKIP;
    return split_flag;
  }

  num_ref = decoder_info->frame_info.num_ref;
  int bipred_possible_flag = num_ref > 1 && decoder_info->bipred;
  int split_possible_flag = size > MIN_BLOCK_SIZE;
  maxbit = 2 + num_ref + split_possible_flag + bipred_possible_flag;

  int interp_ref = decoder_info->frame_info.interp_ref;

  code = get_vlc0_limit(maxbit,stream);

  if (interp_ref) {

    if ((block_context->index == 2 || block_context->index>3) && size>MIN_BLOCK_SIZE){
      /* Move skip down the list */
      if (code<3)
        code = (code + 1) % 3;
    }

    if (split_possible_flag && code==1) {
      /* Set split flag and return */
      split_flag = 1;
      decoder_info->bit_count.super_mode_stat[decoder_info->bit_count.stat_frame_type][idx][STAT_SPLIT] += 1;
      return split_flag;
    }

    if (!split_possible_flag && code > 0) {
      /* Didn't need a codeword for split so adjust for the empty slot */
      code += 1;
    }

    if (!bipred_possible_flag && code >= 3) {
      /* Don't need a codeword for bipred so adjust for the empty slot */
      code += 1;
    }

    if (code == 0) {
      mode = MODE_SKIP;
      stat_mode = STAT_SKIP;
    } else if (code == 2) {
      mode = MODE_MERGE;
      stat_mode = STAT_MERGE;
    } else if (code == 3) {
      mode = MODE_BIPRED;
      stat_mode = STAT_BIPRED;
    } else if (code == 4) {
      mode = MODE_INTRA;
      stat_mode = STAT_INTRA;
    } else if (code == 4 + num_ref) {
      mode = MODE_INTER;
      decoder_info->ref_idx = 0;
      stat_mode = STAT_REF_IDX0;
    }
    else{
      mode = MODE_INTER;
      decoder_info->ref_idx = code - 4;
      stat_mode = STAT_REF_IDX1 + decoder_info->ref_idx-1;
    }
    decoder_info->mode = mode;


  } else {
    if ((block_context->index == 2 || block_context->index>3) && size>MIN_BLOCK_SIZE){
      /* Skip is less likely than split, merge and inter-ref_idx=0 so move skip down the list */
      if (code<4)
        code = (code + 1) % 4;
    }

    if (split_possible_flag && code==1) {
      /* Set split flag and return */
      split_flag = 1;
      decoder_info->bit_count.super_mode_stat[decoder_info->bit_count.stat_frame_type][idx][STAT_SPLIT] += 1;
      return split_flag;
    }

    if (!split_possible_flag && code > 0) {
      /* Didn't need a codeword for split so adjust for the empty slot */
      code += 1;
    }

    if (!bipred_possible_flag && code >= 4) {
      /* Don't need a codeword for bipred so adjust for the empty slot */
      code += 1;
    }

    if (code == 0) {
      mode = MODE_SKIP;
      stat_mode = STAT_SKIP;
    }
    else if (code == 2) {
      mode = MODE_INTER;
      decoder_info->ref_idx = 0;
      stat_mode = STAT_REF_IDX0;
    }
    else if (code == 3) {
      mode = MODE_MERGE;
      stat_mode = STAT_MERGE;
    }
    else if (code == 4) {
      mode = MODE_BIPRED;
      stat_mode = STAT_BIPRED;
    }
    else if (code == 5) {
      mode = MODE_INTRA;
      stat_mode = STAT_INTRA;
    }
    else{
      mode = MODE_INTER;
      decoder_info->ref_idx = code - 5;
      stat_mode = STAT_REF_IDX1 + decoder_info->ref_idx - 1;
    }
    decoder_info->mode = mode;

  }
  decoder_info->bit_count.super_mode_stat[decoder_info->bit_count.stat_frame_type][idx][stat_mode] += 1;

  return split_flag;
}


void process_block_dec(decoder_info_t *decoder_info,int size,int yposY,int xposY)
{
  int width = decoder_info->width;
  int height = decoder_info->height;
  stream_t *stream = decoder_info->stream;
  frame_type_t frame_type = decoder_info->frame_info.frame_type;
  int split_flag = 0;

  if (yposY >= height || xposY >= width)
    return;

  int decode_this_size = (yposY + size <= height) && (xposY + size <= width);
  int decode_rectangular_size = !decode_this_size && frame_type != I_FRAME;

  int bit_start = stream->bitcnt;

  int mode = MODE_SKIP;
 
  block_context_t block_context;
  find_block_contexts(yposY, xposY, height, width, size, decoder_info->deblock_data, &block_context, decoder_info->use_block_contexts);
  decoder_info->block_context = &block_context;

  split_flag = decode_super_mode(decoder_info,size,decode_this_size);
  mode = decoder_info->mode;
  
  /* Read delta_qp and set block-level qp */
  if (size==MAX_BLOCK_SIZE && (split_flag || mode != MODE_SKIP) && decoder_info->max_delta_qp > 0){
    /* Read delta_qp */
    int delta_qp = read_delta_qp(stream);
    decoder_info->frame_info.qpb = decoder_info->frame_info.qp + delta_qp;
  }

  decoder_info->bit_count.super_mode[decoder_info->bit_count.stat_frame_type] += (stream->bitcnt - bit_start);

  if (split_flag){
    int new_size = size/2;
    process_block_dec(decoder_info,new_size,yposY+0*new_size,xposY+0*new_size);
    process_block_dec(decoder_info,new_size,yposY+1*new_size,xposY+0*new_size);
    process_block_dec(decoder_info,new_size,yposY+0*new_size,xposY+1*new_size);
    process_block_dec(decoder_info,new_size,yposY+1*new_size,xposY+1*new_size);
  }
  else if (decode_this_size || decode_rectangular_size){
    decode_block(decoder_info,size,yposY,xposY);
  }
}

