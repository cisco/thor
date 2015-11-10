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
#include "temporal_interp.h"

extern int chroma_qp[52];

static int clpf_true(int k, int l, yuv_frame_t *r, yuv_frame_t *o, const deblock_data_t *d, int s, void *stream) {
  return 1;
}

static int clpf_bit(int k, int l, yuv_frame_t *r, yuv_frame_t *o, const deblock_data_t *d, int s, void *stream) {
  return getbits((stream_t*)stream, 1);
}

void decode_frame(decoder_info_t *decoder_info, yuv_frame_t* rec_buffer)
{
  int height = decoder_info->height;
  int width = decoder_info->width;
  int k,l,r;
  int num_sb_hor = (width + MAX_BLOCK_SIZE - 1)/MAX_BLOCK_SIZE;
  int num_sb_ver = (height + MAX_BLOCK_SIZE - 1)/MAX_BLOCK_SIZE;
  stream_t *stream = decoder_info->stream;
  memset(decoder_info->deblock_data, 0, ((height/MIN_PB_SIZE) * (width/MIN_PB_SIZE) * sizeof(deblock_data_t)) );

  int bit_start = stream->bitcnt;
  int rec_buffer_idx;

  decoder_info->frame_info.frame_type = getbits(stream,1);
  decoder_info->bit_count.stat_frame_type = decoder_info->frame_info.frame_type;
  int qp = getbits(stream,8);
  decoder_info->frame_info.num_intra_modes = getbits(stream,4);

  decoder_info->frame_info.interp_ref = 0;
  if (decoder_info->frame_info.frame_type != I_FRAME) {
    decoder_info->frame_info.num_ref = getbits(stream,2)+1;
    int r;
    for (r=0;r<decoder_info->frame_info.num_ref;r++){
      decoder_info->frame_info.ref_array[r] = getbits(stream,6)-1;
      if (decoder_info->frame_info.ref_array[r]==-1)
        decoder_info->frame_info.interp_ref = 1;
    }
    if (decoder_info->frame_info.num_ref==2 && decoder_info->frame_info.ref_array[0]==-1) {
      decoder_info->frame_info.ref_array[decoder_info->frame_info.num_ref++] = getbits(stream,5)-1;
    }
  } else {
    decoder_info->frame_info.num_ref = 0;
  }
  decoder_info->frame_info.display_frame_num = getbits(stream,16);
  for (r=0; r<decoder_info->frame_info.num_ref; ++r){
    if (decoder_info->frame_info.ref_array[r]!=-1) {
      if (decoder_info->ref[decoder_info->frame_info.ref_array[r]]->frame_num > decoder_info->frame_info.display_frame_num) {
        decoder_info->bit_count.stat_frame_type = B_FRAME;
      }
    }
  }

  rec_buffer_idx = decoder_info->frame_info.display_frame_num%MAX_REORDER_BUFFER;
  decoder_info->rec = &rec_buffer[rec_buffer_idx];
  decoder_info->rec->frame_num = decoder_info->frame_info.display_frame_num;

  if (decoder_info->frame_info.num_ref>2 && decoder_info->frame_info.ref_array[0]==-1) {
    // interpolate from the other references
    yuv_frame_t* ref1=decoder_info->ref[decoder_info->frame_info.ref_array[1]];
    yuv_frame_t* ref2=decoder_info->ref[decoder_info->frame_info.ref_array[2]];
    if (decoder_info->dyadic_coding)
      interpolate_frames(decoder_info->interp_frames[0], ref1, ref2, 2 , 1);
    else if (decoder_info->num_reorder_pics>0){
      int sub_gop = decoder_info->num_reorder_pics+1;
      int phase = (decoder_info->frame_info.decode_order_frame_num + sub_gop - 2) % sub_gop;
      interpolate_frames(decoder_info->interp_frames[0], ref1, ref2, sub_gop-phase, phase!=0 ? 1 : sub_gop-phase-1);
    }
    pad_yuv_frame(decoder_info->interp_frames[0]);
    decoder_info->interp_frames[0]->frame_num = decoder_info->frame_info.display_frame_num;
  }

  decoder_info->bit_count.frame_header[decoder_info->bit_count.stat_frame_type] += (stream->bitcnt - bit_start);
  decoder_info->bit_count.frame_type[decoder_info->bit_count.stat_frame_type] += 1;
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

  if (decoder_info->clpf && getbits(stream, 1)){
    clpf_frame(decoder_info->rec, 0, decoder_info->deblock_data, stream,
               getbits(stream, 1) ? clpf_true : clpf_bit);
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


