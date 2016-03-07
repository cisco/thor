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
#include "wt_matrix.h"
#include "getvlc.h"

extern int chroma_qp[52];

static int clpf_true(int k, int l, yuv_frame_t *r, yuv_frame_t *o, const deblock_data_t *d, int s, int w, int h, void *stream, unsigned int strength) {
  return 1;
}

static int clpf_bit(int k, int l, yuv_frame_t *r, yuv_frame_t *o, const deblock_data_t *d, int s, int w, int h, void *stream, unsigned int strength) {
  return get_flc(1, (stream_t*)stream);
}

void decode_frame(decoder_info_t *decoder_info, yuv_frame_t* rec_buffer)
{
  int height = decoder_info->height;
  int width = decoder_info->width;
  int k,l,r;
  int sb_size = 1 << decoder_info->log2_sb_size;
  int num_sb_hor = (width + sb_size - 1) / sb_size;
  int num_sb_ver = (height + sb_size - 1) / sb_size;
  stream_t *stream = decoder_info->stream;
  memset(decoder_info->deblock_data, 0, ((height/MIN_PB_SIZE) * (width/MIN_PB_SIZE) * sizeof(deblock_data_t)) );

  int bit_start = stream->bitcnt;
  int rec_buffer_idx;

  decoder_info->frame_info.frame_type = get_flc(1, stream);
  decoder_info->bit_count.stat_frame_type = decoder_info->frame_info.frame_type;
  int qp = get_flc(8, stream);

  decoder_info->frame_info.num_intra_modes = get_flc(4, stream);

  decoder_info->frame_info.interp_ref = 0;
  if (decoder_info->frame_info.frame_type != I_FRAME) {
    decoder_info->frame_info.num_ref = get_flc(2, stream)+1;
    int r;
    for (r=0;r<decoder_info->frame_info.num_ref;r++){
      decoder_info->frame_info.ref_array[r] = get_flc(6, stream)-1;
      if (decoder_info->frame_info.ref_array[r]==-1)
        decoder_info->frame_info.interp_ref = 1;
    }
    if (decoder_info->frame_info.num_ref==2 && decoder_info->frame_info.ref_array[0]==-1) {
      decoder_info->frame_info.ref_array[decoder_info->frame_info.num_ref++] = get_flc(5, stream)-1;
    }
  } else {
    decoder_info->frame_info.num_ref = 0;
  }
  decoder_info->frame_info.display_frame_num = get_flc(16, stream);
  for (r=0; r<decoder_info->frame_info.num_ref; ++r){
    if (decoder_info->frame_info.ref_array[r]!=-1) {
      if (decoder_info->ref[decoder_info->frame_info.ref_array[r]]->frame_num > decoder_info->frame_info.display_frame_num) {
        decoder_info->bit_count.stat_frame_type = B_FRAME;
      }
    }
  }

  rec_buffer_idx = decoder_info->frame_info.display_frame_num%MAX_REORDER_BUFFER;
  decoder_info->rec = &rec_buffer[rec_buffer_idx];
  decoder_info->tmp = &rec_buffer[MAX_REORDER_BUFFER];
  decoder_info->rec->frame_num = decoder_info->frame_info.display_frame_num;

  if (decoder_info->frame_info.num_ref>2 && decoder_info->frame_info.ref_array[0]==-1) {
    // interpolate from the other references
    yuv_frame_t* ref1=decoder_info->ref[decoder_info->frame_info.ref_array[1]];
    yuv_frame_t* ref2=decoder_info->ref[decoder_info->frame_info.ref_array[2]];
    int display_frame_num = decoder_info->frame_info.display_frame_num;
    int off1 = ref2->frame_num - display_frame_num;
    int off2 = display_frame_num - ref1->frame_num;
    if (off1 < 0 && off2 < 0) {
      off1 = -off1;
      off2 = -off2;
    }
    if (off1 == off2) {
      off1 = off2 = 1;
    }
    // FIXME: won't work for the 1-sided case
    interpolate_frames(decoder_info->interp_frames[0], ref1, ref2, off1+off2 , off2);
    pad_yuv_frame(decoder_info->interp_frames[0]);
    decoder_info->interp_frames[0]->frame_num = display_frame_num;
  }

  decoder_info->bit_count.frame_header[decoder_info->bit_count.stat_frame_type] += (stream->bitcnt - bit_start);
  decoder_info->bit_count.frame_type[decoder_info->bit_count.stat_frame_type] += 1;
  decoder_info->frame_info.qp = qp;
  decoder_info->frame_info.qpb = qp;

  for (k=0;k<num_sb_ver;k++){
    for (l=0;l<num_sb_hor;l++){
      int xposY = l*sb_size;
      int yposY = k*sb_size;
      process_block_dec(decoder_info, sb_size, yposY, xposY);
    }
  }

  qp = decoder_info->frame_info.qp = decoder_info->frame_info.qpb;

  if (decoder_info->deblocking){
    deblock_frame_y(decoder_info->rec, decoder_info->deblock_data, width, height, qp);
    int qpc = chroma_qp[qp];
    deblock_frame_uv(decoder_info->rec, decoder_info->deblock_data, width, height, qpc);
  }

  if (decoder_info->clpf && get_flc(1, stream)){
    int enable_sb_flag = !get_flc(1, stream);
    int strength = enable_sb_flag ? 2 : 1;
    yuv_frame_t tmp = *decoder_info->rec;
    clpf_frame(decoder_info->tmp, decoder_info->rec, 0, decoder_info->deblock_data, stream, enable_sb_flag, strength, enable_sb_flag ? clpf_bit : clpf_true);
    *decoder_info->rec = *decoder_info->tmp;
    *decoder_info->tmp = tmp;
    decoder_info->rec->frame_num = tmp.frame_num;
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


