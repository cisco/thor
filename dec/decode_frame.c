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
#include "inter_prediction.h"

extern int chroma_qp[52];

static int clpf_true(int k, int l, yuv_frame_t *r, yuv_frame_t *o, const deblock_data_t *d, int s, int w, int h, void *stream, unsigned int strength, unsigned int fb_size_log2) {
  return 1;
}

static int clpf_bit(int k, int l, yuv_frame_t *r, yuv_frame_t *o, const deblock_data_t *d, int s, int w, int h, void *stream, unsigned int strength, unsigned int fb_size_log2) {
  return get_flc(1, (stream_t*)stream);
}

static void swap_chroma_ref(decoder_info_t *decoder_info)
{
  for (int i = 0; i < MAX_REF_FRAMES; i++)
    swap_chroma(decoder_info->ref[i]);

  if (decoder_info->interp_ref)
    for (int i = 0; i < MAX_SKIP_FRAMES; i++)
      swap_chroma(decoder_info->interp_frames[i]);
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
	      decoder_info->frame_info.interp_ref = decoder_info->interp_ref;
    }
    if (decoder_info->frame_info.num_ref==2 && decoder_info->frame_info.ref_array[0]==-1) {
      decoder_info->frame_info.ref_array[decoder_info->frame_info.num_ref++] = get_flc(5, stream)-1;
    }
  } else {
    memset(decoder_info->deblock_data, 0, ((height / MIN_PB_SIZE) * (width / MIN_PB_SIZE) * sizeof(deblock_data_t)));
    decoder_info->frame_info.num_ref = 0;
  }
  decoder_info->frame_info.display_frame_num = get_flc(16, stream);
  decoder_info->frame_info.phase = decoder_info->frame_info.display_frame_num % (decoder_info->num_reorder_pics + 1);
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
    subsample_yuv_frame(decoder_info->interp_frames[0]);
    pad_yuv_frame(decoder_info->interp_frames[0]);
    decoder_info->interp_frames[0]->frame_num = display_frame_num;
  }

  decoder_info->bit_count.frame_header[decoder_info->bit_count.stat_frame_type] += (stream->bitcnt - bit_start);
  decoder_info->bit_count.frame_type[decoder_info->bit_count.stat_frame_type] += 1;
  decoder_info->frame_info.qp = qp;
  decoder_info->frame_info.qpb = qp;

  //Generate new interpolated frame
  for (k=0;k<num_sb_ver;k++){
    for (l=0;l<num_sb_hor;l++){
      int sub, adaptive_chroma;
      if (decoder_info->subsample == 444) {
        sub = adaptive_chroma = get_flc(1, stream);
        if (adaptive_chroma)
          swap_chroma_ref(decoder_info);
      } else {
	adaptive_chroma = 0;
	sub = 1;
      }

      int xposY = l*sb_size;
      int yposY = k*sb_size;
      yuv_frame_t *rec = decoder_info->rec;
      uint8_t *rec_u = &rec->u[yposY*rec->stride_c+xposY];
      uint8_t *rec_v = &rec->v[yposY*rec->stride_c+xposY];
      uint8_t *rec_u2 = &rec->u2[yposY/2*rec->stride_c2+xposY/2];
      uint8_t *rec_v2 = &rec->v2[yposY/2*rec->stride_c2+xposY/2];

      process_block_dec(decoder_info, sb_size, yposY, xposY, sub, adaptive_chroma);

      if (adaptive_chroma) {
	// Make 444 from 420
	swap_chroma_ref(decoder_info);
	for (int i = 0; i < min(yposY + MAX_SB_SIZE, height) - yposY; i += 2) {
	  int pos = i * rec->stride_c;
	  int pos2 = (i+1) * rec->stride_c;
	  if (!rec->sub) {
	    for (int j = 0; j < min(xposY + MAX_SB_SIZE, width) - xposY; j += 2) {
	      rec_u[pos+j] = rec_u[pos+1+j] = rec_u[pos2+j] = rec_u[pos2+1+j] = rec_u2[i/2*rec->stride_c2+j/2];
	      rec_v[pos+j] = rec_v[pos+1+j] = rec_v[pos2+j] = rec_v[pos2+1+j] = rec_v2[i/2*rec->stride_c2+j/2];
	    }
	  }
	}
      } else {
	// Make 420 from 444
	for (int i = 0; i < min(yposY + MAX_SB_SIZE, height) - yposY; i += 2) {
	  int pos = i * rec->stride_c;
	  int pos2 = (i+1) * rec->stride_c;
	  if (!rec->sub) {
	    for (int j = 0; j < min(xposY + MAX_SB_SIZE, width) - xposY; j += 2) {
	      rec_u2[i/2*rec->stride_c2+j/2] =
		(rec_u[pos+j] + rec_u[pos+1+j] + rec_u[pos2+j] + rec_u[pos2+1+j] + 2) >> 2;
	      rec_v2[i/2*rec->stride_c2+j/2] =
		(rec_v[pos+j] + rec_v[pos+1+j] + rec_v[pos2+j] + rec_v[pos2+1+j] + 2) >> 2;
	    }
	  }
	}
      }
    }
  }

  qp = decoder_info->frame_info.qp = decoder_info->frame_info.qpb;

  //Scale and store MVs in decode_frame()
  if (decoder_info->interp_ref>1) {
    int gop_size = decoder_info->num_reorder_pics + 1;
    int coded_phase = (decoder_info->frame_info.decode_order_frame_num + gop_size - 2) % gop_size + 1;
    int b_level = log2i(coded_phase);
    int frame_type = decoder_info->bit_count.stat_frame_type;
    int frame_num = decoder_info->frame_info.display_frame_num;
    store_mv(width, height, b_level, frame_type, frame_num, gop_size, decoder_info->deblock_data);
  }

  if (decoder_info->deblocking){
    deblock_frame_y(decoder_info->rec, decoder_info->deblock_data, width, height, qp);
    int qpc = chroma_qp[qp];
    deblock_frame_uv(decoder_info->rec, decoder_info->deblock_data, width, height, qpc);
  }

  if (decoder_info->clpf) {
    int strength = get_flc(2, stream);
    if (strength) {
      int fb_size_log2 = get_flc(2, stream) + 4;
      int enable_fb_flag = fb_size_log2 != 4;
      if (fb_size_log2 == 4)
        fb_size_log2 = 7;
      yuv_frame_t tmp = *decoder_info->rec;
      clpf_frame(decoder_info->tmp, decoder_info->rec, 0, decoder_info->deblock_data, stream, enable_fb_flag, strength + (strength == 3), fb_size_log2, enable_fb_flag ? clpf_bit : clpf_true);
      *decoder_info->rec = *decoder_info->tmp;
      *decoder_info->tmp = tmp;
      decoder_info->rec->frame_num = tmp.frame_num;
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


