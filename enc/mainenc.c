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
#include "common_frame.h"
#include "encode_frame.h"
#include "putbits.h"
#include "putvlc.h"
#include "transform.h"
#include "temporal_interp.h"
#include "../common/simd.h"
#include "rc.h"
#include "wt_matrix.h"
#include "write_bits.h"

// Coding order to display order
static const int cd1[1] = {0};
static const int cd2[2] = {1,0};
static const int cd4[4] = {3,1,0,2};
static const int cd8[8] = {7,3,1,5,0,2,4,6};
static const int cd16[16] = {15,7,3,11,1,5,9,13,0,2,4,6,8,10,12,14};
static const int* dyadic_reorder_code_to_display[5] = {cd1,cd2,cd4,cd8,cd16};

// Display order to coding order
static const int dc1[1+1] = {-1,0};
static const int dc2[2+1] = {-2,1,0};
static const int dc4[4+1] = {-4,2,1,3,0};
static const int dc8[8+1] = {-8,4,2,5,1,6,3,7,0};
static const int dc16[16+1] = {-16,8,4,9,2,10,5,11,1,12,6,13,3,14,7,15,0};
static const int* dyadic_reorder_display_to_code[5] = {dc1,dc2,dc4,dc8,dc16};

#undef TEMPLATE
#define TEMPLATE(func) (encoder_info.params->frame_bitdepth == 8 ? func ## _lbd : func ## _hbd)

static int reorder_frame_offset(int idx, int sub_gop, int dyadic)
{
  if (dyadic && sub_gop>1) {
    return dyadic_reorder_code_to_display[log2i(sub_gop)][idx]-sub_gop+1;
  } else {
    if (idx==0) return 0;
    else return idx-sub_gop;
  }
}

int main(int argc, char **argv)
{
  FILE *infile, *strfile, *reconfile;

  long input_file_size;
  yuv_frame_t orig,ref[MAX_REF_FRAMES];
  yuv_frame_t rec[MAX_REORDER_BUFFER+1];  // Last one is for temp use
  int rec_available[MAX_REORDER_BUFFER] = {0};
  int last_frame_output=-1;
  int num_encoded_frames,num_bits,start_bits,end_bits;
  int sub_gop=1;
  int rec_buffer_idx;
  int k,frame_num,frame_num0,r;
  int frame_offset;
  int ysize,csize;
  long frame_size;
  int width,height;
  int min_interp_depth;
  int last_intra_frame_num = 0;
  uint32_t acc_num_bits;
  snrvals psnr;
  snrvals accsnr;
  double bit_rate_in_kbps;
  enc_params *params;
  encoder_info_t encoder_info;

  int y4m_output;
  // Keep track of last P frame for using the right references for the tail of a sequence in re-ordered modes
  int last_PorI_frame;

  init_use_simd();

  /* Read commands from command line and from configuration file(s) */
  if (argc < 3)
  {
    fprintf(stdout,"usage: %s <parameters>\n",argv[0]);
    fatalerror("");
  }
  params = parse_config_params(argc, argv);
  if (params == NULL)
  {
    fatalerror("Error while reading encoder paramaters.");
  }
  check_parameters(params);

  /* Open files */
  if (!(infile = fopen(params->infilestr,"rb")))
  {
    fatalerror("Could not open in-file for reading.");
  }
  if (!(strfile = fopen(params->outfilestr,"wb")))
  {
    fatalerror("Could not open out-file for writing.");
  }
  reconfile = NULL;
  y4m_output = 0;
  if (params->reconfilestr) {
    char *p;
    if (!(reconfile = fopen(params->reconfilestr,"wb")))
    {
      fatalerror("Could not open recon-file for reading.");
    }
    p = strrchr(params->reconfilestr,'.');
    y4m_output = p != NULL && strcmp(p,".y4m") == 0;
  }
  
  fseek(infile, 0, SEEK_END);
  input_file_size = ftell(infile);
  fseek(infile, 0, SEEK_SET);

  if (y4m_output) {
    fprintf(reconfile,
            "YUV4MPEG2 W%d H%d F%d:1 Ip A%d:%d C",
            params->width, params->height, (int)params->frame_rate, params->aspectnum, params->aspectden);
    if (params->subsample == 400)
      fprintf(reconfile, "mono");
    else
      fprintf(reconfile, "%d", params->subsample);

    if (params->input_bitdepth > 8)
      fprintf(reconfile, "p%d XYSCSS=%dp%d", params->input_bitdepth, params->subsample, params->input_bitdepth);
    fprintf(reconfile, "\x0a");
  }

  accsnr.y = 0;
  accsnr.u = 0;
  accsnr.v = 0;
  acc_num_bits = 0;

  height = params->height;
  width = params->width;
  ysize = height * width;
  csize = ((ysize >> 2*(params->subsample != 444)) << (params->subsample == 422)) * (params->subsample != 400);
  frame_size = (ysize + 2*csize) * (1 + (params->input_bitdepth > 8));
  encoder_info.params = params;

  /* Create frames*/
  TEMPLATE(create_yuv_frame)(&orig,width,height,params->subsample,0,0,params->bitdepth,params->input_bitdepth);
  for (r=0;r<MAX_REORDER_BUFFER+1;r++){
    TEMPLATE(create_yuv_frame)(&rec[r],width,height,params->subsample,0,0,params->bitdepth,params->input_bitdepth);
  }
  for (r=0;r<MAX_REF_FRAMES;r++){ //TODO: Use Long-term frame instead of a large sliding window
    TEMPLATE(create_yuv_frame)(&ref[r],width,height,params->subsample,PADDING_Y,PADDING_Y,params->bitdepth,params->input_bitdepth);
  }
  if (params->interp_ref) {
    for (r=0;r<MAX_SKIP_FRAMES;r++){
      encoder_info.interp_frames[r] = malloc(sizeof(yuv_frame_t));
      TEMPLATE(create_yuv_frame)(encoder_info.interp_frames[r],width,height,params->subsample,PADDING_Y,PADDING_Y,params->bitdepth,params->input_bitdepth);
    }
  }

  /* Initialize main bit stream */
  stream_t stream;
  stream.bitstream = (uint8_t *)malloc(MAX_BUFFER_SIZE * sizeof(uint8_t));
  stream.bitbuf = 0;
  stream.bitrest = 32;
  stream.bytepos = 0;
  stream.bytesize = MAX_BUFFER_SIZE;

  /* Configure encoder */
  encoder_info.orig = &orig;
  for (r=0;r<MAX_REF_FRAMES;r++){
    encoder_info.ref[r] = &ref[r];
  }
  encoder_info.stream = &stream;
  encoder_info.width = width;
  encoder_info.height = height;
  encoder_info.frame_info.max_clpf_strength = encoder_info.params->max_clpf_strength;

  encoder_info.deblock_data = (deblock_data_t *)malloc((height/MIN_PB_SIZE) * (width/MIN_PB_SIZE) * sizeof(deblock_data_t));

  alloc_wmatrices(encoder_info.wmatrix, 0);
  alloc_wmatrices(encoder_info.iwmatrix, 1);

  /* Write sequence header */ //TODO: Separate function for sequence header
  start_bits = get_bit_pos(&stream);
  write_sequence_header(&stream, params);

  end_bits = get_bit_pos(&stream);
  num_bits = end_bits-start_bits;
  acc_num_bits += num_bits;
  printf("SH:  %4d bits\n",num_bits);

  /* Start encoding sequence */
  num_encoded_frames = 0;
  sub_gop = max(1,params->num_reorder_pics+1);

  min_interp_depth = log2i(params->num_reorder_pics+1)-3;
  if (params->frame_rate > 30) min_interp_depth--;

  last_PorI_frame = -1;

  rate_control_t rc;
  encoder_info.rc = &rc;
  if (params->bitrate > 0) {
    int target_bits = (int)(params->bitrate / params->frame_rate);
    int sb_size = 1 << params->log2_sb_size;
    int num_sb = ((width + sb_size - 1) / sb_size) * ((height + sb_size - 1) / sb_size);
    init_rate_control_per_sequence(&rc, target_bits, num_sb);
  }

  for (frame_num0 = params->skip; frame_num0 < (params->skip + params->num_frames) && (frame_num0+1)*frame_size <= input_file_size; frame_num0+=sub_gop)
  {
    for (k=0; k<sub_gop; k++) {
      int r,r1,r2,r3;
      /* Initialize frame info */
      frame_offset = reorder_frame_offset(k,sub_gop,params->dyadic_coding);
      frame_num = frame_num0 + frame_offset;
      // If there is an initial I frame and reordering need to jump to the next P frame
      if (frame_num<params->skip) continue;

      encoder_info.frame_info.frame_num = frame_num - params->skip;
      rec_buffer_idx = encoder_info.frame_info.frame_num%MAX_REORDER_BUFFER;
      encoder_info.rec = &rec[rec_buffer_idx];
      encoder_info.tmp = &rec[MAX_REORDER_BUFFER];
      encoder_info.rec->frame_num = encoder_info.frame_info.frame_num;
      if (params->num_reorder_pics==0) {
        if (params->intra_period > 0)
          encoder_info.frame_info.frame_type = ((num_encoded_frames%params->intra_period) == 0 ? I_FRAME : P_FRAME);
        else
          encoder_info.frame_info.frame_type = (num_encoded_frames == 0 ? I_FRAME : P_FRAME);
      } else {
        if (params->intra_period > 0)
          encoder_info.frame_info.frame_type = ((encoder_info.frame_info.frame_num%params->intra_period) == 0 ? I_FRAME :
              ((encoder_info.frame_info.frame_num%sub_gop)==0 ? P_FRAME : B_FRAME));
        else
          encoder_info.frame_info.frame_type = (encoder_info.frame_info.frame_num == 0 ? I_FRAME :
              ((encoder_info.frame_info.frame_num%sub_gop)==0 ? P_FRAME : B_FRAME));
      }

      int coded_phase = (num_encoded_frames + sub_gop - 2) % sub_gop + 1;
      int b_level = log2i(coded_phase);
      encoder_info.frame_info.b_level = b_level;

      encoder_info.frame_info.phase = encoder_info.frame_info.frame_num % (encoder_info.params->num_reorder_pics + 1);

      if (encoder_info.frame_info.frame_type == I_FRAME){
        encoder_info.frame_info.qp = params->qp + params->dqpI;
        last_intra_frame_num = encoder_info.frame_info.frame_num;
      }
      else if (params->num_reorder_pics==0) {
        if (num_encoded_frames % params->HQperiod)
          encoder_info.frame_info.qp = (int)(params->mqpP*(float)params->qp) + params->dqpP;
        else
          encoder_info.frame_info.qp = params->qp;
      } else {
        if (encoder_info.frame_info.frame_num % sub_gop) {
          if (params->dyadic_coding){
            if (b_level == 0)
              encoder_info.frame_info.qp = (int)(params->mqpB0*(float)params->qp) + params->dqpB0;
            else if (b_level == 1)
              encoder_info.frame_info.qp = (int)(params->mqpB1*(float)params->qp) + params->dqpB1;
            else if (b_level == 2)
              encoder_info.frame_info.qp = (int)(params->mqpB2*(float)params->qp) + params->dqpB2;
            else if (b_level == 3)
              encoder_info.frame_info.qp = (int)(params->mqpB3*(float)params->qp) + params->dqpB3;
            else
              encoder_info.frame_info.qp = (int)(params->mqpB*(float)params->qp) + params->dqpB;
          }
          else {
            encoder_info.frame_info.qp = (int)(params->mqpB*(float)params->qp) + params->dqpB;
          }
        }  else {
          if (encoder_info.frame_info.frame_num % params->HQperiod) {
            encoder_info.frame_info.qp = (int)(params->mqpP*(float)params->qp) + params->dqpP;
          } else
            encoder_info.frame_info.qp = params->qp;
        }
      }
      encoder_info.frame_info.qp = clip(encoder_info.frame_info.qp, 0, MAX_QP);

      encoder_info.frame_info.num_ref = encoder_info.frame_info.frame_type == I_FRAME ? 0 : min(num_encoded_frames,params->max_num_ref);
      encoder_info.frame_info.interp_ref = 0;

      if (encoder_info.frame_info.num_ref > 0) {
        if (params->num_reorder_pics > 0) {
          if (params->dyadic_coding) {
            /* if we have a P frame then use the previous P frame as a reference */
            if ((num_encoded_frames-1) % sub_gop == 0) {
              if (num_encoded_frames==1)
                encoder_info.frame_info.ref_array[0] = 0;
              else
                encoder_info.frame_info.ref_array[0] = sub_gop-1;
              if (encoder_info.frame_info.num_ref>1 )
                encoder_info.frame_info.ref_array[1] = min(MAX_REF_FRAMES-1,min(num_encoded_frames-1,2*sub_gop-1));

              for (r=2;r<encoder_info.frame_info.num_ref;r++){
                encoder_info.frame_info.ref_array[r] = r-2;
              }

            } else if (encoder_info.frame_info.num_ref>0){

              int display_phase =  (encoder_info.frame_info.frame_num-1) % sub_gop;
              int ref_offset=sub_gop>>(b_level+1);
              if (b_level >= min_interp_depth && params->interp_ref == 1) {
                // Need to add another reference if we are at the beginning
                if (encoder_info.frame_info.num_ref==2) encoder_info.frame_info.num_ref++;

                encoder_info.frame_info.interp_ref = params->interp_ref;

                encoder_info.frame_info.ref_array[1]=min(num_encoded_frames-1,coded_phase-dyadic_reorder_display_to_code[log2i(sub_gop)][display_phase-ref_offset+1]-1);
                encoder_info.frame_info.ref_array[2]=min(num_encoded_frames-1,coded_phase-dyadic_reorder_display_to_code[log2i(sub_gop)][display_phase+ref_offset+1]-1);

                // Interpolate these two reference frames to make a new frame
                encoder_info.frame_info.ref_array[0]=-1;
                // Add this interpolated frame to the reference buffer and use it as the first reference
                yuv_frame_t* ref1=encoder_info.ref[encoder_info.frame_info.ref_array[1]];
                yuv_frame_t* ref2=encoder_info.ref[encoder_info.frame_info.ref_array[2]];
                TEMPLATE(interpolate_frames)(encoder_info.interp_frames[0], ref1, ref2, 2, 1);
                TEMPLATE(pad_yuv_frame)(encoder_info.interp_frames[0]);
                encoder_info.interp_frames[0]->frame_num = encoder_info.frame_info.frame_num;
                /* use most recent frames for the last ref(s)*/
                for (r=3;r<encoder_info.frame_info.num_ref;r++){
                  encoder_info.frame_info.ref_array[r] = r-3;
                }
              } else {
                encoder_info.frame_info.ref_array[0]=min(num_encoded_frames-1,coded_phase-dyadic_reorder_display_to_code[log2i(sub_gop)][display_phase-ref_offset+1]-1);
                encoder_info.frame_info.ref_array[1]=min(num_encoded_frames-1,coded_phase-dyadic_reorder_display_to_code[log2i(sub_gop)][display_phase+ref_offset+1]-1);

                /* use most recent frames for the last ref(s)*/
                for (r=2;r<encoder_info.frame_info.num_ref;r++){
                  encoder_info.frame_info.ref_array[r] = r-2;
                }

              }
            }
          } else {
            /* if we have a P frame then use the previous P frame as a reference */
            if ((num_encoded_frames-1) % sub_gop == 0) {
              if (num_encoded_frames==1)
                encoder_info.frame_info.ref_array[0] = 0;
              else
                encoder_info.frame_info.ref_array[0] = sub_gop-1;
              if (encoder_info.frame_info.num_ref>1 )
                encoder_info.frame_info.ref_array[1] = min(MAX_REF_FRAMES-1,min(num_encoded_frames-1,2*sub_gop-1));

              for (r=2;r<encoder_info.frame_info.num_ref;r++){
                encoder_info.frame_info.ref_array[r] = r-1;
              }

            } else {
              if (params->interp_ref > 0 && params->interp_ref == 1) {
                // Need to add another reference if we are at the beginning
                if (encoder_info.frame_info.num_ref==2) encoder_info.frame_info.num_ref++;

                encoder_info.frame_info.interp_ref = params->interp_ref;

                // Use the last encoded frame as the first true ref
                if (encoder_info.frame_info.num_ref>0) {
                  encoder_info.frame_info.ref_array[1] = 0;
                }
                /* Use the subsequent P frame as the 2nd ref */
                int phase = (num_encoded_frames + sub_gop - 2) % sub_gop;
                if (encoder_info.frame_info.num_ref>1) {
                  if (phase==0)
                    encoder_info.frame_info.ref_array[2] = min(sub_gop, num_encoded_frames-1);
                  else
                    encoder_info.frame_info.ref_array[2] = min(phase, num_encoded_frames-1);
                }
                // Interpolate these two reference frames to make a new frame
                encoder_info.frame_info.ref_array[0]=-1;
                // Add this interpolated frame to the reference buffer and use it as the first reference
                yuv_frame_t* ref1=encoder_info.ref[encoder_info.frame_info.ref_array[1]];
                yuv_frame_t* ref2=encoder_info.ref[encoder_info.frame_info.ref_array[2]];
                TEMPLATE(interpolate_frames)(encoder_info.interp_frames[0], ref1, ref2, sub_gop-phase,phase!=0 ? 1 : sub_gop-phase-1);
                TEMPLATE(pad_yuv_frame)(encoder_info.interp_frames[0]);
                encoder_info.interp_frames[0]->frame_num = encoder_info.frame_info.frame_num;

                /* Use the prior P frame as the 4th ref */
                if (encoder_info.frame_info.num_ref>2) {
                  encoder_info.frame_info.ref_array[3] = min(phase ? phase + sub_gop : 2*sub_gop, num_encoded_frames-1);
                }
                /* use most recent frames for the last ref(s)*/
                for (r=4;r<encoder_info.frame_info.num_ref;r++){
                  encoder_info.frame_info.ref_array[r] = r-4+1;
                }


              } else {
                // Use the last encoded frame as the first ref
                if (encoder_info.frame_info.num_ref>0) {
                  encoder_info.frame_info.ref_array[0] = 0;
                }
                /* Use the subsequent P frame as the 2nd ref */
                int phase = (num_encoded_frames + sub_gop - 2) % sub_gop;
                if (encoder_info.frame_info.num_ref>1) {
                  if (phase==0)
                    encoder_info.frame_info.ref_array[1] = min(sub_gop, num_encoded_frames-1);
                  else
                    encoder_info.frame_info.ref_array[1] = min(phase, num_encoded_frames-1);
                }
                /* Use the prior P frame as the 3rd ref */
                if (encoder_info.frame_info.num_ref>2) {
                  encoder_info.frame_info.ref_array[2] = min(phase ? phase + sub_gop : 2*sub_gop, num_encoded_frames-1);
                }
                /* use most recent frames for the last ref(s)*/
                for (r=3;r<encoder_info.frame_info.num_ref;r++){
                  encoder_info.frame_info.ref_array[r] = r-3+1;
                }
              }
            }
          }

          if (encoder_info.params->num_reorder_pics == 2 && encoder_info.frame_info.frame_type == B_FRAME && b_level == 0) {
            int off = encoder_info.params->interp_ref == 1 ? 1 : 0;
            int tmp = encoder_info.frame_info.ref_array[0 + off];
            encoder_info.frame_info.ref_array[0 + off] = encoder_info.frame_info.ref_array[1 + off];
            encoder_info.frame_info.ref_array[1 + off] = tmp;
          }

        } else {
          if (encoder_info.frame_info.num_ref>=1){
            /* If num_ref==1 always use most recent frame */
            encoder_info.frame_info.ref_array[0] = last_PorI_frame;
          }

          if (encoder_info.frame_info.num_ref==2){
            /* If num_ref==2 use most recent LQ frame and most recent HQ frame */
            r1 = ((num_encoded_frames + params->HQperiod - 2) % params->HQperiod) + 1;
            encoder_info.frame_info.ref_array[1] = r1;
          }
          else if (encoder_info.frame_info.num_ref==3){
            r1 = ((num_encoded_frames + params->HQperiod - 2) % params->HQperiod) + 1;
            r2 = r1==1 ? 2 : 1;
            encoder_info.frame_info.ref_array[1] = r1;
            encoder_info.frame_info.ref_array[2] = r2;
          }
          else if (encoder_info.frame_info.num_ref==4){
            r1 = ((num_encoded_frames + params->HQperiod - 2) % params->HQperiod) + 1;
            r2 = r1==1 ? 2 : 1;
            r3 = r2+1;
            if (r3==r1) r3 += 1;
            encoder_info.frame_info.ref_array[1] = r1;
            encoder_info.frame_info.ref_array[2] = r2;
            encoder_info.frame_info.ref_array[3] = r3;
          }
          else{
            for (r=1;r<encoder_info.frame_info.num_ref;r++){
              encoder_info.frame_info.ref_array[r] = r;
            }
          }
        }
      }

      // Remove duplicate reference frames
      for (r=encoder_info.frame_info.num_ref-1; r>0; --r){
        for (int k=r-1; k>=0; --k) {
          if (encoder_info.frame_info.ref_array[k] == encoder_info.frame_info.ref_array[r]) {
            // remove rth element
            for (int s=r; s<encoder_info.frame_info.num_ref-1; ++s) {
              encoder_info.frame_info.ref_array[s]=encoder_info.frame_info.ref_array[s+1];
            }
            encoder_info.frame_info.num_ref--;
            break;
          }

        }
      }

      // Remove reference frames which break random access
      if (encoder_info.frame_info.frame_num > last_intra_frame_num) {
        for (r=encoder_info.frame_info.num_ref-1; r>=0; --r){
          if (encoder_info.frame_info.ref_array[r]>=0) {
            int ref_frame_num=encoder_info.ref[encoder_info.frame_info.ref_array[r]]->frame_num;
            if (ref_frame_num < last_intra_frame_num) {
              // remove this reference
              for (int s=r; s<encoder_info.frame_info.num_ref-1; ++s) {
                encoder_info.frame_info.ref_array[s]=encoder_info.frame_info.ref_array[s+1];
              }
              encoder_info.frame_info.num_ref--;
            }
          }
        }
      }

      if (params->intra_rdo == 0 || (encoder_info.frame_info.frame_type != I_FRAME && params->encoder_speed > 0))
        encoder_info.frame_info.num_intra_modes = 4;
      else
        encoder_info.frame_info.num_intra_modes = MAX_NUM_INTRA_MODES;

#if 0
      /* To test sliding window operation */
      int offsetx = 500;
      int offsety = 200;
      int offset_rec = offsety * encoder_info.rec->stride_y +  offsetx;
      int offset_ref = offsety * encoder_info.ref[0]->stride_y +  offsetx;
      if (encoder_info.frame_info.num_ref==2){
        int r0 = encoder_info.frame_info.ref_array[0];
        int r1 = encoder_info.frame_info.ref_array[1];
        printf("ref0=%3d ref1=%3d ",encoder_info.ref[r0]->y[offset_ref],encoder_info.ref[r1]->y[offset_ref]);
      }
      else{
        printf("ref0=XXX ref1=XXX ");
      }
#endif

      /* Read input frame */
      fseek(infile, frame_num*(frame_size+params->frame_headerlen)+params->file_headerlen+params->frame_headerlen, SEEK_SET);
      TEMPLATE(read_yuv_frame)(&orig,infile);
      orig.frame_num = encoder_info.frame_info.frame_num;

      /* Encode frame */
      start_bits = get_bit_pos(&stream);
      TEMPLATE(encode_frame)(&encoder_info);

      rec_available[rec_buffer_idx]=1;
      end_bits =  get_bit_pos(&stream);
      num_bits = end_bits-start_bits;
      num_encoded_frames++;

      /* Compute SNR */
      if (params->snrcalc){
        TEMPLATE(snr_yuv)(&psnr,&orig,&rec[rec_buffer_idx],height,width,encoder_info.params->input_bitdepth);
      }
      else{
        psnr.y =  psnr.u = psnr.v = 0.0;
      }
      accsnr.y += psnr.y;
      accsnr.u += psnr.u;
      accsnr.v += psnr.v;

      acc_num_bits += num_bits;

      if (encoder_info.frame_info.frame_type==I_FRAME)
        fprintf(stdout,"%4d I %4d %10d %10.4f %8.4f %8.4f ",frame_num,encoder_info.frame_info.qp,num_bits,psnr.y,psnr.u,psnr.v);
      else if (encoder_info.frame_info.frame_type==P_FRAME)
        fprintf(stdout,"%4d P %4d %10d %10.4f %8.4f %8.4f ",frame_num,encoder_info.frame_info.qp,num_bits,psnr.y,psnr.u,psnr.v);
      else 
        fprintf(stdout,"%4d B %4d %10d %10.4f %8.4f %8.4f ",frame_num,encoder_info.frame_info.qp,num_bits,psnr.y,psnr.u,psnr.v);

      int ref_idx;
      for (ref_idx=0; ref_idx<encoder_info.frame_info.num_ref; ref_idx++){
        encoder_info.frame_info.ref_array[ref_idx]==-1 ? fprintf(stdout,"I(%d,%d) ",encoder_info.frame_info.ref_array[ref_idx+1],encoder_info.frame_info.ref_array[ref_idx+2])
          : fprintf(stdout,"%3d",encoder_info.frame_info.ref_array[ref_idx]);
      }

      for (ref_idx = encoder_info.frame_info.num_ref; ref_idx < encoder_info.params->max_num_ref; ref_idx++) {
        fprintf(stdout, "   ");
      }
      fprintf(stdout, " | ");
      for (ref_idx = 0; ref_idx<encoder_info.frame_info.num_ref; ref_idx++) {
        int r0 = encoder_info.frame_info.ref_array[ref_idx+0];
        int r1 = encoder_info.frame_info.ref_array[ref_idx+1];
        int r2 = encoder_info.frame_info.ref_array[ref_idx+2];
        r0 == -1 ? fprintf(stdout, "I(%d,%d)", encoder_info.ref[r1 + 1]->frame_num, encoder_info.ref[r2 + 1]->frame_num) : fprintf(stdout, "%3d", encoder_info.ref[r0 + 1]->frame_num);
      }
      fprintf(stdout,"\n");
      fflush(stdout);

      /* Write compressed bits for this frame to file */
      flush_all_bits(&stream, strfile);

      if (reconfile){
        /* Write output frame */
        rec_buffer_idx = (last_frame_output+1) % MAX_REORDER_BUFFER;
        if (rec_available[rec_buffer_idx]) {
          last_frame_output++;
          if (y4m_output)
          {
            fprintf(reconfile, "FRAME\x0a");
          }
          TEMPLATE(write_yuv_frame)(&rec[rec_buffer_idx],reconfile);
          rec_available[rec_buffer_idx]=0;
        }
      }

      // Keep track of when the last anchor frame was in the sliding window
      last_PorI_frame = (encoder_info.frame_info.frame_type != B_FRAME ? 0 : last_PorI_frame+1);
    }

    /* Revert to PPP coding if our subgop does not fit in. Keeping track of the last anchor frame
       should mean that the first reference is correct when we do, although subsequent references
       may not be ideal.
     */
    if (((frame_num0+sub_gop+1)*frame_size > input_file_size || frame_num0+sub_gop >= params->skip+params->num_frames )&& sub_gop>=2) {
      params->HQperiod = sub_gop;
      sub_gop = 1;
      params->num_reorder_pics = 0;
    };
  }
  // Write out the tail
  int i;
  if (reconfile) {
    for (i=1; i<=MAX_REORDER_BUFFER; ++i) {
      rec_buffer_idx=(last_frame_output+i) % MAX_REORDER_BUFFER;
      if (rec_available[rec_buffer_idx]) {
        if (y4m_output)
            fprintf(reconfile, "FRAME\x0a");
        TEMPLATE(write_yuv_frame)(&rec[rec_buffer_idx],reconfile);
        rec_available[rec_buffer_idx]=0;
      }
      else
        break;
    }
  }


  bit_rate_in_kbps = 0.001*params->frame_rate*(double)acc_num_bits/num_encoded_frames;

  /* Finised encoding sequence */
  fprintf(stdout,"------------------- Average data for all frames ------------------------------\n");
  fprintf(stdout,"kbps            : %12.3f\n",bit_rate_in_kbps);
  fprintf(stdout,"PSNR Y          : %12.3f\n",accsnr.y/num_encoded_frames);
  fprintf(stdout,"PSNR U          : %12.3f\n",accsnr.u/num_encoded_frames);
  fprintf(stdout,"PSNR V          : %12.3f\n",accsnr.v/num_encoded_frames);
  fprintf(stdout,"------------------------------------------------------------------------------\n");

  /* Append one line of statistics to a file */
  if (params->statfilestr) {
    FILE *cumu_fp;

    int not_exists = !(cumu_fp = fopen(params->statfilestr, "r"));
    if (!not_exists)
      fclose(cumu_fp);
    if ((cumu_fp = fopen(params->statfilestr, "a")) != NULL) {
      if (not_exists)
        fprintf(cumu_fp, " NFR     kbps     PSNRY  PSNRU  PSNRV\n");
      fprintf(cumu_fp, "%4d %12.3f %6.3f %6.3f %6.3f\n",
          params->num_frames,
          bit_rate_in_kbps,
          accsnr.y/(double)num_encoded_frames,
          accsnr.u/(double)num_encoded_frames,
          accsnr.v/(double)num_encoded_frames);
      fclose(cumu_fp);
    }
  }

  TEMPLATE(close_yuv_frame)(&orig);
  for (int i=0; i<MAX_REORDER_BUFFER+1; ++i) {
    TEMPLATE(close_yuv_frame)(&rec[i]);
  }
  for (r=0;r<MAX_REF_FRAMES;r++){
    TEMPLATE(close_yuv_frame)(&ref[r]);
  }
  if (params->interp_ref) {
    for (r=0;r<MAX_SKIP_FRAMES;r++){
      TEMPLATE(close_yuv_frame)(encoder_info.interp_frames[r]);
      free(encoder_info.interp_frames[r]);
    }
  }

  fclose(infile);
  fclose(strfile);
  if (reconfile)
  {
    fclose(reconfile);
  }
  free(stream.bitstream);
  free(encoder_info.deblock_data);

  if (params->bitrate > 0) {
    delete_rate_control_per_sequence(&rc);
  }

  delete_config_params(params);
  return 0;
}    
