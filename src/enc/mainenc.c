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
#include "../common/simd.h"

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

static int reorder_frame_offset(int idx, int sub_gop)
{
#if DYADIC_CODING
  return dyadic_reorder_code_to_display[log2i(sub_gop)][idx]-sub_gop+1;
#else
  if (idx==0) return 0;
  else return idx-sub_gop;
#endif
}

int main(int argc, char **argv)
{
  FILE *infile, *strfile, *reconfile;

  uint32_t input_file_size; //TODO: Support file size values larger than 32 bits 
  yuv_frame_t orig,ref[MAX_REF_FRAMES];
  yuv_frame_t rec[MAX_REORDER_BUFFER];
  int rec_available[MAX_REORDER_BUFFER] = {0};
  int last_frame_output=-1;
  int num_encoded_frames,num_bits,start_bits,end_bits;
  int sub_gop=1;
  int rec_buffer_idx;
  int frame_num,frame_num0,k,r;
  int frame_offset;
  int ysize,csize,frame_size;
  int width,height,input_stride_y,input_stride_c;
  uint32_t acc_num_bits;
  snrvals psnr;
  snrvals accsnr;
  double bit_rate_in_kbps;
  enc_params *params;
  encoder_info_t encoder_info;
  int y4m_output;

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
     "YUV4MPEG2 W%d H%d F%d:1 Ip A0:0 C420jpeg XYSCSS=420JPEG\x0a",
     params->width, params->height, (int)params->frame_rate);
  }

  accsnr.y = 0;
  accsnr.u = 0;
  accsnr.v = 0;
  acc_num_bits = 0;

  height = params->height;
  width = params->width;
  input_stride_y = width;
  input_stride_c = width/2;
  ysize = height * width;
  csize = ysize / 4;
  frame_size = ysize + 2*csize;

  /* Create frames*/
  create_yuv_frame(&orig,width,height,0,0,0,0);
  for (r=0;r<MAX_REORDER_BUFFER;r++){
    create_yuv_frame(&rec[r],width,height,0,0,0,0);
  }
  for (r=0;r<MAX_REF_FRAMES;r++){ //TODO: Use Long-term frame instead of a large sliding window
    create_yuv_frame(&ref[r],width,height,PADDING_Y,PADDING_Y,PADDING_Y/2,PADDING_Y/2);
  }

  /* Initialize main bit stream */
  stream_t stream;
  stream.bitstream = (uint8_t *)malloc(MAX_BUFFER_SIZE * sizeof(uint8_t));
  stream.bitbuf = 0;
  stream.bitrest = 32;
  stream.bytepos = 0;
  stream.bytesize = MAX_BUFFER_SIZE;

  /* Configure encoder */
  encoder_info.params = params;
  encoder_info.orig = &orig;
  for (r=0;r<MAX_REF_FRAMES;r++){
    encoder_info.ref[r] = &ref[r];
  }
  encoder_info.stream = &stream;
  encoder_info.width = width;
  encoder_info.height = height;

  encoder_info.deblock_data = (deblock_data_t *)malloc((height/MIN_PB_SIZE) * (width/MIN_PB_SIZE) * sizeof(deblock_data_t));


  /* Write sequence header */ //TODO: Separate function for sequence header
  start_bits = get_bit_pos(&stream);
  putbits(16,width,&stream);
  putbits(16,height,&stream);
  putbits(1,params->enable_pb_split,&stream);
  putbits(1,params->enable_tb_split,&stream);
  putbits(2,params->max_num_ref-1,&stream); //TODO: Support more than 4 reference frames
  putbits(4,params->num_reorder_pics,&stream);// Max 15 reordered pictures
  putbits(2,params->max_delta_qp,&stream);
  putbits(1,params->deblocking,&stream);
  putbits(1,params->clpf,&stream);
  putbits(1,params->use_block_contexts,&stream);
  putbits(1,params->enable_bipred,&stream);

  end_bits = get_bit_pos(&stream);
  num_bits = end_bits-start_bits;
  acc_num_bits += num_bits;
  printf("SH:  %4d bits\n",num_bits);

  /* Start encoding sequence */
  num_encoded_frames = 0;
  sub_gop = max(1,params->num_reorder_pics+1);
  for (frame_num0 = params->skip; frame_num0 < (params->skip + params->num_frames) && (frame_num0+sub_gop)*frame_size <= input_file_size; frame_num0+=sub_gop)
  {
    for (k=0; k<sub_gop; k++) {
      int r,r0,r1,r2,r3;
      /* Initialize frame info */
      frame_offset = reorder_frame_offset(k,sub_gop);
      frame_num = frame_num0 + frame_offset;
      // If there is an initial I frame and reordering need to jump to the next P frame
      if (frame_num<params->skip) continue;

      encoder_info.frame_info.frame_num = frame_num - params->skip;
      rec_buffer_idx = encoder_info.frame_info.frame_num%MAX_REORDER_BUFFER;
      encoder_info.rec = &rec[rec_buffer_idx];
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

      if (encoder_info.frame_info.frame_type == I_FRAME){
        encoder_info.frame_info.qp = params->qp + params->dqpI;
      }
      else if (params->num_reorder_pics==0) {
        if (num_encoded_frames % params->HQperiod)
          encoder_info.frame_info.qp = (int)(params->mqpP*(float)params->qp) + params->dqpP;
        else
          encoder_info.frame_info.qp = params->qp;
      } else {
        if (encoder_info.frame_info.frame_num % sub_gop){
          float mqpB = params->mqpB;
#if DYADIC_CODING
          mqpB = 1.0+(b_level+1)*((mqpB-1.0)/2.0);
#endif
          encoder_info.frame_info.qp = (int)(mqpB*(float)params->qp) + params->dqpB;
        }  else
          encoder_info.frame_info.qp = params->qp;
      }

      encoder_info.frame_info.num_ref = min(num_encoded_frames,params->max_num_ref);
      if (params->num_reorder_pics > 0) {
#if DYADIC_CODING
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
          int display_phase =  (encoder_info.frame_info.frame_num-1) % sub_gop;
          int ref_offset=sub_gop>>(b_level+1);

           encoder_info.frame_info.ref_array[0]=min(num_encoded_frames-1,coded_phase-dyadic_reorder_display_to_code[log2i(sub_gop)][display_phase-ref_offset+1]-1);
           encoder_info.frame_info.ref_array[1]=min(num_encoded_frames-1,coded_phase-dyadic_reorder_display_to_code[log2i(sub_gop)][display_phase+ref_offset+1]-1);
          /* use most recent frames for the last ref(s)*/
          for (r=2;r<encoder_info.frame_info.num_ref;r++){
            encoder_info.frame_info.ref_array[r] = r-2;
          }
        }
#else
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

#endif
      } else {
        if (encoder_info.frame_info.num_ref==1){
          /* If num_ref==1 always use most recent frame */
          encoder_info.frame_info.ref_array[0] = 0;
        }
        else if (encoder_info.frame_info.num_ref==2){
          /* If num_ref==2 use most recent LQ frame and most recent HQ frame */
          r0 = 0;
          r1 = ((num_encoded_frames + params->HQperiod - 2) % params->HQperiod) + 1;
          encoder_info.frame_info.ref_array[0] = r0;
          encoder_info.frame_info.ref_array[1] = r1;
        }
        else if (encoder_info.frame_info.num_ref==3){
          r0 = 0;
          r1 = ((num_encoded_frames + params->HQperiod - 2) % params->HQperiod) + 1;
          r2 = r1==1 ? 2 : 1;
          encoder_info.frame_info.ref_array[0] = r0;
          encoder_info.frame_info.ref_array[1] = r1;
          encoder_info.frame_info.ref_array[2] = r2;
        }
        else if (encoder_info.frame_info.num_ref==4){
          r0 = 0;
          r1 = ((num_encoded_frames + params->HQperiod - 2) % params->HQperiod) + 1;
          r2 = r1==1 ? 2 : 1;
          r3 = r2+1;
          if (r3==r1) r3 += 1;
          encoder_info.frame_info.ref_array[0] = r0;
          encoder_info.frame_info.ref_array[1] = r1;
          encoder_info.frame_info.ref_array[2] = r2;
          encoder_info.frame_info.ref_array[3] = r3;
        }
        else{
          for (r=0;r<encoder_info.frame_info.num_ref;r++){
            encoder_info.frame_info.ref_array[r] = r;
          }
        }
      }

      if (params->intra_rdo){
        if (encoder_info.frame_info.frame_type == I_FRAME){
          encoder_info.frame_info.num_intra_modes = 10;
        }
        else{
          encoder_info.frame_info.num_intra_modes = params->encoder_speed > 0 ? 4 : 10;
        }
      }
      else{
        encoder_info.frame_info.num_intra_modes = 4;
      }

#if 0
      /* To test sliding window operation */
      int offsetx = 500;
      int offsety = 200;
      int offset_rec = encoder_info.rec->offset_y + offsety * encoder_info.rec->stride_y +  offsetx;
      int offset_ref = encoder_info.ref[0]->offset_y + offsety * encoder_info.ref[0]->stride_y +  offsetx;
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
      read_yuv_frame(&orig,width,height,infile);
      orig.frame_num = encoder_info.frame_info.frame_num;

      /* Encode frame */
      start_bits = get_bit_pos(&stream);
      encode_frame(&encoder_info);
      rec_available[rec_buffer_idx]=1;
      end_bits =  get_bit_pos(&stream);
      num_bits = end_bits-start_bits;
      num_encoded_frames++;

      /* Compute SNR */
      if (params->snrcalc){
        snr_yuv(&psnr,&orig,&rec[rec_buffer_idx],height,width,input_stride_y,input_stride_c);
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

      for (r=0;r<encoder_info.frame_info.num_ref;r++){
        fprintf(stdout,"%3d",encoder_info.frame_info.ref_array[r]);
      }
      fprintf(stdout,"\n");
      fflush(stdout);

      /* Write compressed bits for this frame to file */
      flush_bytebuf(&stream, strfile);

      if (reconfile){
        /* Write output frame */
        rec_buffer_idx = (last_frame_output+1) % MAX_REORDER_BUFFER;
        if (rec_available[rec_buffer_idx]) {
          last_frame_output++;
          if (y4m_output)
          {
            fprintf(reconfile, "FRAME\x0a");
          }
          write_yuv_frame(&rec[rec_buffer_idx],width,height,reconfile);
          rec_available[rec_buffer_idx]=0;
        }
      }
    }
  }
  // Write out the tail
  int i;
  if (reconfile) {
    for (i=1; i<=MAX_REORDER_BUFFER; ++i) {
      rec_buffer_idx=(last_frame_output+i) % MAX_REORDER_BUFFER;
      if (rec_available[rec_buffer_idx]) {
        write_yuv_frame(&rec[rec_buffer_idx],width,height,reconfile);
        rec_available[rec_buffer_idx]=0;
      }
      else
        break;
    }
  }


  flush_all_bits(&stream, strfile);
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

  close_yuv_frame(&orig);
  for (int i=0; i<MAX_REORDER_BUFFER; ++i) {
    close_yuv_frame(&rec[i]);
  }
  for (r=0;r<MAX_REF_FRAMES;r++){
    close_yuv_frame(&ref[r]);
  }
  fclose(infile);
  fclose(strfile);
  if (reconfile)
  {
    fclose(reconfile);
  }
  free(stream.bitstream);
  free(encoder_info.deblock_data);
  delete_config_params(params);
  return 0;
}    
