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
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include "global.h"
#include "maindec.h"
#include "decode_frame.h"
#include "common_frame.h"
#include "getbits.h"
#include "../common/simd.h"

void rferror(char error_text[])
{
    fprintf(stderr,"Run-time error...\n");
    fprintf(stderr,"%s\n",error_text);
    fprintf(stderr,"...now exiting to system...\n");
    exit(1);
}

void parse_arg(int argc, char** argv, FILE **infile, FILE **outfile)
{
    if (argc < 2)
    {
        fprintf(stdout, "usage: %s infile [outfile]\n", argv[0]);
        rferror("Wrong number of arguments.");
    }

    if (!(*infile = fopen(argv[1], "rb")))
    {
        rferror("Could not open in-file for reading.");
    }

    if (argc > 2)
    {
        if (!(*outfile = fopen(argv[2], "wb")))
        {
            rferror("Could not open out-file for writing.");
        }
    }
    else
    {
        *outfile = NULL;
    }
}

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
//static const int* dyadic_reorder_display_to_code[5] = {dc1,dc2,dc4,dc8,dc16}; //Not used in decoder?

static int reorder_frame_offset(int idx, int sub_gop, int dyadic)
{
  if (dyadic && sub_gop>1) {
    return dyadic_reorder_code_to_display[log2i(sub_gop)][idx]-sub_gop+1;
  } else {
    if (idx==0) return 0;
    else return idx-sub_gop;
  }
}

unsigned int leading_zeros(unsigned int code)
{
  unsigned int count = 0;
  int done = 0;

  if (code){
    while (!done){
      code >>= 1;
      if (!code) done = 1;
      else count++;
    }
  }
  return count;
}

int main(int argc, char** argv)
{
    FILE *infile,*outfile;
    decoder_info_t decoder_info;
    stream_t stream;
    yuv_frame_t rec[MAX_REORDER_BUFFER];
    yuv_frame_t ref[MAX_REF_FRAMES];
    int rec_available[MAX_REORDER_BUFFER]={0};
    int rec_buffer_idx;
    int decode_frame_num = 0;
    int frame_count = 0;
    int last_frame_output = -1;
    int done = 0;
    int width;
    int height;
    int r;
    int sub_gop=1;

    init_use_simd();

    parse_arg(argc, argv, &infile, &outfile);
    
	  fseek(infile, 0, SEEK_END);
	  int input_file_size = ftell(infile);
	  fseek(infile, 0, SEEK_SET);
    
    memset(&stream, 0, sizeof(stream));
    initbits_dec(infile, &stream);

    decoder_info.stream = &stream;

    memset(&decoder_info.bit_count,0,sizeof(bit_count_t));

    int bit_start = od_ec_dec_tell(&stream.ec);
    /* Read sequence header */
    width = getbits(&stream,16);
    height = getbits(&stream,16);

    decoder_info.width = width;
    decoder_info.height = height;
    printf("width=%4d height=%4d\n",width,height);

    decoder_info.pb_split = getbits(&stream,1);
    printf("pb_split_enable=%1d\n",decoder_info.pb_split); //TODO: Rename variable to pb_split_enable

    decoder_info.tb_split_enable = getbits(&stream,1);
    printf("tb_split_enable=%1d\n",decoder_info.tb_split_enable);

    decoder_info.max_num_ref = getbits(&stream,2) + 1;
    fprintf(stderr,"num refs is %d\n",decoder_info.max_num_ref);

    decoder_info.num_reorder_pics = getbits(&stream,4);
    if (decoder_info.num_reorder_pics>0)
      decoder_info.dyadic_coding = getbits(&stream,1);
    decoder_info.interp_ref = getbits(&stream,1);
    sub_gop = 1+decoder_info.num_reorder_pics;

    decoder_info.max_delta_qp = getbits(&stream,2);

    decoder_info.deblocking = getbits(&stream,1);
    decoder_info.clpf = getbits(&stream,1);
    decoder_info.use_block_contexts = getbits(&stream,1);
    decoder_info.bipred = getbits(&stream,1);

    decoder_info.bit_count.sequence_header +=
     (od_ec_dec_tell(&stream.ec) - bit_start);

    for (r=0;r<MAX_REORDER_BUFFER;r++){
      create_yuv_frame(&rec[r],width,height,0,0,0,0);
    }
    for (r=0;r<MAX_REF_FRAMES;r++){
      create_yuv_frame(&ref[r],width,height,PADDING_Y,PADDING_Y,PADDING_Y/2,PADDING_Y/2);
      decoder_info.ref[r] = &ref[r];
    }
    if (decoder_info.interp_ref) {
      for (r=0;r<MAX_SKIP_FRAMES;r++){
        decoder_info.interp_frames[r] = malloc(sizeof(yuv_frame_t));
        create_yuv_frame(decoder_info.interp_frames[r],width,height,PADDING_Y,PADDING_Y,PADDING_Y/2,PADDING_Y/2);
      }
    }

    decoder_info.deblock_data = (deblock_data_t *)malloc((height/MIN_PB_SIZE) * (width/MIN_PB_SIZE) * sizeof(deblock_data_t));

    do
    {
      decoder_info.frame_info.decode_order_frame_num = decode_frame_num;
      decoder_info.frame_info.display_frame_num = (frame_count/sub_gop)*sub_gop+reorder_frame_offset(frame_count % sub_gop, sub_gop,decoder_info.dyadic_coding);
      if (decoder_info.frame_info.display_frame_num>=0) {
        rec_buffer_idx = decoder_info.frame_info.display_frame_num%MAX_REORDER_BUFFER;
        decoder_info.rec = &rec[rec_buffer_idx];
        decoder_info.rec->frame_num = decoder_info.frame_info.display_frame_num;
        decode_frame(&decoder_info);
        done = initbits_dec(infile, &stream);
        rec_available[rec_buffer_idx]=1;

        rec_buffer_idx = (last_frame_output+1)%MAX_REORDER_BUFFER;
        if (rec_available[rec_buffer_idx]) {
          last_frame_output++;
          write_yuv_frame(&rec[rec_buffer_idx],width,height,outfile);
          rec_available[rec_buffer_idx] = 0;
        }
        printf("decode_frame_num=%4d display_frame_num=%4d input_file_size=%12d bitcnt=%12d\n",
            decode_frame_num,decoder_info.frame_info.display_frame_num,input_file_size,od_ec_dec_tell(&stream.ec));
        decode_frame_num++;
      }
      frame_count++;
    }
    while (!done);
    // Output the tail
    int i,j;
    for (i=1; i<=MAX_REORDER_BUFFER; ++i) {
      rec_buffer_idx=(last_frame_output+i) % MAX_REORDER_BUFFER;
      if (rec_available[rec_buffer_idx])
        write_yuv_frame(&rec[rec_buffer_idx],width,height,outfile);
      else
        break;
    }

    bit_count_t bit_count = decoder_info.bit_count;
    uint32_t tot_bits[NUM_FRAME_TYPES] = {0};

    for (i=0;i<NUM_FRAME_TYPES;i++){
      tot_bits[i] = bit_count.frame_header[i] +
                    bit_count.super_mode[i] +
                    bit_count.intra_mode[i] +
                    bit_count.mv[i] +
                    bit_count.skip_idx[i] +
                    bit_count.coeff_y[i] +
                    bit_count.coeff_u[i] +
                    bit_count.coeff_v[i] +
                    bit_count.cbp[i] +
                    bit_count.clpf[i];
    }
    tot_bits[0] += bit_count.sequence_header;
    int ni = bit_count.frame_type[0];
    int np = bit_count.frame_type[1];
    int nb = bit_count.frame_type[2];
    if (np==0) np = (1<<30); //Hack to avoid division by zero if there are no P frames
    if (nb==0) nb = (1<<30); //Hack to avoid division by zero if there are no B frames

    printf("\n\nBIT STATISTICS:\n");
    printf("Sequence header: %4d\n",bit_count.sequence_header);
    printf("                           I pictures:           P pictures:           B pictures:\n");
    printf("                           total    average      total    average      total    average\n");
    printf("Frame header:          %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.frame_header[0],bit_count.frame_header[0]/ni,bit_count.frame_header[1],bit_count.frame_header[1]/np,bit_count.frame_header[2],bit_count.frame_header[2]/nb);
    printf("Super mode:            %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.super_mode[0],bit_count.super_mode[0]/ni,bit_count.super_mode[1],bit_count.super_mode[1]/np,bit_count.super_mode[2],bit_count.super_mode[2]/nb);
    printf("Intra mode:            %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.intra_mode[0],bit_count.intra_mode[0]/ni,bit_count.intra_mode[1],bit_count.intra_mode[1]/np, bit_count.intra_mode[2],bit_count.intra_mode[2]/nb);
    printf("MV:                    %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.mv[0],bit_count.mv[0],bit_count.mv[1],bit_count.mv[1]/np, bit_count.mv[2],bit_count.mv[2]/nb);
    printf("Skip idx:              %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.skip_idx[0],bit_count.skip_idx[0],bit_count.skip_idx[1],bit_count.skip_idx[1]/np,bit_count.skip_idx[2],bit_count.skip_idx[2]/nb);
    printf("Coeff_y:               %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.coeff_y[0],bit_count.coeff_y[0]/ni,bit_count.coeff_y[1],bit_count.coeff_y[1]/np,bit_count.coeff_y[2],bit_count.coeff_y[2]/nb);
    printf("Coeff_u:               %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.coeff_u[0],bit_count.coeff_u[0]/ni,bit_count.coeff_u[1],bit_count.coeff_u[1]/np,bit_count.coeff_u[2],bit_count.coeff_u[2]/nb);
    printf("Coeff_v:               %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.coeff_v[0],bit_count.coeff_v[0]/ni,bit_count.coeff_v[1],bit_count.coeff_v[1]/np,bit_count.coeff_v[2],bit_count.coeff_v[2]/nb);
    printf("CBP (TU-split):        %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.cbp[0],bit_count.cbp[0]/ni,bit_count.cbp[1],bit_count.cbp[1]/np,bit_count.cbp[2],bit_count.cbp[2]/nb);
    printf("CLPF:                  %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.clpf[0],bit_count.clpf[0]/ni,bit_count.clpf[1],bit_count.clpf[1]/np,bit_count.clpf[2],bit_count.clpf[2]/nb);
    printf("Total:                 %9d  %9d  %9d  %9d  %9d  %9d\n",tot_bits[0],tot_bits[0],tot_bits[1],tot_bits[1]/np,tot_bits[2],tot_bits[2]/nb);
    printf("---------------------------------------------------------------------------------------\n\n");

    printf("PARAMETER STATISTICS:\n");
    printf("                           I pictures:           P pictures:           B pictures:\n");
    printf("                           total    average      total    average      total    average\n");
    printf("Skip-blocks (8x8):     %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.mode[0][0],bit_count.mode[0][0]/ni,bit_count.mode[1][0],bit_count.mode[1][0]/np,bit_count.mode[2][0],bit_count.mode[2][0]/nb);
    printf("Intra-blocks (8x8):    %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.mode[0][1],bit_count.mode[0][1]/ni,bit_count.mode[1][1],bit_count.mode[1][1]/np,bit_count.mode[2][1],bit_count.mode[2][1]/nb);
    printf("Inter-blocks (8x8):    %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.mode[0][2],bit_count.mode[0][2]/ni,bit_count.mode[1][2],bit_count.mode[1][2]/np,bit_count.mode[2][2],bit_count.mode[2][2]/nb);
    printf("Bipred-blocks (8x8):   %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.mode[0][3],bit_count.mode[0][3]/ni,bit_count.mode[1][3],bit_count.mode[1][3]/np,bit_count.mode[2][3],bit_count.mode[2][3]/nb);
    printf("Merge-blocks (8x8):    %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.mode[0][4],bit_count.mode[0][4]/ni,bit_count.mode[1][4],bit_count.mode[1][4]/np,bit_count.mode[2][4],bit_count.mode[2][4]/nb);

    printf("\n");
    printf("8x8-blocks (8x8):      %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.size[0][0],bit_count.size[0][0]/ni,bit_count.size[1][0],bit_count.size[1][0]/np,bit_count.size[2][0],bit_count.size[2][0]/nb);
    printf("16x16-blocks (8x8):    %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.size[0][1],bit_count.size[0][1]/ni,bit_count.size[1][1],bit_count.size[1][1]/np,bit_count.size[2][1],bit_count.size[2][1]/nb);
    printf("32x32-blocks (8x8):    %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.size[0][2],bit_count.size[0][2]/ni,bit_count.size[1][2],bit_count.size[1][2]/np,bit_count.size[2][2],bit_count.size[2][2]/nb);
    printf("64x64-blocks (8x8):    %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.size[0][3],bit_count.size[0][3]/ni,bit_count.size[1][3],bit_count.size[1][3]/np,bit_count.size[2][3],bit_count.size[2][3]/nb);

    printf("\n");
    printf("Mode and size distribution for P pictures:\n");
    printf("                            SKIP      INTRA      INTER     BIPRED      MERGE\n");
    printf("8x8-blocks (8x8):      %9d  %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[P_FRAME][0][0],bit_count.size_and_mode[P_FRAME][0][1],bit_count.size_and_mode[P_FRAME][0][2],bit_count.size_and_mode[P_FRAME][0][3],bit_count.size_and_mode[P_FRAME][0][4]);
    printf("16x16-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[P_FRAME][1][0],bit_count.size_and_mode[P_FRAME][1][1],bit_count.size_and_mode[P_FRAME][1][2],bit_count.size_and_mode[P_FRAME][1][3],bit_count.size_and_mode[P_FRAME][1][4]);
    printf("32x32-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[P_FRAME][2][0],bit_count.size_and_mode[P_FRAME][2][1],bit_count.size_and_mode[P_FRAME][2][2],bit_count.size_and_mode[P_FRAME][2][3],bit_count.size_and_mode[P_FRAME][2][4]);
    printf("64x64-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[P_FRAME][3][0],bit_count.size_and_mode[P_FRAME][3][1],bit_count.size_and_mode[P_FRAME][3][2],bit_count.size_and_mode[P_FRAME][3][3],bit_count.size_and_mode[P_FRAME][3][4]);

    printf("\n");
    printf("Mode and size distribution for B pictures:\n");
    printf("                            SKIP      INTRA      INTER     BIPRED      MERGE\n");
    printf("8x8-blocks (8x8):      %9d  %9d  %9d  %9d  %9d\n", bit_count.size_and_mode[B_FRAME][0][0], bit_count.size_and_mode[B_FRAME][0][1], bit_count.size_and_mode[B_FRAME][0][2], bit_count.size_and_mode[B_FRAME][0][3], bit_count.size_and_mode[B_FRAME][0][4]);
    printf("16x16-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n", bit_count.size_and_mode[B_FRAME][1][0], bit_count.size_and_mode[B_FRAME][1][1], bit_count.size_and_mode[B_FRAME][1][2], bit_count.size_and_mode[B_FRAME][1][3], bit_count.size_and_mode[B_FRAME][1][4]);
    printf("32x32-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n", bit_count.size_and_mode[B_FRAME][2][0], bit_count.size_and_mode[B_FRAME][2][1], bit_count.size_and_mode[B_FRAME][2][2], bit_count.size_and_mode[B_FRAME][2][3], bit_count.size_and_mode[B_FRAME][2][4]);
    printf("64x64-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n", bit_count.size_and_mode[B_FRAME][3][0], bit_count.size_and_mode[B_FRAME][3][1], bit_count.size_and_mode[B_FRAME][3][2], bit_count.size_and_mode[B_FRAME][3][3], bit_count.size_and_mode[B_FRAME][3][4]);

    int idx;
    int num = 5 + decoder_info.max_num_ref;
    printf("\nSuper-mode distribution for P pictures:\n");
    printf("                    SKIP   SPLIT INTERr0   MERGE   BIPRED  INTRA ");
    for (i = 1; i < decoder_info.max_num_ref; i++) printf("INTERr%1d ", i);
    printf("\n");
    for (idx=0;idx<NUM_BLOCK_SIZES;idx++){
      int size = 8<<idx;
      printf("%2d x %2d-blocks: ",size,size);
      for (i=0;i<num;i++){
        printf("%8d",bit_count.super_mode_stat[P_FRAME][idx][i]);
      }
      printf("\n");
    }
   
    printf("\nSuper-mode distribution for B pictures:\n");
    printf("                    SKIP   SPLIT INTERr0   MERGE   BIPRED  INTRA ");
    for (i = 1; i < decoder_info.max_num_ref; i++) printf("INTERr%1d ", i);
    printf("\n");
    for (idx = 0; idx<NUM_BLOCK_SIZES; idx++) {
      int size = 8 << idx;
      printf("%2d x %2d-blocks: ", size, size);
      for (i = 0; i<num; i++) {
        printf("%8d", bit_count.super_mode_stat[B_FRAME][idx][i]);
      }
      printf("\n");
    }

    int size;
    int max_num_ref = 4;
    printf("\n");
    printf("Ref_idx and size distribution for P pictures:\n");
    for (i=0;i<NUM_BLOCK_SIZES;i++){
      size = 1<<(i+3);
      printf("%2d x %2d-blocks: ",size,size);
      for (j=0;j<decoder_info.max_num_ref;j++){
        printf("%6d",bit_count.size_and_ref_idx[P_FRAME][i][j]);
      }
      printf("\n");
    }

    printf("\n");
    printf("Ref_idx and size distribution for B pictures:\n");
    for (i = 0; i<NUM_BLOCK_SIZES; i++) {
      size = 1 << (i + 3);
      printf("%2d x %2d-blocks: ", size, size);
      for (j = 0; j<decoder_info.max_num_ref; j++) {
        printf("%6d", bit_count.size_and_ref_idx[B_FRAME][i][j]);
      }
      printf("\n");
    }
    printf("\n");
    printf("bi-ref-P:  ");
    for (j=0;j<max_num_ref*max_num_ref;j++){
      printf("%7d",bit_count.bi_ref[P_FRAME][j]);
    }
    printf("\n");
    printf("bi-ref-B:  ");
    for (j = 0; j<max_num_ref*max_num_ref; j++) {
      printf("%7d", bit_count.bi_ref[B_FRAME][j]);
    }
    printf("\n");
    printf("-----------------------------------------------------------------\n");
    for (r=0;r<MAX_REORDER_BUFFER;r++){
      close_yuv_frame(&rec[r]);
    }
    for (r=0;r<MAX_REF_FRAMES;r++){
      close_yuv_frame(&ref[r]);
    }
    if (decoder_info.interp_ref) {
      for (r=0;r<MAX_SKIP_FRAMES;r++){
        close_yuv_frame(decoder_info.interp_frames[r]);
        free(decoder_info.interp_frames[r]);
      }
    }

    free(decoder_info.deblock_data);

    return 0;
}
