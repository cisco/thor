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
#include "getvlc.h"
#include "../common/simd.h"
#include "wt_matrix.h"
#include "read_bits.h"

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

#undef TEMPLATE
#define TEMPLATE(func) (decoder_info.bitdepth == 8 ? func ## _lbd : func ## _hbd)

int main(int argc, char** argv)
{
    FILE *infile,*outfile;
    decoder_info_t decoder_info;
    stream_t stream;
    yuv_frame_t rec[MAX_REORDER_BUFFER+1];  // Last is for temp use
    yuv_frame_t ref[MAX_REF_FRAMES];
    int rec_available[MAX_REORDER_BUFFER]={0};
    int rec_buffer_idx;
    int op_rec_buffer_idx;
    int decode_frame_num = 0;
    int last_frame_output = -1;
    int done = 0;
    int width;
    int height;
    int r;

    init_use_simd();

    parse_arg(argc, argv, &infile, &outfile);
    char *p = strrchr(argv[2], '.');
    int y4m_output = p != NULL && !strcmp(p,".y4m");
    
    fseek(infile, 0, SEEK_END);
    int input_file_size = ftell(infile);
    fseek(infile, 0, SEEK_SET);
    
    initbits_dec(infile, &stream);

    decoder_info.stream = &stream;

    memset(&decoder_info.bit_count,0,sizeof(bit_count_t));

    int bit_start = stream.bitcnt;
    /* Read sequence header */

    read_sequence_header(&decoder_info, &stream);
    width = decoder_info.width;
    height = decoder_info.height;
    if (decoder_info.qmtx) {
      alloc_wmatrices(decoder_info.iwmatrix, 1);
    }

    decoder_info.bit_count.sequence_header += (stream.bitcnt - bit_start);

    for (r=0;r<MAX_REORDER_BUFFER+1;r++){
      TEMPLATE(create_yuv_frame)(&rec[r],width,height,decoder_info.subsample,0,0,decoder_info.bitdepth,decoder_info.input_bitdepth);
    }
    for (r=0;r<MAX_REF_FRAMES;r++){
      TEMPLATE(create_yuv_frame)(&ref[r],width,height,decoder_info.subsample,PADDING_Y,PADDING_Y,decoder_info.bitdepth,decoder_info.input_bitdepth);
      decoder_info.ref[r] = &ref[r];
    }
    if (decoder_info.interp_ref) {
      for (r=0;r<MAX_SKIP_FRAMES;r++){
        decoder_info.interp_frames[r] = malloc(sizeof(yuv_frame_t));
        TEMPLATE(create_yuv_frame)(decoder_info.interp_frames[r],width,height,decoder_info.subsample,PADDING_Y,PADDING_Y,decoder_info.bitdepth,decoder_info.input_bitdepth);
      }
    }

    decoder_info.deblock_data = (deblock_data_t *)malloc((height/MIN_PB_SIZE) * (width/MIN_PB_SIZE) * sizeof(deblock_data_t));

    if (y4m_output) {
        fprintf(outfile,
                "YUV4MPEG2 W%d H%d F%d:1 Ip A%d:%d C",
                width, height, 30, 1, 1);
      if (decoder_info.subsample == 400)
        fprintf(outfile, "mono");
      else
        fprintf(outfile, "%d", decoder_info.subsample);
      if (decoder_info.input_bitdepth > 8)
        fprintf(outfile, "p%d XYSCSS=%dp%d", decoder_info.input_bitdepth, decoder_info.subsample, decoder_info.input_bitdepth);
      fprintf(outfile, "\x0a");
    }

    do
    {
      decoder_info.frame_info.decode_order_frame_num = decode_frame_num;
      decode_frame(&decoder_info,rec);
      rec_buffer_idx = decoder_info.frame_info.display_frame_num%MAX_REORDER_BUFFER;
      rec_available[rec_buffer_idx]=1;

      done = initbits_dec(infile, &stream);

      op_rec_buffer_idx = (last_frame_output+1)%MAX_REORDER_BUFFER;
      if (rec_available[op_rec_buffer_idx]) {
        last_frame_output++;
        if (y4m_output)
          fprintf(outfile, "FRAME\x0a");
        TEMPLATE(write_yuv_frame)(&rec[op_rec_buffer_idx],outfile);
        rec_available[op_rec_buffer_idx] = 0;
      }
      printf("decode_frame_num=%4d display_frame_num=%4d input_file_size=%12d bitcnt=%12d\n",
          decode_frame_num,decoder_info.frame_info.display_frame_num,input_file_size,stream.bitcnt);
      decode_frame_num++;
    }
    while (!done);
    // Output the tail
    int i,j;
    for (i=1; i<=MAX_REORDER_BUFFER; ++i) {
      op_rec_buffer_idx=(last_frame_output+i) % MAX_REORDER_BUFFER;
      if (rec_available[op_rec_buffer_idx]) {
        if (y4m_output)
          fprintf(outfile, "FRAME\x0a");
        TEMPLATE(write_yuv_frame)(&rec[op_rec_buffer_idx],outfile);
      } else
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
    printf("128x128-blocks (8x8):  %9d  %9d  %9d  %9d  %9d  %9d\n",bit_count.size[0][4],bit_count.size[0][4]/ni,bit_count.size[1][4],bit_count.size[1][4]/np,bit_count.size[2][4],bit_count.size[2][4]/nb);

    printf("\n");
    printf("Mode and size distribution for P pictures:\n");
    printf("                            SKIP      INTRA      INTER     BIPRED      MERGE\n");
    printf("8x8-blocks (8x8):      %9d  %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[P_FRAME][0][0],bit_count.size_and_mode[P_FRAME][0][1],bit_count.size_and_mode[P_FRAME][0][2],bit_count.size_and_mode[P_FRAME][0][3],bit_count.size_and_mode[P_FRAME][0][4]);
    printf("16x16-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[P_FRAME][1][0],bit_count.size_and_mode[P_FRAME][1][1],bit_count.size_and_mode[P_FRAME][1][2],bit_count.size_and_mode[P_FRAME][1][3],bit_count.size_and_mode[P_FRAME][1][4]);
    printf("32x32-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[P_FRAME][2][0],bit_count.size_and_mode[P_FRAME][2][1],bit_count.size_and_mode[P_FRAME][2][2],bit_count.size_and_mode[P_FRAME][2][3],bit_count.size_and_mode[P_FRAME][2][4]);
    printf("64x64-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[P_FRAME][3][0],bit_count.size_and_mode[P_FRAME][3][1],bit_count.size_and_mode[P_FRAME][3][2],bit_count.size_and_mode[P_FRAME][3][3],bit_count.size_and_mode[P_FRAME][3][4]);
    printf("128x128-blocks (8x8):  %9d  %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[P_FRAME][4][0],bit_count.size_and_mode[P_FRAME][4][1],bit_count.size_and_mode[P_FRAME][4][2],bit_count.size_and_mode[P_FRAME][4][3],bit_count.size_and_mode[P_FRAME][4][4]);


    printf("\n");
    printf("Mode and size distribution for B pictures:\n");
    printf("                            SKIP      INTRA      INTER     BIPRED      MERGE\n");
    printf("8x8-blocks (8x8):      %9d  %9d  %9d  %9d  %9d\n", bit_count.size_and_mode[B_FRAME][0][0], bit_count.size_and_mode[B_FRAME][0][1], bit_count.size_and_mode[B_FRAME][0][2], bit_count.size_and_mode[B_FRAME][0][3], bit_count.size_and_mode[B_FRAME][0][4]);
    printf("16x16-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n", bit_count.size_and_mode[B_FRAME][1][0], bit_count.size_and_mode[B_FRAME][1][1], bit_count.size_and_mode[B_FRAME][1][2], bit_count.size_and_mode[B_FRAME][1][3], bit_count.size_and_mode[B_FRAME][1][4]);
    printf("32x32-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n", bit_count.size_and_mode[B_FRAME][2][0], bit_count.size_and_mode[B_FRAME][2][1], bit_count.size_and_mode[B_FRAME][2][2], bit_count.size_and_mode[B_FRAME][2][3], bit_count.size_and_mode[B_FRAME][2][4]);
    printf("64x64-blocks (8x8):    %9d  %9d  %9d  %9d  %9d\n", bit_count.size_and_mode[B_FRAME][3][0], bit_count.size_and_mode[B_FRAME][3][1], bit_count.size_and_mode[B_FRAME][3][2], bit_count.size_and_mode[B_FRAME][3][3], bit_count.size_and_mode[B_FRAME][3][4]);
    printf("128x128-blocks (8x8):  %9d  %9d  %9d  %9d  %9d\n", bit_count.size_and_mode[B_FRAME][4][0], bit_count.size_and_mode[B_FRAME][4][1], bit_count.size_and_mode[B_FRAME][4][2], bit_count.size_and_mode[B_FRAME][4][3], bit_count.size_and_mode[B_FRAME][4][4]);

    int idx;
    int num = 5 + decoder_info.max_num_ref;
    printf("\nSuper-mode distribution for P pictures:\n");
    printf("                    SKIP   SPLIT INTERr0   MERGE   BIPRED  INTRA ");
    for (i = 1; i < decoder_info.max_num_ref; i++) printf("INTERr%1d ", i);
    printf("\n");
    for (idx=0;idx<NUM_BLOCK_SIZES;idx++){
      int size = 8<<idx;
      printf("%3d x %3d-blocks: ",size,size);
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
      printf("%3d x %3d-blocks: ", size, size);
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
      printf("%3d x %3d-blocks: ",size,size);
      for (j=0;j<decoder_info.max_num_ref;j++){
        printf("%6d",bit_count.size_and_ref_idx[P_FRAME][i][j]);
      }
      printf("\n");
    }

    printf("\n");
    printf("Ref_idx and size distribution for B pictures:\n");
    for (i = 0; i<NUM_BLOCK_SIZES; i++) {
      size = 1 << (i + 3);
      printf("%3d x %3d-blocks: ", size, size);
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
    for (r=0;r<MAX_REORDER_BUFFER+1;r++){
      TEMPLATE(close_yuv_frame)(&rec[r]);
    }
    for (r=0;r<MAX_REF_FRAMES;r++){
      TEMPLATE(close_yuv_frame)(&ref[r]);
    }
    if (decoder_info.interp_ref) {
      for (r=0;r<MAX_SKIP_FRAMES;r++){
        TEMPLATE(close_yuv_frame)(decoder_info.interp_frames[r]);
        free(decoder_info.interp_frames[r]);
      }
    }

    free(decoder_info.deblock_data);
    if (infile)
      fclose(infile);
    if (outfile)
      fclose(outfile);

    return 0;
}
