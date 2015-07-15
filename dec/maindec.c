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

static int reorder_frame_offset(int idx, int sub_gop)
{
#if DYADIC_CODING
  return dyadic_reorder_code_to_display[log2i(sub_gop)][idx]-sub_gop+1;
#else
  if (idx==0) return 0;
  else return idx-sub_gop;
#endif
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
    int width;
    int height;
    int r;
    int sub_gop=1;

    init_use_simd();

    parse_arg(argc, argv, &infile, &outfile);
    
	  fseek(infile, 0, SEEK_END);
	  int input_file_size = ftell(infile);
	  fseek(infile, 0, SEEK_SET);
    
    initbits_dec(infile, &stream);

    decoder_info.stream = &stream;

    memset(&decoder_info.bit_count,0,sizeof(bit_count_t));

    int bit_start = stream.bitcnt;
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

    decoder_info.num_reorder_pics = getbits(&stream,4);
    sub_gop = 1+decoder_info.num_reorder_pics;

    decoder_info.max_delta_qp = getbits(&stream,2);

    decoder_info.deblocking = getbits(&stream,1);
    decoder_info.clpf = getbits(&stream,1);
    decoder_info.use_block_contexts = getbits(&stream,1);
    decoder_info.bipred = getbits(&stream,1);

    decoder_info.bit_count.sequence_header += (stream.bitcnt - bit_start);

    for (r=0;r<MAX_REORDER_BUFFER;r++){
      create_yuv_frame(&rec[r],width,height,0,0,0,0);
    }
    for (r=0;r<MAX_REF_FRAMES;r++){
      create_yuv_frame(&ref[r],width,height,PADDING_Y,PADDING_Y,PADDING_Y/2,PADDING_Y/2);
      decoder_info.ref[r] = &ref[r];
    }
    decoder_info.deblock_data = (deblock_data_t *)malloc((height/MIN_PB_SIZE) * (width/MIN_PB_SIZE) * sizeof(deblock_data_t));

    while (stream.bitcnt < 8*input_file_size - 8)
    {
      decoder_info.frame_info.decode_order_frame_num = decode_frame_num;
      decoder_info.frame_info.display_frame_num = (frame_count/sub_gop)*sub_gop+reorder_frame_offset(frame_count % sub_gop, sub_gop);
      if (decoder_info.frame_info.display_frame_num>=0) {
        rec_buffer_idx = decoder_info.frame_info.display_frame_num%MAX_REORDER_BUFFER;
        decoder_info.rec = &rec[rec_buffer_idx];
        decoder_info.frame_info.num_ref = min(decode_frame_num,decoder_info.max_num_ref);
        decoder_info.rec->frame_num = decoder_info.frame_info.display_frame_num;
        decode_frame(&decoder_info);
        rec_available[rec_buffer_idx]=1;

        rec_buffer_idx = (last_frame_output+1)%MAX_REORDER_BUFFER;
        if (rec_available[rec_buffer_idx]) {
          last_frame_output++;
          write_yuv_frame(&rec[rec_buffer_idx],width,height,outfile);
          rec_available[rec_buffer_idx] = 0;
        }
        printf("decode_frame_num=%4d display_frame_num=%4d input_file_size=%12d bitcnt=%12d\n",
            decode_frame_num,decoder_info.frame_info.display_frame_num,input_file_size,stream.bitcnt);
        decode_frame_num++;
      }
      frame_count++;
    }
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
    uint32_t tot_bits[2] = {0};

    for (i=0;i<2;i++){
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

    if (np==0) np = (1<<30); //Hack to avoid division by zero if there are no P frames

    printf("\n\nBIT STATISTICS:\n");
    printf("Sequence header: %4d\n",bit_count.sequence_header);
    printf("                           I pictures:           P pictures:\n");
    printf("                           total    average      total    average\n");
    printf("Frame header:          %9d  %9d  %9d  %9d\n",bit_count.frame_header[0],bit_count.frame_header[0]/ni,bit_count.frame_header[1],bit_count.frame_header[1]/np);
    printf("Super mode:            %9d  %9d  %9d  %9d\n",bit_count.super_mode[0],bit_count.super_mode[0]/ni,bit_count.super_mode[1],bit_count.super_mode[1]/np);
    printf("Intra mode:            %9d  %9d  %9d  %9d\n",bit_count.intra_mode[0],bit_count.intra_mode[0]/ni,bit_count.intra_mode[1],bit_count.intra_mode[1]/np);
    printf("MV:                    %9d  %9d  %9d  %9d\n",bit_count.mv[0],bit_count.mv[0],bit_count.mv[1],bit_count.mv[1]/np);
    printf("Skip idx:              %9d  %9d  %9d  %9d\n",bit_count.skip_idx[0],bit_count.skip_idx[0],bit_count.skip_idx[1],bit_count.skip_idx[1]/np);
    printf("Coeff_y:               %9d  %9d  %9d  %9d\n",bit_count.coeff_y[0],bit_count.coeff_y[0]/ni,bit_count.coeff_y[1],bit_count.coeff_y[1]/np);
    printf("Coeff_u:               %9d  %9d  %9d  %9d\n",bit_count.coeff_u[0],bit_count.coeff_u[0]/ni,bit_count.coeff_u[1],bit_count.coeff_u[1]/np);
    printf("Coeff_v:               %9d  %9d  %9d  %9d\n",bit_count.coeff_v[0],bit_count.coeff_v[0]/ni,bit_count.coeff_v[1],bit_count.coeff_v[1]/np);
    printf("CBP (TU-split):        %9d  %9d  %9d  %9d\n",bit_count.cbp[0],bit_count.cbp[0]/ni,bit_count.cbp[1],bit_count.cbp[1]/np);
    printf("CLPF:                  %9d  %9d  %9d  %9d\n",bit_count.clpf[0],bit_count.clpf[0]/ni,bit_count.clpf[1],bit_count.clpf[1]/np);
    printf("Total:                 %9d  %9d  %9d  %9d\n",tot_bits[0],tot_bits[0],tot_bits[1],tot_bits[1]/np);
    printf("-----------------------------------------------------------------\n\n");

    printf("PARAMETER STATISTICS:\n");
    printf("                           I pictures:           P pictures:\n");
    printf("                           total    average      total    average\n");
    printf("Skip-blocks (8x8):     %9d  %9d  %9d  %9d\n",bit_count.mode[0][0],bit_count.mode[0][0]/ni,bit_count.mode[1][0],bit_count.mode[1][0]/np);
    printf("Intra-blocks (8x8):    %9d  %9d  %9d  %9d\n",bit_count.mode[0][1],bit_count.mode[0][1]/ni,bit_count.mode[1][1],bit_count.mode[1][1]/np);
    printf("Inter-blocks (8x8):    %9d  %9d  %9d  %9d\n",bit_count.mode[0][2],bit_count.mode[0][2]/ni,bit_count.mode[1][2],bit_count.mode[1][2]/np);
    printf("Bipred-blocks (8x8):   %9d  %9d  %9d  %9d\n",bit_count.mode[0][3],bit_count.mode[0][3]/ni,bit_count.mode[1][3],bit_count.mode[1][3]/np);

    printf("\n");
    printf("8x8-blocks (8x8):      %9d  %9d  %9d  %9d\n",bit_count.size[0][0],bit_count.size[0][0]/ni,bit_count.size[1][0],bit_count.size[1][0]/np);
    printf("16x16-blocks (8x8):    %9d  %9d  %9d  %9d\n",bit_count.size[0][1],bit_count.size[0][1]/ni,bit_count.size[1][1],bit_count.size[1][1]/np);
    printf("32x32-blocks (8x8):    %9d  %9d  %9d  %9d\n",bit_count.size[0][2],bit_count.size[0][2]/ni,bit_count.size[1][2],bit_count.size[1][2]/np);
    printf("64x64-blocks (8x8):    %9d  %9d  %9d  %9d\n",bit_count.size[0][3],bit_count.size[0][3]/ni,bit_count.size[1][3],bit_count.size[1][3]/np);

    printf("\n");
    printf("Mode and size distribution for P- pictures:\n");
    printf("                            SKIP      INTRA      INTER     BIPRED\n");
    printf("8x8-blocks (8x8):      %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[0][0],bit_count.size_and_mode[0][1],bit_count.size_and_mode[0][2],bit_count.size_and_mode[0][3]);
    printf("16x16-blocks (8x8):    %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[1][0],bit_count.size_and_mode[1][1],bit_count.size_and_mode[1][2],bit_count.size_and_mode[1][3]);
    printf("32x32-blocks (8x8):    %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[2][0],bit_count.size_and_mode[2][1],bit_count.size_and_mode[2][2],bit_count.size_and_mode[2][3]);
    printf("64x64-blocks (8x8):    %9d  %9d  %9d  %9d\n",bit_count.size_and_mode[3][0],bit_count.size_and_mode[3][1],bit_count.size_and_mode[3][2],bit_count.size_and_mode[3][3]);

/*
    for (i=0;i<4;i++){
      double prob;
      int sum = 0;
      size = 1<<(i+3);
      printf("%2d x %2d-blocks: ",size,size);
      for (j=0;j<max_num_ref;j++){
        sum += bit_count.size_and_ref_idx[i][j];
      }
      for (j=0;j<max_num_ref;j++){
        prob = sum > 0 ? (double)bit_count.size_and_ref_idx[i][j]/(double)sum : 0.0;
        printf("%6.3f",prob);
      }
      printf("\n");
    }

    for (i=0;i<4;i++){
      double prob,entropy;
      int sum = 0;
      size = 1<<(i+3);
      printf("%2d x %2d-blocks: ",size,size);
      for (j=0;j<max_num_ref;j++){
        sum += bit_count.size_and_ref_idx[i][j];
      }
      for (j=0;j<max_num_ref;j++){
        prob = sum > 0 ? (double)bit_count.size_and_ref_idx[i][j]/(double)sum : 0.0;
        entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
        printf("%6.1f",entropy);
      }
      printf("\n");
    }
*/
#if 1//STAT
    int idx;//,sum;
    //double prob,entropy;
    int num=9;
    printf("\nSuper-mode distribution for P pictures:\n");
    int index;
    for (index=0;index<1;index++){
      //printf("index=%4d:\n",index);
      //printf("index=%4d:  ",index);
      //for (idx=0;idx<4;idx++){
      for (idx=0;idx<4;idx++){
        int size = 8<<idx;
        //printf("block-size: %2d x %2d\n",size,size);
        printf("%2d x %2d-blocks (8x8): ",size,size);
        //sum = 0;
        //printf("count:           ");
        //int divide = (size*size)/(MIN_BLOCK_SIZE*MIN_BLOCK_SIZE);
        for (i=0;i<num;i++){
          //sum += bit_count.super_mode_stat[index][idx][i];
          printf("%8d",bit_count.super_mode_stat[index][idx][i]/1);
        }
        printf("\n");
        /*
        printf("probability:     ");
        for (i=0;i<num;i++){
          prob = (double)bit_count.super_mode_stat[index][idx][i]/(double)sum;
          printf("%7.2f",prob);
        }
        printf("\n");

        printf("entropy:         ");
        for (i=0;i<num;i++){
          prob = (double)bit_count.super_mode_stat[index][idx][i]/(double)sum;
          entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
          printf("%7.2f",entropy);
        }
        printf("\n");
        */
      }
    }
#endif

    printf("\n");
    printf("Ref_idx and size distribution for P pictures:\n");
    int size;
    int max_num_ref = 4;
    for (i=0;i<4;i++){
      size = 1<<(i+3);
      printf("%2d x %2d-blocks: ",size,size);
      for (j=0;j<max_num_ref;j++){
        printf("%6d",bit_count.size_and_ref_idx[i][j]);
      }
      printf("\n");
    }

    {
      int sum = 0;
      printf("\nbi-ref:  ");
      for (j=0;j<max_num_ref*max_num_ref;j++){
        sum += bit_count.bi_ref[j];
        printf("%7d",bit_count.bi_ref[j]);
      }
      printf("\n");
    }

#if STAT
    int ft,mode,idx,sum,num;
    //double prob,entropy;

    int index;
    for (index=0;index<3;index++){
      printf("index=%4d:\n",index);
      num = 9; //Including tb_split
      printf("I-frames:\n");
      ft = I_FRAME;
      mode = MODE_INTRA - 1;
      for (idx=0;idx<4;idx++){
        sum = 0;
        printf("size=%4d count:           ",8<<idx);
        for (i=0;i<num;i++){
          sum += bit_count.cbp2_stat[index][ft][mode][idx][i];
          printf("%6d",bit_count.cbp2_stat[index][ft][mode][idx][i]);
        }
        printf("\n");
  /*
        printf("probability:     ");
        for (i=0;i<num;i++){
          prob = (double)bit_count.cbp2_stat[ft][mode][idx][i]/(double)sum;
          printf("%6.1f",prob);
        }
        printf("\n");

        printf("entropy:         ");
        for (i=0;i<num;i++){
          prob = (double)bit_count.cbp2_stat[ft][mode][idx][i]/(double)sum;
          entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
          printf("%6.1f",entropy);
        }
        printf("\n");
        */
      }

      printf("P-frames-intra:\n");
      ft = P_FRAME;
      mode = MODE_INTRA - 1;
      for (idx=0;idx<4;idx++){
        sum = 0;
        printf("size=%4d count:           ",8<<idx);
        for (i=0;i<num;i++){
          sum += bit_count.cbp2_stat[index][ft][mode][idx][i];
          printf("%6d",bit_count.cbp2_stat[index][ft][mode][idx][i]);
        }
        printf("\n");
  /*
        printf("probability:     ");
        for (i=0;i<num;i++){
          prob = (double)bit_count.cbp2_stat[ft][mode][idx][i]/(double)sum;
          printf("%6.1f",prob);
        }
        printf("\n");

        printf("entropy:         ");
        for (i=0;i<num;i++){
          prob = (double)bit_count.cbp2_stat[ft][mode][idx][i]/(double)sum;
          entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
          printf("%6.1f",entropy);
        }
        printf("\n");
        */
      }

      printf("P-frames-inter:\n");
      ft = P_FRAME;
      mode = MODE_INTER - 1;
      for (idx=0;idx<4;idx++){
        sum = 0;
        printf("size=%4d count:           ",8<<idx);
        for (i=0;i<num;i++){
          sum += bit_count.cbp2_stat[index][ft][mode][idx][i];
          printf("%6d",bit_count.cbp2_stat[index][ft][mode][idx][i]);
        }
        printf("\n");
  /*
        printf("probability:     ");
        for (i=0;i<num;i++){
          prob = (double)bit_count.cbp2_stat[ft][mode][idx][i]/(double)sum;
          printf("%6.1f",prob);
        }
        printf("\n");

        printf("entropy:         ");
        for (i=0;i<num;i++){
          prob = (double)bit_count.cbp2_stat[ft][mode][idx][i]/(double)sum;
          entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
          printf("%6.1f",entropy);
        }
        printf("\n");
        */
      }
    } //for index
#endif

#if STAT
    int size,sum;
    double prob,entropy;
    printf("\n");

    printf("Intra mode and size distribution for I frames (total):\n");
    for (i=0;i<4;i++){
      size = 1<<(i+3);
      printf("%2d x %2d - blocks: ",size,size);
      for (j=0;j<10;j++){
        printf("%6d",bit_count.size_and_intra_mode[0][i][j]);
      }
      printf("\n");
    }
    for (i=0;i<4;i++){
      size = 1<<(i+3);
      printf("%2d x %2d - blocks: ",size,size);
      sum = 0;
      for (j=0;j<10;j++){
        sum += bit_count.size_and_intra_mode[0][i][j];
      }
      for (j=0;j<10;j++){
        prob = sum > 0 ? (double)bit_count.size_and_intra_mode[0][i][j]/(double)sum : 0.0;
        printf("%6.3f",prob);
      }
      printf("\n");
    }
    for (i=0;i<4;i++){
      size = 1<<(i+3);
      printf("%2d x %2d - blocks: ",size,size);
      sum = 0;
      for (j=0;j<10;j++){
        sum += bit_count.size_and_intra_mode[0][i][j];
      }
      for (j=0;j<10;j++){
        prob = sum > 0 ? (double)bit_count.size_and_intra_mode[0][i][j]/(double)sum : 0.0;
        entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
        printf("%6.1f",entropy);
      }
      printf("\n");
    }

    printf("Intra mode and size distribution for P frames (total):\n");
    for (i=0;i<4;i++){
      size = 1<<(i+3);
      printf("%2d x %2d - blocks: ",size,size);
      for (j=0;j<10;j++){
        printf("%6d",bit_count.size_and_intra_mode[1][i][j]);
      }
      printf("\n");
    }
    for (i=0;i<4;i++){
      size = 1<<(i+3);
      printf("%2d x %2d - blocks: ",size,size);
      sum = 0;
      for (j=0;j<10;j++){
        sum += bit_count.size_and_intra_mode[1][i][j];
      }
      for (j=0;j<10;j++){
        prob = sum > 0 ? (double)bit_count.size_and_intra_mode[1][i][j]/(double)sum : 0.0;
        printf("%6.3f",prob);
      }
      printf("\n");
    }
    for (i=0;i<4;i++){
      size = 1<<(i+3);
      printf("%2d x %2d - blocks: ",size,size);
      sum = 0;
      for (j=0;j<10;j++){
        sum += bit_count.size_and_intra_mode[1][i][j];
      }
      for (j=0;j<10;j++){
        prob = sum > 0 ? (double)bit_count.size_and_intra_mode[1][i][j]/(double)sum : 0.0;
        entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
        printf("%6.1f",entropy);
      }
      printf("\n");
    }

    printf("pred_info and size distribution:\n");
    for (i=0;i<4;i++){
      size = 1<<(i+3);
      printf("%2d x %2d - blocks: ",size,size);
      for (j=0;j<8;j++){
        printf("%6d",bit_count.size_and_pred_info[i][j]);
      }
      printf("\n");
    }

    for (i=0;i<4;i++){
      double prob;
      int sum = 0;
      size = 1<<(i+3);
      printf("%2d x %2d - blocks: ",size,size);
      for (j=0;j<8;j++){
        sum += bit_count.size_and_pred_info[i][j];
      }
      for (j=0;j<8;j++){
        prob = sum > 0 ? (double)bit_count.size_and_pred_info[i][j]/(double)sum : 0.0;
        printf("%6.3f",prob);
      }
      printf("\n");
    }

    for (i=0;i<4;i++){
      double prob,entropy;
      int sum = 0;
      size = 1<<(i+3);
      printf("%2d x %2d - blocks: ",size,size);
      for (j=0;j<8;j++){
        sum += bit_count.size_and_pred_info[i][j];
      }
      for (j=0;j<8;j++){
        prob = sum > 0 ? (double)bit_count.size_and_pred_info[i][j]/(double)sum : 0.0;
        entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
        printf("%6.1f",entropy);
      }
      printf("\n");
    }


    int idx;
    int bit,totbit,allbits;
    int ft;
    for (ft=0;ft<2;ft++){
      if (ft==0)
        printf("\nSuper-mode for intra:\n");
      else
        printf("\nSuper-mode for inter:\n");
      for (idx=0;idx<4;idx++){
        int size = 8<<idx;
        printf("block-size: %2d x %2d\n",size,size);
        sum = 0;
        printf("count:           ");
        for (i=0;i<20;i++){
          sum += bit_count.super_mode_stat[ft][idx][i];
          printf("%7d",bit_count.super_mode_stat[ft][idx][i]);
        }
        printf("\n");

        printf("probability:     ");
        for (i=0;i<20;i++){
          prob = (double)bit_count.super_mode_stat[ft][idx][i]/(double)sum;
          printf("%7.1f",prob);
        }
        printf("\n");

        printf("entropy:         ");
        for (i=0;i<20;i++){
          prob = (double)bit_count.super_mode_stat[ft][idx][i]/(double)sum;
          entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
          printf("%7.1f",entropy);
        }
        printf("\n");

        printf("bits:            ");
        for (i=0;i<20;i++){
          bit = i < 2 ? i + 1 : i/2 + 3;
          printf("%7d",bit);
        }
        printf("\n");

        printf("totbits:         ");
        allbits = 0;
        for (i=0;i<20;i++){
          bit = i < 2 ? i + 1 : i/2 + 3;
          totbit = bit * bit_count.super_mode_stat[ft][idx][i];
          allbits += totbit;
          printf("%7d",totbit);
        }
        printf("   %10d\n",allbits);
      }
    }


    int sum0 = 0;
    for (i=0;i<8;i++){
      sum0 += bit_count.cbp_stat[0][i];
    }
    for (i=0;i<8;i++){
      sum0 += bit_count.cbp_stat[0][i];
    }

    if (sum0){
      /* CBP stat */
      printf("\nCBP for intra:\n");
      idx = 0;
      sum = 0;
      printf("count:           ");
      for (i=0;i<8;i++){
        sum += bit_count.cbp_stat[idx][i];
        printf("%6d",bit_count.cbp_stat[idx][i]);
      }
      printf("\n");

      printf("probability:     ");
      for (i=0;i<8;i++){
        prob = (double)bit_count.cbp_stat[idx][i]/(double)sum;
        printf("%6.1f",prob);
      }
      printf("\n");

      printf("entropy:         ");
      for (i=0;i<8;i++){
        prob = (double)bit_count.cbp_stat[idx][i]/(double)sum;
        entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
        printf("%6.1f",entropy);
      }
      printf("\n");

      printf("\nCBP for inter :\n");
      idx = 1;
      sum = 0;
      printf("count:           ");
      for (i=0;i<8;i++){
        sum += bit_count.cbp_stat[idx][i];
        printf("%6d",bit_count.cbp_stat[idx][i]);
      }
      printf("\n");

      printf("probability:     ");
      for (i=0;i<8;i++){
        prob = (double)bit_count.cbp_stat[idx][i]/(double)sum;
        printf("%6.1f",prob);
      }
      printf("\n");

      printf("entropy:         ");
      for (i=0;i<8;i++){
        prob = (double)bit_count.cbp_stat[idx][i]/(double)sum;
        entropy = prob > 0.0 ? -log(prob)/log(2.0) : 0.0;
        printf("%6.1f",entropy);
      }
      printf("\n");

      printf("bits:            ");
      for (i=0;i<8;i++){
        //bit = 1 + 2*leading_zeros(i+1)
        if (i==0)
          bit = 2;
        else if (i==1)
          bit = 1;
        else
          bit = 5;
        printf("%6d",bit);
      }
      printf("\n");

      printf("totbits:         ");
      allbits = 0;
      for (i=0;i<8;i++){
        if (i==0)
          bit = 2;
        else if (i==1)
          bit = 1;
        else
          bit = 5;
        totbit = bit * bit_count.cbp_stat[idx][i];
        allbits += totbit;
        printf("%6d",totbit);
      }
      printf("   %10d\n",allbits);
    }
#endif
    printf("-----------------------------------------------------------------\n");
    for (r=0;r<MAX_REORDER_BUFFER;r++){
      close_yuv_frame(&rec[r]);
    }
    for (r=0;r<MAX_REF_FRAMES;r++){
      close_yuv_frame(&ref[r]);
    }
    free(decoder_info.deblock_data);

    return 0;
} 
