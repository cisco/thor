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
#include <memory.h>
#include "types.h"
#include "global.h"

int beta_table[52] = {
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
     16, 17, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64
};

static const int tc_table[56] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,6,7,8,9,9,10,10,11,11,12,12,13,13,14,14
};

void deblock_frame_y(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp)
{
  int i,j,k,l,d;
  int stride = rec->stride_y;
#if NEW_DEBLOCK_TEST
  int p12,p02,q02,q12;
  int p15,p05,q05,q15;
#else
  int p22,p12,p02,q02,q12,q22;
  int p25,p15,p05,q05,q15,q25;
#endif
  int p2,p1,p0,q0,q1,q2;
  uint8_t *recY = rec->y;
  uint8_t do_filter;
  uint8_t beta = beta_table[qp];
  uint8_t tc = tc_table[qp]; //TODO: increment with 4 for intra

  int p_index,q_index;
  mv_t p_mv,q_mv; //TODO: Rename to p_mv0 nd q_mv0
  mv_t p_mv1,q_mv1;
  block_mode_t p_mode,q_mode;
  int p_cbp,q_cbp;
  int q_size;
  int mv,mode,cbp,interior;
  int delta;

  /* Vertical filtering */
  for (i=0;i<height;i+=MIN_BLOCK_SIZE){
    for (j = MIN_BLOCK_SIZE; j<width; j += MIN_BLOCK_SIZE) {

#if NEW_DEBLOCK_TEST
      p12 = recY[(i+2)*stride + j - 2];
      p02 = recY[(i+2)*stride + j - 1];
      q02 = recY[(i+2)*stride + j + 0];
      q12 = recY[(i+2)*stride + j + 1];

      p15 = recY[(i+5)*stride + j - 2];
      p05 = recY[(i+5)*stride + j - 1];
      q05 = recY[(i+5)*stride + j + 0];
      q15 = recY[(i+5)*stride + j + 1];
      d = abs(p12-p02) + abs(q12-q02) + abs(p15-p05) + abs(q15-q05);
#else
      p22 = recY[(i+2)*stride + j - 3];
      p12 = recY[(i+2)*stride + j - 2];
      p02 = recY[(i+2)*stride + j - 1];
      q02 = recY[(i+2)*stride + j + 0];
      q12 = recY[(i+2)*stride + j + 1];
      q22 = recY[(i+2)*stride + j + 2];

      p25 = recY[(i+5)*stride + j - 3];
      p15 = recY[(i+5)*stride + j - 2];
      p05 = recY[(i+5)*stride + j - 1];
      q05 = recY[(i+5)*stride + j + 0];
      q15 = recY[(i+5)*stride + j + 1];
      q25 = recY[(i+5)*stride + j + 2];
      d = abs(p22-2*p12+p02) + abs(q22-2*q12+q02) + abs(p25-2*p15+p05) + abs(q25-2*q15+q05);
#endif
      int m;
      for (m=0;m<MIN_BLOCK_SIZE;m+=MIN_PB_SIZE){
        q_index = ((i+m)/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (j/MIN_PB_SIZE);
        p_index = q_index - 1;
        p_mv.x = deblock_data[p_index].mvb.x0;
        p_mv.y = deblock_data[p_index].mvb.y0;
        q_mv.x = deblock_data[q_index].mvb.x0;
        q_mv.y = deblock_data[q_index].mvb.y0;
        p_mv1.x = deblock_data[p_index].mvb.x1;
        p_mv1.y = deblock_data[p_index].mvb.y1;
        q_mv1.x = deblock_data[q_index].mvb.x1;
        q_mv1.y = deblock_data[q_index].mvb.y1;
        p_mode = deblock_data[p_index].mode;
        q_mode = deblock_data[q_index].mode;
        p_cbp = deblock_data[p_index].cbp.y;
        q_cbp = deblock_data[q_index].cbp.y;
        q_size = deblock_data[q_index].size;
        if ((deblock_data[q_index].tb_split || deblock_data[q_index].pb_part == PART_VER || deblock_data[q_index].pb_part == PART_QUAD) && q_size > MIN_BLOCK_SIZE) q_size = q_size/2;

#if NEW_MV_TEST
        mv = abs(p_mv.y) >= 4 || abs(q_mv.y) >= 4 || abs(p_mv.x) >= 4 || abs(q_mv.x) >= 4; //TODO: Investigate >=3 instead
        mv = mv || abs(p_mv1.y) >= 4 || abs(q_mv1.y) >= 4 || abs(p_mv1.x) >= 4 || abs(q_mv1.x) >= 4;
#else
        mv = abs(p_mv.y - q_mv.y) >= 2 || abs(p_mv.x - q_mv.x) >= 2;
        mv = mv || abs(p_mv1.y - q_mv1.y) >= 2 || abs(p_mv1.x - q_mv1.x) >= 2;
#endif
        cbp = p_cbp || q_cbp;
        mode = p_mode == MODE_INTRA || q_mode == MODE_INTRA;
        interior = j%q_size > 0 ? 1 : 0;
        do_filter = (d < beta) && !interior && (mv || cbp || mode); //TODO: This logic needs to support 4x4TUs
        if (do_filter){
          for (k=m;k<m+MIN_PB_SIZE;k++){
            p2 = (int)recY[(i+k)*stride + j - 3];
            p1 = (int)recY[(i+k)*stride + j - 2];
            p0 = (int)recY[(i+k)*stride + j - 1];
            q0 = (int)recY[(i+k)*stride + j + 0];
            q1 = (int)recY[(i+k)*stride + j + 1];
            q2 = (int)recY[(i+k)*stride + j + 2];
#if NEW_DEBLOCK_FILTER
            delta = (18*(q0-p0) - 6*(q1-p1) + 0*(q2-p2) + 16)>>5;
#else
            delta = (13*(q0-p0) + 4*(q1-p1) - 5*(q2-p2) + 16)>>5;
#endif
            delta = clip(delta,-tc,tc);

            recY[(i+k)*stride + j - 2] = (uint8_t)clip255(p1 + delta/2);
            recY[(i+k)*stride + j - 1] = (uint8_t)clip255(p0 + delta);
            recY[(i+k)*stride + j + 0] = (uint8_t)clip255(q0 - delta);
            recY[(i+k)*stride + j + 1] = (uint8_t)clip255(q1 - delta/2);
          }
        }
      }

    }
  }

  /* Horizontal filtering */
  for (i = MIN_BLOCK_SIZE; i<height; i += MIN_BLOCK_SIZE) {
    for (j=0;j<width;j+=MIN_BLOCK_SIZE){

#if NEW_DEBLOCK_TEST
      p12 = recY[(i-2)*stride + j + 2];
      p02 = recY[(i-1)*stride + j + 2];
      q02 = recY[(i+0)*stride + j + 2];
      q12 = recY[(i+1)*stride + j + 2];

      p15 = recY[(i-2)*stride + j + 5];
      p05 = recY[(i-1)*stride + j + 5];
      q05 = recY[(i+0)*stride + j + 5];
      q15 = recY[(i+1)*stride + j + 5];
      d = abs(p12-p02) + abs(q12-q02) + abs(p15-p05) + abs(q15-q05);
#else
      p22 = recY[(i-3)*stride + j + 2];
      p12 = recY[(i-2)*stride + j + 2];
      p02 = recY[(i-1)*stride + j + 2];
      q02 = recY[(i+0)*stride + j + 2];
      q12 = recY[(i+1)*stride + j + 2];
      q22 = recY[(i+2)*stride + j + 2];

      p25 = recY[(i-3)*stride + j + 5];
      p15 = recY[(i-2)*stride + j + 5];
      p05 = recY[(i-1)*stride + j + 5];
      q05 = recY[(i+0)*stride + j + 5];
      q15 = recY[(i+1)*stride + j + 5];
      q25 = recY[(i+2)*stride + j + 5];
      d = abs(p22-2*p12+p02) + abs(q22-2*q12+q02) + abs(p25-2*p15+p05) + abs(q25-2*q15+q05);
#endif

      int n;
      for (n=0;n<MIN_BLOCK_SIZE;n+=MIN_PB_SIZE){
        q_index = (i/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + ((j+n)/MIN_PB_SIZE);
        p_index = q_index - (width/MIN_PB_SIZE);
        p_mv.x = deblock_data[p_index].mvb.x0;
        p_mv.y = deblock_data[p_index].mvb.y0;
        q_mv.x = deblock_data[q_index].mvb.x0;
        q_mv.y = deblock_data[q_index].mvb.y0;

        p_mv1.x = deblock_data[p_index].mvb.x1;
        p_mv1.y = deblock_data[p_index].mvb.y1;
        q_mv1.x = deblock_data[q_index].mvb.x1;
        q_mv1.y = deblock_data[q_index].mvb.y1;

        p_mode = deblock_data[p_index].mode;
        q_mode = deblock_data[q_index].mode;
        p_cbp = deblock_data[p_index].cbp.y;
        q_cbp = deblock_data[q_index].cbp.y;
        q_size = deblock_data[q_index].size;

        if ((deblock_data[q_index].tb_split || deblock_data[q_index].pb_part == PART_HOR || deblock_data[q_index].pb_part == PART_QUAD) && q_size > MIN_BLOCK_SIZE) q_size = q_size/2;

#if NEW_MV_TEST
        mv = abs(p_mv.y) >= 4 || abs(q_mv.y) >= 4 || abs(p_mv.x) >= 4 || abs(q_mv.x) >= 4;
        mv = mv || abs(p_mv1.y) >= 4 || abs(q_mv1.y) >= 4 || abs(p_mv1.x) >= 4 || abs(q_mv1.x) >= 4;
#else
        mv = abs(p_mv.y - q_mv.y) >= 2 || abs(p_mv.x - q_mv.x) >= 2;
        mv = mv || abs(p_mv1.y - q_mv1.y) >= 2 || abs(p_mv1.x - q_mv1.x) >= 2;
#endif

        cbp = p_cbp || q_cbp;
        mode = p_mode == MODE_INTRA || q_mode == MODE_INTRA;
        interior = i%q_size > 0 ? 1 : 0;
        do_filter = (d < beta) && !interior && (mv || cbp || mode); //TODO: This logic needs to support 4x4TUs
        if (do_filter){
          for (l=n;l<n+MIN_PB_SIZE;l++){
            p2 = (int)recY[(i-3)*stride + j + l];
            p1 = (int)recY[(i-2)*stride + j + l];
            p0 = (int)recY[(i-1)*stride + j + l];
            q0 = (int)recY[(i+0)*stride + j + l];
            q1 = (int)recY[(i+1)*stride + j + l];
            q2 = (int)recY[(i+2)*stride + j + l];

#if NEW_DEBLOCK_FILTER
            delta = (18*(q0-p0) - 6*(q1-p1) + 0*(q2-p2) + 16)>>5;
#else
            delta = (13*(q0-p0) + 4*(q1-p1) - 5*(q2-p2) + 16)>>5;
#endif
            delta = clip(delta,-tc,tc);

            recY[(i-2)*stride + j + l] = (uint8_t)clip255(p1 + delta/2);
            recY[(i-1)*stride + j + l] = (uint8_t)clip255(p0 + delta);
            recY[(i+0)*stride + j + l] = (uint8_t)clip255(q0 - delta);
            recY[(i+1)*stride + j + l] = (uint8_t)clip255(q1 - delta/2);
          }
        }
      }

    }
  }
}

void deblock_frame_uv(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp)
{
  int i,j,k,l;
  int stride = rec->stride_c;
  int p1,p0,q0,q1;

  uint8_t do_filter;
  uint8_t tc = tc_table[qp];

  int p_index,q_index;
  block_mode_t p_mode,q_mode;
  int q_size;
  int mode,interior;
  int delta;

  for (int uv=0;uv<2;uv++){

    uint8_t *recC = (uv ? rec->v : rec->u);

    /* Vertical filtering */
    for (i=0;i<height;i+=MIN_BLOCK_SIZE){
      for (j = MIN_BLOCK_SIZE; j<width; j += MIN_BLOCK_SIZE) {
        int i2 = i/2;
        int j2 = j/2;
        q_index = (i/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (j/MIN_PB_SIZE);
        p_index = q_index - 1;

        p_mode = deblock_data[p_index].mode;
        q_mode = deblock_data[q_index].mode;
        q_size = deblock_data[q_index].size;

        mode = p_mode == MODE_INTRA || q_mode == MODE_INTRA;
        interior = j%q_size > 0 ? 1 : 0;
        do_filter = !interior && mode;
        if (do_filter){
          for (k=0;k<MIN_BLOCK_SIZE/2;k++){
            p1 = (int)recC[(i2+k)*stride + j2 - 2];
            p0 = (int)recC[(i2+k)*stride + j2 - 1];
            q0 = (int)recC[(i2+k)*stride + j2 + 0];
            q1 = (int)recC[(i2+k)*stride + j2 + 1];
            delta = (4*(q0-p0) + (p1-q1) + 4)>>3;
            delta = clip(delta,-tc,tc);
            recC[(i2+k)*stride + j2 - 1] = (uint8_t)clip255(p0 + delta);
            recC[(i2+k)*stride + j2 + 0] = (uint8_t)clip255(q0 - delta);
          }
        }
      }
    }

    /* Horizontal filtering */
    for (i = MIN_BLOCK_SIZE; i<height; i += MIN_BLOCK_SIZE) {
      for (j=0;j<width;j+=MIN_BLOCK_SIZE){
        int i2 = i/2;
        int j2 = j/2;
        q_index = (i/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (j/MIN_PB_SIZE);
        p_index = q_index - (width/MIN_PB_SIZE);
        p_mode = deblock_data[p_index].mode;
        q_mode = deblock_data[q_index].mode;
        q_size = deblock_data[q_index].size;

        mode = p_mode == MODE_INTRA || q_mode == MODE_INTRA;
        interior = i%q_size > 0 ? 1 : 0;
        do_filter = !interior && mode;
        if (do_filter){
          for (l=0;l<MIN_BLOCK_SIZE/2;l++){
            p1 = (int)recC[(i2-2)*stride + j2 + l];
            p0 = (int)recC[(i2-1)*stride + j2 + l];
            q0 = (int)recC[(i2+0)*stride + j2 + l];
            q1 = (int)recC[(i2+1)*stride + j2 + l];
            delta = (4*(q0-p0) + (p1-q1) + 4)>>3;
            delta = clip(delta,-tc,tc);
            recC[(i2-1)*stride + j2 + l] = (uint8_t)clip255(p0 + delta);
            recC[(i2+0)*stride + j2 + l] = (uint8_t)clip255(q0 - delta);
          }
        }
      }
    }
  }
}


void create_yuv_frame(yuv_frame_t  *frame, int width, int height, int pad_ver_y, int pad_hor_y, int pad_ver_uv, int pad_hor_uv)
{  
  frame->width = width;
  frame->height = height;
  frame->stride_y = width + 2*pad_hor_y;
  frame->stride_c = width/2 + 2*pad_hor_uv;
  frame->offset_y = pad_ver_y * frame->stride_y + pad_hor_y;
  frame->offset_c = pad_ver_uv * frame->stride_c + pad_hor_uv;  
  frame->y = (uint8_t *)malloc((height + 2*pad_ver_y) * frame->stride_y * sizeof(uint8_t));
  frame->u = (uint8_t *)malloc((height/2 + 2*pad_ver_uv) * frame->stride_c * sizeof(uint8_t));
  frame->v = (uint8_t *)malloc((height/2 + 2*pad_ver_uv) * frame->stride_c * sizeof(uint8_t)); 
}

void close_yuv_frame(yuv_frame_t  *frame)
{
  free(frame->y);
  free(frame->u);
  free(frame->v); 
}

void read_yuv_frame(yuv_frame_t  *frame, int width, int height, FILE *infile)
{
	{
		unsigned int ysize = width*height;
		unsigned int csize = ysize/4;
		if (fread(frame->y, sizeof(unsigned char), ysize, infile) != ysize)
		{
     fatalerror("Error reading Y from file");
		}
		if (fread(frame->u, sizeof(unsigned char), csize, infile) != csize)
		{
			fatalerror("Error reading U from file");
		}
		if (fread(frame->v, sizeof(unsigned char), csize, infile) != csize)
		{
			fatalerror("Error reading V from file");
		}
	}
}

void write_yuv_frame(yuv_frame_t  *frame, int width, int height, FILE *outfile)
{
	unsigned int ysize = width*height;
	unsigned int csize = ysize/4;
	if (fwrite(frame->y, sizeof(unsigned char), ysize, outfile) != ysize)
  {
		fatalerror("Error writing Y to file");
  }
  if (fwrite(frame->u, sizeof(unsigned char), csize, outfile)	!= csize)
  {
		fatalerror("Error writing U to file");    
  }
	if (fwrite(frame->v, sizeof(unsigned char), csize, outfile)	!= csize)
  {
		fatalerror("Error writing V to file");    
  }
}

void create_reference_frame(yuv_frame_t  *ref,yuv_frame_t  *rec)
{
  ref->frame_num = rec->frame_num;
  int height = rec->height;
  int width = rec->width;  
  int i,j;
  int offset_y = PADDING_Y * ref->stride_y + PADDING_Y;
  int offset_c = PADDING_Y/2 * ref->stride_c + PADDING_Y/2;

  uint8_t *ref_y = ref->y + offset_y;
  uint8_t *ref_u = ref->u + offset_c;
  uint8_t *ref_v = ref->v + offset_c;
  for (i=0;i<height;i++){
    memcpy(&ref_y[i*ref->stride_y],&rec->y[i*rec->stride_y],width*sizeof(uint8_t)); 
  }
  for (i=0;i<height/2;i++){
    memcpy(&ref_u[i*ref->stride_c],&rec->u[i*rec->stride_c],width/2*sizeof(uint8_t));
    memcpy(&ref_v[i*ref->stride_c],&rec->v[i*rec->stride_c],width/2*sizeof(uint8_t));
  }

  /* Y */
  /* Left and right */
  for (i=0;i<height;i++)
  {
    for (j=-PADDING_Y;j<0;j++)
    {
      ref_y[i*ref->stride_y + j] = rec->y[i*rec->stride_y];
    }
    for (j=width;j<width+PADDING_Y;j++)
    {
      ref_y[i*ref->stride_y + j] = rec->y[i*rec->stride_y + width - 1];
    }
  }
  /* Top and bottom */
  for (i=-PADDING_Y;i<0;i++)
  {
    for (j=-PADDING_Y;j<width+PADDING_Y;j++)
    {
      ref_y[i*ref->stride_y + j] = ref_y[0*ref->stride_y + j];
    }
  }
  for (i=height;i<height+PADDING_Y;i++)
  {
    for (j=-PADDING_Y;j<width+PADDING_Y;j++)
    {
      ref_y[i*ref->stride_y + j] = ref_y[(height-1)*ref->stride_y + j];
    }
  }

  /* UV */

 /* Left and right */
  for (i=0;i<height/2;i++)
  {
    for (j=-PADDING_Y/2;j<0;j++)
    {
      ref_u[i*ref->stride_c + j] = rec->u[i*rec->stride_c];
      ref_v[i*ref->stride_c + j] = rec->v[i*rec->stride_c];
    }
    for (j=width/2;j<width/2+PADDING_Y/2;j++)
    {
      ref_u[i*ref->stride_c + j] = rec->u[i*rec->stride_c + width/2 - 1];
      ref_v[i*ref->stride_c + j] = rec->v[i*rec->stride_c + width/2 - 1];
    }
  }
  /* Top and bottom */
  for (i=-PADDING_Y/2;i<0;i++)
  {
    for (j=-PADDING_Y/2;j<width/2+PADDING_Y/2;j++)
    {
      ref_u[i*ref->stride_c+j] = ref_u[0*ref->stride_c+j];
      ref_v[i*ref->stride_c+j] = ref_v[0*ref->stride_c+j];
    }
  }
  for (i=height/2;i<height/2+PADDING_Y/2;i++)
  {
    for (j=-PADDING_Y/2;j<width/2+PADDING_Y/2;j++)
    {
      ref_u[i*ref->stride_c+j] = ref_u[(height/2-1)*ref->stride_c+j];
      ref_v[i*ref->stride_c+j] = ref_v[(height/2-1)*ref->stride_c+j];
    }
  }

}
