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
#include "snr.h"

int snr_yuv(snrvals *psnr,yuv_frame_t *f1,yuv_frame_t *f2,int height,int width,int stride_y,int stride_c)
{
    unsigned int ydim,ydim_chr;
    unsigned int i,j,xdim,xdim_chr;
    double sumsqr=0;
    int ival;
    double plse;

    xdim = width;
    xdim_chr = xdim >> 1;
    ydim = height;
    ydim_chr = ydim >> 1;

    /* Calculate psnr for Y */
    sumsqr = 0;
    for (i = 0; i < ydim; i++)
    {
      for (j = 0; j < xdim; j++)
      {
        ival = abs(f1->y[i*stride_y+j] - f2->y[i*stride_y+j]);
        sumsqr += (float)(ival * ival);
      }
    }
    plse = sumsqr / (65025.0 * ydim * xdim);
    psnr->y = -10 * log10(plse);
    
    /* Calculate psnr for U */
    sumsqr = 0;
    for (i = 0; i < ydim_chr; i++)
    {
      for (j = 0; j < xdim_chr; j++)
      {
        ival = abs(f1->u[i*stride_c+j] - f2->u[i*stride_c+j]);
        sumsqr += (ival * ival);
      }
    }
    plse = sumsqr / (65025.0 * ydim_chr * xdim_chr);
    psnr->u = -10 * log10(plse);
    
    /* Calculate psnr for V */
    sumsqr = 0;
    for (i = 0; i < ydim_chr; i++)
    {
      for (j = 0; j < xdim_chr; j++)
      {
        ival = abs(f1->v[i*stride_c+j] - f2->v[i*stride_c+j]);
        sumsqr += (ival * ival);
      }
    }
    plse = sumsqr / (65025.0 * ydim_chr * xdim_chr);
    psnr->v = -10 * log10(plse);
    return 0;
}

