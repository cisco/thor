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

int TEMPLATE(snr_yuv)(snrvals *psnr,yuv_frame_t *f1,yuv_frame_t *f2,int height,int width,int input_bitdepth)
{
    unsigned int ydim,ydim_chr;
    unsigned int i,j,xdim,xdim_chr;
    double sumsqr=0;
    int ival;
    double plse;
    int shift1 = f1->bitdepth - input_bitdepth;
    int shift2 = f2->bitdepth - input_bitdepth;

    xdim = width;
    xdim_chr = xdim >> f1->sub;
    ydim = height;
    ydim_chr = ydim >> f1->sub;

    int s1y = f1->stride_y;
    int s2y = f2->stride_y;
    int s1c = f1->stride_c;
    int s2c = f2->stride_c;
    int round1 = !!shift1 << (shift1 - 1);
    int round2 = !!shift2 << (shift2 - 1);
    double maxsignal = (double)((1 << input_bitdepth) - 1);

    /* Calculate psnr for Y */
    sumsqr = 0;
    for (i = 0; i < ydim; i++)
      for (j = 0; j < xdim; j++) {
        ival =
          (shift1 < 0 ? f1->y[i*s1y+j] << -shift1 : saturate(((f1->y[i*s1y+j] + round1) >> shift1), input_bitdepth)) -
          (shift2 < 0 ? f2->y[i*s2y+j] << -shift2 : saturate(((f2->y[i*s2y+j] + round2) >> shift2), input_bitdepth));
        sumsqr += (float)(ival * ival);
    }
    plse = sumsqr / (maxsignal * maxsignal * ydim * xdim);
    psnr->y = -10 * log10(plse);

    if (f1->subsample == 400) {
      psnr->u = psnr->v = 0;
      return 0;
    }

    /* Calculate psnr for U */
    sumsqr = 0;
    for (i = 0; i < ydim_chr; i++)
      for (j = 0; j < xdim_chr; j++) {
        ival =
          (shift1 < 0 ? f1->u[i*s1c+j] << -shift1 : saturate(((f1->u[i*s1c+j] + round1) >> shift1), input_bitdepth)) -
          (shift2 < 0 ? f2->u[i*s2c+j] << -shift2 : saturate(((f2->u[i*s2c+j] + round2) >> shift2), input_bitdepth));
        sumsqr += (ival * ival);
    }
    plse = sumsqr / (maxsignal * maxsignal * ydim_chr * xdim_chr);
    psnr->u = -10 * log10(plse);
    
    /* Calculate psnr for V */
    sumsqr = 0;
    for (i = 0; i < ydim_chr; i++)
      for (j = 0; j < xdim_chr; j++) {
        ival =
          (shift1 < 0 ? f1->v[i*s1c+j] << -shift1 : saturate(((f1->v[i*s1c+j] + round1) >> shift1), input_bitdepth)) -
          (shift2 < 0 ? f2->v[i*s2c+j] << -shift2 : saturate(((f2->v[i*s2c+j] + round2) >> shift2), input_bitdepth));
        sumsqr += (ival * ival);
    }
    plse = sumsqr / (maxsignal * maxsignal * ydim_chr * xdim_chr);
    psnr->v = -10 * log10(plse);
    return 0;
}

