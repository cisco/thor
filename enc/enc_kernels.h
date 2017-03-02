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

#ifndef ENC_SIMDKERNELS_H
#define ENC_SIMDKERNELS_H

#include <stdint.h>

int TEMPLATE(sad_calc_simd)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int width, int height);
uint64_t TEMPLATE(ssd_calc_simd)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int size);
void TEMPLATE(detect_clpf_simd)(const SAMPLE *rec,const SAMPLE *org,int x0, int y0, int width, int height, int so,int stride, int *sum0, int *sum1, unsigned int strength, unsigned int shift, unsigned int size, unsigned int dmp);
void TEMPLATE(detect_multi_clpf_simd)(const SAMPLE *rec,const SAMPLE *org,int x0, int y0, int width, int height, int so,int stride, int *sum, unsigned int shift, int unsigned size, unsigned int dmp);
unsigned int TEMPLATE(sad_calc_fasthalf_simd)(const SAMPLE *a, const SAMPLE *b, int astride, int bstride, int width, int height, int *x, int *y);
unsigned int TEMPLATE(sad_calc_fastquarter_simd)(const SAMPLE *o, const SAMPLE *r, int os, int rs, int width, int height, int *x, int *y);
unsigned int TEMPLATE(widesad_calc_simd)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int width, int height, int *x);

int calc_cbp_simd(int16_t *block, int size, int threshold);
#endif
