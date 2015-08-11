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

/* -*- mode: c; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2; -*- */

#include "simd.h"

int sad_calc_simd(uint8_t *a, uint8_t *b, int astride, int bstride, int width, int height)
{
  int i, j;

  if (width == 8) {
    sad64_internal s = v64_sad_u8_init();
    for (i = 0; i < height; i += 4) {
      s = v64_sad_u8(s, v64_load_aligned(a + 0*astride), v64_load_aligned(b + 0*bstride));
      s = v64_sad_u8(s, v64_load_aligned(a + 1*astride), v64_load_aligned(b + 1*bstride));
      s = v64_sad_u8(s, v64_load_aligned(a + 2*astride), v64_load_aligned(b + 2*bstride));
      s = v64_sad_u8(s, v64_load_aligned(a + 3*astride), v64_load_aligned(b + 3*bstride));
      a += 4*astride;
      b += 4*bstride;
    }
    return v64_sad_u8_sum(s);
  } else {
    sad128_internal s = v128_sad_u8_init();
    for (i = 0; i < height; i++)
      for (j = 0; j < width; j += 16)
        s = v128_sad_u8(s, v128_load_aligned(a + i*astride + j), v128_load_aligned(b + i*bstride + j));
    return v128_sad_u8_sum(s);
  }
}

int ssd_calc_simd(uint8_t *a, uint8_t *b, int astride, int bstride, int size)
{
  int i, j;

  if (size == 8) {
    ssd64_internal s = v64_ssd_u8_init();
    s = v64_ssd_u8(s, v64_load_aligned(a + 0*astride), v64_load_aligned(b + 0*bstride));
    s = v64_ssd_u8(s, v64_load_aligned(a + 1*astride), v64_load_aligned(b + 1*bstride));
    s = v64_ssd_u8(s, v64_load_aligned(a + 2*astride), v64_load_aligned(b + 2*bstride));
    s = v64_ssd_u8(s, v64_load_aligned(a + 3*astride), v64_load_aligned(b + 3*bstride));
    s = v64_ssd_u8(s, v64_load_aligned(a + 4*astride), v64_load_aligned(b + 4*bstride));
    s = v64_ssd_u8(s, v64_load_aligned(a + 5*astride), v64_load_aligned(b + 5*bstride));
    s = v64_ssd_u8(s, v64_load_aligned(a + 6*astride), v64_load_aligned(b + 6*bstride));
    s = v64_ssd_u8(s, v64_load_aligned(a + 7*astride), v64_load_aligned(b + 7*bstride));
    return v64_ssd_u8_sum(s);
  } else {
    ssd128_internal s = v128_ssd_u8_init();
    for (i = 0; i < size; i++)
      for (j = 0; j < size; j += 16)
        s = v128_ssd_u8(s, v128_load_aligned(a + i*astride + j), v128_load_aligned(b + i*bstride + j));
    return v128_ssd_u8_sum(s);
  }
}
