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
#include "global.h"

int sad_calc_simd(uint8_t *a, uint8_t *b, int astride, int bstride, int width, int height)
{
  int i, j;

  if (width == 8) {
    sad64_internal s = v64_sad_u8_init();
    if ((intptr_t)b & 7)
      for (i = 0; i < height; i += 4) {
        s = v64_sad_u8(s, v64_load_aligned(a + 0*astride), v64_load_unaligned(b + 0*bstride));
        s = v64_sad_u8(s, v64_load_aligned(a + 1*astride), v64_load_unaligned(b + 1*bstride));
        s = v64_sad_u8(s, v64_load_aligned(a + 2*astride), v64_load_unaligned(b + 2*bstride));
        s = v64_sad_u8(s, v64_load_aligned(a + 3*astride), v64_load_unaligned(b + 3*bstride));
        a += 4*astride;
        b += 4*bstride;
      }
    else
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
    if ((intptr_t)b & 15)
      for (i = 0; i < height; i++)
        for (j = 0; j < width; j += 16)
          s = v128_sad_u8(s, v128_load_aligned(a + i*astride + j), v128_load_unaligned(b + i*bstride + j));
    else
      for (i = 0; i < height; i++)
        for (j = 0; j < width; j += 16)
          s = v128_sad_u8(s, v128_load_aligned(a + i*astride + j), v128_load_aligned(b + i*bstride + j));
    return v128_sad_u8_sum(s);
  }
}

unsigned int widesad_calc_simd(uint8_t *a, uint8_t *b, int astride, int bstride, int width, int height, int *x)
{
  // Calculate the 16x16 SAD for five positions x.xXx.x and return the best
  sad128_internal s0 = v128_sad_u8_init();
  sad128_internal s1 = v128_sad_u8_init();
  sad128_internal s2 = v128_sad_u8_init();
  sad128_internal s3 = v128_sad_u8_init();
  sad128_internal s4 = v128_sad_u8_init();
  for (int c = 0; c < 16; c++) {
    v128 aa = v128_load_aligned(a);
    v128 ba = v128_load_unaligned(b-3);
    v128 bb = v128_load_unaligned(b+13);

    s0 = v128_sad_u8(s0, aa, ba);
    s1 = v128_sad_u8(s1, aa, v128_align(bb, ba, 2));
    s2 = v128_sad_u8(s2, aa, v128_align(bb, ba, 3));
    s3 = v128_sad_u8(s3, aa, v128_align(bb, ba, 4));
    s4 = v128_sad_u8(s4, aa, v128_align(bb, ba, 6));

    b += bstride;
    a += astride;
  }

  unsigned int r = min(min(min((v128_sad_u8_sum(s0) << 3) | 0, (v128_sad_u8_sum(s1) << 3) | 2),
                           min((v128_sad_u8_sum(s2) << 3) | 3, (v128_sad_u8_sum(s3) << 3) | 4)), (v128_sad_u8_sum(s4) << 3) | 6);
  *x = (int)(r & 7) - 3;
  return r >> 3;
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

void detect_clpf_simd(const uint8_t *rec,const uint8_t *org,int x0, int y0, int width, int height, int so,int stride, int *sum0, int *sum1, unsigned int strength)
{
  int left = (x0 & ~(MAX_SB_SIZE-1)) - x0;
  int top = (y0 & ~(MAX_SB_SIZE-1)) - y0;
  int right = min(width-1, left + MAX_SB_SIZE-1);
  int bottom = min(height-1, top + MAX_SB_SIZE-1);
  v64 c2 = v64_dup_8(2);
  v64 c128 = v64_dup_8(128);
  v64 s1 = left ? v64_from_64(0x0706050403020100LL) : v64_from_64(0x0605040302010000LL);
  v64 s2 = right == 7 ? v64_from_64(0x0707060504030201LL) : v64_from_64(0x0706050403020100LL);
  v64 sp = v64_dup_8(strength);
  v64 sm = v64_dup_8(-strength);
  ssd64_internal ssd0 = v64_ssd_u8_init();
  ssd64_internal ssd1 = v64_ssd_u8_init();

  rec += x0 + y0*stride;
  org += x0 + y0*so;

  for (int y = 0; y < 8; y++) {
    v64 o = v64_load_aligned(org);
    v64 q = v64_load_aligned(rec);
    v64 x = v64_add_8(c128, q);
    v64 a = v64_add_8(c128, v64_load_aligned(rec - (y!=top)*stride));
    v64 b = v64_add_8(c128, v64_shuffle_8(v64_load_unaligned(rec - !!left), s1));
    v64 c = v64_add_8(c128, v64_shuffle_8(v64_load_unaligned(rec + (right != 7)), s2));
    v64 d = v64_add_8(c128, v64_load_aligned(rec + (y!=bottom)*stride));
    v64 delta = v64_add_8(v64_add_8(v64_max_s8(v64_min_s8(v64_ssub_s8(a, x), sp), sm),
				    v64_max_s8(v64_min_s8(v64_ssub_s8(b, x), sp), sm)),
			  v64_add_8(v64_max_s8(v64_min_s8(v64_ssub_s8(c, x), sp), sm),
				    v64_max_s8(v64_min_s8(v64_ssub_s8(d, x), sp), sm)));
    ssd0 = v64_ssd_u8(ssd0, o, q);
    ssd1 = v64_ssd_u8(ssd1, o, v64_add_8(q, v64_shr_s8(v64_add_8(c2, v64_add_8(delta, v64_cmplt_s8(delta, v64_zero()))), 2)));
    rec += stride;
    org += so;
  }
  *sum0 += v64_ssd_u8_sum(ssd0);
  *sum1 += v64_ssd_u8_sum(ssd1);
}

/* Return the best approximated half-pel position around the centre */
unsigned int sad_calc_fasthalf_simd(const uint8_t *a, const uint8_t *b, int as, int bs, int width, int height, int *x, int *y, unsigned int *maxsad)
{
  unsigned int sad_tl, sad_tr, sad_br, sad_bl;
  unsigned int sad_top, sad_right, sad_down, sad_left;

  if (width == 8) {
    sad64_internal top = v64_sad_u8_init();
    sad64_internal right = v64_sad_u8_init();
    sad64_internal down = v64_sad_u8_init();
    sad64_internal left = v64_sad_u8_init();
    sad64_internal tl = v64_sad_u8_init();
    sad64_internal tr = v64_sad_u8_init();
    sad64_internal br = v64_sad_u8_init();
    sad64_internal bl = v64_sad_u8_init();

    for (int i = 0; i < height; i++) {
      v64 o = v64_load_aligned(a + i*as);

      v64 t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15;
      t7 = v64_load_unaligned(b);
      t8 = v64_load_unaligned(b + bs);
      t9 = v64_load_unaligned(b - bs);
      t10 = v64_load_unaligned(b + bs + 1);
      t11 = v64_load_unaligned(b + bs - 1);
      t12 = v64_load_unaligned(b - bs + 1);
      t13 = v64_load_unaligned(b - bs - 1);
      t14 = v64_load_unaligned(b - 1);
      t15 = v64_load_unaligned(b + 1);

      t2 = v64_avg_u8(t14, t7);
      left = v64_sad_u8(left, o, t2);
      t3 = v64_avg_u8(v64_load_unaligned(b - 2*bs), t8);
      t4 = v64_avg_u8(v64_load_unaligned(b - 2), t15);
      t1 = v64_rdavg_u8(v64_rdavg_u8(v64_avg_u8(v64_load_unaligned(b - 2*bs - 1), t11), t3),
                        v64_rdavg_u8(v64_avg_u8(v64_load_unaligned(b - bs - 2), t12), t4));
      tl = v64_sad_u8(tl, o, v64_rdavg_u8(t1, v64_rdavg_u8(v64_avg_u8(t13, t9), t2)));

      t5 = v64_avg_u8(t7, t15);
      right = v64_sad_u8(right, o, t5);
      t6 = v64_avg_u8(t14, v64_load_unaligned(b + 2));
      t1 = v64_rdavg_u8(v64_rdavg_u8(t3, v64_avg_u8(v64_load_unaligned(b - 2*bs + 1), t10)),
                        v64_rdavg_u8(t6, v64_avg_u8(t13, v64_load_unaligned(b - bs + 2))));
      tr = v64_sad_u8(tr, o, v64_rdavg_u8(t1, v64_rdavg_u8(v64_avg_u8(t9, t12), t5)));

      t3 = v64_avg_u8(t9, v64_load_unaligned(b + 2*bs));
      t1 = v64_rdavg_u8(v64_rdavg_u8(t3, v64_avg_u8(t13, v64_load_unaligned (b + 2*bs - 1))),
                        v64_rdavg_u8(t4, v64_avg_u8(v64_load_unaligned(b + bs - 2), t10)));
      bl = v64_sad_u8(bl, o, v64_rdavg_u8(t1, v64_rdavg_u8(v64_avg_u8(t11, t8), t2)));

      t1 = v64_rdavg_u8(v64_rdavg_u8(t3, v64_avg_u8(t12, v64_load_unaligned(b + 2*bs + 1))),
                        v64_rdavg_u8(t6, v64_avg_u8(t11, v64_load_unaligned(b + bs + 2))));
      br = v64_sad_u8(br, o, v64_rdavg_u8(t1, v64_rdavg_u8(t5, v64_avg_u8(t8, t10))));

      down = v64_sad_u8(down, o, v64_avg_u8(t7, t8));
      top = v64_sad_u8(top, o, v64_avg_u8(t7, t9));

      b += bs;
    }
    sad_top = v64_sad_u8_sum(top);
    sad_right = v64_sad_u8_sum(right);
    sad_down = v64_sad_u8_sum(down);
    sad_left = v64_sad_u8_sum(left);

    sad_tl = v64_sad_u8_sum(tl);
    sad_tr = v64_sad_u8_sum(tr);
    sad_bl = v64_sad_u8_sum(bl);
    sad_br = v64_sad_u8_sum(br);

  } else {
    sad128_internal top = v128_sad_u8_init();
    sad128_internal right = v128_sad_u8_init();
    sad128_internal down = v128_sad_u8_init();
    sad128_internal left = v128_sad_u8_init();
    sad128_internal tl = v128_sad_u8_init();
    sad128_internal tr = v128_sad_u8_init();
    sad128_internal br = v128_sad_u8_init();
    sad128_internal bl = v128_sad_u8_init();

    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j += 16) {
        v128 o = v128_load_aligned(a + i*as + j);

        v128 t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15;
        t7 = v128_load_unaligned(b);
        t8 = v128_load_unaligned(b + bs);
        t9 = v128_load_unaligned(b - bs);
        t10 = v128_load_unaligned(b + bs + 1);
        t11 = v128_load_unaligned(b + bs - 1);
        t12 = v128_load_unaligned(b - bs + 1);
        t13 = v128_load_unaligned(b - bs - 1);
        t14 = v128_load_unaligned(b - 1);
        t15 = v128_load_unaligned(b + 1);

        t2 = v128_avg_u8(t14, t7);
        left = v128_sad_u8(left, o, t2);
        t3 = v128_avg_u8(v128_load_unaligned(b - 2*bs), t8);
        t4 = v128_avg_u8(v128_load_unaligned(b - 2), t15);
        t1 = v128_rdavg_u8(v128_rdavg_u8(v128_avg_u8(v128_load_unaligned(b - 2*bs - 1), t11), t3),
                          v128_rdavg_u8(v128_avg_u8(v128_load_unaligned(b - bs - 2), t12), t4));
        tl = v128_sad_u8(tl, o, v128_rdavg_u8(t1, v128_rdavg_u8(v128_avg_u8(t13, t9), t2)));

        t5 = v128_avg_u8(t7, t15);
        right = v128_sad_u8(right, o, t5);
        t6 = v128_avg_u8(t14, v128_load_unaligned(b + 2));
        t1 = v128_rdavg_u8(v128_rdavg_u8(t3, v128_avg_u8(v128_load_unaligned(b - 2*bs + 1), t10)),
                          v128_rdavg_u8(t6, v128_avg_u8(t13, v128_load_unaligned(b - bs + 2))));
        tr = v128_sad_u8(tr, o, v128_rdavg_u8(t1, v128_rdavg_u8(v128_avg_u8(t9, t12), t5)));

        t3 = v128_avg_u8(t9, v128_load_unaligned(b + 2*bs));
        t1 = v128_rdavg_u8(v128_rdavg_u8(t3, v128_avg_u8(t13, v128_load_unaligned (b + 2*bs - 1))),
                          v128_rdavg_u8(t4, v128_avg_u8(v128_load_unaligned(b + bs - 2), t10)));
        bl = v128_sad_u8(bl, o, v128_rdavg_u8(t1, v128_rdavg_u8(v128_avg_u8(t11, t8), t2)));

        t1 = v128_rdavg_u8(v128_rdavg_u8(t3, v128_avg_u8(t12, v128_load_unaligned(b + 2*bs + 1))),
                          v128_rdavg_u8(t6, v128_avg_u8(t11, v128_load_unaligned(b + bs + 2))));
        br = v128_sad_u8(br, o, v128_rdavg_u8(t1, v128_rdavg_u8(t5, v128_avg_u8(t8, t10))));

        down = v128_sad_u8(down, o, v128_avg_u8(t7, t8));
        top = v128_sad_u8(top, o, v128_avg_u8(t7, t9));

        b += 16;
      }
      b += bs - width;
    }

    sad_top = v128_sad_u8_sum(top);
    sad_right = v128_sad_u8_sum(right);
    sad_down = v128_sad_u8_sum(down);
    sad_left = v128_sad_u8_sum(left);

    sad_tl = v128_sad_u8_sum(tl);
    sad_tr = v128_sad_u8_sum(tr);
    sad_bl = v128_sad_u8_sum(bl);
    sad_br = v128_sad_u8_sum(br);
  }

  int bestx = 0, besty = -2;

  if (sad_down < sad_top) {
    besty = 2;
    sad_top = sad_down;
  }

  if (sad_right < sad_top) {
    bestx = 2;
    besty = 0;
    sad_top = sad_right;
  }

  if (sad_left < sad_top) {
    bestx = -2;
    besty = 0;
    sad_top = sad_left;
  }

  if (sad_tl < sad_top) {
    bestx = -2;
    besty = -2;
    sad_top = sad_tl;
  }

  if (sad_tr < sad_top) {
    bestx = 2;
    besty = -2;
    sad_top = sad_tr;
  }

  if (sad_br < sad_top) {
    bestx = 2;
    besty = 2;
    sad_top = sad_br;
  }

  if (sad_bl < sad_top) {
    bestx = -2;
    besty = 2;
    sad_top = sad_bl;
  }

  *x = bestx;
  *y = besty;
  return sad_top;
}


/* Return the best approximated quarter-pel position around the centre */
unsigned int sad_calc_fastquarter_simd(const uint8_t *po, const uint8_t *r, int os, int rs, int width, int height, int *x, int *y)
{
  unsigned int sad_tl, sad_tr, sad_br, sad_bl;
  unsigned int sad_top, sad_right, sad_down, sad_left;
  int bestx = 0, besty = -1;

  sad64_internal top = v64_sad_u8_init();
  sad64_internal right = v64_sad_u8_init();
  sad64_internal down = v64_sad_u8_init();
  sad64_internal left = v64_sad_u8_init();
  sad64_internal tl = v64_sad_u8_init();
  sad64_internal tr = v64_sad_u8_init();
  sad64_internal br = v64_sad_u8_init();
  sad64_internal bl = v64_sad_u8_init();

  if (width == 8) {
    if (*x & *y) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 8) {

          v64 o = v64_load_aligned(po + j);
          v64 a = v64_load_unaligned(r + j);
          v64 d = v64_load_unaligned(r + j + 1);
          v64 e = v64_load_unaligned(r + j + rs + 1);
          v64 f = v64_load_unaligned(r + j + rs);
          v64 ad = v64_avg_u8(a, d);
          v64 de = v64_avg_u8(d, e);
          v64 af = v64_avg_u8(a, f);
          v64 fe = v64_avg_u8(f, e);

          tl =    v64_sad_u8(tl, o, v64_rdavg_u8(ad, af));
          top =   v64_sad_u8(top, o, v64_rdavg_u8(de,  a));
          tr =    v64_sad_u8(tr, o, v64_rdavg_u8(ad, de));
          left =  v64_sad_u8(left, o, v64_rdavg_u8(ad,  f));
          right = v64_sad_u8(right, o, v64_rdavg_u8(ad,  e));
          bl =    v64_sad_u8(bl, o, v64_rdavg_u8(af, fe));
          down =  v64_sad_u8(down, o, v64_rdavg_u8(de,  f));
          br =    v64_sad_u8(br, o, v64_rdavg_u8(de, fe));
        }
        r += rs;
        po += os;
      }
    } else if (*x) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 8) {

          v64 o = v64_load_aligned(po + j);
          v64 a = v64_load_unaligned(r + j);
          v64 b = v64_load_unaligned(r + j - rs);
          v64 c = v64_load_unaligned(r + j - rs + 1);
          v64 d = v64_load_unaligned(r + j + 1);
          v64 e = v64_load_unaligned(r + j + rs + 1);
          v64 f = v64_load_unaligned(r + j + rs);
          v64 ad = v64_avg_u8(a, d);
          v64 de = v64_avg_u8(d, e);
          v64 dc = v64_avg_u8(d, c);
          v64 af = v64_avg_u8(a, f);
          v64 ab = v64_avg_u8(a, b);

          tl =    v64_sad_u8(tl, o, v64_rdavg_u8(ad, ab));
          top =   v64_sad_u8(top, o, v64_rdavg_u8(dc,  a));
          tr =    v64_sad_u8(tr, o, v64_rdavg_u8(ad, dc));
          left =  v64_sad_u8(left, o, v64_rdavg_u8(ad,  a));
          right = v64_sad_u8(right, o, v64_rdavg_u8(ad,  d));
          bl =    v64_sad_u8(bl, o, v64_rdavg_u8(ad, af));
          down =  v64_sad_u8(down, o, v64_rdavg_u8(af,  d));
          br =    v64_sad_u8(br, o, v64_rdavg_u8(ad, de));
        }
        r += rs;
        po += os;
      }
    } else if (*y) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 8) {

          v64 o = v64_load_aligned(po + j);
          v64 a = v64_load_unaligned(r + j);
          v64 d = v64_load_unaligned(r + j + 1);
          v64 e = v64_load_unaligned(r + j + rs + 1);
          v64 f = v64_load_unaligned(r + j + rs);
          v64 g = v64_load_unaligned(r + j + rs - 1);
          v64 h = v64_load_unaligned(r + j - 1);
          v64 ad = v64_avg_u8(a, d);
          v64 af = v64_avg_u8(a, f);
          v64 fe = v64_avg_u8(f, e);
          v64 ah = v64_avg_u8(a, h);
          v64 gf = v64_avg_u8(g, f);

          tl =    v64_sad_u8(tl, o, v64_rdavg_u8(ah, af));
          top =   v64_sad_u8(top, o, v64_rdavg_u8(af,  a));
          tr =    v64_sad_u8(tr, o, v64_rdavg_u8(ad, af));
          left =  v64_sad_u8(left, o, v64_rdavg_u8(gf,  a));
          right = v64_sad_u8(right, o, v64_rdavg_u8(ad,  f));
          bl =    v64_sad_u8(bl, o, v64_rdavg_u8(af, gf));
          down =  v64_sad_u8(down, o, v64_rdavg_u8(af,  f));
          br =    v64_sad_u8(br, o, v64_rdavg_u8(af, fe));
        }
        r += rs;
        po += os;
      }
    } else {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 8) {

          v64 o = v64_load_aligned(po + j);
          v64 a = v64_load_unaligned(r + j);
          v64 b = v64_load_unaligned(r + j - rs);
          v64 d = v64_load_unaligned(r + j + 1);
          v64 f = v64_load_unaligned(r + j + rs);
          v64 h = v64_load_unaligned(r + j - 1);
          v64 ad = v64_avg_u8(a, d);
          v64 af = v64_avg_u8(a, f);
          v64 ah = v64_avg_u8(a, h);
          v64 ab = v64_avg_u8(a, b);

          tl =    v64_sad_u8(tl, o, v64_rdavg_u8(ah, ab));
          top =   v64_sad_u8(top, o, v64_rdavg_u8(ab,  a));
          tr =    v64_sad_u8(tr, o, v64_rdavg_u8(ad, ab));
          left =  v64_sad_u8(left, o, v64_rdavg_u8(ah,  a));
          right = v64_sad_u8(right, o, v64_rdavg_u8(ad,  a));
          bl =    v64_sad_u8(bl, o, v64_rdavg_u8(ah, af));
          down =  v64_sad_u8(down, o, v64_rdavg_u8(af,  a));
          br =    v64_sad_u8(br, o, v64_rdavg_u8(af, ad));
        }
        r += rs;
        po += os;
      }
    }

    sad_top = v64_sad_u8_sum(top);
    sad_right = v64_sad_u8_sum(right);
    sad_down = v64_sad_u8_sum(down);
    sad_left = v64_sad_u8_sum(left);

    sad_tl = v64_sad_u8_sum(tl);
    sad_tr = v64_sad_u8_sum(tr);
    sad_bl = v64_sad_u8_sum(bl);
    sad_br = v64_sad_u8_sum(br);
  } else {
    if (*x & *y) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 16) {

          v128 o = v128_load_aligned(po + j);
          v128 a = v128_load_unaligned(r + j);
          v128 d = v128_load_unaligned(r + j + 1);
          v128 e = v128_load_unaligned(r + j + rs + 1);
          v128 f = v128_load_unaligned(r + j + rs);
          v128 ad = v128_avg_u8(a, d);
          v128 de = v128_avg_u8(d, e);
          v128 af = v128_avg_u8(a, f);
          v128 fe = v128_avg_u8(f, e);

          tl =    v128_sad_u8(tl, o, v128_rdavg_u8(ad, af));
          top =   v128_sad_u8(top, o, v128_rdavg_u8(de,  a));
          tr =    v128_sad_u8(tr, o, v128_rdavg_u8(ad, de));
          left =  v128_sad_u8(left, o, v128_rdavg_u8(ad,  f));
          right = v128_sad_u8(right, o, v128_rdavg_u8(ad,  e));
          bl =    v128_sad_u8(bl, o, v128_rdavg_u8(af, fe));
          down =  v128_sad_u8(down, o, v128_rdavg_u8(de,  f));
          br =    v128_sad_u8(br, o, v128_rdavg_u8(de, fe));
        }
        r += rs;
        po += os;
      }
    } else if (*x) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 16) {

          v128 o = v128_load_aligned(po + j);
          v128 a = v128_load_unaligned(r + j);
          v128 b = v128_load_unaligned(r + j - rs);
          v128 c = v128_load_unaligned(r + j - rs + 1);
          v128 d = v128_load_unaligned(r + j + 1);
          v128 e = v128_load_unaligned(r + j + rs + 1);
          v128 f = v128_load_unaligned(r + j + rs);
          v128 ad = v128_avg_u8(a, d);
          v128 de = v128_avg_u8(d, e);
          v128 dc = v128_avg_u8(d, c);
          v128 af = v128_avg_u8(a, f);
          v128 ab = v128_avg_u8(a, b);

          tl =    v128_sad_u8(tl, o, v128_rdavg_u8(ad, ab));
          top =   v128_sad_u8(top, o, v128_rdavg_u8(dc,  a));
          tr =    v128_sad_u8(tr, o, v128_rdavg_u8(ad, dc));
          left =  v128_sad_u8(left, o, v128_rdavg_u8(ad,  a));
          right = v128_sad_u8(right, o, v128_rdavg_u8(ad,  d));
          bl =    v128_sad_u8(bl, o, v128_rdavg_u8(ad, af));
          down =  v128_sad_u8(down, o, v128_rdavg_u8(af,  d));
          br =    v128_sad_u8(br, o, v128_rdavg_u8(ad, de));
        }
        r += rs;
        po += os;
      }
    } else if (*y) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 16) {

          v128 o = v128_load_aligned(po + j);
          v128 a = v128_load_unaligned(r + j);
          v128 d = v128_load_unaligned(r + j + 1);
          v128 e = v128_load_unaligned(r + j + rs + 1);
          v128 f = v128_load_unaligned(r + j + rs);
          v128 g = v128_load_unaligned(r + j + rs - 1);
          v128 h = v128_load_unaligned(r + j - 1);
          v128 ad = v128_avg_u8(a, d);
          v128 af = v128_avg_u8(a, f);
          v128 fe = v128_avg_u8(f, e);
          v128 ah = v128_avg_u8(a, h);
          v128 gf = v128_avg_u8(g, f);

          tl =    v128_sad_u8(tl, o, v128_rdavg_u8(ah, af));
          top =   v128_sad_u8(top, o, v128_rdavg_u8(af,  a));
          tr =    v128_sad_u8(tr, o, v128_rdavg_u8(ad, af));
          left =  v128_sad_u8(left, o, v128_rdavg_u8(gf,  a));
          right = v128_sad_u8(right, o, v128_rdavg_u8(ad,  f));
          bl =    v128_sad_u8(bl, o, v128_rdavg_u8(af, gf));
          down =  v128_sad_u8(down, o, v128_rdavg_u8(af,  f));
          br =    v128_sad_u8(br, o, v128_rdavg_u8(af, fe));
        }
        r += rs;
        po += os;
      }
    } else {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 16) {

          v128 o = v128_load_aligned(po + j);
          v128 a = v128_load_unaligned(r + j);
          v128 b = v128_load_unaligned(r + j - rs);
          v128 d = v128_load_unaligned(r + j + 1);
          v128 f = v128_load_unaligned(r + j + rs);
          v128 h = v128_load_unaligned(r + j - 1);
          v128 ad = v128_avg_u8(a, d);
          v128 af = v128_avg_u8(a, f);
          v128 ah = v128_avg_u8(a, h);
          v128 ab = v128_avg_u8(a, b);

          tl =    v128_sad_u8(tl, o, v128_rdavg_u8(ah, ab));
          top =   v128_sad_u8(top, o, v128_rdavg_u8(ab,  a));
          tr =    v128_sad_u8(tr, o, v128_rdavg_u8(ad, ab));
          left =  v128_sad_u8(left, o, v128_rdavg_u8(ah,  a));
          right = v128_sad_u8(right, o, v128_rdavg_u8(ad,  a));
          bl =    v128_sad_u8(bl, o, v128_rdavg_u8(ah, af));
          down =  v128_sad_u8(down, o, v128_rdavg_u8(af,  a));
          br =    v128_sad_u8(br, o, v128_rdavg_u8(af, ad));
        }
        r += rs;
        po += os;
      }
    }

    sad_top = v128_sad_u8_sum(top);
    sad_right = v128_sad_u8_sum(right);
    sad_down = v128_sad_u8_sum(down);
    sad_left = v128_sad_u8_sum(left);

    sad_tl = v128_sad_u8_sum(tl);
    sad_tr = v128_sad_u8_sum(tr);
    sad_bl = v128_sad_u8_sum(bl);
    sad_br = v128_sad_u8_sum(br);
  }

  if (sad_tl < sad_top) {
    bestx = -1;
    sad_top = sad_tl;
  }
  if (sad_tr < sad_top) {
    bestx = 1;
    sad_top = sad_tr;
  }
  if (sad_left < sad_top) {
    bestx = -1;
    besty = 0;
    sad_top = sad_left;
  }
  if (sad_right < sad_top) {
    bestx = 1;
    besty = 0;
    sad_top = sad_right;
  }
  if (sad_bl < sad_top) {
    bestx = -1;
    besty = 1;
    sad_top = sad_bl;
  }
  if (sad_down < sad_top) {
    bestx = 0;
    besty = 1;
    sad_top = sad_down;
  }
  if (sad_br < sad_top) {
    bestx = 1;
    besty = 1;
    sad_top = sad_br;
  }

  *x = bestx;
  *y = besty;
  return sad_top;
}
