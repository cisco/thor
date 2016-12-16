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

// The high bitdepth version of this file is made from the low
// bitdepth version by running scripts/lbd_to_hbd.sh.

#include "simd.h"
#include "global.h"

int TEMPLATE(sad_calc_simd)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int width, int height)
{
  int i, j;

  if (width == 8) {
    sad128_internal_u16 s = v128_sad_u16_init();
#ifdef HBD
    if ((intptr_t)b & 15)
#else
    if ((intptr_t)b & 7)
#endif
      for (i = 0; i < height; i += 4) {
        s = v128_sad_u16(s, v128_load_aligned(a + 0*astride), v128_load_unaligned(b + 0*bstride));
        s = v128_sad_u16(s, v128_load_aligned(a + 1*astride), v128_load_unaligned(b + 1*bstride));
        s = v128_sad_u16(s, v128_load_aligned(a + 2*astride), v128_load_unaligned(b + 2*bstride));
        s = v128_sad_u16(s, v128_load_aligned(a + 3*astride), v128_load_unaligned(b + 3*bstride));
        a += 4*astride;
        b += 4*bstride;
      }
    else
      for (i = 0; i < height; i += 4) {
        s = v128_sad_u16(s, v128_load_aligned(a + 0*astride), v128_load_aligned(b + 0*bstride));
        s = v128_sad_u16(s, v128_load_aligned(a + 1*astride), v128_load_aligned(b + 1*bstride));
        s = v128_sad_u16(s, v128_load_aligned(a + 2*astride), v128_load_aligned(b + 2*bstride));
        s = v128_sad_u16(s, v128_load_aligned(a + 3*astride), v128_load_aligned(b + 3*bstride));
        a += 4*astride;
        b += 4*bstride;
      }
    return v128_sad_u16_sum(s);
  } else {
    sad256_internal_u16 s = v256_sad_u16_init();
#ifdef HBD
    if ((intptr_t)b & 31)
#else
    if ((intptr_t)b & 15)
#endif
      for (i = 0; i < height; i++)
        for (j = 0; j < width; j += 16)
          s = v256_sad_u16(s, v256_load_aligned(a + i*astride + j), v256_load_unaligned(b + i*bstride + j));
    else
      for (i = 0; i < height; i++)
        for (j = 0; j < width; j += 16)
          s = v256_sad_u16(s, v256_load_aligned(a + i*astride + j), v256_load_aligned(b + i*bstride + j));
    return v256_sad_u16_sum(s);
  }
}


unsigned int TEMPLATE(widesad_calc_simd)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int width, int height, int *x)
{
  // Calculate the 16x16 SAD for five positions x.xXx.x and return the best
  sad256_internal_u16 s0 = v256_sad_u16_init();
  sad256_internal_u16 s1 = v256_sad_u16_init();
  sad256_internal_u16 s2 = v256_sad_u16_init();
  sad256_internal_u16 s3 = v256_sad_u16_init();
  sad256_internal_u16 s4 = v256_sad_u16_init();
  for (int c = 0; c < 16; c++) {
    v256 aa = v256_load_aligned(a);
    v256 ba = v256_load_unaligned(b-3);
    v256 bb = v256_load_unaligned(b+13);

    s0 = v256_sad_u16(s0, aa, ba);
#ifdef HBD
    s1 = v256_sad_u16(s1, aa, v256_align(bb, ba, 2*2));
    s2 = v256_sad_u16(s2, aa, v256_align(bb, ba, 3*2));
    s3 = v256_sad_u16(s3, aa, v256_align(bb, ba, 4*2));
    s4 = v256_sad_u16(s4, aa, v256_align(bb, ba, 6*2));
#else
    s1 = v256_sad_u16(s1, aa, v256_align(bb, ba, 2));
    s2 = v256_sad_u16(s2, aa, v256_align(bb, ba, 3));
    s3 = v256_sad_u16(s3, aa, v256_align(bb, ba, 4));
    s4 = v256_sad_u16(s4, aa, v256_align(bb, ba, 6));
#endif
    b += bstride;
    a += astride;
  }

  unsigned int r = min(min(min((v256_sad_u16_sum(s0) << 3) | 0, (v256_sad_u16_sum(s1) << 3) | 2),
                           min((v256_sad_u16_sum(s2) << 3) | 3, (v256_sad_u16_sum(s3) << 3) | 4)), (v256_sad_u16_sum(s4) << 3) | 6);
  *x = (int)(r & 7) - 3;
  return r >> 3;
}

uint64_t TEMPLATE(ssd_calc_simd)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int size)
{
  int i, j;

  if (size == 8) {
    ssd128_internal_u16 s = v128_ssd_u16_init();
    s = v128_ssd_u16(s, v128_load_aligned(a + 0*astride), v128_load_aligned(b + 0*bstride));
    s = v128_ssd_u16(s, v128_load_aligned(a + 1*astride), v128_load_aligned(b + 1*bstride));
    s = v128_ssd_u16(s, v128_load_aligned(a + 2*astride), v128_load_aligned(b + 2*bstride));
    s = v128_ssd_u16(s, v128_load_aligned(a + 3*astride), v128_load_aligned(b + 3*bstride));
    s = v128_ssd_u16(s, v128_load_aligned(a + 4*astride), v128_load_aligned(b + 4*bstride));
    s = v128_ssd_u16(s, v128_load_aligned(a + 5*astride), v128_load_aligned(b + 5*bstride));
    s = v128_ssd_u16(s, v128_load_aligned(a + 6*astride), v128_load_aligned(b + 6*bstride));
    s = v128_ssd_u16(s, v128_load_aligned(a + 7*astride), v128_load_aligned(b + 7*bstride));
    return v128_ssd_u16_sum(s);
  } else {
    uint64_t ssd = 0;
    for (i = 0; i < size; i += 8) {
      ssd256_internal_u16 s = v256_ssd_u16_init();
      for (j = 0; j < size; j += 16) {
        s = v256_ssd_u16(s, v256_load_aligned(a + (i+0)*astride + j), v256_load_aligned(b + (i+0)*bstride + j));
        s = v256_ssd_u16(s, v256_load_aligned(a + (i+1)*astride + j), v256_load_aligned(b + (i+1)*bstride + j));
        s = v256_ssd_u16(s, v256_load_aligned(a + (i+2)*astride + j), v256_load_aligned(b + (i+2)*bstride + j));
        s = v256_ssd_u16(s, v256_load_aligned(a + (i+3)*astride + j), v256_load_aligned(b + (i+3)*bstride + j));
        s = v256_ssd_u16(s, v256_load_aligned(a + (i+4)*astride + j), v256_load_aligned(b + (i+4)*bstride + j));
        s = v256_ssd_u16(s, v256_load_aligned(a + (i+5)*astride + j), v256_load_aligned(b + (i+5)*bstride + j));
        s = v256_ssd_u16(s, v256_load_aligned(a + (i+6)*astride + j), v256_load_aligned(b + (i+6)*bstride + j));
        s = v256_ssd_u16(s, v256_load_aligned(a + (i+7)*astride + j), v256_load_aligned(b + (i+7)*bstride + j));
      }
      ssd += v256_ssd_u16_sum(s);
    }
    return ssd;
  }
}

#ifdef HBD
static ALIGN(32) uint64_t bshuff[] = { 0x0302010001000100LL, 0x0b0a090807060504LL,
                                       0x0302010001000100LL, 0x0b0a090807060504LL };
static ALIGN(32) uint64_t cshuff[] = { 0x0504030201000100LL, 0x0d0c0b0a09080706LL,
                                       0x0504030201000100LL, 0x0d0c0b0a09080706LL };
static ALIGN(32) uint64_t dshuff[] = { 0x0908070605040302LL, 0x0f0e0f0e0d0c0b0aLL,
                                       0x0908070605040302LL, 0x0f0e0f0e0d0c0b0aLL };
static ALIGN(32) uint64_t eshuff[] = { 0x0b0a090807060504LL, 0x0f0e0f0e0f0e0d0cLL,
                                       0x0b0a090807060504LL, 0x0f0e0f0e0f0e0d0cLL };
#else
static ALIGN(16) uint64_t bshuff[] = { 0x0504030201000000LL, 0x0d0c0b0a09080808LL };
static ALIGN(16) uint64_t cshuff[] = { 0x0605040302010000LL, 0x0e0d0c0b0a090808LL };
static ALIGN(16) uint64_t dshuff[] = { 0x0707060503030201LL, 0x0f0f0e0d0b0b0a09LL };
static ALIGN(16) uint64_t eshuff[] = { 0x0707070603030302LL, 0x0f0f0f0e0b0b0b0aLL };
#endif

SIMD_INLINE void calc_diff(v256 o, v256 *a, v256 *b, v256 *c, v256 *d, v256 *e, v256 *f) {
#ifdef HBD
  *a = v256_sub_16(*a, o);
  *b = v256_sub_16(*b, o);
  *c = v256_sub_16(*c, o);
  *d = v256_sub_16(*d, o);
  *e = v256_sub_16(*e, o);
  *f = v256_sub_16(*f, o);
#else
  // The difference will be 9 bit, offset by 128 so we can use saturated
  // sub to avoid going to 16 bit temporarily before "strength" clipping.
  v256 c128 = v256_dup_16(128);
  v256 x = v256_add_16(c128, o);
  *a = v256_ssub_s16(v256_add_16(c128, *a), x);
  *b = v256_ssub_s16(v256_add_16(c128, *b), x);
  *c = v256_ssub_s16(v256_add_16(c128, *c), x);
  *d = v256_ssub_s16(v256_add_16(c128, *d), x);
  *e = v256_ssub_s16(v256_add_16(c128, *e), x);
  *f = v256_ssub_s16(v256_add_16(c128, *f), x);
#endif
}

SIMD_INLINE v256 delta_kernel(v256 o, v256 a, v256 b, v256 c, v256 d, v256 e, v256 f, v256 sp, v256 sm) {
  v256 tmp = v256_add_16(v256_max_s16(v256_min_s16(c, sp), sm),
                        v256_max_s16(v256_min_s16(d, sp), sm));
  v256 delta = v256_add_16(v256_add_16(v256_shl_16(v256_add_16(v256_max_s16(v256_min_s16(a, sp), sm),
                                                           v256_max_s16(v256_min_s16(f, sp), sm)), 2),
                                     v256_add_16(v256_max_s16(v256_min_s16(b, sp), sm),
                                                v256_max_s16(v256_min_s16(e, sp), sm))),
                          v256_add_16(v256_add_16(tmp, tmp), tmp));
  return v256_add_16(o, v256_shr_s16(v256_add_16(v256_dup_16(8), v256_add_16(delta, v256_cmplt_s16(delta, v256_zero()))), 4));
}

SIMD_INLINE v256 calc_delta(v256 o, v256 a, v256 b, v256 c, v256 d, v256 e, v256 f, v256 sp, v256 sm) {
  calc_diff(o, &a, &b, &c, &d, &e, &f);
  return delta_kernel(o, a, b, c, d, e, f, sp, sm);
}

SIMD_INLINE void clip_sides(v256 *b, v256 *c, v256 *d, v256 *e, int left, int right) {
  if (!left) {  // Left clipping
    *b = v256_pshuffle_8(*b, v256_load_aligned(bshuff));
    *c = v256_pshuffle_8(*c, v256_load_aligned(cshuff));
  }
  if (!right) {  // Right clipping
    *d = v256_pshuffle_8(*d, v256_load_aligned(dshuff));
    *e = v256_pshuffle_8(*e, v256_load_aligned(eshuff));
  }
}

SIMD_INLINE void read_two_lines(const SAMPLE *rec, const SAMPLE *org, int stride, int x0, int y0, int bottom, int right, int y, v256 *o, v256 *r, v256 *a, v256 *b, v256 *c, v256 *d, v256 *e, v256 *f) {
  const v128 k1 = v128_load_aligned(org);
  const v128 k2 = v128_load_aligned(org + stride);
  const v128 l1 = v128_load_aligned(rec);
  const v128 l2 = v128_load_aligned(rec + stride);
  *o = v256_from_v128(k1, k2);
  *r = v256_from_v128(l1, l2);
  *a = v256_from_v128(v128_load_aligned(rec - (y != -y0) * stride), l1);
  *f = v256_from_v128(l2, v128_load_aligned(rec + ((y != bottom) + 1) * stride));
  *b = v256_from_v128(v128_load_unaligned(rec - 2 * !!x0), v128_load_unaligned(rec - 2 * !!x0 + stride));
  *c = v256_from_v128(v128_load_unaligned(rec - !!x0), v128_load_unaligned(rec - !!x0 + stride));
  *d = v256_from_v128(v128_load_unaligned(rec + !!right), v128_load_unaligned(rec + !!right + stride));
  *e = v256_from_v128(v128_load_unaligned(rec + 2 * !!right), v128_load_unaligned(rec + 2 * !!right + stride));
  clip_sides(b, c, d, e, x0, right);
}

void TEMPLATE(detect_clpf_simd)(const SAMPLE *rec,const SAMPLE *org,int x0, int y0, int width, int height, int so,int stride, int *sum0, int *sum1, unsigned int strength, unsigned int shift, unsigned int size)
{
  const v256 sp = v256_dup_16(strength);
  const v256 sm = v256_dup_16(-(int)strength);
  const int right = width - 8 - x0;
  const int bottom = height - 2 - y0;
  ssd256_internal_u16 ssd0 = v256_ssd_u16_init();
  ssd256_internal_u16 ssd1 = v256_ssd_u16_init();
  int y;

  rec += x0 + y0 * stride;
  org += x0 + y0 * stride;

  for (y = 0; y < 8; y += 2) {
    v256 a, b, c, d, e, f, o, r;
    read_two_lines(rec, org, stride, x0, y0, bottom, right, y, &o, &r, &a, &b, &c, &d, &e, &f);
    ssd0 = v256_ssd_u16(ssd0, o, r);
    ssd1 = v256_ssd_u16(ssd1, o, calc_delta(r, a, b, c, d, e, f, sp, sm));
    rec += stride * 2;
    org += stride * 2;
  }
  *sum0 += v256_ssd_u16_sum(ssd0) >> (shift*2);
  *sum1 += v256_ssd_u16_sum(ssd1) >> (shift*2);
}

// Test multiple filter strengths at once.
SIMD_INLINE void calc_delta_multi(v256 r, v256 o, v256 a, v256 b, v256 c,
                                  v256 d, v256 e, v256 f, ssd256_internal_u16 *ssd1,
                                  ssd256_internal_u16 *ssd2, ssd256_internal_u16 *ssd3, unsigned int shift) {
  calc_diff(r, &a, &b, &c, &d, &e, &f);
  *ssd1 = v256_ssd_u16(*ssd1, o, delta_kernel(r, a, b, c, d, e, f, v256_dup_16(1 << shift), v256_dup_16(-1 << shift)));
  *ssd2 = v256_ssd_u16(*ssd2, o, delta_kernel(r, a, b, c, d, e, f, v256_dup_16(2 << shift), v256_dup_16(-2 << shift)));
  *ssd3 = v256_ssd_u16(*ssd3, o, delta_kernel(r, a, b, c, d, e, f, v256_dup_16(4 << shift), v256_dup_16(-4 << shift)));
}

void TEMPLATE(detect_multi_clpf_simd)(const SAMPLE *rec,const SAMPLE *org,int x0, int y0, int width, int height, int so,int stride, int *sum, unsigned int shift, unsigned int size)
{
  const int bottom = height - 2 - y0;
  const int right = width - 8 - x0;
  ssd256_internal_u16 ssd0 = v256_ssd_u16_init();
  ssd256_internal_u16 ssd1 = v256_ssd_u16_init();
  ssd256_internal_u16 ssd2 = v256_ssd_u16_init();
  ssd256_internal_u16 ssd3 = v256_ssd_u16_init();
  int y;

  rec += x0 + y0 * stride;
  org += x0 + y0 * stride;

  for (y = 0; y < 8; y += 2) {
    v256 a, b, c, d, e, f, o, r;
    read_two_lines(rec, org, stride, x0, y0, bottom, right, y, &o, &r, &a, &b, &c, &d, &e, &f);
    ssd0 = v256_ssd_u16(ssd0, o, r);
    calc_delta_multi(r, o, a, b, c, d, e, f, &ssd1, &ssd2, &ssd3, shift);
    rec += 2 * stride;
    org += 2 * stride;
  }
  sum[0] += v256_ssd_u16_sum(ssd0) >> (shift*2);
  sum[1] += v256_ssd_u16_sum(ssd1) >> (shift*2);
  sum[2] += v256_ssd_u16_sum(ssd2) >> (shift*2);
  sum[3] += v256_ssd_u16_sum(ssd3) >> (shift*2);
}

/* Return the best approximated half-pel position around the centre */
unsigned int TEMPLATE(sad_calc_fasthalf_simd)(const SAMPLE *a, const SAMPLE *b, int as, int bs, int width, int height, int *x, int *y, unsigned int *maxsad)
{
  unsigned int sad_tl, sad_tr, sad_br, sad_bl;
  unsigned int sad_top, sad_right, sad_down, sad_left;

  if (width == 8) {
    sad128_internal_u16 top = v128_sad_u16_init();
    sad128_internal_u16 right = v128_sad_u16_init();
    sad128_internal_u16 down = v128_sad_u16_init();
    sad128_internal_u16 left = v128_sad_u16_init();
    sad128_internal_u16 tl = v128_sad_u16_init();
    sad128_internal_u16 tr = v128_sad_u16_init();
    sad128_internal_u16 br = v128_sad_u16_init();
    sad128_internal_u16 bl = v128_sad_u16_init();

    for (int i = 0; i < height; i++) {
      v128 o = v128_load_aligned(a + i*as);

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

      t2 = v128_avg_u16(t14, t7);
      left = v128_sad_u16(left, o, t2);
      t3 = v128_avg_u16(v128_load_unaligned(b - 2*bs), t8);
      t4 = v128_avg_u16(v128_load_unaligned(b - 2), t15);
      t1 = v128_rdavg_u16(v128_rdavg_u16(v128_avg_u16(v128_load_unaligned(b - 2*bs - 1), t11), t3),
                        v128_rdavg_u16(v128_avg_u16(v128_load_unaligned(b - bs - 2), t12), t4));
      tl = v128_sad_u16(tl, o, v128_rdavg_u16(t1, v128_rdavg_u16(v128_avg_u16(t13, t9), t2)));

      t5 = v128_avg_u16(t7, t15);
      right = v128_sad_u16(right, o, t5);
      t6 = v128_avg_u16(t14, v128_load_unaligned(b + 2));
      t1 = v128_rdavg_u16(v128_rdavg_u16(t3, v128_avg_u16(v128_load_unaligned(b - 2*bs + 1), t10)),
                        v128_rdavg_u16(t6, v128_avg_u16(t13, v128_load_unaligned(b - bs + 2))));
      tr = v128_sad_u16(tr, o, v128_rdavg_u16(t1, v128_rdavg_u16(v128_avg_u16(t9, t12), t5)));

      t3 = v128_avg_u16(t9, v128_load_unaligned(b + 2*bs));
      t1 = v128_rdavg_u16(v128_rdavg_u16(t3, v128_avg_u16(t13, v128_load_unaligned (b + 2*bs - 1))),
                        v128_rdavg_u16(t4, v128_avg_u16(v128_load_unaligned(b + bs - 2), t10)));
      bl = v128_sad_u16(bl, o, v128_rdavg_u16(t1, v128_rdavg_u16(v128_avg_u16(t11, t8), t2)));

      t1 = v128_rdavg_u16(v128_rdavg_u16(t3, v128_avg_u16(t12, v128_load_unaligned(b + 2*bs + 1))),
                        v128_rdavg_u16(t6, v128_avg_u16(t11, v128_load_unaligned(b + bs + 2))));
      br = v128_sad_u16(br, o, v128_rdavg_u16(t1, v128_rdavg_u16(t5, v128_avg_u16(t8, t10))));

      down = v128_sad_u16(down, o, v128_avg_u16(t7, t8));
      top = v128_sad_u16(top, o, v128_avg_u16(t7, t9));

      b += bs;
    }
    sad_top = v128_sad_u16_sum(top);
    sad_right = v128_sad_u16_sum(right);
    sad_down = v128_sad_u16_sum(down);
    sad_left = v128_sad_u16_sum(left);

    sad_tl = v128_sad_u16_sum(tl);
    sad_tr = v128_sad_u16_sum(tr);
    sad_bl = v128_sad_u16_sum(bl);
    sad_br = v128_sad_u16_sum(br);

  } else {
    sad256_internal_u16 top = v256_sad_u16_init();
    sad256_internal_u16 right = v256_sad_u16_init();
    sad256_internal_u16 down = v256_sad_u16_init();
    sad256_internal_u16 left = v256_sad_u16_init();
    sad256_internal_u16 tl = v256_sad_u16_init();
    sad256_internal_u16 tr = v256_sad_u16_init();
    sad256_internal_u16 br = v256_sad_u16_init();
    sad256_internal_u16 bl = v256_sad_u16_init();

    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j += 16) {
        v256 o = v256_load_aligned(a + i*as + j);

        v256 t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15;
        t7 = v256_load_unaligned(b);
        t8 = v256_load_unaligned(b + bs);
        t9 = v256_load_unaligned(b - bs);
        t10 = v256_load_unaligned(b + bs + 1);
        t11 = v256_load_unaligned(b + bs - 1);
        t12 = v256_load_unaligned(b - bs + 1);
        t13 = v256_load_unaligned(b - bs - 1);
        t14 = v256_load_unaligned(b - 1);
        t15 = v256_load_unaligned(b + 1);

        t2 = v256_avg_u16(t14, t7);
        left = v256_sad_u16(left, o, t2);
        t3 = v256_avg_u16(v256_load_unaligned(b - 2*bs), t8);
        t4 = v256_avg_u16(v256_load_unaligned(b - 2), t15);
        t1 = v256_rdavg_u16(v256_rdavg_u16(v256_avg_u16(v256_load_unaligned(b - 2*bs - 1), t11), t3),
                          v256_rdavg_u16(v256_avg_u16(v256_load_unaligned(b - bs - 2), t12), t4));
        tl = v256_sad_u16(tl, o, v256_rdavg_u16(t1, v256_rdavg_u16(v256_avg_u16(t13, t9), t2)));

        t5 = v256_avg_u16(t7, t15);
        right = v256_sad_u16(right, o, t5);
        t6 = v256_avg_u16(t14, v256_load_unaligned(b + 2));
        t1 = v256_rdavg_u16(v256_rdavg_u16(t3, v256_avg_u16(v256_load_unaligned(b - 2*bs + 1), t10)),
                          v256_rdavg_u16(t6, v256_avg_u16(t13, v256_load_unaligned(b - bs + 2))));
        tr = v256_sad_u16(tr, o, v256_rdavg_u16(t1, v256_rdavg_u16(v256_avg_u16(t9, t12), t5)));

        t3 = v256_avg_u16(t9, v256_load_unaligned(b + 2*bs));
        t1 = v256_rdavg_u16(v256_rdavg_u16(t3, v256_avg_u16(t13, v256_load_unaligned (b + 2*bs - 1))),
                          v256_rdavg_u16(t4, v256_avg_u16(v256_load_unaligned(b + bs - 2), t10)));
        bl = v256_sad_u16(bl, o, v256_rdavg_u16(t1, v256_rdavg_u16(v256_avg_u16(t11, t8), t2)));

        t1 = v256_rdavg_u16(v256_rdavg_u16(t3, v256_avg_u16(t12, v256_load_unaligned(b + 2*bs + 1))),
                          v256_rdavg_u16(t6, v256_avg_u16(t11, v256_load_unaligned(b + bs + 2))));
        br = v256_sad_u16(br, o, v256_rdavg_u16(t1, v256_rdavg_u16(t5, v256_avg_u16(t8, t10))));

        down = v256_sad_u16(down, o, v256_avg_u16(t7, t8));
        top = v256_sad_u16(top, o, v256_avg_u16(t7, t9));

        b += 16;
      }
      b += bs - width;
    }

    sad_top = v256_sad_u16_sum(top);
    sad_right = v256_sad_u16_sum(right);
    sad_down = v256_sad_u16_sum(down);
    sad_left = v256_sad_u16_sum(left);

    sad_tl = v256_sad_u16_sum(tl);
    sad_tr = v256_sad_u16_sum(tr);
    sad_bl = v256_sad_u16_sum(bl);
    sad_br = v256_sad_u16_sum(br);
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
unsigned int TEMPLATE(sad_calc_fastquarter_simd)(const SAMPLE *po, const SAMPLE *r, int os, int rs, int width, int height, int *x, int *y)
{
  unsigned int sad_tl, sad_tr, sad_br, sad_bl;
  unsigned int sad_top, sad_right, sad_down, sad_left;
  int bestx = 0, besty = -1;

  if (width == 8) {
    sad128_internal_u16 top = v128_sad_u16_init();
    sad128_internal_u16 right = v128_sad_u16_init();
    sad128_internal_u16 down = v128_sad_u16_init();
    sad128_internal_u16 left = v128_sad_u16_init();
    sad128_internal_u16 tl = v128_sad_u16_init();
    sad128_internal_u16 tr = v128_sad_u16_init();
    sad128_internal_u16 br = v128_sad_u16_init();
    sad128_internal_u16 bl = v128_sad_u16_init();

    if (*x & *y) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 8) {

          v128 o = v128_load_aligned(po + j);
          v128 a = v128_load_unaligned(r + j);
          v128 d = v128_load_unaligned(r + j + 1);
          v128 e = v128_load_unaligned(r + j + rs + 1);
          v128 f = v128_load_unaligned(r + j + rs);
          v128 ad = v128_avg_u16(a, d);
          v128 de = v128_avg_u16(d, e);
          v128 af = v128_avg_u16(a, f);
          v128 fe = v128_avg_u16(f, e);

          tl =    v128_sad_u16(tl, o, v128_rdavg_u16(ad, af));
          top =   v128_sad_u16(top, o, v128_rdavg_u16(de,  a));
          tr =    v128_sad_u16(tr, o, v128_rdavg_u16(ad, de));
          left =  v128_sad_u16(left, o, v128_rdavg_u16(ad,  f));
          right = v128_sad_u16(right, o, v128_rdavg_u16(ad,  e));
          bl =    v128_sad_u16(bl, o, v128_rdavg_u16(af, fe));
          down =  v128_sad_u16(down, o, v128_rdavg_u16(de,  f));
          br =    v128_sad_u16(br, o, v128_rdavg_u16(de, fe));
        }
        r += rs;
        po += os;
      }
    } else if (*x) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 8) {

          v128 o = v128_load_aligned(po + j);
          v128 a = v128_load_unaligned(r + j);
          v128 b = v128_load_unaligned(r + j - rs);
          v128 c = v128_load_unaligned(r + j - rs + 1);
          v128 d = v128_load_unaligned(r + j + 1);
          v128 e = v128_load_unaligned(r + j + rs + 1);
          v128 f = v128_load_unaligned(r + j + rs);
          v128 ad = v128_avg_u16(a, d);
          v128 de = v128_avg_u16(d, e);
          v128 dc = v128_avg_u16(d, c);
          v128 af = v128_avg_u16(a, f);
          v128 ab = v128_avg_u16(a, b);

          tl =    v128_sad_u16(tl, o, v128_rdavg_u16(ad, ab));
          top =   v128_sad_u16(top, o, v128_rdavg_u16(dc,  a));
          tr =    v128_sad_u16(tr, o, v128_rdavg_u16(ad, dc));
          left =  v128_sad_u16(left, o, v128_rdavg_u16(ad,  a));
          right = v128_sad_u16(right, o, v128_rdavg_u16(ad,  d));
          bl =    v128_sad_u16(bl, o, v128_rdavg_u16(ad, af));
          down =  v128_sad_u16(down, o, v128_rdavg_u16(af,  d));
          br =    v128_sad_u16(br, o, v128_rdavg_u16(ad, de));
        }
        r += rs;
        po += os;
      }
    } else if (*y) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 8) {

          v128 o = v128_load_aligned(po + j);
          v128 a = v128_load_unaligned(r + j);
          v128 d = v128_load_unaligned(r + j + 1);
          v128 e = v128_load_unaligned(r + j + rs + 1);
          v128 f = v128_load_unaligned(r + j + rs);
          v128 g = v128_load_unaligned(r + j + rs - 1);
          v128 h = v128_load_unaligned(r + j - 1);
          v128 ad = v128_avg_u16(a, d);
          v128 af = v128_avg_u16(a, f);
          v128 fe = v128_avg_u16(f, e);
          v128 ah = v128_avg_u16(a, h);
          v128 gf = v128_avg_u16(g, f);

          tl =    v128_sad_u16(tl, o, v128_rdavg_u16(ah, af));
          top =   v128_sad_u16(top, o, v128_rdavg_u16(af,  a));
          tr =    v128_sad_u16(tr, o, v128_rdavg_u16(ad, af));
          left =  v128_sad_u16(left, o, v128_rdavg_u16(gf,  a));
          right = v128_sad_u16(right, o, v128_rdavg_u16(ad,  f));
          bl =    v128_sad_u16(bl, o, v128_rdavg_u16(af, gf));
          down =  v128_sad_u16(down, o, v128_rdavg_u16(af,  f));
          br =    v128_sad_u16(br, o, v128_rdavg_u16(af, fe));
        }
        r += rs;
        po += os;
      }
    } else {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 8) {

          v128 o = v128_load_aligned(po + j);
          v128 a = v128_load_unaligned(r + j);
          v128 b = v128_load_unaligned(r + j - rs);
          v128 d = v128_load_unaligned(r + j + 1);
          v128 f = v128_load_unaligned(r + j + rs);
          v128 h = v128_load_unaligned(r + j - 1);
          v128 ad = v128_avg_u16(a, d);
          v128 af = v128_avg_u16(a, f);
          v128 ah = v128_avg_u16(a, h);
          v128 ab = v128_avg_u16(a, b);

          tl =    v128_sad_u16(tl, o, v128_rdavg_u16(ah, ab));
          top =   v128_sad_u16(top, o, v128_rdavg_u16(ab,  a));
          tr =    v128_sad_u16(tr, o, v128_rdavg_u16(ad, ab));
          left =  v128_sad_u16(left, o, v128_rdavg_u16(ah,  a));
          right = v128_sad_u16(right, o, v128_rdavg_u16(ad,  a));
          bl =    v128_sad_u16(bl, o, v128_rdavg_u16(ah, af));
          down =  v128_sad_u16(down, o, v128_rdavg_u16(af,  a));
          br =    v128_sad_u16(br, o, v128_rdavg_u16(af, ad));
        }
        r += rs;
        po += os;
      }
    }

    sad_top = v128_sad_u16_sum(top);
    sad_right = v128_sad_u16_sum(right);
    sad_down = v128_sad_u16_sum(down);
    sad_left = v128_sad_u16_sum(left);

    sad_tl = v128_sad_u16_sum(tl);
    sad_tr = v128_sad_u16_sum(tr);
    sad_bl = v128_sad_u16_sum(bl);
    sad_br = v128_sad_u16_sum(br);
  } else {
    sad256_internal_u16 top = v256_sad_u16_init();
    sad256_internal_u16 right = v256_sad_u16_init();
    sad256_internal_u16 down = v256_sad_u16_init();
    sad256_internal_u16 left = v256_sad_u16_init();
    sad256_internal_u16 tl = v256_sad_u16_init();
    sad256_internal_u16 tr = v256_sad_u16_init();
    sad256_internal_u16 br = v256_sad_u16_init();
    sad256_internal_u16 bl = v256_sad_u16_init();

    if (*x & *y) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 16) {

          v256 o = v256_load_aligned(po + j);
          v256 a = v256_load_unaligned(r + j);
          v256 d = v256_load_unaligned(r + j + 1);
          v256 e = v256_load_unaligned(r + j + rs + 1);
          v256 f = v256_load_unaligned(r + j + rs);
          v256 ad = v256_avg_u16(a, d);
          v256 de = v256_avg_u16(d, e);
          v256 af = v256_avg_u16(a, f);
          v256 fe = v256_avg_u16(f, e);

          tl =    v256_sad_u16(tl, o, v256_rdavg_u16(ad, af));
          top =   v256_sad_u16(top, o, v256_rdavg_u16(de,  a));
          tr =    v256_sad_u16(tr, o, v256_rdavg_u16(ad, de));
          left =  v256_sad_u16(left, o, v256_rdavg_u16(ad,  f));
          right = v256_sad_u16(right, o, v256_rdavg_u16(ad,  e));
          bl =    v256_sad_u16(bl, o, v256_rdavg_u16(af, fe));
          down =  v256_sad_u16(down, o, v256_rdavg_u16(de,  f));
          br =    v256_sad_u16(br, o, v256_rdavg_u16(de, fe));
        }
        r += rs;
        po += os;
      }
    } else if (*x) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 16) {

          v256 o = v256_load_aligned(po + j);
          v256 a = v256_load_unaligned(r + j);
          v256 b = v256_load_unaligned(r + j - rs);
          v256 c = v256_load_unaligned(r + j - rs + 1);
          v256 d = v256_load_unaligned(r + j + 1);
          v256 e = v256_load_unaligned(r + j + rs + 1);
          v256 f = v256_load_unaligned(r + j + rs);
          v256 ad = v256_avg_u16(a, d);
          v256 de = v256_avg_u16(d, e);
          v256 dc = v256_avg_u16(d, c);
          v256 af = v256_avg_u16(a, f);
          v256 ab = v256_avg_u16(a, b);

          tl =    v256_sad_u16(tl, o, v256_rdavg_u16(ad, ab));
          top =   v256_sad_u16(top, o, v256_rdavg_u16(dc,  a));
          tr =    v256_sad_u16(tr, o, v256_rdavg_u16(ad, dc));
          left =  v256_sad_u16(left, o, v256_rdavg_u16(ad,  a));
          right = v256_sad_u16(right, o, v256_rdavg_u16(ad,  d));
          bl =    v256_sad_u16(bl, o, v256_rdavg_u16(ad, af));
          down =  v256_sad_u16(down, o, v256_rdavg_u16(af,  d));
          br =    v256_sad_u16(br, o, v256_rdavg_u16(ad, de));
        }
        r += rs;
        po += os;
      }
    } else if (*y) {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 16) {

          v256 o = v256_load_aligned(po + j);
          v256 a = v256_load_unaligned(r + j);
          v256 d = v256_load_unaligned(r + j + 1);
          v256 e = v256_load_unaligned(r + j + rs + 1);
          v256 f = v256_load_unaligned(r + j + rs);
          v256 g = v256_load_unaligned(r + j + rs - 1);
          v256 h = v256_load_unaligned(r + j - 1);
          v256 ad = v256_avg_u16(a, d);
          v256 af = v256_avg_u16(a, f);
          v256 fe = v256_avg_u16(f, e);
          v256 ah = v256_avg_u16(a, h);
          v256 gf = v256_avg_u16(g, f);

          tl =    v256_sad_u16(tl, o, v256_rdavg_u16(ah, af));
          top =   v256_sad_u16(top, o, v256_rdavg_u16(af,  a));
          tr =    v256_sad_u16(tr, o, v256_rdavg_u16(ad, af));
          left =  v256_sad_u16(left, o, v256_rdavg_u16(gf,  a));
          right = v256_sad_u16(right, o, v256_rdavg_u16(ad,  f));
          bl =    v256_sad_u16(bl, o, v256_rdavg_u16(af, gf));
          down =  v256_sad_u16(down, o, v256_rdavg_u16(af,  f));
          br =    v256_sad_u16(br, o, v256_rdavg_u16(af, fe));
        }
        r += rs;
        po += os;
      }
    } else {
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 16) {

          v256 o = v256_load_aligned(po + j);
          v256 a = v256_load_unaligned(r + j);
          v256 b = v256_load_unaligned(r + j - rs);
          v256 d = v256_load_unaligned(r + j + 1);
          v256 f = v256_load_unaligned(r + j + rs);
          v256 h = v256_load_unaligned(r + j - 1);
          v256 ad = v256_avg_u16(a, d);
          v256 af = v256_avg_u16(a, f);
          v256 ah = v256_avg_u16(a, h);
          v256 ab = v256_avg_u16(a, b);

          tl =    v256_sad_u16(tl, o, v256_rdavg_u16(ah, ab));
          top =   v256_sad_u16(top, o, v256_rdavg_u16(ab,  a));
          tr =    v256_sad_u16(tr, o, v256_rdavg_u16(ad, ab));
          left =  v256_sad_u16(left, o, v256_rdavg_u16(ah,  a));
          right = v256_sad_u16(right, o, v256_rdavg_u16(ad,  a));
          bl =    v256_sad_u16(bl, o, v256_rdavg_u16(ah, af));
          down =  v256_sad_u16(down, o, v256_rdavg_u16(af,  a));
          br =    v256_sad_u16(br, o, v256_rdavg_u16(af, ad));
        }
        r += rs;
        po += os;
      }
    }

    sad_top = v256_sad_u16_sum(top);
    sad_right = v256_sad_u16_sum(right);
    sad_down = v256_sad_u16_sum(down);
    sad_left = v256_sad_u16_sum(left);

    sad_tl = v256_sad_u16_sum(tl);
    sad_tr = v256_sad_u16_sum(tr);
    sad_bl = v256_sad_u16_sum(bl);
    sad_br = v256_sad_u16_sum(br);
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

#ifndef HBD
int calc_cbp_simd(int32_t *block, int size, int threshold) {
  int cbp = 0;
  if (size ==8 ) {
    v256 thr = v256_dup_32(threshold);
    v256 sum = v256_add_32(v256_load_aligned(block+0*size),
                           v256_load_aligned(block+1*size));
    sum = v256_add_32(sum, v256_load_aligned(block+2*size));
    sum = v256_add_32(sum, v256_load_aligned(block+3*size));
    sum = v256_add_32(sum, v256_load_aligned(block+4*size));
    sum = v256_add_32(sum, v256_load_aligned(block+5*size));
    sum = v256_add_32(sum, v256_load_aligned(block+6*size));
    sum = v256_add_32(sum, v256_load_aligned(block+7*size));
    cbp = !!v256_hadd_u16(v256_cmpgt_s32(sum, thr));
  } else {
    v128 sum = v128_add_32(v128_load_aligned(block+0*size),
                         v128_load_aligned(block+1*size));
    sum = v128_add_32(sum, v128_load_aligned(block+2*size));
    sum = v128_add_32(sum, v128_load_aligned(block+3*size));
    sum = v128_add_64(v128_shr_n_s64(sum, 16),
                     v128_shr_n_s64(v128_shl_n_64(sum, 16), 16));
    cbp = v128_high_v64(sum) > threshold || v128_low_v64(sum) > threshold;
  }
  return cbp;
}
#endif
