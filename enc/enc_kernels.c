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
#include "encode_block.h"

int TEMPLATE(sad_calc_simd)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int width, int height)
{
  int i, j;

  if (width == 8) {
    sad64_internal s = v64_sad_u8_init();
#ifdef HBD
    if ((intptr_t)b & 15)
#else
    if ((intptr_t)b & 7)
#endif
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
#ifdef HBD
    if ((intptr_t)b & 31)
#else
    if ((intptr_t)b & 15)
#endif
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


unsigned int TEMPLATE(widesad_calc_simd)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int width, int height, int *x)
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
#ifdef HBD
    s1 = v128_sad_u8(s1, aa, v128_align(bb, ba, 2*2));
    s2 = v128_sad_u8(s2, aa, v128_align(bb, ba, 3*2));
    s3 = v128_sad_u8(s3, aa, v128_align(bb, ba, 4*2));
    s4 = v128_sad_u8(s4, aa, v128_align(bb, ba, 6*2));
#else
    s1 = v128_sad_u8(s1, aa, v128_align(bb, ba, 2));
    s2 = v128_sad_u8(s2, aa, v128_align(bb, ba, 3));
    s3 = v128_sad_u8(s3, aa, v128_align(bb, ba, 4));
    s4 = v128_sad_u8(s4, aa, v128_align(bb, ba, 6));
#endif
    b += bstride;
    a += astride;
  }

  unsigned int r = min(min(min((v128_sad_u8_sum(s0) << 3) | 0, (v128_sad_u8_sum(s1) << 3) | 2),
                           min((v128_sad_u8_sum(s2) << 3) | 3, (v128_sad_u8_sum(s3) << 3) | 4)), (v128_sad_u8_sum(s4) << 3) | 6);
  *x = (int)(r & 7) - 3;
  return r >> 3;
}

uint64_t TEMPLATE(ssd_calc_simd)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int size)
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
    uint64_t ssd = 0;
    for (i = 0; i < size; i += 8) {
      ssd128_internal s = v128_ssd_u8_init();
      for (j = 0; j < size; j += 16) {
        s = v128_ssd_u8(s, v128_load_aligned(a + (i+0)*astride + j), v128_load_aligned(b + (i+0)*bstride + j));
        s = v128_ssd_u8(s, v128_load_aligned(a + (i+1)*astride + j), v128_load_aligned(b + (i+1)*bstride + j));
        s = v128_ssd_u8(s, v128_load_aligned(a + (i+2)*astride + j), v128_load_aligned(b + (i+2)*bstride + j));
        s = v128_ssd_u8(s, v128_load_aligned(a + (i+3)*astride + j), v128_load_aligned(b + (i+3)*bstride + j));
        s = v128_ssd_u8(s, v128_load_aligned(a + (i+4)*astride + j), v128_load_aligned(b + (i+4)*bstride + j));
        s = v128_ssd_u8(s, v128_load_aligned(a + (i+5)*astride + j), v128_load_aligned(b + (i+5)*bstride + j));
        s = v128_ssd_u8(s, v128_load_aligned(a + (i+6)*astride + j), v128_load_aligned(b + (i+6)*bstride + j));
        s = v128_ssd_u8(s, v128_load_aligned(a + (i+7)*astride + j), v128_load_aligned(b + (i+7)*bstride + j));
      }
      ssd += v128_ssd_u8_sum(s);
    }
    return ssd;
  }
}

#ifdef HBD
static ALIGN(32) uint64_t c_shuff[] = { 0x0302010001000100LL, 0x0b0a090807060504LL,
                                        0x0302010001000100LL, 0x0b0a090807060504LL };
static ALIGN(32) uint64_t d_shuff[] = { 0x0504030201000100LL, 0x0d0c0b0a09080706LL,
                                        0x0504030201000100LL, 0x0d0c0b0a09080706LL };
static ALIGN(32) uint64_t e_shuff[] = { 0x0908070605040302LL, 0x0f0e0f0e0d0c0b0aLL,
                                        0x0908070605040302LL, 0x0f0e0f0e0d0c0b0aLL };
static ALIGN(32) uint64_t f_shuff[] = { 0x0b0a090807060504LL, 0x0f0e0f0e0f0e0d0cLL,
                                        0x0b0a090807060504LL, 0x0f0e0f0e0f0e0d0cLL };
#else
static ALIGN(16) uint64_t c_shuff[] = { 0x0504030201000000LL, 0x0d0c0b0a09080808LL };
static ALIGN(16) uint64_t d_shuff[] = { 0x0605040302010000LL, 0x0e0d0c0b0a090808LL };
static ALIGN(16) uint64_t e_shuff[] = { 0x0707060504030201LL, 0x0f0f0e0d0c0b0a09LL };
static ALIGN(16) uint64_t f_shuff[] = { 0x0707070605040302LL, 0x0f0f0f0e0d0c0b0aLL };
#endif

SIMD_INLINE void calc_diff(v128 o, v128 *a, v128 *b, v128 *c, v128 *d, v128 *e, v128 *f) {
#ifdef HBD
  *a = v128_sub_8(*a, o);
  *b = v128_sub_8(*b, o);
  *c = v128_sub_8(*c, o);
  *d = v128_sub_8(*d, o);
  *e = v128_sub_8(*e, o);
  *f = v128_sub_8(*f, o);
#else
  // The difference will be 9 bit, offset by 128 so we can use saturated
  // sub to avoid going to 16 bit temporarily before "strength" clipping.
  v128 c128 = v128_dup_8(128);
  v128 x = v128_add_8(c128, o);
  *a = v128_ssub_s8(v128_add_8(c128, *a), x);
  *b = v128_ssub_s8(v128_add_8(c128, *b), x);
  *c = v128_ssub_s8(v128_add_8(c128, *c), x);
  *d = v128_ssub_s8(v128_add_8(c128, *d), x);
  *e = v128_ssub_s8(v128_add_8(c128, *e), x);
  *f = v128_ssub_s8(v128_add_8(c128, *f), x);
#endif
}

SIMD_INLINE void clip_sides(v128 *c, v128 *d, v128 *e, v128 *f, int left, int right) {
  if (!left) {  // Left clipping
    *c = v128_shuffle_8(*c, v128_load_aligned(c_shuff));
    *d = v128_shuffle_8(*d, v128_load_aligned(d_shuff));
  }
  if (!right) {  // Right clipping
    *e = v128_shuffle_8(*e, v128_load_aligned(e_shuff));
    *f = v128_shuffle_8(*f, v128_load_aligned(f_shuff));
  }
}

SIMD_INLINE void read_two_lines(const SAMPLE *rec, const SAMPLE *org, int rstride, int ostride, int x0, int y0, int bottom, int right, int y, v128 *o, v128 *r, v128 *a, v128 *b, v128 *c, v128 *d, v128 *e, v128 *f, v128 *g, v128 *h) {
  const v64 k1 = v64_load_aligned(org);
  const v64 k2 = v64_load_aligned(org + ostride);
  const v64 l1 = v64_load_aligned(rec);
  const v64 l2 = v64_load_aligned(rec + rstride);
  const v64 l3 = v64_load_aligned(rec - (y != -y0) * rstride);
  const v64 l4 = v64_load_aligned(rec + ((y != bottom) + 1) * rstride);
  *o = v128_from_v64(k1, k2);
  *r = v128_from_v64(l1, l2);
  *a = v128_from_v64(v64_load_aligned(rec - 2 * (y != -y0) * rstride), l3);
  *b = v128_from_v64(l3, l1);
  *g = v128_from_v64(l2, l4);
  *h = v128_from_v64(l4, v64_load_aligned(rec + (2 * (y != bottom) + 1) * rstride));
  *c = v128_from_v64(v64_load_unaligned(rec - 2 * !!x0), v64_load_unaligned(rec - 2 * !!x0 + rstride));
  *d = v128_from_v64(v64_load_unaligned(rec - !!x0), v64_load_unaligned(rec - !!x0 + rstride));
  *e = v128_from_v64(v64_load_unaligned(rec + !!right), v64_load_unaligned(rec + !!right + rstride));
  *f = v128_from_v64(v64_load_unaligned(rec + 2 * !!right), v64_load_unaligned(rec + 2 * !!right + rstride));
  clip_sides(c, d, e, f, x0, right);
}

// sign(a - b) * max(0, abs(a - b) - max(0, abs(a - b) -
// strength + (abs(a - b) >> (dmp - log2(s)))))
SIMD_INLINE v128 constrain(v128 a, v128 b, unsigned int strength, unsigned int dmp) {
#ifdef HBD
  const v128 diff = v128_sub_8(v128_max_s8(a, b), v128_min_s8(a, b));
  const v128 sign = v128_cmpeq_8(v128_min_s8(a, b), a);  // -(a <= b)
  const v128 zero = v128_zero();
  const v128 s = v128_max_s8(zero, v128_sub_8(v128_dup_8(strength), v128_shr_u8(diff, dmp - log2i(strength))));
  return v128_sub_8(v128_xor(sign, v128_max_s8(zero, v128_sub_8(diff, v128_max_s8(zero, v128_sub_8(diff, s))))), sign);
#else
  const v128 diff = v128_sub_8(v128_max_u8(a, b), v128_min_u8(a, b));
  const v128 sign = v128_cmpeq_8(v128_min_u8(a, b), a);  // -(a <= b)
  const v128 s = v128_ssub_u8(v128_dup_8(strength), v128_shr_u8(diff, dmp - log2i(strength)));
  return v128_sub_8(v128_xor(sign, v128_ssub_u8(diff, v128_ssub_u8(diff, s))), sign);
#endif
}

// delta = 1/16 * constrain(a, x, s) + 3/16 * constrain(b, x, s) +
//         1/16 * constrain(c, x, s) + 3/16 * constrain(d, x, s) +
//         3/16 * constrain(e, x, s) + 1/16 * constrain(f, x, s) +
//         3/16 * constrain(g, x, s) + 1/16 * constrain(h, x, s)
SIMD_INLINE v128 calc_delta(v128 x, v128 a, v128 b, v128 c, v128 d, v128 e,
                            v128 f, v128 g, v128 h, unsigned int s,
                            unsigned int dmp) {
  const v128 bdeg =
      v128_add_8(v128_add_8(constrain(b, x, s, dmp), constrain(d, x, s, dmp)),
                 v128_add_8(constrain(e, x, s, dmp), constrain(g, x, s, dmp)));
  const v128 delta = v128_add_8(
      v128_add_8(v128_add_8(constrain(a, x, s, dmp), constrain(c, x, s, dmp)),
                 v128_add_8(constrain(f, x, s, dmp), constrain(h, x, s, dmp))),
      v128_add_8(v128_add_8(bdeg, bdeg), bdeg));
  return v128_add_8(x, v128_shr_s8(v128_add_8(v128_dup_8(8), v128_add_8(delta, v128_cmplt_s8(delta, v128_zero()))), 4));
}

void TEMPLATE(detect_clpf_simd)(const SAMPLE *rec,const SAMPLE *org,int x0, int y0, int width, int height, int ostride, int rstride, int *sum0, int *sum1, unsigned int strength, unsigned int shift, unsigned int size, unsigned int dmp)
{
  TEMPLATE(detect_clpf)(rec, org, x0, y0, width, height, ostride, rstride, sum0, sum1, strength, shift, size, dmp);
  const int bottom = height - 2 - y0;
  const int right = width - 8 - x0;
  ssd128_internal ssd0 = v128_ssd_u8_init();
  ssd128_internal ssd1 = v128_ssd_u8_init();
  int y;

  if (size != 8) {  // Fallback to plain C
    TEMPLATE(detect_clpf)(rec, org, x0, y0, width, height, ostride, rstride, sum0, sum1, strength, shift, size, dmp);
    return;
  }

  rec += x0 + y0 * rstride;
  org += x0 + y0 * ostride;

  for (y = 0; y < 8; y += 2) {
    v128 a, b, c, d, e, f, g, h, o, r;
    read_two_lines(rec, org, rstride, ostride, x0, y0, bottom, right, y, &o, &r,
                   &a, &b, &c, &d, &e, &f, &g, &h);
    ssd0 = v128_ssd_u8(ssd0, o, r);
    ssd1 = v128_ssd_u8(ssd1, o, calc_delta(r, a, b, c, d, e, f, g, h, strength, dmp));
    rec += rstride * 2;
    org += ostride * 2;
  }
  *sum0 += v128_ssd_u8_sum(ssd0) >> (shift*2);
  *sum1 += v128_ssd_u8_sum(ssd1) >> (shift*2);
}

// Test multiple filter strengths at once.
SIMD_INLINE void calc_delta_multi(v128 r, v128 o, v128 a, v128 b, v128 c,
                                  v128 d, v128 e, v128 f,v128 g, v128 h, ssd128_internal *ssd1,
                                  ssd128_internal *ssd2, ssd128_internal *ssd3, unsigned int shift, unsigned int dmp) {
  *ssd1 = v128_ssd_u8(*ssd1, o, calc_delta(r, a, b, c, d, e, f, g, h, 1 << shift, dmp));
  *ssd2 = v128_ssd_u8(*ssd2, o, calc_delta(r, a, b, c, d, e, f, g, h, 2 << shift, dmp));
  *ssd3 = v128_ssd_u8(*ssd3, o, calc_delta(r, a, b, c, d, e, f, g, h, 4 << shift, dmp));
}

void TEMPLATE(detect_multi_clpf_simd)(const SAMPLE *rec,const SAMPLE *org,int x0, int y0, int width, int height, int ostride,int rstride, int *sum, unsigned int shift, unsigned int size, unsigned int dmp)
{
  const int bottom = height - 2 - y0;
  const int right = width - 8 - x0;
  ssd128_internal ssd0 = v128_ssd_u8_init();
  ssd128_internal ssd1 = v128_ssd_u8_init();
  ssd128_internal ssd2 = v128_ssd_u8_init();
  ssd128_internal ssd3 = v128_ssd_u8_init();
  int y;

  if (size != 8) {  // Fallback to plain C
    TEMPLATE(detect_multi_clpf)(rec, org, x0, y0, width, height, rstride, ostride, sum, shift, size, dmp);
    return;
  }

  rec += x0 + y0 * rstride;
  org += x0 + y0 * ostride;

  for (y = 0; y < 8; y += 2) {
    v128 a, b, c, d, e, f, g, h, o, r;
    read_two_lines(rec, org, rstride, ostride, x0, y0, bottom, right, y, &o, &r,
                   &a, &b, &c, &d, &e, &f, &g, &h);
    ssd0 = v128_ssd_u8(ssd0, o, r);
    calc_delta_multi(r, o, a, b, c, d, e, f, g, h, &ssd1, &ssd2, &ssd3, shift, dmp);
    rec += 2 * rstride;
    org += 2 * ostride;
  }
  sum[0] += v128_ssd_u8_sum(ssd0) >> (shift*2);
  sum[1] += v128_ssd_u8_sum(ssd1) >> (shift*2);
  sum[2] += v128_ssd_u8_sum(ssd2) >> (shift*2);
  sum[3] += v128_ssd_u8_sum(ssd3) >> (shift*2);
}

/* Return the best approximated half-pel position around the centre */
unsigned int TEMPLATE(sad_calc_fasthalf_simd)(const SAMPLE *a, const SAMPLE *b, int as, int bs, int width, int height, int *x, int *y, unsigned int *maxsad)
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
unsigned int TEMPLATE(sad_calc_fastquarter_simd)(const SAMPLE *po, const SAMPLE *r, int os, int rs, int width, int height, int *x, int *y)
{
  unsigned int sad_tl, sad_tr, sad_br, sad_bl;
  unsigned int sad_top, sad_right, sad_down, sad_left;
  int bestx = 0, besty = -1;

  if (width == 8) {
    sad64_internal top = v64_sad_u8_init();
    sad64_internal right = v64_sad_u8_init();
    sad64_internal down = v64_sad_u8_init();
    sad64_internal left = v64_sad_u8_init();
    sad64_internal tl = v64_sad_u8_init();
    sad64_internal tr = v64_sad_u8_init();
    sad64_internal br = v64_sad_u8_init();
    sad64_internal bl = v64_sad_u8_init();

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
    sad128_internal top = v128_sad_u8_init();
    sad128_internal right = v128_sad_u8_init();
    sad128_internal down = v128_sad_u8_init();
    sad128_internal left = v128_sad_u8_init();
    sad128_internal tl = v128_sad_u8_init();
    sad128_internal tr = v128_sad_u8_init();
    sad128_internal br = v128_sad_u8_init();
    sad128_internal bl = v128_sad_u8_init();

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

#ifndef HBD
int calc_cbp_simd(int16_t *block, int size, int threshold) {
  int cbp = 0;
  if (size ==8 ) {
    v128 thr = v128_dup_16(threshold);
    v128 sum = v128_add_16(v128_load_aligned(block+0*size),
                           v128_load_aligned(block+1*size));
    sum = v128_add_16(sum, v128_load_aligned(block+2*size));
    sum = v128_add_16(sum, v128_load_aligned(block+3*size));
    sum = v128_add_16(sum, v128_load_aligned(block+4*size));
    sum = v128_add_16(sum, v128_load_aligned(block+5*size));
    sum = v128_add_16(sum, v128_load_aligned(block+6*size));
    sum = v128_add_16(sum, v128_load_aligned(block+7*size));
    cbp = !!v128_hadd_u8(v128_cmpgt_s16(sum, thr));
  } else {
    v64 sum = v64_add_16(v64_load_aligned(block+0*size),
                         v64_load_aligned(block+1*size));
    sum = v64_add_16(sum, v64_load_aligned(block+2*size));
    sum = v64_add_16(sum, v64_load_aligned(block+3*size));
    sum = v64_add_32(v64_shr_n_s32(sum, 16),
                     v64_shr_n_s32(v64_shl_n_32(sum, 16), 16));
    cbp = v64_high_s32(sum) > threshold || v64_low_s32(sum) > threshold;
  }
  return cbp;
}
#endif
