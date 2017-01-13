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

#include <string.h>

#include "simd.h"
#include "global.h"

void TEMPLATE(block_avg_simd)(SAMPLE *p,SAMPLE *r0, SAMPLE *r1, int sp, int s0, int s1, int width, int height)
{
  int i,j;
  if (width == 4) {
    v128 a, b, out;
    // Assume height is divisible by 2
    for (i=0; i<height; i+=2) {
      a = v128_from_v64(v64_load_unaligned(&r0[i*s0]), v64_load_unaligned(&r0[(i+1)*s0]));
      b = v128_from_v64(v64_load_unaligned(&r1[i*s1]), v64_load_unaligned(&r1[(i+1)*s1]));
      out = v128_avg_u16(a,b);
      v64_store_unaligned(&p[i*sp], v128_high_v64(out));
      v64_store_unaligned(&p[(i+1)*sp], v128_low_v64(out));
    }
  } else {
    v128 a, b, c, d;
    // Assume width divisible by 8, height by 2
    for (i=0; i<height; i+=2) {
      for (j=0; j<width; j+=8) {
        a = v128_load_unaligned(&r0[i*s0+j]);
        b = v128_load_unaligned(&r1[i*s1+j]);
        c = v128_load_unaligned(&r0[(i+1)*s0+j]);
        d = v128_load_unaligned(&r1[(i+1)*s1+j]);
        v128_store_aligned(&p[i*sp+j],v128_avg_u16(a,b));
        v128_store_aligned(&p[(i+1)*sp+j],v128_avg_u16(c,d));
      }
    }
  }

}

int TEMPLATE(sad_calc_simd_unaligned)(SAMPLE *a, SAMPLE *b, int astride, int bstride, int width, int height)
{
  int i, j;

  switch (width) {
    case 8 :
      {
        sad128_internal_u16 s = v128_sad_u16_init();
        for (i = 0; i < height; i += 4) {
          s = v128_sad_u16(s, v128_load_unaligned(a + 0*astride), v128_load_unaligned(b + 0*bstride));
          s = v128_sad_u16(s, v128_load_unaligned(a + 1*astride), v128_load_unaligned(b + 1*bstride));
          s = v128_sad_u16(s, v128_load_unaligned(a + 2*astride), v128_load_unaligned(b + 2*bstride));
          s = v128_sad_u16(s, v128_load_unaligned(a + 3*astride), v128_load_unaligned(b + 3*bstride));
          a += 4*astride;
          b += 4*bstride;
        }
        return v128_sad_u16_sum(s);
      }
    case 4:
      {
        sad256_internal_u16 s = v256_sad_u16_init();
        for (i = 0; i < height; i += 4) {
          s = v256_sad_u16(s, v256_from_v64(v64_load_aligned(a+(i+3)*astride),
                                          v64_load_aligned(a+(i+2)*astride),
                                          v64_load_aligned(a+(i+1)*astride),
                                          v64_load_aligned(a+i*astride)),
                          v256_from_v64(v64_load_aligned(b+(i+3)*bstride),
                                       v64_load_aligned(b+(i+2)*bstride),
                                       v64_load_aligned(b+(i+1)*bstride),
                                       v64_load_aligned(b+i*bstride)));
        }
        return v256_sad_u16_sum(s);
      }
    default:
      {
        sad256_internal_u16 s = v256_sad_u16_init();
        for (i = 0; i < height; i+=4)
          for (j = 0; j < width; j += 16) {
            s = v256_sad_u16(s, v256_load_unaligned(a + 0*astride + j), v256_load_unaligned(b + 0*bstride + j));
            s = v256_sad_u16(s, v256_load_unaligned(a + 1*astride + j), v256_load_unaligned(b + 1*bstride + j));
            s = v256_sad_u16(s, v256_load_unaligned(a + 2*astride + j), v256_load_unaligned(b + 2*bstride + j));
            s = v256_sad_u16(s, v256_load_unaligned(a + 3*astride + j), v256_load_unaligned(b + 3*bstride + j));
            a += 4*astride;
            b += 4*bstride;
          }
        return v256_sad_u16_sum(s);
      }
  };
}

#ifndef HBD

enum {
    COEFF_DC_ONLY = 0,
    COEFF_4x4_ONLY,
    COEFF_8x8_ONLY,
    COEFF_ALL
};

/* Check whether coeffs are DC only, 4x4, 8x8 or larger. */
int check_nz_area(const int32_t *coeff, int size)
{
  uint64_t *c64 = (uint64_t *)coeff;
  int other3, rest;
  int dc_only4 = !((c64[0] >> 16) | c64[size / 4] | c64[size / 2] | c64[size * 3 / 4]);

  if (size == 4)
    return dc_only4 ? COEFF_DC_ONLY : COEFF_ALL;
  other3 = !(c64[1] | c64[size / 4 + 1] | c64[size / 2 + 1] | c64[size * 3 / 4 + 1] |
             c64[size] | c64[size + 1] | c64[size * 5 / 4] | c64[size * 5 / 4 + 1] |
             c64[size * 3 / 2] | c64[size * 3 / 2 + 1] | c64[size * 7 / 4] | c64[size * 7 / 4 + 1]);
  if (size == 8)
    return dc_only4 && other3 ? COEFF_DC_ONLY : (other3 ? COEFF_4x4_ONLY : COEFF_8x8_ONLY);
  /* The test should be in quant/dequant instead */
  rest = !(c64[2+size/4*0] | c64[3+size/4*0] |
           c64[2+size/4*1] | c64[3+size/4*1] |
           c64[2+size/4*2] | c64[3+size/4*2] |
           c64[2+size/4*3] | c64[3+size/4*3] |
           c64[2+size/4*4] | c64[3+size/4*4] |
           c64[2+size/4*5] | c64[3+size/4*5] |
           c64[2+size/4*6] | c64[3+size/4*6] |
           c64[2+size/4*7] | c64[3+size/4*7] |
           c64[0+size/4*8] | c64[1+size/4*8] | c64[2+size/4*8] | c64[3+size/4*8] |
           c64[0+size/4*9] | c64[1+size/4*9] | c64[2+size/4*9] | c64[3+size/4*9] |
           c64[0+size/4*10] | c64[1+size/4*10] | c64[2+size/4*10] | c64[3+size/4*10] |
           c64[0+size/4*11] | c64[1+size/4*11] | c64[2+size/4*11] | c64[3+size/4*11] |
           c64[0+size/4*12] | c64[1+size/4*12] | c64[2+size/4*12] | c64[3+size/4*12] |
           c64[0+size/4*13] | c64[1+size/4*13] | c64[2+size/4*13] | c64[3+size/4*13] |
           c64[0+size/4*14] | c64[1+size/4*14] | c64[2+size/4*14] | c64[3+size/4*14] |
           c64[0+size/4*15] | c64[1+size/4*15] | c64[2+size/4*15] | c64[3+size/4*15]);
  return dc_only4 && other3 && rest ? COEFF_DC_ONLY :
    (other3 && rest ? COEFF_4x4_ONLY : (rest ? COEFF_8x8_ONLY : COEFF_ALL));
}


/* 4x4 transform, both dimensions */
static void transform4(const int32_t *src, int32_t *dst, int bitdepth)
{
  v256 t;
  v256 add1 = v256_dup_64(1 << (bitdepth - 7));
  v256 add2 = v256_dup_64(64);
  int shift1 = bitdepth - 6;
  v128 h0, h1, h2, h3;
  v128 g0 = v128_from_128(0x0040004000400040LL); /*  64  64  64  64 */
  v128 g1 = v128_from_128(0xffadffdc00240053LL); /* -83 -36  36  83 */
  v128 g2 = v128_from_128(0x0040ffc0ffc00040LL); /*  64 -64 -64  64 */
  v128 g3 = v128_from_128(0xffdc0053ffad0024LL); /* -36  83 -83  36 */
  v128 s0 = v128_load_aligned(src + 0*4);
  v128 s1 = v128_load_aligned(src + 1*4);
  v128 s2 = v128_load_aligned(src + 2*4);
  v128 s3 = v128_load_aligned(src + 3*4);

  /* Horizontal transform */
  t = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v128_dotp_s32(s3, g0),
                                            (int32_t)v128_dotp_s32(s2, g0),
                                            (int32_t)v128_dotp_s32(s1, g0),
                                            (int32_t)v128_dotp_s32(s0, g0)), add1), shift1);
  h0 = v128_pack_s64_s32(v256_high_v128(t), v256_low_v128(t));

  t = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v128_dotp_s32(s3, g1),
                                            (int32_t)v128_dotp_s32(s2, g1),
                                            (int32_t)v128_dotp_s32(s1, g1),
                                            (int32_t)v128_dotp_s32(s0, g1)), add1), shift1);
  h1 = v128_pack_s64_s32(v256_high_v128(t), v256_low_v128(t));

  t = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v128_dotp_s32(s3, g2),
                                            (int32_t)v128_dotp_s32(s2, g2),
                                            (int32_t)v128_dotp_s32(s1, g2),
                                            (int32_t)v128_dotp_s32(s0, g2)), add1), shift1);
  h2 = v128_pack_s64_s32(v256_high_v128(t), v256_low_v128(t));

  t = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v128_dotp_s32(s3, g3),
                                            (int32_t)v128_dotp_s32(s2, g3),
                                            (int32_t)v128_dotp_s32(s1, g3),
                                            (int32_t)v128_dotp_s32(s0, g3)), add1), shift1);
  h3 = v128_pack_s64_s32(v256_high_v128(t), v256_low_v128(t));

  /* Vertical transform */
  t = v256_shr_n_s64(v256_add_64(v256_from_v64((int32_t)v128_dotp_s32(h3, g0),
                                              (int32_t)v128_dotp_s32(h2, g0),
                                              (int32_t)v128_dotp_s32(h1, g0),
                                              (int32_t)v128_dotp_s32(h0, g0)), add2), 7);
  v128_store_aligned(dst +  0, v128_pack_s64_s32(v256_high_v128(t), v256_low_v128(t)));

  t = v256_shr_n_s64(v256_add_64(v256_from_v64((int32_t)v128_dotp_s32(h3, g1),
                                              (int32_t)v128_dotp_s32(h2, g1),
                                              (int32_t)v128_dotp_s32(h1, g1),
                                              (int32_t)v128_dotp_s32(h0, g1)), add2), 7);
  v128_store_aligned(dst +  4, v128_pack_s64_s32(v256_high_v128(t), v256_low_v128(t)));

  t = v256_shr_n_s64(v256_add_64(v256_from_v64((int32_t)v128_dotp_s32(h3, g2),
                                              (int32_t)v128_dotp_s32(h2, g2),
                                              (int32_t)v128_dotp_s32(h1, g2),
                                              (int32_t)v128_dotp_s32(h0, g2)), add2), 7);
  v128_store_aligned(dst +  8, v128_pack_s64_s32(v256_high_v128(t), v256_low_v128(t)));

  t = v256_shr_n_s64(v256_add_64(v256_from_v64((int32_t)v128_dotp_s32(h3, g3),
                                              (int32_t)v128_dotp_s32(h2, g3),
                                              (int32_t)v128_dotp_s32(h1, g3),
                                              (int32_t)v128_dotp_s32(h0, g3)), add2), 7);
  v128_store_aligned(dst + 12, v128_pack_s64_s32(v256_high_v128(t), v256_low_v128(t)));
}


static void inverse_transform4(const int32_t *coeff, int32_t *block, int bitdepth) {
  v256 round1 = v256_dup_64(64);
  v256 round2 = v256_dup_64(1 << (19 - bitdepth));
  int shift2 = 20 - bitdepth;
  v128 c83 = v128_dup_32(83);
  v128 c36 = v128_dup_32(36);
  v128 load0 = v128_load_aligned(coeff +  0);
  v128 load1 = v128_load_aligned(coeff +  4);
  v128 load2 = v128_load_aligned(coeff +  8);
  v128 load3 = v128_load_aligned(coeff + 12);

  /* Utilizing symmetry properties to the maximum to minimise the number of multiplications */
  v256 o0 = v256_add_64(v256_mul_s32(load1, c83), v256_mul_s32(load3, c36));
  v256 o1 = v256_sub_64(v256_mul_s32(load1, c36), v256_mul_s32(load3, c83));
  v256 t1 = v256_shl_n_64(v256_unpack_s32_s64(load0), 6);
  v256 t2 = v256_shl_n_64(v256_unpack_s32_s64(load2), 6);
  v256 e0 = v256_add_64(v256_add_64(t1, t2), round1);
  v256 e1 = v256_add_64(v256_sub_64(t1, t2), round1);

  /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
  v256 d0 = v256_shr_n_s64(v256_add_64(e0, o0), 7);
  v256 d1 = v256_shr_n_s64(v256_add_64(e1, o1), 7);
  v256 d2 = v256_shr_n_s64(v256_sub_64(e1, o1), 7);
  v256 d3 = v256_shr_n_s64(v256_sub_64(e0, o0), 7);

  v256 x0 = v256_ziplo_64(d1, d0);
  v256 x1 = v256_ziphi_64(d1, d0);
  v256 x2 = v256_ziplo_64(d3, d2);
  v256 x3 = v256_ziphi_64(d3, d2);

  load0 = v128_pack_s64_s32(v256_low_v128(x2), v256_low_v128(x0));
  load1 = v128_pack_s64_s32(v256_high_v128(x2), v256_high_v128(x0));
  load2 = v128_pack_s64_s32(v256_low_v128(x3), v256_low_v128(x1));
  load3 = v128_pack_s64_s32(v256_high_v128(x3), v256_high_v128(x1));

  o0 = v256_add_64(v256_mul_s32(load1, c83), v256_mul_s32(load3, c36));
  o1 = v256_sub_64(v256_mul_s32(load1, c36), v256_mul_s32(load3, c83));

  t1 = v256_shl_n_64(v256_unpack_s32_s64(load0), 6);
  t2 = v256_shl_n_64(v256_unpack_s32_s64(load2), 6);
  e0 = v256_add_64(t1, t2);
  e1 = v256_sub_64(t1, t2);
  e0 = v256_add_64(e0, round2);
  e1 = v256_add_64(e1, round2);

  d0 = v256_shr_s64(v256_add_64(e0, o0), shift2);
  d1 = v256_shr_s64(v256_add_64(e1, o1), shift2);
  d2 = v256_shr_s64(v256_sub_64(e1, o1), shift2);
  d3 = v256_shr_s64(v256_sub_64(e0, o0), shift2);

  x0 = v256_ziplo_64(d1, d0);
  x1 = v256_ziphi_64(d1, d0);
  x2 = v256_ziplo_64(d3, d2);
  x3 = v256_ziphi_64(d3, d2);

  v128_store_aligned(block +  0, v128_pack_s64_s32(v256_low_v128(x2), v256_low_v128(x0)));
  v128_store_aligned(block +  4, v128_pack_s64_s32(v256_high_v128(x2), v256_high_v128(x0)));
  v128_store_aligned(block +  8, v128_pack_s64_s32(v256_low_v128(x3), v256_low_v128(x1)));
  v128_store_aligned(block + 12, v128_pack_s64_s32(v256_high_v128(x3), v256_high_v128(x1)));
}

static void inverse_transform8_4x4(const int32_t *coeff, int32_t *block, int bitdepth) {
  v256 t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
  v256 round = v256_dup_64(64);
  int shift2 = 20 - bitdepth;
  v256 c0  = v256_dup_64(  83 << 16  |   64);
  v256 c1  = v256_dup_64(-(36 << 16) |   64);
  v256 c2  = v256_dup_64(-(83 << 16) |   64);
  v256 c3  = v256_dup_64(  36 << 16  |   64);
  v256 c4  = v256_dup_64(  18 << 16  | (-75 & 0xffff));
  v256 c5  = v256_dup_64(  50 << 16  | (-18 & 0xffff));
  v256 c6  = v256_dup_64(-(89 << 16) |   50);
  v256 c7  = v256_dup_64(  75 << 16  |   89);
  t0 = v256_from_v128(v128_load_aligned(coeff +  8), v128_load_aligned(coeff +  0));
  t1 = v256_from_v128(v128_load_aligned(coeff + 24), v128_load_aligned(coeff + 16));
  t2 = v256_ziplo_32(t1, t0);
  t3 = v256_ziphi_32(t1, t0);

  t4 = v256_madd_s32(c0, t2);
  t5 = v256_madd_s32(c3, t2);
  t6 = v256_madd_s32(c4, t3);
  t7 = v256_madd_s32(c7, t3);
  t0 = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64(t5, t6), round), 7),
                         v256_shr_n_s64(v256_add_64(v256_add_64(t4, t7), round), 7));
  t1 = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64(t4, t7), round), 7),
                         v256_shr_n_s64(v256_add_64(v256_add_64(t5, t6), round), 7));

  t4 = v256_madd_s32(c1, t2);
  t5 = v256_madd_s32(c2, t2);
  t6 = v256_madd_s32(c5, t3);
  t7 = v256_madd_s32(c6, t3);
  t2 = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64(t5, t6), round), 7),
                         v256_shr_n_s64(v256_add_64(v256_add_64(t4, t7), round), 7));
  t3 = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64(t4, t7), round), 7),
                         v256_shr_n_s64(v256_add_64(v256_add_64(t5, t6), round), 7));

  t4 = v256_ziplo_128(t2, t0);
  t5 = v256_ziphi_128(t2, t0);
  t6 = v256_ziplo_128(t1, t3);
  t7 = v256_ziphi_128(t1, t3);

  t0 = v256_ziplo_32(t5, t4);
  t1 = v256_ziphi_32(t5, t4);
  t2 = v256_ziplo_32(t7, t6);
  t3 = v256_ziphi_32(t7, t6);

  t4 = v256_ziplo_64(t1, t0);
  t5 = v256_ziphi_64(t1, t0);
  t6 = v256_ziplo_64(t3, t2);
  t7 = v256_ziphi_64(t3, t2);

  t0 = v256_ziplo_128(t6, t4);
  t1 = v256_ziphi_128(t6, t4);
  t2 = v256_ziplo_128(t7, t5);
  t3 = v256_ziphi_128(t7, t5);

  t10 = v256_ziplo_32(t2, t0);
  t4  = v256_ziphi_32(t2, t0);
  t11 = v256_ziplo_32(t3, t1);
  t12 = v256_ziphi_32(t3, t1);

  round =  v256_dup_64(1 << (19 - bitdepth));

  t8 = v256_madd_s32(c0, t10);
  t9 = v256_madd_s32(c0, t4);
  t3 = v256_madd_s32(c7, t11);
  t7 = v256_madd_s32(c7, t12);
  t0 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64(t9, t7), round), shift2),
                         v256_shr_s64(v256_add_64(v256_add_64(t8, t3), round), shift2));
  t7 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64(t9, t7), round), shift2),
                         v256_shr_s64(v256_add_64(v256_sub_64(t8, t3), round), shift2));

  t8 = v256_madd_s32(c3, t10);
  t9 = v256_madd_s32(c3, t4);
  t3 = v256_madd_s32(c4, t11);
  t6 = v256_madd_s32(c4, t12);
  t1 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64(t9, t6), round), shift2),
                         v256_shr_s64(v256_add_64(v256_sub_64(t8, t3), round), shift2));
  t6 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64(t9, t6), round), shift2),
                         v256_shr_s64(v256_add_64(v256_add_64(t8, t3), round), shift2));

  t8 = v256_madd_s32(c1, t10);
  t9 = v256_madd_s32(c1, t4);
  t3 = v256_madd_s32(c6, t11);
  t5 = v256_madd_s32(c6, t12);
  t2 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64(t9, t5), round), shift2),
                         v256_shr_s64(v256_add_64(v256_add_64(t8, t3), round), shift2));
  t5 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64(t9, t5), round), shift2),
                         v256_shr_s64(v256_add_64(v256_sub_64(t8, t3), round), shift2));

  t8  = v256_madd_s32(c2, t10);
  t9  = v256_madd_s32(c2, t4);
  t10 = v256_madd_s32(c5, t11);
  t4  = v256_madd_s32(c5, t12);
  t3 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64(t9, t4), round), shift2),
                         v256_shr_s64(v256_add_64(v256_sub_64(t8, t10), round), shift2));
  t4 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64(t9, t4), round), shift2),
                         v256_shr_s64(v256_add_64(v256_add_64(t8, t10), round), shift2));



  /* Transpose */
  t8 = v256_ziplo_32(t1, t0);
  t9 = v256_ziplo_32(t3, t2);
  t10 = v256_ziplo_64(t9, t8);
  t11 = v256_ziphi_64(t9, t8);
  t8 = v256_ziphi_32(t1, t0);
  t0 = v256_ziphi_32(t3, t2);
  t9 = v256_ziplo_64(t0, t8);
  t8 = v256_ziphi_64(t0, t8);
  t2 = v256_ziplo_32(t5, t4);
  t3 = v256_ziplo_32(t7, t6);
  t0 = v256_ziplo_64(t3, t2);
  v256_store_aligned(block +  0, v256_ziplo_128(t0, t10));
  v256_store_aligned(block +  8, v256_ziphi_128(t0, t10));
  t0 = v256_ziphi_64(t3, t2);
  v256_store_aligned(block + 16, v256_ziplo_128(t0, t11));
  v256_store_aligned(block + 24, v256_ziphi_128(t0, t11));
  t0 = v256_ziphi_32(t5, t4);
  t1 = v256_ziphi_32(t7, t6);
  t2 = v256_ziplo_64(t1, t0);
  v256_store_aligned(block + 32, v256_ziplo_128(t2, t9));
  v256_store_aligned(block + 40, v256_ziphi_128(t2, t9));
  t0 = v256_ziphi_64(t1, t0);
  v256_store_aligned(block + 48, v256_ziplo_128(t0, t8));
  v256_store_aligned(block + 56, v256_ziphi_128(t0, t8));
}


/* Inverse transform, take advantage of symmetries to minimise operations */
static void inverse_transform8(const int32_t *coeff, int32_t *block, int bitdepth) {
  v256 t0, t1, t2, t3, t4 ,t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16;
  v256 round = v256_dup_64(64);
  int shift2 = 20 - bitdepth;
  v256 c0  = v256_dup_32(64);
  v256 c1  = v256_dup_64(-(64 << 16) |   64);
  v256 c2  = v256_dup_64(  36 << 16  |   83);
  v256 c3  = v256_dup_64(  83 << 16  | (-36 & 0xffff));
  v256 c4  = v256_dup_64(  18 << 16  | (-75 & 0xffff));
  v256 c5  = v256_dup_64(  50 << 16  | (-18 & 0xffff));
  v256 c6  = v256_dup_64(-(89 << 16) |   50);
  v256 c7  = v256_dup_64(  75 << 16  |   89);
  v256 c8  = v256_dup_64(  50 << 16  |   89);
  v256 c9  = v256_dup_64(  89 << 16  | (-75 & 0xffff));
  v256 c10 = v256_dup_64(  75 << 16  |   18);
  v256 c11 = v256_dup_64(  18 << 16  |   50);

  v256 load0 = v256_load_aligned(coeff +  0);
  v256 load1 = v256_load_aligned(coeff +  8);
  v256 load2 = v256_load_aligned(coeff + 16);
  v256 load3 = v256_load_aligned(coeff + 24);
  v256 load4 = v256_load_aligned(coeff + 32);
  v256 load5 = v256_load_aligned(coeff + 40);
  v256 load6 = v256_load_aligned(coeff + 48);
  v256 load7 = v256_load_aligned(coeff + 56);

  t0 = v256_ziplo_32(load4, load0);
  t1 = v256_ziplo_32(load6, load2);
  t2 = v256_madd_s32(c0, t0);
  t3 = v256_madd_s32(c2, t1);
  t4 = v256_add_64(t2, t3);
  t3 = v256_sub_64(t2, t3);

  t2 = v256_ziphi_32(load4, load0);
  t5 = v256_ziphi_32(load6, load2);
  t6 = v256_madd_s32(c0, t2);
  t7 = v256_madd_s32(c2, t5);
  t8 = v256_add_64(t6, t7);
  t6 = v256_sub_64(t6, t7);

  t0 = v256_madd_s32(c1, t0);
  t1 = v256_madd_s32(c3, t1);
  t9 = v256_add_64(t0, t1);
  t10 = v256_sub_64(t0, t1);

  t0 = v256_madd_s32(c1, t2);
  t1 = v256_madd_s32(c3, t5);
  t2 = v256_add_64(t0, t1);
  t5 = v256_sub_64(t0, t1);

  t0 = v256_ziplo_32(load3, load1);
  t1 = v256_ziplo_32(load7, load5);
  t7 = v256_ziphi_32(load3, load1);
  t11 = v256_ziphi_32(load7, load5);

  t12 = v256_add_64(v256_madd_s32(c4, t0), v256_madd_s32(c8, t1));
  t13 = v256_add_64(v256_madd_s32(c4, t7), v256_madd_s32(c8, t11));
  t14 = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64( t5, t13), round), 7),
                          v256_shr_n_s64(v256_add_64(v256_sub_64(t10, t12), round), 7));
  t13 = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_add_64( t5, t13), round), 7),
                          v256_shr_n_s64(v256_add_64(v256_add_64(t10, t12), round), 7));

  t12 = v256_add_64(v256_madd_s32(c5, t0), v256_madd_s32(c9, t1));
  t10 = v256_add_64(v256_madd_s32(c5, t7), v256_madd_s32(c9, t11));
  t5  = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64(t6, t10), round), 7),
                          v256_shr_n_s64(v256_add_64(v256_sub_64(t3, t12), round), 7));
  t10 = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_add_64(t6, t10), round), 7),
                          v256_shr_n_s64(v256_add_64(v256_add_64(t3, t12), round), 7));

  t12 = v256_add_64(v256_madd_s32(c6, t0), v256_madd_s32(c10, t1));
  t15 = v256_add_64(v256_madd_s32(c6, t7), v256_madd_s32(c10, t11));
  t3  = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_add_64(t2, t15), round), 7),
                          v256_shr_n_s64(v256_add_64(v256_add_64(t9, t12), round), 7));
  t6  = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64(t2, t15), round), 7),
                          v256_shr_n_s64(v256_add_64(v256_sub_64(t9, t12), round), 7));

  t12 = v256_add_64(v256_madd_s32(c7, t0), v256_madd_s32(c11, t1));
  t9  = v256_add_64(v256_madd_s32(c7, t7), v256_madd_s32(c11, t11));
  t2  = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_add_64(t8,  t9), round), 7),
                          v256_shr_n_s64(v256_add_64(v256_add_64(t4, t12), round), 7));
  t9  = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64(t8,  t9), round), 7),
                          v256_shr_n_s64(v256_add_64(v256_sub_64(t4, t12), round), 7));

  t0 = v256_ziplo_32(t14, t2);
  t1 = v256_ziphi_32(t14, t2);

  t2 = v256_ziplo_32(t5, t3);
  t3 = v256_ziphi_32(t5, t3);
  t4 = v256_ziplo_32(t6, t10);
  t5 = v256_ziphi_32(t6, t10);
  t6 = v256_ziplo_32(t9, t13);
  t7 = v256_ziphi_32(t9, t13);

  t8 = v256_ziphi_128(t2, t0);
  t9 = v256_ziplo_128(t2, t0);
  t0 = v256_unziplo_64(t8, t9);
  t2 = v256_unziphi_64(t8, t9);

  t8 = v256_ziphi_128(t3, t1);
  t9 = v256_ziplo_128(t3, t1);
  t1 = v256_unziplo_64(t8, t9);
  t3 = v256_unziphi_64(t8, t9);

  t8 = v256_ziphi_128(t6, t4);
  t9 = v256_ziplo_128(t6, t4);
  t4 = v256_unziplo_64(t8, t9);
  t6 = v256_unziphi_64(t8, t9);

  t8 = v256_ziphi_128(t7, t5);
  t9 = v256_ziplo_128(t7, t5);
  t5 = v256_unziplo_64(t8, t9);
  t7 = v256_unziphi_64(t8, t9);

  t8 = v256_ziplo_128(t2, t0);
  t0 = v256_ziphi_128(t2, t0);
  t2 = v256_ziplo_128(t3, t1);
  t1 = v256_ziphi_128(t3, t1);
  t3 = v256_ziplo_128(t6, t4);
  t4 = v256_ziphi_128(t6, t4);
  t6 = v256_ziplo_128(t7, t5);
  t7 = v256_ziphi_128(t7, t5);
  t9 = v256_ziplo_128(t3, t8);
  t10 = v256_ziplo_128(t4, t0);
  t11 = v256_ziplo_128(t6, t2);
  t12 = v256_ziplo_128(t7, t1);
  t13 = v256_ziphi_32(t11, t9);
  t14 = v256_ziphi_32(t12, t10);

  t0 = v256_ziphi_128(t4, t0);
  t1 = v256_ziphi_128(t7, t1);
  t2 = v256_ziphi_128(t6, t2);
  t3 = v256_ziphi_128(t3, t8);
  t4 = v256_ziphi_32(t0, t3);
  t5 = v256_ziphi_32(t1, t2);

  t6 = v256_madd_s32(c0, t13);
  t7 = v256_madd_s32(c2, t14);
  t8 = v256_add_64(t6, t7);
  t6 = v256_sub_64(t6, t7);
  t7 = v256_madd_s32(c1, t13);
  t13 = v256_madd_s32(c3, t14);
  t14 = v256_add_64(t7, t13);
  t7 = v256_sub_64(t7, t13);

  t9 = v256_ziplo_32(t11, t9);
  t10 = v256_ziplo_32(t12, t10);
  t11 = v256_madd_s32(c0, t9);
  t12 = v256_madd_s32(c2, t10);
  t13 = v256_add_64(t11, t12);
  t12 = v256_sub_64(t11, t12);
  t9 = v256_madd_s32(c1, t9);
  t10 = v256_madd_s32(c3, t10);
  t11 = v256_add_64(t9, t10);
  t16 = v256_sub_64(t9, t10);

  t9 = v256_ziplo_32(t0, t3);
  t10 = v256_ziplo_32(t1, t2);

  t2 = v256_add_64(v256_madd_s32(c4, t4), v256_madd_s32(c8, t5));
  t3 = v256_add_64(v256_madd_s32(c5, t4), v256_madd_s32(c9, t5));
  t15 = v256_add_64(v256_madd_s32(c6, t4), v256_madd_s32(c10, t5));
  t5 = v256_add_64(v256_madd_s32(c7, t4), v256_madd_s32(c11, t5));

  t0 = v256_add_64(v256_madd_s32(c4, t9), v256_madd_s32(c8, t10));
  t1 = v256_add_64(v256_madd_s32(c5, t9), v256_madd_s32(c9, t10));
  t4 = v256_add_64(v256_madd_s32(c6, t9), v256_madd_s32(c10, t10));
  t10 = v256_add_64(v256_madd_s32(c7, t9), v256_madd_s32(c11, t10));

  /* Get transposed results */
  round = v256_dup_64(1 << (19 - bitdepth));
  load0 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64( t8,  t5), round), shift2),
                            v256_shr_s64(v256_add_64(v256_add_64(t13, t10), round), shift2));
  load1 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64( t7,  t2), round), shift2),
                            v256_shr_s64(v256_add_64(v256_sub_64(t16,  t0), round), shift2));
  load2 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64(t14, t15), round), shift2),
                            v256_shr_s64(v256_add_64(v256_add_64(t11,  t4), round), shift2));
  load3 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64( t6,  t3), round), shift2),
                            v256_shr_s64(v256_add_64(v256_sub_64(t12,  t1), round), shift2));
  load4 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64( t6,  t3), round), shift2),
                            v256_shr_s64(v256_add_64(v256_add_64(t12,  t1), round), shift2));
  load5 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64(t14, t15), round), shift2),
                            v256_shr_s64(v256_add_64(v256_sub_64(t11,  t4), round), shift2));
  load6 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64( t7,  t2), round), shift2),
                            v256_shr_s64(v256_add_64(v256_add_64(t16,  t0), round), shift2));
  load7 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64( t8,  t5), round), shift2),
                            v256_shr_s64(v256_add_64(v256_sub_64(t13, t10), round), shift2));

  /* Transpose */
  t0 = v256_ziplo_32(load1, load0);
  t1 = v256_ziplo_32(load3, load2);
  t2 = v256_ziplo_64(t1, t0);
  t3 = v256_ziphi_64(t1, t0);
  t0 = v256_ziphi_32(load1, load0);
  t1 = v256_ziphi_32(load3, load2);
  t4 = v256_ziplo_64(t1, t0);
  t5 = v256_ziphi_64(t1, t0);
  t0 = v256_ziplo_32(load5, load4);
  t1 = v256_ziplo_32(load7, load6);
  t6 = v256_ziplo_64(t1, t0);
  v256_store_aligned(block +  0, v256_ziplo_128(t6, t2));
  v256_store_aligned(block +  8, v256_ziphi_128(t6, t2));
  t7 = v256_ziphi_64(t1, t0);
  v256_store_aligned(block + 16, v256_ziplo_128(t7, t3));
  v256_store_aligned(block + 24, v256_ziphi_128(t7, t3));
  t0 = v256_ziphi_32(load5, load4);
  t1 = v256_ziphi_32(load7, load6);
  t2 = v256_ziplo_64(t1, t0);
  v256_store_aligned(block + 32, v256_ziplo_128(t2, t4));
  v256_store_aligned(block + 40, v256_ziphi_128(t2, t4));
  t3 = v256_ziphi_64(t1, t0);
  v256_store_aligned(block + 48, v256_ziplo_128(t3, t5));
  v256_store_aligned(block + 56, v256_ziphi_128(t3, t5));
}

static void inverse_transform16(const int32_t *src, int32_t *dst, int shift) {
  int j;
  v128 c9 = v128_dup_32(9);
  v128 c25 = v128_dup_32(25);
  v128 c43 = v128_dup_32(43);
  v128 c57 = v128_dup_32(57);
  v128 c70 = v128_dup_32(70);
  v128 c80 = v128_dup_32(80);
  v128 c87 = v128_dup_32(87);
  v128 c90 = v128_dup_32(90);

  v128 c50 = v128_dup_32(50);
  v128 c18 = v128_dup_32(18);
  v128 c64 = v128_dup_32(64);
  v128 c36 = v128_dup_32(36);
  v128 c83 = v128_dup_32(83);
  v128 c89 = v128_dup_32(89);
  v128 c75 = v128_dup_32(75);

  for (j = 0; j < 16; j += 4) {
    v256 E0, E1, E2, E3, E4, E5, E6, E7;
    v256 O0, O1, O2, O3, O4, O5, O6, O7;
    v256 d0, d1, d2, d3, d4, d5, d6, d7;
    v256 o0, o1, o2, o3, o4, o5, o6, o7;
    v256 EO0, EO1, EO2, EO3, EEO0, EEO1, EEE0, EEE1;
    v256 EE0, EE1, EE2, EE3;
    v256 round, tmp, tmp1, tmp2;
    v128 s0 =  v128_load_aligned(src +   0);
    v128 s1 =  v128_load_aligned(src +  16);
    v128 s2 =  v128_load_aligned(src +  32);
    v128 s3 =  v128_load_aligned(src +  48);
    v128 s4 =  v128_load_aligned(src +  64);
    v128 s5 =  v128_load_aligned(src +  80);
    v128 s6 =  v128_load_aligned(src +  96);
    v128 s7 =  v128_load_aligned(src + 112);
    v128 s8 =  v128_load_aligned(src + 128);
    v128 s9 =  v128_load_aligned(src + 144);
    v128 s10 = v128_load_aligned(src + 160);
    v128 s11 = v128_load_aligned(src + 176);
    v128 s12 = v128_load_aligned(src + 192);
    v128 s13 = v128_load_aligned(src + 208);
    v128 s14 = v128_load_aligned(src + 224);
    v128 s15 = v128_load_aligned(src + 240);

    /* O0 = 90*src[16] + 87*src[3*16] + 80*src[5*16] + 70*src[7*16]
      + 57*src[9*16] + 43*src[11*16] + 25*src[13*16] + 9*src[15*16];*/
    O0   = v256_add_64(v256_mul_s32(s1, c90),v256_mul_s32(s3, c87));
    tmp  = v256_add_64(v256_mul_s32(s5, c80),v256_mul_s32(s7, c70));
    tmp1 = v256_add_64(v256_mul_s32(s9, c57),v256_mul_s32(s11, c43));
    tmp2 = v256_add_64(v256_mul_s32(s13, c25),v256_mul_s32(s15, c9));
    O0   = v256_add_64(O0, v256_add_64(tmp, v256_add_64(tmp1, tmp2)));

    /* O1 = 87*src[16] + 57*src[3*16] + 9*src[5*16] - 43*src[7*16]
      - 80*src[9*16] -  90*src[11*16] -  70*src[13*16] - 25*src[15*16];*/

    O1   = v256_add_64(v256_mul_s32(s1, c87),  v256_mul_s32(s3, c57));
    tmp  = v256_sub_64(v256_mul_s32(s5, c9),   v256_mul_s32(s7, c43));
    tmp1 = v256_add_64(v256_mul_s32(s9, c80),  v256_mul_s32(s11, c90));
    tmp2 = v256_add_64(v256_mul_s32(s13 ,c70), v256_mul_s32(s15, c25));
    O1   = v256_add_64(O1, tmp);
    tmp1 = v256_add_64(tmp1, tmp2);
    O1   = v256_sub_64(O1, tmp1);

    /* O2 = 80*src[16] + 9*src[3*16] - 70*src[5*16] - 87*src[7*16]
      + (-25)*src[9 * 16] + 57*src[11 * 16] + 90*src[13 * 16] + 43*src[15 * 16] */
    O2   = v256_add_64(v256_mul_s32(s1, c80), v256_mul_s32(s3, c9));
    tmp  =     v256_add_64(v256_mul_s32(s5, c70), v256_mul_s32(s7, c87));
    tmp1 =    v256_sub_64(v256_mul_s32(s11, c57) ,v256_mul_s32(s9, c25));
    tmp2 = v256_add_64(v256_mul_s32(s13, c90), v256_mul_s32(s15, c43));
    O2   = v256_sub_64(O2, tmp);
    O2   = v256_add_64(O2, v256_add_64(tmp1, tmp2));

    /* O3 = 70*src[16] - 43*src[3*16] - 87*src[5*16] + 9*src[7*16]
      + 90 * src[9 * 16] + 25 *src[11 * 16] - 80 * src[13 * 16] - 57* src[15 * 16]*/
    O3   = v256_sub_64(v256_mul_s32(s1, c70), v256_mul_s32(s3, c43));
    tmp  = v256_sub_64(v256_mul_s32(s5, c87), v256_mul_s32(s7, c9));
    O3   = v256_sub_64(O3, tmp);
    tmp1 = v256_add_64(v256_mul_s32(s9, c90), v256_mul_s32(s11, c25));
    tmp2 = v256_add_64(v256_mul_s32(s13, c80), v256_mul_s32(s15, c57));
    O3   = v256_add_64(O3, v256_sub_64(tmp1, tmp2));

    /* O4 = 57*src[16] - 80*src[3*16] - 25*src[5*16] + 90*src[7*16]
      = - 9 * src[9 * 16] -87 * src[11 * 16] + 43 * src[13 * 16] + 70 * src[15 * 16]*/
    O4   = v256_sub_64(v256_mul_s32(s1, c57), v256_mul_s32(s3, c80));
    tmp  = v256_sub_64(v256_mul_s32(s5, c25), v256_mul_s32(s7, c90));
    O4   = v256_sub_64(O4, tmp);
    tmp1 = v256_add_64(v256_mul_s32(s9, c9),v256_mul_s32(s11, c87));
    tmp2 = v256_add_64(v256_mul_s32(s13, c43),v256_mul_s32(s15, c70));
    O4   = v256_add_64(O4, tmp2);
    O4   = v256_sub_64(O4, tmp1);

    /* O5 = 43*src[16] - 90*src[3*16] + 57*src[5*16] + 25*src[7*16]
      - 87 * src[9 * 16] + 70 * src[11 * 16] + 9 * src[13 * 16] - 80 * src[15 * 16]*/
    O5   = v256_sub_64(v256_mul_s32(s1, c43), v256_mul_s32(s3, c90));
    tmp  = v256_add_64(v256_mul_s32(s5, c57), v256_mul_s32(s7, c25));
    O5   = v256_add_64(O5, tmp);
    tmp1 = v256_sub_64(v256_mul_s32(s11, c70), v256_mul_s32(s9, c87));
    tmp2 = v256_sub_64(v256_mul_s32(s13, c9), v256_mul_s32(s15, c80));
    O5   = v256_add_64(O5, v256_add_64(tmp1, tmp2));

    /* O6 = 25*src[16] - 70*src[3*16] + 90*src[5*16] - 80*src[7*16]
      + 43* src[9 * 16] + 9 * src[11 * 16] -57 * src[13 * 16] + 87 * src[15 * 16]*/
    O6   = v256_sub_64(v256_mul_s32(s1, c25), v256_mul_s32(s3, c70));
    tmp  = v256_sub_64(v256_mul_s32(s5, c90), v256_mul_s32(s7, c80));
    O6   = v256_add_64(O6, tmp);
    tmp1 = v256_add_64(v256_mul_s32(s9, c43), v256_mul_s32(s11, c9));
    tmp2 = v256_sub_64(v256_mul_s32(s15, c87), v256_mul_s32(s13, c57));
    O6   = v256_add_64(O6, v256_add_64(tmp1, tmp2));

    /* O7 = 9*src[16] - 25*src[3*16] + 43*src[5*16] - 57*src[7*16]
      + 70 * src[9 * 16] -80 * src[11 * 16] + 87 * src[13 * 16] - 90 * src[15 * 16]*/
    O7   = v256_sub_64(v256_mul_s32(s1, c9), v256_mul_s32(s3, c25));
    tmp  = v256_sub_64(v256_mul_s32(s5, c43), v256_mul_s32(s7, c57));
    O7   = v256_add_64(O7, tmp);
    tmp1 = v256_sub_64(v256_mul_s32(s9, c70), v256_mul_s32(s11, c80));
    tmp2 = v256_sub_64(v256_mul_s32(s13, c87), v256_mul_s32(s15, c90));
    O7   = v256_add_64(O7, v256_add_64(tmp1, tmp2));

    /* EO[0] = g3mat[ 2][0]*src[ 2*16] + g3mat[ 6][0]*src[ 6*16]
      + g3mat[10][0]*src[10*16] + g3mat[14][0]*src[14*16];*/
    EO0 = v256_add_64(v256_mul_s32(s2, c89), v256_mul_s32(s6, c75));
    tmp1 = v256_add_64(v256_mul_s32(s10, c50), v256_mul_s32(s14, c18));
    EO0 = v256_add_64(EO0, tmp1);

    /* EO[1] = g3mat[ 2][1]*src[ 2*16] + g3mat[ 6][1]*src[ 6*16]
      + g3mat[10][1]*src[10*16] + g3mat[14][1]*src[14*16];*/
    EO1 = v256_sub_64(v256_mul_s32(s2, c75), v256_mul_s32(s6, c18));
    tmp1 = v256_add_64(v256_mul_s32(s10, c89), v256_mul_s32(s14, c50));
    EO1 = v256_sub_64(EO1, tmp1);

    /* EO[2] = g3mat[ 2][2]*src[ 2*16] + g3mat[ 6][2]*src[ 6*16]
      + g3mat[10][2]*src[10*16] + g3mat[14][2]*src[14*16];*/
    EO2 = v256_sub_64(v256_mul_s32(s2, c50), v256_mul_s32(s6, c89));
    tmp1 = v256_add_64(v256_mul_s32(s10, c18), v256_mul_s32(s14, c75));
    EO2 = v256_add_64(EO2, tmp1);

    /* EO[3] = g3mat[ 2][3]*src[ 2*16] + g3mat[ 6][3]*src[ 6*16]
      + g3mat[10][3]*src[10*16] + g3mat[14][3]*src[14*16];*/
    EO3 = v256_sub_64(v256_mul_s32(s2, c18), v256_mul_s32(s6, c50));
    tmp1 = v256_sub_64(v256_mul_s32(s10, c75), v256_mul_s32(s14, c89));
    EO3 = v256_add_64(EO3, tmp1);

    /* ee0 */
    /* EEO[0] = g3mat[4][0]*src[ 4*16 ] + g3mat[12][0]*src[ 12*16 ];*/
    EEO0 = v256_add_64(v256_mul_s32(s4, c83), v256_mul_s32(s12, c36));
    /* EEE[0] = g3mat[0][0]*src[ 0      ] + g3mat[ 8][0]*src[ 8*16  ];*/
    EEE0 = v256_add_64(v256_mul_s32(s0, c64), v256_mul_s32(s8, c64));
    /* EEO[1] = g3mat[4][1]*src[ 4*16 ] + g3mat[12][1]*src[ 12*16 ];*/
    EEO1 = v256_sub_64(v256_mul_s32(s4, c36), v256_mul_s32(s12, c83));
    /* EEE[1] = g3mat[0][1]*src[ 0      ] + g3mat[ 8][1]*src[ 8*16  ];*/
    EEE1 = v256_sub_64(v256_mul_s32(s0, c64), v256_mul_s32(s8, c64));

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EE0 = v256_add_64(EEE0, EEO0);
    EE3 = v256_sub_64(EEE0, EEO0);
    EE1 = v256_add_64(EEE1, EEO1);
    EE2 = v256_sub_64(EEE1, EEO1);

    round = v256_dup_64(1 << (shift-1));
    E0 = v256_add_64(round, v256_add_64(EE0, EO0));
    E7 = v256_add_64(round, v256_sub_64(EE0, EO0));
    E4 = v256_add_64(round, v256_sub_64(EE3, EO3));
    E3 = v256_add_64(round, v256_add_64(EE3, EO3));
    E1 = v256_add_64(round, v256_add_64(EE1, EO1));
    E6 = v256_add_64(round, v256_sub_64(EE1, EO1));
    E5 = v256_add_64(round, v256_sub_64(EE2, EO2));
    E2 = v256_add_64(round, v256_add_64(EE2, EO2));

    d0 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(E1, O1), shift),
                           v256_shr_s64(v256_add_64(E0, O0), shift));
    d1 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(E3, O3), shift),
                           v256_shr_s64(v256_add_64(E2, O2), shift));
    d2 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(E5, O5), shift),
                           v256_shr_s64(v256_add_64(E4, O4), shift));
    d3 = v256_pack_s64_s32(v256_shr_s64(v256_add_64(E7, O7), shift),
                           v256_shr_s64(v256_add_64(E6, O6), shift));
    d4 = v256_pack_s64_s32(v256_shr_s64(v256_sub_64(E6, O6), shift),
                           v256_shr_s64(v256_sub_64(E7, O7), shift));
    d5 = v256_pack_s64_s32(v256_shr_s64(v256_sub_64(E4, O4), shift),
                           v256_shr_s64(v256_sub_64(E5, O5), shift));
    d6 = v256_pack_s64_s32(v256_shr_s64(v256_sub_64(E2, O2), shift),
                           v256_shr_s64(v256_sub_64(E3, O3), shift));
    d7 = v256_pack_s64_s32(v256_shr_s64(v256_sub_64(E0, O0), shift),
                           v256_shr_s64(v256_sub_64(E1, O1), shift));

    o0 = v256_ziplo_32(d1, d0);
    o1 = v256_ziphi_32(d1, d0);
    o2 = v256_ziplo_32(d3, d2);
    o3 = v256_ziphi_32(d3, d2);
    o4 = v256_ziplo_32(d5, d4);
    o5 = v256_ziphi_32(d5, d4);
    o6 = v256_ziplo_32(d7, d6);
    o7 = v256_ziphi_32(d7, d6);

    d0 = v256_ziplo_32(o1, o0);
    d1 = v256_ziphi_32(o1, o0);
    d2 = v256_ziplo_32(o3, o2);
    d3 = v256_ziphi_32(o3, o2);
    d4 = v256_ziplo_32(o5, o4);
    d5 = v256_ziphi_32(o5, o4);
    d6 = v256_ziplo_32(o7, o6);
    d7 = v256_ziphi_32(o7, o6);

    v256_store_aligned(dst +  0, v256_from_v128(v256_low_v128(d2), v256_low_v128(d0)));
    v256_store_aligned(dst +  8, v256_from_v128(v256_low_v128(d6), v256_low_v128(d4)));
    v256_store_aligned(dst + 16, v256_from_v128(v256_high_v128(d2), v256_high_v128(d0)));
    v256_store_aligned(dst + 24, v256_from_v128(v256_high_v128(d6), v256_high_v128(d4)));
    v256_store_aligned(dst + 32, v256_from_v128(v256_low_v128(d3), v256_low_v128(d1)));
    v256_store_aligned(dst + 40, v256_from_v128(v256_low_v128(d7), v256_low_v128(d5)));
    v256_store_aligned(dst + 48, v256_from_v128(v256_high_v128(d3), v256_high_v128(d1)));
    v256_store_aligned(dst + 56, v256_from_v128(v256_high_v128(d7), v256_high_v128(d5)));
    dst += 64;
    src += 4;
  }
}

/* 16x16 inverse transform assuming everything but top left 4x4 is 0 */
static void inverse_transform16_4x4(const int32_t *coeff, int32_t *block, int bitdepth) {
  static const ALIGN(16) int32_t c[] = {
     64,  89,  64,  89,  64,  89,  64,  89,
     90,  87,  90,  87,  90,  87,  90,  87,
     64,  75,  64,  75,  64,  75,  64,  75,
    -87, -57, -87, -57, -87, -57, -87, -57,
     64,  50,  64,  50,  64,  50,  64,  50,
     80,   9,  80,   9,  80,   9,  80,   9,
     64,  18,  64,  18,  64,  18,  64,  18,
    -70,  43, -70,  43, -70,  43, -70,  43,
     64, -18,  64, -18,  64, -18,  64, -18,
     57, -80,  57, -80,  57, -80,  57, -80,
     64, -50,  64, -50,  64, -50,  64, -50,
    -43,  90, -43,  90, -43,  90, -43,  90,
     64, -75,  64, -75,  64, -75,  64, -75,
     25, -70,  25, -70,  25, -70,  25, -70,
     64, -89,  64, -89,  64, -89,  64, -89,
     -9,  25,  -9,  25,  -9,  25,  -9,  25 };
  v256 t[8];
  v256 t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11;
  v256 round = v256_dup_64(64);
  int shift2 = 20 - bitdepth;
  v256 load0 = v256_from_v128(v128_load_aligned(coeff + 16), v128_load_aligned(coeff +  0));
  v256 load1 = v256_from_v128(v128_load_aligned(coeff + 48), v128_load_aligned(coeff + 32));
  v256 lo = v256_ziplo_32(load1, load0);
  v256 hi = v256_ziphi_32(load1, load0);
  int i, j;

  for (i = 0; i < 4; i++) {
    v256 t0 = v256_madd_s32(v256_load_aligned(c + i*32 +  0), lo);
    v256 t1 = v256_madd_s32(v256_load_aligned(c + i*32 +  8), hi);
    v256 t2 = v256_madd_s32(v256_load_aligned(c + i*32 + 16), lo);
    v256 t3 = v256_madd_s32(v256_load_aligned(c + i*32 + 24), hi);
    t[i*2+0] = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64(t2, t3), round), 7),
                                 v256_shr_n_s64(v256_add_64(v256_add_64(t0, t1), round), 7));
    t[i*2+1] = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_sub_64(t0, t1), round), 7),
                                 v256_shr_n_s64(v256_add_64(v256_add_64(t2, t3), round), 7));
  }

  t0 = v256_ziplo_128(t[2], t[0]);
  t1 = v256_ziphi_128(t[2], t[0]);
  t2 = v256_ziplo_32(t1, t0);
  t3 = v256_ziphi_32(t1, t0);
  t4 = v256_ziplo_64(t3, t2);
  t6 = v256_ziphi_64(t3, t2);

  t0 = v256_ziplo_128(t[6], t[4]);
  t1 = v256_ziphi_128(t[6], t[4]);
  t2 = v256_ziplo_32(t1, t0);
  t3 = v256_ziphi_32(t1, t0);
  t8 = v256_ziplo_64(t3, t2);
  t10 = v256_ziphi_64(t3, t2);

  t0 = v256_ziplo_128(t[5], t[7]);
  t1 = v256_ziphi_128(t[5], t[7]);
  t2 = v256_ziplo_32(t1, t0);
  t3 = v256_ziphi_32(t1, t0);
  t9 = v256_ziplo_64(t3, t2);
  t11 = v256_ziphi_64(t3, t2);

  t0 = v256_ziplo_128(t[1], t[3]);
  t1 = v256_ziphi_128(t[1], t[3]);
  t2 = v256_ziplo_32(t1, t0);
  t3 = v256_ziphi_32(t1, t0);
  t5 = v256_ziplo_64(t3, t2);
  t7 = v256_ziphi_64(t3, t2);

  t0 = v256_ziplo_128(t5, t9);
  t1 = v256_ziphi_128(t5, t9);
  t2 = v256_ziplo_128(t8, t4);
  t3 = v256_ziphi_128(t8, t4);
  t4 = v256_ziplo_128(t10, t6);
  t5 = v256_ziphi_128(t10, t6);
  t6 = v256_ziplo_128(t7, t11);
  t7 = v256_ziphi_128(t7, t11);

  t8 = v256_ziplo_32(t7, t1);
  t7 = v256_ziphi_32(t7, t1);
  t1 = v256_ziplo_32(t6, t0);
  t0 = v256_ziphi_32(t6, t0);
  t6 = v256_ziplo_32(t4, t2);
  t10 = v256_ziphi_32(t4, t2);
  t11 = v256_ziplo_32(t5, t3);
  t3 = v256_ziphi_32(t5, t3);

  round = v256_dup_64(1 << (19 - bitdepth));

  for (i = 0; i < 8; i++) {
    int i1 = i*16;
    int i2 = 240 - i1;
    int i3 = i & 1 ? i2 - 120 : i1;
    int i4 = i & 1 ? i1 : i2 - 120;
    int i5 = i & 1 ? i2 : i1 + 120;
    int i6 = i & 1 ? i1 + 120 : i2;
    v256 c0 = v256_load_aligned(c + i1 + 0);
    v256 c1 = v256_load_aligned(c + i1 + 8);
    v256 v0 = v256_madd_s32(c0, t10);
    v256 v1 = v256_madd_s32(c0, t6);
    v256 v2 = v256_madd_s32(c1, t11);
    v256 v3 = v256_madd_s32(c1, t3);
    v256_store_aligned(block + i3,
                       v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64(v0, v3), round), shift2),
                                         v256_shr_s64(v256_add_64(v256_add_64(v1, v2), round), shift2)));
    v256_store_aligned(block + i4,
                       v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64(v0, v3), round), shift2),
                                         v256_shr_s64(v256_add_64(v256_sub_64(v1, v2), round), shift2)));

    v0 = v256_madd_s32(c0, t0);
    v1 = v256_madd_s32(c0, t1);
    v2 = v256_madd_s32(c1, t8);
    v3 = v256_madd_s32(c1, t7);
    v256_store_aligned(block + 8 + i5,
                       v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64(v0, v3), round), shift2),
                                         v256_shr_s64(v256_add_64(v256_add_64(v1, v2), round), shift2)));
    v256_store_aligned(block + 8 + i6,
                       v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64(v0, v3), round), shift2),
                                         v256_shr_s64(v256_add_64(v256_sub_64(v1, v2), round), shift2)));
  }

  for (i = 0; i < 16; i += 8)
    for (j = 0; j < 16; j += 8) {
      int32_t *p = block + i*16 + j;
      v256 load0 = v256_load_aligned(p +   0);
      v256 load1 = v256_load_aligned(p +  16);
      v256 load2 = v256_load_aligned(p +  32);
      v256 load3 = v256_load_aligned(p +  48);
      v256 load4 = v256_load_aligned(p +  64);
      v256 load5 = v256_load_aligned(p +  80);
      v256 load6 = v256_load_aligned(p +  96);
      v256 load7 = v256_load_aligned(p + 112);
      v256 t3  = v256_ziplo_32(load1, load0);
      v256 t4  = v256_ziphi_32(load1, load0);
      v256 t7  = v256_ziplo_32(load3, load2);
      v256 t8  = v256_ziphi_32(load3, load2);
      v256 t11 = v256_ziplo_32(load5, load4);
      v256 t12 = v256_ziphi_32(load5, load4);
      v256 t15 = v256_ziplo_32(load7, load6);
      v256 t16 = v256_ziphi_32(load7, load6);
      v256 t17 = v256_ziplo_64(t7, t3);
      v256 t18 = v256_ziphi_64(t7, t3);
      v256 t19 = v256_ziplo_64(t8, t4);
      v256 t20 = v256_ziphi_64(t8, t4);
      v256 t21 = v256_ziplo_64(t15, t11);
      v256 t22 = v256_ziphi_64(t15, t11);
      v256 t23 = v256_ziplo_64(t16, t12);
      v256 t24 = v256_ziphi_64(t16, t12);
      v256_store_aligned(p +   0, v256_ziplo_128(t21, t17));
      v256_store_aligned(p +  16, v256_ziphi_128(t21, t17));
      v256_store_aligned(p +  32, v256_ziplo_128(t22, t18));
      v256_store_aligned(p +  48, v256_ziphi_128(t22, t18));
      v256_store_aligned(p +  64, v256_ziplo_128(t23, t19));
      v256_store_aligned(p +  80, v256_ziphi_128(t23, t19));
      v256_store_aligned(p +  96, v256_ziplo_128(t24, t20));
      v256_store_aligned(p + 112, v256_ziphi_128(t24, t20));
    }
}

static void transform_1d_32(const int32_t *coeff, const int32_t *tcoeff, int j, int32_t *out, int shift)
{
  v256 t0, t1, t2, t3, t4;
  v256 c = v256_load_aligned(tcoeff + j*8);
  v256 rev32 = v256_from_128(0x0302010007060504LL, 0x0b0a09080f0e0d0cLL);

  t0 = v256_madd_s32(v256_from_128(0x00430049004e0052LL, 0x00550058005a005aLL), c);
  t1 = v256_madd_s32(v256_from_128(0xffcaffe1fffc0016LL, 0x002e00430052005aLL), c);
  t2 = v256_madd_s32(v256_from_128(0xffb2ffa6ffaeffcaLL, 0xfff3001f00430058LL), c);
  t3 = v256_madd_s32(v256_from_128(0x0026ffeaffb7ffa6LL, 0xffbdfff3002e0055LL), c);
  t4 = v256_ziplo_64(t0, t1);
  t1 = v256_ziphi_64(t0, t1);
  t0 = v256_ziplo_64(t2, t3);
  t3 = v256_ziphi_64(t2, t3);
  v256 o0 = v256_add_64(v256_add_64(v256_ziplo_128(t4, t0), v256_ziphi_128(t4, t0)),
                        v256_add_64(v256_ziplo_128(t1, t3), v256_ziphi_128(t1, t3)));

  t0 = v256_madd_s32(v256_from_128(0x0055004e000dffc3LL, 0xffa6ffca00160052LL), c);
  t1 = v256_madd_s32(v256_from_128(0xffea00430055000dLL, 0xffb7ffaefffc004eLL), c);
  t2 = v256_madd_s32(v256_from_128(0xffa6ffda0043004eLL, 0xffeaffa6ffe10049LL), c);
  t3 = v256_madd_s32(v256_from_128(0x0004ffa6ffea0055LL, 0x0026ffb2ffca0043LL), c);
  t4 = v256_ziplo_64(t0, t1);
  t1 = v256_ziphi_64(t0, t1);
  t0 = v256_ziplo_64(t2, t3);
  t3 = v256_ziphi_64(t2, t3);
  v256 o1 = v256_add_64(v256_add_64(v256_ziplo_128(t4, t0), v256_ziphi_128(t4, t0)),
                        v256_add_64(v256_ziplo_128(t1, t3), v256_ziphi_128(t1, t3)));

  t0 = v256_madd_s32(v256_from_128(0x005afff3ffa8001fLL, 0x0052ffd2ffb7003dLL), c);
  t1 = v256_madd_s32(v256_from_128(0x000d0052ffc3ffd2LL, 0x0058fffcffab0036LL), c);
  t2 = v256_madd_s32(v256_from_128(0xffa8003d001fffa6LL, 0x00360026ffa6002eLL), c);
  t3 = v256_madd_s32(v256_from_128(0xffe1ffd2005affbdLL, 0xfffc0049ffa80026LL), c);
  t4 = v256_ziplo_64(t0, t1);
  t1 = v256_ziphi_64(t0, t1);
  t0 = v256_ziplo_64(t2, t3);
  t3 = v256_ziphi_64(t2, t3);
  v256 o2 = v256_add_64(v256_add_64(v256_ziplo_128(t4, t0), v256_ziphi_128(t4, t0)),
                        v256_add_64(v256_ziplo_128(t1, t3), v256_ziphi_128(t1, t3)));

  t0 = v256_madd_s32(v256_from_128(0x0052ffa800360004LL, 0xffc3005affb2001fLL), c);
  t1 = v256_madd_s32(v256_from_128(0x002efffcffda0049LL, 0xffa60055ffc30016LL), c);
  t2 = v256_madd_s32(v256_from_128(0xffb70055ffa60058LL, 0xffb2003dffda000dLL), c);
  t3 = v256_madd_s32(v256_from_128(0xffc30036ffd20026LL, 0xffe10016fff30004LL), c);
  t4 = v256_ziplo_64(t0, t1);
  t1 = v256_ziphi_64(t0, t1);
  t0 = v256_ziplo_64(t2, t3);
  t3 = v256_ziphi_64(t2, t3);
  v256 o3 = v256_add_64(v256_add_64(v256_ziplo_128(t4, t0), v256_ziphi_128(t4, t0)),
                        v256_add_64(v256_ziplo_128(t1, t3), v256_ziphi_128(t1, t3)));

  v128 c64 = v128_from_32(coeff[j+14*32], coeff[j+10*32], coeff[j+6*32], coeff[j+2*32]);
  c = v256_from_v128(c64, c64);
  t0 = v256_madd_s32(c, v256_from_128(0xffd5000900390057LL, 0x004600500057005aLL));
  t1 = v256_madd_s32(c, v256_from_128(0x0009ffa9ffd50046LL, 0xffa9ffba00090050LL));
  v256 oo0 = v256_unziphi_64(v256_add_64(t1, v256_shl_n_word(t1, 4)),
                             v256_add_64(t0, v256_shl_n_word(t0, 4)));
  t0 = v256_madd_s32(c, v256_from_128(0x00190039ffa6002bLL, 0x005affe7ffb00039LL));
  t1 = v256_madd_s32(c, v256_from_128(0xffc7002bffe70009LL, 0xffb0005affba0019LL));
  v256 oo1 = v256_unziphi_64(v256_add_64(t1, v256_shl_n_word(t1, 4)),
                             v256_add_64(t0, v256_shl_n_word(t0, 4)));

  v256 to = v256_madd_s32(v256_dup_64((coeff[j+12*32] << 16) + (coeff[j+4*32] & 0xffff)),
                          v256_from_128(0xffce0012ffa70032LL, 0xffee004b004b0059LL));
  int32_t ooo0 = 0x53*coeff[j+8*32];
  int32_t ooo1 = 0x24*coeff[j+8*32];
  int32_t eee = coeff[j] << 6;
  v256 te = v256_from_v64(eee - ooo0, eee - ooo1, eee + ooo1, eee + ooo0);
  v256 ee0 = v256_add_64(te, to);
  v256 ee1 = v256_pshuffle_8(v256_sub_64(te, to), rev32);

  v256 e0 = v256_add_64(ee0, oo0);
  v256 e1 = v256_add_64(ee1, oo1);
  v256 e2 = v256_sub_64(ee1, oo1);
  v256 e3 = v256_sub_64(ee0, oo0);
  v256 e0r = v256_pshuffle_8(e0, rev32);
  v256 e1r = v256_pshuffle_8(e1, rev32);
  v256 e2r = v256_pshuffle_8(e2, rev32);
  v256 e3r = v256_pshuffle_8(e3, rev32);
  v256 o0r = v256_pshuffle_8(o0, rev32);
  v256 o1r = v256_pshuffle_8(o1, rev32);
  v256 o2r = v256_pshuffle_8(o2, rev32);
  v256 o3r = v256_pshuffle_8(o3, rev32);

  c = v256_dup_64(1 << (shift-1));
  v256_store_aligned(out + 0, v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64(e1, o1r), c), shift),
                                                v256_shr_s64(v256_add_64(v256_add_64(e0, o0r), c), shift)));
  v256_store_aligned(out + 8, v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_add_64(e3r, o3r), c), shift),
                                                v256_shr_s64(v256_add_64(v256_add_64(e2r, o2r), c), shift)));
  v256_store_aligned(out + 16, v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64(e2, o2), c), shift),
                                                 v256_shr_s64(v256_add_64(v256_sub_64(e3, o3), c), shift)));
  v256_store_aligned(out + 24, v256_pack_s64_s32(v256_shr_s64(v256_add_64(v256_sub_64(e0r, o0), c), shift),
                                                 v256_shr_s64(v256_add_64(v256_sub_64(e1r, o1), c), shift)));
}


static void transpose8x8(const int32_t *src, int sstride, int32_t *dst, int dstride)
{
  v256 i0 = v256_load_aligned(src + sstride*0);
  v256 i1 = v256_load_aligned(src + sstride*1);
  v256 i2 = v256_load_aligned(src + sstride*2);
  v256 i3 = v256_load_aligned(src + sstride*3);
  v256 i4 = v256_load_aligned(src + sstride*4);
  v256 i5 = v256_load_aligned(src + sstride*5);
  v256 i6 = v256_load_aligned(src + sstride*6);
  v256 i7 = v256_load_aligned(src + sstride*7);

  v256 t0 = v256_ziplo_32(i1, i0);
  v256 t1 = v256_ziplo_32(i3, i2);
  v256 t2 = v256_ziplo_32(i5, i4);
  v256 t3 = v256_ziplo_32(i7, i6);
  v256 t4 = v256_ziphi_32(i1, i0);
  v256 t5 = v256_ziphi_32(i3, i2);
  v256 t6 = v256_ziphi_32(i5, i4);
  v256 t7 = v256_ziphi_32(i7, i6);

  i0 = v256_ziplo_64(t1, t0);
  i1 = v256_ziplo_64(t3, t2);
  i2 = v256_ziplo_64(t5, t4);
  i3 = v256_ziplo_64(t7, t6);
  i4 = v256_ziphi_64(t1, t0);
  i5 = v256_ziphi_64(t3, t2);
  i6 = v256_ziphi_64(t5, t4);
  i7 = v256_ziphi_64(t7, t6);
  v256_store_aligned(dst + dstride*0, v256_ziplo_128(i1, i0));
  v256_store_aligned(dst + dstride*1, v256_ziphi_128(i1, i0));
  v256_store_aligned(dst + dstride*2, v256_ziplo_128(i5, i4));
  v256_store_aligned(dst + dstride*3, v256_ziphi_128(i5, i4));
  v256_store_aligned(dst + dstride*4, v256_ziplo_128(i3, i2));
  v256_store_aligned(dst + dstride*5, v256_ziphi_128(i3, i2));
  v256_store_aligned(dst + dstride*6, v256_ziplo_128(i7, i6));
  v256_store_aligned(dst + dstride*7, v256_ziphi_128(i7, i6));
}

static void inverse_transform32(const int32_t * coeff, int32_t *block, int bitdepth)
{
  int32_t *tmp = thor_alloc(32*32*2, 32);
  int32_t *tcoeff = thor_alloc(8*32*2, 32);
  memset(tcoeff, 0, 8*32*2);

  /* 1st dimension */
  transpose8x8(coeff + 32, 64, tcoeff +   0, 8);
  transpose8x8(coeff + 40, 64, tcoeff +  64, 8);

  for (int i = 0; i < 16; i++)
    transform_1d_32(coeff, tcoeff, i, tmp + i*32, 7);

  /* 2nd dimension */
  transpose8x8(tmp + 32, 64, tcoeff +   0, 8);
  transpose8x8(tmp + 40, 64, tcoeff +  64, 8);
  transpose8x8(tmp + 48, 64, tcoeff + 128, 8);
  transpose8x8(tmp + 56, 64, tcoeff + 192, 8);

  for (int i = 0; i < 32; i++)
    transform_1d_32(tmp, tcoeff, i, block + i*32, 20 - bitdepth);

  thor_free(tmp);
  thor_free(tcoeff);
}

static void transform8(const int32_t *src, int32_t *dst, int shift)
{
  int j;

  const v256 shuffle1 = v256_from_128(0x0f0e0d0c0b0a0908LL, 0x0100030205040706LL);
  const v256 shuffle2 = v256_from_128(0x0d0c0f0e09080b0aLL, 0x0504070601000302LL);
  const v256 shuffle3 = v256_from_128(0x0302010007060504LL, 0x0b0a09080f0e0d0cLL);
  const v128 shuffle4 = v128_from_128(0x0100030205040706LL);
  const v256 round = v256_dup_64(1 << (shift-1));

  for (j = 0; j < 8; j += 4) {
    v128 c;
    v256 t, EO, EEE, EEO, E, E_, O, O_;

    /* E and O*/
    v256 load0 = v256_pshuffle_8(v256_load_aligned(src + 0), shuffle1);
    v256 load1 = v256_pshuffle_8(v256_load_aligned(src + 8), shuffle1);
    v256 hi = v256_ziphi_128(load0, load1);
    v256 lo = v256_ziplo_128(load0, load1);
    E = v256_add_32(lo, hi);
    O = v256_sub_32(lo, hi);
    load0 = v256_pshuffle_8(v256_load_aligned(src + 16), shuffle1);
    load1 = v256_pshuffle_8(v256_load_aligned(src + 24), shuffle1);
    hi = v256_ziphi_128(load0, load1);
    lo = v256_ziplo_128(load0, load1);
    E_ = v256_add_32(lo, hi);
    O_ = v256_sub_32(lo, hi);

    /* EO */
    EO = v256_sub_32(v256_unziphi_64(E, E_),
                     v256_unziplo_64(v256_pshuffle_8(E, shuffle2), v256_pshuffle_8(E_, shuffle2)));
    EO = v256_pshuffle_8(EO, shuffle3);

    /* EEE and EEO */
    c = v128_dup_32(64);
    EEE = v256_from_v64((int32_t)v128_dotp_s32(v256_low_v128(E_), c),
                       (int32_t)v128_dotp_s32(v256_high_v128(E_), c),
                       (int32_t)v128_dotp_s32(v256_low_v128(E), c),
                       (int32_t)v128_dotp_s32(v256_high_v128(E), c));
    c = v128_from_128(0x0040ffc0ffc00040LL);
    EEO = v256_from_v64((int32_t)v128_dotp_s32(v256_low_v128(E_), c),
                       (int32_t)v128_dotp_s32(v256_high_v128(E_), c),
                       (int32_t)v128_dotp_s32(v256_low_v128(E), c),
                       (int32_t)v128_dotp_s32(v256_high_v128(E), c));

    t = v256_shr_s64(v256_add_64(EEE, round), shift);
    v128_store_aligned(dst+0*8, v256_low_v128(v256_unziplo_32(t, t)));

    t = v256_shr_s64(v256_add_64(EEO, round), shift);
    v128_store_aligned(dst+4*8, v256_low_v128(v256_unziplo_32(t, t)));

    t = v256_shr_s64(v256_add_64(v256_madd_s32(v256_dup_64((83 << 16) | (uint32_t)36), EO), round), shift);
    v128_store_aligned(dst+2*8, v256_low_v128(v256_unziplo_32(t, t)));

    t = v256_shr_s64(v256_add_64(v256_madd_s32(v256_dup_64((36 << 16) | (uint32_t)-83), EO), round), shift);
    v128_store_aligned(dst+6*8, v256_low_v128(v256_unziplo_32(t, t)));

    hi = v256_unziphi_64(O, O_);
    lo = v256_unziplo_64(O, O_);

    t = v256_shr_s64(v256_add_64(v256_add_64(v256_madd_s32(v256_dup_64(( 89 << 16) | (uint32_t)75), hi),
                                             v256_madd_s32(v256_dup_64(( 50 << 16) | (uint32_t)18), lo)), round), shift);
    v128_store_aligned(dst+1*8, v128_shuffle_16(v256_low_v128(v256_unziplo_32(t, t)), shuffle4));

    t = v256_shr_s64(v256_add_64(v256_add_64(v256_madd_s32(v256_dup_64((  75 << 16)  | (uint32_t)-18), hi),
                                             v256_madd_s32(v256_dup_64((-(89 << 16)) | (uint32_t)-50), lo)), round), shift);
    v128_store_aligned(dst+3*8, v128_shuffle_16(v256_low_v128(v256_unziplo_32(t, t)), shuffle4));

    t = v256_shr_s64(v256_add_64(v256_add_64(v256_madd_s32(v256_dup_64((50 << 16) | (uint32_t)-89), hi),
                                             v256_madd_s32(v256_dup_64((18 << 16) | (uint32_t) 75), lo)), round), shift);
    v128_store_aligned(dst+5*8, v128_shuffle_16(v256_low_v128(v256_unziplo_32(t, t)), shuffle4));

    t = v256_shr_s64(v256_add_64(v256_add_64(v256_madd_s32(v256_dup_64((18 << 16) | (uint32_t)-50), hi),
                                             v256_madd_s32(v256_dup_64((75 << 16) | (uint32_t)-89), lo)), round), shift);
    v128_store_aligned(dst+7*8, v128_shuffle_16(v256_low_v128(v256_unziplo_32(t, t)), shuffle4));

    src += 8*4;
    dst += 4;
  }
}

/* 16x16 transform, one dimension, partial butterfly */
static void transform16(const int32_t *src, int32_t *dst, int shift)
{
  int j;
  const v256 shuffle1 = v256_from_128(0x0100030205040706LL, 0x09080b0a0d0c0f0eLL);
  const v256 shuffle2 = v256_from_128(0x0302010007060504LL, 0x0b0a09080f0e0d0cLL);
  const v256 round = v256_dup_64(1 << (shift-1));

  for (j = 0; j < 16; j += 4) {
    v256 t0, t1, t2, t3, t4, t5, t6, t7;
    v256 EEE0, EEE1, EEO0, EEO1;
    v256 EE0, EE1, EE2, EE3;
    v256 EO0, EO1, EO2, EO3;

    v256 load0 = v256_load_aligned(src + 0);
    v256 load1 = v256_pshuffle_8(v256_load_aligned(src + 8), shuffle1);
    v256 load2 = v256_load_aligned(src + 16);
    v256 load3 = v256_pshuffle_8(v256_load_aligned(src + 24), shuffle1);
    v256 load4 = v256_load_aligned(src + 32);
    v256 load5 = v256_pshuffle_8(v256_load_aligned(src + 40), shuffle1);
    v256 load6 = v256_load_aligned(src + 48);
    v256 load7 = v256_pshuffle_8(v256_load_aligned(src + 56), shuffle1);

    /* E and O*/
    v256 E0 = v256_add_32(load0, load1);
    v256 O0 = v256_sub_32(load0, load1);
    v256 E1 = v256_add_32(load2, load3);
    v256 O1 = v256_sub_32(load2, load3);
    v256 E2 = v256_add_32(load4, load5);
    v256 O2 = v256_sub_32(load4, load5);
    v256 E3 = v256_add_32(load6, load7);
    v256 O3 = v256_sub_32(load6, load7);

    /* EE and EO */
    v256 hi = v256_unpackhi_s32_s64(E0);
    v256 lo = v256_pshuffle_8(v256_unpacklo_s32_s64(E0), shuffle2);
    EE0 = v256_add_64(lo, hi);
    EO0 = v256_sub_64(lo, hi);
    hi = v256_unpackhi_s32_s64(E1);
    lo = v256_pshuffle_8(v256_unpacklo_s32_s64(E1), shuffle2);
    EE1 = v256_add_64(lo, hi);
    EO1 = v256_sub_64(lo, hi);
    hi = v256_unpackhi_s32_s64(E2);
    lo = v256_pshuffle_8(v256_unpacklo_s32_s64(E2), shuffle2);
    EE2 = v256_add_64(lo, hi);
    EO2 = v256_sub_64(lo, hi);
    hi = v256_unpackhi_s32_s64(E3);
    lo = v256_pshuffle_8(v256_unpacklo_s32_s64(E3), shuffle2);
    EE3 = v256_add_64(lo, hi);
    EO3 = v256_sub_64(lo, hi);

    t0 = v256_ziphi_128(EO1, EO0);
    t1 = v256_ziphi_128(EO3, EO2);
    t2 = v256_ziplo_128(EO1, EO0);
    t3 = v256_ziplo_128(EO3, EO2);
    EO0 = v256_unziphi_64(t1, t0);
    EO1 = v256_unziplo_64(t1, t0);
    EO2 = v256_unziphi_64(t3, t2);
    EO3 = v256_unziplo_64(t3, t2);

    /* EEE and EEO */
    t0 = v256_ziphi_128(EE1, EE0);
    t1 = v256_ziphi_128(EE3, EE2);
    t2 = v256_ziplo_128(EE1, EE0);
    t3 = v256_ziplo_128(EE3, EE2);
    t4 = v256_unziphi_64(t1, t0);
    t5 = v256_unziplo_64(t3, t2);
    t6 = v256_unziplo_64(t1, t0);
    t7 = v256_unziphi_64(t3, t2);

    EEE0 = v256_add_64(t4, t5);
    EEE1 = v256_add_64(t6, t7);
    EEO0 = v256_sub_64(t4, t5);
    EEO1 = v256_sub_64(t6, t7);

    /* EEEE and EEEO */
    t0 = v256_shr_s64(v256_add_64(v256_shl_n_64(v256_add_64(EEE0, EEE1), 6), round), shift);
    v128_store_aligned(dst, v256_low_v128(v256_unziplo_32(t0, t0)));
    t0 = v256_shr_s64(v256_add_64(v256_shl_n_64(v256_sub_64(EEE0, EEE1), 6), round), shift);
    v128_store_aligned(dst+8*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    /* */
    t0 = v256_shr_s64(v256_add_64(v256_add_64(v256_mullo_s64(EEO0, v256_dup_64(83)),
                                              v256_mullo_s64(EEO1, v256_dup_64(36))), round), shift);
    v128_store_aligned(dst+4*16, v256_low_v128(v256_unziplo_32(t0, t0)));
    t0 = v256_shr_s64(v256_add_64(v256_add_64(v256_mullo_s64(EEO0, v256_dup_64(36)),
                                              v256_mullo_s64(EEO1, v256_dup_64(-83))), round), shift);
    v128_store_aligned(dst+12*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_shr_s64(v256_add_64(v256_add_64(v256_add_64(v256_mullo_s64(EO0, v256_dup_64(89)),
                                                          v256_mullo_s64(EO1, v256_dup_64(75))),
                                              v256_add_64(v256_mullo_s64(EO2, v256_dup_64(50)),
                                                          v256_mullo_s64(EO3, v256_dup_64(18)))), round), shift);
    v128_store_aligned(dst+2*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_shr_s64(v256_add_64(v256_add_64(v256_add_64(v256_mullo_s64(EO0, v256_dup_64(75)),
                                                          v256_mullo_s64(EO1, v256_dup_64(-18))),
                                              v256_add_64(v256_mullo_s64(EO2, v256_dup_64(-89)),
                                                          v256_mullo_s64(EO3, v256_dup_64(-50)))), round), shift);
    v128_store_aligned(dst+6*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_shr_s64(v256_add_64(v256_add_64(v256_add_64(v256_mullo_s64(EO0, v256_dup_64(50)),
                                                          v256_mullo_s64(EO1, v256_dup_64(-89))),
                                              v256_add_64(v256_mullo_s64(EO2, v256_dup_64(18)),
                                                          v256_mullo_s64(EO3, v256_dup_64(75)))), round), shift);
    v128_store_aligned(dst+10*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_shr_s64(v256_add_64(v256_add_64(v256_add_64(v256_mullo_s64(EO0, v256_dup_64(18)),
                                                          v256_mullo_s64(EO1, v256_dup_64(-50))),
                                              v256_add_64(v256_mullo_s64(EO2, v256_dup_64(75)),
                                                          v256_mullo_s64(EO3, v256_dup_64(-89)))), round), shift);
    v128_store_aligned(dst+14*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_from_128(0x00090019002b0039LL, 0x004600500057005aLL);
    t0 = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v256_dotp_s32(O3, t0),
                                               (int32_t)v256_dotp_s32(O2, t0),
                                               (int32_t)v256_dotp_s32(O1, t0),
                                               (int32_t)v256_dotp_s32(O0, t0)), round), shift);
    v128_store_aligned(dst+1*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_from_128(0xffe7ffbaffa6ffb0LL, 0xffd5000900390057LL);
    t0 = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v256_dotp_s32(O3, t0),
                                               (int32_t)v256_dotp_s32(O2, t0),
                                               (int32_t)v256_dotp_s32(O1, t0),
                                               (int32_t)v256_dotp_s32(O0, t0)), round), shift);
    v128_store_aligned(dst+3*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_from_128(0x002b005a0039ffe7LL, 0xffa9ffba00090050LL);
    t0 = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v256_dotp_s32(O3, t0),
                                               (int32_t)v256_dotp_s32(O2, t0),
                                               (int32_t)v256_dotp_s32(O1, t0),
                                               (int32_t)v256_dotp_s32(O0, t0)), round), shift);
    v128_store_aligned(dst+5*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_from_128(0xffc7ffb00019005aLL, 0x0009ffa9ffd50046LL);
    t0 = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v256_dotp_s32(O3, t0),
                                               (int32_t)v256_dotp_s32(O2, t0),
                                               (int32_t)v256_dotp_s32(O1, t0),
                                               (int32_t)v256_dotp_s32(O0, t0)), round), shift);
    v128_store_aligned(dst+7*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_from_128(0x0046002bffa9fff7LL, 0x005affe7ffb00039LL);
    t0 = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v256_dotp_s32(O3, t0),
                                               (int32_t)v256_dotp_s32(O2, t0),
                                               (int32_t)v256_dotp_s32(O1, t0),
                                               (int32_t)v256_dotp_s32(O0, t0)), round), shift);
    v128_store_aligned(dst+9*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_from_128(0xffb000090046ffa9LL, 0x00190039ffa6002bLL);
    t0 = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v256_dotp_s32(O3, t0),
                                               (int32_t)v256_dotp_s32(O2, t0),
                                               (int32_t)v256_dotp_s32(O1, t0),
                                               (int32_t)v256_dotp_s32(O0, t0)), round), shift);
    v128_store_aligned(dst+11*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_from_128(0x0057ffc70009002bLL, 0xffb0005affba0019LL);
    t0 = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v256_dotp_s32(O3, t0),
                                               (int32_t)v256_dotp_s32(O2, t0),
                                               (int32_t)v256_dotp_s32(O1, t0),
                                               (int32_t)v256_dotp_s32(O0, t0)), round), shift);
    v128_store_aligned(dst+13*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    t0 = v256_from_128(0xffa60057ffb00046LL, 0xffc7002bffe70009LL);
    t0 = v256_shr_s64(v256_add_64(v256_from_v64((int32_t)v256_dotp_s32(O3, t0),
                                               (int32_t)v256_dotp_s32(O2, t0),
                                               (int32_t)v256_dotp_s32(O1, t0),
                                               (int32_t)v256_dotp_s32(O0, t0)), round), shift);
    v128_store_aligned(dst+15*16, v256_low_v128(v256_unziplo_32(t0, t0)));

    src += 16*4;
    dst += 4;
  }
}


static void transform32(const int32_t *src, int32_t *dst, int shift, int it)
{
  extern const int32_t g4mat_hevc[32][32];
  int j, k;
  int E[16], O[16];
  int EE[8], EO[8];
  int EEE[4], EEO[4];
  int EEEE[2], EEEO[2];
  int EEEEE;
  int add = 1<<(shift-1);

  for (j = 0; j < it; j++) {
    /* E and O*/
    for (k = 0; k < 16; k++) {
      E[k] = src[k] + src[31-k];
      O[k] = src[k] - src[31-k];
    }
    /* EE and EO */
    for (k = 0; k < 8; k++) {
      EE[k] = E[k] + E[15-k];
      EO[k] = E[k] - E[15-k];
    }
    /* EEE and EEO */
    for (k = 0; k < 4; k++) {
      EEE[k] = EE[k] + EE[7-k];
      EEO[k] = EE[k] - EE[7-k];
    }
    /* EEEE and EEEO */
    EEEE[0] = EEE[0] + EEE[3];
    EEEO[0] = EEE[0] - EEE[3];
    EEEE[1] = EEE[1] + EEE[2];
    EEEO[1] = EEE[1] - EEE[2];

    /* EEEEE */
    EEEEE = EEEE[0] + EEEE[1];

    dst[0] = (g4mat_hevc[ 0][0]*EEEEE + add)>>shift;
    dst[8*32] = (g4mat_hevc[ 8][0]*EEEO[0] + g4mat_hevc[ 8][1]*EEEO[1] + add)>>shift;
    for (k = 4; k < 16; k += 8) {
      dst[k*32] = (g4mat_hevc[k][0]*EEO[0] + g4mat_hevc[k][1]*EEO[1] + g4mat_hevc[k][2]*EEO[2] + g4mat_hevc[k][3]*EEO[3] + add)>>shift;
    }
    for (k = 2; k < 16; k += 4) {
      dst[k*32] = (g4mat_hevc[k][0]*EO[0] + g4mat_hevc[k][1]*EO[1] + g4mat_hevc[k][2]*EO[2] + g4mat_hevc[k][3]*EO[3] +
                   g4mat_hevc[k][4]*EO[4] + g4mat_hevc[k][5]*EO[5] + g4mat_hevc[k][6]*EO[6] + g4mat_hevc[k][7]*EO[7] + add)>>shift;
    }
    for (k = 1; k < 16; k += 2) {
      dst[k*32] = (g4mat_hevc[k][ 0]*O[ 0] + g4mat_hevc[k][ 1]*O[ 1] + g4mat_hevc[k][ 2]*O[ 2] + g4mat_hevc[k][ 3]*O[ 3] +
                   g4mat_hevc[k][ 4]*O[ 4] + g4mat_hevc[k][ 5]*O[ 5] + g4mat_hevc[k][ 6]*O[ 6] + g4mat_hevc[k][ 7]*O[ 7] +
                   g4mat_hevc[k][ 8]*O[ 8] + g4mat_hevc[k][ 9]*O[ 9] + g4mat_hevc[k][10]*O[10] + g4mat_hevc[k][11]*O[11] +
                   g4mat_hevc[k][12]*O[12] + g4mat_hevc[k][13]*O[13] + g4mat_hevc[k][14]*O[14] + g4mat_hevc[k][15]*O[15] + add)>>shift;
    }
    src += 32;
    dst++;
  }
}


void transform_simd(const int32_t *block, int32_t *coeff, int size, int fast, int bitdepth)
{
  if (size == 4) {
    transform4(block, coeff, bitdepth);
  } else if (size == 8) {
    int32_t *tmp = thor_alloc(size*size*2, 32);
    transform8(block, tmp, bitdepth - 5);
    transform8(tmp, coeff, 8);
    thor_free(tmp);
  } else if (size == 16) {
    int32_t *tmp = thor_alloc(size*size*2, 32);
    transform16(block, tmp, bitdepth - 4);
    transform16(tmp, coeff, 9);
    thor_free(tmp);
  } else if (size == 32) {
    if (fast) {
      int32_t *tmp = thor_alloc(16*16*2, 32);
      int32_t *tmp2 = thor_alloc(16*16*2, 32);
      int32_t *tmp3 = thor_alloc(16*16*2, 32);
      for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
          tmp2[i*16+j] = block[(i*2+0)*32+j*2+0] + block[(i*2+1)*32+j*2+0] + block[(i*2+0)*32+j*2+1] + block[(i*2+1)*32+j*2+1];
      transform16(tmp2, tmp, bitdepth - 2);
      transform16(tmp, tmp3, 9);
      for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
          coeff[i*32+j] = tmp3[i*16+j];
      thor_free(tmp);
      thor_free(tmp2);
      thor_free(tmp3);
    } else {
      int32_t *tmp = thor_alloc(size*size*2, 32);
      transform32(block, tmp, bitdepth - 3, 32);
      transform32(tmp, coeff, 10, 16);
      thor_free(tmp);
    }
  } else { // size >= 64
    if (fast) {
      int32_t *tmp = thor_alloc(16*16*2, 32);
      int32_t *tmp2 = thor_alloc(16*16*2, 32);
      int32_t *tmp3 = thor_alloc(16*16*2, 32);
      int scale = size >> 4;
      for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++) {
          tmp2[i*16+j] = 0;
          for (int k = 0; k < scale; k++)
            for (int l = 0; l < scale; l++)
              tmp2[i*16+j] = min(max(tmp2[i*16+j] + block[(i*scale+k)*size+j*scale+l], -16384), 16383);
        }
      transform16(tmp2, tmp, log2i(size) + log2i(scale) + bitdepth - 8);
      transform16(tmp, tmp3, 9);
      for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
          coeff[i*size+j] = tmp3[i*16+j];
      thor_free(tmp);
      thor_free(tmp2);
      thor_free(tmp3);
    } else {
      int32_t *tmp = thor_alloc(32*32*2, 32);
      int32_t *tmp2 = thor_alloc(32*32*2, 32);
      int32_t *tmp3 = thor_alloc(32*32*2, 32);
      int scale = size >> 5;
      for (int i = 0; i < 32; i++)
        for (int j = 0; j < 32; j++) {
          tmp2[i*32+j] = 0;
          for (int k = 0; k < scale; k++)
            for (int l = 0; l < scale; l++)
              tmp2[i*32+j] += block[(i*scale+k)*size+j*scale+l];
        }
      transform32(tmp2, tmp, 2*log2i(size) + bitdepth - 13, 32);
      transform32(tmp, tmp3, 10, 16);
      for (int i = 0; i < 32; i++)
        for (int j = 0; j < 32; j++)
          coeff[i*size+j] = tmp3[i*32+j];
      thor_free(tmp);
      thor_free(tmp2);
      thor_free(tmp3);
    }
  }
}

void inverse_transform_simd(const int32_t *coeff, int32_t *block, int size, int bitdepth)
{
  if (size == 4) {
    inverse_transform4(coeff, block, bitdepth);
  } else if (size == 8) {
    int nz = check_nz_area(coeff, size);
    if (nz == COEFF_4x4_ONLY) {
      inverse_transform8_4x4(coeff, block, bitdepth);
    } else {
      inverse_transform8(coeff, block, bitdepth);
    }
  } else if (size == 16) {
    int nz = check_nz_area(coeff, size);
    if (nz == COEFF_4x4_ONLY) {
      inverse_transform16_4x4(coeff, block, bitdepth);
    } else {
      int32_t *tmp = thor_alloc(size*size*2, 32);
      inverse_transform16(coeff, tmp, 7);
      inverse_transform16(tmp, block, 20 - bitdepth);
      thor_free(tmp);
    }
  } else
    inverse_transform32(coeff, block, bitdepth);
}
#endif

void TEMPLATE(clpf_block4)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int width, int height, unsigned int strength) {
  int right = width-x0-4;
  int bottom = height-y0-4;
  dst += x0 + y0*dstride;
  src += x0 + y0*sstride;

#ifdef HBD
  static ALIGN(32) uint64_t bshuff[] = { 0x0302010001000100LL, 0x0b0a090809080908LL,
                                         0x0302010001000100LL, 0x0b0a090809080908LL };
  static ALIGN(32) uint64_t cshuff[] = { 0x0504030201000100LL, 0x0d0c0b0a09080908LL,
                                         0x0504030201000100LL, 0x0d0c0b0a09080908LL };
  static ALIGN(32) uint64_t dshuff[] = { 0x0706070605040302LL, 0x0f0e0f0e0d0c0b0aLL,
                                         0x0706070605040302LL, 0x0f0e0f0e0d0c0b0aLL };
  static ALIGN(32) uint64_t eshuff[] = { 0x0706070607060504LL, 0x0f0e0f0e0f0e0d0cLL,
                                         0x0706070607060504LL, 0x0f0e0f0e0f0e0d0cLL };
#else
  static ALIGN(16) uint64_t bshuff[] = { 0x0504040401000000LL, 0x0d0c0c0c09080808LL };
  static ALIGN(16) uint64_t cshuff[] = { 0x0605040402010000LL, 0x0e0d0c0c0a090808LL };
  static ALIGN(16) uint64_t dshuff[] = { 0x0707060503030201LL, 0x0f0f0e0d0b0b0a09LL };
  static ALIGN(16) uint64_t eshuff[] = { 0x0707070603030302LL, 0x0f0f0f0e0b0b0b0aLL };
#endif

  v64 l0 = v64_load_aligned(src - !!y0*sstride);
  v64 l1 = v64_load_aligned(src);
  v64 l2 = v64_load_aligned(src + sstride);
  v64 l3 = v64_load_aligned(src + 2*sstride);
  v64 l4 = v64_load_aligned(src + 3*sstride);
  v64 l5 = v64_load_aligned(src + (3+!!bottom)*sstride);
  v256 c128 = v256_dup_16(128);
  v256 o = v256_from_v64(l1, l2, l3, l4);
  v256 x = v256_add_16(c128, o);
  v256 a = v256_add_16(c128, v256_from_v64(l0, l1, l2, l3));
  v256 b = v256_add_16(c128, v256_from_v64(v64_load_unaligned(src - 2*!!x0),
                                         v64_load_unaligned(src + sstride - 2*!!x0),
                                         v64_load_unaligned(src + 2*sstride - 2*!!x0),
                                         v64_load_unaligned(src + 3*sstride - 2*!!x0)));
  v256 c = v256_add_16(c128, v256_from_v64(v64_load_unaligned(src - !!x0),
                                         v64_load_unaligned(src + sstride - !!x0),
                                         v64_load_unaligned(src + 2*sstride - !!x0),
                                         v64_load_unaligned(src + 3*sstride - !!x0)));
  v256 d = v256_add_16(c128, v256_from_v64(v64_load_unaligned(src + !!right),
                                         v64_load_unaligned(src + sstride + !!right),
                                         v64_load_unaligned(src + 2*sstride + !!right),
                                         v64_load_unaligned(src + 3*sstride + !!right)));
  v256 e = v256_add_16(c128, v256_from_v64(v64_load_unaligned(src + 2*!!right),
                                         v64_load_unaligned(src + sstride + 2*!!right),
                                         v64_load_unaligned(src + 2*sstride + 2*!!right),
                                         v64_load_unaligned(src + 3*sstride + 2*!!right)));
  v256 f = v256_add_16(c128, v256_from_v64(l2, l3, l4, l5));

  if (!x0) {
    b = v256_pshuffle_8(b, v256_load_aligned(bshuff));
    c = v256_pshuffle_8(c, v256_load_aligned(cshuff));
  }
  if (!right) {
    d = v256_pshuffle_8(d, v256_load_aligned(dshuff));
    e = v256_pshuffle_8(e, v256_load_aligned(eshuff));
  }
  v256 sp = v256_dup_16(strength);
  v256 sm = v256_dup_16(-(int)strength);

  v256 tmp = v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(c, x), sp), sm),
                        v256_max_s16(v256_min_s16(v256_ssub_s16(d, x), sp), sm));
  v256 delta = v256_add_16(v256_add_16(v256_shl_16(v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(a, x), sp), sm),
                                                           v256_max_s16(v256_min_s16(v256_ssub_s16(f, x), sp), sm)), 2),
                                     v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(b, x), sp), sm),
						v256_max_s16(v256_min_s16(v256_ssub_s16(e, x), sp), sm))),
                          v256_add_16(v256_add_16(tmp, tmp), tmp));
  delta = v256_shr_s16(v256_add_16(v256_dup_16(8), v256_add_16(delta, v256_cmplt_s16(delta, v256_zero()))), 4);
  v256 r = v256_add_16(o, delta);
  v64_store_aligned(dst, v256_low_v64(v256_shr_n_word(r, 12)));
  v64_store_aligned(dst + dstride, v256_low_v64(v256_shr_n_word(r, 8)));
  v64_store_aligned(dst + 2*dstride, v256_low_v64(v256_shr_n_word(r, 4)));
  v64_store_aligned(dst + 3*dstride, v256_low_v64(r));
}

void TEMPLATE(clpf_block8)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int width, int height, unsigned int strength) {
  int bottom = height-2-y0;
  v256 sp = v256_dup_16(strength);
  v256 sm = v256_dup_16(-(int)strength);
  v256 c8 = v256_dup_16(8);
  v256 c128 = v256_dup_16(128);

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

  dst += x0 + y0*dstride;
  src += x0 + y0*sstride;

  if (!x0) { // Clip left
    for (int y = 0; y < 8; y += 2) {
      v128 l1 = v128_load_aligned(src);
      v128 l2 = v128_load_aligned(src+sstride);
      v256 o = v256_from_v128(l1, l2);
      v256 x = v256_add_16(c128, o);
      v256 a = v256_add_16(c128, v256_from_v128(v128_load_aligned(src - (y != -y0)*sstride), l1));
      v256 b = v256_pshuffle_8(x, v256_load_aligned(bshuff));
      v256 c = v256_pshuffle_8(x, v256_load_aligned(cshuff));
      v256 d = v256_add_16(c128, v256_from_v128(v128_load_unaligned(src + 1),
					      v128_load_unaligned(src + 1 + sstride)));
      v256 e = v256_add_16(c128, v256_from_v128(v128_load_unaligned(src + 2),
					      v128_load_unaligned(src + 2 + sstride)));
      v256 f = v256_add_16(c128, v256_from_v128(l2, v128_load_aligned(src + ((y!=bottom)+1)*sstride)));

      v256 tmp = v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(c, x), sp), sm),
			    v256_max_s16(v256_min_s16(v256_ssub_s16(d, x), sp), sm));
      v256 delta = v256_add_16(v256_add_16(v256_shl_16(v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(a, x), sp), sm),
							       v256_max_s16(v256_min_s16(v256_ssub_s16(f, x), sp), sm)), 2),
					 v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(b, x), sp), sm),
						    v256_max_s16(v256_min_s16(v256_ssub_s16(e, x), sp), sm))),
			      v256_add_16(v256_add_16(tmp, tmp), tmp));
      o = v256_add_16(o, v256_shr_s16(v256_add_16(c8, v256_add_16(delta, v256_cmplt_s16(delta, v256_zero()))), 4));
      v128_store_aligned(dst, v256_high_v128(o));
      v128_store_aligned(dst+dstride, v256_low_v128(o));
      src += sstride*2;
      dst += dstride*2;
    }
  } else if (!(width-x0-8)) { // Clip right
    for (int y = 0; y < 8; y += 2) {
      v128 l1 = v128_load_aligned(src);
      v128 l2 = v128_load_aligned(src+sstride);
      v256 o = v256_from_v128(l1, l2);
      v256 x = v256_add_16(c128, o);
      v256 a = v256_add_16(c128, v256_from_v128(v128_load_aligned(src - (y != -y0)*sstride), l1));
      v256 b = v256_add_16(c128, v256_from_v128(v128_load_unaligned(src - 2),
					      v128_load_unaligned(src - 2 + sstride)));
      v256 c = v256_add_16(c128, v256_from_v128(v128_load_unaligned(src - 1),
					      v128_load_unaligned(src - 1 + sstride)));
      v256 d = v256_pshuffle_8(x, v256_load_aligned(dshuff));
      v256 e = v256_pshuffle_8(x, v256_load_aligned(eshuff));
      v256 f = v256_add_16(c128, v256_from_v128(l2, v128_load_aligned(src + ((y!=bottom)+1)*sstride)));

      v256 tmp = v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(c, x), sp), sm),
			    v256_max_s16(v256_min_s16(v256_ssub_s16(d, x), sp), sm));
      v256 delta = v256_add_16(v256_add_16(v256_shl_16(v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(a, x), sp), sm),
							       v256_max_s16(v256_min_s16(v256_ssub_s16(f, x), sp), sm)), 2),
					 v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(b, x), sp), sm),
						    v256_max_s16(v256_min_s16(v256_ssub_s16(e, x), sp), sm))),
			      v256_add_16(v256_add_16(tmp, tmp), tmp));
      o = v256_add_16(o, v256_shr_s16(v256_add_16(c8, v256_add_16(delta, v256_cmplt_s16(delta, v256_zero()))), 4));
      v128_store_aligned(dst, v256_high_v128(o));
      v128_store_aligned(dst+dstride, v256_low_v128(o));
      src += sstride*2;
      dst += dstride*2;
    }
  } else { // No left/right clipping
    for (int y = 0; y < 8; y += 2) {
      v128 l1 = v128_load_aligned(src);
      v128 l2 = v128_load_aligned(src+sstride);
      v256 o = v256_from_v128(l1, l2);
      v256 x = v256_add_16(c128, o);
      v256 a = v256_add_16(c128, v256_from_v128(v128_load_aligned(src - (y != -y0)*sstride), l1));
      v256 b = v256_add_16(c128, v256_from_v128(v128_load_unaligned(src - 2),
					      v128_load_unaligned(src - 2 + sstride)));
      v256 c = v256_add_16(c128, v256_from_v128(v128_load_unaligned(src - 1),
					      v128_load_unaligned(src - 1 + sstride)));
      v256 d = v256_add_16(c128, v256_from_v128(v128_load_unaligned(src + 1),
					      v128_load_unaligned(src + 1 + sstride)));
      v256 e = v256_add_16(c128, v256_from_v128(v128_load_unaligned(src + 2),
					      v128_load_unaligned(src + 2 + sstride)));
      v256 f = v256_add_16(c128, v256_from_v128(l2, v128_load_aligned(src + ((y!=bottom)+1)*sstride)));

      v256 tmp = v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(c, x), sp), sm),
			    v256_max_s16(v256_min_s16(v256_ssub_s16(d, x), sp), sm));
      v256 delta = v256_add_16(v256_add_16(v256_shl_16(v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(a, x), sp), sm),
							       v256_max_s16(v256_min_s16(v256_ssub_s16(f, x), sp), sm)), 2),
					 v256_add_16(v256_max_s16(v256_min_s16(v256_ssub_s16(b, x), sp), sm),
						    v256_max_s16(v256_min_s16(v256_ssub_s16(e, x), sp), sm))),
			      v256_add_16(v256_add_16(tmp, tmp), tmp));
      o = v256_add_16(o, v256_shr_s16(v256_add_16(c8, v256_add_16(delta, v256_cmplt_s16(delta, v256_zero()))), 4));
      v128_store_aligned(dst, v256_high_v128(o));
      v128_store_aligned(dst+dstride, v256_low_v128(o));
      src += sstride*2;
      dst += dstride*2;
    }
  }
}

void TEMPLATE(scale_frame_down2x2_simd)(yuv_frame_t* sin, yuv_frame_t* sout)
{
  int wo=sout->width;
  int ho=sout->height;
  int so=sout->stride_y;
  int si=sin->stride_y;
  int i, j;
  v256 z = v256_dup_16(0);
  for (i=0; i<ho; ++i) {

    for (j=0; j<=wo-8; j+=8) {
      v256 a = v256_load_unaligned(&sin->y[(2*i+0)*si+2*j]);
      v256 b = v256_load_unaligned(&sin->y[(2*i+1)*si+2*j]);
      v256 c = v256_avg_u16(a,b);
      v256 d = v256_shr_s32(v256_padd_s16(c),1);
      v128_store_aligned(&sout->y[i*so+j], v256_low_v128(v256_pack_s32_u16(z,d)));
    }
    for (; j<wo; ++j) {
      sout->y[i*so+j]=( ((sin->y[(2*i+0)*si+(2*j+0)] + sin->y[(2*i+1)*si+(2*j+0)]+1)>>1)+
                      + ((sin->y[(2*i+0)*si+(2*j+1)] + sin->y[(2*i+1)*si+(2*j+1)]+1)>>1) )>>1;
    }

  }
#if TEMP_INTERP_USE_CHROMA
  int soc=sout->stride_c;
  int sic=sin->stride_c;
  ho /= 2;
  wo /= 2;
  for (int i=0; i<ho; ++i) {

    for (j=0; j<=wo-8; j+=8) {
      v256 a = v256_load_aligned(&sin->u[(2*i+0)*sic+2*j]);
      v256 b = v256_load_aligned(&sin->u[(2*i+1)*sic+2*j]);
      v256 c = v256_avg_u16(a,b);
      v256 d = v256_shr_s32(v256_padd_s16(c),1);
      v128_store_aligned(&sout->u[i*soc+j], v256_low_v128(v256_pack_s32_u16(z,d)));
    }
    for (; j<wo; ++j) {
      sout->u[i*soc+j]=( ((sin->u[(2*i+0)*sic+(2*j+0)] + sin->u[(2*i+1)*sic+(2*j+0)]+1)>>1)+
                       + ((sin->u[(2*i+0)*sic+(2*j+1)] + sin->u[(2*i+1)*sic+(2*j+1)]+1)>>1) )>>1;
    }

    for (j=0; j<=wo-8; j+=8) {
      v256 a = v256_load_aligned(&sin->v[(2*i+0)*sic+2*j]);
      v256 b = v256_load_aligned(&sin->v[(2*i+1)*sic+2*j]);
      v256 c = v256_avg_u16(a,b);
      v256 d = v256_shr_s32(v256_madd_s16(c,ones),1);
      v128_store_aligned(&sout->v[i*soc+j], v256_low_v128(v256_pack_s32_u16(z,d)));
    }
    for (; j<wo; ++j) {
      sout->v[i*soc+j]=( ((sin->v[(2*i+0)*sic+(2*j+0)] + sin->v[(2*i+1)*sic+(2*j+0)]+1)>>1)+
                       + ((sin->v[(2*i+0)*sic+(2*j+1)] + sin->v[(2*i+1)*sic+(2*j+1)]+1)>>1) )>>1;
    }

  }
#endif
}

extern const int32_t coeffs_standard[][8];
extern const int32_t coeffs_bipred[][8];
extern const int32_t coeffs_chroma[][4];

static void filter_6tap_edge(int width, int height, int xoff, int yoff,
                             SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip,
                             int istride, int bitdepth, const int32_t coeffs[][8])
{
  int cf = max(xoff, yoff);
  int sx = !yoff;
  int s1 = !xoff * istride;
  const int32_t *c = coeffs[cf];
  int st1 = s1 + sx;

  ip -= istride;
  qp -= qstride;

  if (width == 4) {
    v128 c0 = v128_dup_32(c[0]);
    v128 c1 = v128_dup_32(c[1]);
    v128 c2 = v128_dup_32(c[2]);
    v128 c3 = v128_dup_32(c[3]);
    v128 c4 = v128_dup_32(c[4]);
    v128 c5 = v128_dup_32(c[5]);
    v128 cr = v128_dup_32(32);

    for (int y = 0; y < height; y++) {
      qp += qstride;
      ip += istride;

      const SAMPLE *r = ip - 2 * s1 - 2 * sx;
      v128 r0 = v128_mullo_s32(c0, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*0)));
      v128 r1 = v128_mullo_s32(c1, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*1)));
      v128 r2 = v128_mullo_s32(c2, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*2)));
      v128 r3 = v128_mullo_s32(c3, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*3)));
      v128 r4 = v128_mullo_s32(c4, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*4)));
      v128 r5 = v128_mullo_s32(c5, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*5)));
      v128 rs = v128_add_32(v128_add_32(v128_add_32(v128_add_32(v128_add_32(v128_add_32(cr, r0), r1), r2), r3), r4), r5);
#ifdef HBD
      rs = v128_shr_s32(rs, bitdepth - 10);
      v64_store_aligned(qp, v128_low_v64(v128_shr_u16(v128_pack_s32_u16(rs, rs), 16 - bitdepth)));
#else
      rs = v128_shr_n_s32(rs, 6);
      v64_store_aligned(qp, v128_low_v64(v128_pack_s32_u16(rs, rs)));
#endif
    }
  } else {
    v256 c0 = v256_dup_32(c[0]);
    v256 c1 = v256_dup_32(c[1]);
    v256 c2 = v256_dup_32(c[2]);
    v256 c3 = v256_dup_32(c[3]);
    v256 c4 = v256_dup_32(c[4]);
    v256 c5 = v256_dup_32(c[5]);
    v256 cr = v256_dup_32(32);

    ip += width;
    for (int y = 0; y < height; y++) {
      qp += qstride;
      ip += istride - width;

      for (int x = 0; x < width; x += 8) {
        const SAMPLE *r = ip - 2 * s1 - 2 * sx;
        v256 r0 = v256_mullo_s32(c0, v256_unpack_u16_s32(v128_load_unaligned(r + st1*0)));
        v256 r1 = v256_mullo_s32(c1, v256_unpack_u16_s32(v128_load_unaligned(r + st1*1)));
        v256 r2 = v256_mullo_s32(c2, v256_unpack_u16_s32(v128_load_unaligned(r + st1*2)));
        v256 r3 = v256_mullo_s32(c3, v256_unpack_u16_s32(v128_load_unaligned(r + st1*3)));
        v256 r4 = v256_mullo_s32(c4, v256_unpack_u16_s32(v128_load_unaligned(r + st1*4)));
        v256 r5 = v256_mullo_s32(c5, v256_unpack_u16_s32(v128_load_unaligned(r + st1*5)));
        v256 rs = v256_add_32(v256_add_32(v256_add_32(v256_add_32(v256_add_32(v256_add_32(cr, r0), r1), r2), r3), r4), r5);
        ip += 8;
#ifdef HBD
        rs = v256_shr_s32(rs, bitdepth - 10);
        v128_store_aligned(qp + x, v128_shr_u16(v256_low_v128(v256_pack_s32_u16(rs, rs)), 16 - bitdepth));
#else
        rs = v256_shr_n_s32(rs, 6);
        v128_store_aligned(qp + x, v256_low_v128(v256_pack_s32_u16(rs, rs)));
#endif
      }
    }
  }
}

static void filter_6tap_inner(int width, int height, int xoff, int yoff,
                              SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip,
                              int istride, int bitdepth, const int32_t coeffs[][8]) {
  const int32_t *cf = coeffs[yoff];
  v256 c = v256_load_aligned(coeffs[xoff]);

  if (width == 4) {
    int xtap = coeffs[xoff][5]; // Final tap
    v256 c0 = v256_dup_32(cf[0]);
    v256 c1 = v256_dup_32(cf[1]);
    v256 c2 = v256_dup_32(cf[2]);
    v256 c3 = v256_dup_32(cf[3]);
    v256 c4 = v256_dup_32(cf[4]);
    v256 c5 = v256_dup_32(cf[5]);

    for (int y = 0; y < height; y++) {
      int res;
      v256 ax = v256_unpack_u16_s32(v128_load_unaligned(ip - 2));
      v256 a0 = v256_mullo_s32(c0, v256_unpack_u16_s32(v128_load_unaligned(ip - 2 * istride - 2)));
      v256 a1 = v256_mullo_s32(c1, v256_unpack_u16_s32(v128_load_unaligned(ip - 1 * istride - 2)));
      v256 a2 = v256_mullo_s32(c2, v256_unpack_u16_s32(v128_load_unaligned(ip - 2)));
      v256 a3 = v256_mullo_s32(c3, v256_unpack_u16_s32(v128_load_unaligned(ip + 1 * istride - 2)));
      v256 a4 = v256_mullo_s32(c4, v256_unpack_u16_s32(v128_load_unaligned(ip + 2 * istride - 2)));
      v256 a5 = v256_mullo_s32(c5, v256_unpack_u16_s32(v128_load_unaligned(ip + 3 * istride - 2)));

      for (int x = 0; x < 3; x++) {
        res = (int)((v256_dotp_s32(c, v256_add_32(v256_add_32(v256_add_32(v256_add_32(v256_add_32(a0, a1), a2), a3), a4), a5)) + 2048) >> 12);
        *qp++ = saturate(res, bitdepth);
        ax = v256_shr_n_word(ax, 2);
        a0 = v256_shr_n_word(a0, 2);
        a1 = v256_shr_n_word(a1, 2);
        a2 = v256_shr_n_word(a2, 2);
        a3 = v256_shr_n_word(a3, 2);
        a4 = v256_shr_n_word(a4, 2);
        a5 = v256_shr_n_word(a5, 2);
      }

      int a08 = ip[6-2*istride]*coeffs[yoff][0]*xtap;
      int a18 = ip[6-1*istride]*coeffs[yoff][1]*xtap;
      int a28 = ip[6-0*istride]*coeffs[yoff][2]*xtap;
      int a38 = ip[6+1*istride]*coeffs[yoff][3]*xtap;
      int a48 = ip[6+2*istride]*coeffs[yoff][4]*xtap;
      int a58 = ip[6+3*istride]*coeffs[yoff][5]*xtap;

      res = (int)((v256_dotp_s32(c, v256_add_32(v256_add_32(v256_add_32(v256_add_32(v256_add_32(a0, a1), a2), a3), a4), a5)) +
                   + a08 + a18 + a28 + a38 + a48 + a58 + 2048) >> 12);
      *qp++ = saturate(res, bitdepth);
      ip += istride;
      qp += qstride - 4;
    }
  } else {
    const SAMPLE *restrict ip2 = ip;
    v256 c1, c2, c3;
    int32_t *ax = thor_alloc((width+8)*height*2*sizeof(SAMPLE), 32);
#ifdef HBD
    const int shift = 16;
#else
    const int shift = 8;
#endif
    c1 = v256_dup_32((coeffs[yoff][0] << shift) | (SAMPLE)coeffs[yoff][1]);
    c2 = v256_dup_32((coeffs[yoff][2] << shift) | (SAMPLE)coeffs[yoff][3]);
    c3 = v256_dup_32((coeffs[yoff][4] << shift) | (SAMPLE)coeffs[yoff][5]);

    for (int y = 0; y < height; y++) {
      int32_t *a = ax + y*(width+8);
      for (int i = 0; i <= width; i += 8) {
        v256 t1 = v256_madd_s16(v256_zip_16(v128_load_unaligned(ip - 2 * istride - 2),
                                           v128_load_unaligned(ip - 1 * istride - 2)), c1);
        v256 t2 = v256_madd_s16(v256_zip_16(v128_load_unaligned(ip - 0 * istride - 2),
                                           v128_load_unaligned(ip + 1 * istride - 2)), c2);
        v256 t3 = v256_madd_s16(v256_zip_16(v128_load_unaligned(ip + 2 * istride - 2),
                                           v128_load_unaligned(ip + 3 * istride - 2)), c3);
        v256_store_aligned(a + i, v256_add_32(v256_add_32(t1, t2), t3));
        ip += 8;
      }
      ip += istride - width - 8;
    }
    ip = ip2 - 2;

    for (int y = 0; y < height; y++) {
      int32_t *a = ax + y*(width+8);
      for (int i = 0; i < width; i += 8) {
#ifdef HBD
        v128 r0 = v128_from_32(v256_dotp_s32(c, v256_load_unaligned(a + i + 7)),
                                  v256_dotp_s32(c, v256_load_unaligned(a + i + 6)),
                                  v256_dotp_s32(c, v256_load_unaligned(a + i + 5)),
                                  v256_dotp_s32(c, v256_load_unaligned(a + i + 4)));
        v128 r1 = v128_from_32(v256_dotp_s32(c, v256_load_unaligned(a + i + 3)),
                                  v256_dotp_s32(c, v256_load_unaligned(a + i + 2)),
                                  v256_dotp_s32(c, v256_load_unaligned(a + i + 1)),
                                  v256_dotp_s32(c, v256_load_unaligned(a + i + 0)));
        r0 = v128_shr_s32(v128_add_32(r0, v128_dup_32(2048)), bitdepth - 4);
        r1 = v128_shr_s32(v128_add_32(r1, v128_dup_32(2048)), bitdepth - 4);
        v128_store_aligned(qp + y*qstride + i, v128_shr_u16(v128_pack_s32_u16(r0, r1), 16 - bitdepth));
#else
        v256 r0 = v256_from_v64(v256_dotp_s32(c, v256_load_unaligned(a + i + 7)),
                               v256_dotp_s32(c, v256_load_unaligned(a + i + 6)),
                               v256_dotp_s32(c, v256_load_unaligned(a + i + 5)),
                               v256_dotp_s32(c, v256_load_unaligned(a + i + 4)));
        v256 r1 = v256_from_v64(v256_dotp_s32(c, v256_load_unaligned(a + i + 3)),
                               v256_dotp_s32(c, v256_load_unaligned(a + i + 2)),
                               v256_dotp_s32(c, v256_load_unaligned(a + i + 1)),
                               v256_dotp_s32(c, v256_load_unaligned(a + i + 0)));
        r0 = v256_shr_n_s64(v256_add_64(r0, v256_dup_64(2048)), 12);
        r1 = v256_shr_n_s64(v256_add_64(r1, v256_dup_64(2048)), 12);
        r0 = v256_pack_s64_s32(r0, r1);
        v128_store_aligned(qp + y*qstride + i, v256_low_v128(v256_pack_s32_u16(r0, r0)));
#endif
      }
    }
    thor_free(ax);
  }
}


static void get_inter_prediction_luma_centre(int width, int height,
                                             SAMPLE *restrict qp, int qstride,
                                             const SAMPLE *restrict ip, int istride)
{
  if (width == 4) {
    v128 round = v128_dup_32(8);
    for (int i = 0; i < height; i++) {
      v128 r, s;
      r = v128_add_32(v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip - 1 * istride + 0))),
                     v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip - 1 * istride + 1))));
      r = v128_add_32(r, v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip - 0 * istride - 1))));
      r = v128_add_32(r, v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip + 1 * istride - 1))));
      r = v128_add_32(r, v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip + 1 * istride + 2))));
      r = v128_add_32(r, v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip + 2 * istride + 0))));
      r = v128_add_32(r, v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip + 2 * istride + 1))));
      r = v128_add_32(r, v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip - 0 * istride + 2))));
      s = v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip - 0 * istride + 0)));
      r = v128_add_32(r, v128_add_32(s, s));
      s = v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip - 0 * istride + 1)));
      r = v128_add_32(r, v128_add_32(s, s));
      s = v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip + 1 * istride + 0)));
      r = v128_add_32(r, v128_add_32(s, s));
      s = v128_unpacklo_u16_s32(v128_from_v64(v64_zero(), v64_load_unaligned(ip + 1 * istride + 1)));
      r = v128_add_32(r, v128_add_32(s, s));
      r = v128_shr_s32(v128_add_32(r, round), 4);
      v64_store_aligned(qp + i * qstride, v128_low_v64(v128_pack_s32_u16(r, r)));
      ip += istride;
    }
  } else {
    v256 round = v256_dup_32(8);
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j += 8) {
        v256 r, s;
        r = v256_add_32(v256_unpack_u16_s32(v128_load_unaligned(ip - 1 * istride + 0)),
                        v256_unpack_u16_s32(v128_load_unaligned(ip - 1 * istride + 1)));
        r = v256_add_32(r, v256_unpack_u16_s32(v128_load_unaligned(ip - 0 * istride - 1)));
        r = v256_add_32(r, v256_unpack_u16_s32(v128_load_unaligned(ip + 1 * istride - 1)));
        r = v256_add_32(r, v256_unpack_u16_s32(v128_load_unaligned(ip + 1 * istride + 2)));
        r = v256_add_32(r, v256_unpack_u16_s32(v128_load_unaligned(ip + 2 * istride + 0)));
        r = v256_add_32(r, v256_unpack_u16_s32(v128_load_unaligned(ip + 2 * istride + 1)));
        r = v256_add_32(r, v256_unpack_u16_s32(v128_load_unaligned(ip - 0 * istride + 2)));
        s = v256_unpack_u16_s32(v128_load_unaligned(ip - 0 * istride + 0));
        r = v256_add_32(r, v256_add_32(s, s));
        s = v256_unpack_u16_s32(v128_load_unaligned(ip - 0 * istride + 1));
        r = v256_add_32(r, v256_add_32(s, s));
        s = v256_unpack_u16_s32(v128_load_unaligned(ip + 1 * istride + 0));
        r = v256_add_32(r, v256_add_32(s, s));
        s = v256_unpack_u16_s32(v128_load_unaligned(ip + 1 * istride + 1));
        r = v256_add_32(r, v256_add_32(s, s));
        r = v256_shr_s32(v256_add_32(r, round), 4);
        v128_store_aligned(qp + i * qstride + j, v256_low_v128(v256_pack_s32_u16(r, r)));
        ip += 8;
      }
      ip += istride - width;
    }
  }
}

void TEMPLATE(get_inter_prediction_luma_simd)(int width, int height, int xoff, int yoff,
                                              SAMPLE *restrict qp, int qstride,
                                              const SAMPLE *restrict ip, int istride, int bipred, int bitdepth)
{
  if (xoff == 2 && yoff == 2 && bipred < 2)
    get_inter_prediction_luma_centre(width, height, qp, qstride, ip, istride);
  else
    (!xoff || !yoff ? filter_6tap_edge : filter_6tap_inner)
      (width, height, xoff, yoff, qp, qstride, ip, istride, bitdepth, bipred ? coeffs_bipred : coeffs_standard);
}

static void filter_4tap_edge(int width, int height, int xoff, int yoff,
                             SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip,
                             int istride, int bitdepth, const int32_t coeffs[][4])
{
  int cf = max(xoff, yoff);
  int sx = !yoff;
  int s1 = !xoff * istride;
  const int32_t *c = &coeffs[cf][0];
  int st1 = s1 + sx;

  ip -= istride;
  qp -= qstride;

  if (width == 4) {
    v128 c0 = v128_dup_32(c[0]);
    v128 c1 = v128_dup_32(c[1]);
    v128 c2 = v128_dup_32(c[2]);
    v128 c3 = v128_dup_32(c[3]);
    v128 cr = v128_dup_32(32);

    for (int y = 0; y < height; y++) {
      qp += qstride;
      ip += istride;

      const SAMPLE *r = ip - s1 - sx;
      v128 r0 = v128_mullo_s32(c0, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*0)));
      v128 r1 = v128_mullo_s32(c1, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*1)));
      v128 r2 = v128_mullo_s32(c2, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*2)));
      v128 r3 = v128_mullo_s32(c3, v128_unpacklo_u16_s32(v128_load_unaligned(r + st1*3)));
      v128 rs = v128_add_32(v128_add_32(v128_add_32(v128_add_32(cr, r0), r1), r2), r3);
#ifdef HBD
      rs = v128_shr_s32(rs, bitdepth - 10);
      v64_store_aligned(qp, v128_low_v64(v128_shr_u16(v128_pack_s32_u16(rs, rs), 16 - bitdepth)));
#else
      rs = v128_shr_n_s32(rs, 6);
      v64_store_aligned(qp, v128_low_v64(v128_pack_s32_u16(rs, rs)));
#endif
    }
  } else {
    v256 c0 = v256_dup_32(c[0]);
    v256 c1 = v256_dup_32(c[1]);
    v256 c2 = v256_dup_32(c[2]);
    v256 c3 = v256_dup_32(c[3]);
    v256 cr = v256_dup_32(32);

    ip += width;
    for (int y = 0; y < height; y++) {
      qp += qstride;
      ip += istride - width;

      for (int x = 0; x < width; x += 8) {
        const SAMPLE *r = ip - s1 - sx;
        v256 r0 = v256_mullo_s32(c0, v256_unpack_u16_s32(v128_load_unaligned(r + st1*0)));
        v256 r1 = v256_mullo_s32(c1, v256_unpack_u16_s32(v128_load_unaligned(r + st1*1)));
        v256 r2 = v256_mullo_s32(c2, v256_unpack_u16_s32(v128_load_unaligned(r + st1*2)));
        v256 r3 = v256_mullo_s32(c3, v256_unpack_u16_s32(v128_load_unaligned(r + st1*3)));
        v256 rs = v256_add_32(v256_add_32(v256_add_32(v256_add_32(cr, r0), r1), r2), r3);
        ip += 8;
#ifdef HBD
        rs = v256_shr_s32(rs, bitdepth - 10);
        v128_store_aligned(qp + x, v128_shr_u16(v256_low_v128(v256_pack_s32_u16(rs, rs)), 16 - bitdepth));
#else
        rs = v256_shr_n_s32(rs, 6);
        v128_store_aligned(qp + x, v256_low_v128(v256_pack_s32_u16(rs, rs)));
#endif
      }
    }
  }
}

static void filter_4tap_inner(int width, int height, int xoff, int yoff,
                              SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip,
                              int istride, int bitdepth, const int32_t coeffs[][4]) {
  const int32_t *cf = &coeffs[yoff][0];
  const v256 c0 = v256_dup_32(cf[0]);
  const v256 c1 = v256_dup_32(cf[1]);
  const v256 c2 = v256_dup_32(cf[2]);
  const v256 c3 = v256_dup_32(cf[3]);
  v128 filter = v128_load_aligned(coeffs[xoff]);
#ifdef HBD
  const v128 round = v128_dup_32(2048);
#else
  const v256 round = v256_dup_64(2048);
#endif

  if (width == 4) {
    v256 in0 = v256_unpack_u16_s32(v128_load_unaligned(ip - 1*istride - 1));
    v256 in1 = v256_unpack_u16_s32(v128_load_unaligned(ip + 0*istride - 1));
    v256 in2 = v256_unpack_u16_s32(v128_load_unaligned(ip + 1*istride - 1));

    for (int i = 0; i < height; i++) {
      v256 in3 = v256_unpack_u16_s32(v128_load_unaligned(ip + (i+2)*istride - 1));
      v256 out1 = v256_add_32(v256_add_32(v256_add_32(v256_mullo_s32(c0, in0), v256_mullo_s32(c1, in1)), v256_mullo_s32(c2, in2)), v256_mullo_s32(c3, in3));

#ifdef HBD
      v128 hor_out = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v128_dotp_s32(v256_low_v128(v256_shr_n_word(out1, 6)), filter),
                                                                           (int32_t)v128_dotp_s32(v256_low_v128(v256_shr_n_word(out1, 4)), filter),
                                                                           (int32_t)v128_dotp_s32(v256_low_v128(v256_shr_n_word(out1, 2)), filter),
                                                                           (int32_t)v128_dotp_s32(v256_low_v128(out1), filter)), round), bitdepth - 4);
      v64_store_aligned(qp + qstride * i, v64_shr_u16(v128_low_v64(v128_pack_s32_u16(hor_out, hor_out)), 16 - bitdepth));
#else
      v256 hor_out = v256_shr_n_s64(v256_add_64(v256_from_v64((int32_t)v128_dotp_s32(v256_low_v128(v256_shr_n_word(out1, 6)), filter),
                                                             (int32_t)v128_dotp_s32(v256_low_v128(v256_shr_n_word(out1, 4)), filter),
                                                             (int32_t)v128_dotp_s32(v256_low_v128(v256_shr_n_word(out1, 2)), filter),
                                                             (int32_t)v128_dotp_s32(v256_low_v128(out1), filter)), round), 12);
      v128 out = v128_pack_s64_s32(v256_high_v128(hor_out), v256_low_v128(hor_out));
      v64_store_aligned(qp + qstride * i, v128_low_v64(v128_pack_s32_u16(out, out)));
#endif

      in0 = in1;
      in1 = in2;
      in2 = in3;
    }
  } else {
    for (int j = 0; j < width; j += 8) {
      v256 load0 = v256_load_unaligned(ip - 1*istride + j - 1);
      v256 load1 = v256_load_unaligned(ip + 0*istride + j - 1);
      v256 load2 = v256_load_unaligned(ip + 1*istride + j - 1);
      v256 in00 = v256_unpacklo_u16_s32(load0);
      v256 in01 = v256_unpacklo_u16_s32(load1);
      v256 in02 = v256_unpacklo_u16_s32(load2);
      v256 in10 = v256_unpackhi_u16_s32(load0);
      v256 in11 = v256_unpackhi_u16_s32(load1);
      v256 in12 = v256_unpackhi_u16_s32(load2);

      for (int i = 0; i < height; i++) {
        v256 load3 = v256_load_unaligned(ip + (i+2)*istride + j - 1);
        v256 in03 = v256_unpacklo_u16_s32(load3);
        v256 in13 = v256_unpackhi_u16_s32(load3);

        /* Vertical */
        v256 out0 = v256_add_32(v256_add_32(v256_add_32(v256_mullo_s32(c0, in00), v256_mullo_s32(c1, in01)), v256_mullo_s32(c2, in02)), v256_mullo_s32(c3, in03));

        v256 out1 = v256_add_32(v256_add_32(v256_add_32(v256_mullo_s32(c0, in10), v256_mullo_s32(c1, in11)), v256_mullo_s32(c2, in12)), v256_mullo_s32(c3, in13));

        /* Horizontal */
        uint64_t in0 = v128_dotp_s32(v256_low_v128(out0), filter);
        uint64_t in1 = v128_dotp_s32(v256_low_v128(v256_shr_n_word(out0, 2)), filter);
        uint64_t in2 = v128_dotp_s32(v256_low_v128(v256_shr_n_word(out0, 4)), filter);
        uint64_t in3 = v128_dotp_s32(v256_low_v128(v256_shr_n_word(out0, 6)), filter);
        uint64_t in4 = v128_dotp_s32(v256_high_v128(out0), filter);
        uint64_t in5 = v128_dotp_s32(v256_low_v128(v256_align(out1, out0, 10*sizeof(SAMPLE))), filter);
        uint64_t in6 = v128_dotp_s32(v256_low_v128(v256_align(out1, out0, 12*sizeof(SAMPLE))), filter);
        uint64_t in7 = v128_dotp_s32(v256_low_v128(v256_align(out1, out0, 14*sizeof(SAMPLE))), filter);

#ifdef HBD
        v128 out = v128_pack_s32_u16(v128_shr_s32(v128_add_32(v128_from_32((int32_t)in7, (int32_t)in6, (int32_t)in5, (int32_t)in4), round), bitdepth - 4),
                                             v128_shr_s32(v128_add_32(v128_from_32((int32_t)in3, (int32_t)in2, (int32_t)in1, (int32_t)in0), round), bitdepth - 4));
        v128_store_aligned(qp + qstride * i + j, v128_shr_u16(out, 16 - bitdepth));
#else
        v256 out = v256_pack_s64_s32(v256_shr_n_s64(v256_add_64(v256_from_v64((int32_t)in7, (int32_t)in6, (int32_t)in5, (int32_t)in4), round), 12),
                                     v256_shr_n_s64(v256_add_64(v256_from_v64((int32_t)in3, (int32_t)in2, (int32_t)in1, (int32_t)in0), round), 12));
        v128_store_aligned(qp + qstride * i + j, v256_low_v128(v256_pack_s32_u16(out, out)));
#endif

        /* Shift input one line up */
        in00 = in01;
        in01 = in02;
        in02 = in03;

        in10 = in11;
        in11 = in12;
        in12 = in13;
      }
    }
  }
}

void TEMPLATE(get_inter_prediction_chroma_simd)(int width, int height, int xoff, int yoff,
                                                SAMPLE *restrict qp, int qstride,
                                                const SAMPLE *restrict ip, int istride, int bitdepth) {
  (!xoff || !yoff ? filter_4tap_edge : filter_4tap_inner)
    (width, height, xoff, yoff, qp, qstride, ip, istride, bitdepth, coeffs_chroma);
}
