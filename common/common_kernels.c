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
#include "common_kernels.h"

void TEMPLATE(block_avg_simd)(SAMPLE *p,SAMPLE *r0, SAMPLE *r1, int sp, int s0, int s1, int width, int height)
{
  int i,j;
  if (width == 4) {
    v64 a, b, out;
    // Assume height is divisible by 2
    for (i=0; i<height; i+=2) {
      a = v64_from_32(u32_load_unaligned(&r0[i*s0]), u32_load_unaligned(&r0[(i+1)*s0]));
      b = v64_from_32(u32_load_unaligned(&r1[i*s1]), u32_load_unaligned(&r1[(i+1)*s1]));
      out = v64_avg_u8(a,b);
      u32_store_unaligned(&p[i*sp], v64_high_u32(out));
      u32_store_unaligned(&p[(i+1)*sp], v64_low_u32(out));
    }
  } else {
    v64 a, b, c, d;
    // Assume width divisible by 8, height by 2
    for (i=0; i<height; i+=2) {
      for (j=0; j<width; j+=8) {
        a = v64_load_unaligned(&r0[i*s0+j]);
        b = v64_load_unaligned(&r1[i*s1+j]);
        c = v64_load_unaligned(&r0[(i+1)*s0+j]);
        d = v64_load_unaligned(&r1[(i+1)*s1+j]);
        v64_store_aligned(&p[i*sp+j],v64_avg_u8(a,b));
        v64_store_aligned(&p[(i+1)*sp+j],v64_avg_u8(c,d));
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
        sad64_internal s = v64_sad_u8_init();
        for (i = 0; i < height; i += 4) {
          s = v64_sad_u8(s, v64_load_unaligned(a + 0*astride), v64_load_unaligned(b + 0*bstride));
          s = v64_sad_u8(s, v64_load_unaligned(a + 1*astride), v64_load_unaligned(b + 1*bstride));
          s = v64_sad_u8(s, v64_load_unaligned(a + 2*astride), v64_load_unaligned(b + 2*bstride));
          s = v64_sad_u8(s, v64_load_unaligned(a + 3*astride), v64_load_unaligned(b + 3*bstride));
          a += 4*astride;
          b += 4*bstride;
        }
        return v64_sad_u8_sum(s);
      }
    case 4:
      {
        sad128_internal s = v128_sad_u8_init();
        for (i = 0; i < height; i += 4) {
          s = v128_sad_u8(s, v128_from_32(u32_load_aligned(a+(i+3)*astride),
                                          u32_load_aligned(a+(i+2)*astride),
                                          u32_load_aligned(a+(i+1)*astride),
                                          u32_load_aligned(a+i*astride)),
                          v128_from_32(u32_load_aligned(b+(i+3)*bstride),
                                       u32_load_aligned(b+(i+2)*bstride),
                                       u32_load_aligned(b+(i+1)*bstride),
                                       u32_load_aligned(b+i*bstride)));
        }
        return v128_sad_u8_sum(s);
      }
    default:
      {
        sad128_internal s = v128_sad_u8_init();
        for (i = 0; i < height; i+=4)
          for (j = 0; j < width; j += 16) {
            s = v128_sad_u8(s, v128_load_unaligned(a + 0*astride + j), v128_load_unaligned(b + 0*bstride + j));
            s = v128_sad_u8(s, v128_load_unaligned(a + 1*astride + j), v128_load_unaligned(b + 1*bstride + j));
            s = v128_sad_u8(s, v128_load_unaligned(a + 2*astride + j), v128_load_unaligned(b + 2*bstride + j));
            s = v128_sad_u8(s, v128_load_unaligned(a + 3*astride + j), v128_load_unaligned(b + 3*bstride + j));
            a += 4*astride;
            b += 4*bstride;
          }
        return v128_sad_u8_sum(s);
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
int check_nz_area(const int16_t *coeff, int size)
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
static void transform4(const int16_t *src, int16_t *dst, int bitdepth)
{
  v128 t;
  v128 add1 = v128_dup_32(1 << (bitdepth - 7));
  v128 add2 = v128_dup_32(64);
  int shift1 = bitdepth - 6;
  v64 h0, h1, h2, h3;
  v64 g0 = v64_from_64(0x0040004000400040LL); /*  64  64  64  64 */
  v64 g1 = v64_from_64(0xffadffdc00240053LL); /* -83 -36  36  83 */
  v64 g2 = v64_from_64(0x0040ffc0ffc00040LL); /*  64 -64 -64  64 */
  v64 g3 = v64_from_64(0xffdc0053ffad0024LL); /* -36  83 -83  36 */
  v64 s0 = v64_load_aligned(src + 0*4);
  v64 s1 = v64_load_aligned(src + 1*4);
  v64 s2 = v64_load_aligned(src + 2*4);
  v64 s3 = v64_load_aligned(src + 3*4);

  /* Horizontal transform */
  t = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v64_dotp_s16(s3, g0),
                                            (int32_t)v64_dotp_s16(s2, g0),
                                            (int32_t)v64_dotp_s16(s1, g0),
                                            (int32_t)v64_dotp_s16(s0, g0)), add1), shift1);
  h0 = v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t));

  t = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v64_dotp_s16(s3, g1),
                                            (int32_t)v64_dotp_s16(s2, g1),
                                            (int32_t)v64_dotp_s16(s1, g1),
                                            (int32_t)v64_dotp_s16(s0, g1)), add1), shift1);
  h1 = v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t));

  t = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v64_dotp_s16(s3, g2),
                                            (int32_t)v64_dotp_s16(s2, g2),
                                            (int32_t)v64_dotp_s16(s1, g2),
                                            (int32_t)v64_dotp_s16(s0, g2)), add1), shift1);
  h2 = v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t));

  t = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v64_dotp_s16(s3, g3),
                                            (int32_t)v64_dotp_s16(s2, g3),
                                            (int32_t)v64_dotp_s16(s1, g3),
                                            (int32_t)v64_dotp_s16(s0, g3)), add1), shift1);
  h3 = v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t));

  /* Vertical transform */
  t = v128_shr_n_s32(v128_add_32(v128_from_32((int32_t)v64_dotp_s16(h3, g0),
                                              (int32_t)v64_dotp_s16(h2, g0),
                                              (int32_t)v64_dotp_s16(h1, g0),
                                              (int32_t)v64_dotp_s16(h0, g0)), add2), 7);
  v64_store_aligned(dst +  0, v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t)));

  t = v128_shr_n_s32(v128_add_32(v128_from_32((int32_t)v64_dotp_s16(h3, g1),
                                              (int32_t)v64_dotp_s16(h2, g1),
                                              (int32_t)v64_dotp_s16(h1, g1),
                                              (int32_t)v64_dotp_s16(h0, g1)), add2), 7);
  v64_store_aligned(dst +  4, v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t)));

  t = v128_shr_n_s32(v128_add_32(v128_from_32((int32_t)v64_dotp_s16(h3, g2),
                                              (int32_t)v64_dotp_s16(h2, g2),
                                              (int32_t)v64_dotp_s16(h1, g2),
                                              (int32_t)v64_dotp_s16(h0, g2)), add2), 7);
  v64_store_aligned(dst +  8, v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t)));

  t = v128_shr_n_s32(v128_add_32(v128_from_32((int32_t)v64_dotp_s16(h3, g3),
                                              (int32_t)v64_dotp_s16(h2, g3),
                                              (int32_t)v64_dotp_s16(h1, g3),
                                              (int32_t)v64_dotp_s16(h0, g3)), add2), 7);
  v64_store_aligned(dst + 12, v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t)));
}


static void inverse_transform4(const int16_t *coeff, int16_t *block, int bitdepth) {
  v128 round1 = v128_dup_32(64);
  v128 round2 = v128_dup_32(1 << (19 - bitdepth));
  int shift2 = 20 - bitdepth;
  v64 c83 = v64_dup_16(83);
  v64 c36 = v64_dup_16(36);
  v64 load0 = v64_load_aligned(coeff +  0);
  v64 load1 = v64_load_aligned(coeff +  4);
  v64 load2 = v64_load_aligned(coeff +  8);
  v64 load3 = v64_load_aligned(coeff + 12);

  /* Utilizing symmetry properties to the maximum to minimise the number of multiplications */
  v128 o0 = v128_add_32(v128_mul_s16(load1, c83), v128_mul_s16(load3, c36));
  v128 o1 = v128_sub_32(v128_mul_s16(load1, c36), v128_mul_s16(load3, c83));
  v128 t1 = v128_shl_n_32(v128_unpack_s16_s32(load0), 6);
  v128 t2 = v128_shl_n_32(v128_unpack_s16_s32(load2), 6);
  v128 e0 = v128_add_32(v128_add_32(t1, t2), round1);
  v128 e1 = v128_add_32(v128_sub_32(t1, t2), round1);

  /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
  v128 d0 = v128_shr_n_s32(v128_add_32(e0, o0), 7);
  v128 d1 = v128_shr_n_s32(v128_add_32(e1, o1), 7);
  v128 d2 = v128_shr_n_s32(v128_sub_32(e1, o1), 7);
  v128 d3 = v128_shr_n_s32(v128_sub_32(e0, o0), 7);

  v128 x0 = v128_ziplo_32(d1, d0);
  v128 x1 = v128_ziphi_32(d1, d0);
  v128 x2 = v128_ziplo_32(d3, d2);
  v128 x3 = v128_ziphi_32(d3, d2);

  load0 = v64_pack_s32_s16(v128_low_v64(x2), v128_low_v64(x0));
  load1 = v64_pack_s32_s16(v128_high_v64(x2), v128_high_v64(x0));
  load2 = v64_pack_s32_s16(v128_low_v64(x3), v128_low_v64(x1));
  load3 = v64_pack_s32_s16(v128_high_v64(x3), v128_high_v64(x1));

  o0 = v128_add_32(v128_mul_s16(load1, c83), v128_mul_s16(load3, c36));
  o1 = v128_sub_32(v128_mul_s16(load1, c36), v128_mul_s16(load3, c83));

  t1 = v128_shl_n_32(v128_unpack_s16_s32(load0), 6);
  t2 = v128_shl_n_32(v128_unpack_s16_s32(load2), 6);
  e0 = v128_add_32(t1, t2);
  e1 = v128_sub_32(t1, t2);
  e0 = v128_add_32(e0, round2);
  e1 = v128_add_32(e1, round2);

  d0 = v128_shr_s32(v128_add_32(e0, o0), shift2);
  d1 = v128_shr_s32(v128_add_32(e1, o1), shift2);
  d2 = v128_shr_s32(v128_sub_32(e1, o1), shift2);
  d3 = v128_shr_s32(v128_sub_32(e0, o0), shift2);

  x0 = v128_ziplo_32(d1, d0);
  x1 = v128_ziphi_32(d1, d0);
  x2 = v128_ziplo_32(d3, d2);
  x3 = v128_ziphi_32(d3, d2);

  v64_store_aligned(block +  0, v64_pack_s32_s16(v128_low_v64(x2), v128_low_v64(x0)));
  v64_store_aligned(block +  4, v64_pack_s32_s16(v128_high_v64(x2), v128_high_v64(x0)));
  v64_store_aligned(block +  8, v64_pack_s32_s16(v128_low_v64(x3), v128_low_v64(x1)));
  v64_store_aligned(block + 12, v64_pack_s32_s16(v128_high_v64(x3), v128_high_v64(x1)));
}

static void inverse_transform8_4x4(const int16_t *coeff, int16_t *block, int bitdepth) {
  v128 t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
  v128 round = v128_dup_32(64);
  int shift2 = 20 - bitdepth;
  v128 c0  = v128_dup_32(  83 << 16  |   64);
  v128 c1  = v128_dup_32(-(36 << 16) |   64);
  v128 c2  = v128_dup_32(-(83 << 16) |   64);
  v128 c3  = v128_dup_32(  36 << 16  |   64);
  v128 c4  = v128_dup_32(  18 << 16  | (-75 & 0xffff));
  v128 c5  = v128_dup_32(  50 << 16  | (-18 & 0xffff));
  v128 c6  = v128_dup_32(-(89 << 16) |   50);
  v128 c7  = v128_dup_32(  75 << 16  |   89);
  t0 = v128_from_v64(v64_load_aligned(coeff +  8), v64_load_aligned(coeff +  0));
  t1 = v128_from_v64(v64_load_aligned(coeff + 24), v64_load_aligned(coeff + 16));
  t2 = v128_ziplo_16(t1, t0);
  t3 = v128_ziphi_16(t1, t0);

  t4 = v128_madd_s16(c0, t2);
  t5 = v128_madd_s16(c3, t2);
  t6 = v128_madd_s16(c4, t3);
  t7 = v128_madd_s16(c7, t3);
  t0 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t5, t6), round), 7),
                         v128_shr_n_s32(v128_add_32(v128_add_32(t4, t7), round), 7));
  t1 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t4, t7), round), 7),
                         v128_shr_n_s32(v128_add_32(v128_add_32(t5, t6), round), 7));

  t4 = v128_madd_s16(c1, t2);
  t5 = v128_madd_s16(c2, t2);
  t6 = v128_madd_s16(c5, t3);
  t7 = v128_madd_s16(c6, t3);
  t2 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t5, t6), round), 7),
                         v128_shr_n_s32(v128_add_32(v128_add_32(t4, t7), round), 7));
  t3 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t4, t7), round), 7),
                         v128_shr_n_s32(v128_add_32(v128_add_32(t5, t6), round), 7));

  t4 = v128_ziplo_64(t2, t0);
  t5 = v128_ziphi_64(t2, t0);
  t6 = v128_ziplo_64(t1, t3);
  t7 = v128_ziphi_64(t1, t3);

  t0 = v128_ziplo_16(t5, t4);
  t1 = v128_ziphi_16(t5, t4);
  t2 = v128_ziplo_16(t7, t6);
  t3 = v128_ziphi_16(t7, t6);

  t4 = v128_ziplo_32(t1, t0);
  t5 = v128_ziphi_32(t1, t0);
  t6 = v128_ziplo_32(t3, t2);
  t7 = v128_ziphi_32(t3, t2);

  t0 = v128_ziplo_64(t6, t4);
  t1 = v128_ziphi_64(t6, t4);
  t2 = v128_ziplo_64(t7, t5);
  t3 = v128_ziphi_64(t7, t5);

  t10 = v128_ziplo_16(t2, t0);
  t4  = v128_ziphi_16(t2, t0);
  t11 = v128_ziplo_16(t3, t1);
  t12 = v128_ziphi_16(t3, t1);

  round =  v128_dup_32(1 << (19 - bitdepth));

  t8 = v128_madd_s16(c0, t10);
  t9 = v128_madd_s16(c0, t4);
  t3 = v128_madd_s16(c7, t11);
  t7 = v128_madd_s16(c7, t12);
  t0 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32(t9, t7), round), shift2),
                         v128_shr_s32(v128_add_32(v128_add_32(t8, t3), round), shift2));
  t7 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32(t9, t7), round), shift2),
                         v128_shr_s32(v128_add_32(v128_sub_32(t8, t3), round), shift2));

  t8 = v128_madd_s16(c3, t10);
  t9 = v128_madd_s16(c3, t4);
  t3 = v128_madd_s16(c4, t11);
  t6 = v128_madd_s16(c4, t12);
  t1 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32(t9, t6), round), shift2),
                         v128_shr_s32(v128_add_32(v128_sub_32(t8, t3), round), shift2));
  t6 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32(t9, t6), round), shift2),
                         v128_shr_s32(v128_add_32(v128_add_32(t8, t3), round), shift2));

  t8 = v128_madd_s16(c1, t10);
  t9 = v128_madd_s16(c1, t4);
  t3 = v128_madd_s16(c6, t11);
  t5 = v128_madd_s16(c6, t12);
  t2 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32(t9, t5), round), shift2),
                         v128_shr_s32(v128_add_32(v128_add_32(t8, t3), round), shift2));
  t5 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32(t9, t5), round), shift2),
                         v128_shr_s32(v128_add_32(v128_sub_32(t8, t3), round), shift2));

  t8  = v128_madd_s16(c2, t10);
  t9  = v128_madd_s16(c2, t4);
  t10 = v128_madd_s16(c5, t11);
  t4  = v128_madd_s16(c5, t12);
  t3 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32(t9, t4), round), shift2),
                         v128_shr_s32(v128_add_32(v128_sub_32(t8, t10), round), shift2));
  t4 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32(t9, t4), round), shift2),
                         v128_shr_s32(v128_add_32(v128_add_32(t8, t10), round), shift2));



  /* Transpose */
  t8 = v128_ziplo_16(t1, t0);
  t9 = v128_ziplo_16(t3, t2);
  t10 = v128_ziplo_32(t9, t8);
  t11 = v128_ziphi_32(t9, t8);
  t8 = v128_ziphi_16(t1, t0);
  t0 = v128_ziphi_16(t3, t2);
  t9 = v128_ziplo_32(t0, t8);
  t8 = v128_ziphi_32(t0, t8);
  t2 = v128_ziplo_16(t5, t4);
  t3 = v128_ziplo_16(t7, t6);
  t0 = v128_ziplo_32(t3, t2);
  v128_store_aligned(block +  0, v128_ziplo_64(t0, t10));
  v128_store_aligned(block +  8, v128_ziphi_64(t0, t10));
  t0 = v128_ziphi_32(t3, t2);
  v128_store_aligned(block + 16, v128_ziplo_64(t0, t11));
  v128_store_aligned(block + 24, v128_ziphi_64(t0, t11));
  t0 = v128_ziphi_16(t5, t4);
  t1 = v128_ziphi_16(t7, t6);
  t2 = v128_ziplo_32(t1, t0);
  v128_store_aligned(block + 32, v128_ziplo_64(t2, t9));
  v128_store_aligned(block + 40, v128_ziphi_64(t2, t9));
  t0 = v128_ziphi_32(t1, t0);
  v128_store_aligned(block + 48, v128_ziplo_64(t0, t8));
  v128_store_aligned(block + 56, v128_ziphi_64(t0, t8));
}


/* Inverse transform, take advantage of symmetries to minimise operations */
static void inverse_transform8(const int16_t *coeff, int16_t *block, int bitdepth) {
  v128 t0, t1, t2, t3, t4 ,t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16;
  v128 round = v128_dup_32(64);
  int shift2 = 20 - bitdepth;
  v128 c0  = v128_dup_16(64);
  v128 c1  = v128_dup_32(-(64 << 16) |   64);
  v128 c2  = v128_dup_32(  36 << 16  |   83);
  v128 c3  = v128_dup_32(  83 << 16  | (-36 & 0xffff));
  v128 c4  = v128_dup_32(  18 << 16  | (-75 & 0xffff));
  v128 c5  = v128_dup_32(  50 << 16  | (-18 & 0xffff));
  v128 c6  = v128_dup_32(-(89 << 16) |   50);
  v128 c7  = v128_dup_32(  75 << 16  |   89);
  v128 c8  = v128_dup_32(  50 << 16  |   89);
  v128 c9  = v128_dup_32(  89 << 16  | (-75 & 0xffff));
  v128 c10 = v128_dup_32(  75 << 16  |   18);
  v128 c11 = v128_dup_32(  18 << 16  |   50);

  v128 load0 = v128_load_aligned(coeff +  0);
  v128 load1 = v128_load_aligned(coeff +  8);
  v128 load2 = v128_load_aligned(coeff + 16);
  v128 load3 = v128_load_aligned(coeff + 24);
  v128 load4 = v128_load_aligned(coeff + 32);
  v128 load5 = v128_load_aligned(coeff + 40);
  v128 load6 = v128_load_aligned(coeff + 48);
  v128 load7 = v128_load_aligned(coeff + 56);

  t0 = v128_ziplo_16(load4, load0);
  t1 = v128_ziplo_16(load6, load2);
  t2 = v128_madd_s16(c0, t0);
  t3 = v128_madd_s16(c2, t1);
  t4 = v128_add_32(t2, t3);
  t3 = v128_sub_32(t2, t3);

  t2 = v128_ziphi_16(load4, load0);
  t5 = v128_ziphi_16(load6, load2);
  t6 = v128_madd_s16(c0, t2);
  t7 = v128_madd_s16(c2, t5);
  t8 = v128_add_32(t6, t7);
  t6 = v128_sub_32(t6, t7);

  t0 = v128_madd_s16(c1, t0);
  t1 = v128_madd_s16(c3, t1);
  t9 = v128_add_32(t0, t1);
  t10 = v128_sub_32(t0, t1);

  t0 = v128_madd_s16(c1, t2);
  t1 = v128_madd_s16(c3, t5);
  t2 = v128_add_32(t0, t1);
  t5 = v128_sub_32(t0, t1);

  t0 = v128_ziplo_16(load3, load1);
  t1 = v128_ziplo_16(load7, load5);
  t7 = v128_ziphi_16(load3, load1);
  t11 = v128_ziphi_16(load7, load5);

  t12 = v128_add_32(v128_madd_s16(c4, t0), v128_madd_s16(c8, t1));
  t13 = v128_add_32(v128_madd_s16(c4, t7), v128_madd_s16(c8, t11));
  t14 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32( t5, t13), round), 7),
                          v128_shr_n_s32(v128_add_32(v128_sub_32(t10, t12), round), 7));
  t13 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32( t5, t13), round), 7),
                          v128_shr_n_s32(v128_add_32(v128_add_32(t10, t12), round), 7));

  t12 = v128_add_32(v128_madd_s16(c5, t0), v128_madd_s16(c9, t1));
  t10 = v128_add_32(v128_madd_s16(c5, t7), v128_madd_s16(c9, t11));
  t5  = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t6, t10), round), 7),
                          v128_shr_n_s32(v128_add_32(v128_sub_32(t3, t12), round), 7));
  t10 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(t6, t10), round), 7),
                          v128_shr_n_s32(v128_add_32(v128_add_32(t3, t12), round), 7));

  t12 = v128_add_32(v128_madd_s16(c6, t0), v128_madd_s16(c10, t1));
  t15 = v128_add_32(v128_madd_s16(c6, t7), v128_madd_s16(c10, t11));
  t3  = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(t2, t15), round), 7),
                          v128_shr_n_s32(v128_add_32(v128_add_32(t9, t12), round), 7));
  t6  = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t2, t15), round), 7),
                          v128_shr_n_s32(v128_add_32(v128_sub_32(t9, t12), round), 7));

  t12 = v128_add_32(v128_madd_s16(c7, t0), v128_madd_s16(c11, t1));
  t9  = v128_add_32(v128_madd_s16(c7, t7), v128_madd_s16(c11, t11));
  t2  = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(t8,  t9), round), 7),
                          v128_shr_n_s32(v128_add_32(v128_add_32(t4, t12), round), 7));
  t9  = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t8,  t9), round), 7),
                          v128_shr_n_s32(v128_add_32(v128_sub_32(t4, t12), round), 7));

  t0 = v128_ziplo_16(t14, t2);
  t1 = v128_ziphi_16(t14, t2);

  t2 = v128_ziplo_16(t5, t3);
  t3 = v128_ziphi_16(t5, t3);
  t4 = v128_ziplo_16(t6, t10);
  t5 = v128_ziphi_16(t6, t10);
  t6 = v128_ziplo_16(t9, t13);
  t7 = v128_ziphi_16(t9, t13);

  t8 = v128_ziphi_64(t2, t0);
  t9 = v128_ziplo_64(t2, t0);
  t0 = v128_unziplo_32(t8, t9);
  t2 = v128_unziphi_32(t8, t9);

  t8 = v128_ziphi_64(t3, t1);
  t9 = v128_ziplo_64(t3, t1);
  t1 = v128_unziplo_32(t8, t9);
  t3 = v128_unziphi_32(t8, t9);

  t8 = v128_ziphi_64(t6, t4);
  t9 = v128_ziplo_64(t6, t4);
  t4 = v128_unziplo_32(t8, t9);
  t6 = v128_unziphi_32(t8, t9);

  t8 = v128_ziphi_64(t7, t5);
  t9 = v128_ziplo_64(t7, t5);
  t5 = v128_unziplo_32(t8, t9);
  t7 = v128_unziphi_32(t8, t9);

  t8 = v128_ziplo_64(t2, t0);
  t0 = v128_ziphi_64(t2, t0);
  t2 = v128_ziplo_64(t3, t1);
  t1 = v128_ziphi_64(t3, t1);
  t3 = v128_ziplo_64(t6, t4);
  t4 = v128_ziphi_64(t6, t4);
  t6 = v128_ziplo_64(t7, t5);
  t7 = v128_ziphi_64(t7, t5);
  t9 = v128_ziplo_64(t3, t8);
  t10 = v128_ziplo_64(t4, t0);
  t11 = v128_ziplo_64(t6, t2);
  t12 = v128_ziplo_64(t7, t1);
  t13 = v128_ziphi_16(t11, t9);
  t14 = v128_ziphi_16(t12, t10);

  t0 = v128_ziphi_64(t4, t0);
  t1 = v128_ziphi_64(t7, t1);
  t2 = v128_ziphi_64(t6, t2);
  t3 = v128_ziphi_64(t3, t8);
  t4 = v128_ziphi_16(t0, t3);
  t5 = v128_ziphi_16(t1, t2);

  t6 = v128_madd_s16(c0, t13);
  t7 = v128_madd_s16(c2, t14);
  t8 = v128_add_32(t6, t7);
  t6 = v128_sub_32(t6, t7);
  t7 = v128_madd_s16(c1, t13);
  t13 = v128_madd_s16(c3, t14);
  t14 = v128_add_32(t7, t13);
  t7 = v128_sub_32(t7, t13);

  t9 = v128_ziplo_16(t11, t9);
  t10 = v128_ziplo_16(t12, t10);
  t11 = v128_madd_s16(c0, t9);
  t12 = v128_madd_s16(c2, t10);
  t13 = v128_add_32(t11, t12);
  t12 = v128_sub_32(t11, t12);
  t9 = v128_madd_s16(c1, t9);
  t10 = v128_madd_s16(c3, t10);
  t11 = v128_add_32(t9, t10);
  t16 = v128_sub_32(t9, t10);

  t9 = v128_ziplo_16(t0, t3);
  t10 = v128_ziplo_16(t1, t2);

  t2 = v128_add_32(v128_madd_s16(c4, t4), v128_madd_s16(c8, t5));
  t3 = v128_add_32(v128_madd_s16(c5, t4), v128_madd_s16(c9, t5));
  t15 = v128_add_32(v128_madd_s16(c6, t4), v128_madd_s16(c10, t5));
  t5 = v128_add_32(v128_madd_s16(c7, t4), v128_madd_s16(c11, t5));

  t0 = v128_add_32(v128_madd_s16(c4, t9), v128_madd_s16(c8, t10));
  t1 = v128_add_32(v128_madd_s16(c5, t9), v128_madd_s16(c9, t10));
  t4 = v128_add_32(v128_madd_s16(c6, t9), v128_madd_s16(c10, t10));
  t10 = v128_add_32(v128_madd_s16(c7, t9), v128_madd_s16(c11, t10));

  /* Get transposed results */
  round = v128_dup_32(1 << (19 - bitdepth));
  load0 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32( t8,  t5), round), shift2),
                            v128_shr_s32(v128_add_32(v128_add_32(t13, t10), round), shift2));
  load1 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32( t7,  t2), round), shift2),
                            v128_shr_s32(v128_add_32(v128_sub_32(t16,  t0), round), shift2));
  load2 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32(t14, t15), round), shift2),
                            v128_shr_s32(v128_add_32(v128_add_32(t11,  t4), round), shift2));
  load3 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32( t6,  t3), round), shift2),
                            v128_shr_s32(v128_add_32(v128_sub_32(t12,  t1), round), shift2));
  load4 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32( t6,  t3), round), shift2),
                            v128_shr_s32(v128_add_32(v128_add_32(t12,  t1), round), shift2));
  load5 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32(t14, t15), round), shift2),
                            v128_shr_s32(v128_add_32(v128_sub_32(t11,  t4), round), shift2));
  load6 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32( t7,  t2), round), shift2),
                            v128_shr_s32(v128_add_32(v128_add_32(t16,  t0), round), shift2));
  load7 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32( t8,  t5), round), shift2),
                            v128_shr_s32(v128_add_32(v128_sub_32(t13, t10), round), shift2));

  /* Transpose */
  t0 = v128_ziplo_16(load1, load0);
  t1 = v128_ziplo_16(load3, load2);
  t2 = v128_ziplo_32(t1, t0);
  t3 = v128_ziphi_32(t1, t0);
  t0 = v128_ziphi_16(load1, load0);
  t1 = v128_ziphi_16(load3, load2);
  t4 = v128_ziplo_32(t1, t0);
  t5 = v128_ziphi_32(t1, t0);
  t0 = v128_ziplo_16(load5, load4);
  t1 = v128_ziplo_16(load7, load6);
  t6 = v128_ziplo_32(t1, t0);
  v128_store_aligned(block +  0, v128_ziplo_64(t6, t2));
  v128_store_aligned(block +  8, v128_ziphi_64(t6, t2));
  t7 = v128_ziphi_32(t1, t0);
  v128_store_aligned(block + 16, v128_ziplo_64(t7, t3));
  v128_store_aligned(block + 24, v128_ziphi_64(t7, t3));
  t0 = v128_ziphi_16(load5, load4);
  t1 = v128_ziphi_16(load7, load6);
  t2 = v128_ziplo_32(t1, t0);
  v128_store_aligned(block + 32, v128_ziplo_64(t2, t4));
  v128_store_aligned(block + 40, v128_ziphi_64(t2, t4));
  t3 = v128_ziphi_32(t1, t0);
  v128_store_aligned(block + 48, v128_ziplo_64(t3, t5));
  v128_store_aligned(block + 56, v128_ziphi_64(t3, t5));
}

static void inverse_transform16(const int16_t *src, int16_t *dst, int shift) {
  int j;
  v64 c9 = v64_dup_16(9);
  v64 c25 = v64_dup_16(25);
  v64 c43 = v64_dup_16(43);
  v64 c57 = v64_dup_16(57);
  v64 c70 = v64_dup_16(70);
  v64 c80 = v64_dup_16(80);
  v64 c87 = v64_dup_16(87);
  v64 c90 = v64_dup_16(90);

  v64 c50 = v64_dup_16(50);
  v64 c18 = v64_dup_16(18);
  v64 c64 = v64_dup_16(64);
  v64 c36 = v64_dup_16(36);
  v64 c83 = v64_dup_16(83);
  v64 c89 = v64_dup_16(89);
  v64 c75 = v64_dup_16(75);

  for (j = 0; j < 16; j += 4) {
    v128 E0, E1, E2, E3, E4, E5, E6, E7;
    v128 O0, O1, O2, O3, O4, O5, O6, O7;
    v128 d0, d1, d2, d3, d4, d5, d6, d7;
    v128 o0, o1, o2, o3, o4, o5, o6, o7;
    v128 EO0, EO1, EO2, EO3, EEO0, EEO1, EEE0, EEE1;
    v128 EE0, EE1, EE2, EE3;
    v128 round, tmp, tmp1, tmp2;
    v64 s0 =  v64_load_aligned(src +   0);
    v64 s1 =  v64_load_aligned(src +  16);
    v64 s2 =  v64_load_aligned(src +  32);
    v64 s3 =  v64_load_aligned(src +  48);
    v64 s4 =  v64_load_aligned(src +  64);
    v64 s5 =  v64_load_aligned(src +  80);
    v64 s6 =  v64_load_aligned(src +  96);
    v64 s7 =  v64_load_aligned(src + 112);
    v64 s8 =  v64_load_aligned(src + 128);
    v64 s9 =  v64_load_aligned(src + 144);
    v64 s10 = v64_load_aligned(src + 160);
    v64 s11 = v64_load_aligned(src + 176);
    v64 s12 = v64_load_aligned(src + 192);
    v64 s13 = v64_load_aligned(src + 208);
    v64 s14 = v64_load_aligned(src + 224);
    v64 s15 = v64_load_aligned(src + 240);

    /* O0 = 90*src[16] + 87*src[3*16] + 80*src[5*16] + 70*src[7*16]
      + 57*src[9*16] + 43*src[11*16] + 25*src[13*16] + 9*src[15*16];*/
    O0   = v128_add_32(v128_mul_s16(s1, c90),v128_mul_s16(s3, c87));
    tmp  = v128_add_32(v128_mul_s16(s5, c80),v128_mul_s16(s7, c70));
    tmp1 = v128_add_32(v128_mul_s16(s9, c57),v128_mul_s16(s11, c43));
    tmp2 = v128_add_32(v128_mul_s16(s13, c25),v128_mul_s16(s15, c9));
    O0   = v128_add_32(O0, v128_add_32(tmp, v128_add_32(tmp1, tmp2)));

    /* O1 = 87*src[16] + 57*src[3*16] + 9*src[5*16] - 43*src[7*16]
      - 80*src[9*16] -  90*src[11*16] -  70*src[13*16] - 25*src[15*16];*/

    O1   = v128_add_32(v128_mul_s16(s1, c87),  v128_mul_s16(s3, c57));
    tmp  = v128_sub_32(v128_mul_s16(s5, c9),   v128_mul_s16(s7, c43));
    tmp1 = v128_add_32(v128_mul_s16(s9, c80),  v128_mul_s16(s11, c90));
    tmp2 = v128_add_32(v128_mul_s16(s13 ,c70), v128_mul_s16(s15, c25));
    O1   = v128_add_32(O1, tmp);
    tmp1 = v128_add_32(tmp1, tmp2);
    O1   = v128_sub_32(O1, tmp1);

    /* O2 = 80*src[16] + 9*src[3*16] - 70*src[5*16] - 87*src[7*16]
      + (-25)*src[9 * 16] + 57*src[11 * 16] + 90*src[13 * 16] + 43*src[15 * 16] */
    O2   = v128_add_32(v128_mul_s16(s1, c80), v128_mul_s16(s3, c9));
    tmp  =     v128_add_32(v128_mul_s16(s5, c70), v128_mul_s16(s7, c87));
    tmp1 =    v128_sub_32(v128_mul_s16(s11, c57) ,v128_mul_s16(s9, c25));
    tmp2 = v128_add_32(v128_mul_s16(s13, c90), v128_mul_s16(s15, c43));
    O2   = v128_sub_32(O2, tmp);
    O2   = v128_add_32(O2, v128_add_32(tmp1, tmp2));

    /* O3 = 70*src[16] - 43*src[3*16] - 87*src[5*16] + 9*src[7*16]
      + 90 * src[9 * 16] + 25 *src[11 * 16] - 80 * src[13 * 16] - 57* src[15 * 16]*/
    O3   = v128_sub_32(v128_mul_s16(s1, c70), v128_mul_s16(s3, c43));
    tmp  = v128_sub_32(v128_mul_s16(s5, c87), v128_mul_s16(s7, c9));
    O3   = v128_sub_32(O3, tmp);
    tmp1 = v128_add_32(v128_mul_s16(s9, c90), v128_mul_s16(s11, c25));
    tmp2 = v128_add_32(v128_mul_s16(s13, c80), v128_mul_s16(s15, c57));
    O3   = v128_add_32(O3, v128_sub_32(tmp1, tmp2));

    /* O4 = 57*src[16] - 80*src[3*16] - 25*src[5*16] + 90*src[7*16]
      = - 9 * src[9 * 16] -87 * src[11 * 16] + 43 * src[13 * 16] + 70 * src[15 * 16]*/
    O4   = v128_sub_32(v128_mul_s16(s1, c57), v128_mul_s16(s3, c80));
    tmp  = v128_sub_32(v128_mul_s16(s5, c25), v128_mul_s16(s7, c90));
    O4   = v128_sub_32(O4, tmp);
    tmp1 = v128_add_32(v128_mul_s16(s9, c9),v128_mul_s16(s11, c87));
    tmp2 = v128_add_32(v128_mul_s16(s13, c43),v128_mul_s16(s15, c70));
    O4   = v128_add_32(O4, tmp2);
    O4   = v128_sub_32(O4, tmp1);

    /* O5 = 43*src[16] - 90*src[3*16] + 57*src[5*16] + 25*src[7*16]
      - 87 * src[9 * 16] + 70 * src[11 * 16] + 9 * src[13 * 16] - 80 * src[15 * 16]*/
    O5   = v128_sub_32(v128_mul_s16(s1, c43), v128_mul_s16(s3, c90));
    tmp  = v128_add_32(v128_mul_s16(s5, c57), v128_mul_s16(s7, c25));
    O5   = v128_add_32(O5, tmp);
    tmp1 = v128_sub_32(v128_mul_s16(s11, c70), v128_mul_s16(s9, c87));
    tmp2 = v128_sub_32(v128_mul_s16(s13, c9), v128_mul_s16(s15, c80));
    O5   = v128_add_32(O5, v128_add_32(tmp1, tmp2));

    /* O6 = 25*src[16] - 70*src[3*16] + 90*src[5*16] - 80*src[7*16]
      + 43* src[9 * 16] + 9 * src[11 * 16] -57 * src[13 * 16] + 87 * src[15 * 16]*/
    O6   = v128_sub_32(v128_mul_s16(s1, c25), v128_mul_s16(s3, c70));
    tmp  = v128_sub_32(v128_mul_s16(s5, c90), v128_mul_s16(s7, c80));
    O6   = v128_add_32(O6, tmp);
    tmp1 = v128_add_32(v128_mul_s16(s9, c43), v128_mul_s16(s11, c9));
    tmp2 = v128_sub_32(v128_mul_s16(s15, c87), v128_mul_s16(s13, c57));
    O6   = v128_add_32(O6, v128_add_32(tmp1, tmp2));

    /* O7 = 9*src[16] - 25*src[3*16] + 43*src[5*16] - 57*src[7*16]
      + 70 * src[9 * 16] -80 * src[11 * 16] + 87 * src[13 * 16] - 90 * src[15 * 16]*/
    O7   = v128_sub_32(v128_mul_s16(s1, c9), v128_mul_s16(s3, c25));
    tmp  = v128_sub_32(v128_mul_s16(s5, c43), v128_mul_s16(s7, c57));
    O7   = v128_add_32(O7, tmp);
    tmp1 = v128_sub_32(v128_mul_s16(s9, c70), v128_mul_s16(s11, c80));
    tmp2 = v128_sub_32(v128_mul_s16(s13, c87), v128_mul_s16(s15, c90));
    O7   = v128_add_32(O7, v128_add_32(tmp1, tmp2));

    /* EO[0] = g3mat[ 2][0]*src[ 2*16] + g3mat[ 6][0]*src[ 6*16]
      + g3mat[10][0]*src[10*16] + g3mat[14][0]*src[14*16];*/
    EO0 = v128_add_32(v128_mul_s16(s2, c89), v128_mul_s16(s6, c75));
    tmp1 = v128_add_32(v128_mul_s16(s10, c50), v128_mul_s16(s14, c18));
    EO0 = v128_add_32(EO0, tmp1);

    /* EO[1] = g3mat[ 2][1]*src[ 2*16] + g3mat[ 6][1]*src[ 6*16]
      + g3mat[10][1]*src[10*16] + g3mat[14][1]*src[14*16];*/
    EO1 = v128_sub_32(v128_mul_s16(s2, c75), v128_mul_s16(s6, c18));
    tmp1 = v128_add_32(v128_mul_s16(s10, c89), v128_mul_s16(s14, c50));
    EO1 = v128_sub_32(EO1, tmp1);

    /* EO[2] = g3mat[ 2][2]*src[ 2*16] + g3mat[ 6][2]*src[ 6*16]
      + g3mat[10][2]*src[10*16] + g3mat[14][2]*src[14*16];*/
    EO2 = v128_sub_32(v128_mul_s16(s2, c50), v128_mul_s16(s6, c89));
    tmp1 = v128_add_32(v128_mul_s16(s10, c18), v128_mul_s16(s14, c75));
    EO2 = v128_add_32(EO2, tmp1);

    /* EO[3] = g3mat[ 2][3]*src[ 2*16] + g3mat[ 6][3]*src[ 6*16]
      + g3mat[10][3]*src[10*16] + g3mat[14][3]*src[14*16];*/
    EO3 = v128_sub_32(v128_mul_s16(s2, c18), v128_mul_s16(s6, c50));
    tmp1 = v128_sub_32(v128_mul_s16(s10, c75), v128_mul_s16(s14, c89));
    EO3 = v128_add_32(EO3, tmp1);

    /* ee0 */
    /* EEO[0] = g3mat[4][0]*src[ 4*16 ] + g3mat[12][0]*src[ 12*16 ];*/
    EEO0 = v128_add_32(v128_mul_s16(s4, c83), v128_mul_s16(s12, c36));
    /* EEE[0] = g3mat[0][0]*src[ 0      ] + g3mat[ 8][0]*src[ 8*16  ];*/
    EEE0 = v128_add_32(v128_mul_s16(s0, c64), v128_mul_s16(s8, c64));
    /* EEO[1] = g3mat[4][1]*src[ 4*16 ] + g3mat[12][1]*src[ 12*16 ];*/
    EEO1 = v128_sub_32(v128_mul_s16(s4, c36), v128_mul_s16(s12, c83));
    /* EEE[1] = g3mat[0][1]*src[ 0      ] + g3mat[ 8][1]*src[ 8*16  ];*/
    EEE1 = v128_sub_32(v128_mul_s16(s0, c64), v128_mul_s16(s8, c64));

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EE0 = v128_add_32(EEE0, EEO0);
    EE3 = v128_sub_32(EEE0, EEO0);
    EE1 = v128_add_32(EEE1, EEO1);
    EE2 = v128_sub_32(EEE1, EEO1);

    round = v128_dup_32(1 << (shift-1));
    E0 = v128_add_32(round, v128_add_32(EE0, EO0));
    E7 = v128_add_32(round, v128_sub_32(EE0, EO0));
    E4 = v128_add_32(round, v128_sub_32(EE3, EO3));
    E3 = v128_add_32(round, v128_add_32(EE3, EO3));
    E1 = v128_add_32(round, v128_add_32(EE1, EO1));
    E6 = v128_add_32(round, v128_sub_32(EE1, EO1));
    E5 = v128_add_32(round, v128_sub_32(EE2, EO2));
    E2 = v128_add_32(round, v128_add_32(EE2, EO2));

    d0 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(E1, O1), shift),
                           v128_shr_s32(v128_add_32(E0, O0), shift));
    d1 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(E3, O3), shift),
                           v128_shr_s32(v128_add_32(E2, O2), shift));
    d2 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(E5, O5), shift),
                           v128_shr_s32(v128_add_32(E4, O4), shift));
    d3 = v128_pack_s32_s16(v128_shr_s32(v128_add_32(E7, O7), shift),
                           v128_shr_s32(v128_add_32(E6, O6), shift));
    d4 = v128_pack_s32_s16(v128_shr_s32(v128_sub_32(E6, O6), shift),
                           v128_shr_s32(v128_sub_32(E7, O7), shift));
    d5 = v128_pack_s32_s16(v128_shr_s32(v128_sub_32(E4, O4), shift),
                           v128_shr_s32(v128_sub_32(E5, O5), shift));
    d6 = v128_pack_s32_s16(v128_shr_s32(v128_sub_32(E2, O2), shift),
                           v128_shr_s32(v128_sub_32(E3, O3), shift));
    d7 = v128_pack_s32_s16(v128_shr_s32(v128_sub_32(E0, O0), shift),
                           v128_shr_s32(v128_sub_32(E1, O1), shift));

    o0 = v128_ziplo_16(d1, d0);
    o1 = v128_ziphi_16(d1, d0);
    o2 = v128_ziplo_16(d3, d2);
    o3 = v128_ziphi_16(d3, d2);
    o4 = v128_ziplo_16(d5, d4);
    o5 = v128_ziphi_16(d5, d4);
    o6 = v128_ziplo_16(d7, d6);
    o7 = v128_ziphi_16(d7, d6);

    d0 = v128_ziplo_16(o1, o0);
    d1 = v128_ziphi_16(o1, o0);
    d2 = v128_ziplo_16(o3, o2);
    d3 = v128_ziphi_16(o3, o2);
    d4 = v128_ziplo_16(o5, o4);
    d5 = v128_ziphi_16(o5, o4);
    d6 = v128_ziplo_16(o7, o6);
    d7 = v128_ziphi_16(o7, o6);

    v128_store_aligned(dst +  0, v128_from_v64(v128_low_v64(d2), v128_low_v64(d0)));
    v128_store_aligned(dst +  8, v128_from_v64(v128_low_v64(d6), v128_low_v64(d4)));
    v128_store_aligned(dst + 16, v128_from_v64(v128_high_v64(d2), v128_high_v64(d0)));
    v128_store_aligned(dst + 24, v128_from_v64(v128_high_v64(d6), v128_high_v64(d4)));
    v128_store_aligned(dst + 32, v128_from_v64(v128_low_v64(d3), v128_low_v64(d1)));
    v128_store_aligned(dst + 40, v128_from_v64(v128_low_v64(d7), v128_low_v64(d5)));
    v128_store_aligned(dst + 48, v128_from_v64(v128_high_v64(d3), v128_high_v64(d1)));
    v128_store_aligned(dst + 56, v128_from_v64(v128_high_v64(d7), v128_high_v64(d5)));
    dst += 64;
    src += 4;
  }
}

/* 16x16 inverse transform assuming everything but top left 4x4 is 0 */
static void inverse_transform16_4x4(const int16_t *coeff, int16_t *block, int bitdepth) {
  static const ALIGN(16) int16_t c[] = {
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
  v128 t[8];
  v128 t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11;
  v128 round = v128_dup_32(64);
  int shift2 = 20 - bitdepth;
  v128 load0 = v128_from_v64(v64_load_aligned(coeff + 16), v64_load_aligned(coeff +  0));
  v128 load1 = v128_from_v64(v64_load_aligned(coeff + 48), v64_load_aligned(coeff + 32));
  v128 lo = v128_ziplo_16(load1, load0);
  v128 hi = v128_ziphi_16(load1, load0);
  int i, j;

  for (i = 0; i < 4; i++) {
    v128 t0 = v128_madd_s16(v128_load_aligned(c + i*32 +  0), lo);
    v128 t1 = v128_madd_s16(v128_load_aligned(c + i*32 +  8), hi);
    v128 t2 = v128_madd_s16(v128_load_aligned(c + i*32 + 16), lo);
    v128 t3 = v128_madd_s16(v128_load_aligned(c + i*32 + 24), hi);
    t[i*2+0] = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t2, t3), round), 7),
                                 v128_shr_n_s32(v128_add_32(v128_add_32(t0, t1), round), 7));
    t[i*2+1] = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t0, t1), round), 7),
                                 v128_shr_n_s32(v128_add_32(v128_add_32(t2, t3), round), 7));
  }

  t0 = v128_ziplo_64(t[2], t[0]);
  t1 = v128_ziphi_64(t[2], t[0]);
  t2 = v128_ziplo_16(t1, t0);
  t3 = v128_ziphi_16(t1, t0);
  t4 = v128_ziplo_32(t3, t2);
  t6 = v128_ziphi_32(t3, t2);

  t0 = v128_ziplo_64(t[6], t[4]);
  t1 = v128_ziphi_64(t[6], t[4]);
  t2 = v128_ziplo_16(t1, t0);
  t3 = v128_ziphi_16(t1, t0);
  t8 = v128_ziplo_32(t3, t2);
  t10 = v128_ziphi_32(t3, t2);

  t0 = v128_ziplo_64(t[5], t[7]);
  t1 = v128_ziphi_64(t[5], t[7]);
  t2 = v128_ziplo_16(t1, t0);
  t3 = v128_ziphi_16(t1, t0);
  t9 = v128_ziplo_32(t3, t2);
  t11 = v128_ziphi_32(t3, t2);

  t0 = v128_ziplo_64(t[1], t[3]);
  t1 = v128_ziphi_64(t[1], t[3]);
  t2 = v128_ziplo_16(t1, t0);
  t3 = v128_ziphi_16(t1, t0);
  t5 = v128_ziplo_32(t3, t2);
  t7 = v128_ziphi_32(t3, t2);

  t0 = v128_ziplo_64(t5, t9);
  t1 = v128_ziphi_64(t5, t9);
  t2 = v128_ziplo_64(t8, t4);
  t3 = v128_ziphi_64(t8, t4);
  t4 = v128_ziplo_64(t10, t6);
  t5 = v128_ziphi_64(t10, t6);
  t6 = v128_ziplo_64(t7, t11);
  t7 = v128_ziphi_64(t7, t11);

  t8 = v128_ziplo_16(t7, t1);
  t7 = v128_ziphi_16(t7, t1);
  t1 = v128_ziplo_16(t6, t0);
  t0 = v128_ziphi_16(t6, t0);
  t6 = v128_ziplo_16(t4, t2);
  t10 = v128_ziphi_16(t4, t2);
  t11 = v128_ziplo_16(t5, t3);
  t3 = v128_ziphi_16(t5, t3);

  round = v128_dup_32(1 << (19 - bitdepth));

  for (i = 0; i < 8; i++) {
    int i1 = i*16;
    int i2 = 240 - i1;
    int i3 = i & 1 ? i2 - 120 : i1;
    int i4 = i & 1 ? i1 : i2 - 120;
    int i5 = i & 1 ? i2 : i1 + 120;
    int i6 = i & 1 ? i1 + 120 : i2;
    v128 c0 = v128_load_aligned(c + i1 + 0);
    v128 c1 = v128_load_aligned(c + i1 + 8);
    v128 v0 = v128_madd_s16(c0, t10);
    v128 v1 = v128_madd_s16(c0, t6);
    v128 v2 = v128_madd_s16(c1, t11);
    v128 v3 = v128_madd_s16(c1, t3);
    v128_store_aligned(block + i3,
                       v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32(v0, v3), round), shift2),
                                         v128_shr_s32(v128_add_32(v128_add_32(v1, v2), round), shift2)));
    v128_store_aligned(block + i4,
                       v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32(v0, v3), round), shift2),
                                         v128_shr_s32(v128_add_32(v128_sub_32(v1, v2), round), shift2)));

    v0 = v128_madd_s16(c0, t0);
    v1 = v128_madd_s16(c0, t1);
    v2 = v128_madd_s16(c1, t8);
    v3 = v128_madd_s16(c1, t7);
    v128_store_aligned(block + 8 + i5,
                       v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32(v0, v3), round), shift2),
                                         v128_shr_s32(v128_add_32(v128_add_32(v1, v2), round), shift2)));
    v128_store_aligned(block + 8 + i6,
                       v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32(v0, v3), round), shift2),
                                         v128_shr_s32(v128_add_32(v128_sub_32(v1, v2), round), shift2)));
  }

  for (i = 0; i < 16; i += 8)
    for (j = 0; j < 16; j += 8) {
      int16_t *p = block + i*16 + j;
      v128 load0 = v128_load_aligned(p +   0);
      v128 load1 = v128_load_aligned(p +  16);
      v128 load2 = v128_load_aligned(p +  32);
      v128 load3 = v128_load_aligned(p +  48);
      v128 load4 = v128_load_aligned(p +  64);
      v128 load5 = v128_load_aligned(p +  80);
      v128 load6 = v128_load_aligned(p +  96);
      v128 load7 = v128_load_aligned(p + 112);
      v128 t3  = v128_ziplo_16(load1, load0);
      v128 t4  = v128_ziphi_16(load1, load0);
      v128 t7  = v128_ziplo_16(load3, load2);
      v128 t8  = v128_ziphi_16(load3, load2);
      v128 t11 = v128_ziplo_16(load5, load4);
      v128 t12 = v128_ziphi_16(load5, load4);
      v128 t15 = v128_ziplo_16(load7, load6);
      v128 t16 = v128_ziphi_16(load7, load6);
      v128 t17 = v128_ziplo_32(t7, t3);
      v128 t18 = v128_ziphi_32(t7, t3);
      v128 t19 = v128_ziplo_32(t8, t4);
      v128 t20 = v128_ziphi_32(t8, t4);
      v128 t21 = v128_ziplo_32(t15, t11);
      v128 t22 = v128_ziphi_32(t15, t11);
      v128 t23 = v128_ziplo_32(t16, t12);
      v128 t24 = v128_ziphi_32(t16, t12);
      v128_store_aligned(p +   0, v128_ziplo_64(t21, t17));
      v128_store_aligned(p +  16, v128_ziphi_64(t21, t17));
      v128_store_aligned(p +  32, v128_ziplo_64(t22, t18));
      v128_store_aligned(p +  48, v128_ziphi_64(t22, t18));
      v128_store_aligned(p +  64, v128_ziplo_64(t23, t19));
      v128_store_aligned(p +  80, v128_ziphi_64(t23, t19));
      v128_store_aligned(p +  96, v128_ziplo_64(t24, t20));
      v128_store_aligned(p + 112, v128_ziphi_64(t24, t20));
    }
}

static void transform_1d_32(const int16_t *coeff, const int16_t *tcoeff, int j, int16_t *out, int shift)
{
  v128 t0, t1, t2, t3, t4;
  v128 c = v128_load_aligned(tcoeff + j*8);
  v128 rev32 = v128_from_64(0x0302010007060504LL, 0x0b0a09080f0e0d0cLL);

  t0 = v128_madd_s16(v128_from_64(0x00430049004e0052LL, 0x00550058005a005aLL), c);
  t1 = v128_madd_s16(v128_from_64(0xffcaffe1fffc0016LL, 0x002e00430052005aLL), c);
  t2 = v128_madd_s16(v128_from_64(0xffb2ffa6ffaeffcaLL, 0xfff3001f00430058LL), c);
  t3 = v128_madd_s16(v128_from_64(0x0026ffeaffb7ffa6LL, 0xffbdfff3002e0055LL), c);
  t4 = v128_ziplo_32(t0, t1);
  t1 = v128_ziphi_32(t0, t1);
  t0 = v128_ziplo_32(t2, t3);
  t3 = v128_ziphi_32(t2, t3);
  v128 o0 = v128_add_32(v128_add_32(v128_ziplo_64(t4, t0), v128_ziphi_64(t4, t0)),
                        v128_add_32(v128_ziplo_64(t1, t3), v128_ziphi_64(t1, t3)));

  t0 = v128_madd_s16(v128_from_64(0x0055004e000dffc3LL, 0xffa6ffca00160052LL), c);
  t1 = v128_madd_s16(v128_from_64(0xffea00430055000dLL, 0xffb7ffaefffc004eLL), c);
  t2 = v128_madd_s16(v128_from_64(0xffa6ffda0043004eLL, 0xffeaffa6ffe10049LL), c);
  t3 = v128_madd_s16(v128_from_64(0x0004ffa6ffea0055LL, 0x0026ffb2ffca0043LL), c);
  t4 = v128_ziplo_32(t0, t1);
  t1 = v128_ziphi_32(t0, t1);
  t0 = v128_ziplo_32(t2, t3);
  t3 = v128_ziphi_32(t2, t3);
  v128 o1 = v128_add_32(v128_add_32(v128_ziplo_64(t4, t0), v128_ziphi_64(t4, t0)),
                        v128_add_32(v128_ziplo_64(t1, t3), v128_ziphi_64(t1, t3)));

  t0 = v128_madd_s16(v128_from_64(0x005afff3ffa8001fLL, 0x0052ffd2ffb7003dLL), c);
  t1 = v128_madd_s16(v128_from_64(0x000d0052ffc3ffd2LL, 0x0058fffcffab0036LL), c);
  t2 = v128_madd_s16(v128_from_64(0xffa8003d001fffa6LL, 0x00360026ffa6002eLL), c);
  t3 = v128_madd_s16(v128_from_64(0xffe1ffd2005affbdLL, 0xfffc0049ffa80026LL), c);
  t4 = v128_ziplo_32(t0, t1);
  t1 = v128_ziphi_32(t0, t1);
  t0 = v128_ziplo_32(t2, t3);
  t3 = v128_ziphi_32(t2, t3);
  v128 o2 = v128_add_32(v128_add_32(v128_ziplo_64(t4, t0), v128_ziphi_64(t4, t0)),
                        v128_add_32(v128_ziplo_64(t1, t3), v128_ziphi_64(t1, t3)));

  t0 = v128_madd_s16(v128_from_64(0x0052ffa800360004LL, 0xffc3005affb2001fLL), c);
  t1 = v128_madd_s16(v128_from_64(0x002efffcffda0049LL, 0xffa60055ffc30016LL), c);
  t2 = v128_madd_s16(v128_from_64(0xffb70055ffa60058LL, 0xffb2003dffda000dLL), c);
  t3 = v128_madd_s16(v128_from_64(0xffc30036ffd20026LL, 0xffe10016fff30004LL), c);
  t4 = v128_ziplo_32(t0, t1);
  t1 = v128_ziphi_32(t0, t1);
  t0 = v128_ziplo_32(t2, t3);
  t3 = v128_ziphi_32(t2, t3);
  v128 o3 = v128_add_32(v128_add_32(v128_ziplo_64(t4, t0), v128_ziphi_64(t4, t0)),
                        v128_add_32(v128_ziplo_64(t1, t3), v128_ziphi_64(t1, t3)));

  v64 c64 = v64_from_16(coeff[j+14*32], coeff[j+10*32], coeff[j+6*32], coeff[j+2*32]);
  c = v128_from_v64(c64, c64);
  t0 = v128_madd_s16(c, v128_from_64(0xffd5000900390057LL, 0x004600500057005aLL));
  t1 = v128_madd_s16(c, v128_from_64(0x0009ffa9ffd50046LL, 0xffa9ffba00090050LL));
  v128 oo0 = v128_unziphi_32(v128_add_32(t1, v128_shl_n_byte(t1, 4)),
                             v128_add_32(t0, v128_shl_n_byte(t0, 4)));
  t0 = v128_madd_s16(c, v128_from_64(0x00190039ffa6002bLL, 0x005affe7ffb00039LL));
  t1 = v128_madd_s16(c, v128_from_64(0xffc7002bffe70009LL, 0xffb0005affba0019LL));
  v128 oo1 = v128_unziphi_32(v128_add_32(t1, v128_shl_n_byte(t1, 4)),
                             v128_add_32(t0, v128_shl_n_byte(t0, 4)));

  v128 to = v128_madd_s16(v128_dup_32((coeff[j+12*32] << 16) + (coeff[j+4*32] & 0xffff)),
                          v128_from_64(0xffce0012ffa70032LL, 0xffee004b004b0059LL));
  int32_t ooo0 = 0x53*coeff[j+8*32];
  int32_t ooo1 = 0x24*coeff[j+8*32];
  int32_t eee = coeff[j] << 6;
  v128 te = v128_from_32(eee - ooo0, eee - ooo1, eee + ooo1, eee + ooo0);
  v128 ee0 = v128_add_32(te, to);
  v128 ee1 = v128_shuffle_8(v128_sub_32(te, to), rev32);

  v128 e0 = v128_add_32(ee0, oo0);
  v128 e1 = v128_add_32(ee1, oo1);
  v128 e2 = v128_sub_32(ee1, oo1);
  v128 e3 = v128_sub_32(ee0, oo0);
  v128 e0r = v128_shuffle_8(e0, rev32);
  v128 e1r = v128_shuffle_8(e1, rev32);
  v128 e2r = v128_shuffle_8(e2, rev32);
  v128 e3r = v128_shuffle_8(e3, rev32);
  v128 o0r = v128_shuffle_8(o0, rev32);
  v128 o1r = v128_shuffle_8(o1, rev32);
  v128 o2r = v128_shuffle_8(o2, rev32);
  v128 o3r = v128_shuffle_8(o3, rev32);

  c = v128_dup_32(1 << (shift-1));
  v128_store_aligned(out + 0, v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32(e1, o1r), c), shift),
                                                v128_shr_s32(v128_add_32(v128_add_32(e0, o0r), c), shift)));
  v128_store_aligned(out + 8, v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_add_32(e3r, o3r), c), shift),
                                                v128_shr_s32(v128_add_32(v128_add_32(e2r, o2r), c), shift)));
  v128_store_aligned(out + 16, v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32(e2, o2), c), shift),
                                                 v128_shr_s32(v128_add_32(v128_sub_32(e3, o3), c), shift)));
  v128_store_aligned(out + 24, v128_pack_s32_s16(v128_shr_s32(v128_add_32(v128_sub_32(e0r, o0), c), shift),
                                                 v128_shr_s32(v128_add_32(v128_sub_32(e1r, o1), c), shift)));
}


static void transpose8x8(const int16_t *src, int sstride, int16_t *dst, int dstride)
{
  v128 i0 = v128_load_aligned(src + sstride*0);
  v128 i1 = v128_load_aligned(src + sstride*1);
  v128 i2 = v128_load_aligned(src + sstride*2);
  v128 i3 = v128_load_aligned(src + sstride*3);
  v128 i4 = v128_load_aligned(src + sstride*4);
  v128 i5 = v128_load_aligned(src + sstride*5);
  v128 i6 = v128_load_aligned(src + sstride*6);
  v128 i7 = v128_load_aligned(src + sstride*7);

  v128 t0 = v128_ziplo_16(i1, i0);
  v128 t1 = v128_ziplo_16(i3, i2);
  v128 t2 = v128_ziplo_16(i5, i4);
  v128 t3 = v128_ziplo_16(i7, i6);
  v128 t4 = v128_ziphi_16(i1, i0);
  v128 t5 = v128_ziphi_16(i3, i2);
  v128 t6 = v128_ziphi_16(i5, i4);
  v128 t7 = v128_ziphi_16(i7, i6);

  i0 = v128_ziplo_32(t1, t0);
  i1 = v128_ziplo_32(t3, t2);
  i2 = v128_ziplo_32(t5, t4);
  i3 = v128_ziplo_32(t7, t6);
  i4 = v128_ziphi_32(t1, t0);
  i5 = v128_ziphi_32(t3, t2);
  i6 = v128_ziphi_32(t5, t4);
  i7 = v128_ziphi_32(t7, t6);
  v128_store_aligned(dst + dstride*0, v128_ziplo_64(i1, i0));
  v128_store_aligned(dst + dstride*1, v128_ziphi_64(i1, i0));
  v128_store_aligned(dst + dstride*2, v128_ziplo_64(i5, i4));
  v128_store_aligned(dst + dstride*3, v128_ziphi_64(i5, i4));
  v128_store_aligned(dst + dstride*4, v128_ziplo_64(i3, i2));
  v128_store_aligned(dst + dstride*5, v128_ziphi_64(i3, i2));
  v128_store_aligned(dst + dstride*6, v128_ziplo_64(i7, i6));
  v128_store_aligned(dst + dstride*7, v128_ziphi_64(i7, i6));
}

static void inverse_transform32(const int16_t * coeff, int16_t *block, int bitdepth)
{
  int16_t *tmp = thor_alloc(32*32*2, 32);
  int16_t *tcoeff = thor_alloc(8*32*2, 32);
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

static void transform8(const int16_t *src, int16_t *dst, int shift)
{
  int j;

  const v128 shuffle1 = v128_from_64(0x0f0e0d0c0b0a0908LL, 0x0100030205040706LL);
  const v128 shuffle2 = v128_from_64(0x0d0c0f0e09080b0aLL, 0x0504070601000302LL);
  const v128 shuffle3 = v128_from_64(0x0302010007060504LL, 0x0b0a09080f0e0d0cLL);
  const v64 shuffle4 = v64_from_64(0x0100030205040706LL);
  const v128 round = v128_dup_32(1 << (shift-1));

  for (j = 0; j < 8; j += 4) {
    v64 c;
    v128 t, EO, EEE, EEO, E, E_, O, O_;

    /* E and O*/
    v128 load0 = v128_shuffle_8(v128_load_aligned(src + 0), shuffle1);
    v128 load1 = v128_shuffle_8(v128_load_aligned(src + 8), shuffle1);
    v128 hi = v128_ziphi_64(load0, load1);
    v128 lo = v128_ziplo_64(load0, load1);
    E = v128_add_16(lo, hi);
    O = v128_sub_16(lo, hi);
    load0 = v128_shuffle_8(v128_load_aligned(src + 16), shuffle1);
    load1 = v128_shuffle_8(v128_load_aligned(src + 24), shuffle1);
    hi = v128_ziphi_64(load0, load1);
    lo = v128_ziplo_64(load0, load1);
    E_ = v128_add_16(lo, hi);
    O_ = v128_sub_16(lo, hi);

    /* EO */
    EO = v128_sub_16(v128_unziphi_32(E, E_),
                     v128_unziplo_32(v128_shuffle_8(E, shuffle2), v128_shuffle_8(E_, shuffle2)));
    EO = v128_shuffle_8(EO, shuffle3);

    /* EEE and EEO */
    c = v64_dup_16(64);
    EEE = v128_from_32((int32_t)v64_dotp_s16(v128_low_v64(E_), c),
                       (int32_t)v64_dotp_s16(v128_high_v64(E_), c),
                       (int32_t)v64_dotp_s16(v128_low_v64(E), c),
                       (int32_t)v64_dotp_s16(v128_high_v64(E), c));
    c = v64_from_64(0x0040ffc0ffc00040LL);
    EEO = v128_from_32((int32_t)v64_dotp_s16(v128_low_v64(E_), c),
                       (int32_t)v64_dotp_s16(v128_high_v64(E_), c),
                       (int32_t)v64_dotp_s16(v128_low_v64(E), c),
                       (int32_t)v64_dotp_s16(v128_high_v64(E), c));

    t = v128_shr_s32(v128_add_32(EEE, round), shift);
    v64_store_aligned(dst+0*8, v128_low_v64(v128_unziplo_16(t, t)));

    t = v128_shr_s32(v128_add_32(EEO, round), shift);
    v64_store_aligned(dst+4*8, v128_low_v64(v128_unziplo_16(t, t)));

    t = v128_shr_s32(v128_add_32(v128_madd_s16(v128_dup_32((83 << 16) | (uint16_t)36), EO), round), shift);
    v64_store_aligned(dst+2*8, v128_low_v64(v128_unziplo_16(t, t)));

    t = v128_shr_s32(v128_add_32(v128_madd_s16(v128_dup_32((36 << 16) | (uint16_t)-83), EO), round), shift);
    v64_store_aligned(dst+6*8, v128_low_v64(v128_unziplo_16(t, t)));

    hi = v128_unziphi_32(O, O_);
    lo = v128_unziplo_32(O, O_);

    t = v128_shr_s32(v128_add_32(v128_add_32(v128_madd_s16(v128_dup_32(( 89 << 16) | (uint16_t)75), hi),
                                             v128_madd_s16(v128_dup_32(( 50 << 16) | (uint16_t)18), lo)), round), shift);
    v64_store_aligned(dst+1*8, v64_shuffle_8(v128_low_v64(v128_unziplo_16(t, t)), shuffle4));

    t = v128_shr_s32(v128_add_32(v128_add_32(v128_madd_s16(v128_dup_32((  75 << 16)  | (uint16_t)-18), hi),
                                             v128_madd_s16(v128_dup_32((-(89 << 16)) | (uint16_t)-50), lo)), round), shift);
    v64_store_aligned(dst+3*8, v64_shuffle_8(v128_low_v64(v128_unziplo_16(t, t)), shuffle4));

    t = v128_shr_s32(v128_add_32(v128_add_32(v128_madd_s16(v128_dup_32((50 << 16) | (uint16_t)-89), hi),
                                             v128_madd_s16(v128_dup_32((18 << 16) | (uint16_t) 75), lo)), round), shift);
    v64_store_aligned(dst+5*8, v64_shuffle_8(v128_low_v64(v128_unziplo_16(t, t)), shuffle4));

    t = v128_shr_s32(v128_add_32(v128_add_32(v128_madd_s16(v128_dup_32((18 << 16) | (uint16_t)-50), hi),
                                             v128_madd_s16(v128_dup_32((75 << 16) | (uint16_t)-89), lo)), round), shift);
    v64_store_aligned(dst+7*8, v64_shuffle_8(v128_low_v64(v128_unziplo_16(t, t)), shuffle4));

    src += 8*4;
    dst += 4;
  }
}

/* 16x16 transform, one dimension, partial butterfly */
static void transform16(const int16_t *src, int16_t *dst, int shift)
{
  int j;
  const v128 shuffle1 = v128_from_64(0x0100030205040706LL, 0x09080b0a0d0c0f0eLL);
  const v128 shuffle2 = v128_from_64(0x0302010007060504LL, 0x0b0a09080f0e0d0cLL);
  const v128 round = v128_dup_32(1 << (shift-1));

  for (j = 0; j < 16; j += 4) {
    v128 t0, t1, t2, t3, t4, t5, t6, t7;
    v128 EEE0, EEE1, EEO0, EEO1;
    v128 EE0, EE1, EE2, EE3;
    v128 EO0, EO1, EO2, EO3;

    v128 load0 = v128_load_aligned(src + 0);
    v128 load1 = v128_shuffle_8(v128_load_aligned(src + 8), shuffle1);
    v128 load2 = v128_load_aligned(src + 16);
    v128 load3 = v128_shuffle_8(v128_load_aligned(src + 24), shuffle1);
    v128 load4 = v128_load_aligned(src + 32);
    v128 load5 = v128_shuffle_8(v128_load_aligned(src + 40), shuffle1);
    v128 load6 = v128_load_aligned(src + 48);
    v128 load7 = v128_shuffle_8(v128_load_aligned(src + 56), shuffle1);

    /* E and O*/
    v128 E0 = v128_add_16(load0, load1);
    v128 O0 = v128_sub_16(load0, load1);
    v128 E1 = v128_add_16(load2, load3);
    v128 O1 = v128_sub_16(load2, load3);
    v128 E2 = v128_add_16(load4, load5);
    v128 O2 = v128_sub_16(load4, load5);
    v128 E3 = v128_add_16(load6, load7);
    v128 O3 = v128_sub_16(load6, load7);

    /* EE and EO */
    v128 hi = v128_unpackhi_s16_s32(E0);
    v128 lo = v128_shuffle_8(v128_unpacklo_s16_s32(E0), shuffle2);
    EE0 = v128_add_32(lo, hi);
    EO0 = v128_sub_32(lo, hi);
    hi = v128_unpackhi_s16_s32(E1);
    lo = v128_shuffle_8(v128_unpacklo_s16_s32(E1), shuffle2);
    EE1 = v128_add_32(lo, hi);
    EO1 = v128_sub_32(lo, hi);
    hi = v128_unpackhi_s16_s32(E2);
    lo = v128_shuffle_8(v128_unpacklo_s16_s32(E2), shuffle2);
    EE2 = v128_add_32(lo, hi);
    EO2 = v128_sub_32(lo, hi);
    hi = v128_unpackhi_s16_s32(E3);
    lo = v128_shuffle_8(v128_unpacklo_s16_s32(E3), shuffle2);
    EE3 = v128_add_32(lo, hi);
    EO3 = v128_sub_32(lo, hi);

    t0 = v128_ziphi_64(EO1, EO0);
    t1 = v128_ziphi_64(EO3, EO2);
    t2 = v128_ziplo_64(EO1, EO0);
    t3 = v128_ziplo_64(EO3, EO2);
    EO0 = v128_unziphi_32(t1, t0);
    EO1 = v128_unziplo_32(t1, t0);
    EO2 = v128_unziphi_32(t3, t2);
    EO3 = v128_unziplo_32(t3, t2);

    /* EEE and EEO */
    t0 = v128_ziphi_64(EE1, EE0);
    t1 = v128_ziphi_64(EE3, EE2);
    t2 = v128_ziplo_64(EE1, EE0);
    t3 = v128_ziplo_64(EE3, EE2);
    t4 = v128_unziphi_32(t1, t0);
    t5 = v128_unziplo_32(t3, t2);
    t6 = v128_unziplo_32(t1, t0);
    t7 = v128_unziphi_32(t3, t2);

    EEE0 = v128_add_32(t4, t5);
    EEE1 = v128_add_32(t6, t7);
    EEO0 = v128_sub_32(t4, t5);
    EEO1 = v128_sub_32(t6, t7);

    /* EEEE and EEEO */
    t0 = v128_shr_s32(v128_add_32(v128_shl_n_32(v128_add_32(EEE0, EEE1), 6), round), shift);
    v64_store_aligned(dst, v128_low_v64(v128_unziplo_16(t0, t0)));
    t0 = v128_shr_s32(v128_add_32(v128_shl_n_32(v128_sub_32(EEE0, EEE1), 6), round), shift);
    v64_store_aligned(dst+8*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    /* */
    t0 = v128_shr_s32(v128_add_32(v128_add_32(v128_mullo_s32(EEO0, v128_dup_32(83)),
                                              v128_mullo_s32(EEO1, v128_dup_32(36))), round), shift);
    v64_store_aligned(dst+4*16, v128_low_v64(v128_unziplo_16(t0, t0)));
    t0 = v128_shr_s32(v128_add_32(v128_add_32(v128_mullo_s32(EEO0, v128_dup_32(36)),
                                              v128_mullo_s32(EEO1, v128_dup_32(-83))), round), shift);
    v64_store_aligned(dst+12*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_shr_s32(v128_add_32(v128_add_32(v128_add_32(v128_mullo_s32(EO0, v128_dup_32(89)),
                                                          v128_mullo_s32(EO1, v128_dup_32(75))),
                                              v128_add_32(v128_mullo_s32(EO2, v128_dup_32(50)),
                                                          v128_mullo_s32(EO3, v128_dup_32(18)))), round), shift);
    v64_store_aligned(dst+2*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_shr_s32(v128_add_32(v128_add_32(v128_add_32(v128_mullo_s32(EO0, v128_dup_32(75)),
                                                          v128_mullo_s32(EO1, v128_dup_32(-18))),
                                              v128_add_32(v128_mullo_s32(EO2, v128_dup_32(-89)),
                                                          v128_mullo_s32(EO3, v128_dup_32(-50)))), round), shift);
    v64_store_aligned(dst+6*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_shr_s32(v128_add_32(v128_add_32(v128_add_32(v128_mullo_s32(EO0, v128_dup_32(50)),
                                                          v128_mullo_s32(EO1, v128_dup_32(-89))),
                                              v128_add_32(v128_mullo_s32(EO2, v128_dup_32(18)),
                                                          v128_mullo_s32(EO3, v128_dup_32(75)))), round), shift);
    v64_store_aligned(dst+10*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_shr_s32(v128_add_32(v128_add_32(v128_add_32(v128_mullo_s32(EO0, v128_dup_32(18)),
                                                          v128_mullo_s32(EO1, v128_dup_32(-50))),
                                              v128_add_32(v128_mullo_s32(EO2, v128_dup_32(75)),
                                                          v128_mullo_s32(EO3, v128_dup_32(-89)))), round), shift);
    v64_store_aligned(dst+14*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0x00090019002b0039LL, 0x004600500057005aLL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v128_dotp_s16(O3, t0),
                                               (int32_t)v128_dotp_s16(O2, t0),
                                               (int32_t)v128_dotp_s16(O1, t0),
                                               (int32_t)v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+1*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0xffe7ffbaffa6ffb0LL, 0xffd5000900390057LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v128_dotp_s16(O3, t0),
                                               (int32_t)v128_dotp_s16(O2, t0),
                                               (int32_t)v128_dotp_s16(O1, t0),
                                               (int32_t)v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+3*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0x002b005a0039ffe7LL, 0xffa9ffba00090050LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v128_dotp_s16(O3, t0),
                                               (int32_t)v128_dotp_s16(O2, t0),
                                               (int32_t)v128_dotp_s16(O1, t0),
                                               (int32_t)v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+5*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0xffc7ffb00019005aLL, 0x0009ffa9ffd50046LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v128_dotp_s16(O3, t0),
                                               (int32_t)v128_dotp_s16(O2, t0),
                                               (int32_t)v128_dotp_s16(O1, t0),
                                               (int32_t)v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+7*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0x0046002bffa9fff7LL, 0x005affe7ffb00039LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v128_dotp_s16(O3, t0),
                                               (int32_t)v128_dotp_s16(O2, t0),
                                               (int32_t)v128_dotp_s16(O1, t0),
                                               (int32_t)v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+9*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0xffb000090046ffa9LL, 0x00190039ffa6002bLL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v128_dotp_s16(O3, t0),
                                               (int32_t)v128_dotp_s16(O2, t0),
                                               (int32_t)v128_dotp_s16(O1, t0),
                                               (int32_t)v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+11*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0x0057ffc70009002bLL, 0xffb0005affba0019LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v128_dotp_s16(O3, t0),
                                               (int32_t)v128_dotp_s16(O2, t0),
                                               (int32_t)v128_dotp_s16(O1, t0),
                                               (int32_t)v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+13*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0xffa60057ffb00046LL, 0xffc7002bffe70009LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32((int32_t)v128_dotp_s16(O3, t0),
                                               (int32_t)v128_dotp_s16(O2, t0),
                                               (int32_t)v128_dotp_s16(O1, t0),
                                               (int32_t)v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+15*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    src += 16*4;
    dst += 4;
  }
}


static void transform32(const int16_t *src, int16_t *dst, int shift, int it)
{
  extern const int16_t g4mat_hevc[32][32];
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


void transform_simd(const int16_t *block, int16_t *coeff, int size, int fast, int bitdepth)
{
  if (size == 4) {
    transform4(block, coeff, bitdepth);
  } else if (size == 8) {
    int16_t *tmp = thor_alloc(size*size*2, 32);
    transform8(block, tmp, bitdepth - 5);
    transform8(tmp, coeff, 8);
    thor_free(tmp);
  } else if (size == 16) {
    int16_t *tmp = thor_alloc(size*size*2, 32);
    transform16(block, tmp, bitdepth - 4);
    transform16(tmp, coeff, 9);
    thor_free(tmp);
  } else if (size == 32) {
    if (fast) {
      int16_t *tmp = thor_alloc(16*16*2, 32);
      int16_t *tmp2 = thor_alloc(16*16*2, 32);
      int16_t *tmp3 = thor_alloc(16*16*2, 32);
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
      int16_t *tmp = thor_alloc(size*size*2, 32);
      transform32(block, tmp, bitdepth - 3, 32);
      transform32(tmp, coeff, 10, 16);
      thor_free(tmp);
    }
  } else { // size >= 64
    if (fast) {
      int16_t *tmp = thor_alloc(16*16*2, 32);
      int16_t *tmp2 = thor_alloc(16*16*2, 32);
      int16_t *tmp3 = thor_alloc(16*16*2, 32);
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
      int16_t *tmp = thor_alloc(32*32*2, 32);
      int16_t *tmp2 = thor_alloc(32*32*2, 32);
      int16_t *tmp3 = thor_alloc(32*32*2, 32);
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

void inverse_transform_simd(const int16_t *coeff, int16_t *block, int size, int bitdepth)
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
      int16_t *tmp = thor_alloc(size*size*2, 32);
      inverse_transform16(coeff, tmp, 7);
      inverse_transform16(tmp, block, 20 - bitdepth);
      thor_free(tmp);
    }
  } else
    inverse_transform32(coeff, block, bitdepth);
}
#endif

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

void TEMPLATE(clpf_block4)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizey, boundary_type bt, unsigned int strength, unsigned int dmp) {
  const int right = !(bt & TILE_RIGHT_BOUNDARY);
  const int bottom = bt & TILE_BOTTOM_BOUNDARY ? sizey - 4 : -1;
  const int left = !(bt & TILE_LEFT_BOUNDARY);
  const int top = bt & TILE_ABOVE_BOUNDARY ? 0 : -1;

  dst += x0 + y0*dstride;
  src += x0 + y0*sstride;

#ifdef HBD
  static ALIGN(32) uint64_t c_shuff[] = { 0x0302010001000100LL, 0x0b0a090809080908LL,
                                          0x0302010001000100LL, 0x0b0a090809080908LL };
  static ALIGN(32) uint64_t d_shuff[] = { 0x0504030201000100LL, 0x0d0c0b0a09080908LL,
                                          0x0504030201000100LL, 0x0d0c0b0a09080908LL };
  static ALIGN(32) uint64_t e_shuff[] = { 0x0706070605040302LL, 0x0f0e0f0e0d0c0b0aLL,
                                          0x0706070605040302LL, 0x0f0e0f0e0d0c0b0aLL };
  static ALIGN(32) uint64_t f_shuff[] = { 0x0706070607060504LL, 0x0f0e0f0e0f0e0d0cLL,
                                          0x0706070607060504LL, 0x0f0e0f0e0f0e0d0cLL };
#else
  static ALIGN(16) uint64_t c_shuff[] = { 0x0504040401000000LL, 0x0d0c0c0c09080808LL };
  static ALIGN(16) uint64_t d_shuff[] = { 0x0605040402010000LL, 0x0e0d0c0c0a090808LL };
  static ALIGN(16) uint64_t e_shuff[] = { 0x0707060503030201LL, 0x0f0f0e0d0b0b0a09LL };
  static ALIGN(16) uint64_t f_shuff[] = { 0x0707070603030302LL, 0x0f0f0f0e0b0b0b0aLL };
#endif

  for (int y = 0; y < sizey; y += 4) {
    const uint32_t l0 = u32_load_aligned(src - 2 * (y != top) * sstride);
    const uint32_t l1 = u32_load_aligned(src - (y != top) * sstride);
    const uint32_t l2 = u32_load_aligned(src);
    const uint32_t l3 = u32_load_aligned(src + sstride);
    const uint32_t l4 = u32_load_aligned(src + 2 * sstride);
    const uint32_t l5 = u32_load_aligned(src + 3 * sstride);
    const uint32_t l6 = u32_load_aligned(src + ((y != bottom) + 3) * sstride);
    const uint32_t l7 =
        u32_load_aligned(src + (2 * (y != bottom) + 3) * sstride);
    v128 o = v128_from_32(l2, l3, l4, l5);
    const v128 a = v128_from_32(l0, l1, l2, l3);
    const v128 b = v128_from_32(l1, l2, l3, l4);
    const v128 g = v128_from_32(l3, l4, l5, l6);
    const v128 h = v128_from_32(l4, l5, l6, l7);
    v128 c, d, e, f;

    if (left) {
      c = v128_from_32(u32_load_unaligned(src - 2),
                       u32_load_unaligned(src + sstride - 2),
                       u32_load_unaligned(src + 2 * sstride - 2),
                       u32_load_unaligned(src + 3 * sstride - 2));
      d = v128_from_32(u32_load_unaligned(src - 1),
                       u32_load_unaligned(src + sstride - 1),
                       u32_load_unaligned(src + 2 * sstride - 1),
                       u32_load_unaligned(src + 3 * sstride - 1));
    } else {  // Left clipping
      c = v128_shuffle_8(o, v128_load_aligned(c_shuff));
      d = v128_shuffle_8(o, v128_load_aligned(d_shuff));
    }
    if (right) {
      e = v128_from_32(u32_load_unaligned(src + 1),
                       u32_load_unaligned(src + sstride + 1),
                       u32_load_unaligned(src + 2 * sstride + 1),
                       u32_load_unaligned(src + 3 * sstride + 1));
      f = v128_from_32(u32_load_unaligned(src + 2),
                       u32_load_unaligned(src + sstride + 2),
                       u32_load_unaligned(src + 2 * sstride + 2),
                       u32_load_unaligned(src + 3 * sstride + 2));
    } else {  // Right clipping
      e = v128_shuffle_8(o, v128_load_aligned(e_shuff));
      f = v128_shuffle_8(o, v128_load_aligned(f_shuff));
    }

    o = calc_delta(o, a, b, c, d, e, f, g, h, strength, dmp);
    u32_store_aligned(dst, v128_low_u32(v128_shr_n_byte(o, 12)));
    u32_store_aligned(dst + dstride, v128_low_u32(v128_shr_n_byte(o, 8)));
    u32_store_aligned(dst + 2 * dstride, v128_low_u32(v128_shr_n_byte(o, 4)));
    u32_store_aligned(dst + 3 * dstride, v128_low_u32(o));

    dst += 4 * dstride;
    src += 4 * sstride;
  }
}

void TEMPLATE(clpf_block4_noclip)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizey, unsigned int strength, unsigned int dmp) {
  dst += x0 + y0 * dstride;
  src += x0 + y0 * sstride;

  for (int y = 0; y < sizey; y += 4) {
    const uint32_t l0 = u32_load_aligned(src - 2 * sstride);
    const uint32_t l1 = u32_load_aligned(src - sstride);
    const uint32_t l2 = u32_load_aligned(src);
    const uint32_t l3 = u32_load_aligned(src + sstride);
    const uint32_t l4 = u32_load_aligned(src + 2 * sstride);
    const uint32_t l5 = u32_load_aligned(src + 3 * sstride);
    const uint32_t l6 = u32_load_aligned(src + 4 * sstride);
    const uint32_t l7 = u32_load_aligned(src + 5 * sstride);
    const v128 a = v128_from_32(l0, l1, l2, l3);
    const v128 b = v128_from_32(l1, l2, l3, l4);
    const v128 g = v128_from_32(l3, l4, l5, l6);
    const v128 h = v128_from_32(l4, l5, l6, l7);
    const v128 c = v128_from_32(u32_load_unaligned(src - 2),
                                u32_load_unaligned(src + sstride - 2),
                                u32_load_unaligned(src + 2 * sstride - 2),
                                u32_load_unaligned(src + 3 * sstride - 2));
    const v128 d = v128_from_32(u32_load_unaligned(src - 1),
                                u32_load_unaligned(src + sstride - 1),
                                u32_load_unaligned(src + 2 * sstride - 1),
                                u32_load_unaligned(src + 3 * sstride - 1));
    const v128 e = v128_from_32(u32_load_unaligned(src + 1),
                                u32_load_unaligned(src + sstride + 1),
                                u32_load_unaligned(src + 2 * sstride + 1),
                                u32_load_unaligned(src + 3 * sstride + 1));
    const v128 f = v128_from_32(u32_load_unaligned(src + 2),
                                u32_load_unaligned(src + sstride + 2),
                                u32_load_unaligned(src + 2 * sstride + 2),
                                u32_load_unaligned(src + 3 * sstride + 2));

    const v128 o = calc_delta(v128_from_32(l2, l3, l4, l5), a, b, c, d, e, f, g,
                              h, strength, dmp);

    u32_store_aligned(dst, v128_low_u32(v128_shr_n_byte(o, 12)));
    u32_store_aligned(dst + dstride, v128_low_u32(v128_shr_n_byte(o, 8)));
    u32_store_aligned(dst + 2 * dstride, v128_low_u32(v128_shr_n_byte(o, 4)));
    u32_store_aligned(dst + 3 * dstride, v128_low_u32(o));

    dst += 4 * dstride;
    src += 4 * sstride;
  }
}

void TEMPLATE(clpf_block8)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizey, boundary_type bt, unsigned int strength, unsigned int dmp) {
  const int bottom = bt & TILE_BOTTOM_BOUNDARY ? sizey - 2 : -1;
  const int right = !(bt & TILE_RIGHT_BOUNDARY);
  const int left = !(bt & TILE_LEFT_BOUNDARY);
  const int top = bt & TILE_ABOVE_BOUNDARY ? 0 : -1;

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

  dst += x0 + y0 * dstride;
  src += x0 + y0 * sstride;

  for (int y = 0; y < sizey; y += 2) {
    const v64 l1 = v64_load_aligned(src);
    const v64 l2 = v64_load_aligned(src + sstride);
    const v64 l3 = v64_load_aligned(src - (y != top) * sstride);
    const v64 l4 = v64_load_aligned(src + ((y != bottom) + 1) * sstride);
    v128 o = v128_from_v64(l1, l2);
    const v128 a =
        v128_from_v64(v64_load_aligned(src - 2 * (y != top) * sstride), l3);
    const v128 b = v128_from_v64(l3, l1);
    const v128 g = v128_from_v64(l2, l4);
    const v128 h = v128_from_v64(
        l4, v64_load_aligned(src + (2 * (y != bottom) + 1) * sstride));
    v128 c, d, e, f;

    if (left) {
      c = v128_from_v64(v64_load_unaligned(src - 2),
                        v64_load_unaligned(src - 2 + sstride));
      d = v128_from_v64(v64_load_unaligned(src - 1),
                        v64_load_unaligned(src - 1 + sstride));
    } else {  // Left clipping
      c = v128_shuffle_8(o, v128_load_aligned(c_shuff));
      d = v128_shuffle_8(o, v128_load_aligned(d_shuff));
    }
    if (right) {
      e = v128_from_v64(v64_load_unaligned(src + 1),
                        v64_load_unaligned(src + 1 + sstride));
      f = v128_from_v64(v64_load_unaligned(src + 2),
                        v64_load_unaligned(src + 2 + sstride));
    } else {  // Right clipping
      e = v128_shuffle_8(o, v128_load_aligned(e_shuff));
      f = v128_shuffle_8(o, v128_load_aligned(f_shuff));
    }

    o = calc_delta(o, a, b, c, d, e, f, g, h, strength, dmp);
    v64_store_aligned(dst, v128_high_v64(o));
    v64_store_aligned(dst + dstride, v128_low_v64(o));
    src += sstride * 2;
    dst += dstride * 2;
  }
}

void TEMPLATE(clpf_block8_noclip)(const SAMPLE *src, SAMPLE *dst, int sstride, int dstride, int x0, int y0, int sizey, unsigned int strength, unsigned int dmp) {
  dst += x0 + y0 * dstride;
  src += x0 + y0 * sstride;

  for (int y = 0; y < sizey; y += 2) {
    const v64 l1 = v64_load_aligned(src);
    const v64 l2 = v64_load_aligned(src + sstride);
    const v64 l3 = v64_load_aligned(src - sstride);
    const v64 l4 = v64_load_aligned(src + 2 * sstride);
    const v128 a = v128_from_v64(v64_load_aligned(src - 2 * sstride), l3);
    const v128 b = v128_from_v64(l3, l1);
    const v128 g = v128_from_v64(l2, l4);
    const v128 h = v128_from_v64(l4, v64_load_aligned(src + 3 * sstride));
    const v128 c = v128_from_v64(v64_load_unaligned(src - 2),
                                 v64_load_unaligned(src - 2 + sstride));
    const v128 d = v128_from_v64(v64_load_unaligned(src - 1),
                                 v64_load_unaligned(src - 1 + sstride));
    const v128 e = v128_from_v64(v64_load_unaligned(src + 1),
                                 v64_load_unaligned(src + 1 + sstride));
    const v128 f = v128_from_v64(v64_load_unaligned(src + 2),
                                 v64_load_unaligned(src + 2 + sstride));
    const v128 o = calc_delta(v128_from_v64(l1, l2), a, b, c, d, e, f, g, h,
                              strength, dmp);

    v64_store_aligned(dst, v128_high_v64(o));
    v64_store_aligned(dst + dstride, v128_low_v64(o));
    src += sstride * 2;
    dst += dstride * 2;
  }
}

void TEMPLATE(scale_frame_down2x2_simd)(yuv_frame_t* sin, yuv_frame_t* sout)
{
  int wo=sout->width;
  int ho=sout->height;
  int so=sout->stride_y;
  int si=sin->stride_y;
  int i, j;
  v128 z = v128_dup_8(0);
  for (i=0; i<ho; ++i) {

    for (j=0; j<=wo-8; j+=8) {
      v128 a = v128_load_unaligned(&sin->y[(2*i+0)*si+2*j]);
      v128 b = v128_load_unaligned(&sin->y[(2*i+1)*si+2*j]);
      v128 c = v128_avg_u8(a,b);
      v128 d = v128_shr_s16(v128_padd_u8(c),1);
      v64_store_aligned(&sout->y[i*so+j], v128_low_v64(v128_pack_s16_u8(z,d)));
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
      v128 a = v128_load_aligned(&sin->u[(2*i+0)*sic+2*j]);
      v128 b = v128_load_aligned(&sin->u[(2*i+1)*sic+2*j]);
      v128 c = v128_avg_u8(a,b);
      v128 d = v128_shr_s16(v128_padd_u8(c),1);
      v64_store_aligned(&sout->u[i*soc+j], v128_low_v64(v128_pack_s16_u8(z,d)));
    }
    for (; j<wo; ++j) {
      sout->u[i*soc+j]=( ((sin->u[(2*i+0)*sic+(2*j+0)] + sin->u[(2*i+1)*sic+(2*j+0)]+1)>>1)+
                       + ((sin->u[(2*i+0)*sic+(2*j+1)] + sin->u[(2*i+1)*sic+(2*j+1)]+1)>>1) )>>1;
    }

    for (j=0; j<=wo-8; j+=8) {
      v128 a = v128_load_aligned(&sin->v[(2*i+0)*sic+2*j]);
      v128 b = v128_load_aligned(&sin->v[(2*i+1)*sic+2*j]);
      v128 c = v128_avg_u8(a,b);
      v128 d = v128_shr_s16(v128_madd_us8(c,ones),1);
      v64_store_aligned(&sout->v[i*soc+j], v128_low_v64(v128_pack_s16_u8(z,d)));
    }
    for (; j<wo; ++j) {
      sout->v[i*soc+j]=( ((sin->v[(2*i+0)*sic+(2*j+0)] + sin->v[(2*i+1)*sic+(2*j+0)]+1)>>1)+
                       + ((sin->v[(2*i+0)*sic+(2*j+1)] + sin->v[(2*i+1)*sic+(2*j+1)]+1)>>1) )>>1;
    }

  }
#endif
}

const ALIGN(32) int16_t TEMPLATE(coeffs_standard)[][8] = {
  {  0,   0,  64,   0,   0,   0,    0,   0 },
  {  1,  -7,  55,  19,  -5,   1,    0,   0 },
  {  1,  -7,  38,  38,  -7,   1,    0,   0 },
  {  1,  -5,  19,  55,  -7,   1,    0,   0 }
};

const ALIGN(32) int16_t TEMPLATE(coeffs_bipred)[][8] = {
  {  0,   0,  64,   0,   0,   0,    0,   0 },
  {  2, -10,  59,  17,  -5,   1,    0,   0 },
  {  1,  -8,  39,  39,  -8,   1,    0,   0 },
  {  1,  -5,  17,  59, -10,   2,    0,   0 }
};

const ALIGN(32) int16_t TEMPLATE(coeffs_chroma)[][4] = {
  {  0, 64,  0,  0 },
  { -2, 58, 10, -2 },
  { -4, 54, 16, -2 },
  { -4, 44, 28, -4 },
  { -4, 36, 36, -4 },
  { -4, 28, 44, -4 },
  { -2, 16, 54, -4 },
  { -2, 10, 58, -2 }
};

static void filter_6tap_edge(int width, int height, int xoff, int yoff,
                             SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip,
                             int istride, int bitdepth, const int16_t coeffs[][8])
{
  int cf = max(xoff, yoff);
  int sx = !yoff;
  int s1 = !xoff * istride;
  const int16_t *c = coeffs[cf];
  int st1 = s1 + sx;

  ip -= istride;
  qp -= qstride;

  if (width == 4) {
    v64 c0 = v64_dup_16(c[0]);
    v64 c1 = v64_dup_16(c[1]);
    v64 c2 = v64_dup_16(c[2]);
    v64 c3 = v64_dup_16(c[3]);
    v64 c4 = v64_dup_16(c[4]);
    v64 c5 = v64_dup_16(c[5]);
    v64 cr = v64_dup_16(32);

    for (int y = 0; y < height; y++) {
      qp += qstride;
      ip += istride;

      const SAMPLE *r = ip - 2 * s1 - 2 * sx;
      v64 r0 = v64_mullo_s16(c0, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*0)));
      v64 r1 = v64_mullo_s16(c1, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*1)));
      v64 r2 = v64_mullo_s16(c2, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*2)));
      v64 r3 = v64_mullo_s16(c3, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*3)));
      v64 r4 = v64_mullo_s16(c4, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*4)));
      v64 r5 = v64_mullo_s16(c5, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*5)));
      v64 rs = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);
#ifdef HBD
      rs = v64_shr_s16(rs, bitdepth - 10);
      u32_store_aligned(qp, v64_low_u32(v64_shr_u8(v64_pack_s16_u8(rs, rs), 16 - bitdepth)));
#else
      rs = v64_shr_n_s16(rs, 6);
      u32_store_aligned(qp, v64_low_u32(v64_pack_s16_u8(rs, rs)));
#endif
    }
  } else {
    v128 c0 = v128_dup_16(c[0]);
    v128 c1 = v128_dup_16(c[1]);
    v128 c2 = v128_dup_16(c[2]);
    v128 c3 = v128_dup_16(c[3]);
    v128 c4 = v128_dup_16(c[4]);
    v128 c5 = v128_dup_16(c[5]);
    v128 cr = v128_dup_16(32);

    ip += width;
    for (int y = 0; y < height; y++) {
      qp += qstride;
      ip += istride - width;

      for (int x = 0; x < width; x += 8) {
        const SAMPLE *r = ip - 2 * s1 - 2 * sx;
        v128 r0 = v128_mullo_s16(c0, v128_unpack_u8_s16(v64_load_unaligned(r + st1*0)));
        v128 r1 = v128_mullo_s16(c1, v128_unpack_u8_s16(v64_load_unaligned(r + st1*1)));
        v128 r2 = v128_mullo_s16(c2, v128_unpack_u8_s16(v64_load_unaligned(r + st1*2)));
        v128 r3 = v128_mullo_s16(c3, v128_unpack_u8_s16(v64_load_unaligned(r + st1*3)));
        v128 r4 = v128_mullo_s16(c4, v128_unpack_u8_s16(v64_load_unaligned(r + st1*4)));
        v128 r5 = v128_mullo_s16(c5, v128_unpack_u8_s16(v64_load_unaligned(r + st1*5)));
        v128 rs = v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(cr, r0), r1), r2), r3), r4), r5);
        ip += 8;
#ifdef HBD
        rs = v128_shr_s16(rs, bitdepth - 10);
        v64_store_aligned(qp + x, v64_shr_u8(v128_low_v64(v128_pack_s16_u8(rs, rs)), 16 - bitdepth));
#else
        rs = v128_shr_n_s16(rs, 6);
        v64_store_aligned(qp + x, v128_low_v64(v128_pack_s16_u8(rs, rs)));
#endif
      }
    }
  }
}

static void filter_6tap_inner(int width, int height, int xoff, int yoff,
                              SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip,
                              int istride, int bitdepth, const int16_t coeffs[][8]) {
  const int16_t *cf = coeffs[yoff];
  v128 c = v128_load_aligned(coeffs[xoff]);

  if (width == 4) {
    int xtap = coeffs[xoff][5]; // Final tap
    v128 c0 = v128_dup_16(cf[0]);
    v128 c1 = v128_dup_16(cf[1]);
    v128 c2 = v128_dup_16(cf[2]);
    v128 c3 = v128_dup_16(cf[3]);
    v128 c4 = v128_dup_16(cf[4]);
    v128 c5 = v128_dup_16(cf[5]);

    for (int y = 0; y < height; y++) {
      int res;
      v128 ax = v128_unpack_u8_s16(v64_load_unaligned(ip - 2));
      v128 a0 = v128_mullo_s16(c0, v128_unpack_u8_s16(v64_load_unaligned(ip - 2 * istride - 2)));
      v128 a1 = v128_mullo_s16(c1, v128_unpack_u8_s16(v64_load_unaligned(ip - 1 * istride - 2)));
      v128 a2 = v128_mullo_s16(c2, v128_unpack_u8_s16(v64_load_unaligned(ip - 2)));
      v128 a3 = v128_mullo_s16(c3, v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride - 2)));
      v128 a4 = v128_mullo_s16(c4, v128_unpack_u8_s16(v64_load_unaligned(ip + 2 * istride - 2)));
      v128 a5 = v128_mullo_s16(c5, v128_unpack_u8_s16(v64_load_unaligned(ip + 3 * istride - 2)));

      for (int x = 0; x < 3; x++) {
        res = (int)((v128_dotp_s16(c, v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(a0, a1), a2), a3), a4), a5)) + 2048) >> 12);
        *qp++ = saturate(res, bitdepth);
        ax = v128_shr_n_byte(ax, 2);
        a0 = v128_shr_n_byte(a0, 2);
        a1 = v128_shr_n_byte(a1, 2);
        a2 = v128_shr_n_byte(a2, 2);
        a3 = v128_shr_n_byte(a3, 2);
        a4 = v128_shr_n_byte(a4, 2);
        a5 = v128_shr_n_byte(a5, 2);
      }

      int a08 = ip[6-2*istride]*coeffs[yoff][0]*xtap;
      int a18 = ip[6-1*istride]*coeffs[yoff][1]*xtap;
      int a28 = ip[6-0*istride]*coeffs[yoff][2]*xtap;
      int a38 = ip[6+1*istride]*coeffs[yoff][3]*xtap;
      int a48 = ip[6+2*istride]*coeffs[yoff][4]*xtap;
      int a58 = ip[6+3*istride]*coeffs[yoff][5]*xtap;

      res = (int)((v128_dotp_s16(c, v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(a0, a1), a2), a3), a4), a5)) +
                   + a08 + a18 + a28 + a38 + a48 + a58 + 2048) >> 12);
      *qp++ = saturate(res, bitdepth);
      ip += istride;
      qp += qstride - 4;
    }
  } else {
    const SAMPLE *restrict ip2 = ip;
    v128 c1, c2, c3;
    int16_t *ax = thor_alloc((width+8)*height*2*sizeof(SAMPLE), 32);
#ifdef HBD
    const int shift = 16;
#else
    const int shift = 8;
#endif
    c1 = v128_dup_16((coeffs[yoff][0] << shift) | (SAMPLE)coeffs[yoff][1]);
    c2 = v128_dup_16((coeffs[yoff][2] << shift) | (SAMPLE)coeffs[yoff][3]);
    c3 = v128_dup_16((coeffs[yoff][4] << shift) | (SAMPLE)coeffs[yoff][5]);

    for (int y = 0; y < height; y++) {
      int16_t *a = ax + y*(width+8);
      for (int i = 0; i <= width; i += 8) {
        v128 t1 = v128_madd_us8(v128_zip_8(v64_load_unaligned(ip - 2 * istride - 2),
                                           v64_load_unaligned(ip - 1 * istride - 2)), c1);
        v128 t2 = v128_madd_us8(v128_zip_8(v64_load_unaligned(ip - 0 * istride - 2),
                                           v64_load_unaligned(ip + 1 * istride - 2)), c2);
        v128 t3 = v128_madd_us8(v128_zip_8(v64_load_unaligned(ip + 2 * istride - 2),
                                           v64_load_unaligned(ip + 3 * istride - 2)), c3);
        v128_store_aligned(a + i, v128_add_16(v128_add_16(t1, t2), t3));
        ip += 8;
      }
      ip += istride - width - 8;
    }
    ip = ip2 - 2;

    for (int y = 0; y < height; y++) {
      int16_t *a = ax + y*(width+8);
      for (int i = 0; i < width; i += 8) {
#ifdef HBD
        v64 r0 = quote128_from_32(v128_dotp_s16(c, v128_load_unaligned(a + i + 7)),
                                  v128_dotp_s16(c, v128_load_unaligned(a + i + 6)),
                                  v128_dotp_s16(c, v128_load_unaligned(a + i + 5)),
                                  v128_dotp_s16(c, v128_load_unaligned(a + i + 4)));
        v64 r1 = quote128_from_32(v128_dotp_s16(c, v128_load_unaligned(a + i + 3)),
                                  v128_dotp_s16(c, v128_load_unaligned(a + i + 2)),
                                  v128_dotp_s16(c, v128_load_unaligned(a + i + 1)),
                                  v128_dotp_s16(c, v128_load_unaligned(a + i + 0)));
        r0 = v64_shr_s16(v64_add_16(r0, v64_dup_16(2048)), bitdepth - 4);
        r1 = v64_shr_s16(v64_add_16(r1, v64_dup_16(2048)), bitdepth - 4);
        v64_store_aligned(qp + y*qstride + i, v64_shr_u8(v64_pack_s16_u8(r0, r1), 16 - bitdepth));
#else
        v128 r0 = v128_from_32(v128_dotp_s16(c, v128_load_unaligned(a + i + 7)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 6)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 5)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 4)));
        v128 r1 = v128_from_32(v128_dotp_s16(c, v128_load_unaligned(a + i + 3)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 2)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 1)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 0)));
        r0 = v128_shr_n_s32(v128_add_32(r0, v128_dup_32(2048)), 12);
        r1 = v128_shr_n_s32(v128_add_32(r1, v128_dup_32(2048)), 12);
        r0 = v128_pack_s32_s16(r0, r1);
        v64_store_aligned(qp + y*qstride + i, v128_low_v64(v128_pack_s16_u8(r0, r0)));
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
    v64 round = v64_dup_16(8);
    for (int i = 0; i < height; i++) {
      v64 r, s;
      r = v64_add_16(v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip - 1 * istride + 0))),
                     v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip - 1 * istride + 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip - 0 * istride - 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip + 1 * istride - 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip + 1 * istride + 2))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip + 2 * istride + 0))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip + 2 * istride + 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip - 0 * istride + 2))));
      s = v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip - 0 * istride + 0)));
      r = v64_add_16(r, v64_add_16(s, s));
      s = v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip - 0 * istride + 1)));
      r = v64_add_16(r, v64_add_16(s, s));
      s = v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip + 1 * istride + 0)));
      r = v64_add_16(r, v64_add_16(s, s));
      s = v64_unpacklo_u8_s16(v64_from_32(u32_zero(), u32_load_unaligned(ip + 1 * istride + 1)));
      r = v64_add_16(r, v64_add_16(s, s));
      r = v64_shr_s16(v64_add_16(r, round), 4);
      u32_store_aligned(qp + i * qstride, v64_low_u32(v64_pack_s16_u8(r, r)));
      ip += istride;
    }
  } else {
    v128 round = v128_dup_16(8);
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j += 8) {
        v128 r, s;
        r = v128_add_16(v128_unpack_u8_s16(v64_load_unaligned(ip - 1 * istride + 0)),
                        v128_unpack_u8_s16(v64_load_unaligned(ip - 1 * istride + 1)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip - 0 * istride - 1)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride - 1)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride + 2)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip + 2 * istride + 0)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip + 2 * istride + 1)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip - 0 * istride + 2)));
        s = v128_unpack_u8_s16(v64_load_unaligned(ip - 0 * istride + 0));
        r = v128_add_16(r, v128_add_16(s, s));
        s = v128_unpack_u8_s16(v64_load_unaligned(ip - 0 * istride + 1));
        r = v128_add_16(r, v128_add_16(s, s));
        s = v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride + 0));
        r = v128_add_16(r, v128_add_16(s, s));
        s = v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride + 1));
        r = v128_add_16(r, v128_add_16(s, s));
        r = v128_shr_s16(v128_add_16(r, round), 4);
        v64_store_aligned(qp + i * qstride + j, v128_low_v64(v128_pack_s16_u8(r, r)));
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
      (width, height, xoff, yoff, qp, qstride, ip, istride, bitdepth, bipred ? TEMPLATE(coeffs_bipred) : TEMPLATE(coeffs_standard));
}

static void filter_4tap_edge(int width, int height, int xoff, int yoff,
                             SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip,
                             int istride, int bitdepth, const int16_t coeffs[][4])
{
  int cf = max(xoff, yoff);
  int sx = !yoff;
  int s1 = !xoff * istride;
  const int16_t *c = &coeffs[cf][0];
  int st1 = s1 + sx;

  ip -= istride;
  qp -= qstride;

  if (width == 4) {
    v64 c0 = v64_dup_16(c[0]);
    v64 c1 = v64_dup_16(c[1]);
    v64 c2 = v64_dup_16(c[2]);
    v64 c3 = v64_dup_16(c[3]);
    v64 cr = v64_dup_16(32);

    for (int y = 0; y < height; y++) {
      qp += qstride;
      ip += istride;

      const SAMPLE *r = ip - s1 - sx;
      v64 r0 = v64_mullo_s16(c0, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*0)));
      v64 r1 = v64_mullo_s16(c1, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*1)));
      v64 r2 = v64_mullo_s16(c2, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*2)));
      v64 r3 = v64_mullo_s16(c3, v64_unpacklo_u8_s16(v64_load_unaligned(r + st1*3)));
      v64 rs = v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3);
#ifdef HBD
      rs = v64_shr_s16(rs, bitdepth - 10);
      u32_store_aligned(qp, v64_low_u32(v64_shr_u8(v64_pack_s16_u8(rs, rs), 16 - bitdepth)));
#else
      rs = v64_shr_n_s16(rs, 6);
      u32_store_aligned(qp, v64_low_u32(v64_pack_s16_u8(rs, rs)));
#endif
    }
  } else {
    v128 c0 = v128_dup_16(c[0]);
    v128 c1 = v128_dup_16(c[1]);
    v128 c2 = v128_dup_16(c[2]);
    v128 c3 = v128_dup_16(c[3]);
    v128 cr = v128_dup_16(32);

    ip += width;
    for (int y = 0; y < height; y++) {
      qp += qstride;
      ip += istride - width;

      for (int x = 0; x < width; x += 8) {
        const SAMPLE *r = ip - s1 - sx;
        v128 r0 = v128_mullo_s16(c0, v128_unpack_u8_s16(v64_load_unaligned(r + st1*0)));
        v128 r1 = v128_mullo_s16(c1, v128_unpack_u8_s16(v64_load_unaligned(r + st1*1)));
        v128 r2 = v128_mullo_s16(c2, v128_unpack_u8_s16(v64_load_unaligned(r + st1*2)));
        v128 r3 = v128_mullo_s16(c3, v128_unpack_u8_s16(v64_load_unaligned(r + st1*3)));
        v128 rs = v128_add_16(v128_add_16(v128_add_16(v128_add_16(cr, r0), r1), r2), r3);
        ip += 8;
#ifdef HBD
        rs = v128_shr_s16(rs, bitdepth - 10);
        v64_store_aligned(qp + x, v64_shr_u8(v128_low_v64(v128_pack_s16_u8(rs, rs)), 16 - bitdepth));
#else
        rs = v128_shr_n_s16(rs, 6);
        v64_store_aligned(qp + x, v128_low_v64(v128_pack_s16_u8(rs, rs)));
#endif
      }
    }
  }
}

static void filter_4tap_inner(int width, int height, int xoff, int yoff,
                              SAMPLE *restrict qp, int qstride, const SAMPLE *restrict ip,
                              int istride, int bitdepth, const int16_t coeffs[][4]) {
  const int16_t *cf = &coeffs[yoff][0];
  const v128 c0 = v128_dup_16(cf[0]);
  const v128 c1 = v128_dup_16(cf[1]);
  const v128 c2 = v128_dup_16(cf[2]);
  const v128 c3 = v128_dup_16(cf[3]);
  v64 filter = v64_load_aligned(coeffs[xoff]);
#ifdef HBD
  const quote128 round = quote128_dup_32(2048);
#else
  const v128 round = v128_dup_32(2048);
#endif

  if (width == 4) {
    v128 in0 = v128_unpack_u8_s16(v64_load_unaligned(ip - 1*istride - 1));
    v128 in1 = v128_unpack_u8_s16(v64_load_unaligned(ip + 0*istride - 1));
    v128 in2 = v128_unpack_u8_s16(v64_load_unaligned(ip + 1*istride - 1));

    for (int i = 0; i < height; i++) {
      v128 in3 = v128_unpack_u8_s16(v64_load_unaligned(ip + (i+2)*istride - 1));
      v128 out1 = v128_add_16(v128_add_16(v128_add_16(v128_mullo_s16(c0, in0), v128_mullo_s16(c1, in1)), v128_mullo_s16(c2, in2)), v128_mullo_s16(c3, in3));

#ifdef HBD
      quote128 hor_out = quote128_shr_s32(quote128_add_32(quote128_from_32((int32_t)v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 6)), filter),
                                                                           (int32_t)v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 4)), filter),
                                                                           (int32_t)v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 2)), filter),
                                                                           (int32_t)v64_dotp_s16(v128_low_v64(out1), filter)), round), bitdepth - 4);
      quote64_store_aligned(qp + qstride * i, quote64_shr_u16(quote128_low_v64(quote128_pack_s32_u16(hor_out, hor_out)), 16 - bitdepth));
#else
      v128 hor_out = v128_shr_n_s32(v128_add_32(v128_from_32((int32_t)v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 6)), filter),
                                                             (int32_t)v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 4)), filter),
                                                             (int32_t)v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 2)), filter),
                                                             (int32_t)v64_dotp_s16(v128_low_v64(out1), filter)), round), 12);
      v64 out = v64_pack_s32_s16(v128_high_v64(hor_out), v128_low_v64(hor_out));
      u32_store_aligned(qp + qstride * i, v64_low_u32(v64_pack_s16_u8(out, out)));
#endif

      in0 = in1;
      in1 = in2;
      in2 = in3;
    }
  } else {
    for (int j = 0; j < width; j += 8) {
      v128 load0 = v128_load_unaligned(ip - 1*istride + j - 1);
      v128 load1 = v128_load_unaligned(ip + 0*istride + j - 1);
      v128 load2 = v128_load_unaligned(ip + 1*istride + j - 1);
      v128 in00 = v128_unpacklo_u8_s16(load0);
      v128 in01 = v128_unpacklo_u8_s16(load1);
      v128 in02 = v128_unpacklo_u8_s16(load2);
      v128 in10 = v128_unpackhi_u8_s16(load0);
      v128 in11 = v128_unpackhi_u8_s16(load1);
      v128 in12 = v128_unpackhi_u8_s16(load2);

      for (int i = 0; i < height; i++) {
        v128 load3 = v128_load_unaligned(ip + (i+2)*istride + j - 1);
        v128 in03 = v128_unpacklo_u8_s16(load3);
        v128 in13 = v128_unpackhi_u8_s16(load3);

        /* Vertical */
        v128 out0 = v128_add_16(v128_add_16(v128_add_16(v128_mullo_s16(c0, in00), v128_mullo_s16(c1, in01)), v128_mullo_s16(c2, in02)), v128_mullo_s16(c3, in03));

        v128 out1 = v128_add_16(v128_add_16(v128_add_16(v128_mullo_s16(c0, in10), v128_mullo_s16(c1, in11)), v128_mullo_s16(c2, in12)), v128_mullo_s16(c3, in13));

        /* Horizontal */
        uint64_t in0 = v64_dotp_s16(v128_low_v64(out0), filter);
        uint64_t in1 = v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out0, 2)), filter);
        uint64_t in2 = v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out0, 4)), filter);
        uint64_t in3 = v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out0, 6)), filter);
        uint64_t in4 = v64_dotp_s16(v128_high_v64(out0), filter);
        uint64_t in5 = v64_dotp_s16(v128_low_v64(v128_align(out1, out0, 10*sizeof(SAMPLE))), filter);
        uint64_t in6 = v64_dotp_s16(v128_low_v64(v128_align(out1, out0, 12*sizeof(SAMPLE))), filter);
        uint64_t in7 = v64_dotp_s16(v128_low_v64(v128_align(out1, out0, 14*sizeof(SAMPLE))), filter);

#ifdef HBD
        quote128 out = quote128_pack_s32_u16(quote128_shr_s32(quote128_add_32(quote128_from_32((int32_t)in7, (int32_t)in6, (int32_t)in5, (int32_t)in4), round), bitdepth - 4),
                                             quote128_shr_s32(quote128_add_32(quote128_from_32((int32_t)in3, (int32_t)in2, (int32_t)in1, (int32_t)in0), round), bitdepth - 4));
        quote128_store_aligned(qp + qstride * i + j, quote128_shr_u16(out, 16 - bitdepth));
#else
        v128 out = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_from_32((int32_t)in7, (int32_t)in6, (int32_t)in5, (int32_t)in4), round), 12),
                                     v128_shr_n_s32(v128_add_32(v128_from_32((int32_t)in3, (int32_t)in2, (int32_t)in1, (int32_t)in0), round), 12));
        v64_store_aligned(qp + qstride * i + j, v128_low_v64(v128_pack_s16_u8(out, out)));
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
    (width, height, xoff, yoff, qp, qstride, ip, istride, bitdepth, TEMPLATE(coeffs_chroma));
}

#if CDEF

#ifndef HBD
/* partial A is a 16-bit vector of the form:
   [x8 x7 x6 x5 x4 x3 x2 x1] and partial B has the form:
   [0  y1 y2 y3 y4 y5 y6 y7].
   This function computes (x1^2+y1^2)*C1 + (x2^2+y2^2)*C2 + ...
   (x7^2+y2^7)*C7 + (x8^2+0^2)*C8 where the C1..C8 constants are in const1
   and const2. */
SIMD_INLINE v128 fold_mul_and_sum(v128 partiala, v128 partialb, v128 const1,
                                  v128 const2) {
  v128 tmp;
  /* Reverse partial B. */
  partialb = v128_shuffle_8(
      partialb, v128_from_32(0x0f0e0100, 0x03020504, 0x07060908, 0x0b0a0d0c));
  /* Interleave the x and y values of identical indices and pair x8 with 0. */
  tmp = partiala;
  partiala = v128_ziplo_16(partialb, partiala);
  partialb = v128_ziphi_16(partialb, tmp);
  /* Square and add the corresponding x and y values. */
  partiala = v128_madd_s16(partiala, partiala);
  partialb = v128_madd_s16(partialb, partialb);
  /* Multiply by constant. */
  partiala = v128_mullo_s32(partiala, const1);
  partialb = v128_mullo_s32(partialb, const2);
  /* Sum all results. */
  partiala = v128_add_32(partiala, partialb);
  return partiala;
}

SIMD_INLINE v128 hsum4(v128 x0, v128 x1, v128 x2, v128 x3) {
  v128 t0, t1, t2, t3;
  t0 = v128_ziplo_32(x1, x0);
  t1 = v128_ziplo_32(x3, x2);
  t2 = v128_ziphi_32(x1, x0);
  t3 = v128_ziphi_32(x3, x2);
  x0 = v128_ziplo_64(t1, t0);
  x1 = v128_ziphi_64(t1, t0);
  x2 = v128_ziplo_64(t3, t2);
  x3 = v128_ziphi_64(t3, t2);
  return v128_add_32(v128_add_32(x0, x1), v128_add_32(x2, x3));
}

/* Computes cost for directions 0, 5, 6 and 7. We can call this function again
   to compute the remaining directions. */
v128 compute_directions(v128 lines[8], int32_t tmp_cost1[4]) {
  v128 partial4a, partial4b, partial5a, partial5b, partial7a, partial7b;
  v128 partial6;
  v128 tmp;
  /* Partial sums for lines 0 and 1. */
  partial4a = v128_shl_n_byte(lines[0], 14);
  partial4b = v128_shr_n_byte(lines[0], 2);
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[1], 12));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[1], 4));
  tmp = v128_add_16(lines[0], lines[1]);
  partial5a = v128_shl_n_byte(tmp, 10);
  partial5b = v128_shr_n_byte(tmp, 6);
  partial7a = v128_shl_n_byte(tmp, 4);
  partial7b = v128_shr_n_byte(tmp, 12);
  partial6 = tmp;

  /* Partial sums for lines 2 and 3. */
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[2], 10));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[2], 6));
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[3], 8));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[3], 8));
  tmp = v128_add_16(lines[2], lines[3]);
  partial5a = v128_add_16(partial5a, v128_shl_n_byte(tmp, 8));
  partial5b = v128_add_16(partial5b, v128_shr_n_byte(tmp, 8));
  partial7a = v128_add_16(partial7a, v128_shl_n_byte(tmp, 6));
  partial7b = v128_add_16(partial7b, v128_shr_n_byte(tmp, 10));
  partial6 = v128_add_16(partial6, tmp);

  /* Partial sums for lines 4 and 5. */
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[4], 6));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[4], 10));
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[5], 4));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[5], 12));
  tmp = v128_add_16(lines[4], lines[5]);
  partial5a = v128_add_16(partial5a, v128_shl_n_byte(tmp, 6));
  partial5b = v128_add_16(partial5b, v128_shr_n_byte(tmp, 10));
  partial7a = v128_add_16(partial7a, v128_shl_n_byte(tmp, 8));
  partial7b = v128_add_16(partial7b, v128_shr_n_byte(tmp, 8));
  partial6 = v128_add_16(partial6, tmp);

  /* Partial sums for lines 6 and 7. */
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[6], 2));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[6], 14));
  partial4a = v128_add_16(partial4a, lines[7]);
  tmp = v128_add_16(lines[6], lines[7]);
  partial5a = v128_add_16(partial5a, v128_shl_n_byte(tmp, 4));
  partial5b = v128_add_16(partial5b, v128_shr_n_byte(tmp, 12));
  partial7a = v128_add_16(partial7a, v128_shl_n_byte(tmp, 10));
  partial7b = v128_add_16(partial7b, v128_shr_n_byte(tmp, 6));
  partial6 = v128_add_16(partial6, tmp);

  /* Compute costs in terms of partial sums. */
  partial4a =
      fold_mul_and_sum(partial4a, partial4b, v128_from_32(210, 280, 420, 840),
                       v128_from_32(105, 120, 140, 168));
  partial7a =
      fold_mul_and_sum(partial7a, partial7b, v128_from_32(210, 420, 0, 0),
                       v128_from_32(105, 105, 105, 140));
  partial5a =
      fold_mul_and_sum(partial5a, partial5b, v128_from_32(210, 420, 0, 0),
                       v128_from_32(105, 105, 105, 140));
  partial6 = v128_madd_s16(partial6, partial6);
  partial6 = v128_mullo_s32(partial6, v128_dup_32(105));

  partial4a = hsum4(partial4a, partial5a, partial6, partial7a);
  v128_store_unaligned(tmp_cost1, partial4a);
  return partial4a;
}

/* transpose and reverse the order of the lines -- equivalent to a 90-degree
   counter-clockwise rotation of the pixels. */
void array_reverse_transpose_8x8(v128 *in, v128 *res) {
  const v128 tr0_0 = v128_ziplo_16(in[1], in[0]);
  const v128 tr0_1 = v128_ziplo_16(in[3], in[2]);
  const v128 tr0_2 = v128_ziphi_16(in[1], in[0]);
  const v128 tr0_3 = v128_ziphi_16(in[3], in[2]);
  const v128 tr0_4 = v128_ziplo_16(in[5], in[4]);
  const v128 tr0_5 = v128_ziplo_16(in[7], in[6]);
  const v128 tr0_6 = v128_ziphi_16(in[5], in[4]);
  const v128 tr0_7 = v128_ziphi_16(in[7], in[6]);

  const v128 tr1_0 = v128_ziplo_32(tr0_1, tr0_0);
  const v128 tr1_1 = v128_ziplo_32(tr0_5, tr0_4);
  const v128 tr1_2 = v128_ziphi_32(tr0_1, tr0_0);
  const v128 tr1_3 = v128_ziphi_32(tr0_5, tr0_4);
  const v128 tr1_4 = v128_ziplo_32(tr0_3, tr0_2);
  const v128 tr1_5 = v128_ziplo_32(tr0_7, tr0_6);
  const v128 tr1_6 = v128_ziphi_32(tr0_3, tr0_2);
  const v128 tr1_7 = v128_ziphi_32(tr0_7, tr0_6);

  res[7] = v128_ziplo_64(tr1_1, tr1_0);
  res[6] = v128_ziphi_64(tr1_1, tr1_0);
  res[5] = v128_ziplo_64(tr1_3, tr1_2);
  res[4] = v128_ziphi_64(tr1_3, tr1_2);
  res[3] = v128_ziplo_64(tr1_5, tr1_4);
  res[2] = v128_ziphi_64(tr1_5, tr1_4);
  res[1] = v128_ziplo_64(tr1_7, tr1_6);
  res[0] = v128_ziphi_64(tr1_7, tr1_6);
}
#endif

int TEMPLATE(cdef_find_dir_simd)(const SAMPLE *img, int stride, int32_t *var,
                                 int coeff_shift) {
  int i;
  int32_t cost[8];
  int32_t best_cost = 0;
  int best_dir = 0;
#ifdef HBD
  quote128 lines[8];
  for (i = 0; i < 8; i++) {
    lines[i] = quote128_load_unaligned(&img[i * stride]);
    lines[i] =
        quote128_sub_16(quote128_shr_s16(lines[i], coeff_shift), quote128_dup_16(128));
#else
  v128 lines[8];
  for (i = 0; i < 8; i++) {
    lines[i] = v128_unpacklo_u8_s16(v128_from_v64(v64_zero(), v64_load_unaligned(&img[i * stride])));
    lines[i] =
        v128_sub_16(v128_shr_s16(lines[i], coeff_shift), v128_dup_16(128));
#endif
}

#if defined(__SSE4_1__)
  /* Compute "mostly vertical" directions. */
  __m128i dir47 = compute_directions(lines, cost + 4);

  array_reverse_transpose_8x8(lines, lines);

  /* Compute "mostly horizontal" directions. */
  __m128i dir03 = compute_directions(lines, cost);

  __m128i max = _mm_max_epi32(dir03, dir47);
  max = _mm_max_epi32(max, _mm_shuffle_epi32(max, _MM_SHUFFLE(1, 0, 3, 2)));
  max = _mm_max_epi32(max, _mm_shuffle_epi32(max, _MM_SHUFFLE(2, 3, 0, 1)));
  best_cost = _mm_cvtsi128_si32(max);
  __m128i t =
      _mm_packs_epi32(_mm_cmpeq_epi32(max, dir03), _mm_cmpeq_epi32(max, dir47));
  best_dir = _mm_movemask_epi8(_mm_packs_epi16(t, t));
  best_dir = log2i(best_dir ^ (best_dir - 1));  // Count trailing zeros
#else
  /* Compute "mostly vertical" directions. */
  compute_directions(lines, cost + 4);

  array_reverse_transpose_8x8(lines, lines);

  /* Compute "mostly horizontal" directions. */
  compute_directions(lines, cost);

  for (i = 0; i < 8; i++) {
    if (cost[i] > best_cost) {
      best_cost = cost[i];
      best_dir = i;
    }
  }
#endif

  /* Difference between the optimal variance and the variance along the
     orthogonal direction. Again, the sum(x^2) terms cancel out. */
  *var = best_cost - cost[(best_dir + 4) & 7];
  /* We'd normally divide by 840, but dividing by 1024 is close enough
     for what we're going to do with this. */
  *var >>= 10;
  return best_dir;
}

#ifndef HBD

// sign(a-b) * min(abs(a-b), max(0, threshold - (abs(a-b) >> adjdamp)))
SIMD_INLINE v128 constrain16(v128 a, v128 b, unsigned int threshold,
                             unsigned int adjdamp) {
  v128 diff = v128_sub_16(a, b);
  const v128 sign = v128_shr_n_s16(diff, 15);
  diff = v128_abs_s16(diff);
  const v128 s =
      v128_ssub_u16(v128_dup_16(threshold), v128_shr_u16(diff, adjdamp));
  return v128_xor(v128_add_16(sign, v128_min_s16(diff, s)), sign);
}

// sign(a - b) * min(abs(a - b), max(0, strength - (abs(a - b) >> adjdamp)))
SIMD_INLINE v128 constrain8(v256 a, v256 b, unsigned int strength,
                            unsigned int adjdamp) {
  const v256 diff16 = v256_sub_16(a, b);
  v128 diff = v128_pack_s16_s8(v256_high_v128(diff16), v256_low_v128(diff16));
  const v128 sign = v128_cmplt_s8(diff, v128_zero());
  diff = v128_abs_s8(diff);
  return v128_xor(
      v128_add_8(sign,
                 v128_min_u8(diff, v128_ssub_u8(v128_dup_8(strength),
                                                v128_shr_u8(diff, adjdamp)))),
      sign);
}

extern const int cdef_pri_taps[2][2 + CDEF_FULL];
extern const int cdef_sec_taps[2][2];

static void cdef_filter_block_4x4_8(uint8_t *dst, int dstride,
                                    const uint16_t *in, int sstride, int pri_strength,
                                    int sec_strength, int dir,
                                    int pri_damping, int sec_damping,
                                    int cdef_directions[8][2 + CDEF_FULL], int coeff_shift)
{
  v128 p0, p1, p2, p3;
  v256 sum, row, tap, res;
  v256 max, min, large = v256_dup_16(CDEF_VERY_LARGE);
  int po1 = cdef_directions[dir][0];
  int po2 = cdef_directions[dir][1];
#if CDEF_FULL
  int po3 = cdef_directions[dir][2];
#endif
  int s1o1 = cdef_directions[(dir + 2) & 7][0];
  int s1o2 = cdef_directions[(dir + 2) & 7][1];
  int s2o1 = cdef_directions[(dir + 6) & 7][0];
  int s2o2 = cdef_directions[(dir + 6) & 7][1];

  const int *pri_taps = cdef_pri_taps[(pri_strength >> coeff_shift) & 1];
  const int *sec_taps = cdef_sec_taps[(pri_strength >> coeff_shift) & 1];

  if (pri_strength) pri_damping -= log2i(pri_strength);
  if (sec_strength) sec_damping -= log2i(sec_strength);

  sum = v256_zero();
  row = v256_from_v64(v64_load_aligned(&in[0 * sstride]),
                      v64_load_aligned(&in[1 * sstride]),
                      v64_load_aligned(&in[2 * sstride]),
                      v64_load_aligned(&in[3 * sstride]));
  max = min = row;

  if (pri_strength) {
    // Primary near taps
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride + po1]),
                        v64_load_unaligned(&in[1 * sstride + po1]),
                        v64_load_unaligned(&in[2 * sstride + po1]),
                        v64_load_unaligned(&in[3 * sstride + po1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, pri_strength, pri_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride - po1]),
                        v64_load_unaligned(&in[1 * sstride - po1]),
                        v64_load_unaligned(&in[2 * sstride - po1]),
                        v64_load_unaligned(&in[3 * sstride - po1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[0] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[0]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));

    // Primary far taps
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride + po2]),
                        v64_load_unaligned(&in[1 * sstride + po2]),
                        v64_load_unaligned(&in[2 * sstride + po2]),
                        v64_load_unaligned(&in[3 * sstride + po2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, pri_strength, pri_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride - po2]),
                        v64_load_unaligned(&in[1 * sstride - po2]),
                        v64_load_unaligned(&in[2 * sstride - po2]),
                        v64_load_unaligned(&in[3 * sstride - po2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[1] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[1]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));

#if CDEF_FULL
    // Primary extra taps
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride + po3]),
                        v64_load_unaligned(&in[1 * sstride + po3]),
                        v64_load_unaligned(&in[2 * sstride + po3]),
                        v64_load_unaligned(&in[3 * sstride + po3]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, pri_strength, pri_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride - po3]),
                        v64_load_unaligned(&in[1 * sstride - po3]),
                        v64_load_unaligned(&in[2 * sstride - po3]),
                        v64_load_unaligned(&in[3 * sstride - po3]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[2] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[2]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));
#endif
  }

  if (sec_strength) {
    // Secondary near taps
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride + s1o1]),
                        v64_load_unaligned(&in[1 * sstride + s1o1]),
                        v64_load_unaligned(&in[2 * sstride + s1o1]),
                        v64_load_unaligned(&in[3 * sstride + s1o1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride - s1o1]),
                        v64_load_unaligned(&in[1 * sstride - s1o1]),
                        v64_load_unaligned(&in[2 * sstride - s1o1]),
                        v64_load_unaligned(&in[3 * sstride - s1o1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride + s2o1]),
                        v64_load_unaligned(&in[1 * sstride + s2o1]),
                        v64_load_unaligned(&in[2 * sstride + s2o1]),
                        v64_load_unaligned(&in[3 * sstride + s2o1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p2 = constrain8(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride - s2o1]),
                        v64_load_unaligned(&in[1 * sstride - s2o1]),
                        v64_load_unaligned(&in[2 * sstride - s2o1]),
                        v64_load_unaligned(&in[3 * sstride - s2o1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p3 = constrain8(tap, row, sec_strength, sec_damping);

    // sum += sec_taps[0] * (p0 + p1 + p2 + p3)
    p0 = v128_add_8(p0, p1);
    p2 = v128_add_8(p2, p3);
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(sec_taps[0]),
                                         v256_from_v128(v128_ziphi_8(p0, p2),
                                                        v128_ziplo_8(p0, p2))));

    // Secondary far taps
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride + s1o2]),
                        v64_load_unaligned(&in[1 * sstride + s1o2]),
                        v64_load_unaligned(&in[2 * sstride + s1o2]),
                        v64_load_unaligned(&in[3 * sstride + s1o2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride - s1o2]),
                        v64_load_unaligned(&in[1 * sstride - s1o2]),
                        v64_load_unaligned(&in[2 * sstride - s1o2]),
                        v64_load_unaligned(&in[3 * sstride - s1o2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride + s2o2]),
                        v64_load_unaligned(&in[1 * sstride + s2o2]),
                        v64_load_unaligned(&in[2 * sstride + s2o2]),
                        v64_load_unaligned(&in[3 * sstride + s2o2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p2 = constrain8(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * sstride - s2o2]),
                        v64_load_unaligned(&in[1 * sstride - s2o2]),
                        v64_load_unaligned(&in[2 * sstride - s2o2]),
                        v64_load_unaligned(&in[3 * sstride - s2o2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p3 = constrain8(tap, row, sec_strength, sec_damping);

    // sum += sec_taps[1] * (p0 + p1 + p2 + p3)
    p0 = v128_add_8(p0, p1);
    p2 = v128_add_8(p2, p3);

    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(sec_taps[1]),
                                         v256_from_v128(v128_ziphi_8(p0, p2),
                                                        v128_ziplo_8(p0, p2))));
  }

  // res = row + ((sum - (sum < 0) + 8) >> 4)
  sum = v256_add_16(sum, v256_cmplt_s16(sum, v256_zero()));
  res = v256_add_16(sum, v256_dup_16(8));
  res = v256_shr_n_s16(res, 4);
  res = v256_add_16(row, res);
  res = v256_min_s16(v256_max_s16(res, min), max);
  res = v256_pack_s16_u8(res, res);

  p0 = v256_low_v128(res);
  u32_store_aligned(&dst[0 * dstride], v64_high_u32(v128_high_v64(p0)));
  u32_store_aligned(&dst[1 * dstride], v64_low_u32(v128_high_v64(p0)));
  u32_store_aligned(&dst[2 * dstride], v64_high_u32(v128_low_v64(p0)));
  u32_store_aligned(&dst[3 * dstride], v64_low_u32(v128_low_v64(p0)));
}

static void cdef_filter_block_8x8_8(uint8_t *dst, int dstride,
                                    const uint16_t *in, int sstride,
                                    int pri_strength,
                                    int sec_strength, int dir,
                                    int pri_damping, int sec_damping,
                                    int cdef_directions[8][2 + CDEF_FULL], int coeff_shift)
{
  int i;
  v128 p0, p1, p2, p3;
  v256 sum, row, res, tap;
  v256 max, min, large = v256_dup_16(CDEF_VERY_LARGE);
  int po1 = cdef_directions[dir][0];
  int po2 = cdef_directions[dir][1];
#if CDEF_FULL
  int po3 = cdef_directions[dir][2];
#endif
  int s1o1 = cdef_directions[(dir + 2) & 7][0];
  int s1o2 = cdef_directions[(dir + 2) & 7][1];
  int s2o1 = cdef_directions[(dir + 6) & 7][0];
  int s2o2 = cdef_directions[(dir + 6) & 7][1];

  const int *pri_taps = cdef_pri_taps[(pri_strength >> coeff_shift) & 1];
  const int *sec_taps = cdef_sec_taps[(pri_strength >> coeff_shift) & 1];

  if (pri_strength) pri_damping -= log2i(pri_strength);
  if (sec_strength) sec_damping -= log2i(sec_strength);
  for (i = 0; i < 8; i += 2) {
    sum = v256_zero();
    row = v256_from_v128(v128_load_aligned(&in[i * sstride]),
                         v128_load_aligned(&in[(i + 1) * sstride]));

    max = min = row;
    // Primary near taps
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride + po1]),
                       v128_load_unaligned(&in[(i + 1) * sstride + po1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, pri_strength, pri_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride - po1]),
                       v128_load_unaligned(&in[(i + 1) * sstride - po1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[0] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[0]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));

    // Primary far taps
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride + po2]),
                       v128_load_unaligned(&in[(i + 1) * sstride + po2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, pri_strength, pri_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride - po2]),
                       v128_load_unaligned(&in[(i + 1) * sstride - po2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[1] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[1]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));

#if CDEF_FULL
    // Primary extra taps
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride + po3]),
                       v128_load_unaligned(&in[(i + 1) * sstride + po3]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, pri_strength, pri_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride - po3]),
                       v128_load_unaligned(&in[(i + 1) * sstride - po3]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[2] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[2]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));
#endif

    // Secondary near taps
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride + s1o1]),
                       v128_load_unaligned(&in[(i + 1) * sstride + s1o1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride - s1o1]),
                       v128_load_unaligned(&in[(i + 1) * sstride - s1o1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride + s2o1]),
                       v128_load_unaligned(&in[(i + 1) * sstride + s2o1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p2 = constrain8(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride - s2o1]),
                       v128_load_unaligned(&in[(i + 1) * sstride - s2o1]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p3 = constrain8(tap, row, sec_strength, sec_damping);

    // sum += sec_taps[0] * (p0 + p1 + p2 + p3)
    p0 = v128_add_8(p0, p1);
    p2 = v128_add_8(p2, p3);
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(sec_taps[0]),
                                         v256_from_v128(v128_ziphi_8(p0, p2),
                                                        v128_ziplo_8(p0, p2))));

    // Secondary far taps
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride + s1o2]),
                       v128_load_unaligned(&in[(i + 1) * sstride + s1o2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p0 = constrain8(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride - s1o2]),
                       v128_load_unaligned(&in[(i + 1) * sstride - s1o2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p1 = constrain8(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride + s2o2]),
                       v128_load_unaligned(&in[(i + 1) * sstride + s2o2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p2 = constrain8(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * sstride - s2o2]),
                       v128_load_unaligned(&in[(i + 1) * sstride - s2o2]));
    max = v256_max_s16(max, v256_andn(tap, v256_cmpeq_16(tap, large)));
    min = v256_min_s16(min, tap);
    p3 = constrain8(tap, row, sec_strength, sec_damping);

    // sum += sec_taps[1] * (p0 + p1 + p2 + p3)
    p0 = v128_add_8(p0, p1);
    p2 = v128_add_8(p2, p3);
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(sec_taps[1]),
                                         v256_from_v128(v128_ziphi_8(p0, p2),
                                                        v128_ziplo_8(p0, p2))));

    // res = row + ((sum - (sum < 0) + 8) >> 4)
    sum = v256_add_16(sum, v256_cmplt_s16(sum, v256_zero()));
    res = v256_add_16(sum, v256_dup_16(8));
    res = v256_shr_n_s16(res, 4);
    res = v256_add_16(row, res);
    res = v256_min_s16(v256_max_s16(res, min), max);
    res = v256_pack_s16_u8(res, res);

    p0 = v256_low_v128(res);
    v64_store_aligned(&dst[i * dstride], v128_high_v64(p0));
    v64_store_aligned(&dst[(i + 1) * dstride], v128_low_v64(p0));
  }
}

static void cdef_filter_block_4x4_16(uint16_t *dst, int dstride,
                                     const uint16_t *in, int sstride, int pri_strength,
                                     int sec_strength, int dir,
                                     int pri_damping, int sec_damping,
                                     int cdef_directions[8][2 + CDEF_FULL], int coeff_shift)
{
  int i;
  v128 p0, p1, p2, p3, sum, row, res;
  v128 max, min, large = v128_dup_16(CDEF_VERY_LARGE);
  int po1 = cdef_directions[dir][0];
  int po2 = cdef_directions[dir][1];
#if CDEF_FULL
  int po3 = cdef_directions[dir][2];
#endif
  int s1o1 = cdef_directions[(dir + 2) & 7][0];
  int s1o2 = cdef_directions[(dir + 2) & 7][1];
  int s2o1 = cdef_directions[(dir + 6) & 7][0];
  int s2o2 = cdef_directions[(dir + 6) & 7][1];

  const int *pri_taps = cdef_pri_taps[(pri_strength >> coeff_shift) & 1];
  const int *sec_taps = cdef_sec_taps[(pri_strength >> coeff_shift) & 1];

  if (pri_strength) pri_damping -= log2i(pri_strength);
  if (sec_strength) sec_damping -= log2i(sec_strength);
  for (i = 0; i < 4; i += 2) {
    sum = v128_zero();
    row = v128_from_v64(v64_load_aligned(&in[i * sstride]),
                        v64_load_aligned(&in[(i + 1) * sstride]));
    min = max = row;

    // Primary near taps
    p0 = v128_from_v64(v64_load_unaligned(&in[i * sstride + po1]),
                       v64_load_unaligned(&in[(i + 1) * sstride + po1]));
    p1 = v128_from_v64(v64_load_unaligned(&in[i * sstride - po1]),
                       v64_load_unaligned(&in[(i + 1) * sstride - po1]));
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    min = v128_min_s16(v128_min_s16(min, p0), p1);
    p0 = constrain16(p0, row, pri_strength, pri_damping);
    p1 = constrain16(p1, row, pri_strength, pri_damping);

    // sum += pri_taps[0] * (p0 + p1)
    sum = v128_add_16(
        sum, v128_mullo_s16(v128_dup_16(pri_taps[0]), v128_add_16(p0, p1)));

    // Primary far taps
    p0 = v128_from_v64(v64_load_unaligned(&in[i * sstride + po2]),
                       v64_load_unaligned(&in[(i + 1) * sstride + po2]));
    p1 = v128_from_v64(v64_load_unaligned(&in[i * sstride - po2]),
                       v64_load_unaligned(&in[(i + 1) * sstride - po2]));
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    min = v128_min_s16(v128_min_s16(min, p0), p1);
    p0 = constrain16(p0, row, pri_strength, pri_damping);
    p1 = constrain16(p1, row, pri_strength, pri_damping);

    // sum += pri_taps[1] * (p0 + p1)
    sum = v128_add_16(
        sum, v128_mullo_s16(v128_dup_16(pri_taps[1]), v128_add_16(p0, p1)));

#if CDEF_FULL
    // Primary extra taps
    p0 = v128_from_v64(v64_load_unaligned(&in[i * sstride + po3]),
                       v64_load_unaligned(&in[(i + 1) * sstride + po3]));
    p1 = v128_from_v64(v64_load_unaligned(&in[i * sstride - po3]),
                       v64_load_unaligned(&in[(i + 1) * sstride - po3]));
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    min = v128_min_s16(v128_min_s16(min, p0), p1);
    p0 = constrain16(p0, row, pri_strength, pri_damping);
    p1 = constrain16(p1, row, pri_strength, pri_damping);

    // sum += pri_taps[2] * (p0 + p1)
    sum = v128_add_16(
        sum, v128_mullo_s16(v128_dup_16(pri_taps[2]), v128_add_16(p0, p1)));
#endif

    // Secondary near taps
    p0 = v128_from_v64(v64_load_unaligned(&in[i * sstride + s1o1]),
                       v64_load_unaligned(&in[(i + 1) * sstride + s1o1]));
    p1 = v128_from_v64(v64_load_unaligned(&in[i * sstride - s1o1]),
                       v64_load_unaligned(&in[(i + 1) * sstride - s1o1]));
    p2 = v128_from_v64(v64_load_unaligned(&in[i * sstride + s2o1]),
                       v64_load_unaligned(&in[(i + 1) * sstride + s2o1]));
    p3 = v128_from_v64(v64_load_unaligned(&in[i * sstride - s2o1]),
                       v64_load_unaligned(&in[(i + 1) * sstride - s2o1]));
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p2, v128_cmpeq_16(p2, large))),
                     v128_andn(p3, v128_cmpeq_16(p3, large)));
    min = v128_min_s16(
        v128_min_s16(v128_min_s16(v128_min_s16(min, p0), p1), p2), p3);
    p0 = constrain16(p0, row, sec_strength, sec_damping);
    p1 = constrain16(p1, row, sec_strength, sec_damping);
    p2 = constrain16(p2, row, sec_strength, sec_damping);
    p3 = constrain16(p3, row, sec_strength, sec_damping);

    // sum += sec_taps[0] * (p0 + p1 + p2 + p3)
    sum = v128_add_16(sum, v128_mullo_s16(v128_dup_16(sec_taps[0]),
                                          v128_add_16(v128_add_16(p0, p1),
                                                      v128_add_16(p2, p3))));

    // Secondary far taps
    p0 = v128_from_v64(v64_load_unaligned(&in[i * sstride + s1o2]),
                       v64_load_unaligned(&in[(i + 1) * sstride + s1o2]));
    p1 = v128_from_v64(v64_load_unaligned(&in[i * sstride - s1o2]),
                       v64_load_unaligned(&in[(i + 1) * sstride - s1o2]));
    p2 = v128_from_v64(v64_load_unaligned(&in[i * sstride + s2o2]),
                       v64_load_unaligned(&in[(i + 1) * sstride + s2o2]));
    p3 = v128_from_v64(v64_load_unaligned(&in[i * sstride - s2o2]),
                       v64_load_unaligned(&in[(i + 1) * sstride - s2o2]));
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p2, v128_cmpeq_16(p2, large))),
                     v128_andn(p3, v128_cmpeq_16(p3, large)));
    min = v128_min_s16(
        v128_min_s16(v128_min_s16(v128_min_s16(min, p0), p1), p2), p3);
    p0 = constrain16(p0, row, sec_strength, sec_damping);
    p1 = constrain16(p1, row, sec_strength, sec_damping);
    p2 = constrain16(p2, row, sec_strength, sec_damping);
    p3 = constrain16(p3, row, sec_strength, sec_damping);

    // sum += sec_taps[1] * (p0 + p1 + p2 + p3)
    sum = v128_add_16(sum, v128_mullo_s16(v128_dup_16(sec_taps[1]),
                                          v128_add_16(v128_add_16(p0, p1),
                                                      v128_add_16(p2, p3))));

    // res = row + ((sum - (sum < 0) + 8) >> 4)
    sum = v128_add_16(sum, v128_cmplt_s16(sum, v128_zero()));
    res = v128_add_16(sum, v128_dup_16(8));
    res = v128_shr_n_s16(res, 4);
    res = v128_add_16(row, res);
    res = v128_min_s16(v128_max_s16(res, min), max);
    v64_store_aligned(&dst[i * dstride], v128_high_v64(res));
    v64_store_aligned(&dst[(i + 1) * dstride], v128_low_v64(res));
  }
}

static void cdef_filter_block_8x8_16(uint16_t *dst, int dstride,
                                     const uint16_t *in, int sstride, int pri_strength,
                                     int sec_strength, int dir,
                                     int pri_damping, int sec_damping,
                                     int cdef_directions[8][2 + CDEF_FULL], int coeff_shift)
{
  int i;
  v128 sum, p0, p1, p2, p3, row, res;
  v128 max, min, large = v128_dup_16(CDEF_VERY_LARGE);
  int po1 = cdef_directions[dir][0];
  int po2 = cdef_directions[dir][1];
#if CDEF_FULL
  int po3 = cdef_directions[dir][2];
#endif
  int s1o1 = cdef_directions[(dir + 2) & 7][0];
  int s1o2 = cdef_directions[(dir + 2) & 7][1];
  int s2o1 = cdef_directions[(dir + 6) & 7][0];
  int s2o2 = cdef_directions[(dir + 6) & 7][1];

  const int *pri_taps = cdef_pri_taps[(pri_strength >> coeff_shift) & 1];
  const int *sec_taps = cdef_sec_taps[(pri_strength >> coeff_shift) & 1];

  if (pri_strength) pri_damping -= log2i(pri_strength);
  if (sec_strength) sec_damping -= log2i(sec_strength);

  for (i = 0; i < 8; i++) {
    sum = v128_zero();
    row = v128_load_aligned(&in[i * sstride]);

    min = max = row;
    // Primary near taps
    p0 = v128_load_unaligned(&in[i * sstride + po1]);
    p1 = v128_load_unaligned(&in[i * sstride - po1]);
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    min = v128_min_s16(v128_min_s16(min, p0), p1);
    p0 = constrain16(p0, row, pri_strength, pri_damping);
    p1 = constrain16(p1, row, pri_strength, pri_damping);

    // sum += pri_taps[0] * (p0 + p1)
    sum = v128_add_16(
        sum, v128_mullo_s16(v128_dup_16(pri_taps[0]), v128_add_16(p0, p1)));

    // Primary far taps
    p0 = v128_load_unaligned(&in[i * sstride + po2]);
    p1 = v128_load_unaligned(&in[i * sstride - po2]);
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    min = v128_min_s16(v128_min_s16(min, p0), p1);
    p0 = constrain16(p0, row, pri_strength, pri_damping);
    p1 = constrain16(p1, row, pri_strength, pri_damping);

    // sum += pri_taps[1] * (p0 + p1)
    sum = v128_add_16(
        sum, v128_mullo_s16(v128_dup_16(pri_taps[1]), v128_add_16(p0, p1)));

#if CDEF_FULL
    // Primary extra taps
    p0 = v128_load_unaligned(&in[i * sstride + po3]);
    p1 = v128_load_unaligned(&in[i * sstride - po3]);
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    min = v128_min_s16(v128_min_s16(min, p0), p1);
    p0 = constrain16(p0, row, pri_strength, pri_damping);
    p1 = constrain16(p1, row, pri_strength, pri_damping);

    // sum += pri_taps[2] * (p0 + p1)
    sum = v128_add_16(
        sum, v128_mullo_s16(v128_dup_16(pri_taps[2]), v128_add_16(p0, p1)));
#endif

    // Secondary near taps
    p0 = v128_load_unaligned(&in[i * sstride + s1o1]);
    p1 = v128_load_unaligned(&in[i * sstride - s1o1]);
    p2 = v128_load_unaligned(&in[i * sstride + s2o1]);
    p3 = v128_load_unaligned(&in[i * sstride - s2o1]);
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p2, v128_cmpeq_16(p2, large))),
                     v128_andn(p3, v128_cmpeq_16(p3, large)));
    min = v128_min_s16(
        v128_min_s16(v128_min_s16(v128_min_s16(min, p0), p1), p2), p3);
    p0 = constrain16(p0, row, sec_strength, sec_damping);
    p1 = constrain16(p1, row, sec_strength, sec_damping);
    p2 = constrain16(p2, row, sec_strength, sec_damping);
    p3 = constrain16(p3, row, sec_strength, sec_damping);

    // sum += sec_taps[0] * (p0 + p1 + p2 + p3)
    sum = v128_add_16(sum, v128_mullo_s16(v128_dup_16(sec_taps[0]),
                                          v128_add_16(v128_add_16(p0, p1),
                                                      v128_add_16(p2, p3))));

    // Secondary far taps
    p0 = v128_load_unaligned(&in[i * sstride + s1o2]);
    p1 = v128_load_unaligned(&in[i * sstride - s1o2]);
    p2 = v128_load_unaligned(&in[i * sstride + s2o2]);
    p3 = v128_load_unaligned(&in[i * sstride - s2o2]);
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p0, v128_cmpeq_16(p0, large))),
                     v128_andn(p1, v128_cmpeq_16(p1, large)));
    max =
        v128_max_s16(v128_max_s16(max, v128_andn(p2, v128_cmpeq_16(p2, large))),
                     v128_andn(p3, v128_cmpeq_16(p3, large)));
    min = v128_min_s16(
        v128_min_s16(v128_min_s16(v128_min_s16(min, p0), p1), p2), p3);
    p0 = constrain16(p0, row, sec_strength, sec_damping);
    p1 = constrain16(p1, row, sec_strength, sec_damping);
    p2 = constrain16(p2, row, sec_strength, sec_damping);
    p3 = constrain16(p3, row, sec_strength, sec_damping);

    // sum += sec_taps[1] * (p0 + p1 + p2 + p3)
    sum = v128_add_16(sum, v128_mullo_s16(v128_dup_16(sec_taps[1]),
                                          v128_add_16(v128_add_16(p0, p1),
                                                      v128_add_16(p2, p3))));

    // res = row + ((sum - (sum < 0) + 8) >> 4)
    sum = v128_add_16(sum, v128_cmplt_s16(sum, v128_zero()));
    res = v128_add_16(sum, v128_dup_16(8));
    res = v128_shr_n_s16(res, 4);
    res = v128_add_16(row, res);
    res = v128_min_s16(v128_max_s16(res, min), max);
    v128_store_unaligned(&dst[i * dstride], res);
  }
}


void cdef_filter_block_simd(uint8_t *dst8, uint16_t *dst16, int dstride,
                            const uint16_t *in, int sstride, int pri_strength,
                            int sec_strength, int dir, int pri_damping,
                            int sec_damping, int bsize, int cdef_directions[8][2 + CDEF_FULL], int coeff_shift) {
  if (dst8)
    (bsize == 8 ? cdef_filter_block_8x8_8 : cdef_filter_block_4x4_8)(
        dst8, dstride, in, sstride, pri_strength, sec_strength, dir, pri_damping,
        sec_damping, cdef_directions, coeff_shift);
  else
    (bsize == 8 ? cdef_filter_block_8x8_16 : cdef_filter_block_4x4_16)(
        dst16, dstride, in, sstride, pri_strength, sec_strength, dir, pri_damping,
        sec_damping, cdef_directions, coeff_shift);
}

#endif
#endif
