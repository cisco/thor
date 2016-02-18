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

#include <string.h>

#include "simd.h"
#include "global.h"

void block_avg_simd(uint8_t *p,uint8_t *r0, uint8_t *r1, int sp, int s0, int s1, int width, int height)
{
  int i,j;
  if (width == 4) {
    v64 a, b;
    // Assume height is divisible by 4
    uint32_t * r0u = (uint32_t*) r0;;
    uint32_t * r1u = (uint32_t*) r1;
    uint32_t * pu = (uint32_t*) p;
    int s0u = s0>>2;
    int s1u = s1>>2;
    int spu = sp>>2;
    v64 out;

    for (i=0; i<height; i+=2) {
      a = v64_from_32(r0u[s0u],r0u[0]);
      b = v64_from_32(r1u[s1u],r1u[0]);
      out = v64_avg_u8(a,b);
      pu[0] = v64_low_u32(out);
      pu[spu] = v64_high_u32(out);

      r0u += 2*s0u;
      r1u += 2*s1u;
      pu += 2*spu;
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

int sad_calc_simd_unaligned(uint8_t *a, uint8_t *b, int astride, int bstride, int width, int height)
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
        sad64_internal s = v64_sad_u8_init();
        uint32_t * au= (uint32_t*) a;
        uint32_t * bu= (uint32_t*) b;
        int asu = astride >> 2;
        int bsu = bstride >> 2;
        for (i = 0; i < height; i += 4) {
          s = v128_sad_u8(s, v128_from_32(au[3*asu],au[2*asu],au[asu],au[0]), v128_from_32(bu[3*bsu],bu[2*bsu],bu[bsu],bu[0]));
          au += 4*asu;
          bu += 4*bsu;
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

static void get_inter_prediction_luma_edge_bipred(int width, int height, int xoff, int yoff,
                                                  uint8_t *restrict qp, int qstride,
                                                  const uint8_t *restrict ip, int istride)
{
  static const ALIGN(16) int16_t coeffs[4][6][4] = {
    { {   2,   2,   2,   2 },
      { -10, -10, -10, -10 },
      {  59,  59,  59,  59 },
      {  17,  17,  17,  17 },
      {  -5,  -5,  -5,  -5 },
      {   1,   1,   1,   1 }
    },
    { {   1,   1,   1,   1 },
      {  -8,  -8,  -8,  -8 },
      {  39,  39,  39,  39 },
      {  39,  39,  39,  39 },
      {  -8,  -8,  -8,  -8 },
      {   1,   1,   1,   1 }
    },
    { {   1,   1,   1,   1 },
      {  -5,  -5,  -5,  -5 },
      {  17,  17,  17,  17 },
      {  59,  59,  59,  59 },
      { -10, -10, -10, -10 },
      {   2,   2,   2,   2 } }
  };

  const unsigned char *restrict ip2 = ip;
  int cf = xoff + yoff - 1;
  int sx = !yoff;
  int s1 = !xoff * istride;
  ip += width - 2 * istride / 2;
  ip2 += height - istride;
  qp -= qstride;

  v64 c0 = v64_load_aligned(coeffs[cf][0]);
  v64 c1 = v64_load_aligned(coeffs[cf][1]);
  v64 c2 = v64_load_aligned(coeffs[cf][2]);
  v64 c3 = v64_load_aligned(coeffs[cf][3]);
  v64 c4 = v64_load_aligned(coeffs[cf][4]);
  v64 c5 = v64_load_aligned(coeffs[cf][5]);
  v64 cr = v64_dup_16(32);
  int st1 = s1 + sx;

  for (int y = 0; y < height; y++) {

    qp += qstride;
    ip += istride - width;
    ip2 += istride - width;

    if (width == 4) {
      v64 l0, l1, l2, l3, l4, l5;
      v64 r0, r1, r2, r3, r4, r5;
      v64 rs;
      const unsigned char *r = ip - 2 * s1 - 2 * sx;
      l0 = v64_load_unaligned(r);
      r += st1;
      l1 = v64_load_unaligned(r);
      r += st1;
      l2 = v64_load_unaligned(r);
      r += st1;
      l3 = v64_load_unaligned(r);
      r += st1;
      l4 = v64_load_unaligned(r);
      r += st1;
      l5 = v64_load_unaligned(r);
      r0 = v64_mullo_s16(c0, v64_unpacklo_u8_s16(l0));
      r1 = v64_mullo_s16(c1, v64_unpacklo_u8_s16(l1));
      r2 = v64_mullo_s16(c2, v64_unpacklo_u8_s16(l2));
      r3 = v64_mullo_s16(c3, v64_unpacklo_u8_s16(l3));
      r4 = v64_mullo_s16(c4, v64_unpacklo_u8_s16(l4));
      r5 = v64_mullo_s16(c5, v64_unpacklo_u8_s16(l5));
      rs = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);
      ip += 4;
      rs = v64_shr_n_s16(rs, 6);
      u32_store_aligned(qp, v64_low_u32(v64_pack_s16_u8(rs, rs)));
    } else {
      for (int x = 0; x < width; x += 8) {
        v64 l0, l1, l2, l3, l4, l5;
        v64 r0, r1, r2, r3, r4, r5;
        v64 rs1, rs2;
        const unsigned char *r = ip - 2 * s1 - 2 * sx;
        l0 = v64_load_unaligned(r);
        r += st1;
        l1 = v64_load_unaligned(r);
        r += st1;
        l2 = v64_load_unaligned(r);
        r += st1;
        l3 = v64_load_unaligned(r);
        r += st1;
        l4 = v64_load_unaligned(r);
        r += st1;
        l5 = v64_load_unaligned(r);
        r0 = v64_mullo_s16(c0, v64_unpackhi_u8_s16(l0));
        r1 = v64_mullo_s16(c1, v64_unpackhi_u8_s16(l1));
        r2 = v64_mullo_s16(c2, v64_unpackhi_u8_s16(l2));
        r3 = v64_mullo_s16(c3, v64_unpackhi_u8_s16(l3));
        r4 = v64_mullo_s16(c4, v64_unpackhi_u8_s16(l4));
        r5 = v64_mullo_s16(c5, v64_unpackhi_u8_s16(l5));
        rs1 = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);
        r0 = v64_mullo_s16(c0, v64_unpacklo_u8_s16(l0));
        r1 = v64_mullo_s16(c1, v64_unpacklo_u8_s16(l1));
        r2 = v64_mullo_s16(c2, v64_unpacklo_u8_s16(l2));
        r3 = v64_mullo_s16(c3, v64_unpacklo_u8_s16(l3));
        r4 = v64_mullo_s16(c4, v64_unpacklo_u8_s16(l4));
        r5 = v64_mullo_s16(c5, v64_unpacklo_u8_s16(l5));
        rs2 = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);

        ip += 8;
        rs1 = v64_shr_n_s16(rs1, 6);
        rs2 = v64_shr_n_s16(rs2, 6);
        v64_store_aligned(qp + x, v64_pack_s16_u8(rs1, rs2));
      }
    }
  }
}

static void get_inter_prediction_luma_edge(int width, int height, int xoff, int yoff,
                                             uint8_t *restrict qp, int qstride,
                                             const uint8_t *restrict ip, int istride)
{
  static const ALIGN(16) int16_t coeffs[4][6][4] = {
    { {   1,   1,   1,   1 },
      {  -7,  -7,  -7,  -7 },
      {  55,  55,  55,  55 },
      {  19,  19,  19,  19 },
      {  -5,  -5,  -5,  -5 },
      {   1,   1,   1,   1 }
    },
    { {   1,   1,   1,   1 },
      {  -7,  -7,  -7,  -7 },
      {  38,  38,  38,  38 },
      {  38,  38,  38,  38 },
      {  -7,  -7,  -7,  -7 },
      {   1,   1,   1,   1 }
    },
    { {   1,   1,   1,   1 },
      {  -5,  -5,  -5,  -5 },
      {  19,  19,  19,  19 },
      {  55,  55,  55,  55 },
      {  -7,  -7,  -7,  -7 },
      {   1,   1,   1,   1 } }
  };

  const unsigned char *restrict ip2 = ip;
  int cf = xoff + yoff - 1;
  int sx = !yoff;
  int s1 = !xoff * istride;
  ip += width - 2 * istride / 2;
  ip2 += height - istride;
  qp -= qstride;

  //v64 c0 = v64_load_aligned(coeffs[cf][0]);
  v64 c1 = v64_load_aligned(coeffs[cf][1]);
  v64 c2 = v64_load_aligned(coeffs[cf][2]);
  v64 c3 = v64_load_aligned(coeffs[cf][3]);
  v64 c4 = v64_load_aligned(coeffs[cf][4]);
  //v64 c5 = v64_load_aligned(coeffs[cf][5]);
  v64 cr = v64_dup_16(32);
  int st1 = s1 + sx;
  for (int y = 0; y < height; y++) {

    qp += qstride;
    ip += istride - width;
    ip2 += istride - width;

    if (width == 4) {
      v64 l0, l1, l2, l3, l4, l5;
      v64 r0, r1, r2, r3, r4, r5;
      v64 rs;
      const unsigned char *r = ip - 2 * s1 - 2 * sx;
      l0 = v64_load_unaligned(r);
      r += st1;
      l1 = v64_load_unaligned(r);
      r += st1;
      l2 = v64_load_unaligned(r);
      r += st1;
      l3 = v64_load_unaligned(r);
      r += st1;
      l4 = v64_load_unaligned(r);
      r += st1;
      l5 = v64_load_unaligned(r);
      r0 = v64_unpacklo_u8_s16(l0);//v64_mullo_s16(c0, v64_unpacklo_u8_s16(l0));
      r1 = v64_mullo_s16(c1, v64_unpacklo_u8_s16(l1));
      r2 = v64_mullo_s16(c2, v64_unpacklo_u8_s16(l2));
      r3 = v64_mullo_s16(c3, v64_unpacklo_u8_s16(l3));
      r4 = v64_mullo_s16(c4, v64_unpacklo_u8_s16(l4));
      r5 = v64_unpacklo_u8_s16(l5);//v64_mullo_s16(c5, v64_unpacklo_u8_s16(l5));
      rs = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);
      ip += 4;
      rs = v64_shr_n_s16(rs, 6);
      u32_store_aligned(qp, v64_low_u32(v64_pack_s16_u8(rs, rs)));
    } else {
      for (int x = 0; x < width; x += 8) {
        v64 l0, l1, l2, l3, l4, l5;
        v64 r0, r1, r2, r3, r4, r5;
        v64 rs1, rs2;
        const unsigned char *r = ip - 2 * s1 - 2 * sx;
        l0 = v64_load_unaligned(r);
        r += st1;
        l1 = v64_load_unaligned(r);
        r += st1;
        l2 = v64_load_unaligned(r);
        r += st1;
        l3 = v64_load_unaligned(r);
        r += st1;
        l4 = v64_load_unaligned(r);
        r += st1;
        l5 = v64_load_unaligned(r);
        r0 = v64_unpackhi_u8_s16(l0);//v64_mullo_s16(c0, v64_unpackhi_u8_s16(l0));
        r1 = v64_mullo_s16(c1, v64_unpackhi_u8_s16(l1));
        r2 = v64_mullo_s16(c2, v64_unpackhi_u8_s16(l2));
        r3 = v64_mullo_s16(c3, v64_unpackhi_u8_s16(l3));
        r4 = v64_mullo_s16(c4, v64_unpackhi_u8_s16(l4));
        r5 = v64_unpackhi_u8_s16(l5);//v64_mullo_s16(c5, v64_unpackhi_u8_s16(l5));
        rs1 = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);
        r0 = v64_unpacklo_u8_s16(l0);//v64_mullo_s16(c0, v64_unpacklo_u8_s16(l0));
        r1 = v64_mullo_s16(c1, v64_unpacklo_u8_s16(l1));
        r2 = v64_mullo_s16(c2, v64_unpacklo_u8_s16(l2));
        r3 = v64_mullo_s16(c3, v64_unpacklo_u8_s16(l3));
        r4 = v64_mullo_s16(c4, v64_unpacklo_u8_s16(l4));
        r5 = v64_unpacklo_u8_s16(l5);//v64_mullo_s16(c5, v64_unpacklo_u8_s16(l5));
        rs2 = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);
        ip += 8;
        rs1 = v64_shr_n_s16(rs1, 6);
        rs2 = v64_shr_n_s16(rs2, 6);
        v64_store_aligned(qp + x, v64_pack_s16_u8(rs1, rs2));
      }
    }
  }
}

static void get_inter_prediction_luma_inner_bipred(int width, int height, int xoff, int yoff,
                                                   uint8_t *restrict qp, int qstride,
                                                   const uint8_t *restrict ip, int istride)
{
#define G0 { 0,   0,   1,   0,   0, 0,   0, 0 }
#define G1 { 2, -10,  59,  17,  -5, 1,   0, 0 }
#define G2 { 1,  -5,  17,  59, -10, 2,   0, 0 }
#define G3 { 1,  -8,  39,  39,  -8, 1,   0, 0 }

  static ALIGN(16) int16_t coeffs2[24][2][8] = {
    { G0, G0 }, { G0, G0 }, { G0, G0 }, { G0, G0 },
    { G0, G0 }, { G1, G1 }, { G3, G1 }, { G2, G1 },
    { G0, G0 }, { G1, G3 }, { G3, G3 }, { G2, G3 },
  };

  if (width == 4) {
    static ALIGN(16) int16_t coeffs[3][6][8] = {
      { { 0 } },
      {
        {   2,   2,   2,   2,   2,   2,   2,   2},
        { -10, -10, -10, -10, -10, -10, -10, -10},
        {  59,  59,  59,  59,  59,  59,  59,  59},
        {  17,  17,  17,  17,  17,  17,  17,  17},
        {  -5,  -5,  -5,  -5,  -5,  -5,  -5,  -5},
        {   1,   1,   1,   1,   1,   1,   1,   1}
      },
      { {   1,   1,   1,   1,   1,   1,   1,   1 },
        {  -8,  -8,  -8,  -8,  -8,  -8,  -8,  -8 },
        {  39,  39,  39,  39,  39,  39,  39,  39 },
        {  39,  39,  39,  39,  39,  39,  39,  39 },
        {  -8,  -8,  -8,  -8,  -8,  -8,  -8,  -8 },
        {   1,   1,   1,   1,   1,   1,   1,   1 }
      }
    };

    // Final tap for each of the 2 phases
    int xtap = xoff==1 ? 1 : xoff==2 ? 1 : 2;
    v128 c = v128_load_aligned(coeffs2[xoff + yoff*4][0]);

    v128 c0 = v128_load_aligned(coeffs[yoff][0]);
    v128 c1 = v128_load_aligned(coeffs[yoff][1]);
    v128 c2 = v128_load_aligned(coeffs[yoff][2]);
    v128 c3 = v128_load_aligned(coeffs[yoff][3]);
    v128 c4 = v128_load_aligned(coeffs[yoff][4]);
    v128 c5 = v128_load_aligned(coeffs[yoff][5]);

    for (int y = 0; y < height; y++) {
      int res;
      v128 a0 = v128_mullo_s16(c0, v128_unpack_u8_s16(v64_load_unaligned(ip - 2 * istride - 2)));
      v128 a1 = v128_mullo_s16(c1, v128_unpack_u8_s16(v64_load_unaligned(ip - 1 * istride - 2)));
      v128 a2 = v128_mullo_s16(c2, v128_unpack_u8_s16(v64_load_unaligned(ip - 2)));
      v128 a3 = v128_mullo_s16(c3, v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride - 2)));
      v128 a4 = v128_mullo_s16(c4, v128_unpack_u8_s16(v64_load_unaligned(ip + 2 * istride - 2)));
      v128 a5 = v128_mullo_s16(c5, v128_unpack_u8_s16(v64_load_unaligned(ip + 3 * istride - 2)));

      for (int x = 0; x < 3; x++) {
        res = (v128_dotp_s16(c, v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(a0, a1), a2), a3), a4), a5)) + 2048) >> 12;
        *qp++ = clip255(res);
        a0 = v128_shr_n_byte(a0, 2);
        a1 = v128_shr_n_byte(a1, 2);
        a2 = v128_shr_n_byte(a2, 2);
        a3 = v128_shr_n_byte(a3, 2);
        a4 = v128_shr_n_byte(a4, 2);
        a5 = v128_shr_n_byte(a5, 2);
      }

      int a08, a18, a28, a38, a48, a58;
      switch (yoff == 1) {
      case 0:
        a08 = ip[6-2*istride]*1*xtap;
        a18 = ip[6-1*istride]*-8*xtap;
        a28 = ip[6-0*istride]*39*xtap;
        a38 = ip[6+1*istride]*39*xtap;
        a48 = ip[6+2*istride]*-8*xtap;
        a58 = ip[6+3*istride]*1*xtap;
        break;
      default:
        a08 = ip[6-2*istride]*2*xtap;
        a18 = ip[6-1*istride]*-10*xtap;
        a28 = ip[6-0*istride]*59*xtap;
        a38 = ip[6+1*istride]*17*xtap;
        a48 = ip[6+2*istride]*-5*xtap;
        a58 = ip[6+3*istride]*1*xtap;
        break;
      }

      res = v128_dotp_s16(c, v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(a0, a1), a2), a3), a4), a5)) +
        a08 + a18 + a28 + a38 + a48 + a58;
      *qp++ = clip255((res + 2048) >> 12);
      ip += istride;
      qp += qstride - 4;
    }

  } else {
    v128 c = v128_load_aligned(coeffs2[xoff + yoff*4][0]);
    const uint8_t *restrict ip2 = ip;
    v128 c1, c2, c3;
    int16_t *ax = thor_alloc((width+8)*height*2, 16);

    if (yoff == 1) {
      c1 = v128_dup_16((  2 << 8) | (uint8_t)-10);
      c2 = v128_dup_16(( 59 << 8) | (uint8_t) 17);
      c3 = v128_dup_16(( -5 << 8) | (uint8_t)  1);
    } else {
      c1 = v128_dup_16(( 1 << 8) | (uint8_t)-8);
      c2 = v128_dup_16((39 << 8) | (uint8_t)39);
      c3 = v128_dup_16((-8 << 8) | (uint8_t) 1);
    }

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
        v128 r0 = v128_from_64(v128_dotp_s16(c, v128_load_unaligned(a + i + 7)) << 32 |
                               (uint32_t)v128_dotp_s16(c, v128_load_unaligned(a + i + 6)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 5)) << 32 |
                               (uint32_t)v128_dotp_s16(c, v128_load_unaligned(a + i + 4)));
        v128 r1 = v128_from_64(v128_dotp_s16(c, v128_load_unaligned(a + i + 3)) << 32 |
                               (uint32_t)v128_dotp_s16(c, v128_load_unaligned(a + i + 2)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 1)) << 32 |
                               (uint32_t)v128_dotp_s16(c, v128_load_unaligned(a + i + 0)));
        r0 = v128_shr_n_s32(v128_add_32(r0, v128_dup_32(2048)), 12);
        r1 = v128_shr_n_s32(v128_add_32(r1, v128_dup_32(2048)), 12);
        r0 = v128_pack_s32_s16(r0, r1);
        v64_store_aligned(qp + y*qstride + i, v128_low_v64(v128_pack_s16_u8(r0, r0)));
      }
    }
    thor_free(ax);
  }
}

static void get_inter_prediction_luma_inner(int width, int height, int xoff, int yoff,
                                            uint8_t *restrict qp, int qstride,
                                            const uint8_t *restrict ip, int istride)
{
#define F0 { 0,   0,   1,   0,   0, 0,   0, 0 }
#define F1 { 1,  -7,  55,  19,  -5, 1,   0, 0 }
#define F2 { 1,  -5,  19,  55,  -7, 1,   0, 0 }
#define F3 { 1,  -7,  38,  38,  -7, 1,   0, 0 }

  static ALIGN(16) int16_t coeffs2[24][2][8] = {
    { F0, F0 }, { F0, F0 }, { F0, F0 }, { F0, F0 },
    { F0, F0 }, { F1, F1 }, { F3, F1 }, { F2, F1 },
    { F0, F0 }, { F1, F3 }, { F3, F3 }, { F2, F3 },
  };

  if (width == 4) {
    static ALIGN(16) int16_t coeffs[3][6][8] = {
      { { 0 } },
      {
        {   1,   1,   1,   1,   1,   1,   1,   1 },
        {  -7,  -7,  -7,  -7,  -7,  -7,  -7,  -7 },
        {  55,  55,  55,  55,  55,  55,  55,  55 },
        {  19,  19,  19,  19,  19,  19,  19,  19 },
        {  -5,  -5,  -5,  -5,  -5,  -5,  -5,  -5 },
        {   1,   1,   1,   1,   1,   1,   1,   1 }
      },
      { {   1,   1,   1,   1,   1,   1,   1,   1 },
        {  -7,  -7,  -7,  -7,  -7,  -7,  -7,  -7 },
        {  38,  38,  38,  38,  38,  38,  38,  38 },
        {  38,  38,  38,  38,  38,  38,  38,  38 },
        {  -7,  -7,  -7,  -7,  -7,  -7,  -7,  -7 },
        {   1,   1,   1,   1,   1,   1,   1,   1 }
      }
    };
    v128 c = v128_load_aligned(coeffs2[xoff + yoff*4][0]);

    //v128 c0 = v128_load_aligned(coeffs[yoff][0]);
    v128 c1 = v128_load_aligned(coeffs[yoff][1]);
    v128 c2 = v128_load_aligned(coeffs[yoff][2]);
    v128 c3 = v128_load_aligned(coeffs[yoff][3]);
    v128 c4 = v128_load_aligned(coeffs[yoff][4]);
    //v128 c5 = v128_load_aligned(coeffs[yoff][5]);

    for (int y = 0; y < height; y++) {
      int res;
      v128 ax = v128_unpack_u8_s16(v64_load_unaligned(ip - 2));
      v128 a0 = v128_unpack_u8_s16(v64_load_unaligned(ip - 2 * istride - 2));//v128_mullo_s16(c0, v128_unpack_u8_s16(v64_load_unaligned(ip - 2 * istride - 2)));
      v128 a1 = v128_mullo_s16(c1, v128_unpack_u8_s16(v64_load_unaligned(ip - 1 * istride - 2)));
      v128 a2 = v128_mullo_s16(c2, v128_unpack_u8_s16(v64_load_unaligned(ip - 2)));
      v128 a3 = v128_mullo_s16(c3, v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride - 2)));
      v128 a4 = v128_mullo_s16(c4, v128_unpack_u8_s16(v64_load_unaligned(ip + 2 * istride - 2)));
      v128 a5 = v128_unpack_u8_s16(v64_load_unaligned(ip + 3 * istride - 2));//v128_mullo_s16(c5, v128_unpack_u8_s16(v64_load_unaligned(ip + 3 * istride - 2)));

      for (int x = 0; x < 3; x++) {
        res = (v128_dotp_s16(c, v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(a0, a1), a2), a3), a4), a5)) + 2048) >> 12;
        *qp++ = clip255(res);
        ax = v128_shr_n_byte(ax, 2);
        a0 = v128_shr_n_byte(a0, 2);
        a1 = v128_shr_n_byte(a1, 2);
        a2 = v128_shr_n_byte(a2, 2);
        a3 = v128_shr_n_byte(a3, 2);
        a4 = v128_shr_n_byte(a4, 2);
        a5 = v128_shr_n_byte(a5, 2);
      }

      int a08, a18, a28, a38, a48, a58;
      switch ((yoff == 1)*2+(xoff == 1)) {
      case 0:
        a08 = ip[6-2*istride]*1*1;
        a18 = ip[6-1*istride]*-7*1;
        a28 = ip[6-0*istride]*38*1;
        a38 = ip[6+1*istride]*38*1;
        a48 = ip[6+2*istride]*-7*1;
        a58 = ip[6+3*istride]*1*1;
        break;
      case 1:
        a08 = ip[6-2*istride]*1*1;
        a18 = ip[6-1*istride]*-7*1;
        a28 = ip[6-0*istride]*38*1;
        a38 = ip[6+1*istride]*38*1;
        a48 = ip[6+2*istride]*-7*1;
        a58 = ip[6+3*istride]*1*1;
        break;
      case 2:
        a08 = ip[6-2*istride]*1*1;
        a18 = ip[6-1*istride]*-7*1;
        a28 = ip[6-0*istride]*55*1;
        a38 = ip[6+1*istride]*19*1;
        a48 = ip[6+2*istride]*-5*1;
        a58 = ip[6+3*istride]*1*1;
        break;
      default:
        a08 = ip[6-2*istride]*1*1;
        a18 = ip[6-1*istride]*-7*1;
        a28 = ip[6-0*istride]*55*1;
        a38 = ip[6+1*istride]*19*1;
        a48 = ip[6+2*istride]*-5*1;
        a58 = ip[6+3*istride]*1*1;
        break;
      }

      res = v128_dotp_s16(c, v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(a0, a1), a2), a3), a4), a5)) +
          + a08 + a18 + a28 + a38 + a48 + a58;
      *qp++ = clip255((res + 2048) >> 12);
      ip += istride;
      qp += qstride - 4;
    }

  } else {
    v128 c = v128_load_aligned(coeffs2[xoff + yoff*4][0]);
    const uint8_t *restrict ip2 = ip;
    v128 c1, c2, c3;
    int16_t *ax = thor_alloc((width+8)*height*2, 16);

    if (yoff == 1) {
      c1 = v128_dup_16(( 1 << 8) | (uint8_t)-7);
      c2 = v128_dup_16((55 << 8) | (uint8_t)19);
      c3 = v128_dup_16((-5 << 8) | (uint8_t) 1);
    } else {
      c1 = v128_dup_16(( 1 << 8) | (uint8_t)-7);
      c2 = v128_dup_16((38 << 8) | (uint8_t)38);
      c3 = v128_dup_16((-7 << 8) | (uint8_t) 1);
    }

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
        v128 r0 = v128_from_64(v128_dotp_s16(c, v128_load_unaligned(a + i + 7)) << 32 |
                               (uint32_t)v128_dotp_s16(c, v128_load_unaligned(a + i + 6)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 5)) << 32 |
                               (uint32_t)v128_dotp_s16(c, v128_load_unaligned(a + i + 4)));
        v128 r1 = v128_from_64(v128_dotp_s16(c, v128_load_unaligned(a + i + 3)) << 32 |
                               (uint32_t)v128_dotp_s16(c, v128_load_unaligned(a + i + 2)),
                               v128_dotp_s16(c, v128_load_unaligned(a + i + 1)) << 32 |
                               (uint32_t)v128_dotp_s16(c, v128_load_unaligned(a + i + 0)));
        r0 = v128_shr_n_s32(v128_add_32(r0, v128_dup_32(2048)), 12);
        r1 = v128_shr_n_s32(v128_add_32(r1, v128_dup_32(2048)), 12);
        r0 = v128_pack_s32_s16(r0, r1);
        v64_store_aligned(qp + y*qstride + i, v128_low_v64(v128_pack_s16_u8(r0, r0)));
      }
    }
    thor_free(ax);
  }
}

static void get_inter_prediction_luma_centre(int width, int height,
                                             uint8_t *restrict qp, int qstride,
                                             const uint8_t *restrict ip, int istride)
{
  if (width == 4) {
    v64 round = v64_dup_16(8);
    for (int i = 0; i < height; i++) {
      v64 r, s;
      r = v64_add_16(v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 1 * istride + 0))),
                     v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 1 * istride + 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 0 * istride - 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 1 * istride - 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 1 * istride + 2))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 2 * istride + 0))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 2 * istride + 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 0 * istride + 2))));
      s = v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 0 * istride + 0)));
      r = v64_add_16(r, v64_add_16(s, s));
      s = v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 0 * istride + 1)));
      r = v64_add_16(r, v64_add_16(s, s));
      s = v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 1 * istride + 0)));
      r = v64_add_16(r, v64_add_16(s, s));
      s = v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 1 * istride + 1)));
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

void get_inter_prediction_luma_simd(int width, int height, int xoff, int yoff,
                                    uint8_t *restrict qp, int qstride,
                                    const uint8_t *restrict ip, int istride, int bipred)
{
  if (xoff == 2 && yoff == 2)
    get_inter_prediction_luma_centre(width, height, qp, qstride, ip, istride);
  else {
    /* Use symmetric property of the filter */
    if (yoff == 3) {
      ip += height*istride;
      qp += (height-1)*qstride;
      istride = -istride;
      qstride = -qstride;
      yoff = 1;
    }
    if (bipred)
    (!xoff || !yoff ? get_inter_prediction_luma_edge_bipred : get_inter_prediction_luma_inner_bipred)
      (width, height, xoff, yoff, qp, qstride, ip, istride);
    else
    (!xoff || !yoff ? get_inter_prediction_luma_edge : get_inter_prediction_luma_inner)
      (width, height, xoff, yoff, qp, qstride, ip, istride);
  }
}

void get_inter_prediction_chroma_simd(int width, int height, int xoff, int yoff,
                                      unsigned char *restrict qp, int qstride,
                                      const unsigned char *restrict ip, int istride) {
  static const ALIGN(16) int16_t coeffs[8][4] = {
    { 0, 64,  0,  0},
    {-2, 58, 10, -2},
    {-4, 54, 16, -2},
    {-4, 44, 28, -4},
    {-4, 36, 36, -4},
    {-4, 28, 44, -4},
    {-2, 16, 54, -4},
    {-2, 10, 58, -2}
  };

  const v128 c0 = v128_dup_16(coeffs[yoff][0]);
  const v128 c1 = v128_dup_16(coeffs[yoff][1]);
  const v128 c2 = v128_dup_16(coeffs[yoff][2]);
  const v128 c3 = v128_dup_16(coeffs[yoff][3]);
  const v128 round = v128_dup_32(2048);
  const v64 filter = v64_load_aligned(coeffs[xoff]);
  int i;

  if (width == 4) {
    v128 in0 = v128_unpack_u8_s16(v64_load_unaligned(ip - 1*istride - 1));
    v128 in1 = v128_unpack_u8_s16(v64_load_unaligned(ip + 0*istride - 1));
    v128 in2 = v128_unpack_u8_s16(v64_load_unaligned(ip + 1*istride - 1));
    int i;

    for (i = 0; i < height; i++) {
      v128 in3 = v128_unpack_u8_s16(v64_load_unaligned(ip + (i+2)*istride - 1));
      v128 out1 = v128_add_16(v128_add_16(v128_add_16(v128_mullo_s16(c0, in0), v128_mullo_s16(c1, in1)), v128_mullo_s16(c2, in2)), v128_mullo_s16(c3, in3));

      v128 hor_out = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 6)), filter),
                                                             v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 4)), filter),
                                                             v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 2)), filter),
                                                             v64_dotp_s16(v128_low_v64(out1), filter)), round), 12);
      v64 out = v64_pack_s32_s16(v128_high_v64(hor_out), v128_low_v64(hor_out));
      u32_store_aligned(qp + qstride * i, v64_low_u32(v64_pack_s16_u8(out, out)));

      in0 = in1;
      in1 = in2;
      in2 = in3;
    }
  } else {
    int j;

    for (j = 0; j < width; j += 8) {
      v128 load0 = v128_load_unaligned(ip - 1*istride + j - 1);
      v128 load1 = v128_load_unaligned(ip + 0*istride + j - 1);
      v128 load2 = v128_load_unaligned(ip + 1*istride + j - 1);
      v128 in00 = v128_unpacklo_u8_s16(load0);
      v128 in01 = v128_unpacklo_u8_s16(load1);
      v128 in02 = v128_unpacklo_u8_s16(load2);
      v128 in10 = v128_unpackhi_u8_s16(load0);
      v128 in11 = v128_unpackhi_u8_s16(load1);
      v128 in12 = v128_unpackhi_u8_s16(load2);

      for (i = 0; i < height; i++) {
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
        uint64_t in5 = v64_dotp_s16(v128_low_v64(v128_align(out1, out0, 10)), filter);
        uint64_t in6 = v64_dotp_s16(v128_low_v64(v128_align(out1, out0, 12)), filter);
        uint64_t in7 = v64_dotp_s16(v128_low_v64(v128_align(out1, out0, 14)), filter);

        v128 out = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_from_32(in7, in6, in5, in4), round), 12),
                                     v128_shr_n_s32(v128_add_32(v128_from_32(in3, in2, in1, in0), round), 12));
        v64_store_aligned(qp + qstride * i + j, v128_low_v64(v128_pack_s16_u8(out, out)));

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
static void transform4(const int16_t *src, int16_t *dst)
{
  v128 t;
  v128 add1 = v128_dup_32(2);
  v128 add2 = v128_dup_32(64);

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
  t = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(s3, g0),
                                              v64_dotp_s16(s2, g0),
                                              v64_dotp_s16(s1, g0),
                                              v64_dotp_s16(s0, g0)), add1), 2);
  h0 = v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t));

  t = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(s3, g1),
                                              v64_dotp_s16(s2, g1),
                                              v64_dotp_s16(s1, g1),
                                              v64_dotp_s16(s0, g1)), add1), 2);
  h1 = v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t));

  t = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(s3, g2),
                                              v64_dotp_s16(s2, g2),
                                              v64_dotp_s16(s1, g2),
                                              v64_dotp_s16(s0, g2)), add1), 2);
  h2 = v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t));

  t = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(s3, g3),
                                              v64_dotp_s16(s2, g3),
                                              v64_dotp_s16(s1, g3),
                                              v64_dotp_s16(s0, g3)), add1), 2);
  h3 = v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t));

  /* Vertical transform */
  t = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(h3, g0),
                                              v64_dotp_s16(h2, g0),
                                              v64_dotp_s16(h1, g0),
                                              v64_dotp_s16(h0, g0)), add2), 7);
  v64_store_aligned(dst +  0, v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t)));

  t = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(h3, g1),
                                              v64_dotp_s16(h2, g1),
                                              v64_dotp_s16(h1, g1),
                                              v64_dotp_s16(h0, g1)), add2), 7);
  v64_store_aligned(dst +  4, v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t)));

  t = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(h3, g2),
                                              v64_dotp_s16(h2, g2),
                                              v64_dotp_s16(h1, g2),
                                              v64_dotp_s16(h0, g2)), add2), 7);
  v64_store_aligned(dst +  8, v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t)));

  t = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(h3, g3),
                                              v64_dotp_s16(h2, g3),
                                              v64_dotp_s16(h1, g3),
                                              v64_dotp_s16(h0, g3)), add2), 7);
  v64_store_aligned(dst + 12, v64_pack_s32_s16(v128_high_v64(t), v128_low_v64(t)));
}


static void inverse_transform4(const int16_t *coeff, int16_t *block) {
  v128 round1 = v128_dup_32(64);
  v128 round2 = v128_dup_32(2048);
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

  d0 = v128_shr_n_s32(v128_add_32(e0, o0), 12);
  d1 = v128_shr_n_s32(v128_add_32(e1, o1), 12);
  d2 = v128_shr_n_s32(v128_sub_32(e1, o1), 12);
  d3 = v128_shr_n_s32(v128_sub_32(e0, o0), 12);

  x0 = v128_ziplo_32(d1, d0);
  x1 = v128_ziphi_32(d1, d0);
  x2 = v128_ziplo_32(d3, d2);
  x3 = v128_ziphi_32(d3, d2);

  v64_store_aligned(block +  0, v64_pack_s32_s16(v128_low_v64(x2), v128_low_v64(x0)));
  v64_store_aligned(block +  4, v64_pack_s32_s16(v128_high_v64(x2), v128_high_v64(x0)));
  v64_store_aligned(block +  8, v64_pack_s32_s16(v128_low_v64(x3), v128_low_v64(x1)));
  v64_store_aligned(block + 12, v64_pack_s32_s16(v128_high_v64(x3), v128_high_v64(x1)));
}

static void inverse_transform8_4x4(const int16_t *coeff, int16_t *block) {
  v128 t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
  v128 round =  v128_dup_32(64);
  v128 c0  = v128_dup_32( 83 << 16 |   64);
  v128 c1  = v128_dup_32(-36 << 16 |   64);
  v128 c2  = v128_dup_32(-83 << 16 |   64);
  v128 c3  = v128_dup_32( 36 << 16 |   64);
  v128 c4  = v128_dup_32( 18 << 16 | (-75 & 0xffff));
  v128 c5  = v128_dup_32( 50 << 16 | (-18 & 0xffff));
  v128 c6  = v128_dup_32(-89 << 16 |   50);
  v128 c7  = v128_dup_32( 75 << 16 |   89);
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

  round =  v128_dup_32(2048);

  t8 = v128_madd_s16(c0, t10);
  t9 = v128_madd_s16(c0, t4);
  t3 = v128_madd_s16(c7, t11);
  t7 = v128_madd_s16(c7, t12);
  t0 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(t9, t7), round), 12),
                         v128_shr_n_s32(v128_add_32(v128_add_32(t8, t3), round), 12));
  t7 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t9, t7), round), 12),
                         v128_shr_n_s32(v128_add_32(v128_sub_32(t8, t3), round), 12));

  t8 = v128_madd_s16(c3, t10);
  t9 = v128_madd_s16(c3, t4);
  t3 = v128_madd_s16(c4, t11);
  t6 = v128_madd_s16(c4, t12);
  t1 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t9, t6), round), 12),
                         v128_shr_n_s32(v128_add_32(v128_sub_32(t8, t3), round), 12));
  t6 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(t9, t6), round), 12),
                         v128_shr_n_s32(v128_add_32(v128_add_32(t8, t3), round), 12));

  t8 = v128_madd_s16(c1, t10);
  t9 = v128_madd_s16(c1, t4);
  t3 = v128_madd_s16(c6, t11);
  t5 = v128_madd_s16(c6, t12);
  t2 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(t9, t5), round), 12),
                         v128_shr_n_s32(v128_add_32(v128_add_32(t8, t3), round), 12));
  t5 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t9, t5), round), 12),
                         v128_shr_n_s32(v128_add_32(v128_sub_32(t8, t3), round), 12));

  t8  = v128_madd_s16(c2, t10);
  t9  = v128_madd_s16(c2, t4);
  t10 = v128_madd_s16(c5, t11);
  t4  = v128_madd_s16(c5, t12);
  t3 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t9, t4), round), 12),
                         v128_shr_n_s32(v128_add_32(v128_sub_32(t8, t10), round), 12));
  t4 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(t9, t4), round), 12),
                         v128_shr_n_s32(v128_add_32(v128_add_32(t8, t10), round), 12));



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
static void inverse_transform8(const int16_t *coeff, int16_t *block) {
  v128 t0, t1, t2, t3, t4 ,t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16;
  v128 round = v128_dup_32(64);
  v128 c0  = v128_dup_16(64);
  v128 c1  = v128_dup_32(-64 << 16 |   64);
  v128 c2  = v128_dup_32( 36 << 16 |   83);
  v128 c3  = v128_dup_32( 83 << 16 | (-36 & 0xffff));
  v128 c4  = v128_dup_32( 18 << 16 | (-75 & 0xffff));
  v128 c5  = v128_dup_32( 50 << 16 | (-18 & 0xffff));
  v128 c6  = v128_dup_32(-89 << 16 |   50);
  v128 c7  = v128_dup_32( 75 << 16 |   89);
  v128 c8  = v128_dup_32( 50 << 16 |   89);
  v128 c9  = v128_dup_32( 89 << 16 | (-75 & 0xffff));
  v128 c10 = v128_dup_32( 75 << 16 |   18);
  v128 c11 = v128_dup_32( 18 << 16 |   50);

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
  round = v128_dup_32(2048);
  load0 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32( t8,  t5), round), 12),
                            v128_shr_n_s32(v128_add_32(v128_add_32(t13, t10), round), 12));
  load1 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32( t7,  t2), round), 12),
                            v128_shr_n_s32(v128_add_32(v128_sub_32(t16,  t0), round), 12));
  load2 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(t14, t15), round), 12),
                            v128_shr_n_s32(v128_add_32(v128_add_32(t11,  t4), round), 12));
  load3 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32( t6,  t3), round), 12),
                            v128_shr_n_s32(v128_add_32(v128_sub_32(t12,  t1), round), 12));
  load4 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32( t6,  t3), round), 12),
                            v128_shr_n_s32(v128_add_32(v128_add_32(t12,  t1), round), 12));
  load5 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(t14, t15), round), 12),
                            v128_shr_n_s32(v128_add_32(v128_sub_32(t11,  t4), round), 12));
  load6 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32( t7,  t2), round), 12),
                            v128_shr_n_s32(v128_add_32(v128_add_32(t16,  t0), round), 12));
  load7 = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32( t8,  t5), round), 12),
                            v128_shr_n_s32(v128_add_32(v128_sub_32(t13, t10), round), 12));

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
static void inverse_transform16_4x4(const int16_t *coeff, int16_t *block) {
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

  round = v128_dup_32(2048);

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
                       v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(v0, v3), round), 12),
                                         v128_shr_n_s32(v128_add_32(v128_add_32(v1, v2), round), 12)));
    v128_store_aligned(block + i4,
                       v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(v0, v3), round), 12),
                                         v128_shr_n_s32(v128_add_32(v128_sub_32(v1, v2), round), 12)));

    v0 = v128_madd_s16(c0, t0);
    v1 = v128_madd_s16(c0, t1);
    v2 = v128_madd_s16(c1, t8);
    v3 = v128_madd_s16(c1, t7);
    v128_store_aligned(block + 8 + i5,
                       v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_add_32(v0, v3), round), 12),
                                         v128_shr_n_s32(v128_add_32(v128_add_32(v1, v2), round), 12)));
    v128_store_aligned(block + 8 + i6,
                       v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_sub_32(v0, v3), round), 12),
                                         v128_shr_n_s32(v128_add_32(v128_sub_32(v1, v2), round), 12)));
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


static void inverse_transform32(const int16_t * coeff, int16_t *block)
{
  int16_t *tmp = thor_alloc(32*32*2, 16);
  int16_t *tcoeff = thor_alloc(8*32*2, 16);
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
    transform_1d_32(tmp, tcoeff, i, block + i*32, 12);

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
    EEE = v128_from_32(v64_dotp_s16(v128_low_v64(E_), c),
                       v64_dotp_s16(v128_high_v64(E_), c),
                       v64_dotp_s16(v128_low_v64(E), c),
                       v64_dotp_s16(v128_high_v64(E), c));
    c = v64_from_64(0x0040ffc0ffc00040LL);
    EEO = v128_from_32(v64_dotp_s16(v128_low_v64(E_), c),
                       v64_dotp_s16(v128_high_v64(E_), c),
                       v64_dotp_s16(v128_low_v64(E), c),
                       v64_dotp_s16(v128_high_v64(E), c));

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

    t = v128_shr_s32(v128_add_32(v128_add_32(v128_madd_s16(v128_dup_32(( 75 << 16) | (uint16_t)-18), hi),
                                             v128_madd_s16(v128_dup_32((-89 << 16) | (uint16_t)-50), lo)), round), shift);
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
    t0 = v128_shr_s32(v128_add_32(v128_from_32(v128_dotp_s16(O3, t0), v128_dotp_s16(O2, t0), v128_dotp_s16(O1, t0), v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+1*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0xffe7ffbaffa6ffb0LL, 0xffd5000900390057LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32(v128_dotp_s16(O3, t0), v128_dotp_s16(O2, t0), v128_dotp_s16(O1, t0), v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+3*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0x002b005a0039ffe7LL, 0xffa9ffba00090050LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32(v128_dotp_s16(O3, t0), v128_dotp_s16(O2, t0), v128_dotp_s16(O1, t0), v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+5*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0xffc7ffb00019005aLL, 0x0009ffa9ffd50046LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32(v128_dotp_s16(O3, t0), v128_dotp_s16(O2, t0), v128_dotp_s16(O1, t0), v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+7*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0x0046002bffa9fff7LL, 0x005affe7ffb00039LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32(v128_dotp_s16(O3, t0), v128_dotp_s16(O2, t0), v128_dotp_s16(O1, t0), v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+9*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0xffb000090046ffa9LL, 0x00190039ffa6002bLL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32(v128_dotp_s16(O3, t0), v128_dotp_s16(O2, t0), v128_dotp_s16(O1, t0), v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+11*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0x0057ffc70009002bLL, 0xffb0005affba0019LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32(v128_dotp_s16(O3, t0), v128_dotp_s16(O2, t0), v128_dotp_s16(O1, t0), v128_dotp_s16(O0, t0)), round), shift);
    v64_store_aligned(dst+13*16, v128_low_v64(v128_unziplo_16(t0, t0)));

    t0 = v128_from_64(0xffa60057ffb00046LL, 0xffc7002bffe70009LL);
    t0 = v128_shr_s32(v128_add_32(v128_from_32(v128_dotp_s16(O3, t0), v128_dotp_s16(O2, t0), v128_dotp_s16(O1, t0), v128_dotp_s16(O0, t0)), round), shift);
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


void transform_simd(const int16_t *block, int16_t *coeff, int size, int fast)
{
  if (size == 4) {
    transform4(block, coeff);
  } else if (size == 8) {
    int16_t *tmp = thor_alloc(size*size*2, 16);
    transform8(block, tmp, 3);
    transform8(tmp, coeff, 8);
    thor_free(tmp);
  } else if (size == 16) {
    int16_t *tmp = thor_alloc(size*size*2, 16);
    transform16(block, tmp, 4);
    transform16(tmp, coeff, 9);
    thor_free(tmp);
  } else if (size == 32) {
    if (fast) {
      int16_t *tmp = thor_alloc(16*16*2, 16);
      int16_t *tmp2 = thor_alloc(16*16*2, 16);
      int16_t *tmp3 = thor_alloc(16*16*2, 16);
      for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
          tmp2[i*16+j] = block[(i*2+0)*32+j*2+0] + block[(i*2+1)*32+j*2+0] + block[(i*2+0)*32+j*2+1] + block[(i*2+1)*32+j*2+1];
      transform16(tmp2, tmp, 6);
      transform16(tmp, tmp3, 9);
      for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
          coeff[i*32+j] = tmp3[i*16+j];
      thor_free(tmp);
      thor_free(tmp2);
      thor_free(tmp3);
    } else {
      int16_t *tmp = thor_alloc(size*size*2, 16);
      transform32(block, tmp, 5, 32);
      transform32(tmp, coeff, 10, 16);
      thor_free(tmp);
    }
  } else if (size == 64) {
    if (fast) {
      int16_t *tmp = thor_alloc(16*16*2, 16);
      int16_t *tmp2 = thor_alloc(16*16*2, 16);
      int16_t *tmp3 = thor_alloc(16*16*2, 16);
      for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
          tmp2[i*16+j] =
            block[(i*4+0)*64+j*4+0] + block[(i*4+1)*64+j*4+0] + block[(i*4+2)*64+j*4+0] + block[(i*4+3)*64+j*4+0] +
            block[(i*4+0)*64+j*4+1] + block[(i*4+1)*64+j*4+1] + block[(i*4+2)*64+j*4+1] + block[(i*4+3)*64+j*4+1] +
            block[(i*4+0)*64+j*4+2] + block[(i*4+1)*64+j*4+2] + block[(i*4+2)*64+j*4+2] + block[(i*4+3)*64+j*4+2] +
            block[(i*4+0)*64+j*4+3] + block[(i*4+1)*64+j*4+3] + block[(i*4+2)*64+j*4+3] + block[(i*4+3)*64+j*4+3];
      transform16(tmp2, tmp, 8);
      transform16(tmp, tmp3, 9);
      for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
          coeff[i*64+j] = tmp3[i*16+j];
      thor_free(tmp);
      thor_free(tmp2);
      thor_free(tmp3);
    } else {
      int16_t *tmp = thor_alloc(32*32*2, 16);
      int16_t *tmp2 = thor_alloc(32*32*2, 16);
      int16_t *tmp3 = thor_alloc(32*32*2, 16);
      for (int i = 0; i < 32; i++)
        for (int j = 0; j < 32; j++)
          tmp2[i*32+j] =
            block[(i*2+0)*64+j*2+0] + block[(i*2+1)*64+j*2+0] + block[(i*2+0)*64+j*2+1] + block[(i*2+1)*64+j*2+1];
      transform32(tmp2, tmp, 7, 32);
      transform32(tmp, tmp3, 10, 16);
      for (int i = 0; i < 32; i++)
        for (int j = 0; j < 32; j++)
          coeff[i*64+j] = tmp3[i*32+j];
      thor_free(tmp);
      thor_free(tmp2);
      thor_free(tmp3);
    }
  }
}

void inverse_transform_simd(const int16_t *coeff, int16_t *block, int size)
{
  if (size == 4) {
    inverse_transform4(coeff, block);
  } else if (size == 8) {
    int nz = check_nz_area(coeff, size);
    if (nz == COEFF_4x4_ONLY) {
      inverse_transform8_4x4(coeff, block);
    } else {
      inverse_transform8(coeff, block);
    }
  } else if (size == 16) {
    int nz = check_nz_area(coeff, size);
    if (nz == COEFF_4x4_ONLY) {
      inverse_transform16_4x4(coeff, block);
    } else {
      int16_t *tmp = thor_alloc(size*size*2, 16);
      inverse_transform16(coeff, tmp, 7);
      inverse_transform16(tmp, block, 12);
      thor_free(tmp);
    }
  } else
    inverse_transform32(coeff, block);
}

void clpf_block4(const uint8_t *src, uint8_t *dst, int sstride, int dstride, int x0, int y0, int width, int height) {
  int left = (x0 & ~(MAX_SB_SIZE/2-1)) - x0;
  int top = (y0 & ~(MAX_SB_SIZE/2-1)) - y0;
  int right = min(width-1, left + MAX_SB_SIZE/2-1);
  int bottom = min(height-1, top + MAX_SB_SIZE/2-1);
  dst -= left + top*dstride;
  src += x0 + y0*sstride;

  uint32_t l0 = *(uint32_t*)(src - !!top*sstride);
  uint32_t l1 = *(uint32_t*)(src);
  uint32_t l2 = *(uint32_t*)(src + sstride);
  uint32_t l3 = *(uint32_t*)(src + 2*sstride);
  uint32_t l4 = *(uint32_t*)(src + 3*sstride);
  uint32_t l5 = *(uint32_t*)(src + (3+(bottom != 3))*sstride);
  v128 c128 = v128_dup_8(128);
  v128 o = v128_from_32(l1, l2, l3, l4);
  v128 a = v128_from_32(l0, l1, l2, l3);
  v128 b = v128_from_32(*(uint32_t*)(src - !!left),
                        *(uint32_t*)(src + sstride - !!left),
                        *(uint32_t*)(src + 2*sstride - !!left),
                        *(uint32_t*)(src + 3*sstride - !!left));
  v128 c = v128_from_32(*(uint32_t*)(src + (right != 3)),
                        *(uint32_t*)(src + sstride + (right != 3)),
                        *(uint32_t*)(src + 2*sstride + (right != 3)),
                        *(uint32_t*)(src + 3*sstride + (right != 3)));
  v128 d = v128_from_32(l2, l3, l4, l5);
  v128 x = v128_sub_8(o, c128);
  a = v128_sub_8(a, c128);
  b = v128_sub_8(b, c128);
  c = v128_sub_8(c, c128);
  d = v128_sub_8(d, c128);
  if (!left)
    b = v128_shuffle_8(b, v128_from_v64(v64_from_64(0x0e0d0c0c0a090808LL),
                                        v64_from_64(0x0605040402010000LL)));
  if (right == 3)
    c = v128_shuffle_8(c, v128_from_v64(v64_from_64(0x0f0f0e0d0b0b0a09LL),
                                        v64_from_64(0x0707060503030201LL)));
  v128 c2 = v128_dup_8(-2);
  v128 r1 = v128_add_8(v128_add_8(v128_cmplt_s8(a, x), v128_cmplt_s8(b, x)),
                       v128_add_8(v128_cmplt_s8(c, x), v128_cmplt_s8(d, x)));
  v128 r2 = v128_add_8(v128_add_8(v128_cmpgt_s8(a, x), v128_cmpgt_s8(b, x)),
                       v128_add_8(v128_cmpgt_s8(c, x), v128_cmpgt_s8(d, x)));
  v128 delta = v128_sub_8(v128_cmplt_s8(r1, c2), v128_cmplt_s8(r2, c2));
  v128 r = v128_add_8(o, delta);
  *(uint32_t*)dst = v128_low_u32(v128_shr_n_byte(r, 12));
  *(uint32_t*)(dst + dstride) = v128_low_u32(v128_shr_n_byte(r, 8));
  *(uint32_t*)(dst + 2*dstride) = v128_low_u32(v128_shr_n_byte(r, 4));
  *(uint32_t*)(dst + 3*dstride) = v128_low_u32(r);
}

void clpf_block8(const uint8_t *src, uint8_t *dst, int sstride, int dstride, int x0, int y0, int width, int height) {
  int left = (x0 & ~(MAX_SB_SIZE-1)) - x0;
  int top = (y0 & ~(MAX_SB_SIZE-1)) - y0;
  int right = min(width-1, left + MAX_SB_SIZE-1);
  int bottom = min(height-1, top + MAX_SB_SIZE-1);
  v64 c2 = v64_dup_8(-2);
  v64 c128 = v64_dup_8(128);
  v64 s1 = left ? v64_from_64(0x0706050403020100LL) : v64_from_64(0x0605040302010000LL);
  v64 s2 = right == 7 ? v64_from_64(0x0707060504030201LL) : v64_from_64(0x0706050403020100LL);

  dst -= left + top*dstride;
  src += x0 + y0*sstride;

  for (int y = 0; y < 8; y++) {
    v64 o = v64_load_aligned(src);
    v64 x = v64_sub_8(o, c128);
    v64 a = v64_sub_8(v64_load_aligned(src - (y!=top)*sstride), c128);
    v64 b = v64_shuffle_8(v64_sub_8(v64_load_unaligned(src - !!left), c128), s1);
    v64 c = v64_shuffle_8(v64_sub_8(v64_load_unaligned(src + (right != 7)), c128), s2);
    v64 d = v64_sub_8(v64_load_aligned(src + (y!=bottom)*sstride), c128);
    v64 r1 = v64_add_8(v64_add_8(v64_cmplt_s8(a, x), v64_cmplt_s8(b, x)),
                       v64_add_8(v64_cmplt_s8(c, x), v64_cmplt_s8(d, x)));
    v64 r2 = v64_add_8(v64_add_8(v64_cmpgt_s8(a, x), v64_cmpgt_s8(b, x)),
                       v64_add_8(v64_cmpgt_s8(c, x), v64_cmpgt_s8(d, x)));
    v64 delta = v64_sub_8(v64_cmplt_s8(r1, c2), v64_cmplt_s8(r2, c2));
    v64_store_aligned(dst, v64_add_8(o, delta));
    src += sstride;
    dst += dstride;
  }
}
