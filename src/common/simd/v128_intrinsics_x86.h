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

#ifndef _V128_INTRINSICS_H
#define _V128_INTRINSICS_H

#include "v64_intrinsics_x86.h"

typedef __m128i v128;


SIMD_INLINE uint32_t v128_low_u32(v128 a) {
  return (uint32_t)_mm_cvtsi128_si32(a);
}

SIMD_INLINE v64 v128_low_v64(v128 a) {
  return _mm_unpacklo_epi64(a, v64_zero());
}

SIMD_INLINE v64 v128_high_v64(v128 a) {
  return _mm_srli_si128(a, 8);
}

SIMD_INLINE v128 v128_from_64(uint64_t a, uint64_t b) {
  return _mm_set_epi64(_mm_cvtsi64_m64(a), _mm_cvtsi64_m64(b));
}

SIMD_INLINE v128 v128_from_v64(v64 a, v64 b) {
  return _mm_unpacklo_epi64(b, a);
}

SIMD_INLINE v128 v128_from_32(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
  return _mm_set_epi32(a, b, c, d);
}



SIMD_INLINE v128 v128_load_aligned(const void *p) {
  return _mm_load_si128((__m128i*)p);
}

SIMD_INLINE v128 v128_load_unaligned(const void *p) {
#if defined (__SSSE3__)
  return (__m128i)_mm_lddqu_si128((__m128i *)p);
#else
  return _mm_loadu_si128((__m128i *)p);
#endif
}

SIMD_INLINE void v128_store_aligned(void *p, v128 a) {
  _mm_store_si128((__m128i*)p, a);
}

SIMD_INLINE void v128_store_unaligned(void *p, v128 a) {
  _mm_storeu_si128((__m128i*)p, a);
}


SIMD_INLINE v128 v128_align(v128 a, v128 b, const unsigned int c) {
#if defined(__OPTIMIZE__) || defined(_MSC_VER)
#if defined (__SSSE3__)
  return c ? _mm_alignr_epi8(a, b, c) : b;
#else
  return c ? _mm_or_si128(_mm_srli_si128(b, c), _mm_slli_si128(a, 16 - c)) : b;
#endif
#else
  return c < 8 ?
    v128_from_v64(v64_align(v128_low_v64(a), v128_high_v64(b), c),
                  v64_align(v128_high_v64(b), v128_low_v64(b), c)) :
    v128_from_v64(v64_align(v128_high_v64(a), v128_low_v64(a), c-8),
                  v64_align(v128_low_v64(a), v128_high_v64(b), c-8));
#endif
}


SIMD_INLINE v128  v128_zero() {
  return _mm_setzero_si128();
}

SIMD_INLINE v128 v128_dup_8(uint8_t x) {
  return _mm_set1_epi8(x);
}

SIMD_INLINE v128 v128_dup_16(uint16_t x) {
  return _mm_set1_epi16(x);
}

SIMD_INLINE v128 v128_dup_32(uint32_t x) {
  return _mm_set1_epi32(x);
}



SIMD_INLINE v128 v128_add_8(v128 a, v128 b) {
  return _mm_add_epi8(a, b);
}

SIMD_INLINE v128 v128_add_16(v128 a, v128 b) {
  return _mm_add_epi16(a, b);
}

SIMD_INLINE v128 v128_sadd_s16(v128 a, v128 b) {
  return _mm_adds_epi16(a, b);
}

SIMD_INLINE v128 v128_add_32(v128 a, v128 b) {
  return _mm_add_epi32(a, b);
}

SIMD_INLINE v128 v128_padd_s16(v128 a) {
  return _mm_madd_epi16(a, _mm_set1_epi16(1));
}

SIMD_INLINE v128 v128_sub_8(v128 a, v128 b) {
  return _mm_sub_epi8(a, b);
}

SIMD_INLINE v128 v128_ssub_u8(v128 a, v128 b) {
  return _mm_subs_epu8(a, b);
}

SIMD_INLINE v128 v128_sub_16(v128 a, v128 b) {
  return _mm_sub_epi16(a, b);
}

SIMD_INLINE v128 v128_ssub_s16(v128 a, v128 b) {
  return _mm_subs_epi16(a, b);
}

SIMD_INLINE v128 v128_sub_32(v128 a, v128 b) {
  return _mm_sub_epi32(a, b);
}

SIMD_INLINE v128 v128_abs_s16(v128 a) {
#if defined (__SSSE3__)
  return _mm_abs_epi16(a);
#else
  return _mm_max_epi16(a, _mm_sub_epi16(_mm_setzero_si128(), a));
#endif
}



SIMD_INLINE v128 v128_ziplo_8(v128 a, v128 b) {
  return _mm_unpacklo_epi8(b, a);
}

SIMD_INLINE v128 v128_ziphi_8(v128 a, v128 b) {
  return _mm_unpackhi_epi8(b, a);
}

SIMD_INLINE v128 v128_ziplo_16(v128 a, v128 b) {
  return _mm_unpacklo_epi16(b, a);
}

SIMD_INLINE v128 v128_ziphi_16(v128 a, v128 b) {
  return _mm_unpackhi_epi16(b, a);
}

SIMD_INLINE v128 v128_ziplo_32(v128 a, v128 b) {
  return _mm_unpacklo_epi32(b, a);
}

SIMD_INLINE v128 v128_ziphi_32(v128 a, v128 b) {
  return _mm_unpackhi_epi32(b, a);
}

SIMD_INLINE v128 v128_ziplo_64(v128 a, v128 b) {
  return _mm_unpacklo_epi64(b, a);
}

SIMD_INLINE v128 v128_ziphi_64(v128 a, v128 b) {
  return _mm_unpackhi_epi64(b, a);
}

SIMD_INLINE v128 v128_zip_8(v64 a, v64 b) {
  return _mm_unpacklo_epi8(b, a);
}

SIMD_INLINE v128 v128_zip_16(v64 a, v64 b) {
  return _mm_unpacklo_epi16(b, a);
}

SIMD_INLINE v128 v128_zip_32(v64 a, v64 b) {
  return _mm_unpacklo_epi32(b, a);
}

SIMD_INLINE v128 v128_unziphi_8(v128 a, v128 b) {
  return _mm_packs_epi16(_mm_srai_epi16(b, 8), _mm_srai_epi16(a, 8));
}

SIMD_INLINE v128 v128_unziplo_8(v128 a, v128 b) {
#if defined (__SSSE3__)
  v128 order = _mm_cvtsi64_si128(0x0e0c0a0806040200LL);
  return _mm_unpacklo_epi64(_mm_shuffle_epi8(b, order), _mm_shuffle_epi8(a, order));
#else
  return v128_unziphi_8(_mm_slli_si128(a, 1), _mm_slli_si128(b, 1));
#endif
}

SIMD_INLINE v128 v128_unziphi_16(v128 a, v128 b) {
  return _mm_packs_epi32(_mm_srai_epi32(b, 16), _mm_srai_epi32(a, 16));
}

SIMD_INLINE v128 v128_unziplo_16(v128 a, v128 b) {
#if defined (__SSSE3__)
  v128 order = _mm_cvtsi64_si128(0x0d0c090805040100LL);
  return _mm_unpacklo_epi64(_mm_shuffle_epi8(b, order), _mm_shuffle_epi8(a, order));
#else
  return v128_unziphi_16(_mm_slli_si128(a, 2), _mm_slli_si128(b, 2));
#endif
}

SIMD_INLINE v128 v128_unziphi_32(v128 a, v128 b) {
  return _mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(b), _mm_castsi128_ps(a), _MM_SHUFFLE(3, 1, 3, 1)));
}

SIMD_INLINE v128 v128_unziplo_32(v128 a, v128 b) {
  return _mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(b), _mm_castsi128_ps(a), _MM_SHUFFLE(2, 0, 2, 0)));
}

SIMD_INLINE v128 v128_unpack_u8_s16(v64 a) {
  return _mm_unpacklo_epi8(a, _mm_setzero_si128());
}

SIMD_INLINE v128 v128_unpacklo_u8_s16(v128 a) {
  return _mm_unpacklo_epi8(a, _mm_setzero_si128());
}

SIMD_INLINE v128 v128_unpackhi_u8_s16(v128 a) {
  return _mm_unpackhi_epi8(a, _mm_setzero_si128());
}

SIMD_INLINE v128 v128_pack_s32_s16(v128 a, v128 b) {
  return _mm_packs_epi32(b, a);
}

SIMD_INLINE v128 v128_pack_s16_u8(v128 a, v128 b) {
  return _mm_packus_epi16(b, a);
}

SIMD_INLINE v128 v128_pack_s16_s8(v128 a, v128 b) {
  return _mm_packs_epi16(b, a);
}

SIMD_INLINE v128 v128_unpack_u16_s32(v64 a) {
  return _mm_unpacklo_epi16(a, _mm_setzero_si128());
}

SIMD_INLINE v128 v128_unpack_s16_s32(v64 a) {
  return _mm_srai_epi32(_mm_unpacklo_epi16(a, a), 16);
}

SIMD_INLINE v128 v128_unpacklo_u16_s32(v128 a) {
  return _mm_unpacklo_epi16(a, _mm_setzero_si128());
}

SIMD_INLINE v128 v128_unpacklo_s16_s32(v128 a) {
  return _mm_srai_epi32(_mm_unpacklo_epi16(a, a), 16);
}

SIMD_INLINE v128 v128_unpackhi_u16_s32(v128 a) {
  return _mm_unpackhi_epi16(a, _mm_setzero_si128());
}

SIMD_INLINE v128 v128_unpackhi_s16_s32(v128 a) {
  return _mm_srai_epi32(_mm_unpackhi_epi16(a, a), 16);
}

SIMD_INLINE v128 v128_shuffle_8(v128 x, v128 pattern) {
#if defined (__SSSE3__)
  return _mm_shuffle_epi8(x, pattern);
#else
  v128 output;
  unsigned char *input = (unsigned char *)&x;
  unsigned char *index = (unsigned char *)&pattern;
  char *selected = (char *)&output;
  int counter;

  for (counter = 0; counter < 16; counter++) {
    selected[counter] = input[index[counter] & 15];
  }

  return output;
#endif
}



SIMD_INLINE int64_t v128_dotp_s16(v128 a, v128 b) {
  v128 r = _mm_madd_epi16(a, b);
#if defined (__SSE4_1__)
  v128 c = _mm_add_epi64(_mm_cvtepi32_epi64(r), _mm_cvtepi32_epi64(_mm_srli_si128(r, 8)));
  return _mm_cvtsi128_si64(_mm_add_epi64(c, _mm_srli_si128(c, 8)));
#else
  return (int64_t)_mm_cvtsi128_si32(r) +
    (int64_t)_mm_cvtsi128_si32(_mm_srli_si128(r, 4)) +
    (int64_t)_mm_cvtsi128_si32(_mm_srli_si128(r, 8)) +
    (int64_t)_mm_cvtsi128_si32(_mm_srli_si128(r, 12));
#endif
}

SIMD_INLINE uint64_t v128_hadd_u8(v128 a) {
  v128 t = _mm_sad_epu8(a, _mm_setzero_si128());
  return v64_low_u32(v128_low_v64(t)) + v64_low_u32(v128_high_v64(t));
}

typedef v128 sad128_internal;

SIMD_INLINE sad128_internal v128_sad_u8_init() {
  return _mm_setzero_si128();
}

/* Implementation dependent return value.  Result must be finalised with v128_sad_sum().
   The result for more than 32 v128_sad_u8() calls is undefined. */
SIMD_INLINE sad128_internal v128_sad_u8(sad128_internal s, v128 a, v128 b) {
  return _mm_add_epi64(s, _mm_sad_epu8(a, b));
}

SIMD_INLINE uint32_t v128_sad_u8_sum(sad128_internal s) {
  return v128_low_u32(_mm_add_epi32(s, _mm_unpackhi_epi64(s, s)));
}

typedef v128 ssd128_internal;

SIMD_INLINE ssd128_internal v128_ssd_u8_init() {
  return _mm_setzero_si128();
}

/* Implementation dependent return value.  Result must be finalised with v128_ssd_sum(). */
SIMD_INLINE ssd128_internal v128_ssd_u8(ssd128_internal s, v128 a, v128 b) {
  v128 l = _mm_sub_epi16(_mm_unpacklo_epi8(a, _mm_setzero_si128()),
                         _mm_unpacklo_epi8(b, _mm_setzero_si128()));
  v128 h = _mm_sub_epi16(_mm_unpackhi_epi8(a, _mm_setzero_si128()),
                         _mm_unpackhi_epi8(b, _mm_setzero_si128()));
  v128 rl = _mm_madd_epi16(l, l);
  v128 rh = _mm_madd_epi16(h, h);
  v128 c = _mm_cvtsi32_si128(32);
  rl = _mm_add_epi32(rl, _mm_srli_si128(rl, 8));
  rl = _mm_add_epi32(rl, _mm_srli_si128(rl, 4));
  rh = _mm_add_epi32(rh, _mm_srli_si128(rh, 8));
  rh = _mm_add_epi32(rh, _mm_srli_si128(rh, 4));
  return _mm_add_epi64(s, _mm_srl_epi64(_mm_sll_epi64(_mm_unpacklo_epi64(rl, rh), c), c));
}

SIMD_INLINE uint32_t v128_ssd_u8_sum(ssd128_internal s) {
  return v128_low_u32(_mm_add_epi32(s, _mm_unpackhi_epi64(s, s)));
}



SIMD_INLINE v128 v128_or(v128 a, v128 b) {
  return _mm_or_si128(a, b);
}

SIMD_INLINE v128 v128_xor(v128 a, v128 b) {
  return _mm_xor_si128(a, b);
}

SIMD_INLINE v128 v128_and(v128 a, v128 b) {
  return _mm_and_si128(a, b);
}

SIMD_INLINE v128 v128_andn(v128 a, v128 b) {
  return _mm_andnot_si128(b, a);
}



SIMD_INLINE v128 v128_mul_s16(v64 a, v64 b) {
  v64 lo_bits = v64_mullo_s16(a, b);
  v64 hi_bits = v64_mulhi_s16(a, b);
  return v128_from_v64(v64_ziphi_16(hi_bits, lo_bits), v64_ziplo_16(hi_bits, lo_bits));
}

SIMD_INLINE v128 v128_mullo_s16(v128 a, v128 b) {
  return _mm_mullo_epi16(a, b);
}

SIMD_INLINE v128 v128_mulhi_s16(v128 a, v128 b) {
  return _mm_mulhi_epi16(a, b);
}

SIMD_INLINE v128 v128_mullo_s32(v128 a, v128 b) {
#if defined (__SSE4_1__)
  return _mm_mullo_epi32(a, b);
#else
  return _mm_unpacklo_epi32(_mm_shuffle_epi32(_mm_mul_epu32(a, b), 8),
                            _mm_shuffle_epi32(_mm_mul_epu32(_mm_srli_si128(a, 4), _mm_srli_si128(b, 4)), 8));
#endif
}

SIMD_INLINE v128 v128_madd_s16(v128 a, v128 b) {
  return _mm_madd_epi16(a, b);
}

SIMD_INLINE v128 v128_madd_us8(v128 a, v128 b) {
#if defined (__SSE3__)
  return _mm_maddubs_epi16(a, b);
#else
  return _mm_packs_epi32(_mm_madd_epi16(_mm_unpacklo_epi8(a, _mm_setzero_si128()),
                                        _mm_srai_epi16(_mm_unpacklo_epi8(b, b), 8)),
                         _mm_madd_epi16(_mm_unpackhi_epi8(a, _mm_setzero_si128()),
                                        _mm_srai_epi16(_mm_unpackhi_epi8(b, b), 8)));
#endif
}



SIMD_INLINE v128 v128_avg_u8(v128 a, v128 b) {
  return _mm_avg_epu8(a, b);
}

SIMD_INLINE v128 v128_rdavg_u8(v128 a, v128 b) {
  return _mm_sub_epi8(_mm_avg_epu8(a, b), _mm_and_si128(_mm_xor_si128(a, b), v128_dup_8(1)));
}

SIMD_INLINE v128 v128_avg_u16(v128 a, v128 b) {
  return _mm_avg_epu16(a, b);
}

SIMD_INLINE v128 v128_min_u8(v128 a, v128 b) {
  return _mm_min_epu8(a, b);
}

SIMD_INLINE v128 v128_max_u8(v128 a, v128 b) {
  return _mm_max_epu8(a, b);
}

SIMD_INLINE v128 v128_min_s16(v128 a, v128 b) {
  return _mm_min_epi16(a, b);
}

SIMD_INLINE v128 v128_max_s16(v128 a, v128 b) {
  return _mm_max_epi16(a, b);
}


SIMD_INLINE v128 v128_cmpgt_s8(v128 a, v128 b) {
  return _mm_cmpgt_epi8(a, b);
}

SIMD_INLINE v128 v128_cmpeq_8(v128 a, v128 b) {
  return _mm_cmpeq_epi8(a, b);
}

SIMD_INLINE v128 v128_cmpgt_s16(v128 a, v128 b) {
  return _mm_cmpgt_epi16(a, b);
}

SIMD_INLINE v128 v128_cmpeq_16(v128 a, v128 b) {
  return _mm_cmpeq_epi16(a, b);
}



SIMD_INLINE v128 v128_shl_8(v128 a, unsigned int c) {
  __m128i x = _mm_cvtsi32_si128(c);
  return _mm_packus_epi16(_mm_srli_epi16(_mm_sll_epi16(_mm_unpacklo_epi8(_mm_setzero_si128(), a), x), 8),
                          _mm_srli_epi16(_mm_sll_epi16(_mm_unpackhi_epi8(_mm_setzero_si128(), a), x), 8));
}

SIMD_INLINE v128 v128_shr_u8(v128 a, unsigned int c) {
  __m128i x = _mm_cvtsi32_si128(c + 8);
  return _mm_packus_epi16(_mm_srl_epi16(_mm_unpacklo_epi8(_mm_setzero_si128(), a), x),
                          _mm_srl_epi16(_mm_unpackhi_epi8(_mm_setzero_si128(), a), x));
}

SIMD_INLINE v128 v128_shr_s8(v128 a, unsigned int c) {
  __m128i x = _mm_cvtsi32_si128(c + 8);
  return _mm_packs_epi16(_mm_sra_epi16(_mm_unpacklo_epi8(_mm_setzero_si128(), a), x),
                         _mm_sra_epi16(_mm_unpackhi_epi8(_mm_setzero_si128(), a), x));
}

SIMD_INLINE v128 v128_shl_16(v128 a, unsigned int c) {
  return _mm_sll_epi16(a, _mm_cvtsi32_si128(c));
}

SIMD_INLINE v128 v128_shr_u16(v128 a, unsigned int c) {
  return _mm_srl_epi16(a, _mm_cvtsi32_si128(c));
}

SIMD_INLINE v128 v128_shr_s16(v128 a, unsigned int c) {
  return _mm_sra_epi16(a, _mm_cvtsi32_si128(c));
}

SIMD_INLINE v128 v128_shl_32(v128 a, unsigned int c) {
  return _mm_sll_epi32(a, _mm_cvtsi32_si128(c));
}

SIMD_INLINE v128 v128_shr_u32(v128 a, unsigned int c) {
  return _mm_srl_epi32(a, _mm_cvtsi32_si128(c));
}

SIMD_INLINE v128 v128_shr_s32(v128 a, unsigned int c) {
  return _mm_sra_epi32(a, _mm_cvtsi32_si128(c));
}

/* Fallback to variable argument if we're not compiling with optimise */
#if __OPTIMIZE__

SIMD_INLINE v128 v128_shl_n_byte(v64 a, const unsigned int c) {
  return _mm_slli_si128(a, c);
}

SIMD_INLINE v128 v128_shr_n_byte(v64 a, const unsigned int c) {
  return _mm_srli_si128(a, c);
}

SIMD_INLINE v128 v128_shl_n_8(v128 a, const unsigned int c) {
  return _mm_packus_epi16(_mm_srli_epi16(_mm_slli_epi16(_mm_unpacklo_epi8(_mm_setzero_si128(), a), c), 8),
                          _mm_srli_epi16(_mm_slli_epi16(_mm_unpackhi_epi8(_mm_setzero_si128(), a), c), 8) );
}

SIMD_INLINE v128 v128_shr_n_u8(v128 a, const unsigned int c) {
  return _mm_packus_epi16(_mm_srli_epi16(_mm_unpacklo_epi8(_mm_setzero_si128(), a), c + 8),
                          _mm_srli_epi16(_mm_unpackhi_epi8(_mm_setzero_si128(), a), c + 8));
}

SIMD_INLINE v128 v128_shr_n_s8(v128 a, const unsigned int c) {
  return _mm_packs_epi16(_mm_srai_epi16(_mm_unpacklo_epi8(_mm_setzero_si128(), a), c + 8),
                         _mm_srai_epi16(_mm_unpackhi_epi8(_mm_setzero_si128(), a), c + 8));
}

SIMD_INLINE v128 v128_shl_n_16(v128 a, const unsigned int c) {
  return _mm_slli_epi16(a, c);
}

SIMD_INLINE v128 v128_shr_n_u16(v128 a, const unsigned int c) {
  return _mm_srli_epi16(a, c);
}

SIMD_INLINE v128 v128_shr_n_s16(v128 a, const unsigned int c) {
  return _mm_srai_epi16(a, c);
}

SIMD_INLINE v128 v128_shl_n_32(v128 a, const unsigned int c) {
  return _mm_slli_epi32(a, c);
}

SIMD_INLINE v128 v128_shr_n_u32(v128 a, const unsigned int c) {
  return _mm_srli_epi32(a, c);
}

SIMD_INLINE v128 v128_shr_n_s32(v128 a, const unsigned int c) {
  return _mm_srai_epi32(a, c);
}

#else

SIMD_INLINE v128 v128_shl_n_byte(v128 a, const unsigned int n) {
  if (n < 8)
    return v128_from_v64(v64_or(v64_shl_n_byte(v128_high_v64(a), n),
                                v64_shr_n_byte(v128_low_v64(a), 8 - n)),
                         v64_shl_n_byte(v128_low_v64(a), n));
  else
    return v128_from_v64(v64_shl_n_byte(v128_low_v64(a), n - 8), v64_zero());
}

SIMD_INLINE v128 v128_shr_n_byte(v128 a, const unsigned int n) {
  if (n < 8)
    return v128_from_v64(v64_shr_n_byte(v128_high_v64(a), n),
                         v64_or(v64_shr_n_byte(v128_low_v64(a), n),
                                v64_shl_n_byte(v128_high_v64(a), 8 - n)));
  else
    return v128_from_v64(v64_zero(), v64_shr_n_byte(v128_high_v64(a), n - 8));
}

SIMD_INLINE v128 v128_shl_n_8(v128 a, const unsigned int c) {
  return v128_shl_8(a, c);
}

SIMD_INLINE v128 v128_shr_n_u8(v128 a, const unsigned int c) {
  return v128_shr_u8(a, c);
}

SIMD_INLINE v128 v128_shr_n_s8(v128 a, const unsigned int c) {
  return v128_shr_s8(a, c);
}

SIMD_INLINE v128 v128_shl_n_16(v128 a, const unsigned int c) {
  return v128_shl_16(a, c);
}

SIMD_INLINE v128 v128_shr_n_u16(v128 a, const unsigned int c) {
  return v128_shr_u16(a, c);
}

SIMD_INLINE v128 v128_shr_n_s16(v128 a, const unsigned int c) {
  return v128_shr_s16(a, c);
}

SIMD_INLINE v128 v128_shl_n_32(v128 a, const unsigned int c) {
  return v128_shl_32(a, c);
}

SIMD_INLINE v128 v128_shr_n_u32(v128 a, const unsigned int c) {
  return v128_shr_u32(a, c);
}

SIMD_INLINE v128 v128_shr_n_s32(v128 a, const unsigned int c) {
  return v128_shr_s32(a, c);
}

#endif

#endif /* _V128_INTRINSICS_H */
