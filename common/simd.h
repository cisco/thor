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

#ifndef _SIMD_H
#define _SIMD_H

#include <stdlib.h>
#include <assert.h>

#ifndef SIMD_INLINE
#ifdef __GNUC__
#define SIMD_INLINE static inline __attribute__((always_inline))
#elif __STDC_VERSION__ >= 199901L
#define SIMD_INLINE static inline
#else
#define SIMD_INLINE static
#endif
#endif

#include <stdint.h>

#if defined(__INTEL_COMPILER) || defined (VS_2015)
#define ALIGN(c) __declspec(align(c))
#elif __arm__
#define ALIGN(c) __attribute__((aligned(c)))
#elif __GNUC__
#define ALIGN(c) __attribute__((aligned(c)))
#endif

#ifndef ALIGN
#define ALIGN(c)
#endif

#if defined(_WIN32)
#include <intrin.h>

SIMD_INLINE unsigned int log2i(uint32_t x)
{
  unsigned long y;
  _BitScanReverse(&y, x);
  return y;
}

SIMD_INLINE void *thor_alloc(size_t size, uintptr_t align)
{
  void *m = malloc(size + sizeof(void*) + align);
  void **r = (void**)((((uintptr_t)m) + sizeof(void*) + align - 1) & ~(align - 1));
  r[-1] = m;
  return r;
}
SIMD_INLINE void thor_free(void *p)
{
  free(((void**)p)[-1]);
}

#elif __GNUC__
#include <alloca.h>

SIMD_INLINE unsigned int log2i(uint32_t x)
{
  return 31 - __builtin_clz(x);
}

SIMD_INLINE void *thor_alloc(size_t size, uintptr_t align)
{
  return (void*)((((uintptr_t)alloca(size + align)) + align - 1) & ~(align - 1));
}
SIMD_INLINE void thor_free(void *p) {}

#else

SIMD_INLINE unsigned int log2i(uint32_t n)
{
  assert(n > 0);
  int c = 0;
  while (n >>= 1)
    c++;
  return c;
}

/* Fallback to regular malloc */
SIMD_INLINE void *thor_alloc(size_t size, uintptr_t align)
{
  void *m = malloc(size + sizeof(void*) + align);
  void **r = (void**)((((uintptr_t)m) + sizeof(void*) + align - 1) & ~(align - 1));
  r[-1] = m;
  return r;
}
SIMD_INLINE void thor_free(void *p)
{
  free(((void**)p)[-1]);
}

#endif

static const int simd_check = 1;

#if defined(__ARM_NEON__) && defined(ALIGN)
static const int simd_available = 1;
#include "simd/v256_intrinsics_arm.h"
#elif (defined(__SSE2__) || _M_IX86_FP==2) && defined(ALIGN)
static const int simd_available = 1;
#include "simd/v256_intrinsics_x86.h"
#else
static const int simd_available = 0;
#include "simd/v256_intrinsics.h"
#endif

extern int use_simd;
SIMD_INLINE void init_use_simd()
{
  /* SIMD optimisations supported only for little endian architectures */
  const uint16_t t = 0x100;
  use_simd = simd_available && !*(const uint8_t *)&t;
}

#endif /* _SIMD_H */
