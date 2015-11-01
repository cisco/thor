/*Daala video codec
Copyright (c) 2003-2013 Daala project contributors.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

/*Some common macros for potential platform-specific optimization.*/
#if !defined(_odintrin_H)
# define _odintrin_H (1)

# if defined(_MSC_VER)
#  define _USE_MATH_DEFINES
# endif

# include <math.h>
# include <limits.h>
# include <string.h>
# include "global.h"
# include "simd.h"

# if defined(__GNUC__) && defined(__GNUC_MINOR__) \
 && defined(__GNUC_PATCHLEVEL__)
#  define OD_GNUC_PREREQ(maj, min, pat) \
 ((__GNUC__ << 16) + (__GNUC_MINOR__ << 8) + __GNUC_PATCHLEVEL__ >= \
 ((maj) << 16) + ((min) << 8) + pat)
# else
#  define OD_GNUC_PREREQ(maj, min, pat) (0)
# endif

# if OD_GNUC_PREREQ(3, 4, 0)
#  define OD_WARN_UNUSED_RESULT __attribute__((__warn_unused_result__))
# else
#  define OD_WARN_UNUSED_RESULT
# endif
# if OD_GNUC_PREREQ(3, 4, 0)
#  define OD_ARG_NONNULL(x) __attribute__((__nonnull__(x)))
# else
#  define OD_ARG_NONNULL(x)
# endif

# if defined(OD_ENABLE_ASSERTIONS)

#  define OD_M2STR_WRAPPER(_m) #_m
#  define OD_M2STR(_m) OD_M2STR_WRAPPER(_m)

#  define OD_FATAL(_str) \
 (fatalerror("Fatal (internal) error in " \
  __FILE__ ", line " OD_M2STR(__LINE__) ": " _str))

#  define OD_ASSERT(_cond) \
  do { \
    if (!(_cond)) { \
      OD_FATAL("assertion failed: " # _cond); \
    } \
  } \
  while (0)

#  define OD_ASSERT2(_cond, _message) \
  do { \
    if (!(_cond)) { \
      OD_FATAL("assertion failed: " # _cond "\n" _message); \
    } \
  } \
  while (0)

#  define OD_ALWAYS_TRUE(_cond) OD_ASSERT(_cond)

# else
#  define OD_ASSERT(_cond)
#  define OD_ASSERT2(_cond, _message)
#  define OD_ALWAYS_TRUE(_cond) ((void)(_cond))
# endif

# if !defined(M_PI)
#  define M_PI      (3.1415926535897932384626433832795)
# endif

# if !defined(M_SQRT2)
#  define M_SQRT2 (1.41421356237309504880168872420970)
# endif

# if !defined(M_SQRT1_2)
#  define M_SQRT1_2 (0.70710678118654752440084436210485)
# endif

# if !defined(M_LOG2E)
#  define M_LOG2E (1.4426950408889634073599246810019)
# endif

/*Some specific platforms may have optimized intrinsic or inline assembly
   versions of these functions which can substantially improve performance.
  We define macros for them to allow easy incorporation of these non-ANSI
   features.*/

/*Note that we do not provide a macro for abs(), because it is provided as a
   library function, which we assume is translated into an intrinsic to avoid
   the function call overhead and then implemented in the smartest way for the
   target platform.
  With modern gcc (4.x), this is true: it uses cmov instructions if the
   architecture supports it and branchless bit-twiddling if it does not (the
   speed difference between the two approaches is not measurable).
  Interestingly, the bit-twiddling method was patented in 2000 (US 6,073,150)
   by Sun Microsystems, despite prior art dating back to at least 1996:
   http://web.archive.org/web/19961201174141/www.x86.org/ftp/articles/pentopt/PENTOPT.TXT
  On gcc 3.x, however, our assumption is not true, as abs() is translated to a
   conditional jump, which is horrible on deeply piplined architectures (e.g.,
   all consumer architectures for the past decade or more).
  Also be warned that -C*abs(x) where C is a constant is mis-optimized as
   abs(C*x) on every gcc release before 4.2.3.
  See bug http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34130 */

/*Modern gcc (4.x) can compile the naive versions of min and max with cmov if
   given an appropriate architecture, but the branchless bit-twiddling versions
   are just as fast, and do not require any special target architecture.
  Earlier gcc versions (3.x) compiled both code to the same assembly
   instructions, because of the way they represented ((b) > (a)) internally.*/
/*#define OD_MAXI(a, b) ((a) < (b) ? (b) : (a))*/
# define OD_MAXI(a, b) ((a) ^ (((a) ^ (b)) & -((b) > (a))))
/*#define OD_MINI(a, b) ((a) > (b) ? (b) : (a))*/
# define OD_MINI(a, b) ((a) ^ (((b) ^ (a)) & -((b) < (a))))
/*This has a chance of compiling branchless, and is just as fast as the
   bit-twiddling method, which is slightly less portable, since it relies on a
   sign-extended rightshift, which is not guaranteed by ANSI (but present on
   every relevant platform).*/
# define OD_SIGNI(a) (((a) > 0) - ((a) < 0))
/*Slightly more portable than relying on a sign-extended right-shift (which is
   not guaranteed by ANSI), and just as fast, since gcc (3.x and 4.x both)
   compile it into the right-shift anyway.*/
# define OD_SIGNMASK(a) (-((a) < 0))
/*Unlike copysign(), simply inverts the sign of a if b is negative.*/
# define OD_FLIPSIGNI(a, b) (((a) + OD_SIGNMASK(b)) ^ OD_SIGNMASK(b))
# define OD_COPYSIGNI(a, b) OD_FLIPSIGNI(abs(a), b)
/*Clamps an integer into the given range.
  If a > c, then the lower bound a is respected over the upper bound c (this
   behavior is required to meet our documented API behavior).
  a: The lower bound.
  b: The value to clamp.
  c: The upper boud.*/
# define OD_CLAMPI(a, b, c) (OD_MAXI(a, OD_MINI(b, c)))
/*Clamps a signed integer between 0 and 255, returning an unsigned char.
  This assumes a char is 8 bits.*/
# define OD_CLAMP255(x) \
  ((unsigned char)((((x) < 0) - 1) & ((x) | -((x) > 255))))
/*Divides a signed integer by a positive value with exact rounding.*/
# define OD_DIV_ROUND(x, y) (((x) + OD_FLIPSIGNI((y) >> 1, x))/(y))
# define OD_DIV_R0(x, y) (((x) + OD_FLIPSIGNI((((y) + 1) >> 1) - 1, (x)))/(y))
# define OD_DIV_RE(x, y) \
  (((x) + OD_FLIPSIGNI((((y) + 1) >> 1) - 1 + ((x)/(y) & 1), (x)))/(y))
/*Divides an integer by a power of two, truncating towards 0.
  dividend: The integer to divide.
  shift: The non-negative power of two to divide by.
  rmask: (1 << shift) - 1*/
# define OD_DIV_POW2(dividend, shift, rmask) \
  (((dividend) + (OD_SIGNMASK(dividend) & (rmask))) >> (shift))
/*Divides x by 65536, truncating towards 0.*/
# define OD_DIV2_16(x) OD_DIV_POW2(x, 16, 0xFFFF)
/*Divides x by 2, truncating towards 0.*/
# define OD_DIV2(x) OD_DIV_POW2(x, 1, 0x1)
/*Divides x by 8, truncating towards 0.*/
# define OD_DIV8(x) OD_DIV_POW2(x, 3, 0x7)
/*Divides x by 16, truncating towards 0.*/
# define OD_DIV16(x) OD_DIV_POW2(x, 4, 0xF)
/*Right shifts dividend by shift, adding rval, and subtracting one for
   negative dividends first.
  When rval is (1 << (shift - 1)), this is equivalent to division with rounding
   ties away from zero.*/
# define OD_DIV_ROUND_POW2(dividend, shift, rval) \
  (((dividend) + OD_SIGNMASK(dividend) + (rval)) >> (shift))
/*Divides a x by 2, rounding towards even numbers.*/
# define OD_DIV2_RE(x) ((x) + ((x) >> 1 & 1) >> 1)
/*Divides a x by (1 << (shift)), rounding towards even numbers.*/
# define OD_DIV_POW2_RE(x, shift) \
  ((x) + (((1 << (shift)) + ((x) >> (shift) & 1) - 1) >> 1) >> (shift))
/*Count leading zeros.
  This macro should only be used for implementing od_ilog(), if it is defined.
  All other code should use OD_ILOG() instead.*/
# if defined(_MSC_VER)
#  include <intrin.h>
#  if !defined(snprintf)
#   define snprintf _snprintf
#  endif
/*In _DEBUG mode this is not an intrinsic by default.*/
#  pragma intrinsic(_BitScanReverse)

static __inline int od_bsr(unsigned long x) {
  unsigned long ret;
  _BitScanReverse(&ret, x);
  return (int)ret;
}
#  define OD_CLZ0 (1)
#  define OD_CLZ(x) (-od_bsr(x))
# elif defined(ENABLE_TI_DSPLIB)
#  include "dsplib.h"
#  define OD_CLZ0 (31)
#  define OD_CLZ(x) (_lnorm(x))
# elif OD_GNUC_PREREQ(3, 4, 0)
#  if INT_MAX >= 2147483647
#   define OD_CLZ0 ((int)sizeof(unsigned)*CHAR_BIT)
#   define OD_CLZ(x) (__builtin_clz(x))
#  elif LONG_MAX >= 2147483647L
#   define OD_CLZ0 ((int)sizeof(unsigned long)*CHAR_BIT)
#   define OD_CLZ(x) (__builtin_clzl(x))
#  endif
# endif
# if defined(OD_CLZ)
#  define OD_ILOG_NZ(x) (OD_CLZ0 - OD_CLZ(x))
/*Note that __builtin_clz is not defined when x == 0, according to the gcc
   documentation (and that of the x86 BSR instruction that implements it), so
   we have to special-case it.
  We define a special version of the macro to use when x can be zero.*/
#  define OD_ILOG(x) (OD_ILOG_NZ(x) & -!!(x))
# else
#  define OD_ILOG_NZ(x) (1 + log2i(x))
#  define OD_ILOG(x) (1 + log2i(x))
# endif

# define OD_LOG2(x) (M_LOG2E*log(x))

/*Swaps two integers a and b if a > b.*/
/*#define OD_SORT2I(a, b) \
  if ((a) > (b)) { \
    int t__; \
    t__ = (a); \
    (a) = (b); \
    (b) = t__; \
  }*/
/*This branchless version is significantly faster than the above
   straightforward implementation on modern processors.*/
# define OD_SORT2I(a, b) \
  do { \
    int t__; \
    t__ = ((a) ^ (b)) & -((b) < (a)); \
    (a) ^= t__; \
    (b) ^= t__; \
  } \
  while (0)

/*All of these macros should expect floats as arguments.*/
/*These two should compile as a single SSE instruction.*/
# define OD_MINF(a, b) ((a) < (b) ? (a) : (b))
# define OD_MAXF(a, b) ((a) > (b) ? (a) : (b))
# define OD_CLAMPF(a, b, c) (OD_MAXF(a, OD_MINF(b, c)))
# if defined(__GNUC__)
#  define OD_FABSF(f) (fabsf(f))
#  define OD_SQRTF(f) (sqrtf(f))
#  define OD_POWF(b, e) (powf(b, e))
#  define OD_LOGF(f) (logf(f))
#  define OD_IFLOORF(f) (floorf(f))
#  define OD_ICEILF(f) (ceilf(f))
# else
#  define OD_FABSF(f) ((float)fabs(f))
#  define OD_SQRTF(f) ((float)sqrt(f))
#  define OD_POWF(b, e) ((float)pow(b, e))
#  define OD_LOGF(f) ((float)log(f))
#  define OD_IFLOORF(f) ((int)floor(f))
#  define OD_ICEILF(f)  ((int)ceil(f))
# endif

/** Copy n elements of memory from src to dst. The 0* term provides
    compile-time type checking  */
#if !defined(OVERRIDE_OD_COPY)
# define OD_COPY(dst, src, n) \
  (memcpy((dst), (src), sizeof(*(dst))*(n) + 0*((dst) - (src))))
#endif

/** Copy n elements of memory from src to dst, allowing overlapping regions.
    The 0* term provides compile-time type checking */
#if !defined(OVERRIDE_OD_MOVE)
# define OD_MOVE(dst, src, n) \
 (memmove((dst), (src), sizeof(*(dst))*(n) + 0*((dst) - (src)) ))
#endif

/** Set n elements of dst to zero */
#if !defined(OVERRIDE_OD_CLEAR)
# define OD_CLEAR(dst, n) (memset((dst), 0, sizeof(*(dst))*(n)))
#endif

/** Linkage will break without this if using a C++ compiler, and will issue
 * warnings without this for a C compiler*/
#if defined(__cplusplus)
# define OD_EXTERN extern
#else
# define OD_EXTERN
#endif

/*Some assembly constructs require aligned operands.
  The following macros are _only_ intended for structure member declarations.
  Although they will sometimes work on stack variables, gcc will often silently
   ignore them.
  A separate set of macros could be made for manual stack alignment, but we
   don't actually require it anywhere.*/
# if defined(OD_X86ASM)||defined(OD_ARMASM)
#  if defined(__GNUC__)
#   define OD_ALIGN8(expr) expr __attribute__((aligned(8)))
#   define OD_ALIGN16(expr) expr __attribute__((aligned(16)))
#  elif defined(_MSC_VER)
#   define OD_ALIGN8(expr) __declspec (align(8)) expr
#   define OD_ALIGN16(expr) __declspec (align(16)) expr
#  else
#   error "Alignment macros required for this platform."
#  endif
# endif

# if !defined(OD_ALIGN8)
#  define OD_ALIGN8(expr) expr
# endif
# if !defined(OD_ALIGN16)
#  define OD_ALIGN16(expr) expr
# endif

#endif
