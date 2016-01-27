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

#include <assert.h>
#include <stdlib.h>
#include "global.h"
#include "putbits.h"
#include "putvlc.h"
#include "simd.h"

static void flush_bitbuf(stream_t *str)
{
  if ((str->bytepos+4) > str->bytesize)
  {
    fatalerror("Run out of bits in stream buffer.");
  }
  str->bitstream[str->bytepos++] = (str->bitbuf >> 24) & 0xff;
  str->bitstream[str->bytepos++] = (str->bitbuf >> 16) & 0xff;
  str->bitstream[str->bytepos++] = (str->bitbuf >> 8) & 0xff;
  str->bitstream[str->bytepos++] = str->bitbuf & 0xff;
  str->bitbuf = 0;
  str->bitrest = 32;
}

static unsigned int mask(unsigned int n)
{
  return (1 << n) - 1;
}

static void putbits(unsigned int n, unsigned int val, stream_t *str)
{
  unsigned int rest;

  if (n <= str->bitrest)
  {
    str->bitbuf |= ((val & mask(n)) << (str->bitrest-n));
    str->bitrest -= n;
  }
  else
  {
    rest = n-str->bitrest;
    str->bitbuf |= (val >> rest) & mask(n-rest);
    flush_bitbuf(str);
    str->bitbuf |= (val & mask(rest)) << (32-rest);
    str->bitrest -= rest;
  }
}


unsigned int put_vlc(int n,unsigned int cn,stream_t *str)
{
  if (n < 0) {
    putbits(-n, cn, str);
    return -n;
  }

  unsigned int len,tmp;
  unsigned int code;
  unsigned int e = 5;

  switch (n) {
  case 6:
  case 7:
    if (!cn) {
      putbits(2, 2, str);
      return 2;
    }
    if (n == 6) {
      cn++;
      n = 2;
    } else {
      if (cn == 1)  {
        putbits(3, 6, str);
        return 3;
      }
      if (cn < 4) {
        putbits(3, 7, str);
        putbits(1, cn & 1, str);
        return 4;
      }
      cn += 4;
      n = 3;
    }
    // Intentional fallthrough
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
    if ((int)cn < (e * (1 << n)))
    {
      tmp = 1<<n;
      code = tmp+(cn & (tmp-1));
      len = 1+n+(cn>>n);
    }
    else
    {
      code = cn - (e * (1 << n)) + (1 << n);
      len = (e-n)+1+2*log2i(code);
    }
    break;
  case 8:
    if (cn > 9)
      fatalerror("Code too large for VLC.");
    if (cn < 6) {
      len = 2 + (cn >> 1);
      code = 2 + (cn & 1);
    } else {
      len = 5;
      code = (cn - 6);
    }
    break;
  case 10:
    code = cn+1;
    len = 1+2*log2i(code);
    break;
  case 11:
  case 12:
  case 13:
  case 14:
  case 15:
  case 16:
  case 17:
  case 18:
    if (cn > n - 10)
      fatalerror("Code too large for VLC.");
    len = cn == n - 10 ? n - 10 : cn + 1;
    code = cn != n - 10;
    break;
  default:
    fatalerror("No such VLC table, only 0-18 allowed.");
  }
  putbits(len,code,str);
  return len;
}

