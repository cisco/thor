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

int put_vlc(unsigned int n,unsigned int cn,stream_t *str)
{
  unsigned int len,tmp;
  unsigned int code;

  switch (n) {
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
    if ((int)cn < (6 * (1 << n)))
    {
      tmp = 1<<n;
      code = tmp+(cn & (tmp-1));
      len = 1+n+(cn>>n);
    }
    else
    {
      code = cn - (6 * (1 << n)) + (1 << n);
      len = (6-n)+1+2*log2i(code);
    }
    break;
  case 6:
  case 7: //TODO: Remove this if not used
    tmp = 1<<(n-4);
    code = tmp+cn%tmp;
    len = 1+(n-4)+(cn>>(n-4));
      break;
  case 8:
    if (cn == 0)
    {
      code = 1;
      len = 1;
    }
    else if (cn == 1)
    {
      code = 1;
      len = 2;
    }
    else if (cn == 2)
    {
      code = 0;
      len = 2;
    }
    else fatalerror("Code number too large for VLC8.");
    break;
  case 9:
    if (cn == 0)
    {
      code = 4;
      len = 3;
    }
    else if (cn == 1)
    {
      code = 10;
      len = 4;
    }
    else if (cn == 2)
    {
      code = 11;
      len = 4;
    }
    else if (cn < 11)
    {
      code = cn+21;
      len = 5;
    }
    else
    {
      tmp = 1<<4;
      code = tmp+(cn+5)%tmp;
      len = 5+((cn+5)>>4);
    }
    break;
  case 10:
    code = cn+1;
    len = 1+2*log2i(code);
    break;
  case 11:
    len = cn < 2 ? cn + 1 : cn/2 + 3;
    code = cn < 2 ? 1 : 2 + (cn&1);
    break;
  case 12:
    len = min(4,cn+1);
    code = cn != 4;
    break;
  case 13:
    len = min(6,cn+1);
    code = cn != 6;
    break;
  default:
    fatalerror("No such VLC table, only 0-13 allowed.");
  }
  putbits(len,code,str);
  return len;
}

int quote_vlc(unsigned int n,unsigned int cn)
{
    unsigned int len,tmp;
    unsigned int code;

    switch (n) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      if ((int)cn < (6 * (1 << n)))
      {
        tmp = 1<<n;
        code = tmp+(cn & (tmp-1));
        len = 1+n+(cn>>n);
      }
      else
      {
        code = cn - (6 * (1 << n)) + (1 << n);
        len = (6-n)+1+2*log2i(code);
      }
      break;
    case 6:
    case 7:
      tmp = 1<<(n-4);
      code = tmp+cn%tmp;
      len = 1+(n-4)+(cn>>(n-4));
      break;
    case 8:
      if (cn == 0)
      {
        code = 1;
        len = 1;
      }
      else if (cn == 1)
      {
        code = 1;
        len = 2;
      }
      else if (cn == 2)
      {
        code = 0;
        len = 2;
      }
      else fatalerror("Code number too large for VLC8.");
      break;
    case 9:
      if (cn == 0)
      {
        code = 4;
        len = 3;
      }
      else if (cn == 1)
      {
        code = 10;
        len = 4;
      }
      else if (cn == 2)
      {
        code = 11;
        len = 4;
      }
      else if (cn < 11)
      {
        code = cn+21;
        len = 5;
      }
      else
      {
        tmp = 1<<4;
        code = tmp+(cn+5)%tmp;
        len = 5+((cn+5)>>4);
      }
      break;
    case 10:
      code = cn+1;
      len = 1+2*log2i(code);
      break;
    case 11:
      len = cn < 2 ? cn + 1 : cn/2 + 3;
      code = cn < 2 ? 1 : 2 + (cn&1);
      break;
    case 12:
      len = min(4,cn+1);
      code = cn==4 ? 0 : 1;
      break;
    case 13:
      len = min(6,cn+1);
      code = cn==6 ? 0 : 1;
      break;
    default:
      fatalerror("No such VLC table, only 0-10 allowed.");
    }
    return len;
}
