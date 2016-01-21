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
#include "getbits.h"
#include "getvlc.h"

unsigned int get_vlc(int n,stream_t *str)
{
  if (n < 0)
    return getbits(str, -n);

  unsigned int val = 0;
  unsigned int e = 5;
  int diff = 0;

  switch (n) {
  case 6:
  case 7:
    if (showbits(str, 2) == 2) {
      flushbits(str, 2);
      return 0;
    }
    if (n == 6) {
      diff = 1;
      n = 2;
    } else {
      if (showbits(str, 3) == 6) {
        flushbits(str, 3);
        return 1;
      }
      if (showbits(str, 3) == 7) {
        flushbits(str, 3);
        return 2 + getbits1(str);
      }
      diff = 4;
      n = 3;
    }
    // Intentional fallthough
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
    while (!getbits1(str)) val++;
    if (val <= e)
      val = (val << n) + getbits(str, n);
    else
      val = (((e - 1) + (1 << (val - e))) << n) + getbits(str, n + val - e);
    break;
  case 8:
    while (!getbits1(str) && ++val < 4);
    val = (val*2 + getbits1(str)) ^ (val > 2 ? 14 : 0);
    break;
  case 10:
    while (!getbits1(str)) val++;
    if (val)
      val = (1 << val) - 1 + getbits(str, val);
    break;
  case 11:
  case 12:
  case 13:
  case 14:
  case 15:
  case 16:
  case 17:
  case 18:
      while (!getbits1(str) && ++val < n - 10);
    break;
  default:
    printf("Illegal VLC table number. 0-18 allowed only.");
  }
  return val - diff;
}
