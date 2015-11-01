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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "global.h"
#include "getbits.h"

int initbits_dec(FILE *infile, stream_t *str)
{
  uint8_t frame_bytes_buf[4];
  uint32_t length;
  int ret;
  str->infile = infile;
  length = 0;
  ret = fread(frame_bytes_buf, sizeof(frame_bytes_buf), 1, infile) != 1;
  if (!ret)
  {
    unsigned char *buf;
    length = frame_bytes_buf[0] << 24 | frame_bytes_buf[1] << 16
     | frame_bytes_buf[2] << 8 | frame_bytes_buf[3];
    buf = realloc(str->buf, sizeof(*buf)*length);
    ret = buf == NULL;
    if (!ret)
    {
      ret = fread(buf, sizeof(*buf), length, str->infile) != length;
      if (!ret)
      {
        od_ec_dec_init(&str->ec, buf, length);
      }
    }
  }
  return ret;
}

/*This is meant to be a large, positive constant that can still be efficiently
   loaded as an immediate (on platforms like ARM, for example).
  Even relatively modest values like 100 would work fine.*/
#define OD_EC_LOTS_OF_BITS (0x4000)

unsigned int showbits(stream_t *str, int n)
{
  od_ec_window window;
  int available;
  uint32_t ret;
  OD_ASSERT(n <= 25);
  window = str->ec.end_window;
  available = str->ec.nend_bits;
  if ((unsigned)available < n) {
    const unsigned char *buf;
    const unsigned char *eptr;
    buf = str->ec.buf;
    eptr = str->ec.eptr;
    OD_ASSERT(available <= OD_EC_WINDOW_SIZE - 8);
    do {
      if (eptr <= buf) {
        str->ec.tell_offs += OD_EC_LOTS_OF_BITS - available;
        available = OD_EC_LOTS_OF_BITS;
        break;
      }
      window |= (od_ec_window)*--eptr << available;
      available += 8;
    }
    while (available <= OD_EC_WINDOW_SIZE - 8);
    str->ec.eptr = eptr;
    str->ec.end_window = window;
    str->ec.nend_bits = available;
  }
  ret = (uint32_t)window & (((uint32_t)1 << n) - 1);
  OD_ASSERT(n > 0);
  return bitreverse(ret << (32 - n));
}

void flushbits(stream_t *str, int n)
{
  OD_ASSERT(str->ec.nend_bits >= n);
  str->ec.end_window >>= n;
  str->ec.nend_bits -= n;
}
