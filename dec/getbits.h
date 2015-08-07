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

#if !defined(_GETBITS_H_)
#define _GETBITS_H_

#include <stdio.h>
#include "entdec.h"

typedef struct
{
  od_ec_dec ec;
  FILE *infile;
  unsigned char *buf;
} stream_t;

int initbits_dec(FILE *infile, stream_t *str);

static inline unsigned bitreverse(unsigned val)
{
  val = ((val >> 16) & 0x0000FFFFU) | ((val <<16) & 0xFFFF0000U);
  val = ((val >> 8) & 0x00FF00FFU) | ((val << 8) & 0xFF00FF00U);
  val = ((val >> 4) & 0x0F0F0F0FU) | ((val << 4) & 0xF0F0F0F0U);
  val = ((val >> 2) & 0x33333333U) | ((val << 2) & 0xCCCCCCCCU);
  return ((val >> 1) & 0x55555555U) | ((val << 1) & 0xAAAAAAAAUL);
}

unsigned int showbits(stream_t *str, int n);

static inline unsigned int getbits1(stream_t *str)
{
  return od_ec_dec_bits(&str->ec, 1);
}

void flushbits(stream_t *str, int n);

static inline unsigned int getbits(stream_t *str, int n)
{
  return n > 0 ? bitreverse(od_ec_dec_bits(&str->ec, n) << (32 - n)) : 0;
}

#endif
