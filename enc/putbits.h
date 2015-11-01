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



#if !defined(_PUTBITS_H_)
#define _PUTBITS_H_

#include <stdio.h>
#include <stdint.h>
#include "entenc.h"

typedef struct od_ec_enc stream_t;

typedef struct od_ec_enc stream_pos_t;

void flush_all_bits(stream_t *str, FILE *outfile);

static inline uint32_t bitreverse(uint32_t val)
{
  val = ((val >> 16) & 0x0000FFFFU) | ((val <<16) & 0xFFFF0000U);
  val = ((val >> 8) & 0x00FF00FFU) | ((val << 8) & 0xFF00FF00U);
  val = ((val >> 4) & 0x0F0F0F0FU) | ((val << 4) & 0xF0F0F0F0U);
  val = ((val >> 2) & 0x33333333U) | ((val << 2) & 0xCCCCCCCCU);
  return ((val >> 1) & 0x55555555U) | ((val << 1) & 0xAAAAAAAAUL);
}

static inline void putbits(unsigned int n,unsigned int val,stream_t *str)
{
  OD_ASSERT(n > 0);
  od_ec_enc_bits(str, bitreverse(val << (32 - n)), n);
}

static inline int get_bit_pos(stream_t *str)
{
  return od_ec_enc_tell(str);
}

static inline void write_stream_pos(stream_t *stream, stream_pos_t *stream_pos)
{
  od_ec_enc_rollback(stream, stream_pos);
}

static inline void read_stream_pos(stream_pos_t *stream_pos, stream_t *stream)
{
  od_ec_enc_checkpoint(stream_pos, stream);
}

#endif
