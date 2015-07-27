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

typedef struct
{
  uint32_t bytesize;     //Buffer size - typically maximum compressed frame size
  uint32_t bytepos;      //Byte position in bitstream
  uint8_t *bitstream;   //Compressed bit stream
  uint32_t bitbuf;       //Recent bits not written the bitstream yet
  uint32_t bitrest;      //Empty bits in bitbuf
} stream_t;

typedef struct
{
  uint32_t bytepos;      //Byte position in bitstream
  uint32_t bitbuf;       //Recent bits not written the bitstream yet
  uint32_t bitrest;      //Empty bits in bitbuf
} stream_pos_t;

void flush_all_bits(stream_t *str, FILE *outfile);
void putbits(unsigned int n,unsigned int val,stream_t *str);
void flush_bitbuf(stream_t *str);
int get_bit_pos(stream_t *str);
unsigned int leading_zeros(unsigned int code);

void write_stream_pos(stream_t *stream, stream_pos_t *stream_pos);
void read_stream_pos(stream_pos_t *stream_pos, stream_t *stream);
void copy_stream(stream_t *str1, stream_t *str2);

#endif
