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

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "global.h"
#include "putbits.h"

static unsigned int mask[33] = {
    0x00000000,0x00000001,0x00000003,0x00000007,
    0x0000000f,0x0000001f,0x0000003f,0x0000007f,
    0x000000ff,0x000001ff,0x000003ff,0x000007ff,
    0x00000fff,0x00001fff,0x00003fff,0x00007fff,
    0x0000ffff,0x0001ffff,0x0003ffff,0x0007ffff,
    0x000fffff,0x001fffff,0x003fffff,0x007fffff,
    0x00ffffff,0x01ffffff,0x03ffffff,0x07ffffff,
    0x0fffffff,0x1fffffff,0x3fffffff,0x7fffffff,
    0xffffffff};

void flush_bytebuf(stream_t *str, FILE *outfile)
{
  if (outfile)
  {
    if (fwrite(str->bitstream,sizeof(unsigned char),str->bytepos,outfile) != str->bytepos)
    {
      fatalerror("Problem writing bitstream to file.");
    }
  }
  str->bytepos = 0;
}


void flush_all_bits(stream_t *str, FILE *outfile)
{
  int i;
  int bytes = 4 - str->bitrest/8;

  printf("final flush: bytes=%4d\n",bytes);
  if ((str->bytepos+bytes) > str->bytesize)
  {
    flush_bytebuf(str,outfile);
  }
  for (i = 0; i < bytes; i++)
  {
    str->bitstream[str->bytepos++] = (str->bitbuf >> (24-i*8)) & 0xff;
  }

  if (outfile)
  {
    if (fwrite(str->bitstream,sizeof(unsigned char),str->bytepos,outfile) != str->bytepos)
    {
      fatalerror("Problem writing bitstream to file.");
    }
  }
  str->bytepos = 0;
}    
                    

void flush_bitbuf(stream_t *str)
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

void putbits(unsigned int n, unsigned int val, stream_t *str)
{
  unsigned int rest;

  if (n <= str->bitrest)
  {
    str->bitbuf |= ((val & mask[n]) << (str->bitrest-n));
    str->bitrest -= n;
  }
  else
  {
    rest = n-str->bitrest;
    str->bitbuf |= (val >> rest) & mask[n-rest];
    flush_bitbuf(str);
    str->bitbuf |= (val & mask[rest]) << (32-rest);
    str->bitrest -= rest;
  }
}

int get_bit_pos(stream_t *str){
  int bitpos = 8*str->bytepos + (32 - str->bitrest);
  return bitpos; 
}

void write_stream_pos(stream_t *stream, stream_pos_t *stream_pos){
  stream->bitrest = stream_pos->bitrest;
  stream->bytepos = stream_pos->bytepos;
  stream->bitbuf = stream_pos->bitbuf;
}

void read_stream_pos(stream_pos_t *stream_pos, stream_t *stream){
  stream_pos->bitrest = stream->bitrest;
  stream_pos->bytepos = stream->bytepos;
  stream_pos->bitbuf = stream->bitbuf;
}

void copy_stream(stream_t *str1, stream_t *str2){
  str1->bitrest = str2->bitrest;
  str1->bytepos = str2->bytepos;
  str1->bitbuf = str2->bitbuf;
  memcpy(&(str1->bitstream[0]),&(str2->bitstream[0]),str2->bytepos*sizeof(uint8_t));
}