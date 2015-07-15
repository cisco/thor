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
#include "global.h"
#include "getbits.h"


/* to mask the n least significant bits of an integer */
static const unsigned int msk[33] =
{
  0x00000000,0x00000001,0x00000003,0x00000007,
  0x0000000f,0x0000001f,0x0000003f,0x0000007f,
  0x000000ff,0x000001ff,0x000003ff,0x000007ff,
  0x00000fff,0x00001fff,0x00003fff,0x00007fff,
  0x0000ffff,0x0001ffff,0x0003ffff,0x0007ffff,
  0x000fffff,0x001fffff,0x003fffff,0x007fffff,
  0x00ffffff,0x01ffffff,0x03ffffff,0x07ffffff,
  0x0fffffff,0x1fffffff,0x3fffffff,0x7fffffff,
  0xffffffff
};

int initbits_dec(FILE *infile, stream_t *str)
{
  fpos_t fpos[1];
  long pos1,pos2;

  str->incnt = 0;
  str->rdptr = str->rdbfr + 2048;
  str->bitcnt = 0;
  str->infile = infile;

  fgetpos(str->infile,fpos);
  pos1 = ftell(str->infile);
  fseek(str->infile,0,SEEK_END);
  pos2 = ftell(str->infile);
  fsetpos(str->infile,fpos);
  str->length = pos2 - pos1;
  return 0;
}

int fillbfr(stream_t *str)
{
    //int l;

  while (str->incnt <= 24 && (str->rdptr < str->rdbfr + 2048))
  {
    str->inbfr = (str->inbfr << 8) | *str->rdptr++;
    str->incnt += 8;
  }

  if (str->rdptr >= str->rdbfr + 2048)
  {
    //l = (int)fread(str->rdbfr,sizeof(unsigned char),2048,str->infile);
    fread(str->rdbfr,sizeof(unsigned char),2048,str->infile);
    str->rdptr = str->rdbfr;

    while (str->incnt <= 24 && (str->rdptr < str->rdbfr + 2048))
    {
      str->inbfr = (str->inbfr << 8) | *str->rdptr++;
      str->incnt += 8;
    }
  }

  return 0;
}

unsigned int getbits(stream_t *str, int n)
{

  if (str->incnt < n)
  {
    fillbfr(str);
    if (str->incnt < n)
    {
      unsigned int l = str->inbfr;
      unsigned int k = *str->rdptr++;
      int shift = n-str->incnt;
      str->inbfr = (str->inbfr << 8) | k;
      str->incnt = str->incnt - n + 8;
      str->bitcnt += n;
      return (((l << shift) | (k >> (8-shift))) & msk[n]);
    }
  }

  str->incnt -= n;
  str->bitcnt += n;
  return ((str->inbfr >> str->incnt) & msk[n]);
}

unsigned int getbits1(stream_t *str)
{
  if (str->incnt < 1)
  {
    fillbfr(str);
  }
  str->incnt--;
  str->bitcnt++;
  return ((str->inbfr >> str->incnt) & 1);
}

unsigned int showbits(stream_t *str, int n)
{
  if (str->incnt < n)
  {
    fillbfr(str);
    if (str->incnt < n)
    {
      int shift = n-str->incnt;
      return (((str->inbfr << shift) | (str->rdptr[0] >> (8-shift))) & msk[n]);
    }
  }

  return ((str->inbfr >> (str->incnt-n)) & msk[n]);
}

int flushbits(stream_t *str, int n)
{
  str->incnt -= n;
  str->bitcnt += n;
  return 0;
}
