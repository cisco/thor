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

int get_vlc0_limit(int maxbit,stream_t *str){
  int code;
  int tmp = 0;
  int nbit = 0;
  while (tmp==0 && nbit < maxbit){
    tmp = getbits1(str);
    nbit++;
  }
  code = tmp==0 ? maxbit : nbit - 1;
  return code;
}

int get_vlc(int n,stream_t *str)
{
  int cw,bit,zeroes=0,done=0,tmp;
  unsigned int val = 0;
  int first;
  unsigned int lead = 0;

  if (n < 6)
  {
    while (!done && zeroes < 6)
    {
      bit = getbits1(str);
      if (bit)
      {
        cw = getbits(str,n);
        done = 1;
      }
      else zeroes++;
    }
    if (done) val = (zeroes<<n)+cw;
    else
    {
      lead = n;
      while (!done)
      {
        first = showbits(str,1);
        if (!first)
        {
          lead++;
          flushbits(str,1);
        }
        else
        {
          tmp = getbits(str,lead+1);
          val = 6 * (1 << n) + tmp - (1 << n);
          done = 1;
        }
      }
    }
  }
  else if (n < 8)
  {
    while (!done)
    {
      bit = getbits1(str);
      if (bit)
      {
        cw = getbits(str,(n-4));
        done = 1;
      }
      else zeroes++;
    }
    val = (zeroes<<(n-4))+cw;
  }
  else if (n == 8)
  {
    if (getbits1(str))
    {
      val = 0;
    }
    else if (getbits1(str))
    {
      val = 1;
    }
    else
    {
      val = 2;
    }
  }
  else if (n == 9)
  {
    if (getbits1(str))
    {
      if (getbits1(str))
      {
        val = getbits(str,3)+3;
      }
      else if (getbits1(str))
      {
        val = getbits1(str)+1;
      }
      else
      {
        val = 0;
      }
    }
    else
    {
      while (!done)
      {
        bit = getbits1(str);
        if (bit)
        {
          cw = getbits(str,4);
          done = 1;
        }
        else zeroes++;
      }
      val = (zeroes<<4)+cw+11;
    }
  }
  else if (n == 10)
  {
    while (!done)
    {
      first = showbits(str,1);
      if (!first)
      {
        lead++;
        flushbits(str,1);
      }
      else
      {
        val = getbits(str,lead+1)-1;
        done = 1;
      }
    }
  }
  else if (n == 11){
    int tmp;
    tmp = getbits(str,1);
    if (tmp){
      val = 0;
    }
    else{
      tmp = getbits(str,1);
      if (tmp){
        val = 1;
      }
      else{
        tmp = 0;
        val = 0;
        while (tmp==0){
          tmp = getbits(str,1);
          val = val+2;
        }
        val += getbits(str,1);
      }
    }
  }

  else if (n == 12){
    tmp = 0;
    val = 0;
    while (tmp==0 && val<4){
      tmp = getbits(str,1);
      val += (!tmp);
    }
  }
  else if (n == 13){
    tmp = 0;
    val = 0;
    while (tmp==0 && val<6){
      tmp = getbits(str,1);
      val += (!tmp);
    }
  }


  //else rferror("Illegal VLC table number. 0-10 allowed only.");
  else printf("Illegal VLC table number. 0-10 allowed only.");
  return val;
}
