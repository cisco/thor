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

void flush_all_bits(stream_t *str, FILE *outfile)
{
  unsigned char *buf;
  uint32_t frame_bytes;
  int i;
  buf = od_ec_enc_done(str, &frame_bytes);
  if (outfile)
  {
    uint8_t frame_bytes_buf[4];
    for (i = 0; i < 4; i++)
    {
      frame_bytes_buf[i] = (uint8_t)(frame_bytes >> (24 - i*8));
    }
    if (fwrite(frame_bytes_buf, sizeof(frame_bytes_buf), 1, outfile) != 1
     || fwrite(buf, sizeof(*buf), frame_bytes, outfile) != frame_bytes)
    {
      fatalerror("Problem writing bitstream to file.");
    }
  }
  od_ec_enc_reset(str);
}
