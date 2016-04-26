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

#if !defined(_COMMON_BLOCK_H_)
#define _COMMON_BLOCK_H_

#include "types.h"
#include "simd.h"

int get_left_available(int ypos, int xpos, int bwidth, int bheight, int fwidth, int fheight, int sb_size);
int get_up_available(int ypos, int xpos, int bwidth, int bheight, int fwidth, int fheight, int sb_size);
int get_upright_available(int ypos, int xpos, int bwidth, int bheight, int fwidth, int fheight, int sb_size);
int get_downleft_available(int ypos, int xpos, int bwidth, int bheight, int fwidth, int fheight, int sb_size);

void dequantize (int16_t *coeff, int16_t *rcoeff, int qp, int size, qmtx_t * wt_matrix);
void reconstruct_block(int16_t *block, uint8_t *pblock, uint8_t *rec, int size, int stride);

void find_block_contexts(int ypos, int xpos, int height, int width, int size, deblock_data_t *deblock_data, block_context_t *block_context, int enable);

void clpf_block(const uint8_t *src, uint8_t *dst, int stride, int x0, int y0, int sizex, int sizey, int width, int height, unsigned int strength);

int clpf_sample(int X, int A, int B, int C, int D, int E, int F, int b);

#endif
