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

#if !defined(_COMMON_FRAME_H_)
#define _COMMON_FRAME_H_

#if CDEF
#include "simd.h"
#endif

void create_yuv_frame_lbd(yuv_frame_t  *frame, int width, int height, int sub, int pad_hor, int pad_ver, int bitdepth, int input_bitdepth);
void create_yuv_frame_hbd(yuv_frame_t  *frame, int width, int height, int sub, int pad_hor, int pad_ver, int bitdepth, int input_bitdepth);
void close_yuv_frame_lbd(yuv_frame_t  *frame);
void close_yuv_frame_hbd(yuv_frame_t  *frame);
void read_yuv_frame_lbd(yuv_frame_t  *frame, FILE *infile);
void read_yuv_frame_hbd(yuv_frame_t  *frame, FILE *infile);
void write_yuv_frame_lbd(yuv_frame_t  *frame, FILE *outfile);
void write_yuv_frame_hbd(yuv_frame_t  *frame, FILE *outfile);
void pad_yuv_frame_lbd(yuv_frame_t* f);
void pad_yuv_frame_hbd(yuv_frame_t* f);
void deblock_frame_y_lbd(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp, int bitdepth);
void deblock_frame_y_hbd(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp, int bitdepth);
void deblock_frame_uv_lbd(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp, int bitdepth);
void deblock_frame_uv_hbd(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp, int bitdepth);
void create_reference_frame_lbd(yuv_frame_t  *ref,yuv_frame_t  *rec);
void create_reference_frame_hbd(yuv_frame_t  *ref,yuv_frame_t  *rec);
void clpf_frame_lbd(const yuv_frame_t *frame, const yuv_frame_t *org, const deblock_data_t *deblock_data, void *stream,int enable_sb_flag, unsigned int strength, unsigned int fb_size_log2, int bitdepth, plane_t plane, int qp,
                    int(*decision)(int, int, const yuv_frame_t *, const yuv_frame_t *, const deblock_data_t *, int, int, int, void *, unsigned int, unsigned int, unsigned int, unsigned int, int));
void clpf_frame_hbd(const yuv_frame_t *frame, const yuv_frame_t *org, const deblock_data_t *deblock_data, void *stream,int enable_sb_flag, unsigned int strength, unsigned int fb_size_log2, int bitdepth, plane_t plane, int qp,
                    int(*decision)(int, int, const yuv_frame_t *, const yuv_frame_t *, const deblock_data_t *, int, int, int, void *, unsigned int, unsigned int, unsigned int, unsigned int, int));
#if CDEF
void cdef_frame_lbd(const yuv_frame_t *frame, const yuv_frame_t *org, deblock_data_t *deblock_data, void *stream, int cdef_bits, int bitdepth, unsigned int plane);
void cdef_frame_hbd(const yuv_frame_t *frame, const yuv_frame_t *org, deblock_data_t *deblock_data, void *stream, int cdef_bits, int bitdepth, unsigned int plane);
int cdef_allskip(int xoff, int yoff, int width, int height, deblock_data_t *deblock_data, int fb_size_log2);
void cdef_prepare_input_lbd(int sizex, int sizey, int xpos, int ypos, boundary_type bt, int padding, uint16_t *src16, int stride16, uint8_t *src_buffer, int sstride);
void cdef_prepare_input_hbd(int sizex, int sizey, int xpos, int ypos, boundary_type bt, int padding, uint16_t *src16, int stride16, uint16_t *src_buffer, int sstride);

SIMD_INLINE int adjust_strength(int strength, int32_t var) {
  const int i = var >> 6 ? min(log2i(var >> 6), 12) : 0;
  /* We use the variance of 8x8 blocks to adjust the strength. */
  return var ? (strength * (4 + i) + 8) >> 4 : 0;
}

SIMD_INLINE void cdef_init(int sstride, int cdef_directions[8][2 + CDEF_FULL]) {
  extern const int cdef_directions_x[8][2 + CDEF_FULL];
  extern const int cdef_directions_y[8][2 + CDEF_FULL];
  for (int x = 0; x < 8; x++)
    for (int y = 0; y < 2 + CDEF_FULL; y++)
      cdef_directions[x][y] = cdef_directions_y[x][y] * sstride + cdef_directions_x[x][y];
}
#endif

#endif
