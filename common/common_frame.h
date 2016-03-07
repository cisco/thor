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

void deblock_frame_y(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp);
void deblock_frame_uv(yuv_frame_t  *rec, deblock_data_t *deblock_data, int width, int height, uint8_t qp);
void create_yuv_frame(yuv_frame_t  *frame, int width, int height, int pad_ver_y, int pad_hor_y, int pad_ver_uv, int pad_hor_uv);
void close_yuv_frame(yuv_frame_t  *frame);
void read_yuv_frame(yuv_frame_t  *frame, int width, int height, FILE *infile);
void write_yuv_frame(yuv_frame_t  *frame, int width, int height, FILE *outfile);
void pad_yuv_frame(yuv_frame_t* f);
void create_reference_frame(yuv_frame_t  *ref,yuv_frame_t  *rec);
void clpf_frame(yuv_frame_t *dst, yuv_frame_t *rec, yuv_frame_t *org, const deblock_data_t *deblock_data, void *stream,int enable_sb_flag, unsigned int strength,
                int(*decision)(int, int, yuv_frame_t *, yuv_frame_t *, const deblock_data_t *, int, int, int, void *, unsigned int));
#endif
