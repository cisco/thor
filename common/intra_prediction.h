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

#if !defined(_INTRA_PREDICTION_H_)
#define _INTRA_PREDICTION_H_
#include "types.h"

void TEMPLATE(make_top_and_left)(SAMPLE* left, SAMPLE* top, SAMPLE* top_left, SAMPLE* rec_frame, int fstride, SAMPLE* rblock, int rbstride, int i, int j,
                                 int ypos, int xpos, int size, int upright_available,int downleft_available, int tb_split,int bitdepth);
void TEMPLATE(get_dc_pred)(SAMPLE* left, SAMPLE* top, int size, SAMPLE *pblock, int pstride, int bitdepth);
void TEMPLATE(get_hor_pred)(SAMPLE* left, int size, SAMPLE *pblock, int pstride);
void TEMPLATE(get_ver_pred)(SAMPLE* top, int size,SAMPLE *pblock, int pstride);
void TEMPLATE(get_planar_pred)(SAMPLE* left, SAMPLE* top, SAMPLE top_left,int size,SAMPLE *pblock, int pstride, int bitdepth);
void TEMPLATE(get_upleft_pred)(SAMPLE* left, SAMPLE* top, SAMPLE top_left, int size,SAMPLE *pblock, int pstride);
void TEMPLATE(get_upright_pred)(SAMPLE *top, int size, SAMPLE *pblock, int pstride);
void TEMPLATE(get_upupright_pred)(SAMPLE *top,int size,SAMPLE *pblock, int pstride);
void TEMPLATE(get_upupleft_pred)(SAMPLE *left,SAMPLE * top, SAMPLE top_left, int size,SAMPLE *pblock, int pstride);
void TEMPLATE(get_upleftleft_pred)(SAMPLE* left, SAMPLE* top, SAMPLE top_left, int size,SAMPLE *pblock, int pstride);
void TEMPLATE(get_downleftleft_pred)(SAMPLE *left,int size,SAMPLE *pblock, int pstride);

void TEMPLATE(get_intra_prediction)(SAMPLE* left, SAMPLE* top, SAMPLE top_left, int ypos,int xpos,
                                    int size, SAMPLE *pblock, int pstride, intra_mode_t intra_mode, int bitdepth);
#endif
