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

void get_dc_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,uint8_t *pblock);
void get_hor_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,uint8_t *pblock);
void get_ver_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,uint8_t *pblock);
void get_planar_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,uint8_t *pblock);
void get_upleft_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,uint8_t *pblock);
void get_upright_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,int width,uint8_t *pblock,int upright_available);
void get_upupright_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,int width,uint8_t *pblock,int upright_available);
void get_upupleft_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,uint8_t *pblock);
void get_upleftleft_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,uint8_t *pblock);
void get_downleftleft_pred(uint8_t *rec,int yposY,int xposY,int stride,int size,uint8_t *pblock);
void get_intra_prediction(uint8_t *rec,int yposY,int xposY,int stride,int size,int width,uint8_t *pblock,intra_mode_t intra_mode,int upright_available);
#endif
