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

#if !defined(_RC_H_)
#define _RC_H_

#include "types.h"

typedef struct
{
  int target_bits;                     //  Target number of bits for each frame
  int max_qp;                          //  Maximum QP
  int min_qp;                          //  Minimum QP
  int num_sb;                          //  Numbers of CTUs per frame
  int buffer_level;                    //  Buffer level for leaky bucket model
  int buffer_level_init;               //  Initial buffer level for leaky bucket model
    
  int32_t *sb_bits;                    //  Number of bits used for each SB in sliding window
  int32_t *sb_qp;                      //  QP used for each SB in sliding window
    
  int bits_step_size_sliding_window;   //  bits*stepsize for last sliding window
  int bits_step_size_current_frame;    //  bits*stepsize for current frame
  int over_prod_bits;                  //  Overproduced bits for current sliding frame * 256
  int corr_factor;                     //  Over production ratio for previous frame * 256

  /* Parameters that are only used for statistical purposes*/
  int tot_num_bits;                    //  Total number of bits since startup
  int min_num_bits;                    //  Minimum number of frame-bits since startup
  int max_num_bits;                    //  Maximum number of frame-bits since startup
  int frame_count;                     //  Total number of frames since startup
  int avg_qp;                          //  Average QP;
     
} rate_control_t;

void init_rate_control_per_sequence(rate_control_t *rc, int target_bits, int num_sb);
void delete_rate_control_per_sequence(rate_control_t *rc);
void init_rate_control_per_frame(rate_control_t *rc, int minQP, int maxQP);
void update_rate_control_per_frame(rate_control_t *rc, int num_bits_frame);
int update_rate_control_sb(rate_control_t *rc, int sb_idx, int current_bits, int current_qp);
#endif //_RC_H_
