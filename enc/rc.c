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

#include "global.h"
#include <math.h>
#include <stdlib.h>
#include "rc.h"

/* Implements stepSize = 2^((QP-4)/6) */
const uint8_t qp2stepSize[52] = { 0,0,0,0,1,1,1,1,1,1,2,2,2,2,3,3,4,4,5,5,6,7,8,9,10,11,12,14,16,18,20,22,25,28,32,36,40,45,51,57,64,72,80,90,102,114,128,144,160,180,204,228 };

/* Implements QP = 6*log2(stepSize) + 4 */
const uint8_t stepSize2qp[MAX_STEP_SIZE + 1] = { 
   0, 4,10,14,16,18,20,21,22,23,24,25,26,26,27,27,28,29,29,29,30,30,31,31,32,
  32,32,33,33,33,33,34,34,34,35,35,35,35,35,36,36,36,36,37,37,37,37,37,38,38,
  38,38,38,38,39,39,39,39,39,39,39,40,40,40,40,40,40,40,41,41,41,41,41,41,41,
  41,41,42,42,42,42,42,42,42,42,42,43,43,43,43,43,43,43,43,43,43,44,44,44,44,
  44,44,44,44,44,44,44,44,45,45,45,45,45,45,45,45,45,45,45,45,45,46,46,46,46,
  46,46,46,46,46,46,46,46,46,46,46,47,47,47,47,47,47,47,47,47,47,47,47,47,47,
  47,47,47,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,49,49,49,49,
  49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,50,50,50,50,50,50,50,50,
  50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,51,51,51,51,51,51,51,51,51,51,51,51,51};

void init_rate_control_per_sequence(rate_control_t *rc, int target_bits, int num_sb){
  
  rc->num_sb = num_sb;    
  rc->sb_bits = malloc(num_sb*sizeof(int)); //  Number of bits used for each SB in previous frame
  rc->sb_qp = malloc(num_sb*sizeof(int));   //  QP used for each SB in previous frame    
  rc->target_bits = target_bits;
  rc->buffer_level = 1 * target_bits;
  rc->buffer_level_init = 1 * target_bits;
    
  int bits_per_sb = rc->target_bits/max(1,rc->num_sb);
  int step_size = min(MAX_STEP_SIZE,2048/(max(1,bits_per_sb)));
  int qp = max(INTRA_FIXED_QP,stepSize2qp[step_size]);
  rc->bits_step_size_sliding_window = rc->target_bits*qp2stepSize[qp];
  rc->over_prod_bits = 0;
  rc->corr_factor = 1<<8;
  for (int i=0;i<rc->num_sb;i++){
    rc->sb_bits[i] = bits_per_sb;
    rc->sb_qp[i] = qp;    
  }
      
  /* Initiating parameters that are used only for statistical purposes */
  rc->tot_num_bits = 0;
  rc->frame_count = 0;
  rc->max_num_bits = 0;
  rc->min_num_bits = 9999999;
}

void delete_rate_control_per_sequence(rate_control_t *rc) {
  free(rc->sb_bits);
  free(rc->sb_qp);
}

void init_rate_control_per_frame(rate_control_t *rc, int minQP, int maxQP){
  rc->bits_step_size_current_frame = 0;
  rc->max_qp = maxQP;
  rc->min_qp = minQP;
}

void update_rate_control_per_frame(rate_control_t *rc, int num_bits_frame){

  /* Update virtual buffer level */
  rc->buffer_level += (num_bits_frame - rc->target_bits);

  if (rc->buffer_level < 0){
    /* Virtual buffer can't be negative */
    rc->buffer_level = 0;           
  }

  /* Reset overprod parameter */
  rc->over_prod_bits = (rc->buffer_level - rc->buffer_level_init) << 8;

  if (num_bits_frame > 0) {
    /* Set correction factor for sliding window operation in next frame */
    rc->corr_factor = ((rc->target_bits << 8) + (num_bits_frame >> 1))/num_bits_frame;
  }

  /* Reset sliding window statistics to current frame statistics to ensure exact resync at the end of each frame */
  rc->bits_step_size_sliding_window = rc->bits_step_size_current_frame;
        
#if 0
  /* For statistical/debugging purposes */
  int qp;
  int max_qp = 0;
  int min_qp = 52;
  rc->avg_qp = 0;
  for (int idx = 0; idx<rc->num_sb; idx++) {
    rc->avg_qp += rc->sb_qp[idx];
  }
  rc->avg_qp = rc->avg_qp / rc->num_sb;

  /* Updating parameters that are used only for statistical purposes */
  rc->frame_count += 1;
  rc->tot_num_bits += num_bits_frame;

  if (rc->frame_count > 100){
    if (num_bits_frame > rc->max_num_bits) rc->max_num_bits = num_bits_frame;
    if (num_bits_frame < rc->min_num_bits) rc->min_num_bits = num_bits_frame;
  }

  for (int idx=0; idx<rc->num_sb; idx++){
    qp = rc->sb_qp[idx];
    if (qp < min_qp) min_qp = qp;
    if (qp > max_qp) max_qp = qp;
  }
#if 0
  printf("TARGET=%7d LEVEL=%7d AVGQP=%2d MINQP=%2d MAXQP=%2d AVGBIT=%7d OPB=%7d BTF=%7d MIN=%6d MAX=%6d ",
          rc->target_bits,rc->buffer_level, rc->avg_qp,min_qp,max_qp,rc->tot_num_bits/rc->frame_count,
          (rc->over_prod_bits+128)>>8, num_bits_frame,rc->min_num_bits,rc->max_num_bits);
#endif
#endif
  /* Initialize frame statistics for this frame */
  rc->bits_step_size_current_frame = 0;
}

int update_rate_control_sb(rate_control_t *rc, int sb_idx, int current_bits, int current_qp){
    
  int last_bits,target_bits_current;
  int last_qp,new_qp;
  int current_step_size,last_step_size,new_step_size;

  /* Set bit and QP variables for this SB in current and previous frame */
  last_bits = rc->sb_bits[sb_idx];
  last_qp = rc->sb_qp[sb_idx];

  /* Convert to step size */
  current_step_size = qp2stepSize[current_qp];
  last_step_size = qp2stepSize[last_qp];

  /* Update array for bits and QP values */
  rc->sb_bits[sb_idx] = current_bits;
  rc->sb_qp[sb_idx] = current_qp;

  /* Update sliding window variable */
  rc->bits_step_size_sliding_window += (current_bits*current_step_size - last_bits*last_step_size);

  /* Increment current frame variables */
  rc->bits_step_size_current_frame += current_bits*current_step_size;

  /* Update target number of bits */
  target_bits_current = rc->target_bits - ((rc->over_prod_bits+128)>>8);
  if (target_bits_current<=0) target_bits_current = 1; //Probably not necessarry when overProdBits is clipped as below

  /* Determine step size for next CTU */
  new_step_size = (rc->bits_step_size_sliding_window + (target_bits_current>>1))/target_bits_current;

  /* Find corresponding QP value */
  new_step_size = clip(new_step_size, 1, MAX_STEP_SIZE);
  new_qp = stepSize2qp[new_step_size];
    
  /* Clip QP value */
  if (new_qp > rc->max_qp) new_qp = rc->max_qp;
  if (new_qp < rc->min_qp) new_qp = rc->min_qp;

  /* Update overproduced bits before next CTU */
  rc->over_prod_bits += ((current_bits<<8) - rc->corr_factor*last_bits);

  return new_qp;
}
