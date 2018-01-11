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
#include <string.h>

#include "mainenc.h"
#include "encode_block.h"
#include "common_block.h"
#include "common_kernels.h"
#include "common_frame.h"
#include "putvlc.h"
#include "wt_matrix.h"
#include "enc_kernels.h"
#include "inter_prediction.h"
#include "write_bits.h"

extern int chroma_qp[52];
extern double squared_lambda_QP[52];

#if CDEF
int TEMPLATE(cdef_search)(yuv_frame_t *rec, yuv_frame_t *org, deblock_data_t *deblock_data, const frame_info_t *frame_info, encoder_info_t *encoder_info,
                          int strengths[8], int uv_strengths[8], int speed);

#define TOTAL_STRENGTHS (CDEF_PRI_STRENGTHS * CDEF_SEC_STRENGTHS)

static int priconv[3][CDEF_PRI_STRENGTHS] = { { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
                                              { 0, 1, 2, 3, 5, 7, 10, 13 },
                                              { 0, 1, 3, 6 } };

static int pristrengths[3] = { CDEF_PRI_STRENGTHS * CDEF_SEC_STRENGTHS, 8 * CDEF_SEC_STRENGTHS, 4 * CDEF_SEC_STRENGTHS};

/* Search for the best strength to add as an option, knowing we
   already selected nb_strengths options. */
static uint64_t search_one(int *lev, int nb_strengths,
                           uint64_t mse[][TOTAL_STRENGTHS], int sb_count,
                           int speed) {
  uint64_t tot_mse[TOTAL_STRENGTHS];
  const int total_strengths = pristrengths[speed];
  int i, j;
  uint64_t best_tot_mse = (uint64_t)1 << 63;
  int best_id = 0;
  memset(tot_mse, 0, sizeof(tot_mse));
  for (i = 0; i < sb_count; i++) {
    int gi;
    uint64_t best_mse = (uint64_t)1 << 63;
    /* Find best mse among already selected options. */
    for (gi = 0; gi < nb_strengths; gi++) {
      if (mse[i][lev[gi]] < best_mse) {
        best_mse = mse[i][lev[gi]];
      }
    }
    /* Find best mse when adding each possible new option. */
    for (j = 0; j < total_strengths; j++) {
      uint64_t best = best_mse;
      if (mse[i][j] < best) best = mse[i][j];
      tot_mse[j] += best;
    }
  }
  for (j = 0; j < total_strengths; j++) {
    if (tot_mse[j] < best_tot_mse) {
      best_tot_mse = tot_mse[j];
      best_id = j;
    }
  }
  lev[nb_strengths] = best_id;
  return best_tot_mse;
}

/* Search for the best luma+chroma strength to add as an option, knowing we
   already selected nb_strengths options. */
static uint64_t search_one_dual(int *lev0, int *lev1, int nb_strengths,
                                uint64_t (**mse)[TOTAL_STRENGTHS], int sb_count,
                                int speed) {
  uint64_t tot_mse[TOTAL_STRENGTHS][TOTAL_STRENGTHS];
  int i, j;
  uint64_t best_tot_mse = (uint64_t)1 << 63;
  int best_id0 = 0;
  int best_id1 = 0;
  const int total_strengths = pristrengths[speed];
  memset(tot_mse, 0, sizeof(tot_mse));
  for (i = 0; i < sb_count; i++) {
    int gi;
    uint64_t best_mse = (uint64_t)1 << 63;
    /* Find best mse among already selected options. */
    for (gi = 0; gi < nb_strengths; gi++) {
      uint64_t curr = mse[0][i][lev0[gi]];
      curr += mse[1][i][lev1[gi]];
      if (curr < best_mse) {
        best_mse = curr;
      }
    }
    /* Find best mse when adding each possible new option. */
    for (j = 0; j < total_strengths; j++) {
      int k;
      for (k = 0; k < total_strengths; k++) {
        uint64_t best = best_mse;
        uint64_t curr = mse[0][i][j];
        curr += mse[1][i][k];
        if (curr < best) best = curr;
        tot_mse[j][k] += best;
      }
    }
  }
  for (j = 0; j < total_strengths; j++) {
    int k;
    for (k = 0; k < total_strengths; k++) {
      if (tot_mse[j][k] < best_tot_mse) {
        best_tot_mse = tot_mse[j][k];
        best_id0 = j;
        best_id1 = k;
      }
    }
  }
  lev0[nb_strengths] = best_id0;
  lev1[nb_strengths] = best_id1;
  return best_tot_mse;
}

/* Search for the set of strengths that minimizes mse. */
static uint64_t joint_strength_search(int *best_lev, int nb_strengths,
                                      uint64_t mse[][TOTAL_STRENGTHS],
                                      int sb_count, int speed) {
  uint64_t best_tot_mse;
  int i;
  best_tot_mse = (uint64_t)1 << 63;
  /* Greedy search: add one strength options at a time. */
  for (i = 0; i < nb_strengths; i++) {
    best_tot_mse = search_one(best_lev, i, mse, sb_count, speed);
  }
  /* Trying to refine the greedy search by reconsidering each
     already-selected option. */
  if (!speed) {
    for (i = 0; i < 4 * nb_strengths; i++) {
      int j;
      for (j = 0; j < nb_strengths - 1; j++) best_lev[j] = best_lev[j + 1];
      best_tot_mse =
          search_one(best_lev, nb_strengths - 1, mse, sb_count, speed);
    }
  }
  return best_tot_mse;
}

/* Search for the set of luma+chroma strengths that minimizes mse. */
static uint64_t joint_strength_search_dual(int *best_lev0, int *best_lev1,
                                           int nb_strengths,
                                           uint64_t (**mse)[TOTAL_STRENGTHS],
                                           int sb_count, int speed) {
  uint64_t best_tot_mse;
  int i;
  best_tot_mse = (uint64_t)1 << 63;
  /* Greedy search: add one strength options at a time. */
  for (i = 0; i < nb_strengths; i++) {
    best_tot_mse =
        search_one_dual(best_lev0, best_lev1, i, mse, sb_count, speed);
  }
  /* Trying to refine the greedy search by reconsidering each
     already-selected option. */
  for (i = 0; i < 4 * nb_strengths; i++) {
    int j;
    for (j = 0; j < nb_strengths - 1; j++) {
      best_lev0[j] = best_lev0[j + 1];
      best_lev1[j] = best_lev1[j + 1];
    }
    best_tot_mse = search_one_dual(best_lev0, best_lev1, nb_strengths - 1, mse,
                                   sb_count, speed);
  }
  return best_tot_mse;
}

static uint64_t dist_8x8(SAMPLE *dst, int dstride, SAMPLE *src,
                         int sstride, int coeff_shift) {
  uint64_t svar = 0;
  uint64_t dvar = 0;
  uint64_t sum_s = 0;
  uint64_t sum_d = 0;
  uint64_t sum_s2 = 0;
  uint64_t sum_d2 = 0;
  uint64_t sum_sd = 0;
  int i, j;
  for (i = 0; i < 8; i++) {
    for (j = 0; j < 8; j++) {
      sum_s += src[i * sstride + j];
      sum_d += dst[i * dstride + j];
      sum_s2 += src[i * sstride + j] * src[i * sstride + j];
      sum_d2 += dst[i * dstride + j] * dst[i * dstride + j];
      sum_sd += src[i * sstride + j] * dst[i * dstride + j];
    }
  }
  /* Compute the variance -- the calculation cannot go negative. */
  svar = sum_s2 - ((sum_s * sum_s + 32) >> 6);
  dvar = sum_d2 - ((sum_d * sum_d + 32) >> 6);
  return (uint64_t)floor(
      .5 +
      (sum_d2 + sum_s2 - 2 * sum_sd) * .5 *
          (svar + dvar + (400 << 2 * coeff_shift)) /
          (sqrt((20000 << 4 * coeff_shift) + svar * (double)dvar)));
}


static int cdef_cmp(const void *a, const void *b) {
  return *(uint32_t*)a < *(uint32_t*)b ? -1 : *(uint32_t*)a > *(uint32_t*)b;
}

int TEMPLATE(cdef_search)(yuv_frame_t *rec, yuv_frame_t *org, deblock_data_t *deblock_data, const frame_info_t *frame_info, encoder_info_t *encoder_info,
                          int strengths[8], int uv_strengths[8], int speed) {
  int width = rec->width;
  int height = rec->height;
  const int fb_size_log2 = CDEF_BLOCKSIZE_LOG2;
  const int num_fb_hor = (width + (1 << fb_size_log2) - 1) >> fb_size_log2;
  const int num_fb_ver = (height + (1 << fb_size_log2) - 1) >> fb_size_log2;
  uint64_t best_tot_mse = (uint64_t)1 << 63;
  uint64_t tot_mse;
  uint64_t(*mse[2])[TOTAL_STRENGTHS];
  int pri_damping = encoder_info->cdef_damping;
  int sec_damping = pri_damping;
  const int total_strengths = pristrengths[speed];
  int sb_count = 0;
  const int bs = 8;
  const int bslog = log2i(bs);
  int padding = 2 + CDEF_FULL;
  int hpadding = 16 - padding;
  int stride16 = (64 + 2 * padding + 15) & ~15;
  int offset16 = padding * stride16 + padding + hpadding;
  uint16_t *src16 = thor_alloc((64 + 2 * padding) * stride16 * sizeof(uint16_t) + hpadding, 32);
  SAMPLE *dst = thor_alloc(bs * bs * sizeof(SAMPLE), 32);
  int cdef_directions[8][2 + CDEF_FULL];
  int cdef_directions_copy[8][2 + CDEF_FULL];
  int *ci_index = thor_alloc(num_fb_hor * num_fb_ver * sizeof(*ci_index), 16);
  int *selected_strength = thor_alloc(num_fb_hor * num_fb_ver * sizeof(*ci_index), 16);
  stream_t *stream = encoder_info->stream;
  const int bitdepth = encoder_info->params->bitdepth;

  mse[0] = thor_alloc(sizeof(**mse) * num_fb_hor * num_fb_ver, 32);
  mse[1] = thor_alloc(sizeof(**mse) * num_fb_hor * num_fb_ver, 32);

  cdef_init(stride16, cdef_directions_copy);

  int ci = -1;
  for (int k = 0; k < num_fb_ver; k++) {
    for (int l = 0; l < num_fb_hor; l++) {

      int h, w;
      const int xoff = l << fb_size_log2;
      const int yoff = k << fb_size_log2;
      int allskip = cdef_allskip(xoff, yoff, width, height, deblock_data, fb_size_log2);
      ci++;

      if (allskip)
        continue;

      // Calculate the actual filter block size near frame edges
      h = min(height, (k + 1) << fb_size_log2) & ((1 << fb_size_log2) - 1);
      w = min(width, (l + 1) << fb_size_log2) & ((1 << fb_size_log2) - 1);
      h += !h << fb_size_log2;
      w += !w << fb_size_log2;

      int index = (yoff/MIN_PB_SIZE)*(width/MIN_PB_SIZE) + (xoff/MIN_PB_SIZE);

      int coeff_shift = bitdepth - 8;

      // TODO: HBD
      for (int plane = 0; plane < 3; plane++) {
        const int sub = plane != 0 && rec->sub;
        const int sstride = plane != 0 ? rec->stride_c : rec->stride_y;
        SAMPLE *src_buffer = plane != 0 ? (plane == 1 ? rec->u : rec->v) : rec->y;
        cdef_init(sstride, cdef_directions);  // TODO: calc once

        // Prepare input
	int sizex = min(width - xoff, 64) >> sub;
	int sizey =  min(height - yoff, 64) >> sub;
	int xpos = xoff >> sub;
	int ypos = yoff >> sub;
	boundary_type bt =
	  (TILE_LEFT_BOUNDARY & -!xpos) |
	  (TILE_ABOVE_BOUNDARY & -!ypos) |
	  (TILE_RIGHT_BOUNDARY & -(xpos == (width >> sub) - sizex)) |
	  (TILE_BOTTOM_BOUNDARY & -(ypos == (height >> sub) - sizey));

	TEMPLATE(cdef_prepare_input)(sizex, sizey, xpos, ypos, bt, padding, src16 + offset16, stride16, src_buffer, sstride);

        for (int gi = 0; gi < total_strengths; gi++) {
          int level;
          int pri_strength, sec_strength;
          level = gi / CDEF_SEC_STRENGTHS;
          level = priconv[speed][level];
          pri_strength = level;
          sec_strength = (gi % CDEF_SEC_STRENGTHS);

          if (plane < 2)
            mse[plane][sb_count][gi] = 0;

          for (int m = 0; m < ((h + bs - 1) >> (bslog + sub)); m++) {
            for (int n = 0; n < ((w + bs - 1) >> (bslog + sub)); n++) {
              int sizex, sizey;
              xpos = (xoff >> sub) + n * bs;
              ypos = (yoff >> sub) + m * bs;
              sizex = min((width >> sub) - xpos, bs);
              sizey = min((height >> sub) - ypos, bs);
              index = ((yoff + m * 8) / MIN_PB_SIZE) * (width/MIN_PB_SIZE) + ((xoff + n * 8) / MIN_PB_SIZE);

              if (plane == 0 && gi == 0)
                encoder_info->cdef[ci].dir[m * bs + n] = (use_simd ? TEMPLATE(cdef_find_dir_simd) : TEMPLATE(cdef_find_dir))(src_buffer + ypos * sstride + xpos, sstride, &encoder_info->cdef[ci].var[m * bs + n], coeff_shift);

              if (deblock_data[index].mode != MODE_SKIP) {

                int adj_str = plane ? pri_strength : adjust_strength(pri_strength, encoder_info->cdef[ci].var[m * bs + n]);
                int adj_pri_damping = adj_str ? max(log2i(adj_str), pri_damping - !!plane) : pri_damping - !!plane;
                int adj_sec_damping = sec_damping - !!plane;

                // Apply the filter.
#ifdef HBD
                (use_simd ? cdef_filter_block_simd : cdef_filter_block)(NULL, dst, sizex, src16 + offset16 + n * bs + m * bs * stride16, stride16,
                             adj_str << coeff_shift, sec_strength << coeff_shift,
                             pri_strength ? encoder_info->cdef[ci].dir[m * bs + n] : 0, adj_pri_damping + coeff_shift, adj_sec_damping + coeff_shift, sizex,
                             cdef_directions_copy, coeff_shift);
#else
                (use_simd ? cdef_filter_block_simd : cdef_filter_block)(dst, NULL, sizex, src16 + offset16 + n * bs + m * bs * stride16, stride16,
                             adj_str << coeff_shift, sec_strength << coeff_shift,
                             pri_strength ? encoder_info->cdef[ci].dir[m * bs + n] : 0, adj_pri_damping + coeff_shift, adj_sec_damping + coeff_shift, sizex,
                             cdef_directions_copy, coeff_shift);
#endif

                // Calc mse.  TODO: Improve metric
                SAMPLE *org_buffer = (plane != 0 ? (plane == 1 ? org->u : org->v) : org->y) + ypos * sstride + xpos;
                if (plane || sizex != 8 || sizey != 8)
                  for (int i = 0; i < sizey; i++)
                    for (int j = 0; j < sizex; j++)
                       mse[!!plane][sb_count][gi] += (dst[i * sizex + j] - org_buffer[i * sstride + j]) *
                        (dst[i * sizex + j] - org_buffer[i * sstride + j]);
                else
                  mse[!!plane][sb_count][gi] += dist_8x8(dst, sizex, org_buffer, sstride, coeff_shift);
              }
            }
          }
        }
      }
      ci_index[sb_count++] = ci;
    }
  }

  int nb_strengths;
  int nb_strength_bits;

  nb_strength_bits = 0;
  /* Search for different number of signalling bits.  Currently fixed. */
  for (int i = encoder_info->cdef_bits; i <= encoder_info->cdef_bits; i++) {
    int j;
    int best_lev0[CDEF_MAX_STRENGTHS];
    int best_lev1[CDEF_MAX_STRENGTHS] = { 0 };

    if (encoder_info->params->subsample != 400)
      tot_mse = joint_strength_search_dual(best_lev0, best_lev1, 1 << i,
                                           mse, sb_count, speed);
    else
      tot_mse = joint_strength_search(best_lev0, 1 << i, mse[0], sb_count,
                                      speed);
    /* Count superblock signalling cost. */
    tot_mse += (uint64_t)(sb_count * frame_info->lambda * i);
    /* Count header signalling cost. */
    tot_mse += (uint64_t)((1 << i) * frame_info->lambda * CDEF_STRENGTH_BITS);
    if (tot_mse < best_tot_mse) {
      best_tot_mse = tot_mse;
      nb_strength_bits = i;
      for (j = 0; j < 1 << nb_strength_bits; j++) {
        strengths[j] = best_lev0[j];
        uv_strengths[j] = best_lev1[j];
      }
    }
  }

  // Sort results and remove duplicates
  int gi_trans[8];
  uint32_t list[8];
  for (int i = 0; i < 1 << nb_strength_bits; i++)
    list[i] = (strengths[i] << 16) + (uv_strengths[i] << 8) + i;
  qsort(list, 1 << nb_strength_bits, sizeof(*list), cdef_cmp);
  int j = 0;
  for (int i = 0; i < 1 << nb_strength_bits; i++) {
    if (!i || (list[i] & ~255) != (list[i - 1] & ~255)) {
        strengths[j] = list[i] >> 16;
        uv_strengths[j] = (list[i] >> 8) & 255;
        gi_trans[list[i] & 255] = j++;
    }
  }

  // Reduce the number of bits per block
  nb_strength_bits = log2i(j);

  nb_strengths = 1 << nb_strength_bits;
  
  // Assign the best preset to every filter block
  for (int i = 0; i < sb_count; i++) {
    int gi;
    int best_gi;
    uint64_t best_mse = (uint64_t)1 << 63;
    best_gi = 0;
    for (gi = 0; gi < (1 << nb_strength_bits); gi++) {
      uint64_t curr = mse[0][i][strengths[gi_trans[gi]]];
      if (encoder_info->params->subsample != 400) curr += mse[1][i][uv_strengths[gi_trans[gi]]];
      if (curr < best_mse) {
        best_gi = min(nb_strengths - 1, gi_trans[gi]);
        best_mse = curr;
      }
    }
    selected_strength[i] = best_gi;
    if (nb_strength_bits) {
      put_flc(nb_strength_bits, best_gi, stream);
    }
  }

  for (int j = 0; j < nb_strengths; j++) {
    strengths[j] =
      priconv[speed][strengths[j] / CDEF_SEC_STRENGTHS] *
      CDEF_SEC_STRENGTHS +
      (strengths[j] % CDEF_SEC_STRENGTHS);
    uv_strengths[j] =
      priconv[speed][uv_strengths[j] / CDEF_SEC_STRENGTHS] *
      CDEF_SEC_STRENGTHS +
      (uv_strengths[j] % CDEF_SEC_STRENGTHS);
  }

  for (int i = 0; i < sb_count; i++) {
    for (int plane = 0; plane < 2; plane++) {
      cdef_strength *cdef = &encoder_info->cdef[ci_index[i]].plane[plane != 0];
      cdef->level = (plane ? uv_strengths[selected_strength[i]] : strengths[selected_strength[i]]) >> 2;
      cdef->sec_strength = (plane ? uv_strengths[selected_strength[i]] : strengths[selected_strength[i]]) & 3;
      cdef->pri_damping = cdef->sec_damping = encoder_info->cdef_damping;
    }
  }

  thor_free(mse[0]);
  thor_free(mse[1]);
  thor_free(src16);
  thor_free(dst);

  thor_free(ci_index);
  thor_free(selected_strength);

  return nb_strength_bits;
}
#endif

static int clpf_decision(int k, int l, const yuv_frame_t *rec, const yuv_frame_t *org, const deblock_data_t *deblock_data, int block_size, int w, int h, void *stream, unsigned int strength, unsigned int fb_size_log2, unsigned int shift, unsigned int size, int qp) {
  int sum0 = 0, sum1 = 0;
  int damping = shift + 4 + (qp >> 4);

  for (int m = 0; m < h; m++) {
    for (int n = 0; n < w; n++) {
      int xpos = (l<<fb_size_log2) + n*block_size;
      int ypos = (k<<fb_size_log2) + m*block_size;
      int index = (ypos / MIN_PB_SIZE)*(rec->width / MIN_PB_SIZE) + (xpos / MIN_PB_SIZE);
      if (deblock_data[index].mode != MODE_SKIP) {
        if (use_simd && size == 8)
          TEMPLATE(detect_clpf_simd)(rec->y, org->y, xpos, ypos, rec->width, rec->height, org->stride_y, rec->stride_y, &sum0, &sum1, strength, shift, size, damping);
        else
          TEMPLATE(detect_clpf)(rec->y, org->y, xpos, ypos, rec->width, rec->height, org->stride_y, rec->stride_y, &sum0, &sum1, strength, shift, size, damping);
      }
    }
  }
  put_flc(1, sum1 < sum0, (stream_t*)stream);
  return sum1 < sum0;
}

// Calculate the square error of all filter settings.  Result:
// res[0][0]   : unfiltered
// res[0][1-3] : strength=1,2,4, no signals
// (Only for luma:)
// res[1][0]   : (bit count, fb size = 128)
// res[1][1-3] : strength=1,2,4, fb size = 128
// res[2][0]   : (bit count, fb size = 64)
// res[2][1-3] : strength=1,2,4, fb size = 64
// res[3][0]   : (bit count, fb size = 32)
// res[3][1-3] : strength=1,2,4, fb size = 32
static int clpf_rdo(int y, int x, yuv_frame_t *rec, yuv_frame_t *org, const deblock_data_t *deblock_data, unsigned int block_size, unsigned int fb_size_log2, int w, int h, int64_t res[4][4], int bitdepth, plane_t plane, int qp) {
  int filtered = 0;
  int sum[4];
  int bslog = log2i(block_size);
  int damping = bitdepth - 4 - (plane != PLANE_Y) + (qp >> 4);

  sum[0] = sum[1] = sum[2] = sum[3] = 0;
  if (plane == PLANE_Y && fb_size_log2 > log2i(MAX_SB_SIZE) - 3) {
    fb_size_log2--;
    int w1 = min(1<<(fb_size_log2-bslog), w);
    int h1 = min(1<<(fb_size_log2-bslog), h);
    int w2 = min(w - (1<<(fb_size_log2-bslog)), w>>1);
    int h2 = min(h - (1<<(fb_size_log2-bslog)), h>>1);
    int i = log2i(MAX_SB_SIZE) - fb_size_log2;
    int64_t sum1 = res[i][1], sum2 = res[i][2], sum3 = res[i][3];
    int64_t oldfiltered = res[i][0];
    res[i][0] = 0;

    filtered = clpf_rdo(y, x, rec, org, deblock_data, block_size, fb_size_log2, w1, h1, res, bitdepth, plane, qp);
    if (1<<(fb_size_log2-bslog) < w)
      filtered |= clpf_rdo(y, x+(1<<fb_size_log2), rec, org, deblock_data, block_size, fb_size_log2, w2, h1, res, bitdepth, plane, qp);
    if (1<<(fb_size_log2-bslog) < h) {
      filtered |= clpf_rdo(y+(1<<fb_size_log2), x, rec, org, deblock_data, block_size, fb_size_log2, w1, h2, res, bitdepth, plane, qp);
      filtered |= clpf_rdo(y+(1<<fb_size_log2), x+(1<<fb_size_log2), rec, org, deblock_data, block_size, fb_size_log2, w2, h2, res, bitdepth, plane, qp);
    }

    res[i][1] = min(sum1 + res[i][0], res[i][1]);
    res[i][2] = min(sum2 + res[i][0], res[i][2]);
    res[i][3] = min(sum3 + res[i][0], res[i][3]);
    res[i][0] = oldfiltered + filtered; // Number of signal bits
    return filtered;
  }

  SAMPLE *rec_buffer = plane != PLANE_Y ? (plane == PLANE_U ? rec->u : rec->v) : rec->y;
  SAMPLE *org_buffer = plane != PLANE_Y ? (plane == PLANE_U ? org->u : org->v) : org->y;
  int rec_width = plane != PLANE_Y ? (rec->width >> rec->sub) : rec->width;
  int rec_height = plane != PLANE_Y ? (rec->height >> rec->sub) : rec->height;
  int rec_stride = plane != PLANE_Y ? rec->stride_c : rec->stride_y;
  int org_stride = plane != PLANE_Y ? org->stride_c : org->stride_y;

  for (int m = 0; m < h; m++) {
    for (int n = 0; n < w; n++) {
      int xpos = x + n*block_size;
      int ypos = y + m*block_size;
      int sub = plane != PLANE_Y && org->sub;
      int index = ((ypos << sub) / MIN_PB_SIZE)*(rec->width / MIN_PB_SIZE) + ((xpos << sub) / MIN_PB_SIZE);
      if (deblock_data[index].mode != MODE_SKIP) {
        if (use_simd && block_size == 8)
          TEMPLATE(detect_multi_clpf_simd)(rec_buffer, org_buffer, xpos, ypos, rec_width, rec_height, org_stride, rec_stride, sum, bitdepth - 8, block_size, damping);
        else
          TEMPLATE(detect_multi_clpf)(rec_buffer, org_buffer, xpos, ypos, rec_width, rec_height, org_stride, rec_stride, sum, bitdepth - 8, block_size, damping);
        filtered = 1;
      }
    }
  }

  for (int i = 0; i < (plane == PLANE_Y ? 4 : 1); i++) {
    res[i][0] += sum[0];
    res[i][1] += sum[1];
    res[i][2] += sum[2];
    res[i][3] += sum[3];
  }
  return filtered;
}

static void clpf_test_frame(yuv_frame_t *rec, yuv_frame_t *org, const deblock_data_t *deblock_data, const frame_info_t *frame_info, int *best_strength, int *best_bs, int bitdepth, plane_t plane) {

  int64_t sums[4][4];
  int width = plane != PLANE_Y ? rec->width >> rec->sub : rec->width;
  int height = plane != PLANE_Y ? rec->height >> rec->sub : rec->height;
  const int bs = 8; // Accurate enough for subsampled chroma
  memset(sums, 0, sizeof(sums));
  int fb_size_log2 = log2i(MAX_SB_SIZE);

  if (plane != PLANE_Y)
    clpf_rdo(0, 0, rec, org, deblock_data, bs, fb_size_log2, width/bs, height/bs, sums, bitdepth, plane, frame_info->qp);
  else
    for (int k = 0; k < (height+(1<<fb_size_log2)-bs)>>fb_size_log2; k++) {
      for (int l = 0; l < (width+(1<<fb_size_log2)-bs)>>fb_size_log2; l++) {
        int h = min(height, (k+1)<<fb_size_log2) & ((1<<fb_size_log2)-1);
        int w = min(width, (l+1)<<fb_size_log2) & ((1<<fb_size_log2)-1);
        h += !h << fb_size_log2;
        w += !w << fb_size_log2;
        clpf_rdo((k<<fb_size_log2), (l<<fb_size_log2), rec, org, deblock_data, bs, fb_size_log2, w/bs, h/bs, sums, bitdepth, plane, frame_info->qp);
      }
    }

  for (int j = 0; j < 4; j++) {
    int cost = (int)((frame_info->lambda * sums[j][0] + 6));
    if (plane != PLANE_Y) {
      // Be somewhat conservative in chroma
      sums[j][2] += sums[j][2] >> 7;
      sums[j][3] += sums[j][3] >> 7;
    }
    for (int i = 0; i < 4; i++) {
      int i_max = min(frame_info->max_clpf_strength, 3);
      if (i > i_max) sums[j][i] = 1 << 30;
      sums[j][i] = ((sums[j][i] + (i && j) * cost) << 4) + j * 4 + i;
    }
  }

  int64_t best = (int64_t)1 << 62;
  for (int j = 0; j < (plane == PLANE_Y ? 4 : 1); j++)
    for (int i = 0; i < 4; i++) {
      if ((i || !j) && sums[j][i] < best) {
        best = sums[j][i];
      }
    }
  best &= 15;
  if (best_bs)
    *best_bs = (best > 3) * (5 + (best < 12) + (best < 8));
  *best_strength = best ? 1<<((best-1) & 3) : 0;  
}

void TEMPLATE(encode_frame)(encoder_info_t *encoder_info)
{
  int k,l;
  int width = encoder_info->width;
  int height = encoder_info->height;  
  int sb_size = 1 << encoder_info->params->log2_sb_size;
  int num_sb_hor = (width + sb_size - 1) / sb_size;
  int num_sb_ver = (height + sb_size - 1) / sb_size;
  stream_t *stream = encoder_info->stream;

  if (encoder_info->frame_info.frame_type == I_FRAME)
    memset(encoder_info->deblock_data, 0, ((height/MIN_PB_SIZE) * (width/MIN_PB_SIZE) * sizeof(deblock_data_t)) );

  frame_info_t *frame_info = &(encoder_info->frame_info);
  uint8_t qp = frame_info->qp;

  double lambda_coeff;
  if (frame_info->frame_type == I_FRAME)
    lambda_coeff = encoder_info->params->lambda_coeffI;
  else if(frame_info->frame_type == P_FRAME)
    lambda_coeff = encoder_info->params->lambda_coeffP;
  else{
    if (frame_info->b_level==0)
      lambda_coeff = encoder_info->params->lambda_coeffB0;
    else if(frame_info->b_level == 1)
      lambda_coeff = encoder_info->params->lambda_coeffB1;
    else if (frame_info->b_level == 2)
      lambda_coeff = encoder_info->params->lambda_coeffB2;
    else if (frame_info->b_level == 3)
      lambda_coeff = encoder_info->params->lambda_coeffB3;
    else
      lambda_coeff = encoder_info->params->lambda_coeffB;
  }
  frame_info->lambda_coeff = lambda_coeff;
  frame_info->lambda = lambda_coeff*squared_lambda_QP[frame_info->qp];

  int sb_idx = 0;
  int start_bits_sb, end_bits_sb, num_bits_sb;
  int start_bits_frame=0, end_bits_frame, num_bits_frame;
  if (encoder_info->params->bitrate > 0) {
    start_bits_frame = get_bit_pos(stream);
    int max_qp = frame_info->frame_type == I_FRAME ? encoder_info->params->max_qpI : encoder_info->params->max_qp;
    int min_qp = frame_info->frame_type == I_FRAME ? encoder_info->params->min_qpI : encoder_info->params->min_qp;
    init_rate_control_per_frame(encoder_info->rc, min_qp, max_qp);
  }

#if CDEF
  // Set frame level CDEF parameters by guessing good values.
  encoder_info->cdef_damping = 5;
  encoder_info->cdef_bits = frame_info->frame_type == I_FRAME ? 3 : 3 - (encoder_info->frame_info.qp + 4) / 16;

  for (int i = 0; i < (1 << encoder_info->cdef_bits); i++)
    encoder_info->cdef_strengths[i] = encoder_info->cdef_uv_strengths[i] = 127;
#endif

  write_frame_header(stream, encoder_info);

  // Initialize prev_qp to qp used in frame header
  encoder_info->frame_info.prev_qp = encoder_info->frame_info.qp;

  for (k=0;k<num_sb_ver;k++){
    for (l=0;l<num_sb_hor;l++){
      int sub = encoder_info->params->subsample == 400 ? 31 : encoder_info->params->subsample == 420;
      int xposY = l*sb_size;
      int yposY = k*sb_size;
      for (int ref_idx = 0; ref_idx <= frame_info->num_ref - 1; ref_idx++){
        frame_info->mvcand_num[ref_idx] = 0;
        frame_info->mvcand_mask[ref_idx] = 0;
      }
      frame_info->best_ref = -1;

      int max_delta_qp = encoder_info->params->max_delta_qp;
      if (max_delta_qp){
        /* RDO-based search for best QP value */
        int cost,min_cost,best_qp,qp0,max_delta_qp,min_qp,max_qp;
        max_delta_qp = encoder_info->params->max_delta_qp;
        min_cost = 1<<30;
        stream_pos_t stream_pos_ref;
        read_stream_pos(&stream_pos_ref,stream);
        best_qp = qp;
        min_qp = qp-max_delta_qp;
        max_qp = qp+max_delta_qp;
        int pqp = encoder_info->frame_info.prev_qp; // Save prev_qp in local variable
        for (qp0=min_qp;qp0<=max_qp;qp0+=encoder_info->params->delta_qp_step){
          cost = TEMPLATE(process_block)(encoder_info, sb_size, yposY, xposY, qp0, sub);
          if (cost < min_cost){
            min_cost = cost;
            best_qp = qp0;
          }
        }
        encoder_info->frame_info.prev_qp = pqp; // Restore prev_qp from local variable
        write_stream_pos(stream,&stream_pos_ref);
        TEMPLATE(process_block)(encoder_info, sb_size, yposY, xposY, best_qp, sub);
      }
      else{
        if (encoder_info->params->bitrate > 0) {
          start_bits_sb = get_bit_pos(stream);
          TEMPLATE(process_block)(encoder_info, sb_size, yposY, xposY, qp, sub);
          end_bits_sb = get_bit_pos(stream);
          num_bits_sb = end_bits_sb - start_bits_sb;
          qp = update_rate_control_sb(encoder_info->rc, sb_idx, num_bits_sb, qp);
          sb_idx++;
        }
        else {
          TEMPLATE(process_block)(encoder_info, sb_size, yposY, xposY, qp, sub);
        }
      }
    }
  }

  qp = encoder_info->frame_info.qp = encoder_info->frame_info.prev_qp; //TODO: Consider using average QP instead

  //Scale and store MVs in encode_frame()
  if (encoder_info->params->interp_ref > 1) {
    int gop_size = encoder_info->params->num_reorder_pics + 1;
    int b_level = encoder_info->frame_info.b_level;
    int frame_type = encoder_info->frame_info.frame_type;
    int frame_num = encoder_info->frame_info.frame_num;
    TEMPLATE(store_mv)(width, height, b_level, frame_type, frame_num, gop_size, encoder_info->deblock_data);
  }

  if (encoder_info->params->deblocking){
    //TODO: Use QP per SB or average QP
    TEMPLATE(deblock_frame_y)(encoder_info->rec, encoder_info->deblock_data, width, height, qp, encoder_info->params->bitdepth);
    if (encoder_info->params->subsample != 400) {
      int qpc = encoder_info->params->subsample != 444 ? chroma_qp[qp] : qp;
      TEMPLATE(deblock_frame_uv)(encoder_info->rec, encoder_info->deblock_data, width, height, qpc, encoder_info->params->bitdepth);
    }
  }

#if CDEF
  if (encoder_info->params->cdef) {
    int cdef_bits = TEMPLATE(cdef_search)(encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, frame_info, encoder_info, encoder_info->cdef_strengths, encoder_info->cdef_uv_strengths, encoder_info->params->cdef - 1);

    // Apply the filter using the chosen strengths
    TEMPLATE(cdef_frame)(encoder_info->cdef, encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, stream, 0, encoder_info->params->bitdepth, 0);
    TEMPLATE(cdef_frame)(encoder_info->cdef, encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, stream, 0, encoder_info->params->bitdepth, 1);
    TEMPLATE(cdef_frame)(encoder_info->cdef, encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, stream, 0, encoder_info->params->bitdepth, 2);

    // Modify the uncompressed header
    stream_pos_t cur_stream_pos;
    read_stream_pos(&cur_stream_pos, encoder_info->stream);
    encoder_info->cdef_bits = cdef_bits;
    write_stream_pos(encoder_info->stream, &encoder_info->cdef_header_pos);
    write_cdef_params(encoder_info->stream, encoder_info);
    write_stream_pos(encoder_info->stream, &cur_stream_pos);
  }
#endif

  if (encoder_info->params->clpf){
    if (qp <= 16) // CLPF will have no effect if the quality is very high
      put_flc(2, 0, stream);
    else {
      int enable_fb_flag = 1;
      int fb_size_log2;
      int strength_y, strength_u, strength_v;
      // Find the best strength for the entire frame
      clpf_test_frame(encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, frame_info, &strength_y, &fb_size_log2, encoder_info->params->bitdepth, PLANE_Y);
      clpf_test_frame(encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, frame_info, &strength_u, 0, encoder_info->params->bitdepth, PLANE_U);
      clpf_test_frame(encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, frame_info, &strength_v, 0, encoder_info->params->bitdepth, PLANE_V);
      if (!fb_size_log2) { // Disable sb signal
        enable_fb_flag = 0;
        fb_size_log2 = log2i(MAX_SB_SIZE);
      }

      put_flc(2, strength_y - (strength_y == 4), stream);
      put_flc(2, strength_u - (strength_u == 4), stream);
      put_flc(2, strength_v - (strength_v == 4), stream);
      // Apply the filter using the chosen strengths
      if (strength_y) {
        put_flc(2, (fb_size_log2 - 4)*enable_fb_flag, stream);
        TEMPLATE(clpf_frame)(encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, stream, enable_fb_flag, strength_y, fb_size_log2, encoder_info->params->bitdepth, PLANE_Y, qp, clpf_decision);
      }
      if (strength_u)
        TEMPLATE(clpf_frame)(encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, stream, 0, strength_u, 4, encoder_info->params->bitdepth, PLANE_U, qp, NULL);
      if (strength_v)
        TEMPLATE(clpf_frame)(encoder_info->rec, encoder_info->orig, encoder_info->deblock_data, stream, 0, strength_v, 4, encoder_info->params->bitdepth, PLANE_V, qp, NULL);
    }
  }

  if (encoder_info->params->bitrate > 0) {
    end_bits_frame = get_bit_pos(stream);
    num_bits_frame = end_bits_frame - start_bits_frame;
    update_rate_control_per_frame(encoder_info->rc, num_bits_frame);
  }

  /* Sliding window operation for reference frame buffer by circular buffer */

  /* Store pointer to reference frame that is shifted out of reference buffer */
  yuv_frame_t *tmp = encoder_info->ref[MAX_REF_FRAMES-1];

  /* Update remaining pointers to implement sliding window reference buffer operation */
  memmove(encoder_info->ref+1, encoder_info->ref, sizeof(yuv_frame_t*)*(MAX_REF_FRAMES-1));

  /* Set ref[0] to the memory slot where the new current reconstructed frame wil replace reference frame being shifted out */
  encoder_info->ref[0] = tmp;

  /* Pad the reconstructed frame and write into ref[0] */
  TEMPLATE(create_reference_frame)(encoder_info->ref[0],encoder_info->rec);

#if 0
  /* To test sliding window operation */
  int offsetx = 500;
  int offsety = 200;
  int offset_rec = offsety * encoder_info->rec->stride_y +  offsetx;
  int offset_ref = offsety * encoder_info->ref[0]->stride_y +  offsetx;
  printf("rec: %3d ",encoder_info->rec->y[offset_rec]);
  printf("ref: ");
  for (r=0;r<MAX_REF_FRAMES;r++){
    printf("%3d ",encoder_info->ref[r]->y[offset_ref]);
  }
#endif

}


