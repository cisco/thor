#ifndef __TEMPORAL_INTERP
#define __TEMPORAL_INTERP
#include "types.h"

void interpolate_frames_lbd(yuv_frame_t* new_frame, yuv_frame_t* ref0, yuv_frame_t* ref1, int ratio, int pos);
void interpolate_frames_hbd(yuv_frame_t* new_frame, yuv_frame_t* ref0, yuv_frame_t* ref1, int ratio, int pos);

#endif
