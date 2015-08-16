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

#if !defined(_GLOBAL_H_)
#define _GLOBAL_H_

#include <stdlib.h>
#include <stdio.h>

static inline void fatalerror(char error_text[])
{
    fprintf(stderr,"Run-time error...\n");
    fprintf(stderr,"%s\n",error_text);
    fprintf(stderr,"...now exiting to system...\n");
    exit(1);
}

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#define clip255(n) min(255, max(0, (n)))
#define clip(n, low, high) min ((high), max ((n), (low)))

#define MAX_BLOCK_SIZE 64        //Maximum block size
#define MIN_BLOCK_SIZE 8         //Minimum block size
#define MIN_PB_SIZE 4            //Minimum pu block size
#define MAX_QUANT_SIZE 16        //Maximum quantization block size
#define MAX_BUFFER_SIZE 4000000  //Maximum compressed buffer size per frame
#define MAX_TR_SIZE 64           //Maximum transform size
#define PADDING_Y 96             //One-sided padding range for luma
#define MAX_UINT32 1<<31         //Used e.g. to initialize search for minimum cost
#define EARLY_SKIP_BLOCK_SIZE 8  //Transform size for early skip check
#define MAX_REF_FRAMES 17        //Maximum number of reference frames
#define MAX_REORDER_BUFFER 32    //Maximum number of frames to store for reordering
#define CLPF_PERIOD 4            //Period of CLPF frames
#define CLPF_BIAS 101            //Bias used for CLPF on/off decision

#define DYADIC_CODING 1          // Support hierarchical B frames

/* Experimental */
#define ENABLE_SKIP_BIPRED 1     //Enable bipred in skip mode
#define HEVC_INTERPOLATION 0     //Enable HEVC interpolation filter


#define FILTER_HOR_AND_VER 0     //Filter horizontal and vertical intra modes (1,14,1)
#define NEW_BLOCK_STRUCTURE 0    //Non-quadtree block structure
#define NO_SUBBLOCK_SKIP 1       //Force only zero skip mv in subblocks
#define LIMIT_INTRA_MODES 1      //Allow maximum 8 intra modes
#define NEW_DEBLOCK_TEST 1       //New test for deblocking a block edge
#define NEW_MV_TEST 1            //New MV test for deblocking a line
#define NEW_DEBLOCK_FILTER 1     //New deblocking filter
#define LIMITED_SKIP 1           //Limit number of skip candidates

#if LIMITED_SKIP
#define MAX_NUM_SKIP 2           //Maximum number of skip candidates
#if NO_SUBBLOCK_SKIP
#define TWO_MVP 0                //Choose one of two motion vectors for mvp
#else
#define TWO_MVP 1                //Choose one of two motion vectors for mvp
#endif
#else
#define MAX_NUM_SKIP 4           //Maximum number of skip candidates
#define TWO_MVP 0                //Choose one of two motion vectors for mvp
#endif
/* Testing and analysis*/
#define STAT 0                   //Extended statistics printout in decoder
#define TEST_AVAILABILITY 0      //Test availability functions
#endif
