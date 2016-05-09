#include <math.h>
#include <assert.h>
#include <string.h>
#include "simd.h"
#include "global.h"
#include "temporal_interp.h"
#include "common_frame.h"
#include "simd.h"
#include "common_kernels.h"

#define BLOCK_STEP 16
#define MAX_CANDS 20
#define NUM_GUIDES 4
#define COST_MAX 0x3fffffff

#define COST_FILTER_DIV 10

#define USE_CHROMA 0 // 444 not supported!
#define MAX_LEVELS 4

#define SAD_COSTS
#ifdef SAD_COSTS
#define LAMBDA ((3000*BLOCK_STEP)/16)
#define LAMBDA_SHIFT 4
#else
#define LAMBDA 125000
#define LAMBDA_SHIFT 4
#endif

#define LONG_LIST_SIZE 32

#define SKIP_THRESHOLD 8

#define ACC_BITS 3
#define ACC_SCALE (1<<ACC_BITS)
#define ACC_ROUND (1<<(ACC_BITS-1))

typedef uint32_t cost_t;

typedef struct mv_data_t {
  mv_t * mv[2];
  int * bgmap;
  cost_t * cost[2];
  int wt[2];
  int pos;
  int ratio;
  int reversed;
  int skip_thr;
  mv_t skip_mv;
  mv_t scaled_skip_mv;
  int bbs;
  int bs;
  int step;
  int bw;
  int bh;
  int pw;
  int ph;
} mv_data_t;

static int scale_val(int v, int numer, int denom)
{
  if (denom==0) return 0;
  int prod=v*numer;
  if (denom<0){
    denom=-denom;
    prod=-prod;
  }
  return (prod >= 0) ? ((prod + (denom/2)) / denom) : -((-prod+(denom/2)) / denom);
}

static mv_t scale_mv(mv_t mv, int numer, int denom)
{
  mv_t mv_out;
  if (numer==denom) {
    mv_out = mv;
  } else if (numer==-denom) {
    mv_out.x = -mv.x;
    mv_out.y = -mv.y;
  } else {
    mv_out.x = scale_val(mv.x, numer, denom);
    mv_out.y = scale_val(mv.y, numer, denom);
  }

  return mv_out;
}

mv_data_t* alloc_mv_data(int w, int h, int bs, int bbs, int ratio, int k, int interpolate)
{
  mv_data_t *data;
  data=(mv_data_t*) malloc(sizeof(mv_data_t));
  data->step=bbs/bs;
  int bw=data->step*((w+(bbs-1))/bbs);
  int bh=data->step*((h+(bbs-1))/bbs);
  int area=bw*bh;


  data->bw=bw;
  data->bh=bh;
  data->pw=w;
  data->ph=h;
  data->bbs=bbs;
  data->bs=bs;
  data->skip_thr=SKIP_THRESHOLD;/* FIXME: make adaptive*/
  data->skip_mv.x=0;
  data->skip_mv.y=0;
  data->scaled_skip_mv.x=0;
  data->scaled_skip_mv.y=0;
  data->mv[0]=(mv_t*) malloc(area*sizeof(mv_t));
  data->mv[1]=(mv_t*) malloc(area*sizeof(mv_t));
  data->cost[0]=(cost_t*) malloc(area*sizeof(cost_t));
  data->cost[1]=(cost_t*) malloc(area*sizeof(cost_t));
  data->bgmap=(int*) malloc(area*sizeof(cost_t));

  if (interpolate) {
    data->ratio=ratio;
    data->reversed=k>ratio/2;
    /* Weight of pic0 and pic1*/
    data->wt[0] = data->reversed ? k : ratio-k;
    data->wt[1] = ratio-data->wt[0];
    data->pos = k;
  } else {
    /* extrapolating*/
    data->ratio=ratio+2*k+1;
    data->reversed=0;
    data->wt[0]=(ratio+k+1);
    data->wt[1]=-k;
    data->pos=k;

  }

  return data;

}

void free_mv_data(mv_data_t* mv_data)
{
  free(mv_data->mv[0]);
  free(mv_data->mv[1]);
  free(mv_data->cost[0]);
  free(mv_data->cost[1]);
  free(mv_data->bgmap);
  free(mv_data);
}

static void scale_frame_down2x2(yuv_frame_t* sin, yuv_frame_t* sout)
{
  int wo=sout->width;
  int ho=sout->height;
  int so=sout->stride_y;
  int si=sin->stride_y;
  int soc=sout->stride_c;
  int sic=sin->stride_c;
  int i, j;

  for (i=0; i<ho; ++i) {

    for (j=0; j<wo; ++j) {
      sout->y[i*so+j]=( ((sin->y[(2*i+0)*si+(2*j+0)] + sin->y[(2*i+1)*si+(2*j+0)]+1)>>1)+
                      + ((sin->y[(2*i+0)*si+(2*j+1)] + sin->y[(2*i+1)*si+(2*j+1)]+1)>>1) )>>1;
    }

  }
  ho /= 2;
  wo /= 2;
  for (int i=0; i<ho; ++i) {

    for (j=0; j<wo; ++j) {
      sout->u[i*soc+j]=( ((sin->u[(2*i+0)*sic+(2*j+0)] + sin->u[(2*i+1)*sic+(2*j+0)]+1)>>1)+
                       + ((sin->u[(2*i+0)*sic+(2*j+1)] + sin->u[(2*i+1)*sic+(2*j+1)]+1)>>1) )>>1;
    }

    for (j=0; j<wo; ++j) {
      sout->v[i*soc+j]=( ((sin->v[(2*i+0)*sic+(2*j+0)] + sin->v[(2*i+1)*sic+(2*j+0)]+1)>>1)+
                       + ((sin->v[(2*i+0)*sic+(2*j+1)] + sin->v[(2*i+1)*sic+(2*j+1)]+1)>>1) )>>1;
    }

  }
  pad_yuv_frame(sout);
}

void scale_frame_down2x2_simd(yuv_frame_t* sin, yuv_frame_t* sout)
{
  int wo=sout->width;
  int ho=sout->height;
  int so=sout->stride_y;
  int si=sin->stride_y;
  int i, j;
  v128 ones = v128_dup_8(1);
  v128 z = v128_dup_8(0);
  for (i=0; i<ho; ++i) {

    for (j=0; j<=wo-8; j+=8) {
      v128 a = v128_load_aligned(&sin->y[(2*i+0)*si+2*j]);
      v128 b = v128_load_aligned(&sin->y[(2*i+1)*si+2*j]);
      v128 c = v128_avg_u8(a,b);
      v128 d = v128_shr_s16(v128_madd_us8(c,ones),1);
      v64_store_aligned(&sout->y[i*so+j], v128_low_v64(v128_pack_s16_u8(z,d)));
    }
    for (; j<wo; ++j) {
      sout->y[i*so+j]=( ((sin->y[(2*i+0)*si+(2*j+0)] + sin->y[(2*i+1)*si+(2*j+0)]+1)>>1)+
                      + ((sin->y[(2*i+0)*si+(2*j+1)] + sin->y[(2*i+1)*si+(2*j+1)]+1)>>1) )>>1;
    }

  }
#if USE_CHROMA
  int soc=sout->stride_c;
  int sic=sin->stride_c;
  ho /= 2;
  wo /= 2;
  for (int i=0; i<ho; ++i) {

    for (j=0; j<=wo-8; j+=8) {
      v128 a = v128_load_aligned(&sin->u[(2*i+0)*sic+2*j]);
      v128 b = v128_load_aligned(&sin->u[(2*i+1)*sic+2*j]);
      v128 c = v128_avg_u8(a,b);
      v128 d = v128_shr_s16(v128_madd_us8(c,ones),1);
      v64_store_aligned(&sout->u[i*soc+j], v128_low_v64(v128_pack_s16_u8(z,d)));
    }
    for (; j<wo; ++j) {
      sout->u[i*soc+j]=( ((sin->u[(2*i+0)*sic+(2*j+0)] + sin->u[(2*i+1)*sic+(2*j+0)]+1)>>1)+
                       + ((sin->u[(2*i+0)*sic+(2*j+1)] + sin->u[(2*i+1)*sic+(2*j+1)]+1)>>1) )>>1;
    }

    for (j=0; j<=wo-8; j+=8) {
      v128 a = v128_load_aligned(&sin->v[(2*i+0)*sic+2*j]);
      v128 b = v128_load_aligned(&sin->v[(2*i+1)*sic+2*j]);
      v128 c = v128_avg_u8(a,b);
      v128 d = v128_shr_s16(v128_madd_us8(c,ones),1);
      v64_store_aligned(&sout->v[i*soc+j], v128_low_v64(v128_pack_s16_u8(z,d)));
    }
    for (; j<wo; ++j) {
      sout->v[i*soc+j]=( ((sin->v[(2*i+0)*sic+(2*j+0)] + sin->v[(2*i+1)*sic+(2*j+0)]+1)>>1)+
                       + ((sin->v[(2*i+0)*sic+(2*j+1)] + sin->v[(2*i+1)*sic+(2*j+1)]+1)>>1) )>>1;
    }

  }
#endif
  pad_yuv_frame(sout);
}

static void upscale_mv_data_2x2(mv_data_t* mv_data_in, mv_data_t* mv_data_out)
{

  assert(mv_data_in->ratio==mv_data_out->ratio);
  assert(mv_data_in->pos==mv_data_out->pos);
  assert(mv_data_in->wt[0]==mv_data_out->wt[0]);
  assert(mv_data_in->wt[1]==mv_data_out->wt[1]);

  int bwo=mv_data_out->bw;
  int bho=mv_data_out->bh;
  int bwi=mv_data_in->bw;

  memset(mv_data_out->mv[0], 0, sizeof(mv_t)*bwo*bho);
  memset(mv_data_out->mv[1], 0, sizeof(mv_t)*bwo*bho);

  for (int i=0; i<bho; ++i) {
    for (int j=0; j<bwo; ++j) {
      int po=i*bwo+j;
      int pi=(i/2)*bwi+(j/2);
      mv_data_out->mv[1][po].x=mv_data_in->mv[1][pi].x<<1;
      mv_data_out->mv[1][po].y=mv_data_in->mv[1][pi].y<<1;
      mv_data_out->mv[0][po]=scale_mv(mv_data_out->mv[1][po], -mv_data_out->wt[1], mv_data_out->wt[0]);
    }
  }
}

static int add_cand(mv_t* cand_list, int max, int len, mv_t cand)
{
  if (len<max) {
    cand_list[len]=cand;
    for (int i=0; i<len; ++i) {
      if (cand_list[i].x==cand.x && cand_list[i].y==cand.y) {
        return len;
      }
    }
    return len+1;
  }
  return len;

}

static int get_merge_cands(mv_data_t* mv_data, mv_t* cand_list, int idx, int xp, int yp, int max_cands) {

  int len=0;
  int yoff=((yp&1) ? 2 : 1);
  int xoff=((yp&1) ? 2 : 1);

  len=add_cand(cand_list, max_cands, len, mv_data->mv[idx][yp*mv_data->bw+xp]);
  if (yp-yoff>=0) len=add_cand(cand_list, max_cands, len, mv_data->mv[idx][(yp-yoff)*mv_data->bw+xp]);
  if (yp+yoff<mv_data->bh) len=add_cand(cand_list, max_cands, len, mv_data->mv[idx][(yp+yoff)*mv_data->bw+xp]);
  if (xp-xoff>=0) len=add_cand(cand_list, max_cands, len, mv_data->mv[idx][yp*mv_data->bw+xp-xoff]);
  if (xp+xoff<mv_data->bw) len=add_cand(cand_list, max_cands, len, mv_data->mv[idx][yp*mv_data->bw+xp+xoff]);

  return len;
}

static int get_cands(mv_data_t* mv_data, mv_t* cand_list, mv_data_t** guide_mv_data, int num_guides, int xp, int yp, int max_cands, int xstep, int ystep) {

  // Zero
  int len=0;
  mv_t zero={0,0};
  int pos=yp*mv_data->bw+xp;
  len=add_cand(cand_list, max_cands, len, zero);

  for (int i=0; i<num_guides; ++i) {
    mv_data_t* gmv_data=guide_mv_data[i];
    int numer=(mv_data->reversed==gmv_data->reversed)? mv_data->wt[0] : -mv_data->wt[0];
    int denom=gmv_data->wt[0];
    mv_t gmv;
    gmv = scale_mv(gmv_data->mv[1][pos], numer, denom);
    len=add_cand(cand_list, max_cands, len, gmv);
//    if (xp<mv_data->bw-xstep && yp<mv_data->bh-ystep) {
//      gmv = scale_mv(gmv_data->mv[1][pos+mv_data->bw*ystep+xstep], numer, denom);
//      len=add_cand(cand_list, max_cands, len, gmv);
//    }
//    if (xp<mv_data->bw-xstep) {
//      gmv = scale_mv(gmv_data->mv[1][pos+xstep], numer, denom);
//      len=add_cand(cand_list, max_cands, len, gmv);
//    }
//    if (yp<mv_data->bh-ystep) {
//      gmv = scale_mv(gmv_data->mv[1][pos+ystep*mv_data->bw], numer, denom);
//      len=add_cand(cand_list, max_cands, len, gmv);
//    }
//

  }


  if (yp>0 && xp<mv_data->bw-xstep) {
    len=add_cand(cand_list, max_cands, len, mv_data->mv[1][(yp-ystep)*mv_data->bw+xp+xstep]);
  }
//  if (yp>0 && xp>0) {
//    len=add_cand(cand_list, max_cands, len, mv_data->mv[1][(yp-ystep)*mv_data->bw+xp-xstep]);
//  }
  if (xp>0) {
    len=add_cand(cand_list, max_cands, len, mv_data->mv[1][yp*mv_data->bw+xp-xstep]);
  }
  if (yp>0) {
    len=add_cand(cand_list, max_cands, len, mv_data->mv[1][(yp-ystep)*mv_data->bw+xp]);
  }



  return len;
}

static void make_cross(mv_t* cross, int offset, mv_t centre)
{
  cross[0].x=centre.x-offset;
  cross[0].y=centre.y;
  cross[1].x=centre.x+offset;
  cross[1].y=centre.y;
  cross[2].x=centre.x;
  cross[2].y=centre.y-offset;
  cross[3].x=centre.x;
  cross[3].y=centre.y+offset;

}

static int get_mv_cost(mv_t mv, mv_data_t* mv_data, int idx, int xp, int yp, int xs, int ys, int lambda){
  int diff=0;
  int bw=mv_data->bw;
  mv_t* mv_array = mv_data->mv[idx];
  if (xp==0 && yp==0) {
    diff=0;//abs(mv.x)+abs(mv.y);
  } else if (yp>0 && xp>0 && xp < mv_data->bw-xs) {
    diff=abs(mv.x-mv_array[(yp-ys)*bw+xp+xs].x) + abs(mv.y-mv_array[(yp-ys)*bw+xp+xs].y);
    diff+=abs(mv.x-mv_array[(yp-ys)*bw+xp].x) + abs(mv.y-mv_array[(yp-ys)*bw+xp].y);
    diff+=abs(mv.x-mv_array[(yp-ys)*bw+xp-xs].x) + abs(mv.y-mv_array[(yp-ys)*bw+xp-xs].y);
    diff+=abs(mv.x-mv_array[yp*bw+xp-xs].x) + abs(mv.y-mv_array[yp*bw+xp-xs].y);
  } else if (yp==0){
    diff=abs(mv.x-mv_array[xp-xs].x) + abs(mv.y-mv_array[xp-xs].y);
  } else if (xp==0) {
    diff=abs(mv.x-mv_array[(yp-ys)*bw+xp+xs].x) + abs(mv.y-mv_array[(yp-ys)*bw+xp+xs].y);
    diff+=abs(mv.x-mv_array[(yp-ys)*bw+xp].x) + abs(mv.y-mv_array[(yp-ys)*bw+xp].y);
  }

  return (diff*lambda) >> (LAMBDA_SHIFT+ACC_BITS);
}

void mot_comp_avg(int xstart, int ystart, uint8_t* ref0, int s0, uint8_t * ref1, int s1, uint8_t * pic, int sp, mv_t mv0, mv_t mv1, int wP, int hP, int pad, int size, int wt[2]){

  int xs[2];
  int ys[2];
  // For the moment just round to the nearest integer - don't do subpel
  xs[0]=xstart+((mv0.x+ACC_ROUND)>>ACC_BITS);
  xs[1]=xstart+((mv1.x+ACC_ROUND)>>ACC_BITS);
  ys[0]=ystart+((mv0.y+ACC_ROUND)>>ACC_BITS);
  ys[1]=ystart+((mv1.y+ACC_ROUND)>>ACC_BITS);

  uint8_t* p=&pic[ystart*sp+xstart];

  if (xs[0]>=-pad && xs[0]+size <= wP && ys[0]>=-pad && ys[0]+size<=hP
      && xs[1]>=-pad && xs[1]+size <= wP && ys[1]>=-pad && ys[1]+size<=hP) {

    uint8_t* r0=&ref0[ys[0]*s0+xs[0]];
    uint8_t* r1=&ref1[ys[1]*s1+xs[1]];
    if (use_simd && size>=4) {
      block_avg_simd(p,r0,r1,sp,s0,s1,size,size);
    } else {
      for (int i=0; i<size; ++i) {
        for (int j=0; j<size; ++j) {
          p[i*sp+j] = (r0[i*s0+j]+r1[i*s1+j]+1)/2;
        }
      }
    }

  } else if (xs[1]>=-pad && xs[1]+size <= wP && ys[1]>=-pad && ys[1]+size<=hP){
    uint8_t* r1=&ref1[ys[1]*s1+xs[1]];
    for (int i=0; i<size; ++i) {
      memcpy(&p[i*sp], &r1[i*s1], size*sizeof(uint8_t));
    }
  } else if (xs[0]>=-pad && xs[0]+size <= wP && ys[0]>=-pad && ys[0]+size<=hP){
    uint8_t* r0=&ref0[ys[0]*s0+xs[0]];
    for (int i=0; i<size; ++i) {
      memcpy(&p[i*sp], &r0[i*s1], size*sizeof(uint8_t));
    }

  } else {
    // Clipped version
    uint8_t* r0=ref0;
    uint8_t* r1=ref1;
    for (int i=0; i<size; ++i) {
      for (int j=0; j<size; ++j) {
        int xpos0=min(wP-1, max(-pad, j+xs[0]));
        int xpos1=min(wP-1, max(-pad, j+xs[1]));
        int ypos0=min(hP-1, max(-pad, i+ys[0]));
        int ypos1=min(hP-1, max(-pad, i+ys[1]));
        p[i*sp+j] = (r0[ypos0*s0+xpos0]+r1[ypos1*s1+xpos1]+1)/2;
      }
    }

  }

}

static uint32_t sad_cost(int xstart, int ystart, yuv_frame_t* pic[2], mv_t mv[2], int size, uint32_t cost_start, uint32_t best_cost){
  int xs[2];
  int ys[2];

  // For the moment round to nearest integer. Don't do subpel
  xs[0]=xstart+((mv[0].x+ACC_ROUND)>>ACC_BITS);
  xs[1]=xstart+((mv[1].x+ACC_ROUND)>>ACC_BITS);
  ys[0]=ystart+((mv[0].y+ACC_ROUND)>>ACC_BITS);
  ys[1]=ystart+((mv[1].y+ACC_ROUND)>>ACC_BITS);

  int pady=pic[0]->pad_hor_y;
  int widthP=pic[0]->width+pady;
  int heightP=pic[0]->height+pady;

  uint32_t bcost=cost_start;

  int s0=pic[0]->stride_y;
  int s1=pic[1]->stride_y;
#if USE_CHROMA
  int sc0=pic[0]->stride_c;
  int sc1=pic[1]->stride_c;
#endif
  if (xs[0]>=-pady && xs[0]+size <= widthP && ys[0]>=-pady && ys[0]+size<=heightP
      && xs[1]>=-pady && xs[1]+size <= widthP && ys[1]>=-pady && ys[1]+size<=heightP) {

    uint8_t* p0=&pic[0]->y[ys[0]*s0+xs[0]];
    uint8_t* p1=&pic[1]->y[ys[1]*s1+xs[1]];
    if (use_simd && size >= 4) {
      bcost += sad_calc_simd_unaligned(p0, p1, s0, s1, size, size);
    } else {
      for (int i=0; i<size; ++i) {
        for (int j=0; j<size; ++j) {
          bcost += abs(p1[i*s1+j]-p0[i*s0+j]);
        }
      }
    }
#if USE_CHROMA
    if (bcost < best_cost) {
      uint32_t ccost=0;
      int cpos0=(ys[0]/2)*sc0+(xs[0]/2);
      int cpos1=(ys[1]/2)*sc1+(xs[1]/2);
      if (use_simd && size >= 8) {
        ccost += sad_calc_simd_unaligned(&pic[0]->u[cpos0], &pic[1]->u[cpos1], sc0, sc1, size/2, size/2);
        ccost += sad_calc_simd_unaligned(&pic[0]->v[cpos0], &pic[1]->v[cpos1], sc0, sc1, size/2, size/2);
      } else {
        p0=&pic[0]->u[cpos0];
        p1=&pic[1]->u[cpos1];
        for (int i=0; i<size/2; ++i) {
          for (int j=0; j<size/2; ++j) {
            ccost += abs(p1[i*sc1+j]-p0[i*sc0+j]);
          }
        }
        p0=&pic[0]->v[cpos0];
        p1=&pic[1]->v[cpos1];
        for (int i=0; i<size/2; ++i) {
          for (int j=0; j<size/2; ++j) {
            ccost += abs(p1[i*sc1+j]-p0[i*sc0+j]);
          }
        }
      }
      bcost += 4*ccost;// weight for chroma
    }
#endif

  } else {
    // Clipped version, luma only
    uint8_t* p0=pic[0]->y;
    uint8_t* p1=pic[1]->y;
    for (int i=0; i<size; ++i) {
      for (int j=0; j<size; ++j) {
        int xpos0=min(widthP-1, max(-pady, j+xs[0]));
        int xpos1=min(widthP-1, max(-pady, j+xs[1]));
        int ypos0=min(heightP-1, max(-pady, i+ys[0]));
        int ypos1=min(heightP-1, max(-pady, i+ys[1]));
        bcost += abs(p1[ypos1*s1+xpos1]-p0[ypos0*s0+xpos0]);
      }
    }
  }

  return bcost;
}

static void skip_test(mv_data_t* mv_data, yuv_frame_t* picdata[2], int xp, int yp)
{
  // Do the search with the larger size, but data is stored at the smaller
  int xstart=xp*mv_data->bs;
  int ystart=yp*mv_data->bs;

  mv_t mv1=mv_data->skip_mv;
  mv_t mv0=mv_data->scaled_skip_mv;

  int xs[2];
  int ys[2];


  int pos=yp*mv_data->bw+xp;

  int size=mv_data->bbs;

  int thr=mv_data->skip_thr*8*8;
  int skip=1;
  int pady=picdata[0]->pad_ver_y;
  int padx=picdata[0]->pad_hor_y;
  int hP=picdata[0]->height+pady;
  int wP=picdata[0]->width+padx;
  int s0=picdata[0]->stride_y;
  int s1=picdata[1]->stride_y;
  for (int p=ystart; p<ystart+size && skip; p+=8) {
    for (int q=xstart; q<xstart+size && skip; q+=8) {
      xs[0]=q+((mv0.x+ACC_ROUND)>>ACC_BITS);
      xs[1]=q+((mv1.x+ACC_ROUND)>>ACC_BITS);
      ys[0]=p+((mv0.y+ACC_ROUND)>>ACC_BITS);
      ys[1]=p+((mv1.y+ACC_ROUND)>>ACC_BITS);
      // For the moment just round to the nearest integer - don't do subpel
      if (xs[0]>=-padx && xs[0]+8 <= wP && ys[0]>=-pady && ys[0]+8<=hP
          && xs[1]>=-padx && xs[1]+8 <= wP && ys[1]>=-pady && ys[1]+8<=hP) {
        int sum=0;
        uint8_t* r0=&picdata[0]->y[ys[0]*s0+xs[0]];
        uint8_t* r1=&picdata[1]->y[ys[1]*s1+xs[1]];
        if (use_simd) {
          sum = sad_calc_simd_unaligned(r0, r1, s0, s1, 8, 8);
        } else {
          for (int i=0; i<8; i++) {
            for (int j=0; j<8; j++) {
              sum += abs(r0[i*s0+j]-r1[i*s1+j]);
            }
          }
        }
        if (sum>thr) {
          skip=0;
          break;
        }

      } else {
        skip=0; // Don't support skip if outside the frame and padding
        break;
      }
    }
  }
#if USE_CHROMA
  if (skip) {
    int thrC=thr/4;
    int s0C=picdata[0]->stride_c;
    int s1C=picdata[1]->stride_c;
    xstart /= 2;
    ystart /= 2;
    size /= 2;
    mv0.x /= 2;
    mv0.y /= 2;
    mv1.x /= 2;
    mv1.y /= 2;

    for (int p=ystart; p<ystart+size && skip; p+=8) {
      for (int q=xstart; q<xstart+size && skip; q+=8) {
        xs[0]=q+((mv0.x+ACC_ROUND)>>ACC_BITS);
        xs[1]=q+((mv1.x+ACC_ROUND)>>ACC_BITS);
        ys[0]=p+((mv0.y+ACC_ROUND)>>ACC_BITS);
        ys[1]=p+((mv1.y+ACC_ROUND)>>ACC_BITS);
        int sumU=0;
        int sumV=0;
        uint8_t* r0U=&picdata[0]->u[ys[0]*s0C+xs[0]];
        uint8_t* r1U=&picdata[1]->u[ys[1]*s1C+xs[1]];
        uint8_t* r0V=&picdata[0]->v[ys[0]*s0C+xs[0]];
        uint8_t* r1V=&picdata[1]->v[ys[1]*s1C+xs[1]];
        if (use_simd) {
          sumU = sad_calc_simd_unaligned(r0U, r1U, s0C, s1C, 8, 8);
          sumV = sad_calc_simd_unaligned(r0V, r1V, s0C, s1C, 8, 8);
        } else {
          for (int i=0; i<8; i++) {
            for (int j=0; j<8; j++) {
              sumU += abs(r0U[i*s0C+j]-r1U[i*s1C+j]);
              sumV += abs(r0V[i*s0C+j]-r1V[i*s1C+j]);
            }
          }
        }
        if (sumU>thrC || sumV>thrC) {
          skip=0;
          break;
        }

      }
    }

  }
#endif
  if (skip) {
    mv_data->bgmap[pos]=1;
    mv_data->mv[1][pos]=mv_data->skip_mv;
    mv_data->mv[0][pos]=mv_data->scaled_skip_mv;
    mv_data->cost[1][pos]=0;
    mv_data->cost[0][pos]=0;
  }

  int bw=mv_data->bw;
  mv_data->mv[0][pos+1]=mv_data->mv[0][pos];
  mv_data->mv[1][pos+1]=mv_data->mv[1][pos];
  mv_data->mv[0][pos+bw]=mv_data->mv[0][pos];
  mv_data->mv[1][pos+bw]=mv_data->mv[1][pos];
  mv_data->mv[0][pos+bw+1]=mv_data->mv[0][pos];
  mv_data->mv[1][pos+bw+1]=mv_data->mv[1][pos];
  mv_data->bgmap[pos+1]=mv_data->bgmap[pos];
  mv_data->bgmap[pos+bw]=mv_data->bgmap[pos];
  mv_data->bgmap[pos+bw+1]=mv_data->bgmap[pos];

}


static void adaptive_search_v2(mv_data_t* mv_data, int guided, mv_t * cand_list, int num_cands, yuv_frame_t* picdata[2], int xp, int yp, int xstep, int ystep)
{
  // Do the search with the larger size, but data is stored at the smaller
  int xstart=xp*mv_data->bs;
  int ystart=yp*mv_data->bs;

  mv_t cross[4];

  int size=mv_data->bbs;

  mv_t best_mv=cand_list[0];
  mv_t best_scaled_mv;
  best_scaled_mv=scale_mv(best_mv, -mv_data->wt[1], mv_data->wt[0]);
  mv_t mv[2];
  mv[1] = best_mv;
  mv[0] = best_scaled_mv;
  uint32_t best_cost=COST_MAX;

  mv_t cand_refine_list[MAX_CANDS];
  mv_t cand_refine_scaled_list[MAX_CANDS];
  uint32_t cand_best_costs[MAX_CANDS];

  uint32_t lambda = guided ? LAMBDA/4 : LAMBDA;

  for (int c=0; c<num_cands; c++) {

    mv[1].x = cand_list[c].x;
    mv[1].y = cand_list[c].y;
    mv[0] = scale_mv(cand_list[c], -mv_data->wt[1], mv_data->wt[0]);

    cand_best_costs[c]=get_mv_cost(cand_list[c], mv_data, 1, xp, yp, xstep, ystep, lambda);
    cand_best_costs[c] = sad_cost(xstart, ystart, picdata, mv, size, cand_best_costs[c], COST_MAX);
    cand_refine_list[c]=mv[1];
    cand_refine_scaled_list[c]=mv[0];

    if ((((4+c)*cand_best_costs[c])/8) < best_cost) {
      int shift=guided ? 0+ACC_BITS : 3+ACC_BITS;
      int count=guided ? 8 : 64;
      while (shift>=ACC_BITS && count>0) {

        make_cross(cross, 1<<shift, cand_refine_list[c]);
        int better=0;
        for (int i=0; i<4; ++i) {
          mv_t rmv=cross[i];
          mv[1].x = rmv.x;
          mv[1].y = rmv.y;
          mv[0] = scale_mv(rmv, -mv_data->wt[1], mv_data->wt[0]);

          uint32_t bcost=get_mv_cost(rmv, mv_data, 1, xp, yp, xstep, ystep, lambda);
          bcost = sad_cost(xstart, ystart, picdata, mv, size, bcost, cand_best_costs[c]);

          if (bcost<cand_best_costs[c]){
            cand_best_costs[c]=bcost;
            cand_refine_list[c]=rmv;
            cand_refine_scaled_list[c]=mv[0];
            better=1;
          }
        }
        if (!better)
          shift--;
        count -= 4;
      };
    }
    if (cand_best_costs[c]<best_cost) {
      best_mv=cand_refine_list[c];
      best_scaled_mv=cand_refine_scaled_list[c];
      best_cost=cand_best_costs[c];
    }
  }

  mv_data->mv[1][yp*mv_data->bw+xp]=best_mv;
  mv_data->mv[0][yp*mv_data->bw+xp]=best_scaled_mv;
  mv_data->cost[1][yp*mv_data->bw+xp]=best_cost;
  mv_data->cost[0][yp*mv_data->bw+xp]=best_cost;

}

static void merge_candidate_search(mv_t* cand_list, int num_cands, mv_data_t* mv_data, mv_t* mv0, mv_t* mv1, yuv_frame_t* picdata[2], int xp, int yp)
{
  int xstart=xp*mv_data->bs;
  int ystart=yp*mv_data->bs;

  // Do this search on the smaller blocks
  int size=mv_data->bs;

  uint32_t best_cost=COST_MAX;
  mv_t best_mv={0,0};
  mv_t best_scaled_mv={0,0};
  for (int i=0; i<num_cands; ++i){
    mv_t rmv;
    mv_t mv[2];
    rmv=cand_list[i];
    // Vectors are normalised to 1 for pic1, which is always further away. This means that we get enough accuracy, as the nearest mvs will be smaller.
    mv[1].x = rmv.x;
    mv[1].y = rmv.y;
    mv[0] = scale_mv(rmv, -mv_data->wt[1], mv_data->wt[0]);
    uint32_t bcost=0;
    bcost = sad_cost(xstart, ystart, picdata, mv, size, bcost, best_cost);
    if (bcost<best_cost){
      best_cost=bcost;
      best_mv=rmv;
      best_scaled_mv=mv[0];
    }
  }
  mv1[yp*mv_data->bw+xp]=best_mv;
  mv0[yp*mv_data->bw+xp]=best_scaled_mv;
  mv_data->cost[0][yp*mv_data->bw+xp]=best_cost;
  mv_data->cost[1][yp*mv_data->bw+xp]=best_cost;

}

static mv_t mv_absdist_filter(mv_t* mlist, int num)
{
  int best_idx=0;
  int best_cost=COST_MAX;
  mv_t mv;
  int cost;
  for (int j=0; j<num; ++j) {
    cost=0;
    for (int i=0; i<num; ++i) {
      cost += abs(mlist[i].x - mlist[j].x);
      cost += abs(mlist[i].y - mlist[j].y);
    }
    if (cost<=best_cost){
      best_idx=j;
      best_cost=cost;
    }
  }
  mv.x=mlist[best_idx].x;
  mv.y=mlist[best_idx].y;

  return mv;
}

//static void filter_mvs(mv_data_t* mv_data, int step)
//{
//  mv_t vlist[5];
//  mv_t *mv_cpy = (mv_t*) thor_alloc(mv_data->bw*mv_data->bh*sizeof(mv_t),16);
//  mv_t *scaled_mv_cpy = (mv_t*) thor_alloc(mv_data->bw*mv_data->bh*sizeof(mv_t),16);
//  int bw=mv_data->bw;
//  int bh=mv_data->bh;
//  for (int i=0; i<bh; i+=step) {
//    for (int j=0; j<bw; j+=step) {
//      mv_t* mv=&mv_data->mv[1][i*bw+j];
//      int num=0;
//      vlist[num++]=mv[0];
//      if (j<bw-step) vlist[num++]=mv[step];
//      if (j>0) vlist[num++]=mv[-step];
//      if (i<bh-step) vlist[num++]=mv[bw*step];
//      if (i>0) vlist[num++]=mv[-bw*step];
//      mv_cpy[i*bw+j]=mv_absdist_filter(vlist,num);
//      scaled_mv_cpy[i*bw+j]=scale_mv(mv_cpy[i*bw+j],-mv_data->wt[1],mv_data->wt[0]);
//
//      // Propagate
//      for (int p=0; p<step; ++p) {
//        for (int q=0; q<step; ++q) {
//          mv_cpy[(i+p)*bw+(j+q)]=mv_cpy[i*bw+j];
//          scaled_mv_cpy[(i+p)*bw+(j+q)]=scaled_mv_cpy[i*bw+j];
//        }
//      }
//    }
//  }
//  memcpy(&mv_data->mv[1][bw*step], &mv_cpy[bw*step], bw*(bh-2*step)*sizeof(mv_t));
//  memcpy(&mv_data->mv[0][bw*step], &scaled_mv_cpy[bw*step], bw*(bh-2*step)*sizeof(mv_t));
//
//  thor_free(mv_cpy);
//  thor_free(scaled_mv_cpy);
//}


static void make_skip_vector(mv_data_t* mv_data, int xp, int yp, int xstep, int ystep)
{
  int bw=mv_data->bw;
  mv_data->skip_mv.x=0;
  mv_data->skip_mv.y=0;
  mv_t vlist[3];
  int num=0;
  if (yp>0 && xp<bw-xstep) vlist[num++]=mv_data->mv[1][(yp-ystep)*bw+xp+xstep];
  if (xp>0) vlist[num++]=mv_data->mv[1][yp*bw+xp-xstep];
  if (yp>0) vlist[num++]=mv_data->mv[1][(yp-ystep)*bw+xp];
  if (num) mv_data->skip_mv=mv_absdist_filter(vlist,num);
  mv_data->scaled_skip_mv=scale_mv(mv_data->skip_mv, -mv_data->wt[1], mv_data->wt[0]);
}


/*
static void init_mv_data(mv_data_t* mv_data, const mv_data_t* guide_mv_data, int k)
{
  assert(mv_data->ratio==guide_mv_data->ratio);

  int numer=(mv_data->reversed==guide_mv_data->reversed)? mv_data->wt[0] : -mv_data->wt[0];
  int denom=guide_mv_data->wt[0];
  for (int i=0; i<mv_data->bh; ++i) {
    for (int j=0; j<mv_data->bw; ++j) {
      int pos=i*mv_data->bw+j;
      mv_data->mv[1][pos] = scale_mv(guide_mv_data->mv[1][pos], numer, denom);
      mv_data->mv[0][pos] = scale_mv(mv_data->mv[1][pos], -mv_data->wt[1], mv_data->wt[0]);
    }
  }
}
*/

static void motion_estimate_bi(mv_data_t* mv_data, mv_data_t** guide_mv_data, int num_guides, yuv_frame_t* indata0, yuv_frame_t* indata1, int k)
{
  // Estimate indata0 from indata1 and vice-versa


  const int bw=mv_data->bw;
  const int bh=mv_data->bh;

  if (num_guides==0) {
    memset(mv_data->mv[0], 0, sizeof(mv_t)*bw*bh);
    memset(mv_data->mv[1], 0, sizeof(mv_t)*bw*bh);
  }
  memset(mv_data->bgmap, 0, sizeof(int)*bw*bh);

  const int step=mv_data->step;
  mv_t cand_list[MAX_CANDS];


  yuv_frame_t* pic[2];
  pic[0] = mv_data->reversed ? indata1 : indata0;
  pic[1] = mv_data->reversed ? indata0 : indata1;

  for (int i=0; i<bh; i+=step) {
    for (int j=0; j<bw; j+=step) {
      make_skip_vector(mv_data, j, i, step, step);
      skip_test(mv_data, pic, j, i);
      int pos=i*bw+j;
      if (mv_data->bgmap[pos]==0) {

        int num_cands=get_cands(mv_data, cand_list, guide_mv_data, num_guides, j, i, MAX_CANDS, step, step);
        adaptive_search_v2(mv_data, num_guides!=0, cand_list, num_cands, pic, j, i, step, step);
      }
      // propagate
      const mv_t mv0=mv_data->mv[0][pos];
      const mv_t mv1=mv_data->mv[1][pos];
      int bgval=mv_data->bgmap[pos];
      for (int q=0; q<step; ++q) {
        for (int p=0; p<step; ++p) {
          mv_data->mv[0][pos+q*bw+p]=mv0;
          mv_data->mv[1][pos+q*bw+p]=mv1;
          mv_data->bgmap[pos+q*bw+p]=bgval;
        }
      }
    }
  }

  mv_t * mv0 = (mv_t*) thor_alloc(bw*bh*sizeof(mv_t), 16);
  mv_t * mv1 = (mv_t*) thor_alloc(bw*bh*sizeof(mv_t), 16);

  for (int i=0; i<bh; i++) {
    for (int j=0; j<bw; j++) {
      int num_cands=get_merge_cands(mv_data, cand_list, 1, j, i, MAX_CANDS);
      if (num_cands>1){
        merge_candidate_search(cand_list, num_cands, mv_data, mv0, mv1, pic, j, i);
      } else {
        mv0[i*bw+j]=mv_data->mv[0][i*bw+j];
        mv1[i*bw+j]=mv_data->mv[1][i*bw+j];
      }
    }
  }

  memcpy(mv_data->mv[0], mv0, bw*bh*sizeof(mv_t));
  memcpy(mv_data->mv[1], mv1, bw*bh*sizeof(mv_t));

  thor_free(mv0);
  thor_free(mv1);
}

static void interpolate_comp(mv_data_t* mv_data, uint8_t* p0, int s0, uint8_t* p1, int s1,
    uint8_t* out, int so, int wP, int hP, int pad, int chroma)
{
  const int bw=mv_data->bw;
  const int bh=mv_data->bh;
  const int bs=chroma ? mv_data->bs/2 :  mv_data->bs;

  for (int yp=0; yp<bh; yp++) {
    for (int xp=0; xp<bw; xp++) {

      int xstart=xp*bs;
      int ystart=yp*bs;
      mv_t mv0=mv_data->mv[0][yp*bw+xp];
      mv_t mv1=mv_data->mv[1][yp*bw+xp];
      if (chroma){
        mv1.x >>= 1;
        mv1.y >>= 1;
        mv0 = scale_mv(mv1, -mv_data->wt[1], mv_data->wt[0]);
      }
      mot_comp_avg(xstart, ystart, p0, s0, p1, s1, out, so, mv0, mv1, wP, hP, pad, bs, mv_data->wt);

    }
  }

}

static void interpolate_frame(mv_data_t* mv_data, yuv_frame_t* indata0, yuv_frame_t* indata1, yuv_frame_t* outdata, int w, int h, int k, int ratio)
{

  yuv_frame_t* pic[2];
  pic[0] = mv_data->reversed ? indata1 : indata0;
  pic[1] = mv_data->reversed ? indata0 : indata1;

  // Y
  // For MC purposes, pad by 1/2 a block
  int pad=mv_data->bs/2;
  int wP=w+pad;
  int hP=h+pad;
  int wPc=wP >> indata0->sub;
  int hPc=hP >> indata0->sub;
  int padc=pad >> indata0->sub;

  interpolate_comp(mv_data, pic[0]->y, pic[0]->stride_y, pic[1]->y, pic[1]->stride_y, outdata->y, outdata->stride_y, wP, hP, pad, 0);

  // U
  interpolate_comp(mv_data, pic[0]->u, pic[0]->stride_c, pic[1]->u, pic[1]->stride_c, outdata->u, outdata->stride_c, wPc, hPc, padc, indata0->sub);

  // V
  interpolate_comp(mv_data, pic[0]->v, pic[0]->stride_c, pic[1]->v, pic[1]->stride_c, outdata->v, outdata->stride_c, wPc, hPc, padc, indata0->sub);

}

void interpolate_frames(yuv_frame_t* new_frame, yuv_frame_t* ref0, yuv_frame_t* ref1, int ratio, int pos)
{
  int widthin = ref0->width;
  int heightin = ref0->height;

  int max_levels=min(MAX_LEVELS, (int) (log10(min(widthin, heightin))/log10(2.0)-4.0));

  mv_data_t * mv_data[MAX_LEVELS];
  mv_data_t * spatial_mv_data[MAX_LEVELS];
  mv_data_t * guide_mv_data[NUM_GUIDES];

  yuv_frame_t* in_down[MAX_LEVELS][2];
  yuv_frame_t* out_down[MAX_LEVELS];

  int interpolate = 1;
  for (int j=1; j<max_levels; j++) {
    out_down[j]=malloc(sizeof(yuv_frame_t));
    create_yuv_frame(out_down[j],widthin>>j,heightin>>j, 1, 32, 32, 0);
  }
  out_down[0]=new_frame;

  for (int j=0; j<max_levels; j++) {
    mv_data[j] = alloc_mv_data(widthin>>j, heightin>>j, BLOCK_STEP/2, BLOCK_STEP, ratio, pos, interpolate);
    mv_data[j]->ratio = ratio;
    spatial_mv_data[j] = alloc_mv_data(widthin>>j, heightin>>j, BLOCK_STEP/2, BLOCK_STEP, ratio, pos, interpolate);
    spatial_mv_data[j]->ratio = ratio;
  }

  /* Higher levels are down-sampled*/
  for (int i=1; i<max_levels; ++i) {
    in_down[i][0]=malloc(sizeof(yuv_frame_t));
    in_down[i][1]=malloc(sizeof(yuv_frame_t));
    create_yuv_frame(in_down[i][0],widthin>>i, heightin>>i, 1, 32, 32, 0);
    create_yuv_frame(in_down[i][1],widthin>>i, heightin>>i, 1, 32, 32, 0);
  }
  // Level 0 is just the original pictures
  in_down[0][0]=ref0;
  in_down[0][1]=ref1;

  for (int l=0; l<max_levels-1; ++l) {
    if (use_simd) {
      scale_frame_down2x2_simd(in_down[l][0], in_down[l+1][0]);
      scale_frame_down2x2_simd(in_down[l][1], in_down[l+1][1]);
    } else {
      scale_frame_down2x2(in_down[l][0], in_down[l+1][0]);
      scale_frame_down2x2(in_down[l][1], in_down[l+1][1]);
    }
  }


  for (int lvl=max_levels-1; lvl>=0; --lvl) {
    int num_guides=0;
    if (lvl!=max_levels-1) {
      guide_mv_data[num_guides++]=spatial_mv_data[lvl];
    }
    motion_estimate_bi(mv_data[lvl], guide_mv_data, num_guides, in_down[lvl][0], in_down[lvl][1], pos);
    if (lvl==0) interpolate_frame(mv_data[lvl], in_down[lvl][0], in_down[lvl][1], out_down[lvl], widthin, heightin, pos, ratio);
    if (lvl>0) {
      upscale_mv_data_2x2(mv_data[lvl], spatial_mv_data[lvl-1]);
    }

  }

  for (int j=1; j<max_levels; j++) {
    close_yuv_frame(out_down[j]);
    free(out_down[j]);
  }

  for (int j=0; j<max_levels; j++) {
    free_mv_data(mv_data[j]);
    free_mv_data(spatial_mv_data[j]);
  }

  /* Higher levels are down-sampled*/
  for (int i=1; i<max_levels; ++i) {
    close_yuv_frame(in_down[i][0]);
    close_yuv_frame(in_down[i][1]);
    free(in_down[i][0]);
    free(in_down[i][1]);
  }

}

