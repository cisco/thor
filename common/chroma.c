#include "chroma.h"
#include "global.h"


void chroma_444_to_420(uint8_t *src, uint8_t *dst, int ss, int ds, int w, int h)
{
  for (int i = 0; i < h; i += 2)
    for (int j = 0; j < w; j += 2)
      dst[i/2*ds+j/2] = (src[(i+0)*ss + j+0] + src[(i+0)*ss + j+1] + src[(i+1)*ss + j+0] + src[(i+1)*ss + j+j] + 2) >> 2;
}

void chroma_420_to_444(uint8_t *y444, uint8_t *u420, uint8_t *v420, uint32_t sy, uint32_t suv,
                       int32_t w, int32_t h, uint8_t *dst_u, uint8_t *dst_v, uint32_t sdst)
{
  const int POINT = 24;
  const uint64_t POW_2_POINT_M_1 = 1LL << (POINT-1);
  const int N2 = 4;
  const int N = 2*N2;
  uint16_t y_420_loc, u420_loc, v420_loc;
  uint64_t den_inv = 0;
  uint64_t a, b;
  
  for (int k = 0; k < h-N+1; k += N) {
    for (int l = 0; l < w-N+1; l += N) {
      uint16_t ysum = 0, usum = 0, vsum = 0;
      unsigned int yysum = 0, yusum = 0, uusum = 0, yvsum = 0, vvsum = 0;
      for (int i = k/2; i < k/2+N2; i++) {
        for (int j = l/2; j < l/2+N2; j++) {
          y_420_loc = (uint16_t)(((uint16_t)y444[2*i*sy+2*j]
                                  + (uint16_t)y444[2*i*sy+2*j+1]
                                  + (uint16_t)y444[(2*i+1)*sy+2*j]
                                  + (uint16_t)y444[(2*i+1)*sy+2*j+1]
                                  + 2) >>2);
          yysum += y_420_loc * y_420_loc;
          ysum  += y_420_loc;
          
          u420_loc = (uint16_t)u420[i*suv+j];
          usum  += u420_loc;
          yusum += y_420_loc * u420_loc;
          uusum += u420_loc * u420_loc;
          
          v420_loc = (uint16_t)v420[i*suv+j];
          vsum  += v420_loc;
          yvsum += y_420_loc * v420_loc;
          vvsum += v420_loc * v420_loc;
        }
      }
      int ssyy = (int)yysum - (int)(((int)ysum*(int)ysum+8)>>4);
      int ssyu = (int)yusum - (int)(((int)ysum*(int)usum+8)>>4);
      int ssuu = (int)uusum - (int)(((int)usum*(int)usum+8)>>4);
      int ssyv = (int)yvsum - (int)(((int)ysum*(int)vsum+8)>>4);
      int ssvv = (int)vvsum - (int)(((int)vsum*(int)vsum+8)>>4);
      int ssyc = ssyu;
      int sscc = ssuu;
      int csum = usum;
      uint8_t *dst_c = dst_u;
      uint8_t *c420 = u420;

      if ((ssyy && ssuu && ssyu) || (ssyy && ssvv && ssyv))
        den_inv = (uint64_t)((uint64_t)((uint64_t)1<<POINT) + (uint64_t)(((uint64_t)ssyy)/2)) / (uint64_t)ssyy;

      for (int c = 0; c < 2; c++) {
        if (ssyy && sscc && ssyc && (20*((int64_t)ssyc) * ((int64_t)ssyc) > 19*(int64_t)ssyy*(int64_t)sscc)) {
          a = (int64_t)ssyc*(int64_t)den_inv;
          b = ((((uint64_t)1<<POINT)*csum+8)>>4) - ((int64_t)(a*(int64_t)ysum+8))/16;
          for (int i = 0; i<N; i++)
            for (int j = 0; j<N; j++)
              dst_c[(i+k)*sdst+j+l] = clip255((a*y444[(k+i)*sy+l+j] + b + POW_2_POINT_M_1) >> POINT);
        } else
          for (int i = 0; i<N; i++)
            for (int j = 0; j<N; j++)
              dst_c[(i+k)*sdst+j+l] = c420[((k+i)/2)*suv+(l+j)/2];

        ssyc = (int)yvsum - (int)(((int)ysum*(int)vsum+8)>>4);
        sscc = (int)vvsum - (int)(((int)vsum*(int)vsum+8)>>4);
        csum = vsum;
        dst_c = dst_v;
        c420 = v420;
      }
    }
  }
}
