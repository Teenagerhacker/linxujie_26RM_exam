#pragma once
#include <cmath>
#include <cstdint>
// Scalar equivalents of the four CMSIS-DSP operations used by the actual AHRS.
inline void arm_scale_f32(const float *a,float scale,float *out,uint32_t n) {
  for(uint32_t i=0;i<n;++i) out[i]=a[i]*scale;
}
inline void arm_add_f32(const float *a,const float *b,float *out,uint32_t n) {
  for(uint32_t i=0;i<n;++i) out[i]=a[i]+b[i];
}
inline void arm_quaternion_product_single_f32(const float *a,const float *b,float *out) {
  const float r[4]={a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
    a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
    a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
    a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
  for(int i=0;i<4;++i) out[i]=r[i];
}
inline int arm_atan2_f32(float y,float x,float *out) { *out=std::atan2(y,x); return 0; }
