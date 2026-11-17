#include "platform.hpp"
#include <iostream>
#include <limits>
#define HW_COMPONENTS_DEVICES_IMU_BMI088_HPP_
namespace hello_world::imu {
struct BMI088HWConfig {};
enum BMI088ErrState { kBMI088ErrStateNoErr=0, kTestInitError=1 };
float sample_acc[3]={0,0,9.80665f},sample_gyro[3]={0,0,0};
bool sensor_init_ok=true;
class BMI088 {
 public:
  void init(const BMI088HWConfig &,const float *) {}
  BMI088ErrState imuInit(bool) { return sensor_init_ok ? kBMI088ErrStateNoErr:kTestInitError; }
  void getData(float *acc,float *gyro,float *) {
    for(int i=0;i<3;++i) { acc[i]=sample_acc[i]; gyro[i]=sample_gyro[i]; }
  }
};
}
#include "../Chassis/inc/imu.hpp"
using namespace hello_world::imu;
static void Init(Imu &imu) {
  const float identity[9]={1,0,0,0,1,0,0,0,1};
  imu.init({},identity);
}
static void Flat() {
  sample_acc[0]=0; sample_acc[1]=0; sample_acc[2]=9.80665f;
  for(float &v:sample_gyro) v=0;
}
static void Update(Imu &imu,int n) { for(int i=0;i<n;++i) imu.update(); }
int main() {
  Flat(); sample_gyro[2]=0.02f;
  Imu imu; Init(imu); assert(imu.initialized() && !imu.ready());
  Update(imu,1498); assert(!imu.ready()); imu.update(); assert(imu.ready());
  assert(std::fabs(imu.yaw())<0.001f); // The measured stationary bias is removed.
  sample_gyro[2]=0.52f; Update(imu,1000);
  assert(imu.ready() && std::fabs(imu.yaw()-0.5f)<0.02f);
  sample_acc[0]=std::numeric_limits<float>::quiet_NaN();
  Update(imu,19); assert(imu.ready());
  Flat(); imu.update(); assert(imu.ready()); // A good sample clears the bad-sample streak.
  sample_acc[0]=std::numeric_limits<float>::quiet_NaN();
  Update(imu,19); assert(imu.ready()); imu.update(); assert(!imu.ready());
  Flat(); Update(imu,1499); assert(imu.ready());
  for(float &v:sample_acc) v=0;
  Update(imu,19); assert(imu.ready()); imu.update(); assert(!imu.ready());
  Flat(); Update(imu,1499); assert(imu.ready());
  sample_gyro[0]=31; Update(imu,20); assert(!imu.ready());
  Flat(); Imu moved; Init(moved); Update(moved,800);
  sample_gyro[0]=0.4f; moved.update(); Flat(); Update(moved,1498); assert(!moved.ready());
  moved.update(); assert(moved.ready());
  Imu noisy; Init(noisy);
  for(int i=0;i<2000;++i) { sample_gyro[0]=(i%2 ? 0.03f:-0.03f); noisy.update(); }
  assert(!noisy.ready());
  Flat(); Imu tilted; Init(tilted);
  sample_acc[1]=9.80665f*std::sin(0.4f); sample_acc[2]=9.80665f*std::cos(0.4f);
  Update(tilted,1499); assert(!tilted.ready()); // Sample count alone is insufficient.
  Update(tilted,10000); assert(tilted.ready() && std::fabs(tilted.roll()-0.4f)<0.02f);
  Flat(); sensor_init_ok=false; Imu failed; Init(failed); Update(failed,20000); assert(!failed.ready());
  std::cout<<"PASS actual IMU/Mahony calibration, bias, convergence, movement, variance, invalid samples, init failure\n";
}
