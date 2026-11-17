#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#include <cmath>
#include <cstdint>
#include "BMI088.hpp"
#include "mahony.hpp"
namespace hello_world::imu {
class Imu {
 public:
  void init(const BMI088HWConfig &hw, const float rotation[9], float frequency=1000.0f) {
    bmi_.init(hw,rotation);
    initialized_=true;
    init_ok_=bmi_.imuInit(false)==kBMI088ErrStateNoErr;
    frequency_=frequency;
    resetCalibration();
  }
  bool initialized() const { return initialized_; }
  bool ready() const { return init_ok_ && ready_; }
  void update() {
    if (!init_ok_) return;
    float acc[3]={}, gyro[3]={};
    bmi_.getData(acc,gyro,nullptr);
    for (int i=0;i<3;++i) { last_acc_[i]=acc[i]; last_gyro_[i]=gyro[i]; }
    float acc2=0,gyro2=0;
    for (int i=0;i<3;++i) {
      if (!std::isfinite(acc[i]) || !std::isfinite(gyro[i])) { rejectSample(); return; }
      acc2+=acc[i]*acc[i]; gyro2+=gyro[i]*gyro[i];
    }
    // Reject absent/saturated/corrupt samples. Movement is allowed AFTER convergence.
    if (acc2<1.0f || acc2>900.0f || gyro2>900.0f) { rejectSample(); return; }
    bad_sample_count_=0;
    // BMI088 zero-rate noise on a powered chassis can exceed 0.1 rad/s in
    // magnitude briefly. Keep the movement gate above that noise floor while
    // the variance check below still rejects an unstable calibration window.
    const bool stationary=std::fabs(std::sqrt(acc2)-9.80665f)<0.6f && gyro2<0.04f;
    if (!bias_ready_) {
      if (!stationary) { resetCalibration(); return; }
      // Consecutive stationary samples, with a per-axis variance bound.
      for (int i=0;i<3;++i) { sum_[i]+=gyro[i]; sum2_[i]+=gyro[i]*gyro[i]; }
      if (++sample_count_<1000) return;
      for (int i=0;i<3;++i) {
        bias_[i]=sum_[i]/sample_count_;
        if (sum2_[i]/sample_count_-bias_[i]*bias_[i]>0.0004f) { resetCalibration(); return; }
      }
      bias_ready_=true;
    }
    for (int i=0;i<3;++i) gyro[i]-=bias_[i];
    mahony_.update(acc,gyro);
    mahony_.getEulerAngle(euler_);
    for (float v:euler_) if (!std::isfinite(v)) { resetCalibration(); return; }
    if (!ready_) {
      if (!stationary) { resetCalibration(); return; }
      float q[4]; mahony_.getQuat(q);
      // Predicted unit gravity in sensor/body axes; readiness requires actual agreement.
      const float predicted[3]={2*(q[1]*q[3]-q[0]*q[2]),
                               2*(q[2]*q[3]+q[0]*q[1]),
                               1-2*(q[1]*q[1]+q[2]*q[2])};
      const float norm=std::sqrt(acc2);
      float residual=0;
      for(int i=0;i<3;++i) {
        const float error=predicted[i]-acc[i]/norm;
        residual+=error*error;
      }
      convergence_count_=residual<0.0004f ? convergence_count_+1:0;
      if (convergence_count_>=500) ready_=true;
    }
  }
  float roll() const { return euler_[0]; }
  float pitch() const { return euler_[1]; }
  float yaw() const { return euler_[2]; }
  float acc(int axis) const { return axis>=0 && axis<3 ? last_acc_[axis]:0.0f; }
  float gyro(int axis) const { return axis>=0 && axis<3 ? last_gyro_[axis]:0.0f; }
  uint32_t calibrationSamples() const { return sample_count_; }
  uint32_t convergenceSamples() const { return convergence_count_; }
 private:
  void rejectSample() {
    // DESIGN TRADE-OFF: chassis vibration can corrupt an isolated SPI/IMU
    // sample. Keeping the last valid attitude for less than 20 ms prevents a
    // nuisance 1.5 s recalibration stop, while persistent corruption still
    // withdraws readiness well inside the 100 ms system timeout.
    if (!ready_ || ++bad_sample_count_>=20) resetCalibration();
  }
  void resetCalibration() {
    ready_=false; bias_ready_=false; sample_count_=0; convergence_count_=0;
    bad_sample_count_=0;
    for(int i=0;i<3;++i) { sum_[i]=0; sum2_[i]=0; bias_[i]=0; euler_[i]=0; }
    mahony_.init(frequency_,2.0f,0.0f);
  }
  BMI088 bmi_;
  ahrs::Mahony mahony_;
  float frequency_=1000;
  float sum_[3]={},sum2_[3]={},bias_[3]={},euler_[3]={};
  float last_acc_[3]={},last_gyro_[3]={};
  uint32_t sample_count_=0,convergence_count_=0,bad_sample_count_=0;
  bool initialized_=false,init_ok_=false,bias_ready_=false,ready_=false;
};
}
#endif
