#ifndef HELLO_WORLD_PID_HPP_
#define HELLO_WORLD_PID_HPP_

#include <algorithm>

namespace hello_world::pid {
class Pid {
 public:
  Pid(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
      float integral_limit = 0.0f, float output_limit = 0.0f)
      : kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit), output_limit_(output_limit) {}
  void reset() { integral_ = 0.0f; last_error_ = 0.0f; initialized_ = false; }
  void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
  void setLimits(float integral_limit, float output_limit) { integral_limit_ = integral_limit; output_limit_ = output_limit; }
  float calculate(float error, float dt = 0.001f) {
    if (dt <= 0.0f) dt = 0.001f;
    integral_ += error * dt;
    if (integral_limit_ > 0.0f) integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    const float derivative = initialized_ ? (error - last_error_) / dt : 0.0f;
    initialized_ = true; last_error_ = error;
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    if (output_limit_ > 0.0f) output = std::clamp(output, -output_limit_, output_limit_);
    return output;
  }
  float calculate(float target, float measurement, float dt) { return calculate(target - measurement, dt); }
 private:
  float kp_, ki_, kd_, integral_limit_, output_limit_;
  float integral_ = 0.0f, last_error_ = 0.0f;
  bool initialized_ = false;
};
}  // namespace hello_world::pid

#endif
