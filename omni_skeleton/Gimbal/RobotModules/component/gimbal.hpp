#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

#include "module_state.hpp"
#include "motor_DaMiao.hpp"
#include "pid.hpp"

namespace robot {
class Gimbal {
 public:
  void registerMotor(hello_world::motor::DaMiao *yaw,
                    hello_world::motor::DaMiao *pitch) { yaw_ = yaw; pitch_ = pitch; }
  void registerPid(hello_world::pid::Pid *yaw, hello_world::pid::Pid *pitch) { yaw_pid_ = yaw; pitch_pid_ = pitch; }
  void setTarget(float yaw, float pitch) { target_yaw_ = yaw; target_pitch_ = pitch; }
  void setYawMeasurement(float yaw) { measured_yaw_=yaw; }
  GimbalMotion target() const { return {target_yaw_, target_pitch_}; }
  void update();
  void stop();

 private:
  hello_world::motor::DaMiao *yaw_ = nullptr;
  hello_world::motor::DaMiao *pitch_ = nullptr;
  hello_world::pid::Pid *yaw_pid_ = nullptr;
  hello_world::pid::Pid *pitch_pid_ = nullptr;
  float measured_yaw_ = 0.0f;
  float target_yaw_ = 0.0f;
  float target_pitch_ = 0.0f;
};
}  // namespace robot

#endif
