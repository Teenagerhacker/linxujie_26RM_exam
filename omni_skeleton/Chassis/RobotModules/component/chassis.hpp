#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

#include "module_state.hpp"
#include "motor_base.hpp"
#include "pid.hpp"
#include "robot_config.hpp"

namespace robot {
class Chassis {
 public:
  void registerMotor(int index, hello_world::motor::Motor *motor,
                     hello_world::pid::Pid *pid);
  void setMotion(const ChassisMotion &motion) { motion_ = motion; }
  void setMode(ChassisMode mode) {
    if (mode != mode_) { follow_pid_.reset(); mode_=mode; }
  }
  void setGimbalYaw(float yaw) { gimbal_yaw_ = yaw; }
  ChassisMode mode() const { return mode_; }
  const ChassisMotion &motion() const { return motion_; }
  void update();
  void stop();

 private:
  hello_world::motor::Motor *motors_[4] = {nullptr, nullptr, nullptr, nullptr};
  hello_world::pid::Pid *pids_[4] = {nullptr, nullptr, nullptr, nullptr};
  ChassisMotion motion_;
  ChassisMotion applied_;
  hello_world::pid::Pid follow_pid_{config::kFollowKp, config::kFollowKi,
      config::kFollowKd, config::kFollowIntegralLimit, config::kFollowOutputLimit};
  ChassisMode mode_ = ChassisMode::Separate;
  float gimbal_yaw_ = 0.0f;
};
}  // namespace robot

#endif
