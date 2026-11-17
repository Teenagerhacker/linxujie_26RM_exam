#include "ins_pid.hpp"
#include "pid.hpp"
#include "robot_config.hpp"

hello_world::pid::Pid *GetPidMotorYaw() {
  static hello_world::pid::Pid pid(
      robot::config::kYawKp, robot::config::kYawKi, robot::config::kYawKd,
      robot::config::kYawIntegralLimit, robot::config::kYawOutputLimit);
  return &pid;
}

hello_world::pid::Pid *GetPidMotorPitch() {
  static hello_world::pid::Pid pid(
      robot::config::kPitchKp, robot::config::kPitchKi, robot::config::kPitchKd,
      robot::config::kPitchIntegralLimit, robot::config::kPitchOutputLimit);
  return &pid;
}
