#include "ins_pid.hpp"
#include "pid.hpp"
#include "robot_config.hpp"

hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  static hello_world::pid::Pid pids[4] = {
      {robot::config::kWheelKp, robot::config::kWheelKi, robot::config::kWheelKd,
       robot::config::kWheelIntegralLimit, robot::config::kWheelOutputLimit},
      {robot::config::kWheelKp, robot::config::kWheelKi, robot::config::kWheelKd,
       robot::config::kWheelIntegralLimit, robot::config::kWheelOutputLimit},
      {robot::config::kWheelKp, robot::config::kWheelKi, robot::config::kWheelKd,
       robot::config::kWheelIntegralLimit, robot::config::kWheelOutputLimit},
      {robot::config::kWheelKp, robot::config::kWheelKi, robot::config::kWheelKd,
       robot::config::kWheelIntegralLimit, robot::config::kWheelOutputLimit}};
  return idx >= 0 && idx < 4 ? &pids[idx] : nullptr;
}
