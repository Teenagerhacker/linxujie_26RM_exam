#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

#include <cstdint>
#include "module_state.hpp"
#include "control_math.hpp"
#include "motor_DaMiao.hpp"
#include "pid.hpp"

namespace hello_world { namespace imu { class Imu; } }
namespace robot {
class Gimbal;
class GimbalChassisComm;
class Robot {
 public:
  void registerGimbal(Gimbal *gimbal) { gimbal_ = gimbal; }
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerComm(GimbalChassisComm *comm) { comm_ = comm; }
  void registerMotor(hello_world::motor::DaMiao *yaw, hello_world::motor::DaMiao *pitch) { yaw_ = yaw; pitch_ = pitch; }
  void registerPid(hello_world::pid::Pid *yaw, hello_world::pid::Pid *pitch) { yaw_pid_ = yaw; pitch_pid_ = pitch; }
  void notifyMotorFrame(int index);
  void update();
  LifeState state() const { return state_; }

 private:
  void stopAll();
  Gimbal *gimbal_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;
  hello_world::motor::DaMiao *yaw_ = nullptr;
  hello_world::motor::DaMiao *pitch_ = nullptr;
  hello_world::pid::Pid *yaw_pid_ = nullptr;
  hello_world::pid::Pid *pitch_pid_ = nullptr;
  LifeState state_ = LifeState::Dead;
  Freshness motor_feedback_[2];
  uint32_t last_enable_tick_ = 0;
  bool enable_attempted_ = false;
};
}  // namespace robot

#endif
