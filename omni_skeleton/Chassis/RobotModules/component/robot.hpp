#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

#include <cstdint>

#include "module_state.hpp"
#include "control_math.hpp"
#include "motor_base.hpp"

namespace hello_world { namespace imu { class Imu; } namespace remote_control { class DT7; } }
namespace robot {
class Chassis;
class GimbalChassisComm;

class Robot {
 public:
  void registerChassis(Chassis *chassis) { chassis_ = chassis; }
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerRemote(hello_world::remote_control::DT7 *remote) { remote_ = remote; }
  void registerComm(GimbalChassisComm *comm) { comm_ = comm; }
  void registerMotor(int index, hello_world::motor::Motor *motor);
  void notifyRemoteFrame();
  void invalidateRemote() { remote_seen_=false; }
  void notifyMotorFrame(int index);
  void update();
  LifeState state() const { return state_; }

 private:
  void stopAll();
  Chassis *chassis_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  hello_world::remote_control::DT7 *remote_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;
  hello_world::motor::Motor *motors_[4] = {nullptr, nullptr, nullptr, nullptr};
  LifeState state_ = LifeState::Dead;
  ChassisMode mode_ = ChassisMode::Separate;
  uint32_t last_remote_tick_ = 0;
  bool remote_seen_ = false;
  Freshness motor_feedback_[4];
};
}  // namespace robot

#endif
