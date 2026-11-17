#include "ins_motor.hpp"
#include "motor_DaMiao.hpp"
#include "robot_config.hpp"

hello_world::motor::Motor *GetMotorYaw() {
  static hello_world::motor::DM_J4310 motor;
  static bool initialized = false;
  if (!initialized) {
    hello_world::motor::OptionalParams opt;
    opt.input_type = hello_world::motor::InputType::kTorq;
    opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
    opt.angle_offset = robot::config::kYawOffset;
    opt.max_torq_input_lim = robot::config::kGimbalTorqueLimit;
    motor.init(2, opt, false, false);
    initialized = true;
  }
  return &motor;
}

hello_world::motor::Motor *GetMotorPitch() {
  static hello_world::motor::DM_J4310 motor;
  static bool initialized = false;
  if (!initialized) {
    hello_world::motor::OptionalParams opt;
    opt.input_type = hello_world::motor::InputType::kTorq;
    opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
    opt.dir = hello_world::motor::kDirRev;
    opt.angle_offset = robot::config::kPitchOffset;
    opt.max_torq_input_lim = robot::config::kGimbalTorqueLimit;
    motor.init(1, opt, false, false);
    initialized = true;
  }
  return &motor;
}
