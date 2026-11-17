#include "ins_motor.hpp"
#include "motor_M3508.hpp"
#include "robot_config.hpp"

hello_world::motor::Motor *GetMotorWheel(int idx) {
  static hello_world::motor::M3508 motors[4];
  static bool initialized = false;
  if (!initialized) {
    hello_world::motor::OptionalParams opt;
    opt.input_type = hello_world::motor::InputType::kRaw;
    opt.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf;
    opt.max_raw_input_lim = robot::config::kWheelRawLimit;
    opt.remove_build_in_reducer = true;
    opt.ex_redu_rat = robot::config::kReduction;
    for (int i = 0; i < 4; ++i) {
      opt.dir = robot::config::kWheelReversed[i] ?
          hello_world::motor::kDirRev : hello_world::motor::kDirFwd;
      motors[i].init(robot::config::kWheelIds[i], opt);
    }
    initialized = true;
  }
  return idx >= 0 && idx < 4 ? &motors[idx] : nullptr;
}
