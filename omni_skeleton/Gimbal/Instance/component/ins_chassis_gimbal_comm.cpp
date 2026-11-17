#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"

robot::GimbalChassisComm *GetGimbalChassisComm() {
  static robot::GimbalChassisComm comm(
      robot::GimbalChassisComm::CodePart::Gimbal, 0x211, 0x210);
  return &comm;
}

