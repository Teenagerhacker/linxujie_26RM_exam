#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"

robot::GimbalChassisComm *GetGimbalChassisComm() {
  static robot::GimbalChassisComm comm(
      robot::GimbalChassisComm::CodePart::Chassis, 0x210, 0x211);
  return &comm;
}

