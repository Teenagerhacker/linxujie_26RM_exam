#include "gimbal_chassis_comm.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include "tick.hpp"
namespace {
void PutAngle(uint8_t *dst, float value) {
  if (!std::isfinite(value)) value = 0.0f;
  const auto raw = static_cast<uint16_t>(static_cast<int16_t>(std::lround(robot::NormalizeAngle(value)*1000.0f)));
  dst[0] = static_cast<uint8_t>(raw); dst[1] = static_cast<uint8_t>(raw >> 8);
}
float GetAngle(const uint8_t *src) {
  const unsigned raw = src[0] | (static_cast<unsigned>(src[1]) << 8);
  return (raw >= 32768 ? static_cast<int>(raw)-65536 : static_cast<int>(raw))/1000.0f;
}
uint8_t PutAxis(float value) {
  if (!std::isfinite(value)) value = 0.0f;
  return static_cast<uint8_t>(static_cast<int8_t>(std::lround(std::clamp(value,-1.0f,1.0f)*127)));
}
float GetAxis(uint8_t value) {
  return (value > 127 ? static_cast<int>(value)-256 : static_cast<int>(value))/127.0f;
}
}
robot::GimbalChassisComm::GimbalChassisComm(CodePart part, uint32_t tx, uint32_t rx)
    : code_part_(part), tx_id_(tx), rx_id_(rx) {}
void robot::GimbalChassisComm::encode(uint8_t d[8]) {
  if (!d) return;
  std::memset(d,0,8);
  if (code_part_ == CodePart::Chassis) {
    const auto &v = chassis_data_;
    d[0] = (v.ready ? 0x80:0) | (v.working ? 0x40:0) |
           (v.remote_valid ? 0x20:0) | (v.armed ? 0x10:0) |
           static_cast<uint8_t>(v.mode);
    d[1]=PutAxis(v.remote_lv); d[2]=PutAxis(v.remote_lh);
    d[3]=PutAxis(v.remote_wheel); d[4]=PutAxis(v.remote_rh);
    d[5]=PutAxis(v.remote_rv); d[6]=0xa1;
    d[7]=static_cast<uint8_t>(sequence_++);
  } else {
    const auto &v = gimbal_data_;
    d[0] = (v.ready ? 0x80:0) | (v.working ? 0x40:0);
    PutAngle(d+1,v.yaw); PutAngle(d+3,v.pitch);
    d[5]=0xb2; d[6]=static_cast<uint8_t>(sequence_);
    d[7]=static_cast<uint8_t>(sequence_>>8); ++sequence_;
  }
}
void robot::GimbalChassisComm::decode(const uint8_t d[8]) {
  if (!d) return;
  if (code_part_ == CodePart::Chassis) {
    if ((d[0]&0x3f) || ((d[0]&0x40) && !(d[0]&0x80)) || d[5]!=0xb2 ||
        std::fabs(GetAngle(d+1))>3.142f || std::fabs(GetAngle(d+3))>3.142f) return;
    GimbalData v;
    v.ready=d[0]&0x80; v.working=d[0]&0x40;
    v.yaw=GetAngle(d+1); v.pitch=GetAngle(d+3);
    gimbal_data_=v;
  } else {
    if ((d[0]&0x0c) || (d[0]&3)>2 ||
        ((d[0]&0x40) && !(d[0]&0x80)) ||
        ((d[0]&0x10) && !(d[0]&0x20)) || d[6]!=0xa1 ||
        d[1]==128 || d[2]==128 || d[3]==128 || d[4]==128 || d[5]==128) return;
    ChassisData v;
    v.ready=d[0]&0x80; v.working=d[0]&0x40;
    v.remote_valid=d[0]&0x20; v.armed=d[0]&0x10;
    v.mode=static_cast<ChassisMode>(d[0]&3);
    v.remote_lv=GetAxis(d[1]); v.remote_lh=GetAxis(d[2]);
    v.remote_wheel=GetAxis(d[3]); v.remote_rh=GetAxis(d[4]);
    v.remote_rv=GetAxis(d[5]);
    chassis_data_=v;
  }
  received_.update(hello_world::tick::GetTickMs());
}
bool robot::GimbalChassisComm::isOffline() const {
  return !received_.online(hello_world::tick::GetTickMs());
}
