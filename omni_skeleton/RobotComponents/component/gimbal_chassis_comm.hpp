#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_
#include <cstdint>
#include "module_state.hpp"
#include "control_math.hpp"
namespace robot {
class GimbalChassisComm {
 public:
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };
  struct ChassisData {
    bool ready = false;
    bool working = false;
    bool remote_valid = false;
    bool armed = false;
    ChassisMode mode = ChassisMode::Separate;
    float remote_lv = 0.0f;
    float remote_lh = 0.0f;
    float remote_wheel = 0.0f;
    float remote_rh = 0.0f;
    float remote_rv = 0.0f;
  };
  struct GimbalData {
    bool ready = false;
    bool working = false;
    float yaw = 0.0f;
    float pitch = 0.0f;
  };
  GimbalChassisComm(CodePart part, uint32_t tx_id, uint32_t rx_id);
  void encode(uint8_t data[8]);
  void decode(const uint8_t data[8]);
  bool isOffline() const;
  uint32_t tx_id() const { return tx_id_; }
  uint32_t rx_id() const { return rx_id_; }
  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }
 private:
  CodePart code_part_;
  uint32_t tx_id_, rx_id_;
  Freshness received_;
  uint16_t sequence_ = 0;
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
};
}
#endif
