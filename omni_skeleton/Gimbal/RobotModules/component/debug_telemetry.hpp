#ifndef GIMBAL_DEBUG_TELEMETRY_HPP_
#define GIMBAL_DEBUG_TELEMETRY_HPP_

#include <cstdint>

namespace robot {

// Read-only bench telemetry for Ozone. These values never participate in control.
struct GimbalDebugTelemetry {
  uint32_t tick_ms = 0;
  uint8_t life_state = 0;
  uint8_t commissioning_stage = 0;
  uint8_t right_switch = 0;
  uint8_t imu_ready = 0;
  uint8_t link_online = 0;
  uint8_t local_ready = 0;
  uint8_t peer_ready = 0;
  uint8_t remote_valid = 0;
  uint8_t armed = 0;
  uint8_t yaw_status = 0;
  uint8_t pitch_status = 0;
  uint8_t yaw_online = 0;
  uint8_t pitch_online = 0;
  uint32_t calibration_samples = 0;
  uint32_t convergence_samples = 0;

  float imu_roll = 0.0f;
  float imu_pitch = 0.0f;
  float imu_yaw = 0.0f;
  float imu_acc[3] = {};
  float imu_gyro[3] = {};
  float remote_lv = 0.0f;
  float remote_lh = 0.0f;
  float remote_wheel = 0.0f;
  float remote_rh = 0.0f;
  float remote_rv = 0.0f;
  float joint_yaw = 0.0f;
  float joint_pitch = 0.0f;
  float target_yaw = 0.0f;
  float target_pitch = 0.0f;
  float yaw_error = 0.0f;
  float pitch_error = 0.0f;
  float yaw_output = 0.0f;
  float pitch_output = 0.0f;
};

}  // namespace robot

extern volatile robot::GimbalDebugTelemetry g_gimbal_debug;

#endif  // GIMBAL_DEBUG_TELEMETRY_HPP_
