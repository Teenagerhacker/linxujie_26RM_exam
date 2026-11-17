#ifndef CHASSIS_DEBUG_TELEMETRY_HPP_
#define CHASSIS_DEBUG_TELEMETRY_HPP_

#include <cstdint>

namespace robot {

// Read-only bench telemetry for Ozone. These values never participate in control.
struct ChassisDebugTelemetry {
  uint32_t tick_ms = 0;
  uint8_t life_state = 0;
  uint8_t commissioning_stage = 0;
  uint8_t mode = 0;
  uint8_t requested_mode = 0;
  uint8_t imu_ready = 0;
  uint8_t link_online = 0;
  uint8_t local_ready = 0;
  uint8_t peer_ready = 0;
  uint8_t remote_valid = 0;
  uint8_t armed = 0;
  uint8_t wheel_online[4] = {};
  uint32_t calibration_samples = 0;
  uint32_t convergence_samples = 0;
  // Latched whenever a running robot enters protection. Codes:
  // 1 board link, 2 remote timeout, 3 disarmed, 4 local IMU/module,
  // 5..8 wheel LF/LB/RB/RF, 9 peer not ready.
  uint32_t reset_cause = 0;
  uint32_t trip_tick_ms = 0;
  uint32_t trip_count = 0;
  uint8_t trip_reason = 0;
  uint8_t trip_reserved[3] = {};

  float imu_roll = 0.0f;
  float imu_pitch = 0.0f;
  float imu_yaw = 0.0f;
  float imu_acc[3] = {};
  float imu_gyro[3] = {};
  float remote_vx = 0.0f;
  float remote_vy = 0.0f;
  float remote_rotate = 0.0f;
  float command_vx = 0.0f;
  float command_vy = 0.0f;
  float command_w = 0.0f;
  float applied_vx = 0.0f;
  float applied_vy = 0.0f;
  float applied_w = 0.0f;
  float gimbal_yaw = 0.0f;
  float follow_error = 0.0f;
  float follow_output = 0.0f;
  float wheel_target[4] = {};
  float wheel_speed[4] = {};
  float wheel_error[4] = {};
  float wheel_output[4] = {};
  float wheel_angle[4] = {};
  float motor_rotor_turns[4] = {};
};

}  // namespace robot

extern volatile robot::ChassisDebugTelemetry g_chassis_debug;

#endif  // CHASSIS_DEBUG_TELEMETRY_HPP_
