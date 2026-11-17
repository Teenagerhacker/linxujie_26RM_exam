#ifndef ROBOT_CONFIG_HPP_
#define ROBOT_CONFIG_HPP_
#include <cstdint>
namespace robot::config {
enum class CommissioningStage : uint8_t {
  SingleWheel = 0,
  Chassis = 1,
  Pitch = 2,
  Yaw = 3,
  Gimbal = 4,
  Full = 5,
};

// Change only this line after the current stage passes, then rebuild and flash BOTH boards.
inline constexpr CommissioningStage kHardwareCommissioningStage =
    CommissioningStage::Full;
#ifdef ROBOT_HOST_TEST
inline constexpr CommissioningStage kCommissioningStage = CommissioningStage::Full;
#else
inline constexpr CommissioningStage kCommissioningStage = kHardwareCommissioningStage;
#endif
inline constexpr int kSingleWheelIndex = 3;  // 0=LF, 1=LB, 2=RB, 3=RF.
static_assert(kSingleWheelIndex >= 0 && kSingleWheelIndex < 4,
              "kSingleWheelIndex must be 0..3");

inline constexpr bool ChassisEnabled() {
  return kCommissioningStage == CommissioningStage::SingleWheel ||
         kCommissioningStage == CommissioningStage::Chassis ||
         kCommissioningStage == CommissioningStage::Full;
}
inline constexpr bool YawEnabled() {
  return kCommissioningStage == CommissioningStage::Yaw ||
         kCommissioningStage == CommissioningStage::Gimbal ||
         kCommissioningStage == CommissioningStage::Full;
}
inline constexpr bool PitchEnabled() {
  return kCommissioningStage == CommissioningStage::Pitch ||
         kCommissioningStage == CommissioningStage::Gimbal ||
         kCommissioningStage == CommissioningStage::Full;
}

inline constexpr float kDt = 0.001f;
// DESIGN TRADE-OFF: 100 ms balances fast fail-safe shutdown against false
// offline trips from an occasional lost CAN/DBUS frame. It equals 100 control
// periods, so a single bad frame never causes a mode/output discontinuity.
inline constexpr uint32_t kTimeoutMs = 100;
inline constexpr bool kFeedWatchdog = true; // Set false ONLY for the watchdog reset demonstration.
inline constexpr float kWheelRadius = 0.07786f;
inline constexpr float kHalfWheelbase = 0.15338f;
inline constexpr float kHalfTrack = 0.15338f;
inline constexpr float kReduction = 3591.0f / 187.0f; // PDF; topology says 14: verify on hardware.
inline constexpr uint8_t kWheelIds[4] = {2, 3, 4, 1}; // LF, LB, RB, RF: 2026-11-15 wheel identification.
inline constexpr bool kWheelReversed[4] = {true, true, false, false}; // LF, LB, RB, RF.
inline constexpr float kMaxWheelSpeed = 35.0f;
inline constexpr float kMaxTranslation = 1.5f;
inline constexpr float kMaxRotation = 3.0f;
// DESIGN TRADE-OFF: simultaneous spin and translation loads all four drive
// motors. Ground tests exposed a whole-robot power/protection trip at the full
// limits, so Spin uses a lower power envelope while Separate/Follow retain the
// original top speed. Reliable control is preferred to an unusable peak speed.
inline constexpr float kSpinMaxTranslation = 1.0f;
inline constexpr float kSpinMaxRotation = 2.0f;
inline constexpr float kSingleWheelSpeed = 3.0f;       // rad/s at full left-stick input.
inline constexpr float kChassisTestTranslation = 0.25f; // m/s.
inline constexpr float kChassisTestRotation = 0.5f;     // rad/s.
// Ground test: gimbal-frame forward was transformed approximately pi radians backward.
inline constexpr float kYawOffset = 2.12100f;
inline constexpr float kPitchOffset = 0.02480f;
// DESIGN TRADE-OFF: the loaded lower hard stop was measured near -0.138 rad.
// Stop at -0.10 rad to retain about 0.038 rad (2.2 deg) mechanical margin;
// reduced downward travel is preferable to sustained torque into the stop.
inline constexpr float kPitchMin = -0.10f;
inline constexpr float kPitchMax = 0.45f;
inline constexpr float kYawRate = 1.5f;
inline constexpr float kPitchRate = 0.6f;
// delta = chassis heading - gimbal heading; encoder positive = gimbal CCW relative to chassis.
inline constexpr float kRelativeYawSign = -1.0f;

// Bench-tunable limits and gains. Keep these in one place so every test build is reproducible.
inline constexpr float kWheelRawLimit = 7000.0f; // Absolute driver limit.
inline constexpr float kSingleWheelRawLimit = 1200.0f;
inline constexpr float kChassisTestRawLimit = 2500.0f;
inline constexpr float ActiveWheelRawLimit() {
  return kCommissioningStage == CommissioningStage::SingleWheel
             ? kSingleWheelRawLimit
         : kCommissioningStage == CommissioningStage::Chassis
             ? kChassisTestRawLimit
             : kWheelRawLimit;
}
inline constexpr float kWheelKp = 700.0f;
inline constexpr float kWheelKi = 0.0f;
inline constexpr float kWheelKd = 2.0f;
inline constexpr float kWheelIntegralLimit = 10.0f;
inline constexpr float kWheelOutputLimit = 7000.0f;

// DESIGN TRADE-OFF: fixed gains are predictable and easier to validate than
// gain scheduling. We intentionally do not claim the variable-gain bonus item.
inline constexpr float kFollowKp = 3.0f;
inline constexpr float kFollowKi = 0.1f;
inline constexpr float kFollowKd = 0.0f;
inline constexpr float kFollowIntegralLimit = 0.5f;
inline constexpr float kFollowOutputLimit = 3.0f;

inline constexpr float kGimbalTorqueLimit = 4.0f; // Absolute driver limit.
inline constexpr float kGimbalTestTorqueLimit = 1.0f;
inline constexpr float ActiveGimbalTorqueLimit() {
  return kCommissioningStage == CommissioningStage::Full
             ? kGimbalTorqueLimit
             : kGimbalTestTorqueLimit;
}
inline constexpr float kYawKp = 8.0f;
inline constexpr float kYawKi = 0.0f;
// Extra derivative damping trades a small response-time increase for less
// overshoot in the measured yaw multi-pulse response.
inline constexpr float kYawKd = 0.10f;
inline constexpr float kYawIntegralLimit = 2.0f;
inline constexpr float kYawOutputLimit = 4.0f;
inline constexpr float kPitchKp = 8.0f;
// DESIGN TRADE-OFF: limited integral action learns gravity/static-friction
// torque that pure PD can only create through steady position error. The
// 0.25 rad*s clamp limits the integral contribution to 1 N*m (4 * 0.25),
// avoiding a large release kick after a long obstruction or target change.
inline constexpr float kPitchKi = 4.0f;
// Extra pitch damping absorbs the measured static-friction release instead of
// raising torque: the tuned response had a small 0.32 deg upward overshoot.
inline constexpr float kPitchKd = 0.20f;
inline constexpr float kPitchIntegralLimit = 0.25f;
inline constexpr float kPitchOutputLimit = 4.0f;
// DESIGN TRADE-OFF: yaw uses additional actuator-side velocity damping. Unlike
// derivative-on-error, it brakes real motion without kicking on stick changes;
// pitch retains the lower value validated by its no-overshoot response.
inline constexpr float kYawDmDamping = 0.30f;
inline constexpr float kPitchDmDamping = 0.15f;
} // namespace robot::config
#endif
