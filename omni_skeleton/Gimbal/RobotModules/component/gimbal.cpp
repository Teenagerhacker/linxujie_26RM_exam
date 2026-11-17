#include "gimbal.hpp"
#include <algorithm>
#include <cmath>
#include "debug_telemetry.hpp"
#include "robot_config.hpp"

volatile robot::GimbalDebugTelemetry g_gimbal_debug;

namespace robot {
void Gimbal::update() {
  if (!yaw_ || !pitch_) return;
  target_yaw_=NormalizeAngle(target_yaw_);
  // Keep the commanded upper position below the torque cutoff to avoid
  // repeated cutoff/restart as feedback crosses the safety boundary.
  target_pitch_=std::clamp(target_pitch_,config::kPitchMin,config::kPitchMax-0.02f);
  // DESIGN TRADE-OFF: close yaw on world angle from the gimbal IMU instead of
  // joint angle. This rejects chassis spin, at the cost of IMU calibration and
  // drift dependence; the joint angle is still sent to chassis coordinates.
  const float yaw_error=NormalizeAngle(target_yaw_-measured_yaw_);
  const float pitch_error=target_pitch_-pitch_->angle();
  float yaw_torque=config::YawEnabled() && yaw_pid_ ?
      yaw_pid_->calculate(yaw_error,config::kDt):0.0f;
  float pitch_torque=config::PitchEnabled() && pitch_pid_ ?
      pitch_pid_->calculate(pitch_error,config::kDt):0.0f;
  yaw_torque=std::clamp(yaw_torque,-config::ActiveGimbalTorqueLimit(),
                        config::ActiveGimbalTorqueLimit());
  pitch_torque=std::clamp(pitch_torque,-config::ActiveGimbalTorqueLimit(),
                          config::ActiveGimbalTorqueLimit());
  // DESIGN TRADE-OFF: clamp both the target and outward torque. This gives up
  // some edge travel to avoid sustained torque into a mechanical hard stop.
  if ((pitch_->angle()>=config::kPitchMax && pitch_torque>0) ||
      (pitch_->angle()<=config::kPitchMin && pitch_torque<0)) pitch_torque=0;
  if (!std::isfinite(yaw_torque) || !std::isfinite(pitch_torque)) { stop(); return; }
  g_gimbal_debug.joint_yaw=yaw_->angle();
  g_gimbal_debug.joint_pitch=pitch_->angle();
  g_gimbal_debug.target_yaw=target_yaw_;
  g_gimbal_debug.target_pitch=target_pitch_;
  g_gimbal_debug.yaw_error=yaw_error;
  g_gimbal_debug.pitch_error=pitch_error;
  g_gimbal_debug.yaw_output=yaw_torque;
  g_gimbal_debug.pitch_output=pitch_torque;
  // DESIGN TRADE-OFF: kp=0 avoids a DM internal position-loop discontinuity
  // across +/-pi. kd supplies local damping while the software PID owns the
  // shortest-angle rule and bounded torque; this is safer but needs outer-loop tuning.
  if (config::YawEnabled())
    yaw_->setMitInput(0,0,0,config::kYawDmDamping,yaw_torque);
  else {
    yaw_->setInput(0);
    if (yaw_pid_) yaw_pid_->reset();
  }
  if (config::PitchEnabled())
    pitch_->setMitInput(0,0,0,config::kPitchDmDamping,pitch_torque);
  else {
    pitch_->setInput(0);
    if (pitch_pid_) pitch_pid_->reset();
  }
}
void Gimbal::stop() {
  if (yaw_) yaw_->setInput(0);
  if (pitch_) pitch_->setInput(0);
  if (yaw_) g_gimbal_debug.joint_yaw=yaw_->angle();
  if (pitch_) g_gimbal_debug.joint_pitch=pitch_->angle();
  g_gimbal_debug.target_yaw=target_yaw_;
  g_gimbal_debug.target_pitch=target_pitch_;
  g_gimbal_debug.yaw_error=0.0f;
  g_gimbal_debug.pitch_error=0.0f;
  g_gimbal_debug.yaw_output=0.0f;
  g_gimbal_debug.pitch_output=0.0f;
  if (yaw_pid_) yaw_pid_->reset();
  if (pitch_pid_) pitch_pid_->reset();
}
}
