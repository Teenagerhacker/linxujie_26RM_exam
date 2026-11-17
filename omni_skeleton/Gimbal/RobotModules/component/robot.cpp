#include "robot.hpp"
#include "DT7.hpp"
#include "debug_telemetry.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "irq_guard.hpp"
#include "tick.hpp"
namespace robot {
void Robot::notifyMotorFrame(int index) {
  if (index>=0 && index<2) motor_feedback_[index].update(hello_world::tick::GetTickMs());
}
void Robot::stopAll() {
  if (gimbal_) gimbal_->stop();
  if (yaw_) yaw_->setInput(0);
  if (pitch_) pitch_->setInput(0);
}
void Robot::update() {
  if (imu_) imu_->update();
  g_gimbal_debug.commissioning_stage=
      static_cast<uint8_t>(config::kCommissioningStage);
  g_gimbal_debug.imu_ready=imu_ && imu_->ready();
  g_gimbal_debug.imu_roll=imu_ ? imu_->roll():0.0f;
  g_gimbal_debug.imu_pitch=imu_ ? imu_->pitch():0.0f;
  g_gimbal_debug.imu_yaw=imu_ ? imu_->yaw():0.0f;
  g_gimbal_debug.calibration_samples=imu_ ? imu_->calibrationSamples():0;
  g_gimbal_debug.convergence_samples=imu_ ? imu_->convergenceSamples():0;
  for (int i=0;i<3;++i) {
    g_gimbal_debug.imu_acc[i]=imu_ ? imu_->acc(i):0.0f;
    g_gimbal_debug.imu_gyro[i]=imu_ ? imu_->gyro(i):0.0f;
  }
  if (!comm_) {
    stopAll(); state_=LifeState::Dead; g_gimbal_debug.life_state=0;
    g_gimbal_debug.link_online=0; return;
  }
  uint32_t now;
  bool remote_online,armed,imu_ready,local_ready,peer_ok,link;
  float rc_lv=0,rc_lh=0,rc_wheel=0,rc_rh=0,rc_rv=0;
  auto right_switch=hello_world::remote_control::kSwitchStateUp;
  GimbalChassisComm::ChassisData peer;
  {
    IrqGuard guard;
    now=hello_world::tick::GetTickMs();
    peer=comm_->chassis_data();
    link=!comm_->isOffline();
    remote_online=link && peer.remote_valid;
    armed=remote_online && peer.armed;
    if (remote_online) {
      switch(peer.mode) {
        case ChassisMode::Separate:
          right_switch=hello_world::remote_control::kSwitchStateUp; break;
        case ChassisMode::Follow:
          right_switch=hello_world::remote_control::kSwitchStateMid; break;
        default:
          right_switch=hello_world::remote_control::kSwitchStateDown; break;
      }
      rc_lv=peer.remote_lv; rc_lh=peer.remote_lh; rc_wheel=peer.remote_wheel;
      rc_rh=peer.remote_rh; rc_rv=peer.remote_rv;
    }
    imu_ready=imu_ && imu_->ready();
    g_gimbal_debug.yaw_online=motor_feedback_[0].online(now);
    g_gimbal_debug.pitch_online=motor_feedback_[1].online(now);
    const bool yaw_ready=!config::YawEnabled() ||
        (yaw_ && yaw_->status_code()==hello_world::motor::DaMiao::StatusCode::kMotorEnabled &&
         motor_feedback_[0].online(now));
    const bool pitch_ready=!config::PitchEnabled() ||
        (pitch_ && pitch_->status_code()==hello_world::motor::DaMiao::StatusCode::kMotorEnabled &&
         motor_feedback_[1].online(now));
    local_ready=imu_ready && gimbal_ && yaw_ready && pitch_ready;
    peer_ok=link && peer.ready;
  }
  g_gimbal_debug.tick_ms=now;
  g_gimbal_debug.life_state=static_cast<uint8_t>(state_);
  g_gimbal_debug.right_switch=static_cast<uint8_t>(right_switch);
  g_gimbal_debug.imu_ready=imu_ready;
  g_gimbal_debug.link_online=link;
  g_gimbal_debug.local_ready=local_ready;
  g_gimbal_debug.peer_ready=peer_ok;
  g_gimbal_debug.remote_valid=remote_online;
  g_gimbal_debug.armed=armed;
  g_gimbal_debug.remote_lv=rc_lv;
  g_gimbal_debug.remote_lh=rc_lh;
  g_gimbal_debug.remote_wheel=rc_wheel;
  g_gimbal_debug.remote_rh=rc_rh;
  g_gimbal_debug.remote_rv=rc_rv;
  g_gimbal_debug.yaw_status=yaw_ ? static_cast<uint8_t>(yaw_->status_code()):0;
  g_gimbal_debug.pitch_status=pitch_ ? static_cast<uint8_t>(pitch_->status_code()):0;
  auto &tx=comm_->gimbal_data();
  tx={}; tx.ready=local_ready;
  tx.yaw=yaw_ ? NormalizeAngle(yaw_->angle()):0;
  tx.pitch=pitch_ ? pitch_->angle():0;
  // DESIGN TRADE-OFF: disable the DM motors, rather than only commanding zero
  // torque, on emergency/link loss. Re-enabling is slower but cannot preserve
  // a pending actuator command through a communications fault.
  if (!armed || !peer_ok) {
    stopAll(); state_=LifeState::Dead; g_gimbal_debug.life_state=0;
    // Explicitly override any pending enable request on emergency/link loss.
    if (yaw_) yaw_->disable();
    if (pitch_) pitch_->disable();
    enable_attempted_=false;
    tx.ready=false;
    return;
  }
  if (!local_ready) {
    state_=LifeState::Resurrection; g_gimbal_debug.life_state=1; stopAll();
    // Enabling sends no torque. Do not wait for motor feedback before the first enable.
    if (imu_ready && (!enable_attempted_ || static_cast<uint32_t>(now-last_enable_tick_)>=100)) {
      if (config::YawEnabled() && yaw_ && (!motor_feedback_[0].online(now) ||
          yaw_->status_code()==hello_world::motor::DaMiao::StatusCode::kMotorDisabled)) yaw_->enable();
      if (config::PitchEnabled() && pitch_ && (!motor_feedback_[1].online(now) ||
          pitch_->status_code()==hello_world::motor::DaMiao::StatusCode::kMotorDisabled)) pitch_->enable();
      last_enable_tick_=now; enable_attempted_=true;
    }
    return;
  }
  gimbal_->setYawMeasurement(imu_->yaw());
  // DESIGN TRADE-OFF: align targets to measured pose before re-entering
  // Working. Recovery gains one zero-output cycle but has no return-to-old-
  // target kick after an emergency stop or link outage.
  if (state_!=LifeState::Working) {
    stopAll();
    gimbal_->setTarget(imu_->yaw(),pitch_->angle());
    state_=LifeState::Working; g_gimbal_debug.life_state=2;
    return;
  }
  g_gimbal_debug.life_state=2;
  auto target=gimbal_->target();
  if (config::YawEnabled())
    target.yaw=NormalizeAngle(target.yaw-Deadband(rc_rh)*config::kYawRate*config::kDt);
  if (config::PitchEnabled())
    target.pitch+=Deadband(rc_rv)*config::kPitchRate*config::kDt;
  gimbal_->setTarget(target.yaw,target.pitch);
  gimbal_->update();
  tx.working=true;
}
}
