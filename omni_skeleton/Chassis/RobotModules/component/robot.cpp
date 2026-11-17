#include "robot.hpp"
#include "chassis.hpp"
#include "DT7.hpp"
#include "debug_telemetry.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "irq_guard.hpp"
#include "tick.hpp"
namespace robot {
void Robot::registerMotor(int index, hello_world::motor::Motor *motor) {
  if (index>=0 && index<4) motors_[index]=motor;
}
void Robot::notifyRemoteFrame() {
  remote_seen_=true; last_remote_tick_=hello_world::tick::GetTickMs();
}
void Robot::notifyMotorFrame(int index) {
  if (index>=0 && index<4) motor_feedback_[index].update(hello_world::tick::GetTickMs());
}
void Robot::stopAll() {
  if (chassis_) chassis_->stop();
  for (auto *motor : motors_) if (motor) motor->setInput(0.0f);
}
void Robot::update() {
  if (imu_) imu_->update(); // SPI/delays must run with interrupts enabled.
  g_chassis_debug.commissioning_stage=
      static_cast<uint8_t>(config::kCommissioningStage);
  g_chassis_debug.imu_ready=imu_ && imu_->ready();
  g_chassis_debug.imu_roll=imu_ ? imu_->roll():0.0f;
  g_chassis_debug.imu_pitch=imu_ ? imu_->pitch():0.0f;
  g_chassis_debug.imu_yaw=imu_ ? imu_->yaw():0.0f;
  g_chassis_debug.calibration_samples=imu_ ? imu_->calibrationSamples():0;
  g_chassis_debug.convergence_samples=imu_ ? imu_->convergenceSamples():0;
  for (int i=0;i<3;++i) {
    g_chassis_debug.imu_acc[i]=imu_ ? imu_->acc(i):0.0f;
    g_chassis_debug.imu_gyro[i]=imu_ ? imu_->gyro(i):0.0f;
  }
  if (!comm_) {
    state_=LifeState::Dead; g_chassis_debug.life_state=0;
    g_chassis_debug.link_online=0; stopAll(); return;
  }
  uint32_t now;
  bool local_ready,link,remote_online,armed;
  float rc_lv=0,rc_lh=0,rc_wheel=0,rc_rh=0,rc_rv=0;
  ChassisMode requested_mode=ChassisMode::Separate;
  GimbalChassisComm::GimbalData peer;
  {
    IrqGuard guard;
    now=hello_world::tick::GetTickMs();
    remote_online=remote_ && remote_seen_ &&
        static_cast<uint32_t>(now-last_remote_tick_)<=config::kTimeoutMs;
    armed=remote_online &&
        remote_->rc_l_switch()!=hello_world::remote_control::kSwitchStateDown;
    if (remote_online) {
      rc_lv=remote_->rc_lv(); rc_lh=remote_->rc_lh();
      rc_wheel=remote_->rc_wheel(); rc_rh=remote_->rc_rh(); rc_rv=remote_->rc_rv();
      switch(remote_->rc_r_switch()) {
        case hello_world::remote_control::kSwitchStateUp:
          requested_mode=ChassisMode::Separate; break;
        case hello_world::remote_control::kSwitchStateMid:
          requested_mode=ChassisMode::Follow; break;
        default:
          requested_mode=ChassisMode::Spin; break;
      }
    }
    local_ready=imu_ && imu_->ready() && chassis_;
    for (int i=0;i<4;++i)
      g_chassis_debug.wheel_online[i]=motor_feedback_[i].online(now);
    if (config::kCommissioningStage==config::CommissioningStage::SingleWheel) {
      const int i=config::kSingleWheelIndex;
      local_ready=local_ready && motors_[i] && motor_feedback_[i].online(now);
    } else if (config::kCommissioningStage==config::CommissioningStage::Chassis ||
               config::kCommissioningStage==config::CommissioningStage::Full) {
      for (int i=0;i<4;++i)
        local_ready=local_ready && motors_[i] && motor_feedback_[i].online(now);
    }
    peer=comm_->gimbal_data();
    link=!comm_->isOffline();
  }
  g_chassis_debug.tick_ms=now;
  g_chassis_debug.link_online=link;
  g_chassis_debug.local_ready=local_ready;
  g_chassis_debug.peer_ready=peer.ready;
  g_chassis_debug.remote_valid=remote_online;
  g_chassis_debug.armed=armed;
  g_chassis_debug.requested_mode=static_cast<uint8_t>(requested_mode);
  auto latch_trip=[&](uint8_t reason) {
    if (state_==LifeState::Working) {
      g_chassis_debug.trip_reason=reason;
      g_chassis_debug.trip_tick_ms=now;
      g_chassis_debug.trip_count=g_chassis_debug.trip_count+1u;
    }
  };
  const float remote_vx=Deadband(rc_lv);
  // DBUS horizontal stick is positive to the operator's right, while the
  // required gimbal-frame y axis is positive to the gun's left.
  const float remote_vy=-Deadband(rc_lh);
  const float remote_rotate=-Deadband(rc_wheel);
  g_chassis_debug.remote_vx=remote_vx;
  g_chassis_debug.remote_vy=remote_vy;
  g_chassis_debug.remote_rotate=remote_rotate;
  auto &tx=comm_->chassis_data();
  tx={}; tx.ready=local_ready && link; tx.working=false;
  tx.remote_valid=remote_online; tx.armed=armed; tx.mode=requested_mode;
  tx.remote_lv=rc_lv; tx.remote_lh=rc_lh; tx.remote_wheel=rc_wheel;
  tx.remote_rh=rc_rh; tx.remote_rv=rc_rv;
  // DESIGN TRADE-OFF: safety events bypass all smoothing and immediately clear
  // outputs. This prioritizes bounded energy over a visually smooth stop.
  if (!link || !remote_online || !armed) {
    latch_trip(!link ? 1 : (!remote_online ? 2 : 3));
    state_=LifeState::Dead; g_chassis_debug.life_state=0; stopAll(); return;
  }
  // DESIGN TRADE-OFF: both boards must advertise readiness. The reciprocal
  // gate adds startup/recovery latency, but prevents either board operating
  // alone and propagates a one-sided fault into whole-robot protection.
  if (!local_ready || !peer.ready) {
    uint8_t reason=9;
    if (!local_ready) {
      reason=4;
      for (int i=0;i<4;++i) {
        if (!g_chassis_debug.wheel_online[i]) { reason=static_cast<uint8_t>(5+i); break; }
      }
    }
    latch_trip(reason);
    state_=LifeState::Resurrection; g_chassis_debug.life_state=1; stopAll(); return;
  }
  // DESIGN TRADE-OFF: keep one complete control cycle at zero when entering
  // Working. The extra 1 ms is negligible and blocks stale-command jumps.
  if (state_!=LifeState::Working) {
    stopAll(); state_=LifeState::Working; g_chassis_debug.life_state=2; return;
  }
  g_chassis_debug.life_state=2;
  mode_=config::kCommissioningStage==config::CommissioningStage::Full ?
      requested_mode : ChassisMode::Separate;
  g_chassis_debug.mode=static_cast<uint8_t>(mode_);
  chassis_->setMode(mode_);
  chassis_->setGimbalYaw(NormalizeAngle(config::kRelativeYawSign*peer.yaw));
  if (config::kCommissioningStage==config::CommissioningStage::SingleWheel) {
    chassis_->setMotion({config::kMaxTranslation*remote_vx,0.0f,0.0f});
  } else if (config::kCommissioningStage==config::CommissioningStage::Chassis) {
    chassis_->setMotion({config::kChassisTestTranslation*remote_vx,
                         config::kChassisTestTranslation*remote_vy,
                         config::kChassisTestRotation*remote_rotate});
  } else if (config::kCommissioningStage==config::CommissioningStage::Full) {
    const bool spin=requested_mode==ChassisMode::Spin;
    const float translation_limit=spin ? config::kSpinMaxTranslation
                                       : config::kMaxTranslation;
    const float rotation_limit=spin ? config::kSpinMaxRotation
                                    : config::kMaxRotation;
    chassis_->setMotion({translation_limit*remote_vx,
                         translation_limit*remote_vy,
                         rotation_limit*remote_rotate});
  } else {
    chassis_->setMotion({});
  }
  chassis_->update();
  tx.working=true;
}
}
