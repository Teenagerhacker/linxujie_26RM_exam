#include "chassis.hpp"
#include "control_math.hpp"
#include "debug_telemetry.hpp"
#include <array>

volatile robot::ChassisDebugTelemetry g_chassis_debug;

namespace robot {
void Chassis::registerMotor(int index, hello_world::motor::Motor *motor,
                            hello_world::pid::Pid *pid) {
  if (index>=0 && index<4) { motors_[index]=motor; pids_[index]=pid; }
}
void Chassis::update() {
  if (!config::ChassisEnabled()) { stop(); return; }
  float w=motion_.w;
  g_chassis_debug.command_vx=motion_.vx;
  g_chassis_debug.command_vy=motion_.vy;
  g_chassis_debug.command_w=motion_.w;
  g_chassis_debug.gimbal_yaw=gimbal_yaw_;
  g_chassis_debug.follow_error=0.0f;
  g_chassis_debug.follow_output=0.0f;
  // delta = body - gimbal. Positive yaw encoder -> negative delta -> positive follow rotation.
  if (mode_==ChassisMode::Follow) {
    const float delta=NormalizeAngle(gimbal_yaw_);
    // DESIGN TRADE-OFF: the 0.015 rad (0.86 deg) dead zone accepts a small
    // centering error to avoid hunting on encoder noise and mechanical play.
    w=std::fabs(delta)<0.015f ? 0.0f : -follow_pid_.calculate(delta,config::kDt);
    g_chassis_debug.follow_error=delta;
    g_chassis_debug.follow_output=w;
  } else follow_pid_.reset();
  // DESIGN TRADE-OFF: normal commands use acceleration shaping for smooth mode
  // changes, but zero/emergency commands bypass the ramp for fail-safe stopping.
  // Zero torque can leave a short inertial coast; active reverse braking was
  // rejected because it can kick the chassis and increases tuning risk.
  // Follow mode may still rotate on a nonzero heading error, as explicitly selected by the operator.
  if (motion_.vx==0.0f && motion_.vy==0.0f && w==0.0f) { stop(); return; }
  applied_.vx=motion_.vx==0.0f ? 0.0f : Approach(applied_.vx,motion_.vx,2.0f*config::kDt);
  applied_.vy=motion_.vy==0.0f ? 0.0f : Approach(applied_.vy,motion_.vy,2.0f*config::kDt);
  applied_.w=w==0.0f ? 0.0f : Approach(applied_.w,std::clamp(w,-config::kMaxRotation,config::kMaxRotation),6.0f*config::kDt);
  g_chassis_debug.applied_vx=applied_.vx;
  g_chassis_debug.applied_vy=applied_.vy;
  g_chassis_debug.applied_w=applied_.w;
  std::array<float,4> wheel{};
  if (config::kCommissioningStage==config::CommissioningStage::SingleWheel) {
    const float normalized=std::clamp(motion_.vx/config::kMaxTranslation,-1.0f,1.0f);
    wheel[config::kSingleWheelIndex]=normalized*config::kSingleWheelSpeed;
  } else {
    wheel=WheelTargets(applied_,gimbal_yaw_);
  }
  for (int i=0;i<4;++i) {
    if (!motors_[i]) continue;
    if (config::kCommissioningStage==config::CommissioningStage::SingleWheel &&
        i!=config::kSingleWheelIndex) {
      motors_[i]->setInput(0.0f);
      if (pids_[i]) pids_[i]->reset();
      g_chassis_debug.wheel_target[i]=0.0f;
      g_chassis_debug.wheel_speed[i]=motors_[i]->vel();
      g_chassis_debug.wheel_error[i]=0.0f;
      g_chassis_debug.wheel_output[i]=0.0f;
      continue;
    }
    const float speed=motors_[i]->vel();
    const float raw=pids_[i] ? pids_[i]->calculate(wheel[i],speed,config::kDt) : 0.0f;
    const float output=std::isfinite(raw) ?
        std::clamp(raw,-config::ActiveWheelRawLimit(),config::ActiveWheelRawLimit()) : 0.0f;
    motors_[i]->setInput(output);
    g_chassis_debug.wheel_target[i]=wheel[i];
    g_chassis_debug.wheel_speed[i]=speed;
    g_chassis_debug.wheel_error[i]=wheel[i]-speed;
    g_chassis_debug.wheel_output[i]=motors_[i]->getInput();
    g_chassis_debug.wheel_angle[i]=motors_[i]->angle();
    g_chassis_debug.motor_rotor_turns[i]=motors_[i]->angle()*
        motors_[i]->motor_info().redu_rat/(2.0f*3.14159265358979323846f);
  }
}
void Chassis::stop() {
  motion_={}; applied_={}; follow_pid_.reset();
  g_chassis_debug.command_vx=0.0f;
  g_chassis_debug.command_vy=0.0f;
  g_chassis_debug.command_w=0.0f;
  g_chassis_debug.applied_vx=0.0f;
  g_chassis_debug.applied_vy=0.0f;
  g_chassis_debug.applied_w=0.0f;
  g_chassis_debug.follow_error=0.0f;
  g_chassis_debug.follow_output=0.0f;
  for (int i=0;i<4;++i) {
    if (motors_[i]) {
      motors_[i]->setInput(0.0f);
      g_chassis_debug.wheel_speed[i]=motors_[i]->vel();
      g_chassis_debug.wheel_angle[i]=motors_[i]->angle();
      g_chassis_debug.motor_rotor_turns[i]=motors_[i]->angle()*
          motors_[i]->motor_info().redu_rat/(2.0f*3.14159265358979323846f);
    }
    g_chassis_debug.wheel_target[i]=0.0f;
    g_chassis_debug.wheel_error[i]=0.0f;
    g_chassis_debug.wheel_output[i]=0.0f;
    if (pids_[i]) pids_[i]->reset();
  }
}
}
