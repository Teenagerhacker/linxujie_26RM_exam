#include <array>
#include <iostream>
#include <limits>
#include "platform.hpp"
#include "control_math.hpp"
#include "gimbal_chassis_comm.hpp"
#include "motor_M3508.hpp"
#include "motor_DaMiao.hpp"
#include "DT7.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"

// Only hardware-facing IMU and interrupt masking are replaced in this executable.
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
namespace hello_world::imu {
class Imu {
 public:
  bool valid=true;
  float heading=0;
  void update() {}
  bool ready() const { return valid; }
  float roll() const { return 0.0f; }
  float pitch() const { return 0.0f; }
  float yaw() const { return heading; }
  float acc(int) const { return 0.0f; }
  float gyro(int) const { return 0.0f; }
  uint32_t calibrationSamples() const { return valid ? 1000u : 0u; }
  uint32_t convergenceSamples() const { return valid ? 500u : 0u; }
};
}
#define ROBOT_IRQ_GUARD_HPP_
namespace robot { class IrqGuard { public: ~IrqGuard() {} }; }
#define Robot ChassisRobot
#include "../Chassis/RobotModules/component/robot.hpp"
#include "../Chassis/RobotModules/component/robot.cpp"
#undef Robot
#define Robot GimbalRobot
#include "../Gimbal/RobotModules/component/robot.hpp"
#include "../Gimbal/RobotModules/component/robot.cpp"
#undef Robot

uint32_t now=0;
namespace hello_world::tick { uint32_t GetTickMs() { return now; } }
using namespace robot;
using namespace hello_world::motor;
using namespace hello_world::remote_control;
static void Near(float a,float b,float eps=0.0001f) {
  if (std::fabs(a-b)>eps)
    std::cerr<<"Near failed: actual="<<a<<" expected="<<b<<" eps="<<eps<<"\n";
  assert(std::fabs(a-b)<=eps);
}
static std::array<uint8_t,18> Dbus(int left=1,int right=1,
    int rh=1024,int rv=1024,int lh=1024,int lv=1024,int wheel=1024) {
  std::array<uint8_t,18> d{};
  d[0]=rh; d[1]=(rh>>8)|(rv<<3); d[2]=(rv>>5)|(lh<<6);
  d[3]=lh>>2; d[4]=(lh>>10)|(lv<<1);
  d[5]=(lv>>7)|(right<<4)|(left<<6);
  d[16]=wheel; d[17]=wheel>>8;
  return d;
}
static void Feedback(DM_J4310 &motor,int id,int status=1,int velocity=2048) {
  const uint8_t d[8]={static_cast<uint8_t>((status<<4)|id),0x7f,0xff,
      static_cast<uint8_t>(velocity>>4),static_cast<uint8_t>((velocity<<4)|7),0xff,30,30};
  assert(motor.decode(d));
}

static void ProtocolAndMath() {
  GimbalChassisComm c(GimbalChassisComm::CodePart::Chassis,0x210,0x211);
  GimbalChassisComm g(GimbalChassisComm::CodePart::Gimbal,0x211,0x210);
  now=0; assert(c.isOffline() && g.isOffline());
  auto &v=c.chassis_data();
  v.ready=true; v.working=true; v.remote_valid=true; v.armed=true;
  v.mode=ChassisMode::Spin; v.remote_lv=0.37f; v.remote_lh=-0.7f;
  v.remote_wheel=1; v.remote_rh=-0.23f; v.remote_rv=0.45f;
  uint8_t d[8]; c.encode(d); g.decode(d);
  assert(!g.isOffline());
  Near(g.chassis_data().remote_lv,v.remote_lv,1.0f/127);
  Near(g.chassis_data().remote_lh,v.remote_lh,1.0f/127);
  Near(g.chassis_data().remote_rh,v.remote_rh,1.0f/127);
  assert(g.chassis_data().mode==ChassisMode::Spin && g.chassis_data().armed);
  for (int kind=0;kind<6;++kind) {
    now=0; c.encode(d); g.decode(d); now=50;
    if(kind==0) d[0]|=4;
    if(kind==1) d[0]|=3;
    if(kind==2) d[1]=128;
    if(kind==3) d[0]&=~0x80;
    if(kind==4) d[0]&=~0x20;
    if(kind==5) d[6]=0;
    g.decode(d); now=100; assert(!g.isOffline()); now=101; assert(g.isOffline());
  }
  now=500; g.gimbal_data()={true,true,2.4f,-0.4f};
  g.encode(d); assert(d[5]==0xb2); c.decode(d); assert(!c.isOffline());
  Near(c.gimbal_data().yaw,2.4f,0.00051f);
  Near(c.gimbal_data().pitch,-0.4f,0.00051f);
  now=550; d[5]=0; c.decode(d); now=601; assert(c.isOffline());
  now=UINT32_MAX-50; g.encode(d); c.decode(d);
  now=49; assert(!c.isOffline()); now=50; assert(c.isOffline());
  auto good=Dbus(); assert(ValidDbus(good.data()));
  auto bad=Dbus(0); assert(!ValidDbus(bad.data()));
  bad=Dbus(1,1,363); assert(!ValidDbus(bad.data()));
  bad=good; bad[12]=2; assert(!ValidDbus(bad.data()));
  bad=Dbus(1,1,1684,364,1684,364,1684); assert(ValidDbus(bad.data()));
  const float inv=1.0f/config::kWheelRadius;
  auto wheel=WheelTargets({1,0,0},0); for(float w:wheel) Near(w,inv);
  wheel=WheelTargets({0,1,0},0);
  Near(wheel[0],-inv); Near(wheel[1],inv); Near(wheel[2],-inv); Near(wheel[3],inv);
  wheel=WheelTargets({0,0,1},0);
  assert(wheel[0]<0 && wheel[1]<0 && wheel[2]>0 && wheel[3]>0);
  wheel=WheelTargets({1,0,0},PI/2);
  Near(wheel[0],inv); Near(wheel[1],-inv); Near(wheel[2],inv); Near(wheel[3],-inv);
  auto small=WheelTargets({0.1f,0.15f,0.2f},0.7f);
  auto large=WheelTargets({10,15,20},0.7f);
  float ratio=large[0]/small[0],max=0;
  for(int i=0;i<4;++i) { Near(large[i],small[i]*ratio,0.0001f); max=std::max(max,std::fabs(large[i])); }
  Near(max,config::kMaxWheelSpeed);
  Near(NormalizeAngle(PI+0.1f),-PI+0.1f);
  Near(NormalizeAngle(std::numeric_limits<float>::infinity()),0);
  Near(Deadband(0.02f),0); Near(Deadband(-1),-1);
  hello_world::pid::Pid p(0,2,0,0.5f,0.7f);
  for(int i=0;i<10000;++i) Near(p.calculate(1,0.01f),std::min((i+1)*0.02f,0.7f),0.0001f);
  p.reset(); Near(p.calculate(0),0);
  std::cout<<"PASS protocol, DBUS, kinematics, saturation, PID, tick rollover\n";
}

static void MotorAndAxes() {
  uint8_t wheel_frame[8]={};
  bool seen_id[4]={};
  for(int i=0;i<4;++i) {
    const auto id=config::kWheelIds[i];
    assert(id>=1 && id<=4 && !seen_id[id-1]);
    seen_id[id-1]=true;
    OptionalParams wheel_opt;
    wheel_opt.dir=config::kWheelReversed[i] ? kDirRev : kDirFwd;
    M3508 wheel; wheel.init(id,wheel_opt);
    wheel.setInput(1000.0f*(i+1));
    uint8_t encoded[8]={}; assert(wheel.encode(encoded));
    std::copy_n(encoded+2*(id-1),2,wheel_frame+2*(id-1));
  }
  for(int i=0;i<4;++i) {
    const int slot=2*(config::kWheelIds[i]-1);
    const int16_t raw=static_cast<int16_t>((wheel_frame[slot]<<8)|wheel_frame[slot+1]);
    assert(raw==(config::kWheelReversed[i] ? -1 : 1)*1000*(i+1));
  }
  OptionalParams opt; opt.input_type=InputType::kTorq; opt.max_torq_input_lim=4;
  opt.angle_range=AngleRange::kNegPiToPosPi;
  DM_J4310 yaw,pitch; yaw.init(2,opt,false,false); pitch.init(1,opt,false,false);
  uint8_t d[8]; assert(yaw.encode(d)); // Regression: init must retain MIT mode.
  Feedback(yaw,2,1,0x8ab);
  Near(yaw.vel(),0x8ab*42.0f/4095-21,0.0001f); // Low nibble is in data[4].
  assert(yaw.setMitInput(0,0,0,0.15f,1)==Status::kOk);
  assert(yaw.setMitInput(0,0,501,0,0)==Status::kInputValueOverflow);
  yaw.setInput(0); assert(yaw.encode(d));
  assert(((d[3]&15)<<8|d[4])==0); assert((d[5]<<4|d[6]>>4)==0);
  assert(std::abs(static_cast<int>((d[6]&15)<<8|d[7])-2047)<=1);
  yaw.enable(); yaw.encode(d); assert(d[7]==0xfc);
  yaw.disable(); yaw.encode(d); assert(d[7]==0xfd);
  hello_world::pid::Pid yp(8,0,0,2,4), pp(8,0,0,2,4);
  Gimbal g; g.registerMotor(&yaw,&pitch); g.registerPid(&yp,&pp);
  g.setYawMeasurement(PI-0.01f); g.setTarget(-PI+0.01f,10); g.update();
  Near(yaw.getInput(),0.16f,0.0001f); Near(g.target().pitch,config::kPitchMax-0.02f);
  Near(pitch.getInput(),3.44f,0.0001f);
  g.stop(); Near(yaw.getInput(),0); Near(pitch.getInput(),0);
}

struct Rig {
  M3508 wheel[4]; DM_J4310 yaw,pitch;
  hello_world::pid::Pid wp[4];
  hello_world::pid::Pid yp{8,0,0.05f,2,4},pp{8,0,0.05f,2,4};
  hello_world::imu::Imu ci,gi;
  DT7 remote;
  Chassis chassis; Gimbal gimbal;
  ChassisRobot c; GimbalRobot g;
  GimbalChassisComm cc{GimbalChassisComm::CodePart::Chassis,0x210,0x211};
  GimbalChassisComm gc{GimbalChassisComm::CodePart::Gimbal,0x211,0x210};
  Rig() {
    now=0;
    for(int i=0;i<4;++i) {
      OptionalParams opt; opt.max_raw_input_lim=7000; wheel[i].init(i+1,opt);
      wp[i].setGains(100,1,0); wp[i].setLimits(100,7000);
      chassis.registerMotor(i,&wheel[i],&wp[i]); c.registerMotor(i,&wheel[i]);
    }
    OptionalParams opt; opt.input_type=InputType::kTorq; opt.max_torq_input_lim=4;
    opt.angle_range=AngleRange::kNegPiToPosPi;
    yaw.init(2,opt,false,false); pitch.init(1,opt,false,false);
    gimbal.registerMotor(&yaw,&pitch); gimbal.registerPid(&yp,&pp);
    c.registerChassis(&chassis); c.registerImu(&ci); c.registerRemote(&remote); c.registerComm(&cc);
    g.registerGimbal(&gimbal); g.registerImu(&gi); g.registerComm(&gc);
    g.registerMotor(&yaw,&pitch);
  }
  void receive(int left=1,int right=1,int lv=1024,int wheel_cmd=1024,int rh=1024,
      int lh=1024) {
    const auto d=Dbus(left,right,rh,1024,lh,lv,wheel_cmd);
    assert(ValidDbus(d.data())); remote.decode(d.data()); c.notifyRemoteFrame();
  }
  void feedback(bool dm=true,int missing=-1,int status=1) {
    for(int i=0;i<4;++i) if(i!=missing) c.notifyMotorFrame(i);
    if(dm) { Feedback(yaw,2,status); Feedback(pitch,1,status); g.notifyMotorFrame(0); g.notifyMotorFrame(1); }
  }
  void step(bool g2c=true,bool c2g=true) {
    uint8_t d[8]; c.update(); g.update();
    if(g2c) { gc.encode(d); cc.decode(d); }
    if(c2g) { cc.encode(d); gc.decode(d); }
    ++now;
  }
  void run(int n,int left=1,int right=1,int lv=1024,int wheel_cmd=1024,
      bool g2c=true,bool c2g=true,bool dbus=true,bool dm=true,int missing=-1) {
    for(int i=0;i<n;++i) {
      if(dbus) receive(left,right,lv,wheel_cmd);
      feedback(dm,missing); step(g2c,c2g);
    }
  }
  void zero() { for(auto &w:wheel) Near(w.getInput(),0); Near(yaw.getInput(),0); Near(pitch.getInput(),0); }
  void working() { assert(c.state()==LifeState::Working && g.state()==LifeState::Working); }
};
static void StateMachines() {
  Rig r;
  g_chassis_debug.trip_reason=0; g_chassis_debug.trip_count=0;
  r.step(); r.zero(); assert(r.c.state()==LifeState::Dead && r.g.state()==LifeState::Dead);
  // No initial DM feedback: chassis announces its own readiness, gimbal sends enable.
  for(int i=0;i<5;++i) { r.receive(); r.feedback(false); r.step(); }
  uint8_t d[8]; r.yaw.encode(d); assert(d[7]==0xfc);
  r.pitch.encode(d); assert(d[7]==0xfc);
  r.run(6); r.working(); r.zero();
  r.run(20,1,1,1684); r.working();
  assert(r.cc.chassis_data().remote_lv>0.99f); for(auto &w:r.wheel) assert(w.getInput()>0);
  // A right-stick DBUS value maps to negative gimbal-frame y (gun right).
  for(int i=0;i<10;++i) { r.receive(1,1,1024,1024,1024,1684); r.feedback(); r.step(); }
  assert(g_chassis_debug.remote_vy<0);
  assert(r.wheel[0].getInput()>0 && r.wheel[1].getInput()<0 &&
         r.wheel[2].getInput()>0 && r.wheel[3].getInput()<0);
  // Separate with centered wheel does not auto-rotate; follow uses joint yaw.
  r.run(4); r.zero(); r.cc.gimbal_data().yaw=0.3f;
  r.c.update(); for(auto &w:r.wheel) Near(w.getInput(),0);
  r.run(4,1,3); r.cc.gimbal_data().yaw=0.3f; r.c.update();
  assert(r.wheel[0].getInput()<0 && r.wheel[3].getInput()>0);
  r.run(20,1,2,1024,364); assert(r.chassis.mode()==ChassisMode::Spin);
  Near(r.chassis.motion().w,config::kSpinMaxRotation);
  assert(r.wheel[0].getInput()<0 && r.wheel[3].getInput()>0);
  const float target=r.gimbal.target().yaw;
  r.gi.heading=0.1f; r.run(4); Near(r.gimbal.target().yaw,target); assert(r.yaw.getInput()<0);
  r.run(3,2); r.zero(); assert(r.c.state()==LifeState::Dead && r.g.state()==LifeState::Dead);
  r.yaw.encode(d); assert(d[7]==0xfd);
  r.gi.heading=0.7f; r.receive(); r.feedback(); r.step();
  r.receive(); r.feedback(); r.step();
  Near(r.gimbal.target().yaw,0.7f); r.zero(); // Recovery aligns targets, no kick.
  r.run(4); r.working();
  r.run(103,1,1,1684,1024,true,true,false); r.zero();
  assert(r.g.state()==LifeState::Dead && r.c.state()==LifeState::Dead);
  assert(g_chassis_debug.trip_reason==2 && g_chassis_debug.trip_count>=1);
  r.run(5); r.working();
  r.run(104,1,1,1684,1024,false,true); r.zero(); assert(!r.cc.chassis_data().ready);
  r.run(7); r.working();
  r.run(104,1,1,1684,1024,true,false); r.zero(); assert(!r.gc.gimbal_data().ready);
  r.run(7); r.working();
  r.run(104,1,1,1684,1024,true,true,true,true,2); r.zero();
  assert(r.c.state()!=LifeState::Working && r.g.state()!=LifeState::Working);
  r.run(7); r.working();
  r.run(104,1,1,1684,1024,true,true,true,false); r.zero();
  assert(r.g.state()==LifeState::Resurrection && r.c.state()==LifeState::Resurrection);
  r.run(7); r.working();
  r.ci.valid=false; r.run(3); r.zero(); assert(!r.cc.chassis_data().ready);
  r.ci.valid=true; r.run(7); r.working();
  r.gi.valid=false; r.run(3); r.zero(); assert(!r.gc.gimbal_data().ready);
  r.gi.valid=true; r.run(7); r.working();
  r.receive(); r.feedback(true,-1,DaMiao::kOverCurr); r.step(); r.step(); r.zero();
  assert(r.g.state()!=LifeState::Working && r.c.state()!=LifeState::Working);
  r.run(7); r.working();
  r.c.invalidateRemote(); r.step(); r.step(); r.zero();
  std::cout<<"PASS actual dual-board startup, modes, world-yaw hold, emergency, timeouts, faults, recovery\n";
}
int main() { ProtocolAndMath(); MotorAndAxes(); StateMachines(); }
