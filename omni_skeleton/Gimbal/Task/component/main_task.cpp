#include "main_task.hpp"

#include "comm_task.hpp"
#include "ins_all.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_imu.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "ins_robot.hpp"
#include "robot.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "tim.h"
#include "iwdg.h"
#include "robot_config.hpp"

void MainTaskInit(void) {
  auto *robot = GetRobot();
  auto *gimbal = GetGimbal();
  auto *yaw = static_cast<hello_world::motor::DaMiao *>(GetMotorYaw());
  auto *pitch = static_cast<hello_world::motor::DaMiao *>(GetMotorPitch());
  robot->registerGimbal(gimbal);
  robot->registerImu(GetImu());
  robot->registerComm(GetGimbalChassisComm());
  robot->registerMotor(yaw, pitch);
  robot->registerPid(GetPidMotorYaw(), GetPidMotorPitch());
  gimbal->registerMotor(yaw, pitch);
  gimbal->registerPid(GetPidMotorYaw(), GetPidMotorPitch());
  // DWT measures elapsed execution even while SysTick is pending in an ISR.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT=0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CommTaskInit();
}

void MainTask(void) { GetRobot()->update(); }

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    const uint32_t start=DWT->CYCCNT;
    MainTask();
    CommTask();
    // DESIGN TRADE-OFF: feed only after BOTH tasks return within the 1 ms
    // deadline. A transient overrun may eventually reset the MCU, but feeding
    // unconditionally could hide a blocked control or communications task.
    if (robot::config::kFeedWatchdog &&
        static_cast<uint32_t>(DWT->CYCCNT-start)<SystemCoreClock/1000u)
      HAL_IWDG_Refresh(&hiwdg);
  }
}
