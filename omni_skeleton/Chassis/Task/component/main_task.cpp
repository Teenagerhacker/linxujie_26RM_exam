#include "main_task.hpp"

#include "comm_task.hpp"
#include "ins_all.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_imu.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "ins_robot.hpp"
#include "robot.hpp"
#include "chassis.hpp"
#include "gimbal_chassis_comm.hpp"
#include "debug_telemetry.hpp"
#include "tim.h"
#include "iwdg.h"
#include "robot_config.hpp"

namespace {
constexpr uint32_t kDiagnosticMarkerAddress=0x080E0000u; // Unused sector 11.
constexpr uint32_t kDiagnosticEventAddress =0x080E0004u;
constexpr uint32_t kDiagnosticDetailAddress=0x080E0008u;
constexpr uint32_t kDiagnosticMarker=0x41524D44u; // "ARMD"
constexpr uint32_t kErasedFlash=0xFFFFFFFFu;

bool ProgramDiagnosticWord(uint32_t address,uint32_t value) {
  if (*reinterpret_cast<volatile const uint32_t *>(address)!=kErasedFlash)
    return false;
  if (HAL_FLASH_Unlock()!=HAL_OK) return false;
  const bool ok=HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,value)==HAL_OK;
  HAL_FLASH_Lock();
  return ok && *reinterpret_cast<volatile const uint32_t *>(address)==value;
}
}

void MainTaskInit(void) {
  // Keep hardware reset flags visible in Ozone/J-Link. A nonzero protection
  // trip with no MCU reset is distinct from a brownout/watchdog reboot.
  g_chassis_debug.reset_cause=RCC->CSR;
  auto *robot = GetRobot();
  auto *chassis = GetChassis();
  robot->registerChassis(chassis);
  robot->registerImu(GetImu());
  robot->registerRemote(GetRemoteControl());
  robot->registerComm(GetGimbalChassisComm());
  for (int i = 0; i < 4; ++i) {
    auto *motor = GetMotorWheel(i);
    chassis->registerMotor(i, motor, GetPidMotorWheel(i));
    robot->registerMotor(i, motor);
  }
  // DWT measures elapsed execution even while SysTick is pending in an ISR.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT=0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CommTaskInit();
}

void MainTask(void) { GetRobot()->update(); }

extern "C" void PersistTripTask(void) {
  static bool reset_checked=false;
  const uint32_t marker=*reinterpret_cast<volatile const uint32_t *>(kDiagnosticMarkerAddress);
  const uint32_t event=*reinterpret_cast<volatile const uint32_t *>(kDiagnosticEventAddress);
  if (!reset_checked) {
    // A marker with no event means the MCU reset while the robot was armed.
    if (marker==kDiagnosticMarker && event==kErasedFlash) {
      // Wait for a fresh remote frame. Starting with the emergency switch down
      // is the deliberate readback path and must not be mistaken for a reset
      // during an armed driving test.
      if (g_chassis_debug.tick_ms<500 || !g_chassis_debug.remote_valid) return;
      reset_checked=true;
      if (g_chassis_debug.armed) {
        ProgramDiagnosticWord(kDiagnosticEventAddress,0x52535400u); // "RST".
        ProgramDiagnosticWord(kDiagnosticDetailAddress,g_chassis_debug.reset_cause);
      }
      return;
    }
    reset_checked=true;
  }
  if (event!=kErasedFlash) return;
  if (marker==kErasedFlash && g_chassis_debug.life_state==2)
    ProgramDiagnosticWord(kDiagnosticMarkerAddress,kDiagnosticMarker);
  if (g_chassis_debug.trip_reason!=0) {
    ProgramDiagnosticWord(kDiagnosticEventAddress,
                          0x54524900u | g_chassis_debug.trip_reason); // "TRI" + code.
    ProgramDiagnosticWord(kDiagnosticDetailAddress,g_chassis_debug.trip_tick_ms);
  }
}

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
