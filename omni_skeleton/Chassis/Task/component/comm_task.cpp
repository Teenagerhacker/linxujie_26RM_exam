#include "comm_task.hpp"
#include "HW_can.hpp"
#include "board_can.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"
#include "gimbal_chassis_comm.hpp"
#include "DT7.hpp"
#include "control_math.hpp"
#include "ins_comm.hpp"
#include "ins_robot.hpp"
#include "robot.hpp"
#include "usart.h"
#include <cstring>
#include "robot_config.hpp"
namespace {
uint8_t rx_buf[hello_world::remote_control::kRcRxDataLen];
void ArmReceiver() {
  if(HAL_UARTEx_ReceiveToIdle_DMA(&huart3,rx_buf,sizeof(rx_buf))==HAL_OK)
    __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);
}
}
void CommTaskInit(void) {
  CanFilter_Init(&hcan1); CanFilter_Init(&hcan2);
  if(HAL_CAN_Start(&hcan1)!=HAL_OK || HAL_CAN_Start(&hcan2)!=HAL_OK) Error_Handler();
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
  ArmReceiver();
}
void CommTask(void) {
  robot::DiscardPendingCan();
  uint8_t frame[8]={};
  for(int i=0;i<4;++i) {
    auto *m=GetMotorWheel(i); uint8_t data[8]={};
    if(m && m->encode(data)) {
      const int slot=(robot::config::kWheelIds[i]-1)%4;
      std::memcpy(frame+2*slot,data+2*slot,2);
    }
  }
  CAN_Send_Msg(robot::MotorCan(),frame,0x200,8);
  auto *comm=GetGimbalChassisComm();
  comm->encode(frame); CAN_Send_Msg(robot::BoardLinkCan(),frame,comm->tx_id(),8);
  if(huart3.RxState==HAL_UART_STATE_READY) ArmReceiver();
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart,uint16_t size) {
  if(uart!=&huart3) return;
  if(size==sizeof(rx_buf) && robot::ValidDbus(rx_buf)) {
    GetRemoteControl()->decode(rx_buf); GetRobot()->notifyRemoteFrame();
  }
  // DESIGN TRADE-OFF: discard one partial/invalid DBUS frame without
  // immediately disarming. The last-valid-frame 100 ms timeout still gives a
  // deterministic fail-safe, while UART idle noise cannot disable both boards.
  ArmReceiver();
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
  if(uart!=&huart3) return;
  // Keep the last valid timestamp; a persistent UART failure is handled by the
  // same 100 ms timeout instead of turning one recoverable error into a trip.
  HAL_UART_AbortReceive(uart);
  __HAL_UART_CLEAR_OREFLAG(uart);
  ArmReceiver();
}
