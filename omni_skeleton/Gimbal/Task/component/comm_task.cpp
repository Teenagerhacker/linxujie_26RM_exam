#include "comm_task.hpp"
#include "HW_can.hpp"
#include "board_can.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"
#include "gimbal_chassis_comm.hpp"
void CommTaskInit(void) {
  CanFilter_Init(&hcan1); CanFilter_Init(&hcan2);
  if(HAL_CAN_Start(&hcan1)!=HAL_OK || HAL_CAN_Start(&hcan2)!=HAL_OK) Error_Handler();
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
}
void CommTask(void) {
  robot::DiscardPendingCan();
  uint8_t data[8]={};
  auto *yaw=static_cast<hello_world::motor::DaMiao *>(GetMotorYaw());
  auto *pitch=static_cast<hello_world::motor::DaMiao *>(GetMotorPitch());
  // The vehicle harness puts yaw on the slip-ring CAN1 bus and pitch on CAN2.
  if(yaw && yaw->encode(data)) CAN_Send_Msg(robot::BoardLinkCan(),data,yaw->tx_id(),8);
  if(pitch && pitch->encode(data)) CAN_Send_Msg(robot::MotorCan(),data,pitch->tx_id(),8);
  auto *comm=GetGimbalChassisComm();
  comm->encode(data); CAN_Send_Msg(robot::BoardLinkCan(),data,comm->tx_id(),8);
}
