#include "HW_can.hpp"
#include "board_can.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"
#include "ins_robot.hpp"
#include "robot.hpp"
#include "gimbal_chassis_comm.hpp"
namespace {
// Visible in debugger; failures are retried on the next 1ms cycle.
volatile uint32_t tx_dropped=0;
void RouteFrame(CAN_HandleTypeDef *bus, uint32_t id, const uint8_t d[8]) {
  if(bus==robot::BoardLinkCan()) {
    auto *comm=GetGimbalChassisComm();
    if(comm && id==comm->rx_id()) {
      comm->decode(d);
      return;
    }
    auto *yaw=static_cast<hello_world::motor::DaMiao *>(GetMotorYaw());
    if(yaw && id==hello_world::motor::DaMiao::GetRxId(yaw->id())) {
      if ((d[0]&0x0f)==yaw->id() && yaw->decode(d))
        GetRobot()->notifyMotorFrame(0);
    }
    return;
  }
  if(bus!=robot::MotorCan()) return;
  auto *pitch=static_cast<hello_world::motor::DaMiao *>(GetMotorPitch());
  if(pitch && id==hello_world::motor::DaMiao::GetRxId(pitch->id())) {
    if ((d[0]&0x0f)==pitch->id() && pitch->decode(d))
      GetRobot()->notifyMotorFrame(1);
  }
}
void Drain(CAN_HandleTypeDef *bus, uint32_t fifo) {
  // Bounded work even under continuous bus traffic.
  for(int n=0;n<3 && HAL_CAN_GetRxFifoFillLevel(bus,fifo)>0;++n) {
    CAN_RxHeaderTypeDef header={}; uint8_t data[8]={};
    if(HAL_CAN_GetRxMessage(bus,fifo,&header,data)!=HAL_OK) break;
    if(header.IDE==CAN_ID_STD && header.RTR==CAN_RTR_DATA && header.DLC==8)
      RouteFrame(bus,header.StdId,data);
  }
}
}
void CanFilter_Init(CAN_HandleTypeDef *bus) {
  CAN_FilterTypeDef f={};
  f.FilterMode=CAN_FILTERMODE_IDMASK; f.FilterScale=CAN_FILTERSCALE_32BIT;
  f.FilterActivation=ENABLE; f.SlaveStartFilterBank=14;
  if(bus==&hcan1) { f.FilterFIFOAssignment=CAN_FILTER_FIFO0; f.FilterBank=0; }
  else if(bus==&hcan2) { f.FilterFIFOAssignment=CAN_FILTER_FIFO1; f.FilterBank=14; }
  else return;
  if(HAL_CAN_ConfigFilter(bus,&f)!=HAL_OK) Error_Handler();
}
void CAN_Send_Msg(CAN_HandleTypeDef *bus,uint8_t *data,uint32_t id,uint8_t len) {
  if(!data || id>0x7ff || len!=8) return;
  CAN_TxHeaderTypeDef h={}; uint32_t mailbox;
  h.StdId=id; h.IDE=CAN_ID_STD; h.RTR=CAN_RTR_DATA; h.DLC=len;
  if(HAL_CAN_GetTxMailboxesFreeLevel(bus)==0 ||
      HAL_CAN_AddTxMessage(bus,&h,data,&mailbox)!=HAL_OK) tx_dropped=tx_dropped+1;
}
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *bus) {
  if(bus==&hcan1) Drain(bus,CAN_RX_FIFO0);
}
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *bus) {
  if(bus==&hcan2) Drain(bus,CAN_RX_FIFO1);
}
