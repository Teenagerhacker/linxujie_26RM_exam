#ifndef ROBOT_BOARD_CAN_HPP_
#define ROBOT_BOARD_CAN_HPP_
#include "can.h"
namespace robot {
// CAN1 is the inter-board bus on both boards. On the gimbal it also carries
// yaw motor traffic; CAN2 carries the remaining local motor traffic.
inline CAN_HandleTypeDef *BoardLinkCan() { return &hcan1; }
inline CAN_HandleTypeDef *MotorCan() { return &hcan2; }
inline void DiscardPendingCan() {
  // Drop commands left from a previous control cycle; never queue stale torque after reconnection.
  constexpr uint32_t all=CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2;
  HAL_CAN_AbortTxRequest(BoardLinkCan(),all);
  HAL_CAN_AbortTxRequest(MotorCan(),all);
}
}
#endif
