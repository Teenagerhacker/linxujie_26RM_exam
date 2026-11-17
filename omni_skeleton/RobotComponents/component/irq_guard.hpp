#ifndef ROBOT_IRQ_GUARD_HPP_
#define ROBOT_IRQ_GUARD_HPP_
#include "stm32_hal.hpp"
namespace robot {
// Only use around short packet/control snapshots, never around SPI or delays.
class IrqGuard {
 public:
  IrqGuard() : saved_(__get_PRIMASK()) { __disable_irq(); }
  ~IrqGuard() { __set_PRIMASK(saved_); }
  IrqGuard(const IrqGuard &) = delete;
  IrqGuard &operator=(const IrqGuard &) = delete;
 private:
  uint32_t saved_;
};
}
#endif
