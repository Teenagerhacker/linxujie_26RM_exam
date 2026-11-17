#include "ins_imu.hpp"
#include "imu.hpp"
#include "gpio.h"
#include "spi.h"

hello_world::imu::Imu *GetImu() {
  static hello_world::imu::Imu imu;
  static bool initialized = false;
  if (!initialized) {
    hello_world::imu::BMI088HWConfig hw;
    hw.hspi = &hspi1;
    hw.acc_cs_port = GPIOA; hw.acc_cs_pin = GPIO_PIN_4;
    hw.gyro_cs_port = GPIOB; hw.gyro_cs_pin = GPIO_PIN_0;
    const float rotation[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                               0.0f, 0.0f, 1.0f};
    imu.init(hw, rotation);
    initialized = true;
  }
  return &imu;
}

