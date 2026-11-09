
#pragma once

#include <stdint.h>

namespace Gimbal_Controller {

// Addresses of registers accessible through I2C
namespace I2C_ADDRESS_MAP {
    constexpr uint8_t READY = 0x00;
    constexpr uint8_t STATUS = 0x01;
    constexpr uint8_t TEST_VALUE = 0x02;
    constexpr uint8_t SENSOR_OK = 0x03;
    constexpr uint8_t ELECTRICAL_ANGLE = 0x04;
}

// Data shared with the I2C interface
struct I2C_Registers_t {
    volatile uint8_t ready = 0;
    volatile uint8_t status = 0;
    volatile uint8_t test_value = 0;
    volatile uint8_t sensor_ok = 0;
    volatile uint8_t electrical_angle = 0;
};

extern I2C_Registers_t I2C_Registers;

// Function declarations
bool initialize_i2c();

void receiveEvent(int howMany);
void requestEvent();

} // namespace Gimbal_Controller
