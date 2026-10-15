#pragma once

#include <stdint.h>

namespace Flywheel_Controller {

volatile static struct event_manager_t {
    volatile bool fire_solenoid = false;
    volatile bool cancel_solenoid_fire = false;

    volatile bool arm_escs = false;
    volatile bool disarm_escs = false;

    volatile bool arm_solenoid = false;
    volatile bool disarm_solenoid = false;
} event_manager;

volatile static struct I2C_Registers_t {
    volatile bool ready = false;

    volatile uint32_t active_error_set = 0;
    volatile uint8_t selected_error_id = 0;
    volatile uint8_t active_error_reason_length = 0;

    volatile bool flywheels_armed = false;
    volatile float RPM_target = 0.0f;
    volatile float RPM_actual = 0.0f;

    volatile bool solenoid_armed = false;
} I2C_Registers;

bool initialize_i2c();

void dispatch_fault(uint8_t fault_id);

void receiveEvent(int howMany);
void requestEvent();

}; //namespace Flywheel Controller
