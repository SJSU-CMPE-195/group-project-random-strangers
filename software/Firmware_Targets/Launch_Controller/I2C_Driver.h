#pragma once

#include <stdint.h>

namespace Flywheel_Controller {

struct Event_Manager {
    volatile bool fire_solenoid = false;
    volatile bool cancel_solenoid_fire = false;

    volatile bool arm_escs = false;
    volatile bool disarm_escs = false;

    volatile bool arm_solenoid = false;
    volatile bool disarm_solenoid = false;
};

struct Controller_Registers {
    bool ready = false;
    bool master_error = false;

    bool flywheels_armed = false;
    float RPM_target = 0.0f;
    float RPM_actual = 0.0f;
    bool solenoid_armed = false;
};

extern volatile Event_Manager event_manager;
extern volatile Controller_Registers I2C_Registers;

bool initialize_i2c();

void dispatch_fault(uint8_t fault_id);

void receiveEvent(int howMany);
void requestEvent();

}; //namespace Flywheel Controller
