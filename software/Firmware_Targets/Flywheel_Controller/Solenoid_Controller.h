#pragma once

#include <stdint.h>

namespace Flywheel_Controller {

class Solenoid_Controller {
public:
    //initialize pin state
    bool begin();

    //check if the solenoid needs to be turned off 
    void update(uint32_t now);

    //disable the solenoid
    void force_safe_state();

private:
    bool leave_on = false;
    uint32_t off_time_ms = 0;

    void write_output(bool energized);
};

extern Solenoid_Controller solenoid_controller;

}; //namespace Flywheel Controller
