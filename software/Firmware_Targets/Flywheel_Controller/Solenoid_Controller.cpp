#include "Solenoid_Controller.h"

#include <Arduino.h>

#include "I2C_Driver.h"
#include "config.h"

namespace Flywheel_Controller {

Solenoid_Controller solenoid_controller;

bool Solenoid_Controller::begin() {
    I2C_Registers.solenoid_error = 1;

    if (SOLENOID_PIN == GPIO_NUM_NC) {
        if (DEBUG) Serial.println("Error: Solenoid pin is not configured");
        I2C_Registers.master_error = true;
        return false;
    }

    //configure the pin before writing it so the output cannot briefly be driven from an uninitialized pin mode.
    pinMode(SOLENOID_PIN, OUTPUT);
    write_output(false);

    I2C_Registers.solenoid_error = 0;
    if (DEBUG) Serial.println("Solenoid initialized successfully");
    return true;
}

void Solenoid_Controller::write_output(const bool energized) {
    if (SOLENOID_PIN == GPIO_NUM_NC) return;

    digitalWrite(SOLENOID_PIN, energized ? SOLENOID_ACTIVE_HIGH : !SOLENOID_ACTIVE_HIGH);
}

void Solenoid_Controller::force_safe_state() {
    leave_on = false;
    I2C_Registers.solenoid_armed = false;
    event_manager.fire_solenoid = false;
    event_manager.cancel_solenoid_fire = false;
    event_manager.arm_solenoid = false;
    event_manager.disarm_solenoid = false;
    write_output(false);
}

void Solenoid_Controller::update(const uint32_t now) {
    if (I2C_Registers.master_error) {
        force_safe_state();
        return;
    }

    //disarm takes priority
    if (event_manager.disarm_solenoid) {
        I2C_Registers.solenoid_armed = false;
        leave_on = false;
        if (DEBUG) Serial.println("Solenoid disarmed");
    } else if (event_manager.arm_solenoid) {
        I2C_Registers.solenoid_armed = true;
        if (DEBUG) Serial.println("Solenoid armed");
    }
    event_manager.arm_solenoid = false;
    event_manager.disarm_solenoid = false;

    if (event_manager.cancel_solenoid_fire) {
        leave_on = false;
        event_manager.cancel_solenoid_fire = false;
    }

    // --------- Handle Solenoid Firing ---------
    if (event_manager.fire_solenoid) {
        if (I2C_Registers.solenoid_armed && !leave_on) {
            leave_on = true;
            off_time_ms = now + SOLENOID_SHOT_TIME_MS;
            if (DEBUG) Serial.println("Solenoid firing");
        } else if (DEBUG && !I2C_Registers.solenoid_armed) {
            Serial.println("Solenoid fire ignored: solenoid is disarmed");
        }
        event_manager.fire_solenoid = false;
    }

    //signed subtraction keeps this timeout correct across millis() rollover.
    if (leave_on && static_cast<int32_t>(now - off_time_ms) >= 0) {
        leave_on = false;
        if (DEBUG) Serial.println("Solenoid firing complete");
    }

    write_output(I2C_Registers.solenoid_armed && leave_on);
}

}; //namespace Flywheel Controller
