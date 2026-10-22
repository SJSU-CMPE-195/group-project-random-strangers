//This firmware is designed to run on an esp32 based microcontroller board using the Arduino framework
// It is in charge of controlling the flywheel ESCs based on I2C commands from the Jetson
// It controls two bluejay ESCs using bidirectional DSHOT and maintains precise RPM targets using a PID loop

#include <Arduino.h>

#include "config.h"
#include "ESC_Controller.h"
#include "I2C_Driver.h"
#include "Solenoid_Controller.h"

using namespace Flywheel_Controller;

static void halt_on_initialization_error() {
    if (DEBUG) Serial.println("Initialization failed. Halting...");

    while (true) {
        esc_controller.force_safe_state();
        solenoid_controller.force_safe_state();
        delay(1000);
    }
}

void setup() {
    if (DEBUG) {
        Serial.begin(115200);
        delay(500);

        Serial.println("DShotRMT Flywheel ESC Controller");
        Serial.println("Motors and solenoid off");
        SOLENOID_ACTIVE_HIGH ? Serial.println("Solenoid active high") : Serial.println("Solenoid active low");
        Serial.println("Starting...");
    }

    const bool i2c_success = initialize_i2c();
    const bool esc_success = esc_controller.begin();
    const bool solenoid_success = solenoid_controller.begin();

    if (!i2c_success || !esc_success || !solenoid_success) {
        halt_on_initialization_error();
    }

    I2C_Registers.ready = true;
}

void loop() {
    const uint32_t now = millis();

    esc_controller.update(now);
    solenoid_controller.update(now);
}
