#include <Arduino.h>
#include "I2C_Driver.h"

void setup() {
    Serial.begin(115200);

    bool success =
        Gimbal_Controller::initialize_i2c();

    Gimbal_Controller::I2C_Registers.ready =
        success ? 1 : 0;

    Gimbal_Controller::I2C_Registers.test_value = 42;
}

void loop() {
    // Empty for initial I2C test
}