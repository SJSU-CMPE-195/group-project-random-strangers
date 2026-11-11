#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "I2C_Driver.h"
#include "hall_sensor.h"
#include "motor_control.h"
// Gimbal Controller main program
// Two main functions: setup() and loop()
// setup() is called once when ESP32 starts 
// loop() is called continuoualy to read sensor angle, update registers, and chack status

// Status values for the ESP32
constexpr uint8_t STATUS_NOT_READY = 0;
constexpr uint8_t STATUS_READY = 1;
constexpr uint8_t STATUS_SENSOR_ERROR = 2;

//Called once when the ESP32 starts up
void setup() {

    // Start serial communication for debugging
    Serial.begin(115200);

    Serial.println("Starting Gimbal ESP32...");

    // Initialize I2C communication
    if (!initialize_i2c()) {
        Serial.println("I2C initialization failed!");

        // Stop here because communication failed
        while (true) {
            delay(1000);
        }
    }

    Serial.println("I2C initialized successfully!");

    // Set status to ready for the communication test
    setStatus(STATUS_READY);
}
void loop() {

    // Simulated angle measurement in radians
    // Replace with Hall sensor reading later
    float currentAngle = 0.785f;

    // Update CURRENT_ANGLE register
    setCurrentAngle(currentAngle);

    // Delay for communication bench testing
    delay(10);
}