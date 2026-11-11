
#pragma once

#include <stdint.h>
// Function declarations for the I2C driver

// Initialize ESP32 as an I2C target
// Returns true if initialization is successful, false otherwise
bool initialize_i2c();

// Update the current angle from the sensor
// angle: the current angle measured by the sensor, in radians
void setCurrentAngle(float angle);

// Get the current angle from the sensor
// Returns the current angle measured by the sensor, in radians
float getCurrentAngle();

// Update the initialization-ready bit
void setReady(bool ready);

