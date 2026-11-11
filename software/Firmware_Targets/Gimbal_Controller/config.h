
#pragma once

#include <Arduino.h>

// I2C communication settings
// Yaw ESP32 = 0x54, MTC ESP32 = 0x55
#define I2C_ADDRESS 0x54

// ESP32 I2C pins 
//changed from pin 4 because it 
//conflicted with SimpleFOC PWM pin
#define SDA_PIN 25
#define SCL_PIN 26

// I2C register addresses
namespace I2C_ADDRESS_MAP {

    // Read-only: 1 when initialization is complete
    // Data type: uint8_t (1 byte)
    constexpr uint8_t READY = 0x00;

    // Read/write: target angle in radians
    // Data type: float (4 bytes)
    constexpr uint8_t TARGET_ANGLE = 0x01;

    // Read-only: current measured angle in radians
    // Data type: float (4 bytes)
    constexpr uint8_t CURRENT_ANGLE = 0x02;
}

// I2C communication frequency
#define I2C_FREQUENCY 100000
