
#include "I2C_Driver.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <cstring>
#include <cmath>

// Implementation of the I2C driver functions
// Initialize I2C: Configure its device address and register callbacks
// Receive data: Process register selections and writes from Jetson
// Send data: Return requested register values to Jetson

// Variables to store register data
static float currentAngle = 0.0f;
static float targetAngle = 0.0f;
static uint8_t controllerStatus = 0;

// Register currently selected by Jetson
static uint8_t selectedRegister =
    I2C_ADDRESS_MAP::READY;

// Protect shared variables accessed by different ESP32 tasks
static portMUX_TYPE registerMutex =
    portMUX_INITIALIZER_UNLOCKED;

// Ensure float is exactly 4 bytes
static_assert(sizeof(float) == 4, "Expected 4-byte float");


// Callback: Jetson WRITES to ESP32
static void receiveEvent(int numBytes) {

    // Make sure at least one byte was received
    if (numBytes < 1) return;

    // First byte is the register address
    uint8_t reg = Wire.read();
    numBytes--;

    // Store which register Jetson selected
    portENTER_CRITICAL(&registerMutex);
    selectedRegister = reg;
    portEXIT_CRITICAL(&registerMutex);

    // If only the register address was sent,
    // Jetson is selecting a register for reading.
    if (numBytes == 0) return;

    // Check if Jetson is writing to TARGET_ANGLE
    if (reg == I2C_ADDRESS_MAP::TARGET_ANGLE &&
        numBytes == sizeof(float)) {

        uint8_t bytes[4];

        // Read the 4 data bytes
        for (int i = 0; i < 4; i++) {
            bytes[i] = Wire.read();
        }

        // Convert received bytes into a float
        float value;
        memcpy(&value, bytes, sizeof(value));

        // Only store valid finite values
        if (std::isfinite(value)) {
            portENTER_CRITICAL(&registerMutex);
            targetAngle = value;
            portEXIT_CRITICAL(&registerMutex);
        }
    }

    // Discard any remaining or invalid bytes
    while (Wire.available()) {
        Wire.read();
    }
}


// Callback: Jetson READS from ESP32
static void requestEvent() {

    // Take a consistent snapshot of the selected register
    uint8_t reg;
    float angleSnapshot = 0.0f;
    uint8_t readySnapshot = 0;

    portENTER_CRITICAL(&registerMutex);

    reg = selectedRegister;

    if (reg == I2C_ADDRESS_MAP::CURRENT_ANGLE) {
        angleSnapshot = currentAngle;
    }
    else if (reg == I2C_ADDRESS_MAP::TARGET_ANGLE) {
        angleSnapshot = targetAngle;
    }
    else if (reg == I2C_ADDRESS_MAP::READY) {
        readySnapshot = ready;
    }

    portEXIT_CRITICAL(&registerMutex);

    // Check which register Jetson selected
    switch (reg) {

        case I2C_ADDRESS_MAP::CURRENT_ANGLE:

            // Send 4 bytes containing currentAngle
            Wire.write(
                (const uint8_t*)&angleSnapshot,
                sizeof(angleSnapshot)
            );
            break;

        case I2C_ADDRESS_MAP::TARGET_ANGLE:

            // Send the stored target angle as 4 bytes
            Wire.write(
                (const uint8_t*)&angleSnapshot,
                sizeof(angleSnapshot)
            );
            break;

        case I2C_ADDRESS_MAP::READY:

            // Send the 1-byte status value
            Wire.write(readySnapshot);
            break;

        default:

            // Unknown register selected
            Wire.write((uint8_t)0xFF);
            break;
    }
}


// Initialize ESP32 as an I2C target
bool initialize_i2c() {

    // Register callback for incoming writes
    Wire.onReceive(receiveEvent);

    // Register callback for incoming reads
    Wire.onRequest(requestEvent);

    // Initialize I2C using config.h settings
    return Wire.begin(
        (uint8_t)I2C_ADDRESS,
        SDA_PIN,
        SCL_PIN,
        I2C_FREQUENCY
    );
}

// Update current angle from the sensor
void setCurrentAngle(float angle) {

    // Only store valid angle measurements
    if (!std::isfinite(angle)) return;

    portENTER_CRITICAL(&registerMutex);
    currentAngle = angle;
    portEXIT_CRITICAL(&registerMutex);
}


// Get the latest stored current angle
float getCurrentAngle() {

    float angle;

    portENTER_CRITICAL(&registerMutex);
    angle = currentAngle;
    portEXIT_CRITICAL(&registerMutex);

    return angle;
}

// Update the initialization-ready bit
void setReady(bool value) {
    portENTER_CRITICAL(&registerMutex);
    ready = value ? 1 : 0;
    portEXIT_CRITICAL(&registerMutex);
}
