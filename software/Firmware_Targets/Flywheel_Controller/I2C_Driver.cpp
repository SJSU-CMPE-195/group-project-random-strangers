#include "I2C_Driver.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace Flywheel_Controller {

volatile Event_Manager event_manager;
volatile Controller_Registers I2C_Registers;

static volatile uint8_t active_register_address = 0;

bool initialize_i2c() {
    if (SDA_PIN == GPIO_NUM_NC || SCL_PIN == GPIO_NUM_NC) {
        if (DEBUG) Serial.println("I2C pin(s) are not configured");
        return false;
    }

    if (I2C_ADDRESS <= 0x08 || I2C_ADDRESS >= 0x78) {
        if (DEBUG) Serial.println("I2C address is reserved or outside the 7-bit range");
        return false;
    }

    if (!Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, I2C_FREQUENCY)) {
        if (DEBUG) Serial.println("I2C initialization failed");
        return false;
    }

    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    if (DEBUG) Serial.println("I2C initialized successfully...");
    return true;
}

//I2C Controller write
void receiveEvent(const int howMany) {
    if (howMany < 1 || !Wire.available()) return; // No data sent

    // The first byte selects the register address.
    active_register_address = Wire.read();

    while (Wire.available()) {
        const uint8_t incoming_data = Wire.read();
        const uint8_t register_address = active_register_address;

        switch (register_address) {
            case I2C_ADDRESS_MAP::RPM_TARGET:
                reinterpret_cast<volatile uint8_t *>(&I2C_Registers.RPM_target)[0] = incoming_data;
                break;
            case I2C_ADDRESS_MAP::RPM_TARGET + 1:
                reinterpret_cast<volatile uint8_t *>(&I2C_Registers.RPM_target)[1] = incoming_data;
                break;
            case I2C_ADDRESS_MAP::RPM_TARGET + 2:
                reinterpret_cast<volatile uint8_t *>(&I2C_Registers.RPM_target)[2] = incoming_data;
                break;
            case I2C_ADDRESS_MAP::RPM_TARGET + 3:
                reinterpret_cast<volatile uint8_t *>(&I2C_Registers.RPM_target)[3] = incoming_data;
                break;
            case I2C_ADDRESS_MAP::FLYWHEELS_ARMED:
                if (incoming_data) { //arm requested
                    event_manager.arm_escs = true;
                } else { //disarm requested
                    event_manager.disarm_escs = true;
                }
                break;
            case I2C_ADDRESS_MAP::SOLENOID_ARMED:
                if (incoming_data) { //arm requested
                    event_manager.arm_solenoid = true;
                } else { //disarm requested
                    event_manager.disarm_solenoid = true;
                }
                break;
            case I2C_ADDRESS_MAP::SOLENOID_FIRE:
                if (incoming_data) { //fire requested
                    event_manager.fire_solenoid = true;
                } else { //cancel/reset requested
                    event_manager.fire_solenoid = false;
                    event_manager.cancel_solenoid_fire = true;
                }
                break;
            default:
                break;
        }

        active_register_address++;
    }
}

//I2C Controller read
void requestEvent() {
    uint8_t response = 0;

    switch (active_register_address) {
        case I2C_ADDRESS_MAP::READY:
            response = I2C_Registers.ready;
            break;

        case I2C_ADDRESS_MAP::MASTER_ERROR:
            response = I2C_Registers.master_error;
            break;

        case I2C_ADDRESS_MAP::FLYWHEELS_ARMED:
            response = I2C_Registers.flywheels_armed;
            break;

        case I2C_ADDRESS_MAP::RPM_TARGET:
            response = reinterpret_cast<const volatile uint8_t *>(&I2C_Registers.RPM_target)[0];
            break;
        case I2C_ADDRESS_MAP::RPM_TARGET + 1:
            response = reinterpret_cast<const volatile uint8_t *>(&I2C_Registers.RPM_target)[1];
            break;
        case I2C_ADDRESS_MAP::RPM_TARGET + 2:
            response = reinterpret_cast<const volatile uint8_t *>(&I2C_Registers.RPM_target)[2];
            break;
        case I2C_ADDRESS_MAP::RPM_TARGET + 3:
            response = reinterpret_cast<const volatile uint8_t *>(&I2C_Registers.RPM_target)[3];
            break;

        case I2C_ADDRESS_MAP::ACTUAL_RPM:
            response = reinterpret_cast<const volatile uint8_t *>(&I2C_Registers.RPM_actual)[0];
            break;
        case I2C_ADDRESS_MAP::ACTUAL_RPM + 1:
            response = reinterpret_cast<const volatile uint8_t *>(&I2C_Registers.RPM_actual)[1];
            break;
        case I2C_ADDRESS_MAP::ACTUAL_RPM + 2:
            response = reinterpret_cast<const volatile uint8_t *>(&I2C_Registers.RPM_actual)[2];
            break;
        case I2C_ADDRESS_MAP::ACTUAL_RPM + 3:
            response = reinterpret_cast<const volatile uint8_t *>(&I2C_Registers.RPM_actual)[3];
            break;

        case I2C_ADDRESS_MAP::ESC_ERROR:
            response = I2C_Registers.ESC_error;
            break;

        case I2C_ADDRESS_MAP::SOLENOID_ARMED:
            response = I2C_Registers.solenoid_armed;
            break;

        case I2C_ADDRESS_MAP::SOLENOID_ERROR:
            response = I2C_Registers.solenoid_error;
            break;
    }

    Wire.slaveWrite(&response, 1);
}

}; //namespace Flywheel Controller
