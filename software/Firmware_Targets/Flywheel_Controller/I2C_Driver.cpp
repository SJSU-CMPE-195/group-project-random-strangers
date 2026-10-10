#include "I2C_Driver.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace Flywheel_Controller {

volatile Event_Manager event_manager;
volatile Controller_Registers I2C_Registers;

static volatile uint8_t active_register_address = 0;
static uint32_t active_error_set = 0;
static uint8_t selected_error_id = 0;
static char selected_error_reason[ERROR_REASON_MAX_LENGTH + 1] = {};
static uint8_t selected_error_reason_length = 0;

static void rebuild_error_set() {
    uint32_t error_set = 0;
    const unsigned active_fault_count = fault_manager.get_active_fault_count();

    for (unsigned active_fault_index = 0;
         active_fault_index < active_fault_count;
         ++active_fault_index) {
        const uint8_t fault_id = fault_manager.get_active_fault_code(active_fault_index);
        if (fault_id >= 1 && fault_id <= 32) error_set |= 1UL << (fault_id - 1);
    }

    active_error_set = error_set;
}

static void rebuild_selected_error_reason() {
    const char* reason = fault_manager.get_fault_reason(selected_error_id);
    uint8_t write_index = 0;

    if (reason != nullptr) {
        while (write_index < ERROR_REASON_MAX_LENGTH && reason[write_index] != '\0') {
            selected_error_reason[write_index] = reason[write_index];
            ++write_index;
        }
    }

    selected_error_reason[write_index] = '\0';
    selected_error_reason_length = write_index;
}

static void synchronize_fault_state() {
    I2C_Registers.master_error = fault_manager.get_master_fault_state();
    rebuild_error_set();
    rebuild_selected_error_reason();
}

void dispatch_fault(const uint8_t fault_id) {
    fault_manager.dispatch_fault(fault_id);
    synchronize_fault_state();
}

bool initialize_i2c() {
    fault_manager.attach_master_fault_set_callback(synchronize_fault_state);
    fault_manager.attach_master_fault_clear_callback(synchronize_fault_state);
    synchronize_fault_state();

    if (SDA_PIN == GPIO_NUM_NC || SCL_PIN == GPIO_NUM_NC) {
        if (DEBUG) Serial.println("I2C pin(s) are not configured");
        dispatch_fault(FAULT_I2C_INIT);
        return false;
    }

    if (I2C_ADDRESS <= 0x08 || I2C_ADDRESS >= 0x78) {
        if (DEBUG) Serial.println("I2C address is reserved or outside the 7-bit range");
        dispatch_fault(FAULT_I2C_INIT);
        return false;
    }

    if (!Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, I2C_FREQUENCY)) {
        if (DEBUG) Serial.println("I2C initialization failed");
        dispatch_fault(FAULT_I2C_INIT);
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
            case I2C_ADDRESS_MAP::ERROR_ID:
                selected_error_id = incoming_data;
                rebuild_selected_error_reason();
                break;
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

        active_register_address = static_cast<uint8_t>(active_register_address + 1);
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

        case I2C_ADDRESS_MAP::ERROR_SET_0:
            response = static_cast<uint8_t>(active_error_set);
            break;
        case I2C_ADDRESS_MAP::ERROR_SET_1:
            response = static_cast<uint8_t>(active_error_set >> 8);
            break;
        case I2C_ADDRESS_MAP::ERROR_SET_2:
            response = static_cast<uint8_t>(active_error_set >> 16);
            break;
        case I2C_ADDRESS_MAP::ERROR_SET_3:
            response = static_cast<uint8_t>(active_error_set >> 24);
            break;

        case I2C_ADDRESS_MAP::ERROR_ID:
            response = selected_error_id;
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

        case I2C_ADDRESS_MAP::SOLENOID_ARMED:
            response = I2C_Registers.solenoid_armed;
            break;

        case I2C_ADDRESS_MAP::ERROR_REASON_LENGTH:
            response = selected_error_reason_length;
            break;

        case I2C_ADDRESS_MAP::ERROR_REASON_START:
            if (selected_error_reason_length == 0) {
                Wire.slaveWrite(&response, 1);
            } else {
                Wire.slaveWrite(reinterpret_cast<uint8_t *>(selected_error_reason), selected_error_reason_length);
            }
            return;
    }

    Wire.slaveWrite(&response, 1);
}

}; //namespace Flywheel Controller
