#include "I2C_Driver.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace Flywheel_Controller {

static volatile uint8_t active_register_address = 0;

static uint32_t swap_bytes(uint32_t data){
    return  ((data >> 24) & 0x000000FF) | // Move byte 3 to byte 0
            ((data >>  8) & 0x0000FF00) | // Move byte 2 to byte 1
            ((data <<  8) & 0x00FF0000) | // Move byte 1 to byte 2
            ((data << 24) & 0xFF000000);  // Move byte 0 to byte 3
}

//builds the active error bitset
static void refresh_error_set() {
    uint32_t error_set = 0;
    const unsigned active_fault_count = fault_manager.get_active_fault_count();

    for (unsigned active_fault_index = 0; active_fault_index < active_fault_count; ++active_fault_index) {
        const uint8_t fault_id = fault_manager.get_active_fault_code(active_fault_index);
        if (fault_id >= 1 && fault_id <= 32) error_set |= 1UL << (fault_id - 1);
    }

    I2C_Registers.active_error_set = swap_bytes(error_set);
}

bool initialize_i2c() {
    fault_manager.attach_any_fault_set_callback(refresh_error_set);
    fault_manager.attach_any_fault_clear_callback(refresh_error_set);
    refresh_error_set();

    if (SDA_PIN == GPIO_NUM_NC || SCL_PIN == GPIO_NUM_NC) {
        if (DEBUG) Serial.println("I2C pin(s) are not configured");
        fault_manager.dispatch_fault(FAULT_I2C_INIT);
        return false;
    }

    if (I2C_ADDRESS <= 0x08 || I2C_ADDRESS >= 0x78) {
        if (DEBUG) Serial.println("I2C address is reserved or outside the 7-bit range");
        fault_manager.dispatch_fault(FAULT_I2C_INIT);
        return false;
    }

    if (!Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, I2C_FREQUENCY)) {
        if (DEBUG) Serial.println("I2C initialization failed");
        fault_manager.dispatch_fault(FAULT_I2C_INIT);
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

    //the first byte selects the register address.
    active_register_address = Wire.read();

    while (Wire.available()) {
        const uint8_t incoming_data = Wire.read();
        const uint8_t register_address = active_register_address;

        switch (register_address) {
            case I2C_ADDRESS_MAP::ERROR_ID:
                I2C_Registers.selected_error_id = incoming_data;
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

        active_register_address = active_register_address + 1;
    }
}

//I2C Controller read
void requestEvent() {
    switch (active_register_address) {
        case I2C_ADDRESS_MAP::READY:
            Wire.write(I2C_Registers.ready);
            break;

        case I2C_ADDRESS_MAP::MASTER_ERROR:
            Wire.write(!fault_manager.get_master_fault_state());
            break;

        case I2C_ADDRESS_MAP::ERROR_SET_0: {
            uint32_t error_set_copy = I2C_Registers.active_error_set;
            Wire.write(&reinterpret_cast<const uint8_t*>(&error_set_copy)[0], 4);
            break;
        }
        case I2C_ADDRESS_MAP::ERROR_SET_1: {
            uint32_t error_set_copy = I2C_Registers.active_error_set;
            Wire.write(&reinterpret_cast<const uint8_t*>(&error_set_copy)[1], 3);
            break;
        }
        case I2C_ADDRESS_MAP::ERROR_SET_2: {
            uint32_t error_set_copy = I2C_Registers.active_error_set;
            Wire.write(&reinterpret_cast<const uint8_t*>(&error_set_copy)[2], 2);
            break;
        }
        case I2C_ADDRESS_MAP::ERROR_SET_3: {
            Wire.write(reinterpret_cast<const volatile uint8_t*>(&I2C_Registers.active_error_set)[3]);
            break;
        }

        case I2C_ADDRESS_MAP::ERROR_ID:
            Wire.write(I2C_Registers.selected_error_id);
            break;

        case I2C_ADDRESS_MAP::ERROR_REASON_LENGTH: {
            const char* fault_reason = fault_manager.get_fault_reason(I2C_Registers.selected_error_id);

            uint8_t reason_length;
            if(fault_reason == nullptr){
                reason_length = 0;
            } else {
                reason_length = strnlen(fault_reason, 255);
            }
            Wire.write(reason_length);
            break;
        }

        case I2C_ADDRESS_MAP::ERROR_REASON_START: {
            const char* fault_reason = fault_manager.get_fault_reason(I2C_Registers.selected_error_id);
            if(fault_reason == nullptr){
                return;
            } else {
                uint8_t reason_length = strnlen(fault_reason, 255);
                Wire.write(reinterpret_cast<const unsigned char*>(&fault_reason), reason_length);
            }
        }

        case I2C_ADDRESS_MAP::FLYWHEELS_ARMED:
            Wire.write(I2C_Registers.flywheels_armed);
            break;

        case I2C_ADDRESS_MAP::RPM_TARGET: {
            float RPM_Target_Copy = I2C_Registers.RPM_target;
            Wire.write(&reinterpret_cast<const uint8_t *>(&RPM_Target_Copy)[0], 4);
            break;
        }
        case I2C_ADDRESS_MAP::RPM_TARGET + 1: {
            float RPM_Target_Copy = I2C_Registers.RPM_target;
            Wire.write(&reinterpret_cast<const uint8_t *>(&RPM_Target_Copy)[1], 3);
            break;
        }
        case I2C_ADDRESS_MAP::RPM_TARGET + 2: {
            float RPM_Target_Copy = I2C_Registers.RPM_target;
            Wire.write(&reinterpret_cast<const uint8_t *>(&RPM_Target_Copy)[2], 2);
            break;
        }
        case I2C_ADDRESS_MAP::RPM_TARGET + 3: {
            Wire.write(reinterpret_cast<volatile const uint8_t*>(&I2C_Registers.RPM_target)[3]);
            break;
        }

        case I2C_ADDRESS_MAP::ACTUAL_RPM: {
            float RPM_Actual_Copy = I2C_Registers.RPM_actual;
            Wire.write(&reinterpret_cast<const uint8_t *>(&RPM_Actual_Copy)[0], 4);
            break;
        }
        case I2C_ADDRESS_MAP::ACTUAL_RPM + 1: {
            float RPM_Actual_Copy = I2C_Registers.RPM_actual;
            Wire.write(&reinterpret_cast<const uint8_t *>(&RPM_Actual_Copy)[1], 3);
            break;
        }
        case I2C_ADDRESS_MAP::ACTUAL_RPM + 2: {
            float RPM_Actual_Copy = I2C_Registers.RPM_actual;
            Wire.write(&reinterpret_cast<const uint8_t *>(&RPM_Actual_Copy)[2], 2);
            break;
        }
        case I2C_ADDRESS_MAP::ACTUAL_RPM + 3: {
            Wire.write(reinterpret_cast<volatile const uint8_t*>(&I2C_Registers.RPM_actual)[3]);
            break;
        }

        case I2C_ADDRESS_MAP::SOLENOID_ARMED:
            Wire.write(I2C_Registers.solenoid_armed);
            break;
    }
}

}; //namespace Flywheel Controller
