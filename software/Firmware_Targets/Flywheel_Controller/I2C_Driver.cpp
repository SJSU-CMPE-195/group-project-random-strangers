#pragma once
#include <Wire.h>

namespace Flywheel_Controller {

struct event_manager {
    unsigned long last_dshot_send_ms = 0;

    bool fire_solenoid = 0;
    bool solenoid_leave_on = 0;
    unsigned long solenoid_off_time = 0;

    bool arm_escs = 0;
    bool disarm_escs = 0;

    bool arm_solenoid = 0;
    bool disarm_solenoid = 0;
} event_manager;

static struct {
    bool ready = 0;
    bool master_error = 0;

    bool flywheels_armed = 0;
    float RPM_target = 0.0f;
    float RPM_actual = 0.0f;
    uint8_t ESC_error = 0;

    bool solenoid_armed = 0;
    bool solenoid_fire = 0;
    uint8_t solenoid_error = 0;
} I2C_Registers;

volatile uint8_t active_register_address;

//I2C Controller write
void receiveEvent(int howMany) {
  if (howMany < 1) return; // No data sent

  //get target Register Address
  active_register_address = Wire.read(); 

  //If more data is available, write that data to the register
  while (Wire.available()) {
    uint8_t incoming_data = Wire.read();

    switch (active_register_address){
        case I2C_ADDRESS_MAP::FLYWHEELS_ARMED:
            if(incomming_data){ //arm requested
                event_manager.arm_escs = true;
            } else { //disarm requested
                event_manager.disarm_escs = true;
            }
            break;

        case I2C_ADDRESS_MAP::RPM_TARGET:
            uint8_t* RPM_TARGET_REG_BYTE_ACCESSOR = &I2C_REGISTERS.RPM_TARGET;
            RPM_TARGET_REG_BYTE_ACCESSOR[0] = incoming_data
            break;

        case I2C_ADDRESS_MAP::RPM_TARGET + 1:
            uint8_t* RPM_TARGET_REG_BYTE_ACCESSOR = &I2C_REGISTERS.RPM_TARGET;
            RPM_TARGET_REG_BYTE_ACCESSOR[1] = incoming_data
            break;

        case I2C_ADDRESS_MAP::RPM_TARGET + 2:
            uint8_t* RPM_TARGET_REG_BYTE_ACCESSOR = &I2C_REGISTERS.RPM_TARGET;
            RPM_TARGET_REG_BYTE_ACCESSOR[2] = incoming_data
            break;

        case I2C_ADDRESS_MAP::RPM_TARGET + 3:
            uint8_t* RPM_TARGET_REG_BYTE_ACCESSOR = &I2C_REGISTERS.RPM_TARGET;
            RPM_TARGET_REG_BYTE_ACCESSOR[3] = incoming_data
            break;

        case I2C_ADDRESS_MAP::SOLENOID_ARMED:
            if(incomming_data){ //arm requested
                event_manager.arm_solenoid = true;
            } else { //disarm requested
                event_manager.disarm_solenoid = true;
            }
            break;

        case I2C_ADDRESS_MAP::SOLENOID_FIRE:
            if(incoming_data){ //fire requested
                event_manager.fire_solenoid = true;
            } else { //reset requested
                event_manager.fire_solenoid = true;
                event_manager.solenoid_leave_on = false;
            }
            break;
        
        default:
            break;
    }

    //increment the pointer to allow for sequential writing
    active_register_address++; 
    }
}

//I2C Controller Read
void requestEvent(){
    switch (active_register_address){
        case I2C_ADDRESS_MAP::READY:
            Wire.slaveWrite(&I2C_Registers.ready, 1);
            break;

        case I2C_ADDRESS_MAP::MASTER_ERROR:
            Wire.slaveWrite(&I2C_Registers.master_error, 1);
            break;

        case I2C_ADDRESS_MAP::FLYWHEELS_ARMED:
            Wire.slaveWrite(&I2C_Registers.flywheels_armed, 1);
            break;

        case I2C_ADDRESS_MAP::RPM_TARGET:
            Wire.slaveWrite(&I2C_Registers.RPM_target, 1);
            break;
        case I2C_ADDRESS_MAP::RPM_TARGET + 1:
            Wire.slaveWrite((uint8_t*)(&I2C_Registers.RPM_target) + 1, 1);
            break;
        case I2C_ADDRESS_MAP::RPM_TARGET + 2:
            Wire.slaveWrite((uint8_t*)(&I2C_Registers.RPM_target) + 2, 1);
            break;
        case I2C_ADDRESS_MAP::RPM_TARGET + 3:
            Wire.slaveWrite((uint8_t*)(&I2C_Registers.RPM_target) + 3, 1);
            break;

        case I2C_ADDRESS_MAP::ACTUAL_RPM:
            Wire.slaveWrite(&I2C_Registers.RPM_actual, 1);
            break;
        case I2C_ADDRESS_MAP::ACTUAL_RPM + 1:
            Wire.slaveWrite((uint8_t*)(&I2C_Registers.RPM_actual) + 1, 1);
            break;
        case I2C_ADDRESS_MAP::ACTUAL_RPM + 2:
            Wire.slaveWrite((uint8_t*)(&I2C_Registers.RPM_actual) + 2, 1);
            break;
        case I2C_ADDRESS_MAP::ACTUAL_RPM + 3:
            Wire.slaveWrite((uint8_t*)(&I2C_Registers.RPM_actual) + 3, 1);
            break;
        
        case I2C_ADDRESS_MAP::ESC_ERROR:
            Wire.slaveWrite(&I2C_Registers.ESC_error, 1);
            break;
        
        case I2C_ADDRESS_MAP::SOLENOID_ARMED:
            Wire.slaveWrite(&I2C_Registers.solenoid_armed, 1);
            break;

        case I2C_ADDRESS_MAP::SOLENOID_ERROR:
            Wire.slaveWrite(&I2C_Registers.solenoid_error, 1);
            break;
        default
            Wire.slaveWrite(0b00000000, 1);
    }
}

}; // Flywheel Controller namespace