
#include "I2C_Driver.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace Gimbal_Controller {

//Define shared registers
I2C_Registers_t I2C_Registers;

//Track selected register
static volatile uint8_t active_register_address = 0;


//INITIALIZE I2C

bool initialize_i2c() {

    //Configure ESP32 as I2C peripheral
    if (!Wire.begin(
            I2C_ADDRESS,
            SDA_PIN,
            SCL_PIN,
            I2C_FREQUENCY)) {

        return false;
    }

    //Attach communication callbacks
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    return true;
}


//RECEIVE FROM JETSON

void receiveEvent(int howMany) {

    if (howMany < 1 || !Wire.available()) {
        return;
    }

    // First byte selects register
    active_register_address = Wire.read();

    // No writable control registers in this version.
    // Discard additional bytes.
    while (Wire.available()) {
        Wire.read();
    }
}


//RESPOND TO JETSON

void requestEvent() {

    uint8_t value = 0;

    switch (active_register_address) {

        case I2C_ADDRESS_MAP::READY:
            value = I2C_Registers.ready;
            break;

        case I2C_ADDRESS_MAP::STATUS:
            value = I2C_Registers.status;
            break;

        case I2C_ADDRESS_MAP::TEST_VALUE:
            value = I2C_Registers.test_value;
            break;

        case I2C_ADDRESS_MAP::SENSOR_OK:
            value = I2C_Registers.sensor_ok;
            break;

        case I2C_ADDRESS_MAP::ELECTRICAL_ANGLE:
            value = I2C_Registers.electrical_angle;
            break;

        default:
            value = 0;
            break;
    }

    // Send single-byte response
    Wire.write(value);
}

} // namespace Gimbal_Controller
