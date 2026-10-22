#pragma once

#include <stdint.h>

#include <DShotRMT.h>

#include "../common/FaultManager/FaultManager.h"

namespace Flywheel_Controller {

static constexpr bool DEBUG = false;

// --------- I2C Settings ---------
static constexpr uint8_t I2C_ADDRESS = 0x62;

static constexpr uint32_t I2C_FREQUENCY = 400000;
static constexpr gpio_num_t SDA_PIN = GPIO_NUM_NC;
static constexpr gpio_num_t SCL_PIN = GPIO_NUM_NC;

enum I2C_ADDRESS_MAP : uint8_t {
    READY = 0x00, //R: tell if the peripheral is booted

    //generic diagnostics: bit (error ID - 1) is set in the error set.
    //  write an active error ID to ERROR_ID before reading its description.
    MASTER_ERROR = 0x01, //R: 1 when any error is active
    ERROR_SET_0 = 0x02, //R: bits 0-7 represent error IDs 1-8
    ERROR_SET_1 = 0x03, //R: bits 0-7 represent error IDs 9-16
    ERROR_SET_2 = 0x04, //R: bits 0-7 represent error IDs 17-24
    ERROR_SET_3 = 0x05, //R: bits 0-7 represent error IDs 25-32
    ERROR_ID = 0x06, //R/W: select an active error ID for description reads
    ERROR_REASON_LENGTH = 0x07, //R: length of the selected error description
    ERROR_REASON_START = 0x08, //R: reads the selected error description (read length bytes from here)

    FLYWHEELS_ARMED = 0x10, //R/W
    RPM_TARGET = 0x11, //R/W (float 4 bytes)
    ACTUAL_RPM = 0x15, //R (float 4 bytes): will always read 0 if bidirectional DShot is not available

    SOLENOID_ARMED = 0x19, //R/W
    SOLENOID_FIRE = 0x1A, //W
};

// --------- Fault Settings ---------
static constexpr uint8_t FAULT_I2C_INIT = 1;
static constexpr uint8_t FAULT_LEFT_ESC_INIT = 2;
static constexpr uint8_t FAULT_RIGHT_ESC_INIT = 3;
static constexpr uint8_t FAULT_SOLENOID_INIT = 4;
static constexpr uint8_t FAULT_LEFT_ESC_SIGNAL = 5;
static constexpr uint8_t FAULT_RIGHT_ESC_SIGNAL = 6;
static constexpr uint8_t FAULT_LEFT_ESC_TELEMETRY = 7;
static constexpr uint8_t FAULT_RIGHT_ESC_TELEMETRY = 8;
static constexpr uint8_t FAULT_INVALID_RPM_TARGET = 9;

using Controller_Fault_Manager = FaultManager<
    FaultCondition<"I2C init", FAULT_I2C_INIT>,
    FaultCondition<"L ESC init", FAULT_LEFT_ESC_INIT>,
    FaultCondition<"R ESC init", FAULT_RIGHT_ESC_INIT>,
    FaultCondition<"Solenoid init", FAULT_SOLENOID_INIT>,
    FaultCondition<"L ESC signal", FAULT_LEFT_ESC_SIGNAL>,
    FaultCondition<"R ESC signal", FAULT_RIGHT_ESC_SIGNAL>,
    FaultCondition<"L ESC telemetry", FAULT_LEFT_ESC_TELEMETRY>,
    FaultCondition<"R ESC telemetry", FAULT_RIGHT_ESC_TELEMETRY>,
    FaultCondition<"Invalid RPM", FAULT_INVALID_RPM_TARGET>
>;

using Controller_Fault_Id = Controller_Fault_Manager::FaultId;

inline Controller_Fault_Manager fault_manager;

static constexpr uint8_t ERROR_REASON_MAX_LENGTH = 127;

// --------- DSHOT Settings ---------
static constexpr gpio_num_t ESC_PINS[2] = {GPIO_NUM_NC, GPIO_NUM_NC};
static constexpr dshot_mode_t DSHOT_MODE = DSHOT600;
static constexpr uint32_t DSHOT_REFRESH_MS = 20;

static constexpr bool DSHOT_BIDIRECTIONAL = true;
static constexpr uint32_t RPM_TELEMETRY_TIMEOUT_MS = 500;

static constexpr uint16_t MOTOR_MAGNET_COUNT = 14;

static constexpr float RPM_PID_KP = 0.002f;
static constexpr float RPM_PID_KI = 0.0005f;
static constexpr float RPM_PID_KD = 0.0f;
static constexpr float RPM_PID_INTEGRAL_LIMIT = 100000.0f;

// --------- Solenoid Settings ---------
static constexpr gpio_num_t SOLENOID_PIN = GPIO_NUM_NC;
static constexpr bool SOLENOID_ACTIVE_HIGH = true;

static constexpr uint32_t SOLENOID_SHOT_TIME_MS = 100;

}; //namespace Flywheel Controller
