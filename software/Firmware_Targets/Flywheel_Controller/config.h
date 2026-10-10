#pragma once
#include <stdint.h>
#include <DShotRMT.h>

namespace Flywheel_Controller{

static constexpr bool DEBUG = false;

// --------- I2C Settings ---------
static constexpr uint8_t I2C_ADDRESS = 0x62;

static constexpr uint32_t I2C_FREQUENCY = 400000;
static constexpr gpio_num_t SDA_PIN = GPIO_NUM_NC;
static constexpr gpio_num_t SCL_PIN = GPIO_NUM_NC;

enum class I2C_ADDRESS_MAP {
    READY = 0x0, //R: tell if the flywheel controller is booted
    MASTER_ERROR = 0x1, //R

    FLYWHEELS_ARMED = 0x2, //R/W
    RPM_TARGET = 0x3, //R/W (float 4 bytes)
    ACTUAL_RPM = 0x7, //R (float 4 bytes): will always read 0 if bidirectional DShot is not available
    ESC_ERROR = 0x11, //R

    SOLENOID_ARMED = 0x12, //R/W
    SOLENOID_FIRE = 0x13, //W
    SOLENOID_ERROR = 0x14, //R
} I2C_Address_Map;

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
