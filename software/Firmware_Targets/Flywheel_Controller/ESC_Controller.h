#pragma once

#include <stdint.h>

#include <DShotRMT.h>

namespace Flywheel_Controller {

struct RPM_PID {
    float integral = 0.0f;
    float previous_error = 0.0f;
    uint32_t last_update_ms = 0;
    bool initialized = false;
};

struct ESC {
    gpio_num_t pin = GPIO_NUM_NC;
    DShotRMT comm;
    bool armed = false;
    float throttle_percent = 0.0f;
    float rpm = 0.0f;
    uint32_t last_telemetry_ms = 0;
    bool telemetry_received = false;

    explicit ESC(gpio_num_t esc_pin);
};

class ESC_Controller {
public:
    ESC_Controller();

    //initializes ESCs
    bool begin();

    //refreshes ESC communications and PID loops
    void update(uint32_t now);

    //disables ESCs
    void force_safe_state();

private:
    ESC left_esc;
    ESC right_esc;
    RPM_PID left_rpm_pid;
    RPM_PID right_rpm_pid;
    uint32_t last_dshot_send_ms = 0;
    bool initialized = false;

    //I2C Event triggers
    void handle_arm_events(uint32_t now);
    void update_throttle(uint32_t now);
    void mark_dshot_error(uint8_t error_bit, const char *message);
};

extern ESC_Controller esc_controller;

}; //namespace Flywheel Controller
