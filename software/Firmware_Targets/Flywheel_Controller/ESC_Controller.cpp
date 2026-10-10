#include "ESC_Controller.h"

#include <Arduino.h>

#include <math.h>

#include "I2C_Driver.h"
#include "config.h"

namespace Flywheel_Controller {

static void reset_rpm_pid(RPM_PID &pid) {
    pid.integral = 0.0f;
    pid.previous_error = 0.0f;
    pid.last_update_ms = 0;
    pid.initialized = false;
}

static float update_rpm_pid(RPM_PID &pid, const float target_rpm, const float actual_rpm, const uint32_t now) {
    if (!isfinite(target_rpm) || target_rpm <= 0.0f) {
        reset_rpm_pid(pid);
        return 0.0f;
    }

    const float delta_time = pid.initialized ? static_cast<float>(now - pid.last_update_ms) / 1000.0f : static_cast<float>(DSHOT_REFRESH_MS) / 1000.0f;
    const float error = target_rpm - actual_rpm;
    const float derivative = pid.initialized && delta_time > 0.0f ? (error - pid.previous_error) / delta_time : 0.0f;

    const float candidate_integral = constrain(pid.integral + error * delta_time, -RPM_PID_INTEGRAL_LIMIT, RPM_PID_INTEGRAL_LIMIT);
    const float unconstrained_output = RPM_PID_KP * error + RPM_PID_KI * candidate_integral + RPM_PID_KD * derivative;

    //prevent the integrator from increasing while the output is saturated.
    const bool output_saturated_high = unconstrained_output >= 100.0f && error > 0.0f;
    const bool output_saturated_low = unconstrained_output <= 0.0f && error < 0.0f;
    if (!output_saturated_high && !output_saturated_low) {
        pid.integral = candidate_integral;
    }

    pid.previous_error = error;
    pid.last_update_ms = now;
    pid.initialized = true;

    //output between 0-100%
    return constrain(RPM_PID_KP * error + RPM_PID_KI * pid.integral + RPM_PID_KD * derivative, 0.0f, 100.0f);
}

static dshot_result_t send_esc_throttle(ESC &esc, const float throttle_percent) {
    if (!isfinite(throttle_percent) || throttle_percent <= 0.0f) {
        return esc.comm.sendThrottle(0);
    }

    return esc.comm.sendThrottlePercent(constrain(throttle_percent, 0.0f, 100.0f));
}

ESC::ESC(const gpio_num_t esc_pin)
    : pin(esc_pin),
      comm(esc_pin, DSHOT_MODE, DSHOT_BIDIRECTIONAL, MOTOR_MAGNET_COUNT) {}

ESC_Controller::ESC_Controller()
    : left_esc(ESC_PINS[0]),
      right_esc(ESC_PINS[1]) {}

ESC_Controller esc_controller;

bool ESC_Controller::begin() {
    bool success = true;

    if (left_esc.pin != GPIO_NUM_NC) {
        const dshot_result_t result = left_esc.comm.begin();
        if (!result.success) {
            dispatch_fault(FAULT_LEFT_ESC_INIT);
            success = false;
            if (DEBUG) Serial.println("Error: Left ESC init failed");
        }
    } else {
        dispatch_fault(FAULT_LEFT_ESC_INIT);
        success = false;
        if (DEBUG) Serial.println("Error: Left ESC control pin is not configured");
    }

    if (right_esc.pin != GPIO_NUM_NC) {
        const dshot_result_t result = right_esc.comm.begin();
        if (!result.success) {
            dispatch_fault(FAULT_RIGHT_ESC_INIT);
            success = false;
            if (DEBUG) Serial.println("Error: Right ESC init failed");
        }
    } else {
        dispatch_fault(FAULT_RIGHT_ESC_INIT);
        success = false;
        if (DEBUG) Serial.println("Error: Right ESC control pin is not configured");
    }

    if (!success) {
        return false;
    }

    initialized = true;
    if (DEBUG) Serial.println("ESCs initialized successfully...");
    return true;
}

void ESC_Controller::force_safe_state() {
    left_esc.armed = false;
    right_esc.armed = false;

    left_esc.throttle_percent = 0.0f;
    right_esc.throttle_percent = 0.0f;

    left_esc.rpm = 0.0f;
    right_esc.rpm = 0.0f;

    reset_rpm_pid(left_rpm_pid);
    reset_rpm_pid(right_rpm_pid);

    I2C_Registers.flywheels_armed = false;
    I2C_Registers.RPM_target = 0.0f;
    I2C_Registers.RPM_actual = 0.0f;

    event_manager.arm_escs = false;
    event_manager.disarm_escs = false;

    if (initialized) {
        left_esc.comm.sendThrottle(0);
        right_esc.comm.sendThrottle(0);
    }
}

void ESC_Controller::handle_arm_events(const uint32_t now) {
    // Disarm takes priority if arm and disarm arrive before this loop runs.
    if (event_manager.disarm_escs) {
        const bool was_armed = left_esc.armed || right_esc.armed;

        left_esc.armed = false;
        right_esc.armed = false;
        left_esc.throttle_percent = 0.0f;
        right_esc.throttle_percent = 0.0f;
        left_esc.rpm = 0.0f;
        right_esc.rpm = 0.0f;
        reset_rpm_pid(left_rpm_pid);
        reset_rpm_pid(right_rpm_pid);

        I2C_Registers.flywheels_armed = false;
        I2C_Registers.RPM_target = 0.0f;
        I2C_Registers.RPM_actual = 0.0f;

        if (was_armed && DEBUG) Serial.println("Flywheel ESCs disarmed; target cleared");
    } else if (event_manager.arm_escs) {
        const bool was_disarmed = !left_esc.armed || !right_esc.armed;

        left_esc.armed = true;
        right_esc.armed = true;
        I2C_Registers.flywheels_armed = true;
        left_esc.last_telemetry_ms = now;
        right_esc.last_telemetry_ms = now;
        left_esc.telemetry_received = false;
        right_esc.telemetry_received = false;
        reset_rpm_pid(left_rpm_pid);
        reset_rpm_pid(right_rpm_pid);

        if (was_disarmed) {
            I2C_Registers.RPM_target = 0.0f;
            if (DEBUG) Serial.println("Flywheel ESCs armed at zero throttle");
        }
    }

    event_manager.arm_escs = false;
    event_manager.disarm_escs = false;
}

void ESC_Controller::mark_dshot_error(const uint8_t fault_id, const char *message) {
    const bool first_error = !fault_manager.is_fault_active(fault_id);
    dispatch_fault(fault_id);

    if (DEBUG && first_error) Serial.println(message);
}

void ESC_Controller::update_throttle(const uint32_t now) {
    const uint32_t previous_send_ms = last_dshot_send_ms;
    last_dshot_send_ms = now;

    if (DSHOT_BIDIRECTIONAL) {
        // Read telemetry from the previous DShot frame.
        const dshot_result_t left_telemetry = left_esc.comm.getTelemetry();
        const dshot_result_t right_telemetry = right_esc.comm.getTelemetry();

        if (left_telemetry.success) {
            left_esc.rpm = static_cast<float>(left_telemetry.motor_rpm);
            left_esc.last_telemetry_ms = now;
            left_esc.telemetry_received = true;
        }
        if (right_telemetry.success) {
            right_esc.rpm = static_cast<float>(right_telemetry.motor_rpm);
            right_esc.last_telemetry_ms = now;
            right_esc.telemetry_received = true;
        }

        I2C_Registers.RPM_actual = (left_esc.rpm + right_esc.rpm) * 0.5f;

        if (left_esc.armed && (!left_esc.telemetry_received || now - left_esc.last_telemetry_ms > RPM_TELEMETRY_TIMEOUT_MS)) {
            mark_dshot_error(FAULT_LEFT_ESC_TELEMETRY, "Error: Left ESC RPM telemetry timed out");
        }

        if (right_esc.armed && (!right_esc.telemetry_received || now - right_esc.last_telemetry_ms > RPM_TELEMETRY_TIMEOUT_MS)) {
            mark_dshot_error(FAULT_RIGHT_ESC_TELEMETRY, "Error: Right ESC RPM telemetry timed out");
        }

        if (left_esc.armed && right_esc.armed && !I2C_Registers.master_error) {
            float target_rpm = I2C_Registers.RPM_target;
            if (!isfinite(target_rpm) || target_rpm < 0.0f) {
                dispatch_fault(FAULT_INVALID_RPM_TARGET);
                target_rpm = 0.0f;
                I2C_Registers.RPM_target = 0.0f;
            }

            left_esc.throttle_percent = update_rpm_pid(left_rpm_pid, target_rpm, left_esc.rpm, now);
            right_esc.throttle_percent = update_rpm_pid(right_rpm_pid, target_rpm, right_esc.rpm, now);
        }
    } else {
        // Standard DShot has no RPM feedback; interpret target as 0-100% throttle.
        float throttle_percent = I2C_Registers.RPM_target;
        if (!isfinite(throttle_percent)) throttle_percent = 0.0f;
        throttle_percent = constrain(throttle_percent, 0.0f, 100.0f);

        left_esc.throttle_percent = left_esc.armed ? throttle_percent : 0.0f;
        right_esc.throttle_percent = right_esc.armed ? throttle_percent : 0.0f;
        I2C_Registers.RPM_actual = 0.0f;
    }

    if (I2C_Registers.master_error || !left_esc.armed || !right_esc.armed) {
        left_esc.throttle_percent = 0.0f;
        right_esc.throttle_percent = 0.0f;
    }

    const dshot_result_t left_result = send_esc_throttle(left_esc, left_esc.throttle_percent);

    if (!left_result.success) {
        mark_dshot_error(FAULT_LEFT_ESC_SIGNAL, "Error: Left ESC DShot refresh failed; entering safe state");
        left_esc.armed = false;
    }

    if (I2C_Registers.master_error) right_esc.throttle_percent = 0.0f;
    const dshot_result_t right_result = send_esc_throttle(right_esc, right_esc.throttle_percent);

    if (!right_result.success) {
        mark_dshot_error(FAULT_RIGHT_ESC_SIGNAL, "Error: Right ESC DShot refresh failed; entering safe state");
        right_esc.armed = false;
    }

    if (I2C_Registers.master_error) force_safe_state();

    if (DEBUG && (now / 1000 != previous_send_ms / 1000)) {
        Serial.print("ESC RPM L/R: ");
        Serial.print(left_esc.rpm);
        Serial.print(" / ");
        Serial.print(right_esc.rpm);
        Serial.print("; throttle % L/R: ");
        Serial.print(left_esc.throttle_percent);
        Serial.print(" / ");
        Serial.println(right_esc.throttle_percent);
    }
}

void ESC_Controller::update(const uint32_t now) {
    if (I2C_Registers.master_error) {
        if (now - last_dshot_send_ms >= DSHOT_REFRESH_MS) {
            last_dshot_send_ms = now;
            force_safe_state();
        }
        return;
    }

    handle_arm_events(now);

    if (now - last_dshot_send_ms >= DSHOT_REFRESH_MS) {
        update_throttle(now);
    }
}

}; //namespace Flywheel Controller
