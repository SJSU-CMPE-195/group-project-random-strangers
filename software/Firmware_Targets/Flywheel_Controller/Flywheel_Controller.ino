//This firmware is designed to run on an esp32 based microcontroller board using the Arduino framework
// It is in charge of controlling the flywheel ESCs based on I2C commands from the Jetson
// It controls two bluejay ESCs using bidirectional DSHOT and maintains precise RPM targets using a PID loop 

#include <Arduino.h>
#include <Wire.h>
#include <DShotRMT.h>
#include <stdint.h>

#include "config.h"
#include "I2C_Driver.cpp"

using namespace Flywheel_Controller;

struct ESC {
    gpio_num_t pin = GPIO_NUM_NC;
    DShotRMT comm;
    bool armed = false;
    bool current_direction = false; //false -> normal, true -> reverse
    float throttle_percent = 0.0f;
    float rpm = 0.0f;
    uint32_t last_telemetry_ms = 0;
    bool telemetry_received = false;

    struct RPM_PID {
        float integral = 0.0f;
        float previous_error = 0.0f;
        uint32_t last_update_ms = 0;
    };

    explicit ESC(gpio_num_t esc_pin): 
        pin(esc_pin), comm(esc_pin, DSHOT_MODE, BIDIRECTIONAL_MODE, ESC_MOTOR_MAGNET_COUNT),
        armed(false), current_direction(false), throttle_percent(0.0f), rpm(0.0f),
        last_telemetry_ms(0), telemetry_received(false) {}
};

static void reset_rpm_pid(RPM_PID &pid) {
    pid.integral = 0.0f;
    pid.previous_error = 0.0f;
    pid.last_update_ms = 0;
    pid.initialized = false;
}

static float update_rpm_pid(RPM_PID &pid, const float& target_rpm, const float& actual_rpm, const uint32_t& now) {
    if (target_rpm <= 0.0f) {
        pid = {0, 0, 0, 0};
        return 0.0f;
    }

    const float time_delta = pid.initialized ? static_cast<float>(now - pid.last_update_ms) / 1000.0f : static_cast<float>(DSHOT_REFRESH_MS) / 1000.0f;
    const float error = target_rpm - actual_rpm;
    const float derivative = pid.initialized && dt > 0.0f ? (error - pid.previous_error) / dt : 0.0f;

    const float candidate_integral = constrain(pid.integral + error * dt, -RPM_PID_INTEGRAL_LIMIT, RPM_PID_INTEGRAL_LIMIT);
    const float unconstrained_output = RPM_PID_KP * error + RPM_PID_KI * candidate_integral + RPM_PID_KD * derivative;

    //prevent the integrator from oversaturating
    if (!((unconstrained_output > 100.0f && error > 0.0f) || (unconstrained_output < 0.0f && error < 0.0f))) {
        pid.integral = candidate_integral;
    }

    pid.previous_error = error;
    pid.last_update_ms = now;
    pid.initialized = true;
    return constrain(RPM_PID_KP * error + pid.integral * RPM_PID_KI + RPM_PID_KD * derivative, 0.0f, 100.0f);
}

static dshot_result_t send_esc_throttle(ESC &esc, float throttle_percent) {
    return throttle_percent <= 0.0f ? esc.comm.sendThrottle(0) : esc.comm.sendThrottlePercent(constrain(throttle_percent, 0.0f, 100.0f));
}

void check_reset_master_error(){
    if(I2C_Registers.solenoid_error == 0 && I2C_Registers.ESC_error == 0){
        I2C_Registers.master_error = 0;
    }
}

static ESC left_esc(ESC_PINS[0]);
static ESC right_esc(ESC_PINS[1]);

void setup(){
    if(DEBUG == true){
        Serial.begin(115200);
        delay(500);

        Serial.println("DShotRMT Flywheel ESC Controller")
        Serial.println("Motors and Solenoid off")
        SOLENOID_ACTIVE_HIGH ? Serial.println("Solenoid Active High") : Serial.println("Solenoid Active Low");
        Serial.println("Starting...")
    }

    // --------- initialize I2C ---------
    bool I2C_success = false;
    if(SDA_PIN == GPIO_NUM_NC || SCL_PIN == GPIO_NUM_NC){
        if(DEBUG) Serial.println("I2C Pin(s) are not configured");
    } else {
        if(I2C_ADDRESS > 0x08 && I2C_ADDRESS < 0x78){
            I2C_success |= Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, I2C_FREQUENCY);
        } else if(I2C_ADDRESS > 7F && DEBUG){
            Serial.println("I2C addresses cannot be more than 7 bits");
        } else if(DEBUG){
            Serial.println("I2C address is reserved");
        }
    }

    if(I2C_success == true && DEBUG){
        Serial.println("I2C initialized successfully...");
    }

    // --------- initialize ESCs ---------
    dshot_result_t init_result_left = {0, DSHOT_INIT_FAILED};
    dshot_result_t init_result_right = {0, DSHOT_INIT_FAILED}; 

    if(left_esc.pin != GPIO_NUM_NC){
        init_result_left = left_esc.comm.begin()
    } else if(DEBUG == true){
        Serial.println("Error: Left ESC control pin is not configured")
    }

    if(right_esc.pin != GPIO_NUM_NC){
        init_result_right = right_esc.comm.begin()
    } else if(DEBUG == true){
        Serial.println("Error: Right ESC control pin is not configured")
    }
    
    if(init_result_left.success != true){
        I2C_Registers.master_error = 1;
        I2C_Registers.ESC_error |= 0b00000001;
        
        if(DEBUG) Serial.println("Error: Left ESC init failed")
    }

    if(init_result_right.success != true){
        I2C_Registers.master_error = 1;
        I2C_Registers.ESC_Error |= 0b00000010

        if(DEBUG) Serial.println("Error: Right ESC init failed")
    }

    //ESCs are necessary so halt if they don't work
    if(init_result_left.success != true || init_result_right.sucess != true) while(true);
    if(DEBUG) Serial.println("ESCs initialized successfully...")

    // --------- initialize Solenoid ---------
    I2C_Registers.solenoid_error = 1; //in case the pin manipulation causes problems
    I2C_Registers.master_error = 1;

    digitalWrite(SOLENOID_PIN, !SOLENOID_ACTIVE_HIGH); //disable the solenoid
    pinMode(SOLENOID_PIN, OUTPUT); //enable the solenoid pin
    digitalWrite(SOLENOID_PIN, !SOLENOID_ACTIVE_HIGH);

    I2C_Registers.solenoid_error = 0;
    check_reset_master_error();
    if(DEBUG) Serial.println("Solenoid initialized successfully")

    //Halt on error
    if(I2C_Registers.master_error == true || I2C_success == false){
        if(DEBUG) Serial.println("Initialization Failed. Halting...");
        while(true);
    }

    I2C_Registers.ready == true;
}

void loop(){
    const uint32_t now = millis();
    const bool has_master_error = I2C_Registers.master_error;

    //master error forces everything off and ignores new arm/fire requests.
    if (has_master_error) {
        event_manager.arm_escs = false;
        event_manager.disarm_escs = false;
        left_esc.armed = false;
        right_esc.armed = false;

        I2C_Registers.flywheels_armed = false;
        I2C_Registers.RPM_target = 0.0f;

        I2C_Registers.solenoid_armed = false;
        event_manager.solenoid_leave_on = false;
        event_manager.arm_solenoid = false;
        event_manager.disarm_solenoid = false;
        event_manager.fire_solenoid = false;
    } else {
        // --------- Handle ESC Arming and Disarming ---------
        //disarm takes priority if arm and disarm arrive before this loop runs.
        if (event_manager.disarm_escs) {
            const bool was_armed = left_esc.armed || right_esc.armed;

            left_esc.armed = false;
            right_esc.armed = false;
            left_esc.throttle_percent = 0.0f;
            right_esc.throttle_percent = 0.0f;
            reset_rpm_pid(left_rpm_pid);
            reset_rpm_pid(right_rpm_pid);
            I2C_Registers.flywheels_armed = false;
            I2C_Registers.RPM_target = 0.0f;

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

        // --------- Handle Solenoid Arming and Disarming ---------
        if (event_manager.disarm_solenoid) {
            I2C_Registers.solenoid_armed = false;
            event_manager.solenoid_leave_on = false;
            if (DEBUG) Serial.println("Solenoid disarmed");
        } else if (event_manager.arm_solenoid) {
            I2C_Registers.solenoid_armed = true;
            if (DEBUG) Serial.println("Solenoid armed");
        }
        event_manager.arm_solenoid = false;
        event_manager.disarm_solenoid = false;

        // --------- Hangle Soenoid Firing ---------
        if (event_manager.fire_solenoid) {
            if (I2C_Registers.solenoid_armed && !event_manager.solenoid_leave_on) {
                event_manager.solenoid_leave_on = true;
                event_manager.solenoid_off_time = now + SOLENOID_SHOT_TIME_MS;
                if (DEBUG) Serial.println("Solenoid firing");
            } else if (DEBUG && !I2C_Registers.solenoid_armed) {
                Serial.println("Solenoid fire ignored: solenoid is disarmed");
            }
            event_manager.fire_solenoid = false;
        }

        //(signed subtraction keeps this timeout correct across millis() rollover)
        if (event_manager.solenoid_leave_on && static_cast<int32_t>(now - event_manager.solenoid_off_time) >= 0) {
            event_manager.solenoid_leave_on = false;
            if (DEBUG) Serial.println("Solenoid firing complete");
        }

        digitalWrite(SOLENOID_PIN, (!SOLENOID_ACTIVE_HIGH) ^ (I2C_Registers.solenoid_armed && event_manager.solenoid_leave_on));
    } else {
        digitalWrite(SOLENOID_PIN, !SOLENOID_ACTIVE_HIGH);
    }

    // --------- Handle ESC Throttle ---------

    //Debug message for new target setpoints
    if(DEBUG){ 
        static float last_reported_rpm_target = 0.0f;
        if (I2C_Registers.RPM_target != last_reported_rpm_target) {
            last_reported_rpm_target = I2C_Registers.RPM_target;
            Serial.print(BIDIRECTIONAL_MODE ? "New RPM target: " : "New Throttle target (%): ");
            Serial.println(last_reported_rpm_target);
        }
    }

    //refresh DShot commands and update the PID controllers
    if (now - event_manager.last_dshot_send_ms >= DSHOT_REFRESH_MS) {
        const uint32_t elapsed_ms = now - last_dshot_send_ms;
        last_dshot_send_ms = now;

        if (BIDIRECTIONAL_MODE) {
            //telemetry from last frame
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
                const bool first_error = !(I2C_Registers.ESC_error & 0b00000100);
                I2C_Registers.ESC_error |= 0b00000100;
                if (DEBUG && first_error) {
                    Serial.println("Error: Left ESC RPM telemetry timed out");
                    I2C_Registers.master_error = true;
                }
            }

            if (right_esc.armed && (!right_esc.telemetry_received || now - right_esc.last_telemetry_ms > RPM_TELEMETRY_TIMEOUT_MS)) {
                const bool first_error = !(I2C_Registers.ESC_error & 0b00001000);
                I2C_Registers.ESC_error |= 0b00001000;
                if (DEBUG && first_error) {
                    Serial.println("Error: Right ESC RPM telemetry timed out");
                    I2C_Registers.master_error = true;
                }
            }

            if (left_esc.armed && right_esc.armed && !I2C_Registers.master_error) {
                left_esc.throttle_percent = update_rpm_pid(left_rpm_pid, I2C_Registers.RPM_target, left_esc.rpm, now);
                right_esc.throttle_percent = update_rpm_pid(right_rpm_pid, I2C_Registers.RPM_target, right_esc.rpm, now);
            }
        } else {
            // Standard DShot has no RPM feedback; interpret target as 0-100% throttle.
            const float throttle_percent = constrain(I2C_Registers.RPM_target, 0.0f, 100.0f);
            left_esc.throttle_percent = left_esc.armed ? throttle_percent : 0.0f;
            right_esc.throttle_percent = right_esc.armed ? throttle_percent : 0.0f;
            I2C_Registers.RPM_actual = 0.0f;
        }

        if (I2C_Registers.master_error || !left_esc.armed || !right_esc.armed) {
            left_esc.throttle_percent = 0.0f;
            right_esc.throttle_percent = 0.0f;
        }

        const dshot_result_t left_result = send_esc_throttle(left_esc, left_esc.throttle_percent);
        const dshot_result_t right_result = send_esc_throttle(right_esc, right_esc.throttle_percent);

        if (!left_result.success) {
            const bool first_left_error = !(I2C_Registers.ESC_error & 0b00000001);
            I2C_Registers.ESC_error |= 0b00000001;
            I2C_Registers.master_error = true;
            left_esc.armed = false;
            if (DEBUG && first_left_error) Serial.println("Error: Left ESC DShot refresh failed; entering safe state");
        }
        
        if (!right_result.success) {
            const bool first_right_error = !(I2C_Registers.ESC_error & 0b00000010);
            I2C_Registers.ESC_error |= 0b00000010;
            I2C_Registers.master_error = true;
            right_esc.armed = false;
            if (DEBUG && first_right_error) Serial.println("Error: Right ESC DShot refresh failed; entering safe state");
        }

        if (I2C_Registers.master_error) {
            left_esc.armed = false;
            right_esc.armed = false;
            left_esc.throttle_percent = 0.0f;
            right_esc.throttle_percent = 0.0f;
            I2C_Registers.flywheels_armed = false;
            I2C_Registers.RPM_target = 0.0f;
            I2C_Registers.solenoid_armed = false;
            event_manager.solenoid_leave_on = false;
            digitalWrite(SOLENOID_PIN, !SOLENOID_ACTIVE_HIGH);
        }

        if (DEBUG && (now / 1000 != (now - elapsed_ms) / 1000)) {
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
}
