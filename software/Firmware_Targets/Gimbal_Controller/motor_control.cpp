//Gives motor parameters and creates the SimpleFOC objects
//Also configures the intended angle-control mode and SVPWM setting
//Settings are stored but not executed by this file


#include "motor_control.h"
#include "hall_sensor.h"

#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>


// ======================================
// ORIGINAL MOTOR CONFIGURATION
// ======================================

// 15 pole pairs
// 14.6 ohm phase resistance
// 15 KV rating
static BLDCMotor motor(15, 14.6f, 15.0f);

// SimpleFOC Mini:
// PWM A = 18
// PWM B = 19
// PWM C = 4
// ENABLE = 21
static BLDCDriver3PWM driver(18, 19, 4, 21);

// Connect Hall calculation to SimpleFOC
static GenericSensor sensor(readHallAngle);

// Sensor-monitoring readiness
static bool monitoringReady = false;

// ======================================
// INITIALIZE SENSOR MONITORING
// ======================================

bool initializeMotorControl() {

    monitoringReady = false;

    // Set the intended SimpleFOC control modes
    motor.controller = MotionControlType::angle;
    motor.torque_controller =
        TorqueControlType::voltage;

    motor.foc_modulation =
        FOCModulationType::SpaceVectorPWM;

    // Original configuration parameters
    motor.voltage_limit = 2.0f;
    motor.velocity_limit = 2.0f;

    motor.P_angle.P = 10.0f;

    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 20.0f;
    motor.PID_velocity.D = 0.0f;

    motor.LPF_velocity.Tf = 0.01f;
    motor.P_angle.output_ramp = 100.0f;

    // Connect sensor object to motor object
    motor.linkSensor(&sensor);

    // Connect driver object to motor object
    motor.linkDriver(&driver);

    // Initialize ONLY the sensor interface
    sensor.init();

    // Read initial position
    sensor.update();

    float angle = sensor.getAngle();

    if (!isfinite(angle)) {
        return false;
    }

    monitoringReady = true;
    return true;
}

// ======================================
// UPDATE SENSOR MEASUREMENT
// ======================================

void updateMotorControl() {

    if (!monitoringReady) {
        return;
    }

    // Updates GenericSensor measurement
    // No motor PWM is generated
    sensor.update();
}

// ======================================
// GET MEASURED ANGLE
// ======================================

float getMotorAngle() {

    if (!monitoringReady) {
        return NAN;
    }

    return sensor.getAngle();
}

// ======================================
// GET MONITORING STATE
// ======================================

bool isMotorReady() {
    return monitoringReady;
}
