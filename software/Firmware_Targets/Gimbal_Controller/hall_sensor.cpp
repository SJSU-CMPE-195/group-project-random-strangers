//Contains the ADC initialization signal calibration, 
//angle calculation, and sensor validity checks

#include "hall_sensor.h"

#include <Arduino.h>
#include <math.h>

// ======================================
// SENSOR CONFIGURATION
// ======================================

static constexpr int SENSOR_A_PIN = 32;
static constexpr int SENSOR_B_PIN = 33;
static constexpr int SENSOR_C_PIN = 34;

static constexpr float A_OFFSET_MV = 1700.0f;
static constexpr float B_OFFSET_MV = 1700.0f;
static constexpr float C_OFFSET_MV = 1700.0f;

static constexpr float A_AMPLITUDE_MV = 400.0f;
static constexpr float B_AMPLITUDE_MV = 400.0f;
static constexpr float C_AMPLITUDE_MV = 400.0f;

static constexpr int POLE_PAIRS = 15;

static constexpr float SENSOR_DIRECTION = 1.0f;
static constexpr float ELECTRICAL_OFFSET = 0.0f;

static constexpr int ADC_SAMPLES = 4;
static constexpr float TWO_PI = 2.0f * PI;

// ======================================
// SENSOR STATE
// ======================================

static float lastElectricalAngle = 0.0f;
static float unwrappedElectricalAngle = 0.0f;
static bool angleInitialized = false;
static bool sensorConfigured = false;

// ======================================
// NORMALIZE ANGLE TO [0, 2*PI)
// ======================================

static float normalizeAngle(float angle) {

    angle = fmodf(angle, TWO_PI);

    if (angle < 0.0f) {
        angle += TWO_PI;
    }

    return angle;
}

// ======================================
// SHORTEST ANGULAR DIFFERENCE
// ======================================

static float angleDifference(
    float current,
    float previous
) {

    float difference = current - previous;

    while (difference > PI) {
        difference -= TWO_PI;
    }

    while (difference < -PI) {
        difference += TWO_PI;
    }

    return difference;
}

// ======================================
// AVERAGE ADC READINGS
// ======================================

static float readAnalogMillivoltsAveraged(int pin) {

    uint32_t total = 0;

    for (int i = 0; i < ADC_SAMPLES; i++) {
        total += analogReadMilliVolts(pin);
    }

    return total / (float)ADC_SAMPLES;
}

// ======================================
// INITIALIZE ANALOG HALL SENSORS
// ======================================

bool initializeHallSensor() {

    sensorConfigured = false;

    pinMode(SENSOR_A_PIN, INPUT);
    pinMode(SENSOR_B_PIN, INPUT);
    pinMode(SENSOR_C_PIN, INPUT);

    analogSetPinAttenuation(
        SENSOR_A_PIN, ADC_11db
    );

    analogSetPinAttenuation(
        SENSOR_B_PIN, ADC_11db
    );

    analogSetPinAttenuation(
        SENSOR_C_PIN, ADC_11db
    );

    lastElectricalAngle = 0.0f;
    unwrappedElectricalAngle = 0.0f;
    angleInitialized = false;

    sensorConfigured = true;

    // Verify initial signal is plausible
    return isfinite(readHallAngle());
}

// ======================================
// READ HALL SENSOR ANGLE
// ======================================

float readHallAngle() {

    if (!sensorConfigured) {
        return NAN;
    }

    // Read analog Hall voltages
    float sensorA =
        readAnalogMillivoltsAveraged(SENSOR_A_PIN);

    float sensorB =
        readAnalogMillivoltsAveraged(SENSOR_B_PIN);

    float sensorC =
        readAnalogMillivoltsAveraged(SENSOR_C_PIN);

    // Remove offsets and normalize amplitudes
    float a =
        (sensorA - A_OFFSET_MV) / A_AMPLITUDE_MV;

    float b =
        (sensorB - B_OFFSET_MV) / B_AMPLITUDE_MV;

    float c =
        (sensorC - C_OFFSET_MV) / C_AMPLITUDE_MV;

    // Clarke-style components
    float x = 2.0f * a - b - c;
    float y = 1.7320508f * (b - c);

    // Reject very weak or invalid signals
    float magnitudeSquared = x*x + y*y;

    if (!isfinite(magnitudeSquared) ||
        magnitudeSquared < 0.09f) {
        return NAN;
    }

    // Calculate electrical angle
    float electricalAngle = atan2f(y, x);

    electricalAngle += ELECTRICAL_OFFSET;
    electricalAngle = normalizeAngle(electricalAngle);

    if (!isfinite(electricalAngle)) {
        return NAN;
    }

    // Unwrap electrical angle
    if (!angleInitialized) {

        lastElectricalAngle = electricalAngle;
        unwrappedElectricalAngle = electricalAngle;

        angleInitialized = true;
    }
    else {

        float delta = angleDifference(
            electricalAngle,
            lastElectricalAngle
        );

        unwrappedElectricalAngle +=
            SENSOR_DIRECTION * delta;

        lastElectricalAngle = electricalAngle;
    }

    // Convert electrical angle to mechanical
    float mechanicalAngle =
        unwrappedElectricalAngle / (float)POLE_PAIRS;

    return normalizeAngle(mechanicalAngle);
}
