#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>


// ================= SENSOR CONFIGURATION =================

// Use ADC1 pins on the ESP32.
// ADC1 pins are preferable because ADC2 conflicts with Wi-Fi.
const int SENSOR_A_PIN = 32;
const int SENSOR_B_PIN = 33;
const int SENSOR_C_PIN = 34;

// approximate sensor range is 1.3–2.1 V.
const float SENSOR_A_OFFSET_MV = 1700.0f;
const float SENSOR_B_OFFSET_MV = 1700.0f;
const float SENSOR_C_OFFSET_MV = 1700.0f;

const float SENSOR_A_AMPLITUDE_MV = 400.0f;
const float SENSOR_B_AMPLITUDE_MV = 400.0f;
const float SENSOR_C_AMPLITUDE_MV = 400.0f;

// Motor pole pairs.
const int POLE_PAIRS = 15;

// Change to -1 if the measured angle moves in the wrong direction.
const float SENSOR_DIRECTION = 1.0f;

// Electrical angle correction.
// Leave at zero initially. Adjust after testing if needed.
const float SENSOR_ELECTRICAL_OFFSET = 0.0f;


// ================= MOTOR CONFIGURATION =================

BLDCMotor motor = BLDCMotor(15, 14.6, 15);

// Driver pins: (PWMA, PWMB, PWMC, ENABLE)
BLDCDriver3PWM driver = BLDCDriver3PWM(18, 19, 4, 21);


// ================= SENSOR STATE =================

// These variables unwrap the repeated electrical angle so that
// the sensor provides a continuous mechanical angle.
float lastElectricalAngle = 0.0f;
float unwrappedElectricalAngle = 0.0f;
bool sensorAngleInitialized = false;


// ================= COMMANDER =================

Commander command = Commander(Serial);

void doMotor(char* cmd)
{
  command.motor(&motor, cmd);
}


// Return the shortest signed angular difference.
float angleDifference(float current, float previous)
{
  float difference = current - previous;

  while (difference > PI) {
    difference -= _2PI;
  }

  while (difference < -PI) {
    difference += _2PI;
  }

  return difference;
}

float readAnalogMillivoltsAveraged(int pin)
{
  const int samples = 4;
  uint32_t total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogReadMilliVolts(pin);
  }

  return total / (float)samples;
}


// This callback is called by GenericSensor whenever SimpleFOC
// needs the current motor angle.
float readAnalogHallAngle()
{
  float sensorA = (float)readAnalogMillivoltsAveraged(SENSOR_A_PIN);
  float sensorB = (float)readAnalogMillivoltsAveraged(SENSOR_B_PIN);
  float sensorC = (float)readAnalogMillivoltsAveraged(SENSOR_C_PIN);

  // Remove each channel's DC offset and normalize amplitude.
  float a = (sensorA - SENSOR_A_OFFSET_MV) / SENSOR_A_AMPLITUDE_MV;
  float b = (sensorB - SENSOR_B_OFFSET_MV) / SENSOR_B_AMPLITUDE_MV;
  float c = (sensorC - SENSOR_C_OFFSET_MV) / SENSOR_C_AMPLITUDE_MV;

  /*
    Clarke-style angle calculation for three evenly spaced signals.

    This assumes the channels are approximately 120 electrical degrees apart.
  */
  float electricalAngle = _atan2(
    1.7320508f * (b - c),
    2.0f * a - b - c
  );

  electricalAngle += SENSOR_ELECTRICAL_OFFSET;
  electricalAngle = _normalizeAngle(electricalAngle);

  if (!sensorAngleInitialized) {
    lastElectricalAngle = electricalAngle;
    unwrappedElectricalAngle = electricalAngle;
    sensorAngleInitialized = true;
  }
  else {
    float delta = angleDifference(
      electricalAngle,
      lastElectricalAngle
    );

    unwrappedElectricalAngle += SENSOR_DIRECTION * delta;
    lastElectricalAngle = electricalAngle;
  }

  /*
    Convert electrical angle to mechanical angle.

    SimpleFOC's motor.pole_pairs value then relates the mechanical
    sensor angle to the motor's electrical angle.
  */
  float mechanicalAngle =
    unwrappedElectricalAngle / (float)POLE_PAIRS;

  return _normalizeAngle(mechanicalAngle);
}


GenericSensor sensor = GenericSensor(readAnalogHallAngle);


// ================= SETUP =================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println(F("Analog three-Hall SimpleFOC controller"));
  Serial.println(F("--------------------------------------"));

  Serial.print(F("Sensor A: GPIO "));
  Serial.println(SENSOR_A_PIN);

  Serial.print(F("Sensor B: GPIO "));
  Serial.println(SENSOR_B_PIN);

  Serial.print(F("Sensor C: GPIO "));
  Serial.println(SENSOR_C_PIN);

  Serial.print(F("Pole pairs: "));
  Serial.println(POLE_PAIRS);

  // The sensor outputs are analog, so do not use INPUT_PULLUP.
  pinMode(SENSOR_A_PIN, INPUT);
  pinMode(SENSOR_B_PIN, INPUT);
  pinMode(SENSOR_C_PIN, INPUT);

  // 11 dB attenuation allows the ESP32 ADC to measure approximately
  // the full 0–3.3 V range.
  analogSetPinAttenuation(SENSOR_A_PIN, ADC_11db);
  analogSetPinAttenuation(SENSOR_B_PIN, ADC_11db);
  analogSetPinAttenuation(SENSOR_C_PIN, ADC_11db);


  // Driver configuration.
  driver.voltage_power_supply = 9.0f;
  driver.enable_active_high = true;
  driver.init();

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);


  /*
    Closed-loop angle control.

    This is a conservative starting configuration.
  */
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::voltage;

  // Start low for safety.
  motor.voltage_limit = 2.0f;

  // Position-loop P gain.
  motor.P_angle.P = 10.0f;

  // Inner velocity-loop PI controller.
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20.0f;
  motor.PID_velocity.D = 0.0f;

  // Smooth velocity measurement.
  motor.LPF_velocity.Tf = 0.01f;

  // Limit maximum speed commanded by the position loop.
  motor.velocity_limit = 2.0f;

  // Limit how quickly the velocity command changes.
  motor.P_angle.output_ramp = 100.0f;


  // Monitoring.
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 100;

  // Initialize the sensor, motor, and FOC.
  sensor.init();

  motor.init();
  motor.initFOC();

  // Add the full motor Commander interface.
  command.add('M', doMotor, "motor");

  motor.target = 0.0f;

  Serial.println();
  Serial.println(F("Motor ready."));
  Serial.println(F("Closed-loop angle control is configured."));
  Serial.println(F("Use the commands listed below."));
}


// ================= MAIN LOOP =================

void loop()
{
  // Run as frequently as possible.
  motor.loopFOC();

  // Run the position and velocity control loops.
  motor.move();

  // Process Commander commands.
  command.run();
}