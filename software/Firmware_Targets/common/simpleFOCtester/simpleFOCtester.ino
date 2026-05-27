//a generic simplefoc tester that exposes the commander interface
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"

//Board Configuration File
#include "../../mks_esp32_dualfoc_v2/board_config.h"

//User Configuration
const float input_voltage = 12.0f;

const unsigned motor0_poles = 7;
const float motor0_resistance = 0.5f;
const int motor0_kv = 90;
const float motor0_phase_inductance = 0.0005;

const unsigned motor1_poles = 7;
const float motor1_resistance = 0.5f;
const int motor1_kv = 90;
const float motor1_phase_inductance = 0.0005;

const float max_motor1_voltage = 12.0f;
const float max_motor1_current = 2.5f;
const float max_motor2_voltage = 12.0f;
const float max_motor2_current = 2.5f;

#define DISABLE_ENCODERS 0;
#define simpleFOCencodertype MXLEMMINGObserverSensor

#ifndef SIMPLEFOC_ENCODER0_INIT
    #define SIMPLEFOC_ENCODER0_INIT {MXLEMMINGObserverSensor(motor0)}
#endif

#ifndef SIMPLEFOC_ENCODER1_INIT
    #define SIMPLEFOC_ENCODER1_INIT {MXLEMMINGObserverSensor(motor1)}
#endif


//Board Configuration
const maximum_current1 = BOARD_PER_CHANNEL_CURRENT_LIMIT > max_motor1_current ? max_motor1_current : BOARD_PER_CHANNEL_CURRENT_LIMIT;
const maximum_current2 = BOARD_PER_CHANNEL_CURRENT_LIMIT > max_motor2_current ? max_motor2_current : BOARD_PER_CHANNEL_CURRENT_LIMIT;

Commander command = Commander(Serial);

#if BOARD_MOTOR_CHANNELS == 1 || BOARD_MOTOR_CHANNELS == 2
        BLDCMotor motor0 = BLDCMotor(motor0_poles, motor0_resistance, motor0_kv, motor0_phase_inductance);
        void doMotor0(char* cmd) {
            command.motor(&motor0, cmd);
        }
    #ifdef BOARD_MOTOR_EN0
        #if BOARD_PWM_TYPE == 3
            BLDCDriver3PWM driver0 = BLDCDriver3PWM(BOARD_MOTOR_A0, BOARD_MOTOR_B0, BOARD_MOTOR_C0, BOARD_MOTOR_EN0);
        #else
            #error "6PWM not supported"
        #endif
    #else
        #if BOARD_PWM_TYPE == 6
            BLDCDriver3PWM driver0 = BLDCDriver3PWM(BOARD_MOTOR_A0, BOARD_MOTOR_B0, BOARD_MOTOR_C0);
        #else
            #error "6PWM not supported"
        #endif
    #endif
#endif

#if BOARD_MOTOR_CHANNELS == 2
    BLDCMotor motor1 = BLDCMotor(motor1_poles, motor1_resistance, motor1_kv, motor1_phase_inductance);
    void doMotor1(char* cmd) {
        command.motor(&motor1, cmd);
    }

    #ifdef BOARD_MOTOR_EN1
        #if BOARD_PWM_TYPE == 6
            BLDCDriver3PWM driver1 = BLDCDriver3PWM(BOARD_MOTOR_A1, BOARD_MOTOR_B1, BOARD_MOTOR_C1, BOARD_MOTOR_EN1);
        #else
            #error "6PWM not supported"
        #endif
    #else
        #if BOARD_PWM_TYPE == 6
            BLDCDriver3PWM driver1 = BLDCDriver3PWM(BOARD_MOTOR_A1, BOARD_MOTOR_B1, BOARD_MOTOR_C1);
        #else
            #error "6PWM not supported"
        #endif
    #endif
#endif

#if BOARD_MOTOR_CHANNELS != 1 && BOARD_MOTOR_CHANNELS != 2
    #error "Board must have 1 or 2 motor channels"
#endif

#if BOARD_CURRENT_SENSE_TYPE == 0
    InlineCurrentSense current_sense0 = InlineCurrentSense(
        BOARD_CURRENT_SENSE_RESISTANCE,
        BOARD_CURRENT_SENSE_GAIN,
        BOARD_CURRENT_SENSE_A0,
        BOARD_CURRENT_SENSE_B0
        );

    #if BOARD_MOTOR_CHANNELS == 2
        InlineCurrentSense current_sense1 = InlineCurrentSense(
            BOARD_CURRENT_SENSE_RESISTANCE,
            BOARD_CURRENT_SENSE_GAIN,
            BOARD_CURRENT_SENSE_A1,
            BOARD_CURRENT_SENSE_B1
            );
    #endif
#elif BOARD_CURRENT_SENSE_TYPE == 1
    LowsideCurrentSense current_sense0 = LowsideCurrentSense(
        BOARD_CURRENT_SENSE_RESISTANCE,
        BOARD_CURRENT_SENSE_GAIN,
        BOARD_CURRENT_SENSE_A0,
        BOARD_CURRENT_SENSE_B0
        );
    #if BOARD_MOTOR_CHANNELS == 2
        LowsideCurrentSense current_sense1 = LowsideCurrentSense(
            BOARD_CURRENT_SENSE_RESISTANCE,
            BOARD_CURRENT_SENSE_GAIN,
            BOARD_CURRENT_SENSE_A1,
            BOARD_CURRENT_SENSE_B1
            );
    #endif
#endif

#if !DISABLE_ENCODERS
    simpleFOCencodertype encoder0 SIMPLEFOC_ENCODER0_INIT;
    #if BOARD_MOTOR_CHANNELS == 2
        simpleFOCencodertype encoder1 SIMPLEFOC_ENCODER1_INIT;
    #endif
#endif

static void setupMotor0(BLDCMotor& motor, BLDCDriver3PWM& driver) {
    driver.voltage_power_supply = input_voltage;
    driver.init();

    motor.linkDriver(&driver);
    motor.voltage_limit = max_motor1_voltage;
    motor.current_limit = maximum_current1;
    motor.controller = MotionControlType::velocity_openloop;

    #if !DISABLE_ENCODERS
        encoder0.init();
        motor.linkSensor(&encoder0);
    #endif

    #if BOARD_CURRENT_SENSE_TYPE == 0 || BOARD_CURRENT_SENSE_TYPE == 1
        current_sense0.linkDriver(&driver);
        current_sense0.init();
        motor.linkCurrentSense(&current_sense0);
    #endif

    motor.init();
    motor.initFOC();
}

#if BOARD_MOTOR_CHANNELS == 2
    static void setupMotor1(BLDCMotor& motor, BLDCDriver3PWM& driver) {
        driver.voltage_power_supply = input_voltage;
        driver.init();

        motor.linkDriver(&driver);
        motor.voltage_limit = max_motor2_voltage;
        motor.current_limit = maximum_current2;
        motor.controller = MotionControlType::velocity_openloop;

        #if !DISABLE_ENCODERS
            encoder1.init();
            motor.linkSensor(&encoder1);
        #endif

        #if BOARD_CURRENT_SENSE_TYPE == 0 || BOARD_CURRENT_SENSE_TYPE == 1
            current_sense1.linkDriver(&driver);
            current_sense1.init();
            motor.linkCurrentSense(&current_sense1);
        #endif

        motor.init();
        motor.initFOC();
    }
#endif

void setup() {
    Serial.begin(115200);

    #if BOARD_MOTOR_CHANNELS == 1 || BOARD_MOTOR_CHANNELS == 2
        setupMotor0(motor0, driver0);
        command.add('M', doMotor0, "motor0");
    #endif

    #if BOARD_MOTOR_CHANNELS == 2
        setupMotor1(motor1, driver1);
        command.add('N', doMotor1, "motor1");
    #endif
}

void loop() {
    #if BOARD_MOTOR_CHANNELS == 1 || BOARD_MOTOR_CHANNELS == 2
        motor0.loopFOC();
        motor0.move();
    #endif

    #if BOARD_MOTOR_CHANNELS == 2
        motor1.loopFOC();
        motor1.move();
    #endif

    command.run();
}

