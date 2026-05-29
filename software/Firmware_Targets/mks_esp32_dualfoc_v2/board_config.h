//Board specific pin definitions for the ESP32 dualFOC V2.0 Board from MKS
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

//configuration
    //motor
    #define BOARD_MOTOR_CHANNELS 2
    #define BOARD_PWM_TYPE 3 //PWM3

    #define BOARD_PER_CHANNEL_CURRENT_LIMIT 2.0f //amps

    //hall sense
    #define BOARD_HALL_INTERFACE_COUNT 2
    #define BOARD_HALL_INTERFACE_TYPE 0 //I2C

    //current sense
    #define BOARD_CURRENT_SENSE_TYPE 0 //INLINE
    #define BOARD_CURRENT_SENSE_RESISTANCE 0.01f
    #define BOARD_CURRENT_SENSE_GAIN 50.0f


//pins
    //motor driver interfaces
    #define BOARD_MOTOR_A0 32
    #define BOARD_MOTOR_B0 33
    #define BOARD_MOTOR_C0 25
    #define BOARD_MOTOR_EN0 12

    #define BOARD_MOTOR_A1 26
    #define BOARD_MOTOR_B1 27
    #define BOARD_MOTOR_C1 14
    #define BOARD_MOTOR_EN1 12

    //hall interfaces
    #define BOARD_HALL_U0 18 //SCL
    #define BOARD_HALL_V0 19 //SDA
    #define BOARD_HALL_W0 15 //Int

    #define BOARD_HALL_U1 5  //SCL
    #define BOARD_HALL_V1 23 //SDA
    #define BOARD_HALL_W1 13 //Int

    //current sense interfaces
    #define BOARD_CURRENT_SENSE_A0 39
    #define BOARD_CURRENT_SENSE_B0 36

    #define BOARD_CURRENT_SENSE_A1 35
    #define BOARD_CURRENT_SENSE_B1 34

#endif