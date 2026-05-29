#ifndef MXLEMMING_OBSERVER_SENSOR_H
#define MXLEMMING_OBSERVER_SENSOR_H

#include "BLDCMotor.h"
#include "common/base_classes/Sensor.h"

class MXLEMMINGObserverSensor : public Sensor {
public:
    explicit MXLEMMINGObserverSensor(const BLDCMotor& motor);

    void update() override;
    void init() override;
    float getSensorAngle() override;

    // For sensors with slow communication, use these to poll less often
    unsigned int sensor_downsample = 0; // ratio of downsampling for sensor update
    unsigned int sensor_cnt = 0;        // counting variable for downsampling
    float flux_alpha = 0.0f;            // Flux Alpha
    float flux_beta = 0.0f;             // Flux Beta
    float flux_linkage = 0.0f;          // Flux linkage, calculated based on KV and pole number
    float i_alpha_prev = 0.0f;          // Previous Alpha current
    float i_beta_prev = 0.0f;           // Previous Beta current
    float electrical_angle = 0.0f;      // Electrical angle
    float electrical_angle_prev = 0.0f; // Previous electrical angle
    float angle_track = 0.0f;           // Total Electrical angle

protected:
    const BLDCMotor& _motor;
};

#endif
