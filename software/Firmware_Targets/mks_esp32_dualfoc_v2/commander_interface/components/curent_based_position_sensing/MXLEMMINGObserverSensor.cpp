#include "MXLEMMINGObserverSensor.h"

#include "common/foc_utils.h"
#include "common/time_utils.h"

#include <cmath>

MXLEMMINGObserverSensor::MXLEMMINGObserverSensor(const BLDCMotor& motor) : _motor(motor)
{
    // Derive Flux linkage from KV_rating and pole_pairs
    if (_isset(_motor.pole_pairs) && _isset(_motor.KV_rating)) {
        flux_linkage = 60.0f / (_SQRT3 * _PI * _motor.KV_rating * _motor.pole_pairs * 2.0f);
    }
}

void MXLEMMINGObserverSensor::update()
{
	if (!_motor.current_sense) {
		return;
	}

	if (
		_motor.phase_inductance == 0.0f ||
		_motor.phase_resistance == 0.0f ||
		flux_linkage == 0.0f
	) {
		return;
	}

	if (sensor_cnt++ < sensor_downsample) {
		return;
	}

	sensor_cnt = 0;

	PhaseCurrent_s phase_current = _motor.current_sense->getPhaseCurrents();

	float i_alpha = phase_current.a;
	float i_beta = _1_SQRT3 * phase_current.a + _2_SQRT3 * phase_current.b;

	long now_us = _micros();

	if (angle_prev_ts == 0) {
		angle_prev_ts = now_us;
		i_alpha_prev = i_alpha;
		i_beta_prev = i_beta;
		return;
	}

	float dt = (now_us - angle_prev_ts) * 1e-6f;

	if (dt <= 0.0f || dt > 0.005f) {
		angle_prev_ts = now_us;
		i_alpha_prev = i_alpha;
		i_beta_prev = i_beta;
		return;
	}

	float resistive_term_a = _motor.phase_resistance * i_alpha;
	float resistive_term_b = _motor.phase_resistance * i_beta;

	float inductive_term_a = _motor.phase_inductance * (i_alpha - i_alpha_prev);
	float inductive_term_b = _motor.phase_inductance * (i_beta - i_beta_prev);

	static constexpr float FLUX_CLAMP_MULTIPLIER = 3.0f;

	flux_alpha = _constrain(
		flux_alpha + (_motor.Ualpha - resistive_term_a) * dt - inductive_term_a,
		-flux_linkage * FLUX_CLAMP_MULTIPLIER,
		 flux_linkage * FLUX_CLAMP_MULTIPLIER
	);

	flux_beta = _constrain(
		flux_beta + (_motor.Ubeta - resistive_term_b) * dt - inductive_term_b,
		-flux_linkage * FLUX_CLAMP_MULTIPLIER,
		 flux_linkage * FLUX_CLAMP_MULTIPLIER
	);

	electrical_angle = _normalizeAngle(atan2f(flux_beta, flux_alpha));

	angle_prev = _normalizeAngle(electrical_angle / _motor.pole_pairs);

	i_alpha_prev = i_alpha;
	i_beta_prev = i_beta;
	angle_prev_ts = now_us;
	electrical_angle_prev = electrical_angle;
}

void MXLEMMINGObserverSensor::init()
{
    Sensor::init();
}

float MXLEMMINGObserverSensor::getSensorAngle()
{
    return angle_prev;
}
