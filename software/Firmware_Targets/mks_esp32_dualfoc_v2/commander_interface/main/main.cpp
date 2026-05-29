#include "../../board_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_timer.h"

#include <math.h>

#include <esp_simplefoc.h>
#include <MXLEMMINGObserverSensor.h>
#include <current_sense/InlineCurrentSense.h>

// User parameters
#define DISABLE_ENCODERS 1
#define DISABLE_MXLEMMING_OBSERVER 1
#define DISABLE_CURRENT_SENSE 0
#define DISABLE_MOTOR_0 1
#define DISABLE_MOTOR_1 0

static constexpr float USER_MAX_MOTOR_VOLTAGE = 15.0f;
static constexpr float USER_MAX_MOTOR_CURRENT = 1.0f;

// Observer attach/detach thresholds.
// Detach below the lower threshold.
// Reattach above the upper threshold.
// This hysteresis prevents rapid attach/detach toggling near the cutoff.
static constexpr float OBSERVER_DETACH_VELOCITY = 20.0f;
static constexpr float OBSERVER_ATTACH_VELOCITY = 40.0f;

static constexpr int MOTOR0_POLE_PAIRS = 12;
static constexpr float MOTOR0_RESISTANCE = 9.2f;
static constexpr float MOTOR0_PHASE_INDUCTANCE = 0.001f;
static constexpr float MOTOR0_KV = 30.0f;

static constexpr int MOTOR1_POLE_PAIRS = 12;
static constexpr float MOTOR1_RESISTANCE = 9.2f;
static constexpr float MOTOR1_PHASE_INDUCTANCE = 0.001f;
static constexpr float MOTOR1_KV = 30.0f;

// Dedicated timer periods
static constexpr uint64_t FOC_PERIOD_US = 50;          // 20 kHz
static constexpr uint64_t MOVEMENT_PERIOD_US = 75;    // 20 kHz
static constexpr uint64_t COMMANDER_PERIOD_US = 10000; // 100 Hz

Commander commander = Commander(Serial);

static esp_timer_handle_t foc_timer = nullptr;
static esp_timer_handle_t movement_timer = nullptr;
static esp_timer_handle_t commander_timer = nullptr;

#if !DISABLE_MOTOR_0
	static char motor0_label[] = "motor0";

	BLDCMotor motor0(
		MOTOR0_POLE_PAIRS,
		MOTOR0_RESISTANCE,
		MOTOR0_KV,
		MOTOR0_PHASE_INDUCTANCE
	);

	BLDCDriver3PWM driver0(
		BOARD_MOTOR_A0,
		BOARD_MOTOR_B0,
		BOARD_MOTOR_C0,
		BOARD_MOTOR_EN0
	);

	#if BOARD_CURRENT_SENSE_TYPE == 0 && !DISABLE_CURRENT_SENSE
		InlineCurrentSense currentSense0(
			BOARD_CURRENT_SENSE_RESISTANCE,
			BOARD_CURRENT_SENSE_GAIN,
			BOARD_CURRENT_SENSE_A0,
			BOARD_CURRENT_SENSE_B0
		);
	#endif

	#if !DISABLE_ENCODERS
		HallSensor hall0(
			BOARD_HALL_U0,
			BOARD_HALL_V0,
			BOARD_HALL_W0,
			MOTOR0_POLE_PAIRS
		);

		void doHall0A() { hall0.handleA(); }
		void doHall0B() { hall0.handleB(); }
		void doHall0C() { hall0.handleC(); }
	#elif !DISABLE_MXLEMMING_OBSERVER
		MXLEMMINGObserverSensor observer0(motor0);
		static bool observer0_attached = false;
	#endif

	void doMotor0(char* cmd) {
		commander.motor(&motor0, cmd);
	}
#endif

#if !DISABLE_MOTOR_1
	static char motor1_label[] = "motor1";

	BLDCMotor motor1(
		MOTOR1_POLE_PAIRS,
		MOTOR1_RESISTANCE,
		MOTOR1_KV,
		MOTOR1_PHASE_INDUCTANCE
	);

	BLDCDriver3PWM driver1(
		BOARD_MOTOR_A1,
		BOARD_MOTOR_B1,
		BOARD_MOTOR_C1,
		BOARD_MOTOR_EN1
	);

	#if BOARD_CURRENT_SENSE_TYPE == 0 && !DISABLE_CURRENT_SENSE
		InlineCurrentSense currentSense1(
			BOARD_CURRENT_SENSE_RESISTANCE,
			BOARD_CURRENT_SENSE_GAIN,
			BOARD_CURRENT_SENSE_A1,
			BOARD_CURRENT_SENSE_B1
		);
	#endif

	#if !DISABLE_ENCODERS
		HallSensor hall1(
			BOARD_HALL_U1,
			BOARD_HALL_V1,
			BOARD_HALL_W1,
			MOTOR1_POLE_PAIRS
		);

		void doHall1A() { hall1.handleA(); }
		void doHall1B() { hall1.handleB(); }
		void doHall1C() { hall1.handleC(); }
	#elif !DISABLE_MXLEMMING_OBSERVER
		MXLEMMINGObserverSensor observer1(motor1);
		static bool observer1_attached = false;
	#endif

	void doMotor1(char* cmd) {
		commander.motor(&motor1, cmd);
	}
#endif


#if !DISABLE_MOTOR_0 && DISABLE_ENCODERS && !DISABLE_MXLEMMING_OBSERVER
static void update_motor0_observer_attachment() {
	const float velocity = observer1_attached
		? fabsf(motor0.shaft_velocity)
		: fabsf(motor0.target); // replace with requested velocity if target is not velocity

	if (observer0_attached && velocity < OBSERVER_DETACH_VELOCITY) {
		motor0.linkSensor(nullptr);
		observer0_attached = false;
		return;
	}

	if (!observer0_attached && velocity > OBSERVER_ATTACH_VELOCITY) {
		motor0.linkSensor(&observer0);
		observer0_attached = true;

		// Give the sensor timing state a clean start if supported by your Sensor class.
		observer0.init();
		return;
	}
}
#endif

#if !DISABLE_MOTOR_1 && DISABLE_ENCODERS && !DISABLE_MXLEMMING_OBSERVER
static void update_motor1_observer_attachment() {
	const float velocity = observer1_attached
		? fabsf(motor1.shaft_velocity)
		: fabsf(motor1.target); // replace with requested velocity if target is not velocity

	if (observer1_attached && velocity < OBSERVER_DETACH_VELOCITY) {
		motor1.linkSensor(nullptr);
		observer1_attached = false;
		return;
	}

	if (!observer1_attached && velocity > OBSERVER_ATTACH_VELOCITY) {
		motor1.linkSensor(&observer1);
		observer1_attached = true;

		// Give the sensor timing state a clean start if supported by your Sensor class.
		observer1.init();
		return;
	}
}
#endif

static void init_motors() {
	Serial.begin(115200);

	SimpleFOCDebug::enable(&Serial);

	#if !DISABLE_MOTOR_0
		motor0.voltage_sensor_align = min(2.0f, USER_MAX_MOTOR_VOLTAGE);

		driver0.voltage_power_supply = USER_MAX_MOTOR_VOLTAGE;
		driver0.pwm_frequency = 50000;
		driver0.init();

		motor0.linkDriver(&driver0);

		motor0.voltage_limit = USER_MAX_MOTOR_VOLTAGE;
		motor0.current_limit = USER_MAX_MOTOR_CURRENT;

		motor0.useMonitoring(Serial);
		motor0.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_VOLT_Q | _MON_CURR_Q;

		#if BOARD_CURRENT_SENSE_TYPE == 0 && !DISABLE_CURRENT_SENSE
			currentSense0.linkDriver(&driver0);
			currentSense0.init();
			motor0.linkCurrentSense(&currentSense0);
		#endif

		#if !DISABLE_ENCODERS
			hall0.init();
			hall0.enableInterrupts(doHall0A, doHall0B, doHall0C);
			motor0.linkSensor(&hall0);
		#elif !DISABLE_MXLEMMING_OBSERVER
			motor0.sensor_direction = Direction::CW;
			motor0.zero_electric_angle = 0;

			// Start detached. It will attach once the requested velocity exceeds
			// OBSERVER_ATTACH_VELOCITY.
			motor0.linkSensor(nullptr);
			observer0_attached = false;
		#endif

		motor0.init();
		motor0.disable();

		#if !DISABLE_ENCODERS
			motor0.initFOC();
		#elif !DISABLE_MXLEMMING_OBSERVER
			// Temporarily attach for FOC initialization, then detach again.
			motor0.linkSensor(&observer0);
			observer0_attached = true;

			motor0.initFOC();

			motor0.linkSensor(nullptr);
			observer0_attached = false;
		#endif

		commander.add('A', doMotor0, motor0_label);
	#endif

	#if !DISABLE_MOTOR_1
		motor1.voltage_sensor_align = min(2.0f, USER_MAX_MOTOR_VOLTAGE);

		driver1.voltage_power_supply = USER_MAX_MOTOR_VOLTAGE;
		driver1.pwm_frequency = 50000;
		driver1.init();

		motor1.linkDriver(&driver1);

		motor1.voltage_limit = USER_MAX_MOTOR_VOLTAGE;
		motor1.current_limit = USER_MAX_MOTOR_CURRENT;

		motor1.useMonitoring(Serial);
		motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_VOLT_Q | _MON_CURR_Q;

		#if BOARD_CURRENT_SENSE_TYPE == 0 && !DISABLE_CURRENT_SENSE
			currentSense1.skip_align = true;
			currentSense1.linkDriver(&driver1);
			currentSense1.init();
			motor1.linkCurrentSense(&currentSense1);
		#endif

		#if !DISABLE_ENCODERS
			hall1.init();
			hall1.enableInterrupts(doHall1A, doHall1B, doHall1C);
			motor1.linkSensor(&hall1);
		#elif !DISABLE_MXLEMMING_OBSERVER
			motor1.sensor_direction = Direction::CW;
			motor1.zero_electric_angle = 0;

			// Start detached. It will attach once the requested velocity exceeds
			// OBSERVER_ATTACH_VELOCITY.
			motor1.linkSensor(nullptr);
			observer1_attached = false;
		#endif

		motor1.init();
		motor1.disable();

		#if !DISABLE_ENCODERS
			motor1.initFOC();
		#elif !DISABLE_MXLEMMING_OBSERVER
			// Temporarily attach for FOC initialization, then detach again.
			motor1.linkSensor(&observer1);
			observer1_attached = true;

			motor1.initFOC();

			motor1.linkSensor(nullptr);
			observer1_attached = false;
		#endif

		commander.add('B', doMotor1, motor1_label);
	#endif
}

static void foc_timer_callback(void* /*arg*/) {
	#if !DISABLE_MOTOR_0
		#if DISABLE_ENCODERS && !DISABLE_MXLEMMING_OBSERVER
			update_motor0_observer_attachment();
		#endif

		motor0.loopFOC();
	#endif

	#if !DISABLE_MOTOR_1
		#if DISABLE_ENCODERS && !DISABLE_MXLEMMING_OBSERVER
			update_motor1_observer_attachment();
		#endif

		motor1.loopFOC();
	#endif
}

static void movement_timer_callback(void* /*arg*/) {
	#if !DISABLE_MOTOR_0
		motor0.move();
	#endif

	#if !DISABLE_MOTOR_1
		motor1.move();
	#endif
}

static void commander_timer_callback(void* /*arg*/) {
	#if !DISABLE_MOTOR_0
		//motor0.monitor();
	#endif

	#if !DISABLE_MOTOR_1
		//motor1.monitor();
	#endif

	commander.run();
}

static void start_motor_timers() {
	esp_timer_create_args_t foc_timer_args = {};
	foc_timer_args.callback = &foc_timer_callback;
	foc_timer_args.arg = nullptr;
	foc_timer_args.dispatch_method = ESP_TIMER_TASK;
	foc_timer_args.name = "foc_20khz";

	esp_timer_create_args_t movement_timer_args = {};
	movement_timer_args.callback = &movement_timer_callback;
	movement_timer_args.arg = nullptr;
	movement_timer_args.dispatch_method = ESP_TIMER_TASK;
	movement_timer_args.name = "movement_5khz";

	esp_timer_create_args_t commander_timer_args = {};
	commander_timer_args.callback = &commander_timer_callback;
	commander_timer_args.arg = nullptr;
	commander_timer_args.dispatch_method = ESP_TIMER_TASK;
	commander_timer_args.name = "commander_100hz";

	ESP_ERROR_CHECK(esp_timer_create(&foc_timer_args, &foc_timer));
	ESP_ERROR_CHECK(esp_timer_create(&movement_timer_args, &movement_timer));
	ESP_ERROR_CHECK(esp_timer_create(&commander_timer_args, &commander_timer));

	ESP_ERROR_CHECK(esp_timer_start_periodic(foc_timer, FOC_PERIOD_US));
	ESP_ERROR_CHECK(esp_timer_start_periodic(movement_timer, MOVEMENT_PERIOD_US));
	ESP_ERROR_CHECK(esp_timer_start_periodic(commander_timer, COMMANDER_PERIOD_US));
}

static void motor_init_task(void* /*arg*/) {
	init_motors();
	start_motor_timers();

	vTaskDelete(nullptr);
}

extern "C" void app_main(void) {
	xTaskCreatePinnedToCore(
		motor_init_task,
		"motor_init_task",
		8192,
		nullptr,
		3,
		nullptr,
		1
	);
}