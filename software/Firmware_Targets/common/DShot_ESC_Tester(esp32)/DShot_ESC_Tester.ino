#include <Arduino.h>
#include <DShotRMT.h>

// ---------- User settings ----------
static constexpr uint32_t SERIAL_BAUD = 115200;

// Change this to the GPIO connected to your ESC DShot signal wire.
static constexpr gpio_num_t ESC_PIN = GPIO_NUM_2;

// Common choices: DSHOT150, DSHOT300, DSHOT600, DSHOT1200.
// DSHOT300 is a conservative default.
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// Keep bidirectional telemetry off unless your wiring and ESC support it.
static constexpr bool BIDIRECTIONAL = false;

// How often to refresh the ESC command.
// DShot ESCs generally expect repeated frames.
static constexpr uint32_t DSHOT_REFRESH_MS = 20;
// -----------------------------------

DShotRMT esc(ESC_PIN, DSHOT_MODE, BIDIRECTIONAL);

bool armed = false;
float dutyPercent = 0.0f;
uint32_t lastDShotSendMs = 0;

void printHelp();
void handleSerialLine(String line);
bool parseNumberOnly(const String &s, float &value);
void sendAndReportThrottle(float percent, const char *reason);
void sendAndReportCommand(dshotCommands_e command, const char *name);
void printResult(const dshot_result_t &result);

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("DShotRMT Serial ESC Controller");
  Serial.println("Motor is OFF by default.");

  dshot_result_t initResult = esc.begin();
  Serial.print("ESC init: ");
  printResult(initResult);

  armed = false;
  dutyPercent = 0.0f;

  // Explicitly command motor stop on boot.
  sendAndReportThrottle(0.0f, "boot safety stop");

  printHelp();
}

void loop() {
  // Read serial commands.
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      handleSerialLine(line);
    }
  }

  // Keep refreshing the ESC.
  const uint32_t now = millis();
  if (now - lastDShotSendMs >= DSHOT_REFRESH_MS) {
    lastDShotSendMs = now;

    if (armed) {
      esc.sendThrottlePercent(dutyPercent);
    } else {
      esc.sendThrottlePercent(0.0f);
    }
  }
}

void handleSerialLine(String line) {
  line.trim();
  line.toLowerCase();

  Serial.print("Received command: ");
  Serial.println(line);

  if (line == "arm") {
    armed = true;
    dutyPercent = 0.0f;
    sendAndReportThrottle(0.0f, "armed at 0%");
    Serial.println("Status: ARMED. Send a number from 0 to 100 to set duty cycle.");
    return;
  }

  if (line == "disarm") {
    armed = false;
    dutyPercent = 0.0f;
    sendAndReportThrottle(0.0f, "disarmed / motor stop");
    Serial.println("Status: DISARMED. Motor command forced to 0%.");
    return;
  }

  if (line == "beep") {
    // Beacon commands are intended for ESC beeping / locating.
    // Keep throttle at zero around the command.
    sendAndReportThrottle(0.0f, "pre-beep safety stop");
    sendAndReportCommand(DSHOT_CMD_BEACON1, "beep / beacon1");
    sendAndReportThrottle(0.0f, "post-beep safety stop");
    return;
  }

  if (line == "help" || line == "h" || line == "?") {
    Serial.println("Command: help");
    printHelp();
    return;
  }

  float requestedDuty = 0.0f;
  if (parseNumberOnly(line, requestedDuty)) {
    if (requestedDuty < 0.0f || requestedDuty > 100.0f) {
      Serial.print("Rejected duty cycle: ");
      Serial.print(requestedDuty, 2);
      Serial.println("% is outside valid range 0-100.");
      return;
    }

    if (!armed) {
      dutyPercent = 0.0f;
      sendAndReportThrottle(0.0f, "ignored duty cycle while disarmed");
      Serial.println("Status: DISARMED. Duty command ignored; motor remains off.");
      return;
    }

    dutyPercent = requestedDuty;
    sendAndReportThrottle(dutyPercent, "duty cycle update");

    Serial.print("Status: ARMED, duty cycle set to ");
    Serial.print(dutyPercent, 2);
    Serial.println("%.");
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(line);
  Serial.println("Valid commands: arm, disarm, beep, help, or a number from 0 to 100.");
}

bool parseNumberOnly(const String &s, float &value) {
  char *endPtr = nullptr;
  value = strtof(s.c_str(), &endPtr);

  // No conversion happened.
  if (endPtr == s.c_str()) {
    return false;
  }

  // Allow trailing spaces only.
  while (*endPtr != '\0') {
    if (!isspace(*endPtr)) {
      return false;
    }
    endPtr++;
  }

  return true;
}

void sendAndReportThrottle(float percent, const char *reason) {
  dshot_result_t result = esc.sendThrottlePercent(percent);

  Serial.print("Throttle command [");
  Serial.print(reason);
  Serial.print("]: ");
  Serial.print(percent, 2);
  Serial.print("% -> ");

  printResult(result);
}

void sendAndReportCommand(dshotCommands_e command, const char *name) {
  dshot_result_t result = esc.sendCommand(command);

  Serial.print("DShot command [");
  Serial.print(name);
  Serial.print("]: ");
  Serial.print((int)command);
  Serial.print(" -> ");

  printResult(result);
}

void printResult(const dshot_result_t &result) {
  Serial.print(result.success ? "OK" : "FAIL");
  Serial.print(" / code=");
  Serial.println((int)result.result_code);
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  arm       - arm ESC, throttle remains 0%");
  Serial.println("  disarm    - force motor off and ignore duty commands");
  Serial.println("  beep      - send DShot beacon/beep command");
  Serial.println("  0-100     - set duty cycle percent, e.g. 12, 12.5, 100");
  Serial.println("  help      - show this menu");
  Serial.println();
}