#include <Arduino.h>
#include <DShotRMT.h>

// ---------- User settings ----------
static constexpr uint32_t SERIAL_BAUD = 115200;

// Change these to the GPIOs connected to your ESC DShot signal wires.
// Use GPIO_NUM_NC to disable an ESC slot.
static constexpr gpio_num_t ESC1_PIN = GPIO_NUM_2;
static constexpr gpio_num_t ESC2_PIN = GPIO_NUM_NC;

// Common choices: DSHOT150, DSHOT300, DSHOT600, DSHOT1200.
// DSHOT300 is a conservative default.
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// Keep bidirectional telemetry off unless your wiring and ESC support it.
static constexpr bool BIDIRECTIONAL = false;

// How often to refresh the ESC command.
// DShot ESCs generally expect repeated frames.
static constexpr uint32_t DSHOT_REFRESH_MS = 20;

// DShot command values for spin direction.
static constexpr dshotCommands_e DSHOT_CMD_DIR_FWD = static_cast<dshotCommands_e>(20);
static constexpr dshotCommands_e DSHOT_CMD_DIR_REV = static_cast<dshotCommands_e>(21);
// -----------------------------------

struct EscState {
  DShotRMT *esc = nullptr;
  bool armed = false;
  float dutyPercent = 0.0f;
};

EscState esc1;
EscState esc2;
uint32_t lastDShotSendMs = 0;

void printHelp();
void handleSerialLine(String line);
bool parseSelector(String &line, EscState *&targetA, EscState *&targetB, String &label);
bool parseNumberOnly(const String &s, float &value);
void sendAndReportThrottle(EscState &escState, float percent, const char *reason, const char *label);
void sendAndReportCommand(EscState &escState, dshotCommands_e command, const char *name, const char *label);
void printResult(const dshot_result_t &result);
void initEsc(EscState &escState, gpio_num_t pin, const char *label);
bool hasEsc(const EscState &escState);
int enabledEscCount();

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("DShotRMT Serial ESC Controller");
  Serial.println("Motor is OFF by default.");

  initEsc(esc1, ESC1_PIN, "M1");
  initEsc(esc2, ESC2_PIN, "M2");

  if (enabledEscCount() == 0) {
    Serial.println("ERROR: No ESCs are enabled. Set ESC1_PIN or ESC2_PIN to a valid GPIO.");
  }

  // Explicitly command motor stop on boot.
  if (hasEsc(esc1)) {
    sendAndReportThrottle(esc1, 0.0f, "boot safety stop", "M1");
  }
  if (hasEsc(esc2)) {
    sendAndReportThrottle(esc2, 0.0f, "boot safety stop", "M2");
  }

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

    if (hasEsc(esc1)) {
      esc1.esc->sendThrottlePercent(esc1.armed ? esc1.dutyPercent : 0.0f);
    }
    if (hasEsc(esc2)) {
      esc2.esc->sendThrottlePercent(esc2.armed ? esc2.dutyPercent : 0.0f);
    }
  }
}

void handleSerialLine(String line) {
  line.trim();
  line.toLowerCase();

  EscState *targetA = nullptr;
  EscState *targetB = nullptr;
  String selectorLabel = "B";
  if (!parseSelector(line, targetA, targetB, selectorLabel)) {
    return;
  }

  Serial.print("Received command [");
  Serial.print(selectorLabel);
  Serial.print("]: ");
  Serial.println(line);

  if (line.length() == 0) {
    Serial.println("Missing command after selector.");
    return;
  }

  if (line == "arm") {
    if (targetA && hasEsc(*targetA)) {
      targetA->armed = true;
      targetA->dutyPercent = 0.0f;
      sendAndReportThrottle(*targetA, 0.0f, "armed at 0%", selectorLabel.c_str());
    }
    if (targetB && hasEsc(*targetB)) {
      targetB->armed = true;
      targetB->dutyPercent = 0.0f;
      sendAndReportThrottle(*targetB, 0.0f, "armed at 0%", selectorLabel.c_str());
    }
    Serial.println("Status: ARMED. Send a number from 0 to 100 to set duty cycle.");
    return;
  }

  if (line == "disarm") {
    if (targetA && hasEsc(*targetA)) {
      targetA->armed = false;
      targetA->dutyPercent = 0.0f;
      sendAndReportThrottle(*targetA, 0.0f, "disarmed / motor stop", selectorLabel.c_str());
    }
    if (targetB && hasEsc(*targetB)) {
      targetB->armed = false;
      targetB->dutyPercent = 0.0f;
      sendAndReportThrottle(*targetB, 0.0f, "disarmed / motor stop", selectorLabel.c_str());
    }
    Serial.println("Status: DISARMED. Motor command forced to 0%.");
    return;
  }

  if (line == "beep") {
    // Beacon commands are intended for ESC beeping / locating.
    // Keep throttle at zero around the command.
    if (targetA && hasEsc(*targetA)) {
      sendAndReportThrottle(*targetA, 0.0f, "pre-beep safety stop", selectorLabel.c_str());
      sendAndReportCommand(*targetA, DSHOT_CMD_BEACON1, "beep / beacon1", selectorLabel.c_str());
      sendAndReportThrottle(*targetA, 0.0f, "post-beep safety stop", selectorLabel.c_str());
    }
    if (targetB && hasEsc(*targetB)) {
      sendAndReportThrottle(*targetB, 0.0f, "pre-beep safety stop", selectorLabel.c_str());
      sendAndReportCommand(*targetB, DSHOT_CMD_BEACON1, "beep / beacon1", selectorLabel.c_str());
      sendAndReportThrottle(*targetB, 0.0f, "post-beep safety stop", selectorLabel.c_str());
    }
    return;
  }

  if (line == "fwd" || line == "rev") {
    const dshotCommands_e directionCommand = (line == "fwd") ? DSHOT_CMD_DIR_FWD : DSHOT_CMD_DIR_REV;
    const char *directionLabel = (line == "fwd") ? "direction fwd" : "direction rev";

    if (targetA && hasEsc(*targetA)) {
      sendAndReportThrottle(*targetA, 0.0f, "pre-direction safety stop", selectorLabel.c_str());
      sendAndReportCommand(*targetA, directionCommand, directionLabel, selectorLabel.c_str());
      sendAndReportThrottle(*targetA, 0.0f, "post-direction safety stop", selectorLabel.c_str());
    }
    if (targetB && hasEsc(*targetB)) {
      sendAndReportThrottle(*targetB, 0.0f, "pre-direction safety stop", selectorLabel.c_str());
      sendAndReportCommand(*targetB, directionCommand, directionLabel, selectorLabel.c_str());
      sendAndReportThrottle(*targetB, 0.0f, "post-direction safety stop", selectorLabel.c_str());
    }
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

    bool anyArmed = false;
    if (targetA && hasEsc(*targetA)) {
      if (!targetA->armed) {
        targetA->dutyPercent = 0.0f;
        sendAndReportThrottle(*targetA, 0.0f, "ignored duty cycle while disarmed", selectorLabel.c_str());
      } else {
        targetA->dutyPercent = requestedDuty;
        sendAndReportThrottle(*targetA, targetA->dutyPercent, "duty cycle update", selectorLabel.c_str());
        anyArmed = true;
      }
    }
    if (targetB && hasEsc(*targetB)) {
      if (!targetB->armed) {
        targetB->dutyPercent = 0.0f;
        sendAndReportThrottle(*targetB, 0.0f, "ignored duty cycle while disarmed", selectorLabel.c_str());
      } else {
        targetB->dutyPercent = requestedDuty;
        sendAndReportThrottle(*targetB, targetB->dutyPercent, "duty cycle update", selectorLabel.c_str());
        anyArmed = true;
      }
    }

    if (!anyArmed) {
      Serial.println("Status: DISARMED. Duty command ignored; motor remains off.");
      return;
    }

    Serial.print("Status: ARMED, duty cycle set to ");
    Serial.print(requestedDuty, 2);
    Serial.println("%.");
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(line);
  Serial.println("Valid commands: arm, disarm, beep, help, or a number from 0 to 100.");
}

bool parseSelector(String &line, EscState *&targetA, EscState *&targetB, String &label) {
  const int enabled = enabledEscCount();
  if (enabled == 0) {
    Serial.println("No ESCs enabled (GPIO_NUM_NC). Commands are ignored.");
    return false;
  }

  int spaceIndex = line.indexOf(' ');
  String firstToken = (spaceIndex >= 0) ? line.substring(0, spaceIndex) : line;
  if (firstToken != "m1" && firstToken != "m2" && firstToken != "b") {
    Serial.println("Missing selector. Use M1, M2, or B before each command.");
    return false;
  }

  if (firstToken == "m1") {
    if (!hasEsc(esc1)) {
      Serial.println("ESC M1 is disabled (GPIO_NUM_NC). Use M2 instead.");
      return false;
    }
    targetA = &esc1;
    targetB = nullptr;
    label = "M1";
  } else if (firstToken == "m2") {
    if (!hasEsc(esc2)) {
      Serial.println("ESC M2 is disabled (GPIO_NUM_NC). Use M1 instead.");
      return false;
    }
    targetA = &esc2;
    targetB = nullptr;
    label = "M2";
  } else {
    if (enabled < 2) {
      Serial.println("Selector B requires both ESCs enabled.");
      return false;
    }
    targetA = &esc1;
    targetB = &esc2;
    label = "B";
  }

  if (spaceIndex >= 0) {
    line = line.substring(spaceIndex + 1);
    line.trim();
  } else {
    line = "";
  }

  return true;
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

void sendAndReportThrottle(EscState &escState, float percent, const char *reason, const char *label) {
  if (!hasEsc(escState)) {
    return;
  }
  dshot_result_t result = escState.esc->sendThrottlePercent(percent);

  Serial.print("Throttle command [");
  Serial.print(label);
  Serial.print(" / ");
  Serial.print(reason);
  Serial.print("]: ");
  Serial.print(percent, 2);
  Serial.print("% -> ");

  printResult(result);
}

void sendAndReportCommand(EscState &escState, dshotCommands_e command, const char *name, const char *label) {
  if (!hasEsc(escState)) {
    return;
  }
  dshot_result_t result = escState.esc->sendCommand(command);

  Serial.print("DShot command [");
  Serial.print(label);
  Serial.print(" / ");
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
  const int enabled = enabledEscCount();
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  m1 <cmd>  - target ESC 1 (M1)");
  Serial.println("  m2 <cmd>  - target ESC 2 (M2)");
  Serial.println("  b  <cmd>  - target both ESCs (requires both enabled)");
  Serial.println("  arm       - arm ESC, throttle remains 0%");
  Serial.println("  disarm    - force motor off and ignore duty commands");
  Serial.println("  beep      - send DShot beacon/beep command");
  Serial.println("  fwd       - set motor direction forward");
  Serial.println("  rev       - set motor direction reverse");
  Serial.println("  0-100     - set duty cycle percent, e.g. 12, 12.5, 100");
  Serial.println("  help      - show this menu");
  Serial.println();
  Serial.println("Examples:");
  if (enabled == 0) {
    Serial.println("  (no ESCs enabled)");
  } else if (enabled == 1) {
    Serial.println("  m1 arm");
    Serial.println("  m1 15");
    Serial.println("  m1 rev");
  } else {
    Serial.println("  m1 arm");
    Serial.println("  m2 15");
    Serial.println("  b beep");
    Serial.println("  b fwd");
  }
  Serial.println();
}

void initEsc(EscState &escState, gpio_num_t pin, const char *label) {
  if (pin == GPIO_NUM_NC) {
    Serial.print("ESC ");
    Serial.print(label);
    Serial.println(" disabled (GPIO_NUM_NC).");
    escState.esc = nullptr;
    escState.armed = false;
    escState.dutyPercent = 0.0f;
    return;
  }

  escState.esc = new DShotRMT(pin, DSHOT_MODE, BIDIRECTIONAL);
  dshot_result_t initResult = escState.esc->begin();
  Serial.print("ESC ");
  Serial.print(label);
  Serial.print(" init: ");
  printResult(initResult);
  escState.armed = false;
  escState.dutyPercent = 0.0f;
}

bool hasEsc(const EscState &escState) {
  return escState.esc != nullptr;
}

int enabledEscCount() {
  return (hasEsc(esc1) ? 1 : 0) + (hasEsc(esc2) ? 1 : 0);
}