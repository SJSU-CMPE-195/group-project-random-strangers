#include <Arduino.h>
#include "../../Launch_Controller/ESP32C3DSHOT/src/DShotRMT.h"

// ---------- User settings ----------
static constexpr unsigned SERIAL_BAUD = 115200;

// Change these to the GPIOs connected to your ESC DShot signal wires.
// Use GPIO_NUM_NC to disable an ESC slot.
static constexpr gpio_num_t ESC_PINS[] = {GPIO_NUM_9};

// Common choices: DSHOT150, DSHOT300, DSHOT600, DSHOT1200.
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// Keep bidirectional telemetry off unless your wiring and ESC support it.
static constexpr bool BIDIRECTIONAL = false;

// How often to refresh the ESC command.
// DShot ESCs generally expect repeated frames.
static constexpr uint32_t DSHOT_REFRESH_MS = 1; // Approximately 1,000 DShot frames/s per ESC.

// Bluejay needs ~300 ms of true motor-stop frames before it can arm.
// Use a 1-second margin and require a substantial number of successful sends.
static constexpr uint32_t ARM_ZERO_MS = 1000;
static constexpr uint16_t MIN_ARM_STOP_FRAMES = 300;

// -----------------------------------

static constexpr char MOTOR_SELECTOR_SYMBOL = 'm';
static constexpr const char *MOTOR_LABEL = "M";

struct Esc {
  gpio_num_t pin = GPIO_NUM_NC;
  DShotRMT comm;
  bool initialized = false;
  bool armed = false;       // User has requested arm AND zero-throttle preamble finished.
  bool arming = false;      // Still sending the mandatory zero-throttle preamble.
  uint32_t armStartMs = 0;
  uint16_t armStopFrames = 0;
  bool currentDirection = false; //false -> normal, true -> reverse
  float dutyPercent = 0.0f;

  explicit Esc(gpio_num_t escPin)
    : pin(escPin),
      comm(escPin, DSHOT_MODE, BIDIRECTIONAL),
      initialized(false),
      armed(false),
      arming(false),
      armStartMs(0),
      armStopFrames(0),
      currentDirection(false),
      dutyPercent(0.0f) {}
};

static constexpr unsigned ESC_PINS_COUNT = sizeof(ESC_PINS) / sizeof(ESC_PINS[0]);

namespace { //recursive template BS

template <unsigned... I>
struct IndexList {};

template <typename List, unsigned NewIndex>
struct AppendIndex;

template <unsigned... I, unsigned NewIndex>
struct AppendIndex<IndexList<I...>, NewIndex> {
  typedef IndexList<I..., NewIndex> type;
};

template <bool Condition, typename TrueType, typename FalseType>
struct ChooseType {
  typedef TrueType type;
};

template <typename TrueType, typename FalseType>
struct ChooseType<false, TrueType, FalseType> {
  typedef FalseType type;
};

template <unsigned N, typename List>
struct FilterEnabledIndices {
  typedef typename FilterEnabledIndices<N - 1, List>::type previousIndices;
  typedef typename ChooseType<
      (ESC_PINS[N - 1] != GPIO_NUM_NC),
      typename AppendIndex<previousIndices, N - 1>::type,
      previousIndices>::type type;
};

template <typename List>
struct FilterEnabledIndices<0, List> {
  typedef List type;
};

template <typename List>
struct ListSize;

template <unsigned... I>
struct ListSize<IndexList<I...> > {
  static constexpr unsigned value = sizeof...(I);
};

// Non-empty overload: expands the enabled pin indices directly into a real
// static C array and returns it by reference. This avoids the EscArray
// wrapper/dummy struct while keeping ESC_PINS as the single constexpr pin list.
template <unsigned I0, unsigned... I>
Esc (&makeEscArray(IndexList<I0, I...>))[1 + sizeof...(I)] {
  static Esc escs[1 + sizeof...(I)] = { Esc(ESC_PINS[I0]), Esc(ESC_PINS[I])... };
  return escs;
}

} // recursive template BS

typedef typename FilterEnabledIndices<ESC_PINS_COUNT, IndexList<> >::type EnabledIndices;
static constexpr unsigned ESC_COUNT = ListSize<EnabledIndices>::value;
static_assert(ESC_COUNT > 0, "No ESCs are enabled. Set ESC_PINS entries to valid GPIOs.");
static Esc (&escs)[ESC_COUNT] = makeEscArray(EnabledIndices());

uint32_t lastDShotSendMs = 0;
// Nonblocking line input: an incomplete serial command must not interrupt DShot output.
char serialInput[96] = {};
size_t serialInputLen = 0;

void printHelp();
void printEscList();
void handleSerialLine(String& line);
bool parseSelector(String& line, int& selectedTarget);
bool parseNumberOnly(const String &text, float &value);
void sendAndReportThrottle(Esc &escState, float percent, const char *reason, unsigned motorIndex);
void sendAndReportCommand(Esc &escState, dshotCommands_e command, const char *name, unsigned motorIndex);
void printResult(const dshot_result_t &result);
dshot_result_t sendSafeThrottle(Esc &escState, float percent);
void pollSerial();
void refreshEscs();
void beginArming(Esc &escState, unsigned motorIndex);

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("DShotRMT Serial ESC Controller");
  Serial.println("Motors are OFF by default. DShot300, unidirectional.");
  Serial.println("Zero-throttle (raw DShot 0) is continuously transmitted.");

  //initialize all the motors
  for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
    dshot_result_t initResult = escs[escIndex].comm.begin();
    escs[escIndex].initialized = initResult.success;

    //print the initialization result
    Serial.print("ESC ");
    Serial.print(MOTOR_LABEL);
    Serial.print(escIndex);
    Serial.print(" init: ");
    printResult(initResult);
  }

  // Explicitly command motor stop on boot.
  for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
    if (escs[escIndex].initialized) {
      sendAndReportThrottle(escs[escIndex], 0.0f, "boot motor stop (raw 0)", escIndex);
    }
  }

  printHelp();
}

void loop() {
  // Give throttle frames priority over command parsing; neither routine blocks.
  refreshEscs();
  pollSerial();
}

void pollSerial() {
  // Bound the amount of parsing per loop so the stream remains responsive.
  unsigned budget = 64;
  while (budget-- && Serial.available() > 0) {
    int incoming = Serial.read();
    if (incoming < 0) break;
    char c = static_cast<char>(incoming);
    if (c == '\r' || c == '\n') {
      if (serialInputLen != 0) {
        serialInput[serialInputLen] = '\0';
        String line(serialInput);
        serialInputLen = 0;
        handleSerialLine(line);
      }
    } else if (serialInputLen < sizeof(serialInput) - 1) {
      serialInput[serialInputLen++] = c;
    } else {
      serialInputLen = 0;
      Serial.println("Serial input too long; command discarded.");
    }
  }
}

void refreshEscs() {
  const uint32_t now = millis();
  if (now - lastDShotSendMs < DSHOT_REFRESH_MS) return;
  lastDShotSendMs = now;

  for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
    Esc &esc = escs[escIndex];
    if (!esc.initialized) continue;

    // STOP (raw DShot 0) is sent continuously during boot, disarmed,
    // and throughout the entire Bluejay arming preamble.
    float output = esc.armed ? esc.dutyPercent : 0.0f;
    dshot_result_t result = sendSafeThrottle(esc, output);
    if (!result.success) {
      if (esc.arming || esc.armed) {
        Serial.print("ESC M"); Serial.print(escIndex);
        Serial.print(" transmission failed, returning to DISARMED: ");
        printResult(result);
      }
      esc.arming = false;
      esc.armed = false;
      esc.dutyPercent = 0.0f;
      continue;
    }

    if (esc.arming) {
      if (esc.armStopFrames < UINT16_MAX) ++esc.armStopFrames;
      if ((uint32_t)(now - esc.armStartMs) >= ARM_ZERO_MS &&
          esc.armStopFrames >= MIN_ARM_STOP_FRAMES) {
        esc.arming = false;
        esc.armed = true;
        // Still output 0 until the user explicitly requests >0 throttle.
        esc.dutyPercent = 0.0f;
        Serial.print("ESC M"); Serial.print(escIndex);
        Serial.print(" arming preamble COMPLETE (");
        Serial.print(esc.armStopFrames);
        Serial.println(" stop-frame send calls). Ready for throttle.");
      }
    }
  }
}

void beginArming(Esc &esc, unsigned motorIndex) {
  if (!esc.initialized) {
    Serial.print("ESC M"); Serial.print(motorIndex);
    Serial.println(" cannot arm: DShotRMT initialization failed.");
    return;
  }
  esc.armed = false;
  esc.arming = true;
  esc.dutyPercent = 0.0f;
  esc.armStartMs = millis();
  esc.armStopFrames = 0;
  // Send a stop immediately, then continue at the periodic refresh rate.
  dshot_result_t firstStop = sendSafeThrottle(esc, 0.0f);
  Serial.print("ESC M"); Serial.print(motorIndex);
  Serial.print(" ARMING: sending DShot motor-stop 0 for at least ");
  Serial.print(ARM_ZERO_MS);
  Serial.println(" ms before enabling throttle.");
  if (!firstStop.success) {
    esc.arming = false;
    Serial.print("First stop frame failed: ");
    printResult(firstStop);
  }
}

void handleSerialLine(String& line) {
  line.trim();
  line.toLowerCase();

  if (line == "list" || line == "ls") {
    printEscList();
    return;
  }

  if (line == "help" || line == "h" || line == "?") {
    printHelp();
    return;
  }

  int selectedTarget = -1;

  if (!parseSelector(line, selectedTarget)) {
    Serial.print("Missing motor selector (\"");
    Serial.print(MOTOR_LABEL);
    Serial.print("0\", \"");
    Serial.print(MOTOR_LABEL);
    Serial.println("1\", \"A\", etc.)");
    return;
  } 

  if (selectedTarget > -1 && static_cast<unsigned>(selectedTarget) >= ESC_COUNT) {
    Serial.print("Selected Motor does not exist: [");
    Serial.print(MOTOR_LABEL);
    Serial.print(selectedTarget);
    Serial.print("] is greater than the maximum: [");
    Serial.print(MOTOR_LABEL);
    Serial.print(ESC_COUNT - 1);
    Serial.println("]");
    return;
  }

  if (selectedTarget < -1) {
    Serial.print("Selected Motor does not exist: [");
    Serial.print(MOTOR_LABEL);
    Serial.print(selectedTarget);
    Serial.println("] is less than 0");
    return;
  }

  Serial.print("Received command [");
  if (selectedTarget == -1) {
    Serial.print("ALL");
  } else {
    Serial.print(MOTOR_LABEL);
    Serial.print(selectedTarget);
  }
  Serial.print("]: ");
  Serial.println(line);

  if (line.length() == 0) {
    Serial.println("Missing command after selector.");
    return;
  }
  

  if (line == "arm") {
    if (selectedTarget == -1) {
      for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex)
        beginArming(escs[escIndex], escIndex);
    } else {
      beginArming(escs[selectedTarget], selectedTarget);
    }
    Serial.println("Status: ARMING. Motor-stop frames only; wait for COMPLETE before throttle.");
    return;
  }

  if (line == "disarm") {
    if (selectedTarget == -1) {
      for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
        escs[escIndex].armed = false;
        escs[escIndex].arming = false;
        escs[escIndex].armStopFrames = 0;
        escs[escIndex].dutyPercent = 0.0f;

        sendAndReportThrottle(escs[escIndex], 0.0f, "disarmed / motor stop", escIndex);
      }
    } else {
      escs[selectedTarget].armed = false;
      escs[selectedTarget].arming = false;
      escs[selectedTarget].armStopFrames = 0;
      escs[selectedTarget].dutyPercent = 0.0f;
      sendAndReportThrottle(escs[selectedTarget], 0.0f, "disarmed / motor stop", selectedTarget);
    }

    Serial.println("Status: DISARMED. Motor command forced to 0%.");
    return;
  }

  if (line == "beep" || line == "fwd" || line == "rev") {
    bool safe = true;
    for (unsigned i = 0; i < ESC_COUNT; ++i) {
      if (selectedTarget != -1 && static_cast<unsigned>(selectedTarget) != i) continue;
      if (escs[i].armed || escs[i].arming) safe = false;
    }
    if (!safe) {
      Serial.println("Disarm the selected motor(s) before sending special commands.");
      return;
    }
  }

  if (line == "beep") {
    if (selectedTarget == -1) {
      for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
        sendAndReportThrottle(escs[escIndex], 0.0f, "pre-beep safety stop", escIndex);
        sendAndReportCommand(escs[escIndex], DSHOT_CMD_BEACON1, "beep / beacon1", escIndex);
        sendAndReportThrottle(escs[escIndex], 0.0f, "post-beep safety stop", escIndex);
      }
    } else {
      sendAndReportThrottle(escs[selectedTarget], 0.0f, "pre-beep safety stop", selectedTarget);
      sendAndReportCommand(escs[selectedTarget], DSHOT_CMD_BEACON1, "beep / beacon1", selectedTarget);
      sendAndReportThrottle(escs[selectedTarget], 0.0f, "post-beep safety stop", selectedTarget);
    }

    return;
  }

  if (line == "fwd" || line == "rev") {
    const dshotCommands_e directionCommand = line == "fwd" ? DSHOT_CMD_SPIN_DIRECTION_NORMAL : DSHOT_CMD_SPIN_DIRECTION_REVERSED;
    const bool newDirection = line == "rev";

    if (selectedTarget == -1) {
      for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
        if (escs[escIndex].currentDirection == newDirection) {
          Serial.print("[");
        Serial.print(MOTOR_LABEL);
          Serial.print(escIndex);
          Serial.println("]: Direction unchanged, skipping...");
        } else {
          sendAndReportThrottle(escs[escIndex], 0.0f, "pre-direction safety stop", escIndex);
          sendAndReportCommand(escs[escIndex], directionCommand, "Setting motor direction", escIndex);
          sendAndReportThrottle(escs[escIndex], 0.0f, "post-direction safety stop", escIndex);
          escs[escIndex].currentDirection = newDirection;
        }
      }
    } else {
      if (escs[selectedTarget].currentDirection == newDirection) {
        Serial.print("[");
        Serial.print(MOTOR_LABEL);
        Serial.print(selectedTarget);
        Serial.println("]: Direction unchanged, skipping...");
      } else {
        sendAndReportThrottle(escs[selectedTarget], 0.0f, "pre-direction safety stop", selectedTarget);
        sendAndReportCommand(escs[selectedTarget], directionCommand, "Setting motor direction", selectedTarget);
        sendAndReportThrottle(escs[selectedTarget], 0.0f, "post-direction safety stop", selectedTarget);
        escs[selectedTarget].currentDirection = newDirection;
      }
    }

    return;
  }

  float requestedDuty;

  if (parseNumberOnly(line, requestedDuty)) {
    if (requestedDuty < 0.0f || requestedDuty > 100.0f) {
      Serial.print("Rejected duty cycle: ");
      Serial.print(requestedDuty, 2);
      Serial.println("% is outside valid range 0-100.");
      return;
    }

    bool anyArmed = false;

    if (selectedTarget == -1) {
      for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
        if (!escs[escIndex].armed) {
          escs[escIndex].dutyPercent = 0.0f;
          Serial.print("ESC M"); Serial.print(escIndex);
          Serial.println(" not ready (DISARMED or ARMING): throttle ignored.");
        } else {
          escs[escIndex].dutyPercent = requestedDuty;
          sendAndReportThrottle(escs[escIndex], escs[escIndex].dutyPercent, "duty cycle update", escIndex);
          anyArmed = true;
        }
      }
    } else {
      if (!escs[selectedTarget].armed) {
        escs[selectedTarget].dutyPercent = 0.0f;
        Serial.print("ESC M"); Serial.print(selectedTarget);
        Serial.println(" not ready (DISARMED or ARMING): throttle ignored.");
      } else {
        escs[selectedTarget].dutyPercent = requestedDuty;
        sendAndReportThrottle(escs[selectedTarget], escs[selectedTarget].dutyPercent, "duty cycle update", selectedTarget);
        anyArmed = true;
      }
    }

    if (!anyArmed) {
      Serial.println("Status: not armed. Duty command ignored; motor remains off.");
    } else {
      Serial.print("Status: ARMED, duty cycle set to ");
      Serial.print(requestedDuty, 2);
      Serial.println("%.");
    }
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(line);
  printHelp();
}

bool parseSelector(String& line, int& selectedTarget) {
  int spaceIndex = line.indexOf(' ');
  String firstToken = (spaceIndex >= 0) ? line.substring(0, spaceIndex) : line;

  if (firstToken == "a") {
    selectedTarget = -1; // special value for "all"
  } else if (firstToken.length() >= 2 && firstToken[0] == MOTOR_SELECTOR_SYMBOL) {
    String indexText = firstToken.substring(1);

    for (unsigned int charIndex = 0; charIndex < indexText.length(); charIndex++) {
      if (!isDigit(indexText[charIndex])) {
        Serial.print("Invalid motor selector: ");
        Serial.print(firstToken);
        Serial.println(". Motor number must contain digits only.");
        return false;
      }
    }

    selectedTarget = indexText.toInt();
  } else {
    return false;
  }

  if (spaceIndex >= 0) {
    line = line.substring(spaceIndex + 1);
    line.trim();
  } else {
    line = "";
  }

  return true;
}

bool parseNumberOnly(const String &text, float &value) {
  char *endPtr = nullptr;
  value = strtof(text.c_str(), &endPtr);

  // No conversion happened.
  if (endPtr == text.c_str()) {
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

// DShot throttle 0 is a distinct motor-stop instruction; percent(0) is NOT stop.
// Use a single raw stop-command frame per refresh. Calling sendThrottle(0)
// in DShotRMT 0.9.5 actually repeats that command 10 times internally.
dshot_result_t sendSafeThrottle(Esc &escState, float percent) {
  if (percent <= 0.0f)
    return escState.comm.sendCommand(DSHOT_CMD_MOTOR_STOP, 1, 0);
  return escState.comm.sendThrottlePercent(percent);
}

void sendAndReportThrottle(Esc &escState, float percent, const char *reason, unsigned motorIndex) {
  if (!escState.initialized) return;
  dshot_result_t result = sendSafeThrottle(escState, percent);

  Serial.print("Throttle command [");
  Serial.print(MOTOR_LABEL);
  Serial.print(motorIndex);
  Serial.print(" / ");
  Serial.print(reason);
  Serial.print("]: ");
  Serial.print(percent, 2);
  Serial.print("% -> ");

  printResult(result);
}

void sendAndReportCommand(Esc &escState, dshotCommands_e command, const char *name, unsigned motorIndex) {
  if (!escState.initialized) return;
  dshot_result_t result = escState.comm.sendCommand(command);

  Serial.print("DShot command [");
  Serial.print(MOTOR_LABEL);
  Serial.print(motorIndex);
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
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  Mn <cmd>  - target ESC N (M0..Mn)");
  Serial.println("  a  <cmd>  - target all enabled ESCs");
  Serial.println("  arm       - send raw motor-stop for 1s; then mark ready");
  Serial.println("  disarm    - continually send motor-stop and reject throttle");
  Serial.println("  beep      - send DShot beacon/beep command");
  Serial.println("  fwd       - set motor direction forward");
  Serial.println("  rev       - set motor direction reverse");
  Serial.println("  0-100     - set throttle percent only AFTER ARMING completes");
  Serial.println("  help      - show this menu");
  Serial.println("  list      - list enabled ESC count");
  Serial.println();
}

void printEscList() {
  Serial.print("Enabled ESCs: ");
  Serial.print(ESC_COUNT);

  for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
    Serial.print("  ");
    Serial.print(MOTOR_LABEL);
    Serial.print(escIndex);
    Serial.print(": ");
    Serial.print("GPIO ");
    Serial.println(static_cast<int>(escs[escIndex].pin));
  }
}