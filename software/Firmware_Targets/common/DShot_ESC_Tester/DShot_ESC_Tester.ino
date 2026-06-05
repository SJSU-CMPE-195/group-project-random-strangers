#include <Arduino.h>
#include <DShotRMT.h>

// ---------- User settings ----------
static constexpr unsigned SERIAL_BAUD = 115200;

// Change these to the GPIOs connected to your ESC DShot signal wires.
// Use GPIO_NUM_NC to disable an ESC slot.
static constexpr gpio_num_t ESC_PINS[] = {GPIO_NUM_2};

// Common choices: DSHOT150, DSHOT300, DSHOT600, DSHOT1200.
// DSHOT300 is a conservative default.
static constexpr dshot_mode_t DSHOT_MODE = DSHOT600;

// Keep bidirectional telemetry off unless your wiring and ESC support it.
static constexpr bool BIDIRECTIONAL = false;

// How often to refresh the ESC command.
// DShot ESCs generally expect repeated frames.
static constexpr uint32_t DSHOT_REFRESH_MS = 20;

// -----------------------------------

static constexpr char MOTOR_SELECTOR_SYMBOL = 'm';
static constexpr const char *MOTOR_LABEL = "M";

struct Esc {
  gpio_num_t pin = GPIO_NUM_NC;
  DShotRMT comm;
  bool armed = false;
  bool currentDirection = false; //false -> normal, true -> reverse
  float dutyPercent = 0.0f;

  explicit Esc(gpio_num_t escPin)
    : pin(escPin),
      comm(escPin, DSHOT_MODE, BIDIRECTIONAL),
      armed(false),
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

void printHelp();
void printEscList();
void handleSerialLine(String& line);
bool parseSelector(String& line, int& selectedTarget);
bool parseNumberOnly(const String &text, float &value);
void sendAndReportThrottle(Esc &escState, float percent, const char *reason, unsigned motorIndex);
void sendAndReportCommand(Esc &escState, dshotCommands_e command, const char *name, unsigned motorIndex);
void printResult(const dshot_result_t &result);

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("DShotRMT Serial ESC Controller");
  Serial.println("Motors are OFF by default.");

  //initialize all the motors
  for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
    dshot_result_t initResult = escs[escIndex].comm.begin();

    //print the initialization result
    Serial.print("ESC ");
    Serial.print(MOTOR_LABEL);
    Serial.print(escIndex);
    Serial.print(" init: ");
    printResult(initResult);
  }

  // Explicitly command motor stop on boot.
  for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
    sendAndReportThrottle(escs[escIndex], 0.0f, "boot safety stop", escIndex);
  }

  printHelp();
}

void loop() {
  // Read serial commands.
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');

    if (line.length() > 0) {
      handleSerialLine(line);
    }
  }

  // Keep refreshing the ESC.
  const uint32_t now = millis();
  if (now - lastDShotSendMs >= DSHOT_REFRESH_MS) {
    lastDShotSendMs = now;

    for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
      escs[escIndex].comm.sendThrottlePercent(escs[escIndex].armed ? escs[escIndex].dutyPercent : 0.0f);
    }
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
      for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
        escs[escIndex].armed = true;
        escs[escIndex].dutyPercent = 0.0f;

        sendAndReportThrottle(escs[escIndex], 0.0f, "armed at 0%", escIndex);
      }
    } else {
      escs[selectedTarget].armed = true;
      escs[selectedTarget].dutyPercent = 0.0f;
      sendAndReportThrottle(escs[selectedTarget], 0.0f, "armed at 0%", selectedTarget);
    }

    Serial.println("Status: ARMED. Send a number from 0 to 100 to set duty cycle.");
    return;
  }

  if (line == "disarm") {
    if (selectedTarget == -1) {
      for (unsigned escIndex = 0; escIndex < ESC_COUNT; ++escIndex) {
        escs[escIndex].armed = false;
        escs[escIndex].dutyPercent = 0.0f;

        sendAndReportThrottle(escs[escIndex], 0.0f, "disarmed / motor stop", escIndex);
      }
    } else {
      escs[selectedTarget].armed = false;
      escs[selectedTarget].dutyPercent = 0.0f;
      sendAndReportThrottle(escs[selectedTarget], 0.0f, "disarmed / motor stop", selectedTarget);
    }

    Serial.println("Status: DISARMED. Motor command forced to 0%.");
    return;
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
          sendAndReportThrottle(escs[escIndex], 0.0f, "ignored duty cycle while disarmed", escIndex);
        } else {
          escs[escIndex].dutyPercent = requestedDuty;
          sendAndReportThrottle(escs[escIndex], escs[escIndex].dutyPercent, "duty cycle update", escIndex);
          anyArmed = true;
        }
      }
    } else {
      if (!escs[selectedTarget].armed) {
        escs[selectedTarget].dutyPercent = 0.0f;
        sendAndReportThrottle(escs[selectedTarget], 0.0f, "ignored duty cycle while disarmed", selectedTarget);
      } else {
        escs[selectedTarget].dutyPercent = requestedDuty;
        sendAndReportThrottle(escs[selectedTarget], escs[selectedTarget].dutyPercent, "duty cycle update", selectedTarget);
        anyArmed = true;
      }
    }

    if (!anyArmed) {
      Serial.println("Status: DISARMED. Duty command ignored; motor remains off.");
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

void sendAndReportThrottle(Esc &escState, float percent, const char *reason, unsigned motorIndex) {
  dshot_result_t result = escState.comm.sendThrottlePercent(percent);

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
  Serial.println("  arm       - arm ESC, throttle remains 0%");
  Serial.println("  disarm    - force motor off and ignore duty commands");
  Serial.println("  beep      - send DShot beacon/beep command");
  Serial.println("  fwd       - set motor direction forward");
  Serial.println("  rev       - set motor direction reverse");
  Serial.println("  0-100     - set duty cycle percent, e.g. 12, 12.5, 100");
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