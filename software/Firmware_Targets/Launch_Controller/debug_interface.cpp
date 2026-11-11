#include <Arduino.h>
#include <cmath>
#include <cstdlib>

#include "config.h"
#include "debug_interface.h"
#include "I2C_Driver.h"

namespace Launch_Controller {

namespace {

constexpr unsigned MAX_COMMAND_TOKENS = 4;

bool parse_non_negative_float(const String &text, float &result) {
    if (text.length() == 0) {
        return false;
    }

    const char *start = text.c_str();
    char *end = nullptr;

    result = strtof(start, &end);

    return end != start && *end == '\0' && std::isfinite(result) && result >= 0.0f;
}

unsigned tokenize_command(
    String line,
    String *tokens,
    unsigned maximum_tokens
) {
    unsigned token_count = 0;

    line.trim();

    while (line.length() > 0 && token_count < maximum_tokens) {
        line.trim();

        const int separator = line.indexOf(' ');

        if (separator < 0) {
            tokens[token_count++] = line;
            break;
        }

        tokens[token_count++] = line.substring(0, separator);
        line = line.substring(separator + 1);
    }

    return token_count;
}

void print_faults() {
    Serial.print("Master fault state: ");
    Serial.println(
        fault_manager.get_master_fault_state() ? "ACTIVE" : "CLEAR"
    );

    const unsigned active_fault_count =
        fault_manager.get_active_fault_count();

    if (active_fault_count == 0) {
        Serial.println("No active faults.");
        return;
    }

    for (unsigned index = 0; index < active_fault_count; ++index) {
        Serial.print("[");
        Serial.print(fault_manager.get_active_fault_code(index));
        Serial.print("] ");
        Serial.println(fault_manager.get_active_fault_reason(index));
    }
}

void print_invalid_command() {
    Serial.println("Invalid command.");
    print_serial_command_help();
}

} // namespace

void print_serial_command_help() {
    Serial.println("Commands:");
    Serial.println("  ARM M|S");
    Serial.println("  DISARM M|S");
    Serial.println(DSHOT_BIDIRECTIONAL ? "  TARGET M <RPM>" : "  TARGET M <%>");
    Serial.println("  FIRE S");
    Serial.println("  FAULTS");
}

void handle_serial_command(String line) {
    line.trim();
    line.toUpperCase();

    if (line.length() == 0) {
        return;
    }

    String tokens[MAX_COMMAND_TOKENS];
    const unsigned token_count =
        tokenize_command(line, tokens, MAX_COMMAND_TOKENS);

    const String &command = tokens[0];

    if (command == "FAULTS") {
        if (token_count != 1) {
            Serial.println("FAULTS does not accept arguments.");
            return;
        }

        print_faults();
        return;
    }

    if (command == "ARM" || command == "DISARM") {
        if (token_count != 2) {
            Serial.print(command);
            Serial.println(" requires exactly one selector: M or S.");
            return;
        }

        const String &selector = tokens[1];

        if (selector != "M" && selector != "S") {
            Serial.println("Invalid selector. Use M for ESCs or S for solenoid.");
            return;
        }

        const bool arm_command = command == "ARM";

        if (selector == "M") {
            if (arm_command) {
                event_manager.arm_escs = true;
            } else {
                event_manager.disarm_escs = true;
            }
        } else {
            if (arm_command) {
                event_manager.arm_solenoid = true;
            } else {
                event_manager.disarm_solenoid = true;
            }
        }

        Serial.print(command);
        Serial.print(" ");
        Serial.println(selector);
        return;
    }

    if (command == "TARGET") {
        if (token_count != 3) {
            Serial.println("TARGET requires: TARGET M <RPM>.");
            return;
        }

        // Selector is accepted for command compatibility but is not differentiated.
        if (tokens[1] != "M") {
            Serial.println("Invalid target selector. Use 1 or 2.");
            return;
        }

        float rpm = 0.0f;

        if (!parse_non_negative_float(tokens[2], rpm)) {
            Serial.println("Invalid RPM. Use a non-negative number.");
            return;
        }

        I2C_Registers.RPM_target = rpm;

        Serial.print("TARGET ");
        Serial.print(tokens[1]);
        Serial.print(" ");
        Serial.println(rpm, 2);
        return;
    }

    if (command == "FIRE") {
        if (token_count != 2 || tokens[1] != "S") {
            Serial.println("FIRE requires the selector S.");
            return;
        }

        event_manager.fire_solenoid = true;
        Serial.println("FIRE S");
        return;
    }

    Serial.print("Unknown command: ");
    Serial.println(command);
    print_invalid_command();
}

} // namespace Launch_Controller