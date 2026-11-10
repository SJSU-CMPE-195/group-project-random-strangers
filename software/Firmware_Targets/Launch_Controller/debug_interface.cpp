#include <Arduino.h>

#include "config.h"
#include "debug_interface.h"
#include "I2C_Driver.h"

namespace Launch_Controller {

void print_serial_command_help() {
    Serial.println("Commands:");
    Serial.println("  ARM 1|2|S");
    Serial.println("  DISARM 1|2|S");
    Serial.println("  TARGET 1|2 <RPM>");
    Serial.println("  FAULTS");
}

static void print_faults() {
    Serial.print("Master fault state: ");
    Serial.println(fault_manager.get_master_fault_state() ? "ACTIVE" : "CLEAR");

    const unsigned active_fault_count = fault_manager.get_active_fault_count();
    for (unsigned fault_index = 0; fault_index < active_fault_count; ++fault_index) {
        Serial.print("[");
        Serial.print(fault_manager.get_active_fault_code(fault_index));
        Serial.print("] ");
        Serial.println(fault_manager.get_active_fault_reason(fault_index));
    }
}

static bool parse_rpm(const String &value, float &rpm) {
    char *end = nullptr;
    const char *text = value.c_str();
    rpm = strtof(text, &end);

    return end != text && *end == '\0' && isfinite(rpm) && rpm >= 0.0f;
}

void handle_serial_command(String line) {
    line.trim();
    line.toUpperCase();

    if (line == "FAULTS") {
        print_faults();
        return;
    }

    const int first_space = line.indexOf(' ');
    if (first_space < 0) {
        Serial.println("Invalid command.");
        print_serial_command_help();
        return;
    }

    const String command = line.substring(0, first_space);
    String arguments = line.substring(first_space + 1);
    arguments.trim();

    const int argument_space = arguments.indexOf(' ');
    const String selector = argument_space < 0 ? arguments : arguments.substring(0, argument_space);
    String value = argument_space < 0 ? "" : arguments.substring(argument_space + 1);
    value.trim();

    if (selector != "1" && selector != "2" && selector != "S") {
        Serial.println("Invalid selector. Use 1, 2, or S.");
        return;
    }

    if (command == "ARM" || command == "DISARM") {
        if (value.length() != 0) {
            Serial.println("ARM and DISARM accept one selector only.");
            return;
        }

        const bool arm = command == "ARM";
        if (selector == "S") {
            if (arm) {
                event_manager.arm_solenoid = true;
            } else {
                event_manager.disarm_solenoid = true;
            }
        } else {
            if (arm) {
                event_manager.arm_escs = true;
            } else {
                event_manager.disarm_escs = true;
            }
        }

        Serial.print(command);
        Serial.print(" ");
        Serial.println(selector);
        return;
    }

    if (command == "TARGET") {
        if (selector == "S" || value.length() == 0) {
            Serial.println("TARGET requires selector 1 or 2 and an RPM value.");
            return;
        }

        float rpm = 0.0f;
        if (!parse_rpm(value, rpm)) {
            Serial.println("Invalid RPM. Use a non-negative number.");
            return;
        }

        I2C_Registers.RPM_target = rpm;
        Serial.print("TARGET ");
        Serial.print(selector);
        Serial.print(" ");
        Serial.println(rpm);
        return;
    }

    Serial.println("Unknown command.");
    print_serial_command_help();
}

} //namespace Launch_Controller