#pragma once

#include <Arduino.h>

namespace Launch_Controller {

void print_serial_command_help();
void handle_serial_command(String line);

} // namespace Launch_Controller
