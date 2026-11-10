// Arduino IDE compiles source files in the sketch directory, not nested library
// source directories. Include the embedded library implementation here so its
// symbols are linked into the firmware.
#include "ESP32C3DSHOT/src/DShotRMT.cpp"
#include "ESP32C3DSHOT/src/dshot_init.cpp"
