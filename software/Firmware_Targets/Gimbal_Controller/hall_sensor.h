//Declares the function needed by the main program
//Sensor initialization and obtaining measured angle

#pragma once

// Initialize analog Hall sensor inputs
bool initializeHallSensor();

// Read the latest Hall-derived shaft angle
// Returns a wrapped angle in radians [0, 2*pi)
// Returns NAN for detected invalid signals
float readHallAngle();
