//Exposes the SimpleFOC sensor monitoring functions
//to gimbal.ino

#pragma once

// Configure the SimpleFOC sensor interface.
// Does not initialize or energize motor hardware.
bool initializeMotorControl();

// Refresh SimpleFOC's sensor position tracking
void updateMotorControl();

// Get latest accumulated sensor angle in radians
float getMotorAngle();

// Check whether sensor monitoring initialized
bool isMotorReady();
