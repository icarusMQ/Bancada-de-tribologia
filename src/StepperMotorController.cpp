#include "StepperMotorController.h"

// Define motor pin arrays
const uint8_t StepperMotorController::STEP_PINS[5] = {A5, A3, A1, 5, 9};
const uint8_t StepperMotorController::DIRECTION_PINS[5] = {A4, A2, A0, 8, 8};

// Direction inversion flags - set to true if a motor's direction is reversed
// This addresses hardware-specific wiring issues without changing the code
const bool StepperMotorController::INVERT_DIRECTION[5] = {false, false, false, false, false};

// Microstepping configuration
const uint16_t StepperMotorController::STEPS_PER_REV_MOTOR3 = 3200;      // Motor 3 (16 microsteps)
const uint16_t StepperMotorController::STEPS_PER_REV_OTHER_MOTORS = 1600; // Motors 1, 2, 4, 5 (8 microsteps)

// Bit masks for motor status
#define MOTOR_RUNNING_MASK(idx) (1 << (idx))

// Constructor
StepperMotorController::StepperMotorController() : _motorStatus(0) {
  // Initialize arrays with zeros
  for (uint8_t i = 0; i < 5; i++) {
    _stepStates[i] = LOW;
    _motorSpeeds[i] = 0;
    _motorDirections[i] = HIGH;
    _lastStepTime[i] = 0;
    _stepIntervals[i] = 0;
  }
}

// Initialize pins
void StepperMotorController::init() {
  // Set all motor pins to outputs
  for (uint8_t i = 0; i < 5; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIRECTION_PINS[i], OUTPUT);
    
    digitalWrite(STEP_PINS[i], LOW);
    digitalWrite(DIRECTION_PINS[i], HIGH);
  }
}

// Get steps per revolution for a motor
uint16_t StepperMotorController::_getStepsPerRev(uint8_t motorID) {
  if (motorID == 3) {
    return STEPS_PER_REV_MOTOR3;
  } else {
    return STEPS_PER_REV_OTHER_MOTORS;
  }
}

// Calculate step interval from RPM
uint16_t StepperMotorController::_calculateStepInterval(uint8_t motorID, uint8_t rpm) {
  if (rpm == 0) return 0; // Avoid division by zero
  
  // Get steps per revolution for this motor
  uint16_t stepsPerRev = _getStepsPerRev(motorID);
  
  // Calculate microseconds per step
  // 60,000,000 us / (steps_per_rev * rpm)
  return 60000000UL / (stepsPerRev * (uint32_t)rpm);
}

// Set a motor's speed
void StepperMotorController::setMotorSpeed(uint8_t motorID, uint8_t rpm, uint8_t direction) {
  if (motorID < 1 || motorID > 5) return;
  
  uint8_t index = motorID - 1;
  
  // Store actual direction in state
  _motorDirections[index] = direction;
  
  // Apply direction inversion if needed
  uint8_t actualDirection = direction;
  if (INVERT_DIRECTION[index]) {
    actualDirection = !direction;
  }
  
  // Set direction pin
  digitalWrite(DIRECTION_PINS[index], actualDirection);
  
  // CRITICAL FIX FOR MOTORS 1 & 2: 
  // Since they share a direction pin, we need special handling
  if (motorID == 1 || motorID == 2) {
    // If the other motor is running, make sure it uses the right direction
    uint8_t otherIndex = (motorID == 1) ? 1 : 0;
    uint8_t otherMotorID = (motorID == 1) ? 2 : 1;
    
    if ((_motorStatus & MOTOR_RUNNING_MASK(otherIndex)) && 
        (_motorDirections[otherIndex] != direction)) {
      // Direction conflict - motors need different directions but share a pin
      // We stop the other motor when trying to run motors in different directions
      Serial.print(F("WARNING: Direction conflict between motors 1 & 2. Stopping motor "));
      Serial.println(otherMotorID);
      stopMotor(otherMotorID);
    }
  }
  
  if (rpm > 0) {
    // Calculate step interval
    _stepIntervals[index] = _calculateStepInterval(motorID, rpm);
    _motorSpeeds[index] = rpm;
    
    // Mark motor as running
    _motorStatus |= MOTOR_RUNNING_MASK(index);
    
    Serial.print(F("Motor "));
    Serial.print(motorID);
    Serial.print(F(" running at "));
    Serial.print(rpm);
    Serial.print(F(" RPM, direction: "));
    Serial.println(direction == HIGH ? F("Forward") : F("Reverse"));
  } else {
    // Stop the motor if RPM is zero
    stopMotor(motorID);
  }
}

// Stop a specific motor
void StepperMotorController::stopMotor(uint8_t motorID) {
  if (motorID < 1 || motorID > 5) return;
  
  uint8_t index = motorID - 1;
  
  // Don't do anything if motor is already stopped
  if (!(_motorStatus & MOTOR_RUNNING_MASK(index))) {
    return;
  }
  
  // Clear the running bit
  _motorStatus &= ~MOTOR_RUNNING_MASK(index);
  
  // Set step pin low
  digitalWrite(STEP_PINS[index], LOW);
  
  _stepStates[index] = LOW;
  _motorSpeeds[index] = 0;
  _stepIntervals[index] = 0;
  
  Serial.print(F("Motor "));
  Serial.print(motorID);
  Serial.println(F(" stopped."));
}

// Stop all motors
void StepperMotorController::stopAllMotors() {
  for (uint8_t i = 1; i <= 5; i++) {
    if (isMotorRunning(i)) {
      stopMotor(i);
    }
  }
  
  // Ensure status is clear
  _motorStatus = 0;
}

// Check if a motor is running
bool StepperMotorController::isMotorRunning(uint8_t motorID) {
  if (motorID < 1 || motorID > 5) return false;
  
  uint8_t index = motorID - 1;
  return (_motorStatus & MOTOR_RUNNING_MASK(index)) != 0;
}

// Update all motors - must be called in the main loop
void StepperMotorController::update() {
  unsigned long currentMicros = micros();
  
  // Update each motor
  for (uint8_t i = 0; i < 5; i++) {
    // Skip if motor is not running
    if (!(_motorStatus & MOTOR_RUNNING_MASK(i))) continue;
    
    // Check if it's time for a step
    if (_stepIntervals[i] > 0 && 
        (currentMicros - _lastStepTime[i] >= _stepIntervals[i])) {
      
      // Toggle step pin
      _stepStates[i] = !_stepStates[i];
      digitalWrite(STEP_PINS[i], _stepStates[i]);
      
      // Update last step time
      _lastStepTime[i] = currentMicros;
    }
  }
}