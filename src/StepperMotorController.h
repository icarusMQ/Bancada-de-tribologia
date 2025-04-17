#ifndef STEPPERMOTORCONTROLLER_H
#define STEPPERMOTORCONTROLLER_H

#include "Arduino.h"

// Improved stepper motor controller with fixed direction control
class StepperMotorController {
  public:
    // Constructor
    StepperMotorController();
    
    // Initialize pins and state
    void init();
    
    // Set target speed for a motor
    // motorID: 1-5 for the physical motor
    // rpm: the target rotation speed
    // direction: HIGH for forward, LOW for reverse
    void setMotorSpeed(uint8_t motorID, uint8_t rpm, uint8_t direction);
    
    // Stop a specific motor
    void stopMotor(uint8_t motorID);
    
    // Stop all motors
    void stopAllMotors();
    
    // Must be called in the main loop for non-blocking operation
    void update();
    
    // Check if a particular motor is running
    bool isMotorRunning(uint8_t motorID);
    
  private:
    // Status bits for each motor (packed in one byte)
    uint8_t _motorStatus;
    
    // Current step states
    uint8_t _stepStates[5];
    
    // Speed and timing data
    uint8_t _motorSpeeds[5];
    uint8_t _motorDirections[5];
    unsigned long _lastStepTime[5];
    uint16_t _stepIntervals[5];
    
    // Pin definitions
    static const uint8_t STEP_PINS[5];
    static const uint8_t DIRECTION_PINS[5];
    
    // Direction inversion flags - to handle hardware wiring differences
    static const bool INVERT_DIRECTION[5];
    
    // Microstepping configuration
    static const uint16_t STEPS_PER_REV_MOTOR3;
    static const uint16_t STEPS_PER_REV_OTHER_MOTORS;
    
    // Calculate step interval from RPM
    uint16_t _calculateStepInterval(uint8_t motorID, uint8_t rpm);
    
    // Get steps per revolution for a motor
    uint16_t _getStepsPerRev(uint8_t motorID);
};

#endif