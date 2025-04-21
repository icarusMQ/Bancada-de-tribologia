#ifndef STEPPERMOTORCONTROLLER_H
#define STEPPERMOTORCONTROLLER_H

#include "Arduino.h"

// This class manages multiple stepper motors.
// It provides functionality to run and stop motors with specified RPMs and directions.
class StepperMotorController {
  public:
    // Constructor: Initializes the class variables.
    StepperMotorController();

    // Runs the specified motor at the given RPM and direction.
    // Parameters:
    // - motorID: The ID of the motor to run (1, 2, 3, 4, or 5).
    // - rpm: The speed in revolutions per minute (RPM).
    // - direction: The direction to run the motor (HIGH for forward, LOW for reverse).
    void RunMotor(uint8_t motorID, int rpm, uint8_t direction);

    // Stops the specified motor.
    // Parameters:
    // - motorID: The ID of the motor to stop (1, 2, 3, 4, or 5).
    void StopMotor(uint8_t motorID);

  private:
    // Pin definitions for the step and direction pins of each motor.
    // These pins must be connected to the corresponding motor driver inputs.
    static const uint8_t MOTOR1_STEP_PIN = 10;
    static const uint8_t MOTOR1_DIR_PIN = 9;
    static const uint8_t MOTOR2_STEP_PIN = 5;
    static const uint8_t MOTOR2_DIR_PIN = 9;
    static const uint8_t MOTOR3_STEP_PIN = A1;
    static const uint8_t MOTOR3_DIR_PIN = A0;
    static const uint8_t MOTOR4_STEP_PIN = A2;
    static const uint8_t MOTOR4_DIR_PIN = A4;
    static const uint8_t MOTOR5_STEP_PIN = A3;
    static const uint8_t MOTOR5_DIR_PIN = A5;

    // Helper function to set the RPM and direction for a motor.
    // Parameters:
    // - stepPin: The step pin of the motor.
    // - dirPin: The direction pin of the motor.
    // - rpm: The speed in revolutions per minute (RPM).
    // - direction: The direction to run the motor (HIGH for forward, LOW for reverse).
    void setMotorSpeed(uint8_t stepPin, uint8_t dirPin, int rpm, uint8_t direction);

    // Helper function to stop a motor.
    // Parameters:
    // - stepPin: The step pin of the motor.
    void stopMotor(uint8_t stepPin);
};

#endif