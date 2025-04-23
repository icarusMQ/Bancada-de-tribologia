#ifndef STEPPERMOTORCONTROLLER_H
#define STEPPERMOTORCONTROLLER_H

#include "Arduino.h"
#include <AccelStepper.h> // Include the AccelStepper library

// This class manages multiple stepper motors using the AccelStepper library.
// It provides functionality to run and stop motors continuously.
class StepperMotorController {
  public:
    // Constructor: Initializes the AccelStepper objects.
    StepperMotorController();

    // Sets the target speed and direction for the specified motor.
    // The motor will run continuously once UpdateMotors() is called repeatedly.
    // Parameters:
    // - motorID: The ID of the motor to run (1, 2, 3, 4, or 5).
    // - rpm: The target speed in revolutions per minute (RPM). Positive for forward, negative for reverse.
    void RunMotor(uint8_t motorID, int rpm); // Direction is now implicit in the sign of rpm

    // Stops the specified motor smoothly.
    // Parameters:
    // - motorID: The ID of the motor to stop (1, 2, 3, 4, or 5).
    void StopMotor(uint8_t motorID);

    // Updates the state of all motors.
    // This function MUST be called frequently in the main loop for motors to move.
    void UpdateMotors();

  private:
    // Pin definitions
    // Note: MOTOR1_DIR_PIN and MOTOR2_DIR_PIN are both set to 9 in the original code. Verify hardware connections.
    static const uint8_t MOTOR1_STEP_PIN = 9;
    static const uint8_t MOTOR1_DIR_PIN = 8;
    static const uint8_t MOTOR2_STEP_PIN = 5;
    static const uint8_t MOTOR2_DIR_PIN = 8; // Shared with Motor 1?
    static const uint8_t MOTOR3_STEP_PIN = A1;
    static const uint8_t MOTOR3_DIR_PIN = A0;
    static const uint8_t MOTOR4_STEP_PIN = A3;
    static const uint8_t MOTOR4_DIR_PIN = A2;
    static const uint8_t MOTOR5_STEP_PIN = A5;
    static const uint8_t MOTOR5_DIR_PIN = A4;

    // AccelStepper objects for each motor
    // Assuming 1 = DRIVER interface (STEP/DIR pins)
    AccelStepper motor1;
    AccelStepper motor2;
    AccelStepper motor3;
    AccelStepper motor4;
    AccelStepper motor5;

    // Helper function to convert RPM to steps per second
    float rpmToStepsPerSecond(int rpm);

    // Assuming 200 steps per revolution for calculation
    const int stepsPerRevolution = 200;

    // Remove old private methods declarations
    // void setMotorSpeed(uint8_t stepPin, uint8_t dirPin, int rpm, uint8_t direction);
    // void stopMotor(uint8_t stepPin);
};

#endif