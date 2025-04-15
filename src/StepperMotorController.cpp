#include "StepperMotorController.h"

// Constructor: Initializes the class variables and sets up the motor pins.
StepperMotorController::StepperMotorController() {
  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_STEP_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR3_STEP_PIN, OUTPUT);
  pinMode(MOTOR3_DIR_PIN, OUTPUT);
  pinMode(MOTOR4_STEP_PIN, OUTPUT);
  pinMode(MOTOR4_DIR_PIN, OUTPUT);
}

// Runs the specified motor at the given RPM and direction.
void StepperMotorController::RunMotor(uint8_t motorID, int rpm, uint8_t direction) {
  switch (motorID) {
    case 1:
      setMotorSpeed(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, rpm, direction);
      break;
    case 2:
      setMotorSpeed(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, rpm, direction);
      break;
    case 3:
      setMotorSpeed(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, rpm, direction);
      break;
    case 4:
      setMotorSpeed(MOTOR4_STEP_PIN, MOTOR4_DIR_PIN, rpm, direction);
      break;
    default:
      Serial.println("Invalid motor ID. Use 1, 2, 3, or 4.");
      break;
  }
}

// Stops the specified motor.
void StepperMotorController::StopMotor(uint8_t motorID) {
  switch (motorID) {
    case 1:
      stopMotor(MOTOR1_STEP_PIN);
      break;
    case 2:
      stopMotor(MOTOR2_STEP_PIN);
      break;
    case 3:
      stopMotor(MOTOR3_STEP_PIN);
      break;
    case 4:
      stopMotor(MOTOR4_STEP_PIN);
      break;
    default:
      Serial.println("Invalid motor ID. Use 1, 2, 3, or 4.");
      break;
  }
}

// Helper function to set the RPM and direction for a motor.
void StepperMotorController::setMotorSpeed(uint8_t stepPin, uint8_t dirPin, int rpm, uint8_t direction) {
  // Set the direction.
  digitalWrite(dirPin, direction);

  // Calculate the delay between steps based on the RPM.
  int delayMicrosecondsPerStep = 60000000 / (200 * rpm); // Assuming 200 steps per revolution.

  // Example: Run the motor for a short duration (adjust as needed).
  for (int i = 0; i < 200; i++) { // Example: 1 revolution.
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayMicrosecondsPerStep / 2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayMicrosecondsPerStep / 2);
  }
}

// Helper function to stop a motor.
void StepperMotorController::stopMotor(uint8_t stepPin) {
  digitalWrite(stepPin, LOW);
}