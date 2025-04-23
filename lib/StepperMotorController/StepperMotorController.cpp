#include "StepperMotorController.h"

// Constructor: Initializes the AccelStepper objects.
StepperMotorController::StepperMotorController() :
  motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN),
  motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN),
  motor3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN),
  motor4(AccelStepper::DRIVER, MOTOR4_STEP_PIN, MOTOR4_DIR_PIN),
  motor5(AccelStepper::DRIVER, MOTOR5_STEP_PIN, MOTOR5_DIR_PIN)
{
  // Set some default maximum speed and acceleration
  // Adjust these values based on your motors and power supply
  motor1.setMaxSpeed(1000); // steps per second
  motor1.setAcceleration(500); // steps per second squared
  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(500);
  motor3.setMaxSpeed(1000);
  motor3.setAcceleration(500);
  motor4.setMaxSpeed(1000);
  motor4.setAcceleration(500);
  motor5.setMaxSpeed(1000);
  motor5.setAcceleration(500);
}

// Helper function to convert RPM to steps per second
float StepperMotorController::rpmToStepsPerSecond(int rpm) {
    // Assuming 200 steps per revolution
    return (float)stepsPerRevolution * rpm / 60.0;
}

// Sets the target speed for the specified motor.
// Positive RPM for forward, negative RPM for reverse.
void StepperMotorController::RunMotor(uint8_t motorID, int rpm) {
  float speed = rpmToStepsPerSecond(rpm);
  switch (motorID) {
    case 1:
      motor1.setSpeed(speed);
      break;
    case 2:
      motor2.setSpeed(speed);
      break;
    case 3:
      motor3.setSpeed(speed);
      break;
    case 4:
      motor4.setSpeed(speed);
      break;
    case 5:
      motor5.setSpeed(speed);
      break;
    default:
      Serial.println("Invalid motor ID. Use 1, 2, 3, 4, or 5.");
      break;
  }
}

// Stops the specified motor by setting its speed to 0.
void StepperMotorController::StopMotor(uint8_t motorID) {
  switch (motorID) {
    case 1:
      motor1.setSpeed(0);
      // Optionally, use motor1.stop() for abrupt stop and deceleration
      // motor1.stop();
      // motor1.runToPosition(); // Wait for stop
      break;
    case 2:
      motor2.setSpeed(0);
      break;
    case 3:
      motor3.setSpeed(0);
      break;
    case 4:
      motor4.setSpeed(0);
      break;
    case 5:
      motor5.setSpeed(0);
      break;
    default:
      Serial.println("Invalid motor ID. Use 1, 2, 3, 4, or 5.");
      break;
  }
}

// Updates the state of all motors. Call this frequently in loop().
void StepperMotorController::UpdateMotors() {
  motor1.runSpeed();
  motor2.runSpeed();
  motor3.runSpeed();
  motor4.runSpeed();
  motor5.runSpeed();
}