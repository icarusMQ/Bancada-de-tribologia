#ifndef STEPPERMOTORCONTROLLER_H
#define STEPPERMOTORCONTROLLER_H

#include "Arduino.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// This class manages multiple stepper motors.
// It provides functionality to run and stop motors with specified RPMs and directions.
class StepperMotorController {
  public:
    // Constructor: Initializes the class variables.
    StepperMotorController();

    // Runs the specified motor at the given RPM and direction.
    // Parameters:
    // - motorID: The ID of the motor to run (1, 2, 3, or 4).
    // - rpm: The speed in revolutions per minute (RPM).
    // - direction: The direction to run the motor (HIGH for forward, LOW for reverse).
    void RunMotor(uint8_t motorID, int rpm, uint8_t direction);

    // Stops the specified motor.
    // Parameters:
    // - motorID: The ID of the motor to stop (1, 2, 3, or 4).
    void StopMotor(uint8_t motorID);

  private:
    // Pin definitions
    static const uint8_t MOTOR1_STEP_PIN = 2;
    static const uint8_t MOTOR1_DIR_PIN = 3;
    static const uint8_t MOTOR2_STEP_PIN = 4;
    static const uint8_t MOTOR2_DIR_PIN = 5;
    static const uint8_t MOTOR3_STEP_PIN = 6;
    static const uint8_t MOTOR3_DIR_PIN = 7;
    static const uint8_t MOTOR4_STEP_PIN = 8;
    static const uint8_t MOTOR4_DIR_PIN = 9;
    
    // Constants
    static const uint8_t MAX_MOTORS = 4;
    
    // Motor state arrays
    bool motorRunning[MAX_MOTORS];  // Track if each motor is running
    int motorSpeed[MAX_MOTORS];     // Current RPM of each motor
    uint8_t motorDirection[MAX_MOTORS]; // Current direction of each motor
    
    // RTOS objects
    SemaphoreHandle_t motorMutex;   // Mutex for motor state access
    TaskHandle_t motorTaskHandle;   // Handle to the motor pulse task
    
    // Static task function for motor pulse generation
    static void MotorPulseTask(void* pvParameters);
};

#endif