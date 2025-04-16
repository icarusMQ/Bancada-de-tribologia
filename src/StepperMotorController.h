#ifndef STEPPERMOTORCONTROLLER_H
#define STEPPERMOTORCONTROLLER_H

#include "Arduino.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// This class manages 5 stepper motors using FreeRTOS. It provides
// deferred initialization (via begin()) so that RTOS functions are only used
// after the scheduler has started.
class StepperMotorController {
  public:
    // Constructor: Only perform low-level initialization that does not depend on the scheduler.
    StepperMotorController();
    
    // Deferred initialization: Call this from a task (after the scheduler starts) to set up RTOS objects.
    void begin();
    
    // Runs the specified motor (1 to 5) at the given RPM and direction.
    void RunMotor(uint8_t motorID, int rpm, uint8_t direction);
    
    // Stops the specified motor (1 to 5).
    void StopMotor(uint8_t motorID);

  private:
    // Motor pin definitions for 5 motors
    static const uint8_t MOTOR1_STEP_PIN = 2;
    static const uint8_t MOTOR1_DIR_PIN  = 3;
    static const uint8_t MOTOR2_STEP_PIN = 4;
    static const uint8_t MOTOR2_DIR_PIN  = 5;
    static const uint8_t MOTOR3_STEP_PIN = 6;
    static const uint8_t MOTOR3_DIR_PIN  = 7;
    static const uint8_t MOTOR4_STEP_PIN = 8;
    static const uint8_t MOTOR4_DIR_PIN  = 9;
    static const uint8_t MOTOR5_STEP_PIN = 10;
    static const uint8_t MOTOR5_DIR_PIN  = 11;
    
    // Total number of motors.
    static const uint8_t MAX_MOTORS = 5;
    
    // Arrays for motor state.
    bool motorRunning[MAX_MOTORS];
    int motorSpeed[MAX_MOTORS];
    uint8_t motorDirection[MAX_MOTORS];
    
    // RTOS objects.
    SemaphoreHandle_t motorMutex;
    TaskHandle_t motorTaskHandle;
    
    // Static task function for generating motor pulses.
    static void MotorPulseTask(void* pvParameters);
};

#endif
