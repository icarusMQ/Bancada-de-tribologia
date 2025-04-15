#include "StepperMotorController.h"
#include <Arduino_FreeRTOS.h>

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
  
  // Initialize motor states
  for (int i = 0; i < MAX_MOTORS; i++) {
    motorRunning[i] = false;
    motorSpeed[i] = 0;
    motorDirection[i] = HIGH;
  }
  
  // Create a semaphore for motor control access
  motorMutex = xSemaphoreCreateMutex();
  
  // Create task for motor pulse generation
  xTaskCreate(
    MotorPulseTask,        // Task function
    "MotorPulseTask",      // Task name
    128,                   // Stack size (words)
    this,                  // Task parameters (pass the object pointer)
    3,                     // Priority
    &motorTaskHandle       // Task handle
  );
}

// Runs the specified motor at the given RPM and direction.
void StepperMotorController::RunMotor(uint8_t motorID, int rpm, uint8_t direction) {
  // Validate motor ID
  if (motorID < 1 || motorID > MAX_MOTORS) {
    Serial.println("Invalid motor ID. Use 1, 2, 3, or 4.");
    return;
  }
  
  // Convert to zero-based index
  uint8_t motorIndex = motorID - 1;
  
  // Get mutex before updating motor parameters
  if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
    // Update motor parameters
    motorRunning[motorIndex] = true;
    motorSpeed[motorIndex] = rpm;
    motorDirection[motorIndex] = direction;
    
    // Set direction pin
    switch(motorID) {
      case 1:
        digitalWrite(MOTOR1_DIR_PIN, direction);
        break;
      case 2:
        digitalWrite(MOTOR2_DIR_PIN, direction);
        break;
      case 3:
        digitalWrite(MOTOR3_DIR_PIN, direction);
        break;
      case 4:
        digitalWrite(MOTOR4_DIR_PIN, direction);
        break;
    }
    
    xSemaphoreGive(motorMutex);
  }
}

// Stops the specified motor.
void StepperMotorController::StopMotor(uint8_t motorID) {
  // Validate motor ID
  if (motorID < 1 || motorID > MAX_MOTORS) {
    Serial.println("Invalid motor ID. Use 1, 2, 3, or 4.");
    return;
  }
  
  // Convert to zero-based index
  uint8_t motorIndex = motorID - 1;
  
  // Get mutex before updating motor parameters
  if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
    // Update motor parameters
    motorRunning[motorIndex] = false;
    
    // Ensure motor step pin is low
    switch(motorID) {
      case 1:
        digitalWrite(MOTOR1_STEP_PIN, LOW);
        break;
      case 2:
        digitalWrite(MOTOR2_STEP_PIN, LOW);
        break;
      case 3:
        digitalWrite(MOTOR3_STEP_PIN, LOW);
        break;
      case 4:
        digitalWrite(MOTOR4_STEP_PIN, LOW);
        break;
    }
    
    xSemaphoreGive(motorMutex);
  }
}

// Static task function for motor pulse generation
void StepperMotorController::MotorPulseTask(void* pvParameters) {
  // Get a pointer to the controller instance
  StepperMotorController* controller = (StepperMotorController*)pvParameters;
  
  // Motor timing variables
  unsigned long lastStepTime[MAX_MOTORS] = {0};
  unsigned long stepInterval[MAX_MOTORS] = {0};
  bool stepState[MAX_MOTORS] = {false};
  
  // Main task loop
  for (;;) {
    // Lock access while getting motor parameters
    if (xSemaphoreTake(controller->motorMutex, 5) == pdTRUE) {
      // Update step intervals based on current RPM
      for (int i = 0; i < MAX_MOTORS; i++) {
        if (controller->motorRunning[i] && controller->motorSpeed[i] > 0) {
          // Calculate step interval in microseconds
          stepInterval[i] = 60000000UL / (200 * controller->motorSpeed[i]);
        }
      }
      xSemaphoreGive(controller->motorMutex);
    }
    
    // Current time in microseconds
    unsigned long currentMicros = micros();
    
    // Process each motor
    for (int i = 0; i < MAX_MOTORS; i++) {
      // Check if this motor should be running
      if (xSemaphoreTake(controller->motorMutex, 0) == pdTRUE) {
        bool isRunning = controller->motorRunning[i];
        xSemaphoreGive(controller->motorMutex);
        
        if (isRunning && stepInterval[i] > 0) {
          // Calculate if it's time for a step
          if (currentMicros - lastStepTime[i] >= (stepState[i] ? stepInterval[i]/2 : stepInterval[i]/2)) {
            lastStepTime[i] = currentMicros;
            
            // Toggle step pin state
            stepState[i] = !stepState[i];
            
            // Set the appropriate pin
            switch(i+1) { // Convert back to 1-based index
              case 1:
                digitalWrite(controller->MOTOR1_STEP_PIN, stepState[i] ? HIGH : LOW);
                break;
              case 2:
                digitalWrite(controller->MOTOR2_STEP_PIN, stepState[i] ? HIGH : LOW);
                break;
              case 3:
                digitalWrite(controller->MOTOR3_STEP_PIN, stepState[i] ? HIGH : LOW);
                break;
              case 4:
                digitalWrite(controller->MOTOR4_STEP_PIN, stepState[i] ? HIGH : LOW);
                break;
            }
          }
        }
      }
    }
    
    // Short delay to prevent task from hogging CPU
    vTaskDelay(1); // Just yield to other tasks
  }
}