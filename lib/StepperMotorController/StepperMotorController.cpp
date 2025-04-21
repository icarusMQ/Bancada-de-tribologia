#include "StepperMotorController.h"

StepperMotorController::StepperMotorController() {
  // Initialize motor pins.
  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_STEP_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR3_STEP_PIN, OUTPUT);
  pinMode(MOTOR3_DIR_PIN, OUTPUT);
  pinMode(MOTOR4_STEP_PIN, OUTPUT);
  pinMode(MOTOR4_DIR_PIN, OUTPUT);
  pinMode(MOTOR5_STEP_PIN, OUTPUT);
  pinMode(MOTOR5_DIR_PIN, OUTPUT);
  
  // Initialize motor state arrays.
  for (int i = 0; i < MAX_MOTORS; i++) {
    motorRunning[i] = false;
    motorSpeed[i] = 0;
    motorDirection[i] = HIGH;
  }
  
  // Create a mutex for motor access.
  motorMutex = xSemaphoreCreateMutex();
  // Note: Do not call any RTOS tasks here; use begin() instead.
  motorTaskHandle = NULL;
}

void StepperMotorController::begin() {
  // Create the MotorPulseTask after the scheduler is running.
  BaseType_t result = xTaskCreate(
    MotorPulseTask,         // Task function
    "MotorPulseTask",       // Task name
    256,                    // Stack size (words)
    this,                   // Task parameter (pass this instance)
    3,                      // Priority
    &motorTaskHandle        // Task handle
  );
  
  if(result != pdPASS) {
    Serial.println("Error: Could not create MotorPulseTask");
  }
}

// Run a motor (1 to 5) with the given RPM and direction.
void StepperMotorController::RunMotor(uint8_t motorID, int rpm, uint8_t direction) {
  if (motorID < 1 || motorID > MAX_MOTORS) {
    Serial.println("Invalid motor ID. Use 1 to 5.");
    return;
  }
  
  uint8_t index = motorID - 1;
  if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
    motorRunning[index] = true;
    motorSpeed[index] = rpm;
    motorDirection[index] = direction;
    
    // Update the direction pin.
    switch(motorID) {
      case 1: digitalWrite(MOTOR1_DIR_PIN, direction); break;
      case 2: digitalWrite(MOTOR2_DIR_PIN, direction); break;
      case 3: digitalWrite(MOTOR3_DIR_PIN, direction); break;
      case 4: digitalWrite(MOTOR4_DIR_PIN, direction); break;
      case 5: digitalWrite(MOTOR5_DIR_PIN, direction); break;
    }
    xSemaphoreGive(motorMutex);
  }
}

// Stop a motor (1 to 5).
void StepperMotorController::StopMotor(uint8_t motorID) {
  if (motorID < 1 || motorID > MAX_MOTORS) {
    Serial.println("Invalid motor ID. Use 1 to 5.");
    return;
  }
  
  uint8_t index = motorID - 1;
  if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
    motorRunning[index] = false;
    
    // Ensure the step pin is low.
    switch(motorID) {
      case 1: digitalWrite(MOTOR1_STEP_PIN, LOW); break;
      case 2: digitalWrite(MOTOR2_STEP_PIN, LOW); break;
      case 3: digitalWrite(MOTOR3_STEP_PIN, LOW); break;
      case 4: digitalWrite(MOTOR4_STEP_PIN, LOW); break;
      case 5: digitalWrite(MOTOR5_STEP_PIN, LOW); break;
    }
    
    xSemaphoreGive(motorMutex);
  }
}

// MotorPulseTask: generates pulses for each active motor.
// Note: This task assumes 200 steps per revolution for calculating step intervals.
void StepperMotorController::MotorPulseTask(void* pvParameters) {
  StepperMotorController* controller = (StepperMotorController*) pvParameters;
  
  // Timing and state arrays for each motor.
  unsigned long lastStepTime[MAX_MOTORS] = {0};
  unsigned long stepInterval[MAX_MOTORS] = {0};
  bool stepState[MAX_MOTORS] = {false};
  
  for(;;) {
    // Update step intervals based on current RPM.
    if (xSemaphoreTake(controller->motorMutex, 10) == pdTRUE) {
      for (int i = 0; i < MAX_MOTORS; i++) {
        if (controller->motorRunning[i] && controller->motorSpeed[i] > 0) {
          // Calculate the step interval (in microseconds).
          stepInterval[i] = 60000000UL / (200 * controller->motorSpeed[i]);
        } else {
          stepInterval[i] = 0;
        }
      }
      xSemaphoreGive(controller->motorMutex);
    }
    
    unsigned long currentMicros = micros();
    // Process each motor.
    for (int i = 0; i < MAX_MOTORS; i++) {
      // Check motor state.
      if (xSemaphoreTake(controller->motorMutex, 10) == pdTRUE) {
        bool running = controller->motorRunning[i];
        xSemaphoreGive(controller->motorMutex);
        
        if (running && stepInterval[i] > 0) {
          // Toggle the motor step pin when the time for a half-period has elapsed.
          if (currentMicros - lastStepTime[i] >= (stepInterval[i] / 2)) {
            lastStepTime[i] = currentMicros;
            stepState[i] = !stepState[i];
            
            // Set the step pin based on motor index.
            switch(i + 1) {
              case 1: digitalWrite(controller->MOTOR1_STEP_PIN, stepState[i] ? HIGH : LOW); break;
              case 2: digitalWrite(controller->MOTOR2_STEP_PIN, stepState[i] ? HIGH : LOW); break;
              case 3: digitalWrite(controller->MOTOR3_STEP_PIN, stepState[i] ? HIGH : LOW); break;
              case 4: digitalWrite(controller->MOTOR4_STEP_PIN, stepState[i] ? HIGH : LOW); break;
              case 5: digitalWrite(controller->MOTOR5_STEP_PIN, stepState[i] ? HIGH : LOW); break;
            }
          }
        }
      }
    }
    
    // Short delay to yield to other tasks.
    vTaskDelay(1);
  }
}
