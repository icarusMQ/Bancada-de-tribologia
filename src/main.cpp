#include "MultiScaleSensors.h"
#include "StepperMotorController.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// Create instances of the libraries
MultiScaleSensors sensors;
StepperMotorController motorController;

// Constants for the experiment
// Motors assignment:
// 1 - Flow control for free sphere
// 2 - Flow control for fixed sphere
// 3 - Distance control for free sphere
// 4 - Distance control for fixed sphere
// 5 - Load control motor for free sphere (adjust position to change applied force)

// Rotation speed setting (RPM)
const int ROTATION_SPEED = 80;

// Flow rate setting (simulating 30 drops per minute)
const int FLOW_SPEED = 10; // Adjust this value to achieve 30 drops/min

// Current experiment parameters
float targetLoadFree = 0.1; // Starting with 0.1N for free sphere
float targetLoadFixed = 5.0; // Starting with 5N for fixed sphere
bool experimentRunning = false;

// RTOS synchronization objects
SemaphoreHandle_t paramMutex;

// Task function prototypes
void CommandHandlerTask(void *pvParameters);
void LoadControlTask(void *pvParameters);
void SensorReadTask(void *pvParameters);
void StatusReportTask(void *pvParameters);

// Function prototypes
void adjustFreeLoad(float currentLoad, float targetLoad);
void startExperiment(float freeLoad, float fixedLoad);
void stopExperiment();
void printStatus();

void setup() {
  Serial.begin(115200);
  Serial.println("Tribology Experimental Setup with RTOS");
  Serial.println("-------------------------------------");
  
  // Initialize sensors
  if (!sensors.BeginSensors()) {
    Serial.println("WARNING: No sensors responded properly during startup!");
    Serial.println("Check connections and restart the system.");
  } else {
    Serial.println("Sensors initialized successfully.");
  }
  
  Serial.println("\nExperiment ready. Send commands via Serial:");
  Serial.println("start - Start experiment with current parameters");
  Serial.println("stop - Stop experiment");
  Serial.println("free:X.X - Set free sphere load (N)");
  Serial.println("fixed:X.X - Set fixed sphere load (N)");
  Serial.println("status - Display current status");

  // Create mutex for parameter protection
  paramMutex = xSemaphoreCreateMutex();
  
  // Create RTOS tasks
  xTaskCreate(
    CommandHandlerTask,     // Task function
    "CommandHandler",       // Task name
    128,                    // Stack size (words)
    NULL,                   // Parameters
    3,                      // Priority (higher number = higher priority)
    NULL                    // Task handle
  );
  
  xTaskCreate(
    LoadControlTask,
    "LoadControl",
    128,
    NULL,
    2,
    NULL
  );
  
  xTaskCreate(
    SensorReadTask,
    "SensorRead",
    128,
    NULL,
    1,
    NULL
  );
  
  xTaskCreate(
    StatusReportTask,
    "StatusReport",
    128,
    NULL,
    1,
    NULL
  );
  
  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty - everything is handled by RTOS tasks
}

// Task to handle serial commands
void CommandHandlerTask(void *pvParameters) {
  String command = "";
  char inChar;
  
  for (;;) {
    if (Serial.available() > 0) {
      inChar = Serial.read();
      
      if (inChar == '\n') {
        command.trim();
        
        if (command == "start") {
          // Take mutex before accessing shared parameters
          if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
            startExperiment(targetLoadFree, targetLoadFixed);
            xSemaphoreGive(paramMutex);
          }
        } 
        else if (command == "stop") {
          if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
            stopExperiment();
            xSemaphoreGive(paramMutex);
          }
        }
        else if (command == "status") {
          if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
            printStatus();
            xSemaphoreGive(paramMutex);
          }
        }
        else if (command.startsWith("free:")) {
          float newLoad = command.substring(5).toFloat();
          if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
            if (newLoad >= 0.05 && newLoad <= 1.0) {
              targetLoadFree = newLoad;
              Serial.print("Free sphere target load set to: ");
              Serial.print(targetLoadFree, 2);
              Serial.println(" N");
            } else {
              Serial.println("Invalid free sphere load. Use value between 0.05 and 1.0 N");
            }
            xSemaphoreGive(paramMutex);
          }
        }
        else if (command.startsWith("fixed:")) {
          float newLoad = command.substring(6).toFloat();
          if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
            if (newLoad >= 1.0 && newLoad <= 20.0) {
              targetLoadFixed = newLoad;
              Serial.print("Fixed sphere target load set to: ");
              Serial.print(targetLoadFixed, 2);
              Serial.println(" N");
            } else {
              Serial.println("Invalid fixed sphere load. Use value between 1.0 and 20.0 N");
            }
            xSemaphoreGive(paramMutex);
          }
        }
        else {
          Serial.println("Unknown command");
        }
        
        command = ""; // Clear command for next input
      } else {
        command += inChar;
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); // Small delay to prevent CPU hogging
  }
}

// Task to control the load of the free sphere
void LoadControlTask(void *pvParameters) {
  float currentLoad, targetLoad;
  bool running;
  
  for (;;) {
    // Take mutex before accessing shared parameters
    if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
      running = experimentRunning;
      targetLoad = targetLoadFree;
      xSemaphoreGive(paramMutex);
      
      if (running) {
        currentLoad = sensors.ReadSensor(1); // Free sphere load sensor
        adjustFreeLoad(currentLoad, targetLoad);
      }
    }
    
    vTaskDelay(200 / portTICK_PERIOD_MS); // Adjust load every 200ms
  }
}

// Task to read and store sensor values
void SensorReadTask(void *pvParameters) {
  float freeSphereLoad, fixedSphereLoad, auxSensor1, auxSensor2;
  
  for (;;) {
    // Read all sensors
    freeSphereLoad = sensors.ReadSensor(1);
    fixedSphereLoad = sensors.ReadSensor(2);
    auxSensor1 = sensors.ReadSensor(3);
    auxSensor2 = sensors.ReadSensor(4);
    
    // Optionally, store values in a data buffer or SD card
    // This task could be expanded for data logging
    
    vTaskDelay(100 / portTICK_PERIOD_MS); // Read sensors every 100ms
  }
}

// Task to report status periodically
void StatusReportTask(void *pvParameters) {
  bool running;
  
  for (;;) {
    // Take mutex before accessing shared parameters
    if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
      running = experimentRunning;
      xSemaphoreGive(paramMutex);
      
      if (running) {
        float freeSphereLoad = sensors.ReadSensor(1);
        float fixedSphereLoad = sensors.ReadSensor(2);
        float auxSensor1 = sensors.ReadSensor(3);
        float auxSensor2 = sensors.ReadSensor(4);
        
        Serial.print("Free sphere: ");
        Serial.print(freeSphereLoad, 2);
        Serial.print("N | Fixed sphere: ");
        Serial.print(fixedSphereLoad, 2);
        Serial.print("N | Aux1: ");
        Serial.print(auxSensor1, 2);
        Serial.print(" | Aux2: ");
        Serial.println(auxSensor2, 2);
      }
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Report every 1 second
  }
}

void startExperiment(float freeLoad, float fixedLoad) {
  if (experimentRunning) {
    Serial.println("Experiment already running");
    return;
  }
  
  Serial.println("Starting experiment with parameters:");
  Serial.print("- Free sphere load: ");
  Serial.print(freeLoad, 2);
  Serial.println(" N");
  Serial.print("- Fixed sphere load: ");
  Serial.print(fixedLoad, 2);
  Serial.println(" N");
  Serial.println("- Rotation speed: 80 RPM");
  Serial.println("- Flow rate: 30 drops/min");
  Serial.println("- Distance: 30 meters");
  
  // Start flow control motors (1 and 2)
  motorController.RunMotor(1, FLOW_SPEED, HIGH);
  motorController.RunMotor(2, FLOW_SPEED, HIGH);
  
  // Start rotation motors (3 and 4) at 80 RPM
  motorController.RunMotor(3, ROTATION_SPEED, HIGH);
  motorController.RunMotor(4, ROTATION_SPEED, HIGH);
  
  experimentRunning = true;
}

void stopExperiment() {
  if (!experimentRunning) {
    Serial.println("No experiment running");
    return;
  }
  
  // Stop all motors
  for (int i = 1; i <= 5; i++) {
    motorController.StopMotor(i);
  }
  
  Serial.println("Experiment stopped");
  experimentRunning = false;
}

void adjustFreeLoad(float currentLoad, float targetLoad) {
  // Simple proportional control for the load
  // Motor 5 controls the position of the loading mechanism
  
  // Calculate error
  float error = targetLoad - currentLoad;
  
  // If load is within acceptable range (±5%), do nothing
  if (abs(error) < (targetLoad * 0.05)) {
    motorController.StopMotor(5);
    return;
  }
  
  // Determine direction and speed based on error
  uint8_t direction = (error > 0) ? HIGH : LOW; // HIGH to increase load, LOW to decrease
  
  // Calculate a proportional speed (larger error = faster adjustment)
  // Limit speed between 5 and 20 RPM for precise control
  int speed = constrain(abs(error) * 20, 5, 20);
  
  // Run the motor to adjust the load
  motorController.RunMotor(5, speed, direction);
  
  // Add a small delay to allow the mechanism to move
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
  // Stop the motor to prevent overshooting
  motorController.StopMotor(5);
}

void printStatus() {
  Serial.println("\n=== System Status ===");
  Serial.print("Experiment running: ");
  Serial.println(experimentRunning ? "Yes" : "No");
  
  Serial.print("Target free sphere load: ");
  Serial.print(targetLoadFree, 2);
  Serial.println(" N");
  
  Serial.print("Target fixed sphere load: ");
  Serial.print(targetLoadFixed, 2);
  Serial.println(" N");
  
  Serial.print("Current free sphere load: ");
  Serial.print(sensors.ReadSensor(1), 2);
  Serial.println(" N");
  
  Serial.print("Current fixed sphere load: ");
  Serial.print(sensors.ReadSensor(2), 2);
  Serial.println(" N");
  
  Serial.println("== Experiment Parameters ==");
  Serial.println("- Rotation speed: 80 RPM");
  Serial.println("- Flow rate: 30 drops/min");
  Serial.println("- Alumina: 1 micron metallographic");
  Serial.println("- Distance: 30 meters");
  Serial.println("=======================");
}
