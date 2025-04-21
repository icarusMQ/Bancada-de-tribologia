#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <MultiScaleSensors.h>
#include <StepperMotorController.h>


// Global objects
MultiScaleSensors sensors;
StepperMotorController motorController;

// Experiment parameters
const int ROTATION_SPEED = 80;
const int FLOW_SPEED = 10; // Simulating 30 drops/min (tweak as required)
float targetLoadFree = 0.1;   // Free sphere load (N)
float targetLoadFixed = 5.0;    // Fixed sphere load (N)
bool experimentRunning = false;

// RTOS synchronization object for shared parameters.
SemaphoreHandle_t paramMutex;

// Task function prototypes.
void InitTask(void *pvParameters);
void CommandHandlerTask(void *pvParameters);
void LoadControlTask(void *pvParameters);
void SensorReadTask(void *pvParameters);
void StatusReportTask(void *pvParameters);

// Function prototypes.
void adjustFreeLoad(float currentLoad, float targetLoad);
void startExperiment(float freeLoad, float fixedLoad);
void stopExperiment();
void printStatus();

void setup() {
  // Initialize Serial (only once).
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial (particularly for boards like Leonardo)
  }
  // (Do not use any blocking delay() calls here that may interfere with scheduler startup.)
  
  Serial.println("Tribology Experimental Setup with RTOS");
  Serial.println("-------------------------------------");
  
  // Create the mutex for parameter protection.
  paramMutex = xSemaphoreCreateMutex();
  if (paramMutex == NULL) {
    Serial.println("Error: Could not create paramMutex");
  }
  
  // Create the initialization task.
  BaseType_t result;
  result = xTaskCreate(InitTask, "InitTask", 256, NULL, 4, NULL);
  if (result != pdPASS) {
    Serial.println("Error: Could not create InitTask");
  }
  
  // Create other RTOS tasks.
  result = xTaskCreate(CommandHandlerTask, "CommandHandler", 300, NULL, 3, NULL);
  if (result != pdPASS) {
    Serial.println("Error: Could not create CommandHandlerTask");
  }
  
  result = xTaskCreate(LoadControlTask, "LoadControl", 300, NULL, 2, NULL);
  if (result != pdPASS) {
    Serial.println("Error: Could not create LoadControlTask");
  }
  
  result = xTaskCreate(SensorReadTask, "SensorRead", 200, NULL, 1, NULL);
  if (result != pdPASS) {
    Serial.println("Error: Could not create SensorReadTask");
  }
  
  result = xTaskCreate(StatusReportTask, "StatusReport", 200, NULL, 1, NULL);
  if (result != pdPASS) {
    Serial.println("Error: Could not create StatusReportTask");
  }
  
  // Start the FreeRTOS scheduler.
  vTaskStartScheduler();
}

void loop() {
  // Empty; all activities are managed by tasks.
}

/*--------------------------------------------------
   InitTask:
   This initialization task runs after the scheduler starts.
   It performs sensor initialization and starts the motor
   controller’s RTOS task (via motorController.begin()).
---------------------------------------------------*/
void InitTask(void *pvParameters) {
  Serial.println("InitTask: Starting deferred initialization...");

  // Sensor initialization.
  // Replace blocking delay() calls in sensors.BeginSensors() with vTaskDelay()
  // in the library so that we do not block the RTOS. (Update the MultiScaleSensors
  // library accordingly.)
  if (!sensors.BeginSensors()) {
    Serial.println("WARNING: One or more sensors failed to initialize.");
  } else {
    Serial.println("Sensors initialized successfully.");
  }

  // Initialize the motor controller (which creates its motor pulse task).
  motorController.begin();
  Serial.println("Motor controller initialized.");

  // Initialization complete, delete this task.
  Serial.println("InitTask: Initialization complete. Deleting InitTask...");
  vTaskDelete(NULL);
}

/*--------------------------------------------------
   CommandHandlerTask:
   Reads commands from Serial to control the experiment.
---------------------------------------------------*/
void CommandHandlerTask(void *pvParameters) {
  String command = "";
  char inChar;
  
  for (;;) {
    if (Serial.available() > 0) {
      inChar = Serial.read();
      if (inChar == '\n') {
        command.trim();
        if (command == "start") {
          if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
            startExperiment(targetLoadFree, targetLoadFixed);
            xSemaphoreGive(paramMutex);
          }
        } else if (command == "stop") {
          if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
            stopExperiment();
            xSemaphoreGive(paramMutex);
          }
        } else if (command == "status") {
          if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
            printStatus();
            xSemaphoreGive(paramMutex);
          }
        } else if (command.startsWith("free:")) {
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
        } else if (command.startsWith("fixed:")) {
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
        } else {
          Serial.println("Unknown command");
        }
        command = "";
      } else {
        command += inChar;
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

/*--------------------------------------------------
   LoadControlTask:
   Manages the free sphere load via sensor feedback and adjusts
   motor 5 accordingly.
---------------------------------------------------*/
void LoadControlTask(void *pvParameters) {
  float currentLoad, targetLoad;
  bool running;
  
  for (;;) {
    if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
      running = experimentRunning;
      targetLoad = targetLoadFree;
      xSemaphoreGive(paramMutex);
      
      if (running) {
        currentLoad = sensors.ReadSensor(1); // Free sphere sensor
        adjustFreeLoad(currentLoad, targetLoad);
      }
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

/*--------------------------------------------------
   SensorReadTask:
   Periodically reads sensor values.
---------------------------------------------------*/
void SensorReadTask(void *pvParameters) {
  float freeSphereLoad, fixedSphereLoad, auxSensor1, auxSensor2;
  
  for (;;) {
    freeSphereLoad = sensors.ReadSensor(1);
    fixedSphereLoad = sensors.ReadSensor(2);
    auxSensor1 = sensors.ReadSensor(3);
    auxSensor2 = sensors.ReadSensor(4);
    
    // Optionally store or log the sensor data.
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/*--------------------------------------------------
   StatusReportTask:
   Periodically prints the status of the experiment.
---------------------------------------------------*/
void StatusReportTask(void *pvParameters) {
  bool running;
  
  for (;;) {
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
        Serial.print(" N | Fixed sphere: ");
        Serial.print(fixedSphereLoad, 2);
        Serial.print(" N | Aux1: ");
        Serial.print(auxSensor1, 2);
        Serial.print(" | Aux2: ");
        Serial.println(auxSensor2, 2);
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

/*--------------------------------------------------
   Experiment Control Functions
---------------------------------------------------*/
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
  
  // Start flow control motors: motors 1 & 2.
  motorController.RunMotor(1, FLOW_SPEED, HIGH);
  motorController.RunMotor(2, FLOW_SPEED, HIGH);
  
  // Start rotation motors: motors 3 & 4.
  motorController.RunMotor(3, ROTATION_SPEED, HIGH);
  motorController.RunMotor(4, ROTATION_SPEED, HIGH);
  
  experimentRunning = true;
}

void stopExperiment() {
  if (!experimentRunning) {
    Serial.println("No experiment running");
    return;
  }
  
  // Stop all motors (motors 1 to 5).
  for (int i = 1; i <= 5; i++) {
    motorController.StopMotor(i);
  }
  
  Serial.println("Experiment stopped");
  experimentRunning = false;
}

void adjustFreeLoad(float currentLoad, float targetLoad) {
  float error = targetLoad - currentLoad;
  
  // If within ±5%, no adjustment is needed.
  if (abs(error) < (targetLoad * 0.05)) {
    motorController.StopMotor(5);
    return;
  }
  
  // Determine motor direction.
  uint8_t direction = (error > 0) ? HIGH : LOW;
  
  // Calculate a proportional speed between 5 and 20 RPM.
  int speed = constrain(abs(error) * 20, 5, 20);
  motorController.RunMotor(5, speed, direction);
  
  // Allow a brief adjustment interval.
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
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
