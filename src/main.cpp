/**
 * @file main.cpp
 * @brief Main application file for the Tribology Experimental Setup using FreeRTOS.
 *
 * This file initializes hardware, creates RTOS tasks for managing sensors,
 * motors, user commands (Serial and Nextion), and experiment logic.
 * It utilizes mutexes for thread-safe access to shared experiment parameters
 * and sensor data. Implements PID control for load adjustment and watchdog timer.
 *
 * @note Assumes specific hardware connections detailed within the code.
 * @warning Check for pin conflicts between motors, sensors, and Nextion display.
 */

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <avr/wdt.h> // Include for Watchdog Timer
#include "MultiScaleSensors.h"
#include "StepperMotorController.h"
#include <Nextion.h>  // Include Nextion library

// --- Global Objects ---
MultiScaleSensors sensors;             ///< Object for managing load cell sensors.
StepperMotorController motorController; ///< Object for managing stepper motors.

// --- Nextion Display Objects ---
NexButton btnStart = NexButton(0, 1, "bStart");       ///< Nextion Start button object (Page 0, ID 1).
NexButton btnStop = NexButton(0, 2, "bStop");         ///< Nextion Stop button object (Page 0, ID 2).
NexSlider sliderFree = NexSlider(0, 3, "sFree");      ///< Nextion Free Load slider object (Page 0, ID 3).
NexSlider sliderFixed = NexSlider(0, 4, "sFixed");    ///< Nextion Fixed Load slider object (Page 0, ID 4).
NexWaveform waveFormLoad = NexWaveform(0, 5, "wLoad"); ///< Nextion Load waveform object (Page 0, ID 5).
NexText txtFreeVal = NexText(0, 6, "tFreeVal");       ///< Nextion Free Load value text object (Page 0, ID 6).
NexText txtFixedVal = NexText(0, 7, "tFixedVal");     ///< Nextion Fixed Load value text object (Page 0, ID 7).
NexText txtStatus = NexText(0, 8, "tStatus");         ///< Nextion Status text object (Page 0, ID 8).

NexTouch *nex_listen_list[] = {
  &btnStart,
  &btnStop,
  &sliderFree,
  &sliderFixed,
  NULL // String terminated with NULL
};

// --- Experiment Parameters ---
// Constants for experiment control
const int ROTATION_SPEED = 80; ///< Default rotation speed in RPM for motors 3 & 4.
const int FLOW_SPEED = 10;     ///< Default flow speed in RPM for motors 1 & 2 (simulating drops/min).

// Target loads (can be changed via Serial or Nextion)
volatile float targetLoadFree = 0.1;   ///< Target load for the free sphere (N). Range: 0.05 - 1.0 N.
volatile float targetLoadFixed = 5.0;  ///< Target load for the fixed sphere (N). Range: 1.0 - 20.0 N.

// State variable
volatile bool experimentRunning = false; ///< Flag indicating if the experiment is currently active.

// --- Centralized Sensor Data ---
volatile float currentFreeLoad = 0.0;
volatile float currentFixedLoad = 0.0;
volatile float currentAux1 = 0.0;
volatile float currentAux2 = 0.0;

// --- PID Control Variables ---
// PID Tuning Parameters (Manually set these values)
const float Kp = 100.0; // Proportional Gain - TUNE ME!
const float Ki = 5.0;   // Integral Gain - TUNE ME!
const float Kd = 1.0;   // Derivative Gain - TUNE ME!

// PID State Variables
float pidSetpoint = 0.0; // Target load for PID (set from targetLoadFree)
float pidInput = 0.0;    // Current load reading (from currentFreeLoad)
float pidOutput = 0.0;   // PID controller output (motor speed/action)
float pidIntegral = 0.0;
float pidLastError = 0.0;
unsigned long pidLastTime = 0;
const float PID_OUT_MIN = -50.0; // Min PID output (e.g., reverse speed) - TUNE ME!
const float PID_OUT_MAX = 50.0;  // Max PID output (e.g., forward speed) - TUNE ME!
const float PID_INTEGRAL_LIMIT = 100.0; // Anti-windup limit - TUNE ME!

// --- RTOS Synchronization ---
/**
 * @brief Mutex to protect shared access to experiment parameters
 * (targetLoadFree, targetLoadFixed, experimentRunning) and centralized sensor data
 * (currentFreeLoad, currentFixedLoad, etc.) across different tasks.
 */
SemaphoreHandle_t paramMutex;

// --- Task Function Prototypes ---
void InitTask(void *pvParameters);           ///< Task for deferred hardware initialization.
void CommandHandlerTask(void *pvParameters); ///< Task for handling commands from Serial Monitor.
void LoadControlTask(void *pvParameters);    ///< Task for controlling the free sphere load motor using PID.
void SensorReadTask(void *pvParameters);     ///< Task for periodically reading sensors and updating globals.
void StatusReportTask(void *pvParameters);   ///< Task for reporting status to Serial Monitor.
void NextionTask(void *pvParameters);        ///< Task for handling Nextion display communication.

// --- Nextion Callback Function Prototypes ---
void btnStartCallback(void *ptr);   ///< Callback function for Nextion Start button press.
void btnStopCallback(void *ptr);    ///< Callback function for Nextion Stop button press.
void sliderFreeCallback(void *ptr); ///< Callback function for Nextion Free Load slider release.
void sliderFixedCallback(void *ptr);///< Callback function for Nextion Fixed Load slider release.

// --- Helper Function Prototypes ---
void computePID();                                        ///< Computes the PID output based on current state.
void applyPIDOutput(float output);                        ///< Applies the PID output to Motor 5.
void startExperiment();                                   ///< Starts the experiment sequence.
void stopExperiment();                                    ///< Stops the experiment sequence.
void printStatus();                                       ///< Prints the current system status to Serial Monitor.
void updateNextionValues(bool updateSliders);             ///< Updates values on the Nextion display.

// --- Constants for Nextion Slider Mapping ---
const float MIN_FREE_LOAD = 0.05;   ///< Minimum value for the free load slider (N).
const float MAX_FREE_LOAD = 1.0;    ///< Maximum value for the free load slider (N).
const float MIN_FIXED_LOAD = 1.0;   ///< Minimum value for the fixed load slider (N).
const float MAX_FIXED_LOAD = 20.0;  ///< Maximum value for the fixed load slider (N).
const uint32_t SLIDER_MIN_VAL = 0;  ///< Minimum raw value from Nextion sliders.
const uint32_t SLIDER_MAX_VAL = 100; ///< Maximum raw value from Nextion sliders.

/**
 * @brief Standard Arduino setup function. Initializes Serial, Watchdog,
 *        Mutex and creates the initial RTOS tasks.
 */
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for Serial port

  // IMPORTANT: Enable watchdog timer EARLY, before starting tasks.
  wdt_enable(WDTO_2S); // Enable 2-second watchdog timer
  wdt_reset(); // Reset watchdog timer

  Serial.println(F("Tribology Experimental Setup with RTOS - v2.1 (No EEPROM)"));
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F("Watchdog Enabled (2s timeout)"));

  // Create the mutex BEFORE any tasks that use it.
  paramMutex = xSemaphoreCreateMutex();
  if (paramMutex == NULL) {
    Serial.println(F("FATAL Error: Could not create paramMutex!"));
    while(1); // Halt
  }
  wdt_reset(); // Reset watchdog

  // Create the initialization task (runs once after scheduler starts).
  BaseType_t result;
  result = xTaskCreate(InitTask, "InitTask", 256, NULL, tskIDLE_PRIORITY + 4, NULL); // Reduced stack size
  if (result != pdPASS) { Serial.println(F("Error: Could not create InitTask")); }
  wdt_reset();

  // Create the command handler task.
  result = xTaskCreate(CommandHandlerTask, "CommandHandler", 300, NULL, tskIDLE_PRIORITY + 3, NULL); // Reduced stack size
  if (result != pdPASS) { Serial.println(F("Error: Could not create CommandHandlerTask")); }
  wdt_reset();

  // Create the load control task (PID).
  result = xTaskCreate(LoadControlTask, "LoadControl", 384, NULL, tskIDLE_PRIORITY + 2, NULL); // Keep stack for PID
  if (result != pdPASS) { Serial.println(F("Error: Could not create LoadControlTask")); }
  wdt_reset();

  // Create the sensor reading task.
  result = xTaskCreate(SensorReadTask, "SensorRead", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
  if (result != pdPASS) { Serial.println(F("Error: Could not create SensorReadTask")); }
  wdt_reset();

  // Create the status reporting task.
  result = xTaskCreate(StatusReportTask, "StatusReport", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
  if (result != pdPASS) { Serial.println(F("Error: Could not create StatusReportTask")); }
  wdt_reset();

  // Create the Nextion communication task.
  result = xTaskCreate(NextionTask, "NextionTask", 300, NULL, tskIDLE_PRIORITY + 2, NULL); // Reduced stack size
  if (result != pdPASS) { Serial.println(F("Error: Could not create NextionTask")); }
  wdt_reset();

  Serial.println(F("Starting FreeRTOS scheduler..."));
  // Start the FreeRTOS scheduler. This function should not return.
  vTaskStartScheduler();

  // Code below here should never execute.
  Serial.println(F("FATAL Error: Scheduler failed to start!"));
  while(1);
}

/**
 * @brief Standard Arduino loop function.
 *        Kept empty as all functionality is handled by FreeRTOS tasks.
 */
void loop() {
  // Empty.
}

/**
 * @brief RTOS task for deferred initialization.
 */
void InitTask(void *pvParameters) {
  wdt_reset();
  Serial.println(F("InitTask: Starting deferred initialization..."));

  // Initialize sensors.
  if (!sensors.BeginSensors()) {
    Serial.println(F("WARNING: One or more sensors failed to initialize. Check connections and pins."));
  } else {
    Serial.println(F("Sensors initialized successfully."));
  }
  wdt_reset();

  // Initialize the motor controller.
  motorController.begin();
  Serial.println(F("Motor controller initialized."));
  wdt_reset();

  // Initialization complete, delete this task.
  Serial.println(F("InitTask: Initialization complete. Deleting InitTask..."));
  vTaskDelete(NULL); // Delete self.
}

/**
 * @brief RTOS task for handling commands received via the Serial Monitor.
 */
void CommandHandlerTask(void *pvParameters) {
  String command = "";
  char inChar;

  Serial.println(F("CommandHandlerTask: Ready for commands (start, stop, status, free:X.X, fixed:Y.Y)"));

  for (;;) {
    wdt_reset(); // Reset watchdog in task loop

    if (Serial.available() > 0) {
      inChar = Serial.read();
      if (inChar == '\n' || inChar == '\r') {
        command.trim();
        if (command.length() > 0) {
          Serial.print(F("Received command: "));
          Serial.println(command);

          // Process commands
          if (command == "start") {
            if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
              startExperiment();
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
              if (newLoad >= MIN_FREE_LOAD && newLoad <= MAX_FREE_LOAD) {
                targetLoadFree = newLoad; // Update global variable
                Serial.print(F("Free sphere target load set to: "));
                Serial.print(targetLoadFree, 2);
                Serial.println(F(" N"));
                // Update Nextion display immediately
                updateNextionValues(true); // Update sliders too
              } else {
                Serial.print(F("Invalid free sphere load. Use value between "));
                Serial.print(MIN_FREE_LOAD, 2); Serial.print(F(" and ")); Serial.print(MAX_FREE_LOAD, 2); Serial.println(F(" N"));
              }
              xSemaphoreGive(paramMutex);
            }
          } else if (command.startsWith("fixed:")) {
            float newLoad = command.substring(6).toFloat();
             if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
              if (newLoad >= MIN_FIXED_LOAD && newLoad <= MAX_FIXED_LOAD) {
                targetLoadFixed = newLoad; // Update global variable
                Serial.print(F("Fixed sphere target load set to: "));
                Serial.print(targetLoadFixed, 2);
                Serial.println(F(" N"));
                 // Update Nextion display immediately
                updateNextionValues(true); // Update sliders too
              } else {
                 Serial.print(F("Invalid fixed sphere load. Use value between "));
                 Serial.print(MIN_FIXED_LOAD, 1); Serial.print(F(" and ")); Serial.print(MAX_FIXED_LOAD, 1); Serial.println(F(" N"));
              }
              xSemaphoreGive(paramMutex);
            }
          } else {
            Serial.println(F("Unknown command"));
          }
          command = ""; // Clear command buffer
        }
      } else {
        command += inChar; // Append character
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Yield
  }
}

/**
 * @brief RTOS task for controlling the free sphere load using PID.
 */
void LoadControlTask(void *pvParameters) {
  bool running;
  float targetLoad; // Local copy of target

  pidLastTime = micros(); // Initialize PID timer

  for (;;) {
    wdt_reset(); // Reset watchdog

    // Safely get the current state, target load, and current load reading.
    if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
      running = experimentRunning;
      targetLoad = targetLoadFree; // Use global target
      pidInput = currentFreeLoad; // Use centralized sensor reading
      xSemaphoreGive(paramMutex);

      if (running) {
        pidSetpoint = targetLoad; // Update PID setpoint
        computePID();             // Calculate PID output
        applyPIDOutput(pidOutput); // Apply output to motor 5
      } else {
        // Ensure motor 5 is stopped and PID is reset if experiment is not running.
        motorController.StopMotor(5);
        pidIntegral = 0; // Reset integral term
        pidLastError = 0;
        pidOutput = 0;
      }
    }
    // Control loop frequency - adjust as needed for PID stability.
    vTaskDelay(pdMS_TO_TICKS(100)); // Run PID loop at ~10 Hz
  }
}

/**
 * @brief RTOS task for periodically reading sensors and updating global variables.
 */
void SensorReadTask(void *pvParameters) {
  float tempFree, tempFixed, tempAux1, tempAux2;

  for (;;) {
    wdt_reset(); // Reset watchdog

    // Read sensor values.
    tempFree = sensors.ReadSensor(1);
    tempFixed = sensors.ReadSensor(2);
    tempAux1 = sensors.ReadSensor(3);
    tempAux2 = sensors.ReadSensor(4);

    // Safely update the global sensor reading variables.
    if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
      currentFreeLoad = tempFree;
      currentFixedLoad = tempFixed;
      currentAux1 = tempAux1;
      currentAux2 = tempAux2;
      xSemaphoreGive(paramMutex);
    }

    // Delay to set the reading frequency.
    vTaskDelay(pdMS_TO_TICKS(100)); // Read sensors at ~10 Hz.
  }
}

/**
 * @brief RTOS task for periodically reporting system status to the Serial Monitor.
 */
void StatusReportTask(void *pvParameters) {
  bool running;
  float freeLoad, fixedLoad, aux1, aux2; // Local copies for printing

  for (;;) {
    wdt_reset(); // Reset watchdog

    // Safely get the current state and sensor readings.
    if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
      running = experimentRunning;
      // Copy global sensor values to local variables inside the mutex lock.
      freeLoad = currentFreeLoad;
      fixedLoad = currentFixedLoad;
      aux1 = currentAux1;
      aux2 = currentAux2;
      xSemaphoreGive(paramMutex);

      if (running) {
        // Print formatted status line using the local copies.
        Serial.print("Free: "); Serial.print(freeLoad, 2);
        Serial.print(" N | Fixed: "); Serial.print(fixedLoad, 2);
        Serial.print(" N | Aux1: "); Serial.print(aux1, 2);
        Serial.print(" | Aux2: "); Serial.println(aux2, 2);
      }
    }
    // Reporting frequency.
    vTaskDelay(pdMS_TO_TICKS(1000)); // Report status every 1 second.
  }
}

/**
 * @brief RTOS task for handling communication with the Nextion HMI display.
 */
void NextionTask(void *pvParameters) {
  nexInit(); // Initialize Nextion library communication.
  vTaskDelay(pdMS_TO_TICKS(500)); // Allow display to initialize.
  wdt_reset();

  Serial.println(F("NextionTask: Initializing display and callbacks..."));

  // Attach callback functions.
  btnStart.attachPush(btnStartCallback, &btnStart);
  btnStop.attachPush(btnStopCallback, &btnStop);
  sliderFree.attachPop(sliderFreeCallback, &sliderFree);
  sliderFixed.attachPop(sliderFixedCallback, &sliderFixed);
  wdt_reset();

  // Send initial values from the system to the display.
  if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
    updateNextionValues(true); // Update all values including sliders
    xSemaphoreGive(paramMutex);
  }
  wdt_reset();

  // Clear the waveform channel 0 initially.
  waveFormLoad.addValue(0, 0);

  Serial.println(F("NextionTask: Running event loop and updates..."));

  for (;;) {
    wdt_reset(); // Reset watchdog

    // Process incoming events from the Nextion display.
    nexLoop(nex_listen_list);

    // Periodically update the display with current data.
    if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
      // Update display elements using global sensor data
      updateNextionValues(false); // Don't force slider updates here
      xSemaphoreGive(paramMutex);
    }

    // Task delay - controls the update rate of the display.
    vTaskDelay(pdMS_TO_TICKS(100)); // Update display ~10 times per second.
  }
}

/** @brief Callback for Nextion 'Start' button press. */
void btnStartCallback(void *ptr) {
  Serial.println(F("Nextion Event: Start Button Pressed"));
  wdt_reset(); // Reset watchdog on interaction
  if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
    startExperiment();
    txtStatus.setText("Running"); // Update status immediately
    xSemaphoreGive(paramMutex);
  }
}

/** @brief Callback for Nextion 'Stop' button press. */
void btnStopCallback(void *ptr) {
  Serial.println(F("Nextion Event: Stop Button Pressed"));
  wdt_reset(); // Reset watchdog on interaction
  if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
    stopExperiment();
    txtStatus.setText("Ready"); // Update status immediately
    xSemaphoreGive(paramMutex);
  }
}

/** @brief Callback for Nextion 'Free Load' slider release. */
void sliderFreeCallback(void *ptr) {
  uint32_t value;
  NexSlider *slider = (NexSlider *)ptr;
  wdt_reset(); // Reset watchdog on interaction

  if (slider && slider->getValue(&value)) {
    float newLoad = MIN_FREE_LOAD + (float)(value - SLIDER_MIN_VAL) * (MAX_FREE_LOAD - MIN_FREE_LOAD) / (SLIDER_MAX_VAL - SLIDER_MIN_VAL);
    newLoad = constrain(newLoad, MIN_FREE_LOAD, MAX_FREE_LOAD);

    if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
      if (abs(targetLoadFree - newLoad) > 0.001) { // Check if value actually changed
          targetLoadFree = newLoad; // Update global variable

          // Update the associated text display on Nextion.
          char buffer[10];
          dtostrf(targetLoadFree, 4, 2, buffer);
          txtFreeVal.setText(buffer);

          Serial.print(F("Nextion Event: Free Load Slider set to: "));
          Serial.print(targetLoadFree, 2);
          Serial.println(F(" N"));
      }
      xSemaphoreGive(paramMutex);
    }
  } else {
     Serial.println(F("Error reading Free Load slider value"));
  }
}

/** @brief Callback for Nextion 'Fixed Load' slider release. */
void sliderFixedCallback(void *ptr) {
   uint32_t value;
   NexSlider *slider = (NexSlider *)ptr;
   wdt_reset(); // Reset watchdog on interaction

   if (slider && slider->getValue(&value)) {
    float newLoad = MIN_FIXED_LOAD + (float)(value - SLIDER_MIN_VAL) * (MAX_FIXED_LOAD - MIN_FIXED_LOAD) / (SLIDER_MAX_VAL - SLIDER_MIN_VAL);
    newLoad = constrain(newLoad, MIN_FIXED_LOAD, MAX_FIXED_LOAD);

    if (xSemaphoreTake(paramMutex, portMAX_DELAY) == pdTRUE) {
       if (abs(targetLoadFixed - newLoad) > 0.001) { // Check if value actually changed
          targetLoadFixed = newLoad; // Update global variable

          // Update the associated text display on Nextion.
          char buffer[10];
          dtostrf(targetLoadFixed, 4, 2, buffer);
          txtFixedVal.setText(buffer);

          Serial.print(F("Nextion Event: Fixed Load Slider set to: "));
          Serial.print(targetLoadFixed, 2);
          Serial.println(F(" N"));
       }
      xSemaphoreGive(paramMutex);
    }
  } else {
     Serial.println(F("Error reading Fixed Load slider value"));
  }
}

/**
 * @brief Computes the PID output. Call periodically.
 * Assumes pidSetpoint and pidInput are updated before calling.
 * Assumes mutex is NOT held when called (uses micros()).
 */
void computePID() {
    unsigned long now = micros();
    float timeChange = (float)(now - pidLastTime);

    // Avoid division by zero and ensure reasonable time delta
    if (timeChange <= 0) {
        pidLastTime = now; // Update time anyway
        return;
    }

    // Calculate error
    float error = pidSetpoint - pidInput;

    // Proportional term
    float pTerm = Kp * error; // Use global Kp

    // Integral term (with anti-windup)
    pidIntegral += error * timeChange / 1000000.0; // Scale timeChange to seconds
    pidIntegral = constrain(pidIntegral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    float iTerm = Ki * pidIntegral; // Use global Ki

    // Derivative term
    float dError = (error - pidLastError);
    float dTerm = Kd * (dError * 1000000.0 / timeChange); // Use global Kd

    // Calculate total PID output
    pidOutput = pTerm + iTerm + dTerm;

    // Apply output limits
    pidOutput = constrain(pidOutput, PID_OUT_MIN, PID_OUT_MAX);

    // Store values for next iteration
    pidLastError = error;
    pidLastTime = now;
}

/**
 * @brief Applies the PID output value to control Motor 5.
 * Maps the PID output range to motor speed and direction.
 * @param output The calculated PID output value.
 */
void applyPIDOutput(float output) {
    const int MIN_PID_MOTOR_SPEED = 5;  // Minimum speed motor will move reliably
    const int MAX_PID_MOTOR_SPEED = 30; // Maximum speed for adjustment

    if (abs(output) < 1.0) { // Deadband around zero output - TUNE ME!
        motorController.StopMotor(5);
    } else {
        uint8_t direction = (output > 0) ? HIGH : LOW; // Assuming HIGH increases load
        int speed = map(abs(output), 0, PID_OUT_MAX, MIN_PID_MOTOR_SPEED, MAX_PID_MOTOR_SPEED);
        speed = constrain(speed, MIN_PID_MOTOR_SPEED, MAX_PID_MOTOR_SPEED);
        motorController.RunMotor(5, speed, direction);
    }
}

/**
 * @brief Updates multiple value displays on the Nextion screen.
 * Reads required values from global variables under mutex protection.
 * @param updateSliders If true, forces update of slider positions based on global targets.
 */
void updateNextionValues(bool updateSliders) {
  // Assumes paramMutex is held.

  // --- Update Waveform ---
  uint8_t loadValueScaled = constrain(map(currentFreeLoad * 100, 0, MAX_FREE_LOAD * 100, 0, 255), 0, 255);
  waveFormLoad.addValue(0, loadValueScaled);

  // --- Update Text Displays ---
  char buffer[10];
  dtostrf(currentFreeLoad, 4, 2, buffer);
  txtFreeVal.setText(buffer);

  dtostrf(currentFixedLoad, 4, 2, buffer);
  txtFixedVal.setText(buffer);

  // --- Update Status Text ---
  txtStatus.setText(experimentRunning ? "Running" : "Ready");

  // --- Update Sliders (Optional) ---
  if (updateSliders) {
      // Map global target loads back to slider value (0-100)
      uint32_t freeSpherePos = SLIDER_MIN_VAL + (uint32_t)((targetLoadFree - MIN_FREE_LOAD) * (SLIDER_MAX_VAL - SLIDER_MIN_VAL) / (MAX_FREE_LOAD - MIN_FREE_LOAD));
      freeSpherePos = constrain(freeSpherePos, SLIDER_MIN_VAL, SLIDER_MAX_VAL);
      sliderFree.setValue(freeSpherePos);

      uint32_t fixedSpherePos = SLIDER_MIN_VAL + (uint32_t)((targetLoadFixed - MIN_FIXED_LOAD) * (SLIDER_MAX_VAL - SLIDER_MIN_VAL) / (MAX_FIXED_LOAD - MIN_FIXED_LOAD));
      fixedSpherePos = constrain(fixedSpherePos, SLIDER_MIN_VAL, SLIDER_MAX_VAL);
      sliderFixed.setValue(fixedSpherePos);
  }
}

/**
 * @brief Starts the tribology experiment. Reads targets from global variables.
 * Assumes mutex is taken before calling.
 */
void startExperiment() {
  // Assumes paramMutex is held.
  if (experimentRunning) {
    Serial.println(F("Experiment already running"));
    return;
  }

  Serial.println(F("Starting experiment with parameters:"));
  Serial.print(F("- Free sphere load target: ")); Serial.print(targetLoadFree, 2); Serial.println(F(" N")); // Use global
  Serial.print(F("- Fixed sphere load target: ")); Serial.print(targetLoadFixed, 2); Serial.println(F(" N")); // Use global
  Serial.print(F("- Rotation speed: ")); Serial.print(ROTATION_SPEED); Serial.println(F(" RPM"));
  Serial.print(F("- Flow speed: ")); Serial.print(FLOW_SPEED); Serial.println(F(" RPM"));

  // ... start motors ...
  motorController.RunMotor(1, FLOW_SPEED, HIGH);
  motorController.RunMotor(2, FLOW_SPEED, HIGH);
  motorController.RunMotor(3, ROTATION_SPEED, HIGH);
  motorController.RunMotor(4, ROTATION_SPEED, HIGH);

  // Reset PID state before starting control
  pidIntegral = 0;
  pidLastError = 0;
  pidLastTime = micros(); // Reset timer

  experimentRunning = true;
  Serial.println(F("Experiment started."));
}

/**
 * @brief Stops the tribology experiment.
 * Assumes mutex is taken before calling.
 */
void stopExperiment() {
  // Assumes paramMutex is held.
  if (!experimentRunning) {
    Serial.println(F("No experiment running to stop."));
    return;
  }
  Serial.println(F("Stopping experiment..."));

  // Stop all motors.
  for (int i = 1; i <= StepperMotorController::MAX_MOTORS; i++) {
    motorController.StopMotor(i);
  }

  experimentRunning = false;
  // Reset PID state
  pidIntegral = 0;
  pidLastError = 0;
  pidOutput = 0;

  Serial.println(F("Experiment stopped."));
}

/**
 * @brief Prints a detailed status report to the Serial Monitor.
 * Reads required values from global variables under mutex protection.
 * Assumes mutex is taken before calling.
 */
void printStatus() {
  // Assumes paramMutex is held.
  Serial.println(F("\n=== System Status ==="));
  Serial.print(F("Experiment running: ")); Serial.println(experimentRunning ? "Yes" : "No");

  Serial.print(F("Target free sphere load: ")); Serial.print(targetLoadFree, 2); Serial.println(F(" N")); // Use global
  Serial.print(F("Target fixed sphere load: ")); Serial.print(targetLoadFixed, 2); Serial.println(F(" N")); // Use global

  // Use global sensor readings
  Serial.print(F("Current free sphere load: ")); Serial.print(currentFreeLoad, 2); Serial.println(F(" N"));
  Serial.print(F("Current fixed sphere load: ")); Serial.print(currentFixedLoad, 2); Serial.println(F(" N"));
  Serial.print(F("Current Aux1: ")); Serial.print(currentAux1, 2); Serial.println();
  Serial.print(F("Current Aux2: ")); Serial.print(currentAux2, 2); Serial.println();

  Serial.println(F("== PID Parameters (Fixed) ==")); // Changed title
  Serial.print(F("- Kp: ")); Serial.println(Kp);
  Serial.print(F("- Ki: ")); Serial.println(Ki);
  Serial.print(F("- Kd: ")); Serial.println(Kd);

  Serial.println(F("== Fixed Parameters =="));
  Serial.print(F("- Rotation speed: ")); Serial.print(ROTATION_SPEED); Serial.println(F(" RPM"));
  Serial.print(F("- Flow speed: ")); Serial.print(FLOW_SPEED); Serial.println(F(" RPM"));
  Serial.println(F("======================="));
}
