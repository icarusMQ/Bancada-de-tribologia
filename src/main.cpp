#include <Arduino.h>
#include <HX711.h>
#include <AccelStepper.h>
#include <Arduino_FreeRTOS.h> // Include FreeRTOS for Arduino
#include <task.h>             // For task handles and utilities
#include <avr/wdt.h>          // AVR watchdog controls
#include <avr/pgmspace.h>     // For storing strings in flash memory

// --- Stack Overflow Hook ---
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  Serial.print(F("❌ Stack Overflow in "));
  Serial.println(pcTaskName);
  taskDISABLE_INTERRUPTS();
  for(;;);
}

// Store strings in flash memory instead of RAM
const char str_init[] PROGMEM = "--- FreeRTOS Tribology Experiment Controller ---";
const char str_init_motors[] PROGMEM = "Initializing Motors...";
const char str_motors_init[] PROGMEM = "Motors Initialized.";
const char str_init_sensors[] PROGMEM = "Initializing Sensors...";
const char str_sensors_init[] PROGMEM = "Sensors initialized.";
const char str_taring[] PROGMEM = "Taring ";
const char str_remove_load[] PROGMEM = "... Remove any load.";
const char str_tared[] PROGMEM = " tared successfully.";
const char str_fail_tare[] PROGMEM = " FAILED to become ready (timeout). Skipping.";

// --- Pin Definitions ---
const uint8_t MOTOR1_STEP_PIN = 9;  // MPFix Step
const uint8_t MOTOR1_DIR_PIN = 8;   // MPFix Dir
const uint8_t MOTOR2_STEP_PIN = 5;  // MPFre Step
const uint8_t MOTOR2_DIR_PIN = 8;   // MPFre Dir (Shared with Motor 1? Verify hardware)
const uint8_t MOTOR3_STEP_PIN = A1; // MA Step
const uint8_t MOTOR3_DIR_PIN = A0;  // MA Dir
const uint8_t MOTOR4_STEP_PIN = A3; // MAFre Step
const uint8_t MOTOR4_DIR_PIN = A2;  // MAFre Dir
const uint8_t MOTOR5_STEP_PIN = A5; // MAFix Step
const uint8_t MOTOR5_DIR_PIN = A4;  // MAFix Dir

// Sensors (Scales)
const uint8_t SCALE1_DT_PIN = 2;  // SFix DT
const uint8_t SCALE1_SCK_PIN = 3; // SFix SCK
const uint8_t SCALE2_DT_PIN = 4;  // SFiz DT
const uint8_t SCALE2_SCK_PIN = 6; // SFiz SCK
const uint8_t SCALE3_DT_PIN = 10; // SFrx DT
const uint8_t SCALE3_SCK_PIN = 7; // SFrx SCK
const uint8_t SCALE4_DT_PIN = 12; // SFrz DT
const uint8_t SCALE4_SCK_PIN = 11; // SFrz SCK

// --- Motor & Sensor Mapping Constants ---
const int MPFix = 1;
const int MPFre = 2;
const int MA = 3;
const int MAFre = 4;
const int MAFix = 5;

const int SFix = 1;
const int SFiz = 2;
const int SFrx = 3;
const int SFrz = 4;

// --- Experiment Parameters ---
const float TARGET_FORCE_FREE_SPHERE = 0.3; // Target force for free sphere (N)
const float PUMP_RPM = 0.5;                 // Pump speed (RPM)
const int ROTATION_RPM = 80;                // Rotation speed for axes (RPM)
const int MA_RPM = 60;                      // RPM for manual control
const float FORCE_TOLERANCE = 0.03;         // Tolerance band around the target force (N)
const float MIN_ADJUST_RPM = 1.0;           // Minimum speed for adjustment motor
const float MAX_ADJUST_RPM = MA_RPM;        // Maximum speed for adjustment motor
const float FORCE_PROPORTIONAL_GAIN = 80;   // Proportional gain for force adjustment
const unsigned long ADJUST_PRINT_INTERVAL = 500; // Print adjustment status every 500ms
const int ADJUSTMENT_READING_COUNT = 10;    // Number of readings to average for adjustment control
const float HYSTERESIS_MARGIN = 0.02;       // N - Only change direction if error > TOLERANCE + MARGIN
const TickType_t MIN_DWELL_TIME = pdMS_TO_TICKS(500); // Minimum time between direction changes

// --- Stepper Motor Configuration ---
const int MOTOR1_STEPS_PER_REV = 200;
const int MOTOR2_STEPS_PER_REV = 200;
const int MOTOR3_STEPS_PER_REV = 3200; // MA (Force Adjustment)
const int MOTOR4_STEPS_PER_REV = 1600;
const int MOTOR5_STEPS_PER_REV = 1600;
const int DEFAULT_MAX_SPEED = 1000;    // steps per second
const int DEFAULT_ACCELERATION = 500; // steps per second squared

// --- Sensor Configuration ---
float calibration_factor1 = 0.02808;  // SFix
float calibration_factor2 = 0.0342;   // SFiz
float calibration_factor3 = -0.05671; // SFrx
float calibration_factor4 = -0.0614;  // SFrz
const int NUM_SAMPLES = 8;            // Samples for median reading
const float smoothingFactor = 0.6;    // Smoothing factor for readings
const unsigned long TARE_TIMEOUT = 5000UL; // Timeout for taring scales (ms)
const float GRAMS_TO_NEWTONS = 0.00980665; // Conversion factor

// --- Global Objects ---
HX711 scale1, scale2, scale3, scale4;
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper motor3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN); // MA (Force Adjustment)
AccelStepper motor4(AccelStepper::DRIVER, MOTOR4_STEP_PIN, MOTOR4_DIR_PIN);
AccelStepper motor5(AccelStepper::DRIVER, MOTOR5_STEP_PIN, MOTOR5_DIR_PIN);

// --- Global State Variables ---
volatile bool experimentRunning = false; // Use volatile for shared flags
volatile bool forceAdjusted = false;
volatile bool pumpsStarted = false;
volatile bool rotationStarted = false;
volatile bool adjustingForce = false; // Flag to indicate force adjustment motor (MA) is active
enum ForceAdjustmentState { IDLE, ADJUSTING, CONFIRMING, ADJUSTED };
volatile ForceAdjustmentState forceState = IDLE; // Use volatile for shared state enum
int forceInRangeCounter = 0; // Counter for consecutive in-range readings (primarily used by control task)
const int FORCE_CONFIRMATION_COUNT = 50; // Required consecutive readings in range

// Sensor State (primarily used by sensor task, read by control task)
bool active1 = false, active2 = false, active3 = false, active4 = false;
float lastStable1 = 0, lastStable2 = 0, lastStable3 = 0, lastStable4 = 0;

// --- Forward declarations for Tasks ---
void vMotorTask(void* pvParameters);
void vSensorTask(void* pvParameters);
void vControlTask(void* pvParameters);
void vSerialTask(void* pvParameters);

// --- Forward declarations for Helper Functions ---
void initializeMotors();
void initializeScales();
float computeMedian(float arr[], int count);
float getMedianReadingGrams(HX711 &scale, float cal, int numSamples, bool isActive);
float readAverageSensor(uint8_t sensorID, int numReadings);
float readSensor(uint8_t sensorID); // Reads smoothed value
bool tareScale(HX711 &scale, const char* scaleName);
float rpmToStepsPerSecond(float rpm, int stepsPerRev);
void runMotor(uint8_t motorID, float rpm);
void stopMotor(uint8_t motorID);
void stopAllMotors();
void handleSerialInput(); // Now called by vSerialTask
void runExperimentLogic(); // Now called by vControlTask
void updateMotors(); // Now called by vMotorTask
void printProgString(const char* str); // Helper function to print strings from PROGMEM

// --- Setup ---
void setup() {
  wdt_disable();       // turn off the bootloader watchdog
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection
  printProgString(str_init);
  Serial.println();

  // Initialize hardware
  initializeMotors();
  initializeScales();

  Serial.println(F("Hardware Initialized. Creating Tasks..."));

  // Create FreeRTOS tasks with reduced stack sizes
  xTaskCreate(vMotorTask,    "MotorTask",   96,  NULL, tskIDLE_PRIORITY + 3, NULL);  // Reduced from 128
  xTaskCreate(vSensorTask,   "SensorTask",  192, NULL, tskIDLE_PRIORITY + 2, NULL);  // Reduced from 256
  xTaskCreate(vSerialTask,   "SerialTask",  160, NULL, tskIDLE_PRIORITY + 1, NULL);  // Reduced from 192
  xTaskCreate(vControlTask,  "ControlTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL);  // Reduced from 384

  Serial.println(F("Tasks Created. Starting Scheduler."));
  // Start the scheduler - setup() will not return from here
  vTaskStartScheduler();

  // Should never get here
  Serial.println(F("Error: Scheduler failed to start."));
  while(1);
}

// --- loop() ---
void loop() {
  // Empty: FreeRTOS scheduler handles everything now
}

// --- Task Implementations ---
void vMotorTask(void* pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1);

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    updateMotors();
  }
}

void vSensorTask(void* pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (active1) readSensor(SFix);
    if (active2) readSensor(SFiz);
    if (active3) readSensor(SFrx);
    if (active4) readSensor(SFrz);
  }
}

void vControlTask(void* pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    runExperimentLogic();
  }
}

void vSerialTask(void* pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    handleSerialInput();
  }
}

// --- Helper Function Implementations ---
void initializeMotors() {
  printProgString(str_init_motors);
  Serial.println();
  motor1.setMaxSpeed(DEFAULT_MAX_SPEED); motor1.setAcceleration(DEFAULT_ACCELERATION);
  motor2.setMaxSpeed(DEFAULT_MAX_SPEED); motor2.setAcceleration(DEFAULT_ACCELERATION);
  motor3.setMaxSpeed(DEFAULT_MAX_SPEED); motor3.setAcceleration(DEFAULT_ACCELERATION);
  motor4.setMaxSpeed(DEFAULT_MAX_SPEED); motor4.setAcceleration(DEFAULT_ACCELERATION);
  motor5.setMaxSpeed(DEFAULT_MAX_SPEED); motor5.setAcceleration(DEFAULT_ACCELERATION);
  printProgString(str_motors_init);
  Serial.println();
}

void initializeScales() {
  printProgString(str_init_sensors);
  Serial.println();
  scale1.begin(SCALE1_DT_PIN, SCALE1_SCK_PIN);
  scale2.begin(SCALE2_DT_PIN, SCALE2_SCK_PIN);
  scale3.begin(SCALE3_DT_PIN, SCALE3_SCK_PIN);
  scale4.begin(SCALE4_DT_PIN, SCALE4_SCK_PIN);

  active1 = tareScale(scale1, "Scale 1 (SFix)");
  active2 = tareScale(scale2, "Scale 2 (SFiz)");
  active3 = tareScale(scale3, "Scale 3 (SFrx)");
  active4 = tareScale(scale4, "Scale 4 (SFrz)");

  if (!active1 && !active2 && !active3 && !active4) {
    Serial.println(F("WARNING: No sensors responded. Check connections."));
  } else {
    printProgString(str_sensors_init);
    Serial.println();
  }
}

// Computes the median of an array of floats using insertion sort.
// Only sorts the elements that we have, up to count.
float computeMedian(float arr[], int count) {
  // Perform an insertion sort only on the valid samples
  for (int i = 1; i < count; i++) {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
  // Return the median
  if (count % 2 == 1) return arr[count / 2];
  else return (arr[(count - 1) / 2] + arr[count / 2]) / 2.0;
}

// Reads a batch from a given scale and return the median weight in grams.
// Optimized to use a smaller array with just enough space for valid samples
float getMedianReadingGrams(HX711 &scale, float cal, int numSamples, bool isActive) {
  if (!isActive) return 0;
  
  // Use a reduced size array - we'll only store valid readings
  // Most of the time we'll get 3-5 valid readings out of 8 attempts
  float validSamples[5]; // Reduced from NUM_SAMPLES (8) to save RAM
  int count = 0;
  
  for (int i = 0; i < numSamples && count < 5; i++) {
    if (scale.is_ready()) {
      long raw = scale.read();
      float weight = (raw - scale.get_offset()) * cal;
      // Only add to array if we have space
      if (count < 5) {
        validSamples[count++] = weight;
      }
    }
  }
  
  if (count == 0) return 0;
  else return computeMedian(validSamples, count);
}

// Reads an average weight from a sensor over multiple samples in Newtons.
float readAverageSensor(uint8_t sensorID, int numReadings) {
  float totalWeightGrams = 0;
  int validReadings = 0;
  float calFactor = 0;
  HX711* scalePtr = nullptr;
  bool isActive = false;
  long offset = 0;

  switch(sensorID) {
    case SFix: isActive = active1; calFactor = calibration_factor1; scalePtr = &scale1; if(isActive) offset = scale1.get_offset(); break;
    case SFiz: isActive = active2; calFactor = calibration_factor2; scalePtr = &scale2; if(isActive) offset = scale2.get_offset(); break;
    case SFrx: isActive = active3; calFactor = calibration_factor3; scalePtr = &scale3; if(isActive) offset = scale3.get_offset(); break;
    case SFrz: isActive = active4; calFactor = calibration_factor4; scalePtr = &scale4; if(isActive) offset = scale4.get_offset(); break;
    default: Serial.println(F("Invalid sensor ID for average reading.")); return 0;
  }

  if (!isActive || !scalePtr) return 0;

  for (int i = 0; i < numReadings; ++i) {
      unsigned long startWait = millis();
      while(!scalePtr->is_ready()) {
        if (millis() - startWait > 50) break;
        taskYIELD();  // Properly yield to other tasks while waiting
      }
      if (scalePtr->is_ready()) {
          long raw = scalePtr->read();
          float weight = (raw - offset) * calFactor;
          totalWeightGrams += weight;
          validReadings++;
      }
  }

  if (validReadings > 0) {
      float averageGrams = totalWeightGrams / validReadings;
      return averageGrams * GRAMS_TO_NEWTONS;
  } else {
      return 0;
  }
}

// Reads the filtered weight value from a specific sensor in Newtons.
float readSensor(uint8_t sensorID) {
  float currentGrams = 0;
  float* lastStablePtr = nullptr;
  bool isActive = false;
  float calFactor = 0;
  HX711* scalePtr = nullptr;

  switch(sensorID) {
    case SFix: lastStablePtr = &lastStable1; isActive = active1; calFactor = calibration_factor1; scalePtr = &scale1; break;
    case SFiz: lastStablePtr = &lastStable2; isActive = active2; calFactor = calibration_factor2; scalePtr = &scale2; break;
    case SFrx: lastStablePtr = &lastStable3; isActive = active3; calFactor = calibration_factor3; scalePtr = &scale3; break;
    case SFrz: lastStablePtr = &lastStable4; isActive = active4; calFactor = calibration_factor4; scalePtr = &scale4; break;
    default: Serial.println(F("Invalid sensor ID.")); return 0;
  }

  if (isActive && scalePtr) {
    currentGrams = getMedianReadingGrams(*scalePtr, calFactor, NUM_SAMPLES, isActive);
    *lastStablePtr = (1.0 - smoothingFactor) * (*lastStablePtr) + smoothingFactor * currentGrams;
    return (*lastStablePtr) * GRAMS_TO_NEWTONS;
  } else {
    if (lastStablePtr) *lastStablePtr = 0;
    return 0;
  }
}

// Blocks for 5 s, then waits up to TARE_TIMEOUT for the HX711 to be ready.
// All of this runs before the scheduler ever starts.
bool tareScale(HX711 &scale, const char* scaleName) {
  printProgString(str_taring);
  Serial.print(scaleName); 
  printProgString(str_remove_load);
  Serial.println();
  
  // give the user 5 s to remove any weight
  delay(5000);

  // now wait up to TARE_TIMEOUT for the scale to get ready
  unsigned long start = millis();
  while (!scale.is_ready() && (millis() - start < TARE_TIMEOUT)) {
    delay(10);   // small blocking delay is OK here, pre-scheduler
  }

  if (!scale.is_ready()) {
    Serial.print(scaleName);
    printProgString(str_fail_tare);
    Serial.println();
    return false;
  }

  // finally tare
  scale.tare();
  Serial.print(scaleName);
  printProgString(str_tared);
  Serial.println();
  return true;
}

// Converts RPM to steps per second.
float rpmToStepsPerSecond(float rpm, int stepsPerRev) {
    if (stepsPerRev <= 0) return 0;
    return (float)stepsPerRev * rpm / 60.0;
}

// Sets the target speed for the specified motor.
void runMotor(uint8_t motorID, float rpm) {
  float speed = 0;
  int steps = 0;
  AccelStepper* motorPtr = nullptr;

  switch (motorID) {
    case MPFix: steps = MOTOR1_STEPS_PER_REV; motorPtr = &motor1; break;
    case MPFre: steps = MOTOR2_STEPS_PER_REV; motorPtr = &motor2; break;
    case MA:    steps = MOTOR3_STEPS_PER_REV; motorPtr = &motor3; break;
    case MAFre: steps = MOTOR4_STEPS_PER_REV; motorPtr = &motor4; break;
    case MAFix: steps = MOTOR5_STEPS_PER_REV; motorPtr = &motor5; break;
    default: Serial.println(F("Invalid motor ID for run.")); return;
  }

  if (motorPtr) {
    speed = rpmToStepsPerSecond(rpm, steps);
    motorPtr->setSpeed(speed);
  }
}

// Stops the specified motor by setting its speed to 0.
void stopMotor(uint8_t motorID) {
  AccelStepper* motorPtr = nullptr;
  switch (motorID) {
    case MPFix: motorPtr = &motor1; break;
    case MPFre: motorPtr = &motor2; break;
    case MA:    motorPtr = &motor3; break;
    case MAFre: motorPtr = &motor4; break;
    case MAFix: motorPtr = &motor5; break;
    default: Serial.println(F("Invalid motor ID for stop.")); return;
  }
  if (motorPtr) {
    motorPtr->setSpeed(0);
  }
}

// Stops all motors.
void stopAllMotors() {
    stopMotor(MPFix);
    stopMotor(MPFre);
    stopMotor(MA);
    stopMotor(MAFre);
    stopMotor(MAFix);
}

// Updates motor positions/speeds (called by vMotorTask)
void updateMotors() {
  motor1.runSpeed();
  motor2.runSpeed();
  motor3.runSpeed();
  motor4.runSpeed();
  motor5.runSpeed();
}

// Handles serial input (called by vSerialTask)
void handleSerialInput() {
  if (Serial.available()) {
    // Read into a fixed buffer instead
    char cmdBuf[16];
    size_t idx = 0;
    while (Serial.available() && idx < sizeof(cmdBuf)-1) {
      char c = Serial.read();
      if (c=='\n') break;
      cmdBuf[idx++] = c;
    }
    cmdBuf[idx] = '\0';
    
    if (!adjustingForce) stopAllMotors();
    else { stopMotor(MPFix); stopMotor(MPFre); stopMotor(MAFre); stopMotor(MAFix); }

    if (strcmp(cmdBuf, "start") == 0) {
      Serial.println(F("CMD: Start Experiment"));
      experimentRunning = true;
      forceAdjusted = false;
      pumpsStarted = false;
      rotationStarted = false;
      adjustingForce = false;
      forceState = IDLE;
      forceInRangeCounter = 0;
      stopAllMotors();
    } else if (strcmp(cmdBuf, "stop") == 0) {
      Serial.println(F("CMD: Stop Experiment"));
      experimentRunning = false;
      adjustingForce = false;
      forceState = IDLE;
      forceInRangeCounter = 0;
      stopAllMotors();
    } else if (strcmp(cmdBuf, "print") == 0) {
      Serial.println(F("CMD: Print Scale Readings"));
      float v1 = lastStable1 * GRAMS_TO_NEWTONS;
      float v2 = lastStable2 * GRAMS_TO_NEWTONS;
      float v3 = lastStable3 * GRAMS_TO_NEWTONS;
      float v4 = lastStable4 * GRAMS_TO_NEWTONS;
      Serial.print(F("SFix: ")); Serial.print(v1, 3); Serial.print(F(" N, "));
      Serial.print(F("SFiz: ")); Serial.print(v2, 3); Serial.print(F(" N, "));
      Serial.print(F("SFrx: ")); Serial.print(v3, 3); Serial.print(F(" N, "));
      Serial.print(F("SFrz: ")); Serial.println(v4, 3); Serial.print(F(" N"));
    } else if (idx == 1 && !experimentRunning && !adjustingForce) {
      char cmdChar = cmdBuf[0];
      Serial.print(F("Manual CMD: ")); Serial.println(cmdChar);
      switch (cmdChar) {
        case 'q': runMotor(MPFix, MA_RPM); break;
        case 'a': runMotor(MPFix, -MA_RPM); break;
        case 'w': runMotor(MPFre, MA_RPM); break;
        case 's': runMotor(MPFre, -MA_RPM); break;
        case 'e': runMotor(MA, MA_RPM); break;
        case 'd': runMotor(MA, -MA_RPM); break;
        case 'r': runMotor(MAFre, MA_RPM); break;
        case 'f': runMotor(MAFre, -MA_RPM); break;
        case 't': runMotor(MAFix, MA_RPM); break;
        case 'g': runMotor(MAFix, -MA_RPM); break;
        case 'h': Serial.println(F("Manual: All Motors Forward")); runMotor(MPFix, MA_RPM); runMotor(MPFre, MA_RPM); runMotor(MA, MA_RPM); runMotor(MAFre, MA_RPM); runMotor(MAFix, MA_RPM); break;
        case 'n': Serial.println(F("Manual: All Motors Backward")); runMotor(MPFix, -MA_RPM); runMotor(MPFre, -MA_RPM); runMotor(MA, -MA_RPM); runMotor(MAFre, -MA_RPM); runMotor(MAFix, -MA_RPM); break;
        default: Serial.println(F("Unknown manual command.")); break;
      }
    } else if (idx > 0) {
        Serial.print(F("Unknown command or manual control disabled: "));
        Serial.println(cmdBuf);
    }
  }
}

// Runs the experiment logic state machine (called by vControlTask)
void runExperimentLogic() {
  // Consolidate static timekeeping variables to save RAM
  static struct {
    TickType_t directionChange;
    unsigned long printTime;
    TickType_t logTime;
  } timers = {0, 0, 0};
  
  static int lastAdjustmentDirectionCmd = 0;
  const TickType_t DATA_LOGGING_INTERVAL_TICKS = pdMS_TO_TICKS(100);

  if (!experimentRunning) {
     if (adjustingForce) {
        stopMotor(MA);
        adjustingForce = false;
        forceState = IDLE;
        forceInRangeCounter = 0;
        lastAdjustmentDirectionCmd = 0;
     }
    return;
  }

  unsigned long currentMillis = millis();

  if (!forceAdjusted) {
    float currentForce;
    if (forceState == ADJUSTING) {
        currentForce = readAverageSensor(SFrx, ADJUSTMENT_READING_COUNT);
        if (currentForce == 0 && active3) {
             currentForce = readSensor(SFrx);
        }
    } else {
        currentForce = readSensor(SFrx);
    }

    float error = TARGET_FORCE_FREE_SPHERE - currentForce;

    if (abs(error) <= FORCE_TOLERANCE) {
      if (forceState == ADJUSTING || forceState == IDLE) {
          stopMotor(MA);
          adjustingForce = false;
          forceState = CONFIRMING;
          forceInRangeCounter = 1;
          lastAdjustmentDirectionCmd = 0;
          Serial.print(F("Force in range (")); Serial.print(forceInRangeCounter); Serial.print(F("/"));
          Serial.print(FORCE_CONFIRMATION_COUNT); Serial.print(F("). Current: "));
          Serial.print(currentForce, 3); Serial.println(F(" N. Confirming..."));
      } else if (forceState == CONFIRMING) {
          forceInRangeCounter++;
          if (forceInRangeCounter % 10 == 0 || forceInRangeCounter == FORCE_CONFIRMATION_COUNT) {
              Serial.print(F("Force still in range (")); Serial.print(forceInRangeCounter); Serial.print(F("/"));
              Serial.print(FORCE_CONFIRMATION_COUNT); Serial.print(F("). Current: "));
              Serial.print(currentForce, 3); Serial.println(F(" N."));
          }
          if (forceInRangeCounter >= FORCE_CONFIRMATION_COUNT) {
              forceState = ADJUSTED;
              forceAdjusted = true;
              adjustingForce = false;
              forceInRangeCounter = 0;
              lastAdjustmentDirectionCmd = 0;
              Serial.println(F("Force adjustment confirmed and complete."));
              Serial.print(F("Final Force: ")); Serial.print(currentForce, 3); Serial.println(F(" N"));
          }
      }
    } else {
      forceInRangeCounter = 0;
      forceState = ADJUSTING;
      adjustingForce = true;

      int desiredDirection = (error < 0) ? 1 : -1;

      bool dwellTimePassed = true;
      if (desiredDirection != lastAdjustmentDirectionCmd) {
          if (xTaskGetTickCount() - timers.directionChange < MIN_DWELL_TIME) {
              dwellTimePassed = false;
          }
      }

      bool outsideHysteresis = abs(error) > (FORCE_TOLERANCE + HYSTERESIS_MARGIN);

      if (dwellTimePassed && outsideHysteresis) {
          float dynamicGain = FORCE_PROPORTIONAL_GAIN * (abs(error) / TARGET_FORCE_FREE_SPHERE);
          dynamicGain = max(dynamicGain, 1.0);
          float targetSpeedRPM = MIN_ADJUST_RPM + dynamicGain * abs(error);
          targetSpeedRPM = constrain(targetSpeedRPM, MIN_ADJUST_RPM, MAX_ADJUST_RPM);
          float motorSpeed = (desiredDirection == 1) ? targetSpeedRPM : -targetSpeedRPM;

          runMotor(MA, motorSpeed);

          if (desiredDirection != lastAdjustmentDirectionCmd) {
              timers.directionChange = xTaskGetTickCount();
              lastAdjustmentDirectionCmd = desiredDirection;
              Serial.print(desiredDirection == 1 ? F("Adjusting UP -> ") : F("Adjusting DOWN -> "));
              Serial.print(F("Target: ")); Serial.print(TARGET_FORCE_FREE_SPHERE, 3);
              Serial.print(F(" N, Avg Current: ")); Serial.print(currentForce, 3);
              Serial.print(F(" N, Speed: ")); Serial.println(abs(motorSpeed), 1);
          }

          if (currentMillis - timers.printTime >= ADJUST_PRINT_INTERVAL) {
              timers.printTime = currentMillis;
              Serial.print(F("Adjusting... Avg Force: ")); Serial.print(currentForce, 3); Serial.println(F(" N"));
          }

      } else {
          if (motor3.speed() != 0) {
              stopMotor(MA);
              Serial.println(F("Adjustment paused (hysteresis/dwell)"));
          }
          if (currentMillis - timers.printTime >= ADJUST_PRINT_INTERVAL) {
              timers.printTime = currentMillis;
              Serial.print(F("Adjusting paused... Avg Force: ")); Serial.print(currentForce, 3); Serial.println(F(" N"));
          }
      }
    }
  } else if (!pumpsStarted) {
    Serial.println(F("Starting Pumps..."));
    runMotor(MPFix, PUMP_RPM);
    runMotor(MPFre, PUMP_RPM);
    pumpsStarted = true;
    Serial.println(F("Pumps started."));
  } else if (!rotationStarted) {
    Serial.println(F("Starting Rotation..."));
    runMotor(MA, ROTATION_RPM);
    runMotor(MAFre, ROTATION_RPM);
    rotationStarted = true;
    Serial.println(F("Rotation started. Experiment running."));
    Serial.println(F("Send \"stop\" to halt."));
  } else {
      if (xTaskGetTickCount() - timers.logTime >= DATA_LOGGING_INTERVAL_TICKS) {
          timers.logTime = xTaskGetTickCount();

          float v1 = lastStable1 * GRAMS_TO_NEWTONS;
          float v2 = lastStable2 * GRAMS_TO_NEWTONS;
          float v3 = lastStable3 * GRAMS_TO_NEWTONS;
          float v4 = lastStable4 * GRAMS_TO_NEWTONS;

          Serial.print(F(">SFix:")); Serial.print(v1, 3);
          Serial.print(F(",SFiz:")); Serial.print(v2, 3);
          Serial.print(F(",SFrx:")); Serial.print(v3, 3);
          Serial.print(F(",SFrz:")); Serial.println(v4, 3);
      }
  }
}

// Helper function to print strings from PROGMEM (for AVR compatibility)
void printProgString(const char* str) {
  char c;
  while ((c = pgm_read_byte(str++))) {
    Serial.print(c);
  }
}