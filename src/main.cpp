#include "MultiScaleSensors.h"
#include "StepperMotorController.h"

// Create instances of our optimized classes
MultiScaleSensors sensors;
StepperMotorController motors;

// Program constants stored in flash memory (PROGMEM)
const char WELCOME_MSG[] PROGMEM = "Tribology Experiment Control";
const char READY_MSG[] PROGMEM = "System ready. Type 'help' for commands.";
const char CMD_PROMPT[] PROGMEM = "> ";
const char HELP_CMD[] PROGMEM = "help";
const char START_CMD[] PROGMEM = "start";
const char PAUSE_CMD[] PROGMEM = "pause";
const char STOP_CMD[] PROGMEM = "stop";
const char RESTART_CMD[] PROGMEM = "restart";
const char STATUS_CMD[] PROGMEM = "status";
const char MOTOR_CMD[] PROGMEM = "motor";
const char STOPMOTOR_CMD[] PROGMEM = "stopmotor";
const char SENSORS_CMD[] PROGMEM = "sensors";
const char MONITOR_CMD[] PROGMEM = "monitor";
const char TARE_CMD[] PROGMEM = "tare";

// Function to print a string from program memory
void printPgmString(const char* str) {
  char c;
  while ((c = pgm_read_byte(str++))) {
    Serial.write(c);
  }
}

// Experiment parameters
const float TARGET_FORCE = 0.3;     // Target force in Newtons
const float FORCE_MARGIN = 0.2;     // 20% error margin
const float FORCE_MIN = TARGET_FORCE * (1.0 - FORCE_MARGIN);
const float FORCE_MAX = TARGET_FORCE * (1.0 + FORCE_MARGIN);

// Motor speed constants
const uint8_t PUMP_RPM = 10;
const uint8_t ROTATION_RPM = 80;
const uint8_t FORCE_INC_RPM = 20;
const uint8_t FORCE_DEC_RPM = 8;
const uint8_t DEFAULT_RPM = 30;

// Timing constants
const unsigned long SERIAL_TIMEOUT = 100;
const unsigned long SENSOR_READ_INTERVAL = 500;
const unsigned long STATUS_INTERVAL = 2000;

// Command buffer (small and efficient)
char cmdBuffer[16];
uint8_t cmdIndex = 0;

// State machine states
enum SystemState : uint8_t {
  STATE_IDLE,
  STATE_ADJUSTING_FORCE,
  STATE_RUNNING_PUMPS,
  STATE_RUNNING_FULL,
  STATE_PAUSED
};

// Force adjustment states
enum ForceState : uint8_t {
  FORCE_INCREASE,
  FORCE_DECREASE,
  FORCE_OK
};

// System flags (all packed in a single byte)
uint8_t systemFlags = 0;
#define FLAG_FORCE_ADJUSTED  0x01
#define FLAG_PUMPS_RUNNING   0x02
#define FLAG_ROTATION_RUNNING 0x04
#define FLAG_MONITOR_ACTIVE  0x08
#define FLAG_EMERGENCY_STOP  0x10
#define FLAG_NEED_HELP       0x20
#define FLAG_NEED_STATUS     0x40
#define FLAG_NEED_PROMPT     0x80

// Current system state
SystemState currentState = STATE_IDLE;
ForceState forceState = FORCE_INCREASE;

// Timing variables
unsigned long lastSensorTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastSerialCheck = 0;

// Helper functions to get/set flags
inline bool getFlag(uint8_t flag) { return (systemFlags & flag) != 0; }
inline void setFlag(uint8_t flag, bool value) {
  if (value) systemFlags |= flag;
  else systemFlags &= ~flag;
}

// Helper macros for common flag operations
#define isForceAdjusted() getFlag(FLAG_FORCE_ADJUSTED)
#define arePumpsRunning() getFlag(FLAG_PUMPS_RUNNING)
#define isRotationRunning() getFlag(FLAG_ROTATION_RUNNING)
#define isMonitorActive() getFlag(FLAG_MONITOR_ACTIVE)
#define isEmergencyStop() getFlag(FLAG_EMERGENCY_STOP)
#define needsHelp() getFlag(FLAG_NEED_HELP)
#define needsStatus() getFlag(FLAG_NEED_STATUS)
#define needsPrompt() getFlag(FLAG_NEED_PROMPT)

#define setForceAdjusted(x) setFlag(FLAG_FORCE_ADJUSTED, x)
#define setPumpsRunning(x) setFlag(FLAG_PUMPS_RUNNING, x)
#define setRotationRunning(x) setFlag(FLAG_ROTATION_RUNNING, x)
#define setMonitorActive(x) setFlag(FLAG_MONITOR_ACTIVE, x)
#define setEmergencyStop(x) setFlag(FLAG_EMERGENCY_STOP, x)
#define setNeedsHelp(x) setFlag(FLAG_NEED_HELP, x)
#define setNeedsStatus(x) setFlag(FLAG_NEED_STATUS, x)
#define setNeedsPrompt(x) setFlag(FLAG_NEED_PROMPT, x)

// Function prototypes
void processCommand();
void handleSerialInput();
void updateStateMachine();
void displayHelp();
void displayStatus();
void displayPrompt();
void startExperiment();
void pauseExperiment();
void stopExperiment();
void restartExperiment();

void setup() {
  // Initialize serial at high baud rate
  Serial.begin(115200);
  
  // Wait for serial to initialize (but not too long)
  uint8_t timeout = 20;  // 100ms timeout
  while (!Serial && timeout > 0) {
    delay(5);
    timeout--;
  }
  
  // Print welcome message
  Serial.println();
  printPgmString(WELCOME_MSG);
  Serial.println();
  
  // Initialize sensors
  Serial.println(F("Initializing sensors..."));
  if (sensors.beginSensors()) {  // Updated to use the new method name
    Serial.println(F("Sensors initialized successfully."));
  } else {
    Serial.println(F("WARNING: Sensor initialization failed or no sensors detected!"));
  }
  
  // Tare sensors
  Serial.println(F("Taring sensors (remove all weight)..."));
  sensors.tareAllSensors();  // Updated to use the new method name
  Serial.println(F("Tare complete."));
  
  // Initialize motors
  Serial.println(F("Initializing motors..."));
  motors.init();
  motors.stopAllMotors();
  Serial.println(F("Motors initialized."));
  
  // System is ready
  printPgmString(READY_MSG);
  Serial.println();
  displayPrompt();
}

void loop() {
  // Non-blocking check for serial input
  if (millis() - lastSerialCheck >= SERIAL_TIMEOUT) {
    handleSerialInput();
    lastSerialCheck = millis();
  }
  
  // Update motor states (non-blocking)
  motors.update();
  
  // Update system state machine
  updateStateMachine();
  
  // Display sensor readings if monitoring is active
  if (isMonitorActive() && millis() - lastSensorTime >= SENSOR_READ_INTERVAL) {
    Serial.print(F("LC1:"));
    Serial.print(sensors.readSensor(1), 3);
    Serial.print(F("N LC2:"));
    Serial.print(sensors.readSensor(2), 3);
    Serial.print(F("N LC3:"));
    Serial.print(sensors.readSensor(3), 3);
    Serial.print(F("N LC4:"));
    Serial.print(sensors.readSensor(4), 3);
    Serial.println(F("N"));
    
    lastSensorTime = millis();
  }
  
  // Display periodic status if needed
  if (currentState != STATE_IDLE && millis() - lastStatusTime >= STATUS_INTERVAL) {
    displayStatus();
    lastStatusTime = millis();
  }
  
  // Print help if requested
  if (needsHelp()) {
    displayHelp();
    setNeedsHelp(false);
    setNeedsPrompt(true);
  }
  
  // Print status if requested
  if (needsStatus()) {
    displayStatus();
    setNeedsStatus(false);
    setNeedsPrompt(true);
  }
  
  // Print prompt if needed
  if (needsPrompt()) {
    displayPrompt();
    setNeedsPrompt(false);
  }
}

// Handle serial input (non-blocking)
void handleSerialInput() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Handle end of command (CR or LF)
    if (inChar == '\r' || inChar == '\n') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        Serial.println(); // Echo newline
        processCommand();
        cmdIndex = 0;
      }
    }
    // Handle backspace
    else if (inChar == 8 || inChar == 127) {
      if (cmdIndex > 0) {
        cmdIndex--;
        Serial.print(F("\b \b")); // Erase char from terminal
      }
    }
    // Store character if there's room
    else if (cmdIndex < sizeof(cmdBuffer) - 1 && inChar >= 32) {
      cmdBuffer[cmdIndex++] = inChar;
      Serial.write(inChar); // Echo character
    }
  }
}

// Process a complete command
void processCommand() {
  // Convert command to lowercase for case-insensitive comparison
  for (uint8_t i = 0; i < cmdIndex; i++) {
    if (cmdBuffer[i] >= 'A' && cmdBuffer[i] <= 'Z') {
      cmdBuffer[i] += 32; // Convert to lowercase
    }
  }
  
  // Compare with command strings from flash memory
  if (strcmp_P(cmdBuffer, HELP_CMD) == 0) {
    setNeedsHelp(true);
  }
  else if (strcmp_P(cmdBuffer, START_CMD) == 0) {
    startExperiment();
  }
  else if (strcmp_P(cmdBuffer, PAUSE_CMD) == 0) {
    pauseExperiment();
  }
  else if (strcmp_P(cmdBuffer, STOP_CMD) == 0) {
    stopExperiment();
  }
  else if (strcmp_P(cmdBuffer, RESTART_CMD) == 0) {
    restartExperiment();
  }
  else if (strcmp_P(cmdBuffer, STATUS_CMD) == 0) {
    setNeedsStatus(true);
  }
  else if (strcmp_P(cmdBuffer, TARE_CMD) == 0) {
    Serial.println(F("Taring all sensors..."));
    sensors.tareAllSensors();  // Updated to use the new method name
    Serial.println(F("Tare complete."));
    setNeedsPrompt(true);
  }
  else if (strcmp_P(cmdBuffer, MONITOR_CMD) == 0) {
    setMonitorActive(!isMonitorActive());
    
    if (isMonitorActive()) {
      Serial.println(F("Sensor monitoring enabled. Use 'monitor' again to disable."));
    } else {
      Serial.println(F("Sensor monitoring disabled."));
    }
    
    setNeedsPrompt(true);
  }
  else if (strncmp_P(cmdBuffer, MOTOR_CMD, 5) == 0) {
    // Parse "motor X D rpm" command
    char* ptr = cmdBuffer + 5; // Skip "motor"
    
    // Skip spaces
    while (*ptr == ' ') ptr++;
    
    if (*ptr >= '1' && *ptr <= '5') {
      uint8_t motorID = *ptr - '0';
      ptr++; // Move past motor number
      
      // Skip spaces
      while (*ptr == ' ') ptr++;
      
      if (*ptr == 'f' || *ptr == 'r') {
        uint8_t direction = (*ptr == 'f') ? HIGH : LOW;
        ptr++; // Move past direction
        
        // Skip spaces and get RPM if provided
        while (*ptr == ' ') ptr++;
        
        uint8_t rpm = DEFAULT_RPM;
        if (*ptr >= '0' && *ptr <= '9') {
          rpm = atoi(ptr);
        }
        
        // Apply the motor command
        motors.setMotorSpeed(motorID, rpm, direction);
        
        Serial.print(F("Motor "));
        Serial.print(motorID);
        Serial.print(F(" running at "));
        Serial.print(rpm);
        Serial.print(F(" RPM, direction: "));
        Serial.println(direction == HIGH ? F("Forward") : F("Reverse"));
      } else {
        Serial.println(F("Invalid direction. Use 'f' for forward or 'r' for reverse."));
      }
    } else {
      Serial.println(F("Invalid motor ID. Use 1-5."));
    }
    
    setNeedsPrompt(true);
  }
  else if (strncmp_P(cmdBuffer, STOPMOTOR_CMD, 9) == 0) {
    // Parse "stopmotor X" command
    char* ptr = cmdBuffer + 9; // Skip "stopmotor"
    
    // Skip spaces
    while (*ptr == ' ') ptr++;
    
    if (*ptr >= '1' && *ptr <= '5') {
      uint8_t motorID = *ptr - '0';
      motors.stopMotor(motorID);
      
      Serial.print(F("Motor "));
      Serial.print(motorID);
      Serial.println(F(" stopped."));
    } else {
      Serial.println(F("Invalid motor ID. Use 1-5."));
    }
    
    setNeedsPrompt(true);
  }
  else if (strcmp_P(cmdBuffer, SENSORS_CMD) == 0) {
    Serial.println(F("\n----- Sensor Readings -----"));
    for (uint8_t i = 1; i <= 4; i++) {
      Serial.print(F("Load Cell "));
      Serial.print(i);
      Serial.print(F(": "));
      Serial.print(sensors.readSensor(i), 3);
      Serial.println(F(" N"));
    }
    Serial.println(F("--------------------------\n"));
    
    setNeedsPrompt(true);
  }
  else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmdBuffer);
    Serial.println(F("Type 'help' for available commands."));
    
    setNeedsPrompt(true);
  }
}

// Update the state machine (non-blocking)
void updateStateMachine() {
  // Emergency stop takes precedence
  if (isEmergencyStop()) {
    motors.stopAllMotors();
    currentState = STATE_IDLE;
    setEmergencyStop(false);
    return;
  }
  
  // Update based on current state
  switch (currentState) {
    case STATE_IDLE:
      // Nothing to do in idle state
      break;
      
    case STATE_ADJUSTING_FORCE:
    {
      // Read current force
      float currentForce = sensors.readSensor(3);
      
      // Determine if we need to adjust force
      if (currentForce >= FORCE_MIN && currentForce <= FORCE_MAX) {
        // Force is in acceptable range
        motors.stopMotor(5);
        setForceAdjusted(true);
        forceState = FORCE_OK;
        
        Serial.print(F("Force adjusted to target: "));
        Serial.print(currentForce, 3);
        Serial.print(F(" N (Target: "));
        Serial.print(TARGET_FORCE);
        Serial.println(F(" N)"));
        
        // Advance to next state
        currentState = STATE_RUNNING_PUMPS;
      } 
      // Force too low - increase
      else if (currentForce < FORCE_MIN) {
        if (forceState != FORCE_INCREASE) {
          motors.stopMotor(5);
          delay(50);
          forceState = FORCE_INCREASE;
          motors.setMotorSpeed(5, FORCE_INC_RPM, HIGH);
        }
      } 
      // Force too high - decrease
      else if (currentForce > FORCE_MAX) {
        if (forceState != FORCE_DECREASE) {
          motors.stopMotor(5);
          delay(50);
          forceState = FORCE_DECREASE;
          motors.setMotorSpeed(5, FORCE_DEC_RPM, LOW);
        }
      }
      break;
    }
    
    case STATE_RUNNING_PUMPS:
      // Start pumps if not already running
      if (!arePumpsRunning()) {
        motors.setMotorSpeed(1, PUMP_RPM, HIGH);
        motors.setMotorSpeed(2, PUMP_RPM, HIGH);
        setPumpsRunning(true);
        
        Serial.println(F("Pumps started."));
        
        // Advance to next state
        currentState = STATE_RUNNING_FULL;
      }
      break;
      
    case STATE_RUNNING_FULL:
      // Start rotation if not already running
      if (!isRotationRunning()) {
        motors.setMotorSpeed(3, ROTATION_RPM, HIGH);
        motors.setMotorSpeed(4, ROTATION_RPM, HIGH);
        setRotationRunning(true);
        
        Serial.println(F("Rotation started."));
      }
      break;
      
    case STATE_PAUSED:
      // Nothing to do in paused state
      break;
  }
}

// Start the experiment
void startExperiment() {
  if (currentState == STATE_PAUSED) {
    // Resume from paused state
    Serial.println(F("Resuming experiment..."));
    
    // Restart motors that were previously running
    if (isForceAdjusted() && arePumpsRunning()) {
      motors.setMotorSpeed(1, PUMP_RPM, HIGH);
      motors.setMotorSpeed(2, PUMP_RPM, HIGH);
    }
    
    if (isForceAdjusted() && arePumpsRunning() && isRotationRunning()) {
      motors.setMotorSpeed(3, ROTATION_RPM, HIGH);
      motors.setMotorSpeed(4, ROTATION_RPM, HIGH);
    }
    
    // Set appropriate state based on progress
    if (isRotationRunning()) {
      currentState = STATE_RUNNING_FULL;
    } else if (arePumpsRunning()) {
      currentState = STATE_RUNNING_PUMPS;
    } else if (isForceAdjusted()) {
      currentState = STATE_RUNNING_PUMPS;
    } else {
      currentState = STATE_ADJUSTING_FORCE;
    }
  } 
  else if (currentState == STATE_IDLE) {
    // Start from idle state
    Serial.println(F("Starting experiment..."));
    
    // Reset state flags
    setForceAdjusted(false);
    setPumpsRunning(false);
    setRotationRunning(false);
    
    // Begin with force adjustment
    forceState = FORCE_INCREASE;
    currentState = STATE_ADJUSTING_FORCE;
  } 
  else {
    Serial.println(F("Experiment already running."));
  }
  
  setNeedsPrompt(true);
}

// Pause the experiment
void pauseExperiment() {
  if (currentState != STATE_IDLE && currentState != STATE_PAUSED) {
    Serial.println(F("Experiment paused."));
    
    // Keep track of which components were running
    // but stop all motors
    motors.stopAllMotors();
    
    currentState = STATE_PAUSED;
  } else {
    Serial.println(F("Experiment not running, nothing to pause."));
  }
  
  setNeedsPrompt(true);
}

// Stop the experiment
void stopExperiment() {
  // Emergency stop all motors
  motors.stopAllMotors();
  
  // Reset all state flags
  setForceAdjusted(false);
  setPumpsRunning(false);
  setRotationRunning(false);
  
  currentState = STATE_IDLE;
  
  Serial.println(F("EMERGENCY STOP - All motors stopped!"));
  setNeedsPrompt(true);
}

// Restart the experiment
void restartExperiment() {
  Serial.println(F("Restarting experiment from beginning..."));
  
  // Stop all motors and reset state
  motors.stopAllMotors();
  
  // Reset all state flags
  setForceAdjusted(false);
  setPumpsRunning(false);
  setRotationRunning(false);
  
  // Start from the beginning
  forceState = FORCE_INCREASE;
  currentState = STATE_ADJUSTING_FORCE;
  
  setNeedsPrompt(true);
}

// Display command help
void displayHelp() {
  Serial.println(F("\n----- Tribology Experiment Commands -----"));
  Serial.println(F("start    - Start or resume the experiment"));
  Serial.println(F("pause    - Pause the experiment"));
  Serial.println(F("stop     - Emergency stop all motors"));
  Serial.println(F("restart  - Reset and restart experiment"));
  Serial.println(F("status   - Show current experiment status"));
  
  Serial.println(F("\n----- Motor Control Commands -----"));
  Serial.println(F("motor X D rpm - Run motor X at rpm speed"));
  Serial.println(F("              - X = motor ID (1-5)"));
  Serial.println(F("              - D = direction (f/r)"));
  Serial.println(F("              - rpm = speed (optional)"));
  Serial.println(F("              - Ex: motor 3 f 50"));
  Serial.println(F("stopmotor X   - Stop motor X (1-5)"));
  
  Serial.println(F("\n----- Sensor Commands -----"));
  Serial.println(F("sensors  - Read all sensors once"));
  Serial.println(F("monitor  - Toggle continuous readings"));
  Serial.println(F("tare     - Tare all sensors"));
  
  Serial.println(F("\n----- Other Commands -----"));
  Serial.println(F("help     - Show this help menu"));
  Serial.println(F("-------------------------------"));
}

// Display system status
void displayStatus() {
  Serial.println(F("\n----- Experiment Status -----"));
  
  Serial.print(F("State: "));
  switch (currentState) {
    case STATE_IDLE:
      Serial.println(F("IDLE"));
      break;
    case STATE_ADJUSTING_FORCE:
      Serial.println(F("ADJUSTING FORCE"));
      break;
    case STATE_RUNNING_PUMPS:
      Serial.println(F("RUNNING PUMPS"));
      break;
    case STATE_RUNNING_FULL:
      Serial.println(F("RUNNING FULL"));
      break;
    case STATE_PAUSED:
      Serial.println(F("PAUSED"));
      break;
  }
  
  Serial.print(F("Force adjusted: "));
  Serial.println(isForceAdjusted() ? F("YES") : F("NO"));
  
  Serial.print(F("Pumps running: "));
  Serial.println(arePumpsRunning() ? F("YES") : F("NO"));
  
  Serial.print(F("Rotation running: "));
  Serial.println(isRotationRunning() ? F("YES") : F("NO"));
  
  Serial.println(F("\nCurrent Forces:"));
  for (uint8_t i = 1; i <= 4; i++) {
    Serial.print(F("LC"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(sensors.readSensor(i), 3);
    Serial.println(F(" N"));
  }
  
  Serial.println(F("-----------------------------"));
}

// Display the command prompt
void displayPrompt() {
  printPgmString(CMD_PROMPT);
}
