#include <MultiScaleSensors.h>
#include <StepperMotorController.h>

// Create instances of the libraries.
MultiScaleSensors sensors;
StepperMotorController motorController;

// Experiment parameters
const float TARGET_FORCE_FREE_SPHERE = 0.3; // Target force for free sphere (N)
const float PUMP_RPM = 0.5;                    // Pump speed to achieve 30 drops/min (use float for low RPM)
const int ROTATION_RPM = 80;                // Rotation speed for axes (RPM)
const int MA_RPM = 60;             // RPM for manual control

// Define constants for motor and sensor mappings
const int MPFix = 1;
const int MPFre = 2;
const int MA = 3;
const int MAFre = 4;
const int MAFix = 5;

const int SFix = 1;
const int SFiz = 2;
const int SFrx = 3;
const int SFrz = 4;

// Experiment state
bool forceAdjusted = false;
bool pumpsStarted = false;
bool rotationStarted = false;
bool experimentRunning = false;
bool adjustingForce = false; // Flag to indicate force adjustment motor is active

void setup() {
  Serial.begin(115200);
  Serial.println("Tribology Experiment Setup");

  // Initialize sensors
  if (!sensors.BeginSensors()) {
    Serial.println("WARNING: No sensors responded properly during startup!");
    Serial.println("Check connections and restart the system.");
  } else {
    Serial.println("Sensors initialized successfully.");
    Serial.println(" Use \"start\" to start, \"stop\" to stop, or print to print scale readings.");
    Serial.println(" Manual controls: q/a (M1), w/s (M2), e/d (M3), r/f (M4), t/g (M5)");
  }
}

void loop() {
  // **** THIS IS CRUCIAL for AccelStepper ****
  // Update all motor positions/speeds frequently
  motorController.UpdateMotors();
  // ******************************************

  // Check for serial input
  if (Serial.available() && !adjustingForce) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove potential whitespace

    // Stop any manual movement before processing new command
    motorController.StopMotor(MPFix);
    motorController.StopMotor(MPFre);
    motorController.StopMotor(MA);
    motorController.StopMotor(MAFre);
    if (!adjustingForce) { // Don't stop motor 5 if it's adjusting force
        motorController.StopMotor(MAFix);
    }

    if (command == "start") {
        Serial.println("Starting experiment.");
        experimentRunning = true;
        forceAdjusted = false; // Reset state for new experiment
        pumpsStarted = false;
        rotationStarted = false;
        adjustingForce = false;
        // Stop all motors initially
        motorController.StopMotor(MPFix);
        motorController.StopMotor(MPFre);
        motorController.StopMotor(MA);
        motorController.StopMotor(MAFre);
        motorController.StopMotor(MAFix);
    } else if (command == "stop") {
        Serial.println("Stopping experiment.");
        experimentRunning = false;
        adjustingForce = false;
        // Stop all motors
        motorController.StopMotor(MPFix);
        motorController.StopMotor(MPFre);
        motorController.StopMotor(MA);
        motorController.StopMotor(MAFre);
        motorController.StopMotor(MAFix);
    } else if (command == "print") {
        Serial.println("Print scale readings");
        double value1 = sensors.ReadSensor(SFix);
        double value2 = sensors.ReadSensor(SFiz);
        double value3 = sensors.ReadSensor(SFrx);
        double value4 = sensors.ReadSensor(SFrz);
        Serial.print("scale1: ");
        Serial.print(value1, 2);
        Serial.print(", scale2: ");
        Serial.print(value2, 2);
        Serial.print(", scale3: ");
        Serial.print(value3, 2);
        Serial.print(", scale4: ");
        Serial.println(value4, 2);
    } else if (command.length() == 1) { // Handle single-character commands
        switch (command[0]) {
            case 'q': // Motor 1 forward
                Serial.println("Moving Motor 1 forward.");
                motorController.RunMotor(MPFix, MA_RPM);
                break;
            case 'a': // Motor 1 backward
                Serial.println("Moving Motor 1 backward.");
                motorController.RunMotor(MPFix, -MA_RPM);
                break;
            case 'w': // Motor 2 forward
                Serial.println("Moving Motor 2 forward.");
                motorController.RunMotor(MPFre, MA_RPM);
                break;
            case 's': // Motor 2 backward
                Serial.println("Moving Motor 2 backward.");
                motorController.RunMotor(MPFre, -MA_RPM);
                break;
            case 'e': // Motor 3 forward
                Serial.println("Moving Motor 3 forward.");
                motorController.RunMotor(MA, MA_RPM);
                break;
            case 'd': // Motor 3 backward
                Serial.println("Moving Motor 3 backward.");
                motorController.RunMotor(MA, -MA_RPM);
                break;
            case 'r': // Motor 4 forward
                Serial.println("Moving Motor 4 forward.");
                motorController.RunMotor(MAFre, MA_RPM);
                break;
            case 'f': // Motor 4 backward
                Serial.println("Moving Motor 4 backward.");
                motorController.RunMotor(MAFre, -MA_RPM);
                break;
            case 't': // Motor 5 forward
                Serial.println("Moving Motor 5 forward.");
                motorController.RunMotor(MAFix, MA_RPM);
                break;
            case 'g': // Motor 5 backward
                Serial.println("Moving Motor 5 backward.");
                motorController.RunMotor(MAFix, -MA_RPM);
                break;
            default:
                Serial.println("Unknown command.");
                break;
        }
    } else {
        Serial.println("Unknown command.");
    }
  }

  // Experiment Logic
  if (experimentRunning) {
    // Step 1: Adjust the free sphere force
    // --- Force Adjustment State Machine ---
    enum ForceAdjustmentState {
      IDLE,
      ADJUSTING,
      ADJUSTED
    };
    static ForceAdjustmentState forceState = IDLE; // Keep state between loop iterations
    const float FORCE_TOLERANCE = 0.03; // Tolerance band around the target force (N)
    const float MIN_ADJUST_RPM = 1.0;   // Minimum speed for adjustment motor
    const float MAX_ADJUST_RPM = MA_RPM; // Maximum speed for adjustment motor
    const float FORCE_PROPORTIONAL_GAIN = 50.0; // Proportional gain (tune this value) - higher means faster response for larger errors

    if (!forceAdjusted) {
      float currentForce = sensors.ReadSensor(SFrx); // Read load cell for free sphere
      float error = TARGET_FORCE_FREE_SPHERE - currentForce;

      // Check if force is within the target tolerance
      if (abs(error) <= FORCE_TOLERANCE) {
      if (forceState == ADJUSTING) {
        motorController.StopMotor(MA); // Stop adjustment motor
        Serial.println("Force within tolerance.");
        Serial.print("Final Force: ");
        Serial.print(currentForce, 2);
        Serial.println(" N");
      }
      forceState = ADJUSTED;
      forceAdjusted = true; // Mark this step as complete
      adjustingForce = false; // Ensure flag is reset
      } else {
      // Force needs adjustment
      forceState = ADJUSTING;
      adjustingForce = true; // Indicate motor 5 is active for adjustment

      // Calculate speed based on error (proportional control)
      float targetSpeed = MIN_ADJUST_RPM + FORCE_PROPORTIONAL_GAIN * abs(error);
      // Clamp speed between min and max
      targetSpeed = constrain(targetSpeed, MIN_ADJUST_RPM, MAX_ADJUST_RPM);

      // Keep track of the last commanded direction for motor 5
      static int lastAdjustmentDirection = 0; // 0 = stopped/unknown, 1 = UP (positive error), -1 = DOWN (negative error)

      if (error > 0) {
        // Force is too low, need to increase (move motor 5 forward - verify direction!)
        if (lastAdjustmentDirection != 1) { // Print only when changing direction/starting to move UP
         Serial.print("Adjusting force UP. Current: ");
         Serial.print(currentForce, 2);
         Serial.print(" N, Target: ");
         Serial.print(TARGET_FORCE_FREE_SPHERE);
         Serial.print(" N, Speed: ");
         Serial.println(targetSpeed);
         lastAdjustmentDirection = 1; // Mark as moving UP
        }
        motorController.RunMotor(MA, -targetSpeed); // Positive speed assumed for UP
      } else { // error < 0
        // Force is too high, need to decrease (move motor 5 backward - verify direction!)
         if (lastAdjustmentDirection != -1) { // Print only when changing direction/starting to move DOWN
         Serial.print("Adjusting force DOWN. Current: ");
         Serial.print(currentForce, 2);
         Serial.print(" N, Target: ");
         Serial.print(TARGET_FORCE_FREE_SPHERE);
         Serial.print(" N, Speed: ");
         Serial.println(targetSpeed);
         lastAdjustmentDirection = -1; // Mark as moving DOWN
         }
        motorController.RunMotor(MA, targetSpeed); // Negative speed for reverse
      }
      forceAdjusted = false; // Ensure we stay in this adjustment step
      return; // Skip to next loop iteration to avoid running pumps/rotation
      }
      return; // Skip to next loop iteration to avoid running pumps/rotation
    }
    // --- End Force Adjustment State Machine ---

    // Step 2: Start the pumps (only if force is adjusted and pumps not already started)
    else if (!pumpsStarted) {
      motorController.RunMotor(MPFix, PUMP_RPM); // Start pump 1
      motorController.RunMotor(MPFre, PUMP_RPM); // Start pump 2
      pumpsStarted = true;
      Serial.println("Pumps started.");
    }
    // Step 3: Start the rotation of the axes (only if pumps started and rotation not already started)
    else if (!rotationStarted) {
      motorController.RunMotor(MA, ROTATION_RPM); // Start free sphere axis
      motorController.RunMotor(MAFre, ROTATION_RPM); // Start fixed sphere axis
      rotationStarted = true;
      Serial.println("Rotation started.");
      Serial.println("Experiment running. Send \"stop\" to stop.");
    }

    // Continuous Data Logging during experiment
    if (rotationStarted) { // Log data only after rotation starts
        float value1 = sensors.ReadSensor(SFix);
        float value2 = sensors.ReadSensor(SFiz);
        float value3 = sensors.ReadSensor(SFrx);
        float value4 = sensors.ReadSensor(SFrz);

        // Format the output for the Serial Plotter
        Serial.print(">");
        Serial.print("scale1:");
        Serial.print(value1, 2);
        Serial.print(",");
        Serial.print("scale2:");
        Serial.print(value2, 2);
        Serial.print(",");
        Serial.print("scale3:");
        Serial.print(value3, 2);
        Serial.print(",");
        Serial.print("scale4:");
        Serial.print(value4, 2);
        Serial.println(); // Ends the line with \r\n
    }
  } // end if(experimentRunning)


  // Add a small delay to prevent overwhelming the serial port,
  // but keep it short enough for smooth motor operation.
}
