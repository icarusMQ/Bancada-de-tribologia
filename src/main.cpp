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
const float FORCE_TOLERANCE = 0.03; // Tolerance band around the target force (N)
const float MAX_ADJUST_RPM = MA_RPM; // Maximum speed for adjustment motor
const float ADJUSTMENT_SPEED = MAX_ADJUST_RPM; // Fixed speed for adjustment

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

// Function to adjust force using a blocking loop
void adjustFreeSphereForce(float targetForce, float tolerance) {
  Serial.println("Starting force adjustment...");
  bool adjustmentInterrupted = false;

  while (true) {
    // **** THIS IS CRUCIAL for AccelStepper ****
    // Update motor positions/speeds frequently, even during adjustment
    motorController.UpdateMotors();
    // ******************************************

    // Check for serial input to allow interruption
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "stop") {
        Serial.println("Force adjustment interrupted by user.");
        motorController.StopMotor(MA); // Stop the adjustment motor
        experimentRunning = false; // Stop the overall experiment
        adjustmentInterrupted = true;
        break; // Exit the adjustment loop
      } else {
        Serial.print("Command '");
        Serial.print(command);
        Serial.println("' ignored during force adjustment. Use 'stop' to interrupt.");
      }
    }

    float currentForce = sensors.ReadSensor(SFrx); // Read load cell
    Serial.print("Current Force: ");
    Serial.print(currentForce, 2);
    float error = targetForce - currentForce;

    // Check if force is within the target tolerance
    if (abs(error) <= tolerance) {
      motorController.StopMotor(MA); // Stop adjustment motor
      Serial.println("Force within tolerance.");
      Serial.print("Final Force: ");
      Serial.print(currentForce, 2);
      Serial.println(" N");
      break; // Exit the adjustment loop
    }

    // Force needs adjustment - Use fixed speed
    if (error > 0) {
      // Force is too low, need to increase (move motor UP - verify direction!)
      motorController.RunMotor(MA, -ADJUSTMENT_SPEED); // Negative speed assumed for UP, use fixed speed
    } else { // error < 0
      // Force is too high, need to decrease (move motor DOWN - verify direction!)
      motorController.RunMotor(MA, ADJUSTMENT_SPEED); // Positive speed assumed for DOWN, use fixed speed
    }

    // Small delay to prevent overwhelming the loop and allow serial check
    //delay(1);
  }

  if (!adjustmentInterrupted) {
    Serial.println("Force adjustment complete.");
  }
}

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

  // Check for serial input (only if experiment is NOT running or already past force adjustment)
  // Force adjustment loop handles its own serial check for 'stop'
  if (Serial.available() && (!experimentRunning || forceAdjusted)) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove potential whitespace

    // Stop any manual movement before processing new command
    motorController.StopMotor(MPFix);
    motorController.StopMotor(MPFre);
    motorController.StopMotor(MAFre);
    motorController.StopMotor(MAFix);

    if (command == "start") {
        Serial.println("Starting experiment.");
        experimentRunning = true;
        forceAdjusted = false; // Reset state for new experiment
        pumpsStarted = false;
        rotationStarted = false;
        // Stop all motors initially
        motorController.StopMotor(MPFix);
        motorController.StopMotor(MPFre);
        motorController.StopMotor(MA);
        motorController.StopMotor(MAFre);
        motorController.StopMotor(MAFix);
    } else if (command == "stop") {
        Serial.println("Stopping experiment.");
        experimentRunning = false;
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
    } else if (command.length() == 1 && !experimentRunning) { // Only allow manual control if experiment not running
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
    } else if (command.length() == 1 && experimentRunning) {
         Serial.println("Manual motor control disabled while experiment is running. Use 'stop' first.");
    }
     else {
        Serial.println("Unknown command.");
    }
  }

  // Experiment Logic
  if (experimentRunning) {
    // Step 1: Adjust the free sphere force (runs until complete or interrupted)
    if (!forceAdjusted) {
      adjustFreeSphereForce(TARGET_FORCE_FREE_SPHERE, FORCE_TOLERANCE);
      // If adjustFreeSphereForce was interrupted by 'stop', experimentRunning will be false.
      if (experimentRunning) {
          forceAdjusted = true; // Mark as complete only if not interrupted
      } else {
          return; // Skip rest of the loop if stopped during adjustment
      }
    }

    // Step 2: Start the pumps (only if force is adjusted and pumps not already started)
    else if (!pumpsStarted) {
      motorController.RunMotor(MPFix, PUMP_RPM); // Start pump 1
      motorController.RunMotor(MPFre, PUMP_RPM); // Start pump 2
      pumpsStarted = true;
      Serial.println("Pumps started.");
    }
    // Step 3: Start the rotation of the axes (only if pumps started and rotation not already started)
    else if (!rotationStarted) {
      // NOTE: MA motor (Motor 3) was used for force adjustment. Now it's used for rotation.
      // Ensure it's stopped from adjustment before starting rotation if needed,
      // although adjustFreeSphereForce should handle stopping it.
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
  delay(1); // Reduced delay as adjust loop has its own
}