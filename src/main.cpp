#include "MultiScaleSensors.h"
#include "StepperMotorController.h"

// Create instances of the libraries.
MultiScaleSensors sensors;
StepperMotorController motorController;

// Experiment parameters
const float TARGET_FORCE_FREE_SPHERE = 0.3; // Target force for free sphere (N)
const int PUMP_RPM = 0.5;                    // Pump speed to achieve 30 drops/min
const int ROTATION_RPM = 80;                // Rotation speed for axes (RPM)

// Experiment state
bool forceAdjusted = false;
bool pumpsStarted = false;
bool rotationStarted = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Tribology Experiment Setup");

  // Initialize sensors
  if (!sensors.BeginSensors()) {
    Serial.println("WARNING: No sensors responded properly during startup!");
    Serial.println("Check connections and restart the system.");
  } else {
    Serial.println("Sensors initialized successfully.");
  }
}

void loop() {
  // Step 1: Adjust the free sphere force
  if (!forceAdjusted) {
    float currentForce = sensors.ReadSensor(1); // Read load cell for free sphere
    if (currentForce < TARGET_FORCE_FREE_SPHERE) {
      motorController.RunMotor(5, 10, HIGH); // Adjust force motor
      Serial.print("Adjusting force... Current: ");
      Serial.print(currentForce, 2);
      Serial.println(" N");
    } else {
      motorController.StopMotor(5); // Stop motor once target force is reached
      forceAdjusted = true;
      Serial.println("Free sphere force adjusted to target.");
    }
    return; // Exit loop to avoid running other steps prematurely
  }

  // Step 2: Start the pumps
  if (forceAdjusted) {
    motorController.RunMotor(1, PUMP_RPM, HIGH); // Start pump 1
    motorController.RunMotor(2, PUMP_RPM, HIGH); // Start pump 2
    pumpsStarted = true;
    Serial.println("Pumps started.");
    return; // Exit loop to avoid running other steps prematurely
  }

  // Step 3: Start the rotation of the axes
  if (pumpsStarted) {
    motorController.RunMotor(3, ROTATION_RPM, HIGH); // Start free sphere axis
    motorController.RunMotor(4, ROTATION_RPM, HIGH); // Start fixed sphere axis
    rotationStarted = true;
    Serial.println("Rotation started.");
    return; // Exit loop to avoid running other steps prematurely
  }

  // Step 4: Monitor and log sensor values
  float value1 = sensors.ReadSensor(1);
  float value2 = sensors.ReadSensor(2);
  float value3 = sensors.ReadSensor(3);
  float value4 = sensors.ReadSensor(4);

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

  delay(50); // Adjust the delay as needed
}
