#include "MultiScaleSensors.h"
#include "StepperMotorController.h"

// Create instances of the libraries.
MultiScaleSensors sensors;
StepperMotorController motorController;

void setup() {
  Serial.begin(115200);

  // Initialize sensors.
  if (!sensors.BeginSensors()) {
    Serial.println("Warning: No sensor responded properly during startup!");
  }

  // Example: Start motor 1 at 60 RPM in forward direction.
  motorController.RunMotor(1, 60, HIGH);
  delay(2000); // Run for 2 seconds.

  // Stop motor 1.
  motorController.StopMotor(1);
}

void loop() {
  // Read sensor values.
  float value1 = sensors.ReadSensor(1);
  float value2 = sensors.ReadSensor(2);
  float value3 = sensors.ReadSensor(3);
  float value4 = sensors.ReadSensor(4);

  // Print sensor values to the Serial Monitor.
  Serial.print("Scale 1: ");
  Serial.print(value1, 2);
  Serial.print(" | Scale 2: ");
  Serial.print(value2, 2);
  Serial.print(" | Scale 3: ");
  Serial.print(value3, 2);
  Serial.print(" | Scale 4: ");
  Serial.println(value4, 2);

  // Example: Control motors based on sensor values.
  if (value1 > 50.0) {
    // If Scale 1 reads more than 50, run motor 2 in reverse at 100 RPM.
    motorController.RunMotor(2, 100, LOW);
  } else {
    // Otherwise, stop motor 2.
    motorController.StopMotor(2);
  }

  if (value2 > 30.0) {
    // If Scale 2 reads more than 30, run motor 3 forward at 80 RPM.
    motorController.RunMotor(3, 80, HIGH);
  } else {
    // Otherwise, stop motor 3.
    motorController.StopMotor(3);
  }

  delay(500); // Delay for half a second before the next loop iteration.
}
