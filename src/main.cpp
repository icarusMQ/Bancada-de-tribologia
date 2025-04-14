#include "MultiScaleSensors.h"

MultiScaleSensors sensors;

void setup() {
  // Initialize sensors.
  if (!sensors.BeginSensors()) {
    Serial.println("Warning: No sensor responded properly during startup!");
  }
}

void loop() {
  // For example, read sensor 1 and sensor 2 and print their values.
  float value1 = sensors.ReadSensor(1);
  float value2 = sensors.ReadSensor(2);
  float value3 = sensors.ReadSensor(3);
  float value4 = sensors.ReadSensor(4);
  
  Serial.print("Scale 1: ");
  Serial.print(value1, 2);
  Serial.print(" | Scale 2: ");
  Serial.print(value2, 2);
  Serial.print(" | Scale 3: ");
  Serial.print(value3, 2);
  Serial.print(" | Scale 4: ");
  Serial.println(value4, 2);

  delay(20);  // Minimal delay for responsive output.
}
