#include "MultiScaleSensors.h"

// Constructor: initialize variables.
MultiScaleSensors::MultiScaleSensors() : 
  lastStable1(0), lastStable2(0), lastStable3(0), lastStable4(0),
  active1(false), active2(false), active3(false), active4(false)
{
  // Nothing further needed here.
}

// Helper: Tare a scale with timeout.
// It prints status messages and returns true if the scale responds.
bool MultiScaleSensors::tareScale(HX711 &scale, const char* scaleName) {
  Serial.print("Taring ");
  Serial.print(scaleName);
  Serial.println("... Remove any load.");
  delay(15000);  // Wait before taring

  unsigned long startTime = millis();
  while (!scale.is_ready() && (millis() - startTime < TARE_TIMEOUT)) {
    delay(50);
  }

  if (scale.is_ready()) {
    scale.tare();
    Serial.print(scaleName);
    Serial.println(" is ready.");
    return true;
  } else {
    Serial.print(scaleName);
    Serial.println(" not responding in time. Skipping this load cell.");
    return false;
  }
}

// Helper: Compute the median of an array of floats.
float MultiScaleSensors::computeMedian(float arr[], int n) {
  // Simple insertion sort
  for (int i = 1; i < n; i++) {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
  
  if (n % 2 == 1)
    return arr[n / 2];
  else
    return (arr[(n - 1) / 2] + arr[n / 2]) / 2.0;
}

// Helper: Read a batch from a given scale and return the median weight.
// If the sensor is inactive or has no valid sample, it returns 0.
float MultiScaleSensors::getMedianReading(HX711 &scale, float cal, int numSamples, bool isActive) {
  if (!isActive) return 0;

  float validSamples[NUM_SAMPLES];
  int count = 0;
  
  for (int i = 0; i < numSamples; i++) {
    if (scale.is_ready()) {
      long raw = scale.read();
      float weight = (raw - scale.get_offset()) * cal;
      validSamples[count++] = weight;
    }
    delay(20);
  }
  
  if (count == 0)
    return 0;
  else
    return computeMedian(validSamples, count);
}

// Public: Initialize all sensors.
// This sets up pins, begins the sensors, and tares each sensor.
// It returns true if at least one sensor becomes active.
bool MultiScaleSensors::BeginSensors() {
  Serial.begin(115200);  // Ensure correct baud rate

  // Begin scale instances with pin assignments.
  scale1.begin(SCALE1_DT_PIN, SCALE1_SCK_PIN);
  scale2.begin(SCALE2_DT_PIN, SCALE2_SCK_PIN);
  scale3.begin(SCALE3_DT_PIN, SCALE3_SCK_PIN);
  scale4.begin(SCALE4_DT_PIN, SCALE4_SCK_PIN);

  // Tare each sensor.
  active1 = tareScale(scale1, "Scale 1");
  active2 = tareScale(scale2, "Scale 2");
  active3 = tareScale(scale3, "Scale 3");
  active4 = tareScale(scale4, "Scale 4");

  Serial.println("\nTare process complete. Active scales will be read; inactive scales will be 0.");
  Serial.println("Starting data output for Serial Plotter...");
  return (active1 || active2 || active3 || active4);
}

// Public: Returns the filtered reading for the given sensor.
// sensorID should be 1, 2, 3, or 4.
float MultiScaleSensors::ReadSensor(uint8_t sensorID) {
  float current = 0;
  switch(sensorID) {
    case 1:
      current = getMedianReading(scale1, calibration_factor1, NUM_SAMPLES, active1);
      if (active1 && current != 0)
        lastStable1 = (1 - smoothingFactor) * lastStable1 + smoothingFactor * current;
      else
        lastStable1 = 0;
      return lastStable1;
    case 2:
      current = getMedianReading(scale2, calibration_factor2, NUM_SAMPLES, active2);
      if (active2 && current != 0)
        lastStable2 = (1 - smoothingFactor) * lastStable2 + smoothingFactor * current;
      else
        lastStable2 = 0;
      return lastStable2;
    case 3:
      current = getMedianReading(scale3, calibration_factor3, NUM_SAMPLES, active3);
      if (active3 && current != 0)
        lastStable3 = (1 - smoothingFactor) * lastStable3 + smoothingFactor * current;
      else
        lastStable3 = 0;
      return lastStable3;
    case 4:
      current = getMedianReading(scale4, calibration_factor4, NUM_SAMPLES, active4);
      if (active4 && current != 0)
        lastStable4 = (1 - smoothingFactor) * lastStable4 + smoothingFactor * current;
      else
        lastStable4 = 0;
      return lastStable4;
    default:
      Serial.println("Invalid sensor ID. Use 1, 2, 3, or 4.");
      return 0;
  }
}
