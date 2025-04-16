#include "MultiScaleSensors.h"

// Constructor: Initialize member variables.
MultiScaleSensors::MultiScaleSensors() : 
  lastStable1(0), lastStable2(0), lastStable3(0), lastStable4(0),
  active1(false), active2(false), active3(false), active4(false)
{
  // Nothing else required in constructor.
}

// Helper: Tare a scale with a timeout.
// Uses blocking delay() calls to allow taring during initialization.
bool MultiScaleSensors::tareScale(HX711 &scale, const char* scaleName) {
  Serial.print("Taring ");
  Serial.print(scaleName);
  Serial.println("... Remove any load.");
  
 
  vTaskDelay(15000 / portTICK_PERIOD_MS);  // Wait for 15 seconds to ensure scale is unloaded.

  unsigned long startTime = millis();
  while (!scale.is_ready() && (millis() - startTime < TARE_TIMEOUT)) {
   
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }

  if (scale.is_ready()) {
    scale.tare();  // Perform the tare.
    Serial.print(scaleName);
    Serial.println(" is ready.");
    return true;
  } else {
    Serial.print(scaleName);
    Serial.println(" not responding in time. Skipping this load cell.");
    return false;
  }
}

// Helper: Compute the median value for an array of floats.
float MultiScaleSensors::computeMedian(float arr[], int n) {
  // Insertion sort.
  for (int i = 1; i < n; i++) {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
  
  // Return median value.
  if (n % 2 == 1)
    return arr[n / 2];
  else
    return (arr[(n - 1) / 2] + arr[n / 2]) / 2.0;
}

// Helper: Read a batch from a scale and return the median value.
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
    // Use vTaskDelay instead of Arduino delay.
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
  
  if (count == 0)
    return 0;
  else
    return computeMedian(validSamples, count);
}

// Public: Initialize and tare all sensors.
bool MultiScaleSensors::BeginSensors() {
  Serial.println("Initializing scales...");
  
  // Begin scale instances with pin assignments.
  scale1.begin(SCALE1_DT_PIN, SCALE1_SCK_PIN);
  scale2.begin(SCALE2_DT_PIN, SCALE2_SCK_PIN);
  scale3.begin(SCALE3_DT_PIN, SCALE3_SCK_PIN);
  scale4.begin(SCALE4_DT_PIN, SCALE4_SCK_PIN);

  // Tare each scale.
  active1 = tareScale(scale1, "Scale 1");
  active2 = tareScale(scale2, "Scale 2");
  active3 = tareScale(scale3, "Scale 3");
  active4 = tareScale(scale4, "Scale 4");

  Serial.println("\nTare process complete. Active scales will be read; inactive scales will be 0.");
  Serial.println("Starting data output for Serial Plotter...");
  
  // Return true if at least one scale is active.
  return (active1 || active2 || active3 || active4);
}

// Public: Return filtered reading for a given sensor.
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
