#include "MultiScaleSensors.h"

// Constructor: initialize variables.
// This initializes the class variables, including the last stable readings
// for each scale and their active states.
MultiScaleSensors::MultiScaleSensors() : 
  lastStable1(0), lastStable2(0), lastStable3(0), lastStable4(0),
  active1(false), active2(false), active3(false), active4(false)
{
  // Nothing further needed here.
}

// Helper: Tare a scale with timeout.
// This function tars (zeros) a specific scale. It waits for the scale to be ready
// and then performs the tare operation. If the scale does not respond within
// the timeout period, it skips the scale and marks it as inactive.
// Parameters:
// - scale: The HX711 object representing the scale.
// - scaleName: A string representing the name of the scale (e.g., "Scale 1").
// Returns:
// - true if the scale was successfully tared, false otherwise.
bool MultiScaleSensors::tareScale(HX711 &scale, const char* scaleName) {
  Serial.print("Taring ");
  Serial.print(scaleName);
  Serial.println("... Remove any load.");
  delay(15000);  // Wait before taring to ensure the scale is unloaded.

  unsigned long startTime = millis();
  while (!scale.is_ready() && (millis() - startTime < TARE_TIMEOUT)) {
    delay(50);  // Check periodically if the scale is ready.
  }

  if (scale.is_ready()) {
    scale.tare();  // Perform the tare operation.
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
// This function calculates the median value of an array of floats using
// a simple insertion sort algorithm to sort the array.
// Parameters:
// - arr: The array of float values.
// - n: The number of elements in the array.
// Returns:
// - The median value of the array.
float MultiScaleSensors::computeMedian(float arr[], int n) {
  // Sort the array using insertion sort.
  for (int i = 1; i < n; i++) {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
  
  // Return the median value.
  if (n % 2 == 1)
    return arr[n / 2];  // Odd number of elements.
  else
    return (arr[(n - 1) / 2] + arr[n / 2]) / 2.0;  // Even number of elements.
}

// Helper: Read a batch from a given scale and return the median weight.
// This function reads multiple samples from a scale, calculates the weight
// for each sample, and returns the median weight. If the scale is inactive
// or no valid samples are collected, it returns 0.
// Parameters:
// - scale: The HX711 object representing the scale.
// - cal: The calibration factor for the scale.
// - numSamples: The number of samples to read.
// - isActive: Whether the scale is active.
// Returns:
// - The median weight of the valid samples, or 0 if no valid samples are collected.
float MultiScaleSensors::getMedianReading(HX711 &scale, float cal, int numSamples, bool isActive) {
  if (!isActive) return 0;  // Return 0 if the scale is inactive.

  float validSamples[NUM_SAMPLES];  // Array to store valid samples.
  int count = 0;  // Counter for valid samples.
  
  for (int i = 0; i < numSamples; i++) {
    if (scale.is_ready()) {
      long raw = scale.read();  // Read raw data from the scale.
      float weight = (raw - scale.get_offset()) * cal;  // Convert raw data to weight.
      validSamples[count++] = weight;  // Store the valid sample.
    }
    delay(20);  // Delay between samples.
  }
  
  if (count == 0)
    return 0;  // Return 0 if no valid samples were collected.
  else
    return computeMedian(validSamples, count);  // Return the median of the valid samples.
}

// Public: Initialize all sensors.
// This function initializes all the scales by setting up their pins,
// starting their communication, and taring them. It also determines
// which scales are active based on the success of the taring process.
// Returns:
// - true if at least one scale is active, false otherwise.
bool MultiScaleSensors::BeginSensors() {
  Serial.begin(115200);  // Start serial communication at 115200 baud.

  // Begin scale instances with pin assignments.
  scale1.begin(SCALE1_DT_PIN, SCALE1_SCK_PIN);
  scale2.begin(SCALE2_DT_PIN, SCALE2_SCK_PIN);
  scale3.begin(SCALE3_DT_PIN, SCALE3_SCK_PIN);
  scale4.begin(SCALE4_DT_PIN, SCALE4_SCK_PIN);

  // Tare each sensor and determine if it is active.
  active1 = tareScale(scale1, "Scale 1");
  active2 = tareScale(scale2, "Scale 2");
  active3 = tareScale(scale3, "Scale 3");
  active4 = tareScale(scale4, "Scale 4");

  Serial.println("\nTare process complete. Active scales will be read; inactive scales will be 0.");
  Serial.println("Starting data output for Serial Plotter...");
  return (active1 || active2 || active3 || active4);  // Return true if any scale is active.
}

// Public: Returns the filtered reading for the given sensor.
// This function reads the current weight from the specified sensor,
// applies a smoothing filter to stabilize the reading, and returns
// the filtered value. If the sensor is inactive or has no valid reading,
// it returns 0.
// Parameters:
// - sensorID: The ID of the sensor to read (1, 2, 3, or 4).
// Returns:
// - The filtered weight reading for the specified sensor, or 0 if invalid.
float MultiScaleSensors::ReadSensor(uint8_t sensorID) {
  float current = 0;  // Variable to store the current reading.
  switch(sensorID) {
    case 1:
      current = getMedianReading(scale1, calibration_factor1, NUM_SAMPLES, active1);
      if (active1 && current != 0)
        lastStable1 = (1 - smoothingFactor) * lastStable1 + smoothingFactor * current;  // Apply smoothing.
      else
        lastStable1 = 0;  // Reset if inactive or invalid.
      return lastStable1;
    case 2:
      current = getMedianReading(scale2, calibration_factor2, NUM_SAMPLES, active2);
      if (active2 && current != 0)
        lastStable2 = (1 - smoothingFactor) * lastStable2 + smoothingFactor * current;  // Apply smoothing.
      else
        lastStable2 = 0;  // Reset if inactive or invalid.
      return lastStable2;
    case 3:
      current = getMedianReading(scale3, calibration_factor3, NUM_SAMPLES, active3);
      if (active3 && current != 0)
        lastStable3 = (1 - smoothingFactor) * lastStable3 + smoothingFactor * current;  // Apply smoothing.
      else
        lastStable3 = 0;  // Reset if inactive or invalid.
      return lastStable3;
    case 4:
      current = getMedianReading(scale4, calibration_factor4, NUM_SAMPLES, active4);
      if (active4 && current != 0)
        lastStable4 = (1 - smoothingFactor) * lastStable4 + smoothingFactor * current;  // Apply smoothing.
      else
        lastStable4 = 0;  // Reset if inactive or invalid.
      return lastStable4;
    default:
      Serial.println("Invalid sensor ID. Use 1, 2, 3, or 4.");  // Handle invalid sensor ID.
      return 0;
  }
}