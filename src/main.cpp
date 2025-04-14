#include "HX711.h"

// Create HX711 objects for four scales
HX711 scale1;
HX711 scale2;
HX711 scale3;
HX711 scale4;

// Pin definitions for Scale 1
#define SCALE1_DT_PIN 2
#define SCALE1_SCK_PIN 3
// Pin definitions for Scale 2
#define SCALE2_DT_PIN 4
#define SCALE2_SCK_PIN 6
// Pin definitions for Scale 3 (new)
#define SCALE3_DT_PIN 10
#define SCALE3_SCK_PIN 7
// Pin definitions for Scale 4 (new)
#define SCALE4_DT_PIN 12
#define SCALE4_SCK_PIN 11

// Calibration factors for each scale; adjust as necessary
float calibration_factor1 = 0.02808;
float calibration_factor2 = 0.0342;
float calibration_factor3 = -0.05671;
float calibration_factor4 = -0.0614;

// Number of samples to average for each measurement batch
const int NUM_SAMPLES = 8;

// Stability filter smoothing factor (0 < smoothingFactor <= 1)
// Higher value gives faster response.
const float smoothingFactor = 0.6;

// Global variables to store the last stable (filtered) measurements
float lastStable1 = 0;
float lastStable2 = 0;
float lastStable3 = 0;
float lastStable4 = 0;

// Boolean flags to indicate whether each scale is active
bool active1 = false;
bool active2 = false;
bool active3 = false;
bool active4 = false;

// Tare timeout period in milliseconds
const long TARE_TIMEOUT = 5000;  // Reduced timeout for faster startup

//--------------------------------------------------------------------
// Helper function to tare a scale with timeout.
// Returns true if the scale became ready and was tared, false otherwise.
bool tareScale(HX711 &scale, const char* scaleName) {
  Serial.print("Taring ");
  Serial.print(scaleName);
  Serial.println("... Remove any load.");
  delay(15000);
  unsigned long startTime = millis();

  // Wait for the scale to be ready or until timeout is reached.
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

//--------------------------------------------------------------------
// Helper function: computes the median of an array of floats.
float computeMedian(float arr[], int n) {
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
  // Return median value (or average of the two center values for even count)
  if (n % 2 == 1)
    return arr[n / 2];
  else
    return (arr[(n - 1) / 2] + arr[n / 2]) / 2.0;
}

//--------------------------------------------------------------------
// Helper function to read a batch from a scale and return the median weight.
// If the scale is inactive or no valid sample is obtained, returns 0.
float getMedianReading(HX711 &scale, float cal, int numSamples, bool isActive) {
  if (!isActive) return 0;

  float validSamples[NUM_SAMPLES];
  int count = 0;
  
  for (int i = 0; i < numSamples; i++) {
    if (scale.is_ready()) {
      long raw = scale.read();
      float weight = (raw - scale.get_offset()) * cal;
      validSamples[count++] = weight;
    }
    delay(20);  // Reduced delay between individual readings in the batch
  }
  
  if (count == 0) {
    return 0;
  } else {
    return computeMedian(validSamples, count);
  }
}

//--------------------------------------------------------------------
void setup() {
  Serial.begin(115200);  // Make sure your serial monitor in PlatformIO is set to this baud rate

  // Initialize scales with their pin configurations
  scale1.begin(SCALE1_DT_PIN, SCALE1_SCK_PIN);
  scale2.begin(SCALE2_DT_PIN, SCALE2_SCK_PIN);
  scale3.begin(SCALE3_DT_PIN, SCALE3_SCK_PIN);
  scale4.begin(SCALE4_DT_PIN, SCALE4_SCK_PIN);

  // Tare each scale with timeout; record their active status.
  active1 = tareScale(scale1, "Scale 1");
  active2 = tareScale(scale2, "Scale 2");
  active3 = tareScale(scale3, "Scale 3");
  active4 = tareScale(scale4, "Scale 4");

  Serial.println("\nTare process complete. Active scales will be read; inactive scales will be 0.");

  // (Optional) Informative line (will be ignored by the serial plotter)
  Serial.println("Starting data output for Serial Plotter...");
}

//--------------------------------------------------------------------
void loop() {
  // Get median readings from each scale (or 0 if not active or not responding)
  float current1 = getMedianReading(scale1, calibration_factor1, NUM_SAMPLES, active1);
  float current2 = getMedianReading(scale2, calibration_factor2, NUM_SAMPLES, active2);
  float current3 = getMedianReading(scale3, calibration_factor3, NUM_SAMPLES, active3);
  float current4 = getMedianReading(scale4, calibration_factor4, NUM_SAMPLES, active4);

  // Apply a low-pass filter (stability filtering) for each scale.
  if (active1 && current1 != 0) {
    lastStable1 = (1 - smoothingFactor) * lastStable1 + smoothingFactor * current1;
  } else {
    lastStable1 = 0;
  }
  
  if (active2 && current2 != 0) {
    lastStable2 = (1 - smoothingFactor) * lastStable2 + smoothingFactor * current2;
  } else {
    lastStable2 = 0;
  }
  
  if (active3 && current3 != 0) {
    lastStable3 = (1 - smoothingFactor) * lastStable3 + smoothingFactor * current3;
  } else {
    lastStable3 = 0;
  }
  
  if (active4 && current4 != 0) {
    lastStable4 = (1 - smoothingFactor) * lastStable4 + smoothingFactor * current4;
  } else {
    lastStable4 = 0;
  }

  // Output in the format required by the VS Code Serial Plotter.
  // The line starts with ">" and then contains variable:value pairs.
  Serial.print(">");
  Serial.print("scale1:"); Serial.print(lastStable1, 2);
  Serial.print(",");
  Serial.print("scale2:"); Serial.print(lastStable2, 2);
  Serial.print(",");
  Serial.print("scale3:"); Serial.print(lastStable3, 2);
  Serial.print(",");
  Serial.print("scale4:"); Serial.print(lastStable4, 2);
  Serial.println();  // This prints the newline (\r\n)

  // Minimal delay for fast, responsive output.
  delay(20);
}
