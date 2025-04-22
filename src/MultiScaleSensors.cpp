#include "MultiScaleSensors.h"

// Define pin connections for all 4 scales
const uint8_t MultiScaleSensors::DOUT_PINS[4] = {SCALE1_DT_PIN, SCALE2_DT_PIN, SCALE3_DT_PIN, SCALE4_DT_PIN};
const uint8_t MultiScaleSensors::SCK_PINS[4] = {SCALE1_SCK_PIN, SCALE2_SCK_PIN, SCALE3_SCK_PIN, SCALE4_SCK_PIN};

// Default calibration factors for each scale (grams per raw unit)
const float MultiScaleSensors::DEFAULT_CALIBRATION_FACTORS[4] = {0.02808, 0.0342, -0.05671, -0.0614};

// Smoothing factors for different filter levels (0=None, 1=Light, 2=Medium, 3=Heavy)
const float MultiScaleSensors::SMOOTHING_FACTORS[4] = {0.0, 0.3, 0.6, 0.8};

// Samples per reading for different filter levels
const uint8_t MultiScaleSensors::SAMPLES_PER_READING[4] = {1, 3, 5, 8};

// Conversion constant
const float MultiScaleSensors::GRAMS_TO_NEWTONS = 0.00980665;  // Convert from grams to Newtons

// Constructor
MultiScaleSensors::MultiScaleSensors() : activeSensors(0), filterLevel(2) {
  // Initialize arrays
  for (uint8_t i = 0; i < 4; i++) {
    lastReadings[i] = 0.0f;
    sensorErrors[i] = SENSOR_NOT_CONNECTED;
    calibrationFactors[i] = DEFAULT_CALIBRATION_FACTORS[i];
  }
}

// Helper to validate sensor ID and optionally convert to index
bool MultiScaleSensors::isValidSensorID(uint8_t sensorID, uint8_t* index) {
  if (sensorID < 1 || sensorID > 4) {
    return false;
  }
  
  if (index != NULL) {
    *index = sensorID - 1;
  }
  
  return true;
}

// Helper to get HX711 reference by index
HX711& MultiScaleSensors::getScaleByIndex(uint8_t index) {
  switch (index) {
    case 0: return scale1;
    case 1: return scale2;
    case 2: return scale3;
    case 3: return scale4;
    default: return scale1; // Should never happen if isValidSensorID is used
  }
}

// Helper: Tare a scale with timeout
bool MultiScaleSensors::tareScale(HX711 &scale, uint8_t sensorIndex) {
  Serial.print(F("Taring sensor "));
  Serial.print(sensorIndex + 1);
  Serial.println(F("... Remove any load."));
  
  // Give a brief moment for user to remove load if needed
  delay(5000);
  
  // Try to tare with timeout
  unsigned long startTime = millis();
  
  // Wait for sensor to be ready
  while (!scale.is_ready() && (millis() - startTime < TARE_TIMEOUT)) {
    delay(50);
  }
  
  if (scale.is_ready()) {
    scale.tare(10);  // Use 5 readings for more accurate tare
    
    // Mark as active
    activeSensors |= (1 << sensorIndex);
    sensorErrors[sensorIndex] = SENSOR_OK;
    
    Serial.print(F("Sensor "));
    Serial.print(sensorIndex + 1);
    Serial.println(F(" tared successfully."));
    return true;
  } else {
    // Mark as inactive
    activeSensors &= ~(1 << sensorIndex);
    sensorErrors[sensorIndex] = SENSOR_TIMEOUT;
    
    Serial.print(F("Sensor "));
    Serial.print(sensorIndex + 1);
    Serial.println(F(" not responding. Marked as inactive."));
    return false;
  }
}

// Helper: Compute the median of an array of floats
float MultiScaleSensors::computeMedian(float arr[], int n) {
  // Sort the array using insertion sort (efficient for small arrays)
  for (int i = 1; i < n; i++) {
    float key = arr[i];
    int j = i - 1;
    
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    
    arr[j + 1] = key;
  }
  
  // Return the median value
  if (n % 2 == 1) {
    return arr[n / 2];  // Odd number of elements
  } else {
    return (arr[(n - 1) / 2] + arr[n / 2]) / 2.0;  // Even number of elements
  }
}

// Helper: Get median reading from a sensor
float MultiScaleSensors::getMedianReading(HX711 &scale, float calibrationFactor, uint8_t samples, 
                                         bool isActive, uint8_t sensorIndex) {
  if (!isActive) {
    return 0.0f;  // Return 0 if sensor is inactive
  }
  
  // Cap the number of samples to avoid memory issues
  uint8_t actualSamples = min(samples, MAX_SAMPLES);
  
  float validSamples[MAX_SAMPLES];  // Array to store valid samples
  uint8_t count = 0;  // Counter for valid samples
  
  // Get samples with timeout protection
  unsigned long sampleTimeout = 50 * actualSamples; // Maximum time to spend collecting samples
  unsigned long startTime = millis();
  
  while (count < actualSamples && (millis() - startTime < sampleTimeout)) {
    if (scale.is_ready()) {
      // Apply calibration factor to convert raw reading to weight
      scale.set_scale(calibrationFactor);
      validSamples[count++] = scale.get_units(1);
    }
    delay(10);  // Short delay between readings
  }
  
  if (count == 0) {
    sensorErrors[sensorIndex] = SENSOR_TIMEOUT;
    return 0.0f;  // Return 0 if no valid samples were collected
  }
  
  sensorErrors[sensorIndex] = SENSOR_OK;
  return computeMedian(validSamples, count);  // Return the median of the valid samples
}

// Public: Initialize all sensors
bool MultiScaleSensors::beginSensors() {
  bool anyActive = false;
  
  Serial.println(F("Initializing sensors..."));
  
  // Begin scale instances with pin assignments
  scale1.begin(SCALE1_DT_PIN, SCALE1_SCK_PIN);
  scale2.begin(SCALE2_DT_PIN, SCALE2_SCK_PIN);
  scale3.begin(SCALE3_DT_PIN, SCALE3_SCK_PIN);
  scale4.begin(SCALE4_DT_PIN, SCALE4_SCK_PIN);
  
  // Set default gain for all scales
  scale1.set_gain(128);
  scale2.set_gain(128);
  scale3.set_gain(128);
  scale4.set_gain(128);
  
  // Attempt to read from each sensor to see if it's responsive
  for (uint8_t i = 0; i < 4; i++) {
    HX711 &scale = getScaleByIndex(i);
    
    // Check if this sensor is responding
    if (scale.wait_ready_timeout(500)) {
      // Mark sensor as active
      activeSensors |= (1 << i);
      sensorErrors[i] = SENSOR_OK;
      anyActive = true;
      
      Serial.print(F("Sensor "));
      Serial.print(i+1);
      Serial.println(F(" detected"));
      
      // Warm up the sensor with a few readings
      for (uint8_t j = 0; j < 3; j++) {
        scale.read();
        delay(10);
      }
    } else {
      sensorErrors[i] = SENSOR_NOT_CONNECTED;
      Serial.print(F("Sensor "));
      Serial.print(i+1);
      Serial.println(F(" not detected"));
    }
  }
  
  return anyActive;
}

// Public: Tare a specific sensor
bool MultiScaleSensors::tareSensor(uint8_t sensorID) {
  uint8_t index;
  if (!isValidSensorID(sensorID, &index)) {
    Serial.println(F("Invalid sensor ID. Use 1, 2, 3, or 4."));
    return false;
  }
  
  HX711 &scale = getScaleByIndex(index);
  return tareScale(scale, index);
}

// Public: Tare all sensors
bool MultiScaleSensors::tareAllSensors() {
  Serial.println(F("Taring all sensors..."));
  
  bool anySuccess = false;
  
  for (uint8_t i = 0; i < 4; i++) {
    HX711 &scale = getScaleByIndex(i);
    bool success = tareScale(scale, i);
    
    if (success) {
      anySuccess = true;
    }
    
    delay(100); // Short delay between sensors
  }
  
  Serial.println(F("Tare process complete."));
  return anySuccess;
}

// Public: Check if a sensor is active
bool MultiScaleSensors::isSensorActive(uint8_t sensorID) {
  uint8_t index;
  if (!isValidSensorID(sensorID, &index)) {
    return false;
  }
  
  return (activeSensors & (1 << index)) != 0;
}

// Public: Read a specific sensor
float MultiScaleSensors::readSensor(uint8_t sensorID) {
  uint8_t index;
  if (!isValidSensorID(sensorID, &index)) {
    return 0.0f;
  }
  
  // Return 0 if sensor is inactive
  if (!(activeSensors & (1 << index))) {
    return 0.0f;
  }
  
  HX711 &scale = getScaleByIndex(index);
  
  // Get smoothing factor and number of samples based on filter level
  float smoothingFactor = SMOOTHING_FACTORS[filterLevel];
  uint8_t samples = SAMPLES_PER_READING[filterLevel];
  
  // Get median reading from the sensor
  float currentReading = getMedianReading(scale, calibrationFactors[index], samples, 
                                          true, index);
  
  // Apply smoothing filter if we have historical data
  if (lastReadings[index] != 0.0f && smoothingFactor > 0.0f) {
    lastReadings[index] = (1.0f - smoothingFactor) * lastReadings[index] + 
                           smoothingFactor * currentReading;
  } else {
    lastReadings[index] = currentReading;
  }
  
  // Convert to Newtons
  return lastReadings[index] * GRAMS_TO_NEWTONS;
}

// Public: Get the last error code for a specific sensor
uint8_t MultiScaleSensors::getSensorErrorCode(uint8_t sensorID) {
  uint8_t index;
  if (!isValidSensorID(sensorID, &index)) {
    return SENSOR_ERROR;
  }
  
  return sensorErrors[index];
}

// Public: Calibrate a specific sensor with a known weight
bool MultiScaleSensors::calibrateSensor(uint8_t sensorID, float knownWeightGrams) {
  uint8_t index;
  if (!isValidSensorID(sensorID, &index) || knownWeightGrams <= 0) {
    return false;
  }
  
  HX711 &scale = getScaleByIndex(index);
  
  // First tare the sensor to ensure we start from zero
  if (!tareScale(scale, index)) {
    return false;
  }
  
  Serial.print(F("Place "));
  Serial.print(knownWeightGrams);
  Serial.println(F("g on sensor and wait..."));
  
  // Wait for user to place weight
  delay(3000);
  
  // Get average of multiple readings for better accuracy
  const uint8_t CAL_SAMPLES = 10;
  float totalValue = 0.0f;
  uint8_t validSamples = 0;
  
  Serial.println(F("Calibrating..."));
  
  for (uint8_t i = 0; i < CAL_SAMPLES; i++) {
    if (scale.wait_ready_timeout(500)) {
      // Use a temp scale of 1.0 to get raw units
      scale.set_scale(1.0);
      totalValue += scale.get_units(1);
      validSamples++;
      Serial.print(F("."));
    }
    delay(100);
  }
  
  Serial.println();
  
  if (validSamples == 0) {
    sensorErrors[index] = SENSOR_TIMEOUT;
    Serial.println(F("Calibration failed - no readings received."));
    return false;
  }
  
  // Calculate new calibration factor
  float averageValue = totalValue / validSamples;
  
  // Avoid division by zero
  if (abs(averageValue) < 0.000001) {
    Serial.println(F("Calibration failed - zero reading."));
    return false;
  }
  
  // New calibration = known weight / average raw reading
  calibrationFactors[index] = knownWeightGrams / averageValue;
  
  // Set the new calibration factor
  scale.set_scale(calibrationFactors[index]);
  
  Serial.print(F("Calibration factor set to: "));
  Serial.println(calibrationFactors[index], 6);
  
  return true;
}

// Public: Reset all sensors and reinitialize
bool MultiScaleSensors::resetSensors() {
  // Power down all sensors first
  powerDownAllSensors();
  
  // Reset all tracking variables
  activeSensors = 0;
  
  for (uint8_t i = 0; i < 4; i++) {
    lastReadings[i] = 0.0f;
    sensorErrors[i] = SENSOR_NOT_CONNECTED;
    // Don't reset calibration factors - these are valuable
  }
  
  delay(100);  // Give time for hardware to reset
  
  // Reinitialize all sensors
  return beginSensors();
}

// Public: Power down a specific sensor
void MultiScaleSensors::powerDownSensor(uint8_t sensorID) {
  uint8_t index;
  if (!isValidSensorID(sensorID, &index)) {
    return;
  }
  
  HX711 &scale = getScaleByIndex(index);
  scale.power_down();
  
  // Mark as inactive when powered down
  activeSensors &= ~(1 << index);
}

// Public: Power down all sensors
void MultiScaleSensors::powerDownAllSensors() {
  for (uint8_t i = 0; i < 4; i++) {
    HX711 &scale = getScaleByIndex(i);
    scale.power_down();
  }
  
  // Mark all as inactive
  activeSensors = 0;
}

// Public: Power up a specific sensor
void MultiScaleSensors::powerUpSensor(uint8_t sensorID) {
  uint8_t index;
  if (!isValidSensorID(sensorID, &index)) {
    return;
  }
  
  HX711 &scale = getScaleByIndex(index);
  scale.power_up();
  
  // Check if responsive
  if (scale.wait_ready_timeout(1000)) {
    activeSensors |= (1 << index);
    sensorErrors[index] = SENSOR_OK;
  } else {
    sensorErrors[index] = SENSOR_TIMEOUT;
  }
}

// Public: Set filter level
void MultiScaleSensors::setFilterLevel(uint8_t level) {
  // Constrain to valid range (0-3)
  filterLevel = min(level, 3);
}