#include "MultiScaleSensors.h"

// Define pin connections for all 4 scales
const uint8_t MultiScaleSensors::DOUT_PINS[4] = {2, 4, 10, 12};  // Data pins
const uint8_t MultiScaleSensors::SCK_PINS[4] = {3, 6, 7, 11};    // Clock pins

// Calibration factors for each scale (grams per raw unit)
const float MultiScaleSensors::CALIBRATION_FACTORS[4] = {0.02808, 0.0342, -0.05671, -0.0614};

// Constants
const float MultiScaleSensors::GRAMS_TO_NEWTONS = 0.00980665;  // Convert from grams to Newtons
const float MultiScaleSensors::SMOOTHING_FACTOR = 0.7;         // Higher = more responsive to new readings

// Constructor
MultiScaleSensors::MultiScaleSensors() : activeSensors(0) {
  // Initialize arrays
  for (uint8_t i = 0; i < 4; i++) {
    offsetValues[i] = 0;
    lastReadings[i] = 0.0f;
  }
}

// Configure the scale for a specific sensor
void MultiScaleSensors::setupScale(uint8_t sensorIndex) {
  // Disconnect from previous pins
  scale.power_down();
  
  // Setup scale with appropriate pins
  scale.begin(DOUT_PINS[sensorIndex], SCK_PINS[sensorIndex]);
  
  // Set the previously saved offset
  scale.set_offset(offsetValues[sensorIndex]);
  
  // Use a fixed gain (128 is common)
  scale.set_gain(128);
  
  // Power up the scale
  scale.power_up();
}

// Initialize all sensors
bool MultiScaleSensors::beginSensors() {
  // Try to initialize each sensor
  bool anyActive = false;
  
  for (uint8_t i = 0; i < 4; i++) {
    setupScale(i);
    
    // Check if this sensor is responding
    if (scale.wait_ready_timeout(500)) {
      // Mark sensor as active
      activeSensors |= (1 << i);
      anyActive = true;
      
      // Warm up the sensor with a few readings
      for (uint8_t j = 0; j < 3; j++) {
        scale.read();
        delay(10);
      }
    }
  }
  
  return anyActive;
}

// Tare a specific sensor
bool MultiScaleSensors::tareSensor(uint8_t sensorID) {
  if (sensorID < 1 || sensorID > 4) {
    return false;
  }
  
  uint8_t index = sensorID - 1;
  
  // Configure the scale for this sensor
  setupScale(index);
  
  // Try to tare
  bool success = scale.wait_ready_timeout(TARE_TIMEOUT);
  if (success) {
    scale.tare();
    offsetValues[index] = scale.get_offset();
    
    // Make sure to mark as active if taring succeeded
    activeSensors |= (1 << index);
  } else {
    // Mark as inactive if failed to respond
    activeSensors &= ~(1 << index);
  }
  
  return success;
}

// Tare all sensors
bool MultiScaleSensors::tareAllSensors() {
  Serial.println(F("Taring all sensors..."));
  
  bool anySuccess = false;
  
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F("Taring sensor "));
    Serial.print(i+1);
    Serial.print(F("..."));
    
    bool success = tareSensor(i+1);
    
    if (success) {
      Serial.println(F("OK"));
      anySuccess = true;
    } else {
      Serial.println(F("Failed or not connected"));
    }
    
    delay(100); // Short delay between sensors
  }
  
  return anySuccess;
}

// Check if a sensor is active
bool MultiScaleSensors::isSensorActive(uint8_t sensorID) {
  if (sensorID < 1 || sensorID > 4) {
    return false;
  }
  
  uint8_t index = sensorID - 1;
  return (activeSensors & (1 << index)) != 0;
}

// Get median reading (helper function)
float MultiScaleSensors::getMedianReading(uint8_t sensorIndex, uint8_t samples) {
  // Only use 3 samples max to save memory
  const uint8_t MAX_SAMPLES = 3;
  uint8_t actualSamples = min(samples, MAX_SAMPLES);
  
  float readings[MAX_SAMPLES];
  uint8_t validCount = 0;
  
  for (uint8_t i = 0; i < actualSamples; i++) {
    if (scale.wait_ready_retry(2, 50)) {
      readings[validCount++] = scale.get_units(1);
    }
  }
  
  if (validCount == 0) {
    return 0.0;
  }
  
  // For just one valid reading, return it directly
  if (validCount == 1) {
    return readings[0];
  }
  
  // Simple sort for small array
  for (uint8_t i = 0; i < validCount - 1; i++) {
    for (uint8_t j = 0; j < validCount - i - 1; j++) {
      if (readings[j] > readings[j+1]) {
        float temp = readings[j];
        readings[j] = readings[j+1];
        readings[j+1] = temp;
      }
    }
  }
  
  // Return median
  return readings[validCount / 2];
}

// Read a specific sensor
float MultiScaleSensors::readSensor(uint8_t sensorID) {
  if (sensorID < 1 || sensorID > 4) {
    return 0.0f;
  }
  
  uint8_t index = sensorID - 1;
  
  // Return 0 if sensor is inactive
  if (!(activeSensors & (1 << index))) {
    return 0.0f;
  }
  
  // Configure the scale for this sensor
  setupScale(index);
  
  // Apply calibration factor
  scale.set_scale(CALIBRATION_FACTORS[index]);
  
  // Get the median of multiple readings
  float currentReading = getMedianReading(index, 3);
  
  // Apply smoothing filter
  if (lastReadings[index] != 0.0f) {
    lastReadings[index] = SMOOTHING_FACTOR * currentReading + 
                          (1.0f - SMOOTHING_FACTOR) * lastReadings[index];
  } else {
    lastReadings[index] = currentReading;
  }
  
  // Convert to Newtons
  return lastReadings[index] * GRAMS_TO_NEWTONS;
}