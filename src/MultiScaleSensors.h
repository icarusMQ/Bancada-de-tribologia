#ifndef MULTISCALESENSORS_H
#define MULTISCALESENSORS_H

#include "Arduino.h"
#include "HX711.h"

// This class manages multiple HX711-based load cells (scales).
// It provides functionality to initialize, tare, and read filtered weight values
// from up to four scales with improved error handling and robustness.
class MultiScaleSensors {
  public:
    // Status codes for sensor operations
    static const uint8_t SENSOR_OK = 0;
    static const uint8_t SENSOR_ERROR = 1;
    static const uint8_t SENSOR_TIMEOUT = 2;
    static const uint8_t SENSOR_NOT_CONNECTED = 3;

    // Constructor: Initializes the class variables.
    MultiScaleSensors();

    // Initializes and tares all the sensors.
    // This function should be called in the `setup()` function of your Arduino sketch.
    // It returns `true` if at least one sensor is active after taring.
    bool beginSensors();

    // Reads the current filtered weight value from a specific sensor.
    // Parameters:
    // - sensorID: The ID of the sensor to read (1, 2, 3, or 4).
    // Returns:
    // - The current filtered weight value in Newtons, or 0 if the sensor is inactive.
    float readSensor(uint8_t sensorID);

    // Check if a specific sensor is active
    // Parameters:
    // - sensorID: The ID of the sensor to check (1, 2, 3, or 4).
    bool isSensorActive(uint8_t sensorID);
    
    // Tare a specific sensor
    // Parameters:
    // - sensorID: The ID of the sensor to tare (1, 2, 3, or 4).
    bool tareSensor(uint8_t sensorID);

    // Tare all sensors
    // This function will tare all connected sensors
    bool tareAllSensors();

    // Get the last error code for a specific sensor
    // Parameters:
    // - sensorID: The ID of the sensor (1, 2, 3, or 4).
    uint8_t getSensorErrorCode(uint8_t sensorID);

    // Calibrate a specific sensor with a known weight
    // Parameters:
    // - sensorID: The ID of the sensor to calibrate (1, 2, 3, or 4).
    // - knownWeightGrams: The known weight in grams used for calibration.
    bool calibrateSensor(uint8_t sensorID, float knownWeightGrams);

    // Reset all sensors
    bool resetSensors();

    // Power down specific sensor to save power
    void powerDownSensor(uint8_t sensorID);

    // Power down all sensors
    void powerDownAllSensors();

    // Power up a specific sensor
    void powerUpSensor(uint8_t sensorID);

    // Set filter level (0 = none, 1 = light, 2 = medium, 3 = heavy)
    void setFilterLevel(uint8_t level);

  private:
    // Individual HX711 objects for the four scales
    HX711 scale1;
    HX711 scale2;
    HX711 scale3;
    HX711 scale4;

    // Pin definitions for the data (DT) and clock (SCK) pins of each scale.
    static const uint8_t SCALE1_DT_PIN = 2;
    static const uint8_t SCALE1_SCK_PIN = 3;
    static const uint8_t SCALE2_DT_PIN = 4;
    static const uint8_t SCALE2_SCK_PIN = 6;
    static const uint8_t SCALE3_DT_PIN = 10;
    static const uint8_t SCALE3_SCK_PIN = 7;
    static const uint8_t SCALE4_DT_PIN = 12;
    static const uint8_t SCALE4_SCK_PIN = 11;

    // Arrays to store the data and clock pins for each sensor
    static const uint8_t DOUT_PINS[4];
    static const uint8_t SCK_PINS[4];

    // Calibration factors for each scale
    float calibrationFactors[4];

    // Default calibration factors
    static const float DEFAULT_CALIBRATION_FACTORS[4];

    // Variables to store the last stable weight values for each scale.
    float lastReadings[4];

    // Error status for each sensor
    uint8_t sensorErrors[4];

    // Bit mask to track active sensors - uses 1 byte instead of 4 booleans
    uint8_t activeSensors;

    // Current filtering level
    uint8_t filterLevel;

    // Smoothing factors for different filter levels
    static const float SMOOTHING_FACTORS[4];

    // Samples per reading for different filter levels
    static const uint8_t SAMPLES_PER_READING[4];

    // Conversion constant from grams to Newtons
    static const float GRAMS_TO_NEWTONS;

    // Timeout period for tare operation in milliseconds
    static const unsigned long TARE_TIMEOUT = 5000;

    // Timeout period for calibration operation in milliseconds
    static const unsigned long CALIBRATION_TIMEOUT = 10000;

    // Maximum number of samples for median calculation
    static const uint8_t MAX_SAMPLES = 8;

    // Helper: Tare a scale with timeout.
    bool tareScale(HX711 &scale, uint8_t sensorIndex);

    // Helper: Get median reading from a sensor
    float getMedianReading(HX711 &scale, float calibrationFactor, uint8_t samples, bool isActive, uint8_t sensorIndex);

    // Helper: Compute the median of an array of floats
    float computeMedian(float arr[], int n);

    // Helper: Check if sensor ID is valid and convert to index
    bool isValidSensorID(uint8_t sensorID, uint8_t* index = NULL);

    // Helper: Get HX711 reference by index
    HX711& getScaleByIndex(uint8_t index);
};

#endif
