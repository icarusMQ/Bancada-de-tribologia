#ifndef MULTISCALESENSORS_H
#define MULTISCALESENSORS_H

#include "Arduino.h"
#include "HX711.h"

// This class manages multiple HX711-based load cells (scales).
// It provides functionality to initialize, tare, and read filtered weight values
// from up to four scales.
class MultiScaleSensors {
  public:
    // Constructor: Initializes the class variables.
    MultiScaleSensors();

    // Initializes and tares all the sensors.
    // This function should be called in the `setup()` function of your Arduino sketch.
    // It returns `true` if at least one sensor is active after taring.
    bool BeginSensors();

    // Reads the current filtered weight value from a specific sensor.
    // This function should be called in the `loop()` function of your Arduino sketch.
    // Parameters:
    // - sensorID: The ID of the sensor to read (1, 2, 3, or 4).
    // Returns:
    // - The current filtered weight value, or 0 if the sensor is inactive.
    float ReadSensor(uint8_t sensorID);

  private:
    // HX711 objects for the four scales.
    // These objects handle communication with the HX711 modules connected to the scales.
    HX711 scale1;
    HX711 scale2;
    HX711 scale3;
    HX711 scale4;

    // Pin definitions for the data (DT) and clock (SCK) pins of each scale.
    // These pins must be connected to the corresponding pins on the HX711 modules.
    static const uint8_t SCALE1_DT_PIN = 2;
    static const uint8_t SCALE1_SCK_PIN = 3;
    static const uint8_t SCALE2_DT_PIN = 4;
    static const uint8_t SCALE2_SCK_PIN = 6;
    static const uint8_t SCALE3_DT_PIN = 10;
    static const uint8_t SCALE3_SCK_PIN = 7;
    static const uint8_t SCALE4_DT_PIN = 12;
    static const uint8_t SCALE4_SCK_PIN = 11;

    // Calibration factors for each scale.
    // These factors are used to convert raw HX711 readings into weight values.
    float calibration_factor1 = 0.02808;
    float calibration_factor2 = 0.0342;
    float calibration_factor3 = -0.05671;
    float calibration_factor4 = -0.0614;

    // Number of samples to average per batch when reading from a scale.
    // This determines how many readings are taken to compute the median weight.
    static const int NUM_SAMPLES = 8;

    // Smoothing factor for the low-pass filter applied to the weight readings.
    // This helps stabilize the readings by reducing noise.
    // The value should be between 0 (no smoothing) and 1 (maximum smoothing).
    const float smoothingFactor = 0.6;

    // Variables to store the last stable (filtered) weight values for each scale.
    // These are updated after each valid reading.
    float lastStable1;
    float lastStable2;
    float lastStable3;
    float lastStable4;

    // Flags to indicate whether each scale is active.
    // A scale is considered active if it successfully tared during initialization.
    bool active1;
    bool active2;
    bool active3;
    bool active4;

    // Timeout period (in milliseconds) for the tare operation.
    // If a scale does not respond within this time, it is marked as inactive.
    const unsigned long TARE_TIMEOUT = 5000UL;

    // Private helper functions:

    // Tares a specific scale and checks if it is ready.
    // Parameters:
    // - scale: The HX711 object representing the scale.
    // - scaleName: A string representing the name of the scale (e.g., "Scale 1").
    // Returns:
    // - `true` if the scale was successfully tared, `false` otherwise.
    bool tareScale(HX711 &scale, const char* scaleName);

    // Computes the median value of an array of floats.
    // Parameters:
    // - arr: The array of float values.
    // - n: The number of elements in the array.
    // Returns:
    // - The median value of the array.
    float computeMedian(float arr[], int n);

    // Reads multiple samples from a scale, calculates the weight for each sample,
    // and returns the median weight.
    // Parameters:
    // - scale: The HX711 object representing the scale.
    // - cal: The calibration factor for the scale.
    // - numSamples: The number of samples to read.
    // - isActive: Whether the scale is active.
    // Returns:
    // - The median weight of the valid samples, or 0 if no valid samples are collected.
    float getMedianReading(HX711 &scale, float cal, int numSamples, bool isActive);
};

#endif
