#ifndef MULTISCALESENSORS_H
#define MULTISCALESENSORS_H

#include "Arduino.h"
#include "HX711.h"
#include <Arduino_FreeRTOS.h>  // Retain include if you use RTOS functions in tasks later

// This class manages multiple HX711-based load cells.
// It provides functionality to initialize, tare, and read filtered weight values
// from up to four scales.
class MultiScaleSensors {
  public:
    // Constructor: Initializes class variables.
    MultiScaleSensors();

    // Initializes and tares all sensors.
    // Uses blocking delay() calls during initialization so that RTOS functions are avoided
    // until after the scheduler is started.
    bool BeginSensors();

    // Returns the filtered reading for the given sensor.
    float ReadSensor(uint8_t sensorID);

  private:
    // HX711 objects for the four scales.
    HX711 scale1;
    HX711 scale2;
    HX711 scale3;
    HX711 scale4;

    // Pin definitions.
    static const uint8_t SCALE1_DT_PIN = 2;
    static const uint8_t SCALE1_SCK_PIN = 3;
    static const uint8_t SCALE2_DT_PIN = 4;
    static const uint8_t SCALE2_SCK_PIN = 6;
    static const uint8_t SCALE3_DT_PIN = 10;
    static const uint8_t SCALE3_SCK_PIN = 7;
    static const uint8_t SCALE4_DT_PIN = 12;
    static const uint8_t SCALE4_SCK_PIN = 11;

    // Calibration factors.
    float calibration_factor1 = 0.02808;
    float calibration_factor2 = 0.0342;
    float calibration_factor3 = -0.05671;
    float calibration_factor4 = -0.0614;

    // Sampling and smoothing parameters.
    static const int NUM_SAMPLES = 8;
    const float smoothingFactor = 0.6;

    // Storage for filtered readings.
    float lastStable1;
    float lastStable2;
    float lastStable3;
    float lastStable4;

    // Active flags for each sensor.
    bool active1;
    bool active2;
    bool active3;
    bool active4;

    // Timeout setting for taring.
    const unsigned long TARE_TIMEOUT = 5000UL;

    // Helper methods.
    bool tareScale(HX711 &scale, const char* scaleName);
    float computeMedian(float arr[], int n);
    float getMedianReading(HX711 &scale, float cal, int numSamples, bool isActive);
};

#endif
