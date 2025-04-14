#ifndef MULTISCALESENSORS_H
#define MULTISCALESENSORS_H

#include "Arduino.h"
#include "HX711.h"

class MultiScaleSensors {
  public:
    MultiScaleSensors();

    // Call this in setup() to initialize and tare the sensors.
    // It returns true if at least one sensor is active.
    bool BeginSensors();

    // Call this in your loop() to get a sensor reading.
    // sensorID should be 1, 2, 3, or 4.
    // It returns the current filtered reading (or 0 if the sensor is inactive).
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

    // Number of samples to average per batch.
    static const int NUM_SAMPLES = 8;

    // Smoothing parameter for the low-pass filter (0 < smoothingFactor <= 1).
    const float smoothingFactor = 0.6;

    // Variables to store the last stable (filtered) values.
    float lastStable1;
    float lastStable2;
    float lastStable3;
    float lastStable4;

    // Active flag for each sensor.
    bool active1;
    bool active2;
    bool active3;
    bool active4;

    // Tare timeout period in milliseconds.
    const long TARE_TIMEOUT = 5000;

    // Private helper functions.
    bool tareScale(HX711 &scale, const char* scaleName);
    float computeMedian(float arr[], int n);
    float getMedianReading(HX711 &scale, float cal, int numSamples, bool isActive);
};

#endif
