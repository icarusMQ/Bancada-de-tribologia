#ifndef MULTISCALESENSORS_H
#define MULTISCALESENSORS_H

#include "Arduino.h"
#include "HX711.h"

// Improved class to manage multiple load cells using the official HX711 library
// but optimized for low memory usage
class MultiScaleSensors {
  public:
    // Constructor
    MultiScaleSensors();
    
    // Initialize the sensors (must be called once at startup)
    bool beginSensors();
    
    // Read a specific sensor value (1-4) in Newtons
    float readSensor(uint8_t sensorID);
    
    // Check if a sensor is active
    bool isSensorActive(uint8_t sensorID);
    
    // Tare a specific sensor
    bool tareSensor(uint8_t sensorID);
    
    // Tare all sensors
    bool tareAllSensors();
    
  private:
    // Single HX711 instance that's reused for all sensors
    // This significantly reduces RAM usage
    HX711 scale;
    
    // Bit mask to track active sensors - uses 1 byte instead of 4 booleans
    uint8_t activeSensors;
    
    // Offsets for each sensor
    long offsetValues[4];
    
    // Last valid reading (to filter noise)
    float lastReadings[4];
    
    // Pin definitions for the data and clock pins
    static const uint8_t DOUT_PINS[4];
    static const uint8_t SCK_PINS[4];
    
    // Calibration factors (one per scale)
    static const float CALIBRATION_FACTORS[4];
    
    // Conversion constant
    static const float GRAMS_TO_NEWTONS;
    
    // Smoothing factor for readings
    static const float SMOOTHING_FACTOR;
    
    // Tare timeout in milliseconds
    static const unsigned long TARE_TIMEOUT = 4000;
    
    // Helper: configure scale for a specific sensor
    void setupScale(uint8_t sensorIndex);
    
    // Helper: get median reading from a sensor
    float getMedianReading(uint8_t sensorIndex, uint8_t samples);
};

#endif
