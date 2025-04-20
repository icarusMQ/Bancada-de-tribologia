
## 📋 Overview

This firmware controls an automated tribology test bench designed for measuring friction and wear characteristics of material pairs under controlled conditions. The system uses an Arduino-based controller with multiple sensors and actuators, all managed by a real-time operating system (FreeRTOS) for reliability and precise timing.

## ✨ Features

- **Real-time operation** using FreeRTOS for deterministic timing and multitasking
- **PID control** for precise load application during experiments
- **Nextion HMI integration** for user-friendly control and real-time data visualization
- **Centralized sensor data management** with advanced filtering techniques
- **Multiple experiment types** with configurable parameters
- **Watchdog timer** integration for system reliability
- **Multiple load cell integration** for force measurement in multiple axes
- **Precision stepper motor control** for load application and sample movement

## 🧰 Hardware Components

- **Controller Board**: Arduino (compatible with Uno or Mega)
- **HMI**: Nextion touch display (configurable for different sizes)
- **Sensors**:
  - 4× Load cells with HX711 amplifiers for force measurement
  - Auxiliary sensors (temperature, vibration) optional
- **Actuators**:
  - 5× Stepper motors for precise motion control:
    - 2× For lubricant/abrasive flow control
    - 2× For rotation/reciprocation
    - 1× For normal load application

## 🔌 Pin Assignments

### Load Cells (HX711)
| Sensor | DT Pin | SCK Pin |
|--------|--------|---------|
| Scale 1 | 2 | 3 |
| Scale 2 | 4 | 6 |
| Scale 3 | 10 | 7 |
| Scale 4 | 12 | 11 |

### Stepper Motors
| Motor | STEP Pin | DIR Pin | Function |
|-------|----------|---------|----------|
| Motor 1 | 2 | 3 | Flow Control 1 |
| Motor 2 | 4 | 5 | Flow Control 2 |
| Motor 3 | 6 | 7 | Rotation 1 |
| Motor 4 | 8 | 9 | Rotation 2 |
| Motor 5 | 10 | 11 | Load Application |

### Nextion Display
- Uses Serial/Serial1/Serial2 (configurable)

## 🛠️ Setup and Configuration

### Prerequisites
- PlatformIO IDE (recommended) or Arduino IDE
- Required libraries (automatically installed with PlatformIO):
  - FreeRTOS for Arduino
  - HX711 for load cell integration
  - AccelStepper for motor control
  - Nextion library for HMI

## 🧠 System Architecture

The firmware uses a task-based architecture:
- **InitTask**: Initializes hardware components
- **CommandHandlerTask**: Processes user inputs
- **LoadControlTask**: PID control for load application
- **SensorReadTask**: Reads and filters sensor data
- **StatusReportTask**: Reports system status
- **NextionTask**: Manages the HMI interface

## 👨‍💻 Development

### Project Structure
```
├── src/
│   ├── main.cpp             # Main application code
│   ├── MultiScaleSensors.h  # Load cell sensor management
│   ├── MultiScaleSensors.cpp
│   ├── StepperMotorController.h  # Motor control
│   └── StepperMotorController.cpp
├── include/                 # Additional header files
├── lib/                     # Project-specific libraries
├── platformio.ini           # PlatformIO configuration
└── README.md                # This documentation
```

### Extending the System
- Add new experiment types in `main.cpp`
- Modify PID control parameters for different materials
- Add data logging capabilities via SD card
- Integrate additional sensors as needed

## 🙏 Acknowledgments

- UFSC Department of Mechanical Engineering
- Tribology Research Group
- FreeRTOS community
- Arduino community

---

*Developed at the Federal University of Santa Maria (UFSM), Brazil*
