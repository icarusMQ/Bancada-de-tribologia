#include <MultiScaleSensors.h>
#include <StepperMotorController.h>

// Create instances of the libraries.
MultiScaleSensors sensors;
StepperMotorController motorController;

// Experiment parameters
const float TARGET_FORCE_FREE_SPHERE = 0.3; // Target force for free sphere (N)
const int PUMP_RPM = 0.5;                    // Pump speed to achieve 30 drops/min
const int ROTATION_RPM = 80;                // Rotation speed for axes (RPM)

// Experiment state
bool forceAdjusted = false;
bool pumpsStarted = false;
bool rotationStarted = false;
bool experimentRunning = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Tribology Experiment Setup");

  // Initialize sensors
  if (!sensors.BeginSensors()) {
    Serial.println("WARNING: No sensors responded properly during startup!");
    Serial.println("Check connections and restart the system.");
  } else {
    Serial.println("Sensors initialized successfully.");
    Serial.println(" Use \"start\" to start, \"stop\" to stop, or print to print scale readings.");
  }
}

void loop() {
  // Check for serial input
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    switch (command[0]) { // Use the first character of the command for the switch
      case 's': // "start"
        if (command == "start") {
          Serial.println("Starting experiment.");
          experimentRunning = true;
          forceAdjusted = false; // Reset state for new experiment
          pumpsStarted = false;
          rotationStarted = false;

        } else if (command == "stop") { // "stop"
          Serial.println("Stopping experiment.");
          experimentRunning = false;

        } else if (command == "s") { // "s" move motor 2 backwards
          Serial.println("Moving Motor 2 backward.");
          motorController.RunMotor(2, 10, LOW);
          delay(1000); // Run for 1 second
          motorController.StopMotor(2); // Stop motor after 1 second
        }
        break;

      case 'p': // "print"
        if (command == "print") {
          Serial.println("Print scale readings");
          double value1 = sensors.ReadSensor(1);
          double value2 = sensors.ReadSensor(2);
          double value3 = sensors.ReadSensor(3);
          double value4 = sensors.ReadSensor(4);
          printf("scale1: %.2f, scale2: %.2f, scale3: %.2f, scale4: %.2f\n", value1, value2, value3, value4);
        }
        break;
      
      case 'q': // "move Motor 1 forward"
        Serial.println("Moving Motor 1 forward.");
        motorController.RunMotor(1, 10, HIGH);
        delay(1000); // Run for 1 second
        motorController.StopMotor(1); // Stop motor after 1 second
        break;

      case 'a': // "reverse Motor 1"
        Serial.println("Moving Motor 1 backward.");
        motorController.RunMotor(1, 10, LOW);
        delay(1000);
        motorController.StopMotor(1);
        break;
      
      case 'w': // "move Motor 2 forward"
        Serial.println("Moving Motor 2 forward.");
        motorController.RunMotor(2, 10, HIGH);
        delay(1000); // Run for 1 second
        motorController.StopMotor(2); // Stop motor after 1 second
        break;
      
      case 'e': // "move motor 2 forward"
       Serial.println("Moving Motor 3 forward.");
        motorController.RunMotor(3, 10, HIGH);
        delay(1000); // Run for 1 second
        motorController.StopMotor(3); // Stop motor after 1 second
        break;
      
      case 'd': // "reverse Motor 3"
        Serial.println("Moving Motor 3 backward.");
        motorController.RunMotor(3, 10, LOW);
        delay(1000);
        motorController.StopMotor(3);
        break;
      
      case 'r': // "move Motor 4 forward"
        Serial.println("Moving Motor 4 forward.");
        motorController.RunMotor(4, 10, HIGH);
        delay(1000); // Run for 1 second
        motorController.StopMotor(4); // Stop motor after 1 second
        break;
      
      case 'f': // "reverse Motor 4"
        Serial.println("Moving Motor 4 backward.");
        motorController.RunMotor(4, 10, LOW);
        delay(1000);
        motorController.StopMotor(4);
        break;

      case 't': // "move Motor 5 forward"
        Serial.println("Moving Motor 5 forward.");
        motorController.RunMotor(5, 10, HIGH);
        delay(1000); // Run for 1 second
        motorController.StopMotor(5); // Stop motor after 1 second
        break;
      
      case 'g': // "reverse Motor 5"
        Serial.println("Moving Motor 5 backward.");
        motorController.RunMotor(5, 10, LOW);
        delay(1000);
        motorController.StopMotor(5);
        break;

      default:
        Serial.println("Unknown command. Use \"start\" to start, \"stop\" to stop, or print to print scale readings.");
        break;
    }
  }

  // Step 1: Adjust the free sphere force
  if (!forceAdjusted && experimentRunning) {
    float currentForce = sensors.ReadSensor(1); // Read load cell for free sphere
    if (currentForce < TARGET_FORCE_FREE_SPHERE) {
      motorController.RunMotor(5, 10, HIGH); // Adjust force motor
      Serial.print("Adjusting force... Current: ");
      Serial.print(currentForce, 2);
      Serial.println(" N");
    } else {
      motorController.StopMotor(5); // Stop motor once target force is reached
      forceAdjusted = true;
      Serial.println("Free sphere force adjusted to target.");
    }
    return; // Exit loop to avoid running other steps prematurely
  }

  // Step 2: Start the pumps
  if (forceAdjusted && experimentRunning) {
    motorController.RunMotor(1, PUMP_RPM, HIGH); // Start pump 1
    motorController.RunMotor(2, PUMP_RPM, HIGH); // Start pump 2
    pumpsStarted = true;
    Serial.println("Pumps started.");
    return; // Exit loop to avoid running other steps prematurely
  }

  // Step 3: Start the rotation of the axes
  if (pumpsStarted && experimentRunning) {
    motorController.RunMotor(3, ROTATION_RPM, HIGH); // Start free sphere axis
    motorController.RunMotor(4, ROTATION_RPM, HIGH); // Start fixed sphere axis
    rotationStarted = true;
    Serial.println("Rotation started.");
    return; 
    // Exit loop to avoid running other steps prematurely
  }

  if (rotationStarted && experimentRunning) {
    Serial.println("Experiment running. Send \"stop\" to stop.");
  }

  if(experimentRunning){

  float value1 = sensors.ReadSensor(1);
  float value2 = sensors.ReadSensor(2);
  float value3 = sensors.ReadSensor(3);
  float value4 = sensors.ReadSensor(4);

  // Format the output for the Serial Plotter
  Serial.print(">");
  Serial.print("scale1:");
  Serial.print(value1, 2);
  Serial.print(",");
  Serial.print("scale2:");
  Serial.print(value2, 2);
  Serial.print(",");
  Serial.print("scale3:");
  Serial.print(value3, 2);
  Serial.print(",");
  Serial.print("scale4:");
  Serial.print(value4, 2);
  Serial.println(); // Ends the line with \r\n
  }
  

  delay(50); // Adjust the delay as needed
}
