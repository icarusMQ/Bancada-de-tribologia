#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <HX711.h>

//---------------------------------------------------------------------------------------------------
// Global variables and constants

  //Create an instance of the AccelStepper class for controlling the stepper motor
  AccelStepper Motor_FreeAxis(AccelStepper::DRIVER, 9, 8); 
  AccelStepper Motor_FixedAxis(AccelStepper::DRIVER, 5, 8); 
  AccelStepper Motor_ForceAxis(AccelStepper::DRIVER, A1, A0); 
  AccelStepper Motor_Pump_FreeAxis(AccelStepper::DRIVER, A3, A2); 
  AccelStepper Motor_Pump_FixedAxis(AccelStepper::DRIVER, A5, A4); 
  
  // Set steps/revolution for each stepper motor in the accelStepper library
  #define STEPS_PER_REVOLUTION_PUMPS 200.0 
  #define STEPS_PER_REVOLUTION_AXIS_MOTORS 1600.0 
  #define STEPS_PER_REVOLUTION_FORCE_AXIS 3200.0 
  
  // Set Rpm for each stepper motor in the accelStepper library
  #define RPM_PUMPS 20.0 // Set the RPM for the pumps
  #define RPM_AXIS_MOTORS 20.0 // Set the RPM for the axis motors
  #define RPM_FORCE_AXIS 20.0 // Set the RPM for the force axis motor
  
  // Set the steps per second for each stepper based on the RPM and steps per revolution
  float STEPS_PER_SECOND_PUMPS = (STEPS_PER_REVOLUTION_PUMPS * RPM_PUMPS / 60.0); // Steps per second for the pumps
  float STEPS_PER_SECOND_AXIS_MOTORS = (STEPS_PER_REVOLUTION_AXIS_MOTORS * RPM_AXIS_MOTORS / 60.0); // Steps per second for the axis motors
  float STEPS_PER_SECOND_FORCE_AXIS = (STEPS_PER_REVOLUTION_FORCE_AXIS * RPM_FORCE_AXIS / 60.0); // Steps per second for the force axis motor

      // Create and instance of each of the four load cells
      HX711 scale1; // Create an instance of the HX711 class for the first load cell
      HX711 scale2; // Create an instance of the HX711 class for the second load cell
      HX711 scale3; // Create an instance of the HX711 class for the third load cell
      HX711 scale4; // Create an instance of the HX711 class for the fourth load cell

      // Define the pins for each of the HX711 load cells
      #define LOADCELL_DOUT_PIN  2
      #define LOADCELL_SCK_PIN   3 
      #define LOADCELL_1_DOUT_PIN  4
      #define LOADCELL_1_SCK_PIN  6
      #define LOADCELL_2_DOUT_PIN  10
      #define LOADCELL_2_SCK_PIN  7
      #define LOADCELL_3_DOUT_PIN  12
      #define LOADCELL_3_SCK_PIN  11

      // Define the calibration factors for each load cell
      float calibration_factor1 = 0.02808; 
      float calibration_factor2 = 0.0342;
      float calibration_factor3 = -0.05671;
      float calibration_factor4 = -0.0614;

      // Define the target force and tolerance for the force adjustment
      float target_force = 5.0; 
      float tolerance = 0.5; 

      // defining variables to store load cell values
      float f1 = 0.0; 
      float f2 = 0.0; 
      float f3 = 0.0; 
      float f4 = 0.0; 

int state = 0; // Variable to store the state of the system
int experiment = 0; // Variable to store the experiment type, 0 = fixed and 1 = free

float experiment_duration_seconds = 40.0; // Duration of the experiment in seconds
float experiment_duration_milliseconds = experiment_duration_seconds * 1000.0; // Duration of the experiment in milliseconds

TimerHandle_t experiment_timer; // Timer handle for the experiment timer

SemaphoreHandle_t xMutex; // Mutex for synchronizing access to shared resources


//---------------------------------------------------------------------------------------------------
// Tasks

void load_read_Task(void *pvParameters) {
  // This task will read the load cells and update the values
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      // Read the load cells and store the values in the respective variables
      if (experiment == 0) {
        f1 = scale1.read_average(5);
        f2 = scale2.read_average(5);
      } else {
        f3 = scale3.read_average(5);
        f4 = scale4.read_average(5);
      }
      xSemaphoreGive(xMutex); // Release the mutex after updating values
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Serial_output_Task(void *pvParameters) {
  // This task will print the values of the load cells to the serial monitor
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) { // Take the mutex before accessing shared variables
      if (state == 1 && experiment == 0) {
        Serial.print("Load Cell 1: ");
        Serial.print(f1, 2); // Get the weight from load cell 1
        Serial.print(" N\tLoad Cell 2: ");
        Serial.print(f2, 2); // Get the weight from load cell 2
        Serial.println(" N");

        vTaskDelay(1000 / portTICK_PERIOD_MS);

      } else if (state == 1 && experiment == 1) {
        Serial.print("Load Cell 3: ");
        Serial.print(f3, 2); // Get the weight from load cell 3
        Serial.print(" N\tLoad Cell 4: ");
        Serial.print(f4, 2); // Get the weight from load cell 4
        Serial.println(" N");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
      xSemaphoreGive(xMutex); // Release the mutex after accessing shared variables
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void Update_Motor_Task(void *pvParameters) {
  for(;;){
    // Take the mutex before accessing shared variables
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      // This task is for updating the motors speed
      if (experiment == 0){
        Motor_FixedAxis.run(); // Update the fixed axis motor
        Motor_Pump_FixedAxis.run(); // Update the fixed axis pump motor
      } else {
        Motor_ForceAxis.run(); // Update the force axis motor
        Motor_FreeAxis.run(); // Update the free axis motor
        Motor_Pump_FreeAxis.run(); // Update the free axis pump motor
      }
      xSemaphoreGive(xMutex); // Release the mutex after updating motors
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Small delay to yield
  }
}

void Force_Adjustment_task(void *pvParameters) {
  for(;;){
    if (state == 0 && experiment == 1){
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) { // Take the mutex before accessing shared variables
        // This task will move the force arm motor until the target force is reached
        if(f1 > target_force + tolerance) { // If the force is greater than the target force plus tolerance
          Motor_ForceAxis.setSpeed(-STEPS_PER_SECOND_FORCE_AXIS); // Move the force arm motor in the negative direction
        } else if(f1 < target_force - tolerance) { // If the force is less than the target force minus tolerance
          Motor_ForceAxis.setSpeed(STEPS_PER_SECOND_FORCE_AXIS); // Move the force arm motor in the positive direction
        } else {
          Motor_ForceAxis.stop(); // Stop the force arm motor
          Motor_ForceAxis.setSpeed(0); // Set the speed to 0
          state = 1; // Set the state to 1 to indicate that the target force has been reached
        }
        xSemaphoreGive(xMutex); // Release the mutex after updating shared variables
      }
      vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 10 milliseconds
    }
  }
}

void Force_Adjustment_task_2(void *pvParameters) {
  for(;;){
    if (state == 0 && experiment == 1){
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) { // Take the mutex before accessing shared variables
        // This task will print the value of the force being adjusted to the serial monitor
        Serial.print("Force: ");  
        Serial.print(f1);
        Serial.print(" N\tTarget Force: ");
        Serial.print(target_force);
        Serial.println();
        xSemaphoreGive(xMutex); // Release the mutex after accessing shared variables
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2 seconds
    }
  }
}

void Remaining_Motors_task(void *pvParameters) {
  static int prev_state =  - 1 ; // Variable to store the previous state
  for(;;){
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) { // Take the mutex before accessing shared variables
      if(state == 1 && prev_state != 1) {
        xTimerStart(experiment_timer, 0); // Start the experiment timer
      }
      prev_state = state;
      if (state == 1 && experiment == 0){
        // This task will move the remaining motors to their respective positions
        Motor_Pump_FixedAxis.setSpeed(STEPS_PER_SECOND_PUMPS); // Move the fixed axis pump motor in the positive direction
        Motor_FixedAxis.setSpeed(STEPS_PER_SECOND_AXIS_MOTORS); // Move the fixed axis motor in the positive direction

      }else if (state == 1 && experiment == 1){
        Motor_Pump_FreeAxis.setSpeed(STEPS_PER_SECOND_PUMPS); // Move the free axis pump motor in the positive direction
        Motor_FreeAxis.setSpeed(STEPS_PER_SECOND_AXIS_MOTORS); // Move the free axis motor in the positive direction

      }
      xSemaphoreGive(xMutex); // Release the mutex after updating shared variables
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 10 milliseconds
  }
}

void expriment_timer_callback(TimerHandle_t xTimer) {
  // This function will be called when the experiment timer expires
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    if (state == 1 && experiment == 0) {
      Motor_FixedAxis.stop(); // Stop the fixed axis motor
      Motor_Pump_FixedAxis.stop(); // Stop the fixed axis pump motor
      state = 2; // Set the state to 2 to indicate that the experiment is finished
      Serial.println("Experiment finished.");
    } else if (state == 1 && experiment == 1) {
      Motor_FreeAxis.stop(); // Stop the free axis motor
      Motor_Pump_FreeAxis.stop(); // Stop the free axis pump motor
      state = 2; // Set the state to 2 to indicate that the experiment is finished
      Serial.println("Experiment finished.");
    }
    xSemaphoreGive(xMutex);
  }
}


//---------------------------------------------------------------------------------------------------
// setup function


void setup() {
  Serial.begin(115200);

   // Create a mutex for synchronizing access to shared resources
   xMutex = xSemaphoreCreateMutex(); // Create the mutex

  Serial.print("================================================\n");
  Serial.print("============ Bancada de tribologia =============\n");
  Serial.print("================================================\n");

      // Initialize the load cells with the defined pins
      scale1.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
      scale2.begin(LOADCELL_1_DOUT_PIN, LOADCELL_1_SCK_PIN);  
      scale3.begin(LOADCELL_2_DOUT_PIN, LOADCELL_2_SCK_PIN);
      scale4.begin(LOADCELL_3_DOUT_PIN, LOADCELL_3_SCK_PIN);

       // Set the maximum speed and acceleration for each motor
       Motor_FreeAxis.setMaxSpeed(STEPS_PER_SECOND_AXIS_MOTORS); // Set the maximum speed for the free axis motor
       Motor_FreeAxis.setAcceleration(1000); // Set the acceleration for the free axis motor
       Motor_FixedAxis.setMaxSpeed(STEPS_PER_SECOND_AXIS_MOTORS); // Set the maximum speed for the fixed axis motor
       Motor_FixedAxis.setAcceleration(500); // Set the acceleration for the fixed axis motor
       Motor_ForceAxis.setMaxSpeed(STEPS_PER_SECOND_FORCE_AXIS);
       Motor_ForceAxis.setAcceleration(600);
       Motor_Pump_FreeAxis.setMaxSpeed(STEPS_PER_SECOND_PUMPS); // Set the maximum speed for the free axis pump motor
       Motor_Pump_FreeAxis.setAcceleration(1000); // Set the acceleration for the free axis pump motor
       Motor_Pump_FixedAxis.setMaxSpeed(STEPS_PER_SECOND_PUMPS); // Set the maximum speed for the fixed axis pump motor
       Motor_Pump_FixedAxis.setAcceleration(1000); // Set the acceleration for the fixed axis pump motor

    // Set the calibration factors for each load cell
    scale1.set_scale(calibration_factor1); // Set the calibration factor for the first load cell
    scale2.set_scale(calibration_factor2); // Set the calibration factor for the second load cell
    scale3.set_scale(calibration_factor3); // Set the calibration factor for the third load cell
    scale4.set_scale(calibration_factor4); // Set the calibration factor for the fourth load cell

    delay(10000); // Wait for 10 second to allow the load cells to stabilize

    // Tare the load cells to zero out any initial weight
    scale1.tare(); 
    scale2.tare(); 
    scale3.tare(); 
    scale4.tare(); 

    Serial.println("Load cells tared successfully.");

    if(xTaskCreate(load_read_Task, "Load Read Task", 128, NULL, 5, NULL) == pdPASS) {
      Serial.println("Load read task created successfully.");
    } else {
      Serial.println("Failed to create load read task.");
    }
    if(xTaskCreate(Serial_output_Task, "Serial Output Task", 512, NULL, 1, NULL) == pdPASS) {
      Serial.println("Serial output task created successfully.");
    } else {
      Serial.println("Failed to create serial output task.");
    }
    if(xTaskCreate(Update_Motor_Task, "Update Motor Task", 128, NULL, 6, NULL) == pdPASS) {
      Serial.println("Update motor task created successfully.");
    } else {
      Serial.println("Failed to create update motor task.");
    }
    if(xTaskCreate(Force_Adjustment_task, "Force Adjustment Task", 256, NULL, 4, NULL) == pdPASS) {
      Serial.println("Force adjustment task created successfully.");
    } else {
      Serial.println("Failed to create force adjustment task.");
    }
    if(xTaskCreate(Force_Adjustment_task_2, "Force Adjustment Task 2", 512, NULL, 2, NULL) == pdPASS) {
      Serial.println("Force adjustment task 2 created successfully.");
    } else {
      Serial.println("Failed to create force adjustment task 2.");
    }
    if(xTaskCreate(Remaining_Motors_task, "Remaining Motors Task", 128, NULL, 3, NULL) == pdPASS) {
      Serial.println("Remaining motors task created successfully.");
    } else {
      Serial.println("Failed to create remaining motors task.");
    }


    // Create a timer for the experiment
   experiment_timer = xTimerCreate("Experiment Timer", pdMS_TO_TICKS(experiment_duration_milliseconds), pdFALSE, (void *)0, expriment_timer_callback); // Create the experiment timer
   if (experiment_timer != NULL) {
      Serial.println("Experiment timer created"); 
   } else {
      Serial.println("Failed to create experiment timer.");
   }
}

void loop() {
  // Not used with FreeRTOS
}

