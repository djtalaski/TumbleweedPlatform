/* Daria Talaski
 *  December 31, 2018
 *  Objective: Output values for distance sensor and gyroscope, and run motor at the same time. 
 */

 // Set Up Timer ///////////////////////
 // unsigned long time

// Header for Distance sensor /////////////////////////////////////////////////////////////////////////
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Header for Gyroscope //////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void displaySensorDetails()
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus()
{
  // Get the system status values (mostly for debugging purposes)
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  // Display the results in the Serial Monitor
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displayCalStatus()
{
  /* Get the four calibration values (0..3)
   * Any sensor data reporting 0 should be ignored,
   * 3 means 'fully calibrated"
   */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // The data should be ignored until the system calibration is > 0
  Serial.print("\t");
  if (!system) Serial.print("! ");

  // Display the individual values
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

// Header for Motor Driver //////////////////////////////////////////////////////////////////////////////
// Declare pin functions on Arduino
#define stp 2
#define dir 3
#define MS1 4
#define MS2 5
#define MS3 6
#define EN  7

// Declare variables for functions
char user_input;
int state;

void setup()
{
   Serial.begin(9600);

  // Distance Sensor Checks//////////////////////////////////////////////////
  // wait until serial port opens for native USB devices
  while (!Serial) delay(1);
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin())
  {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  // Gyroscope Sensor Checks/////////////////////////////////////////////////
  Serial.println("Orientation Sensor Test\n");

  // Initialise the sensor
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  // Display some basic information on this sensor
  displaySensorDetails();

  // Optional: Display current status
  displaySensorStatus();

  bno.setExtCrystalUse(true);

  // Motor Sensor Checks/////////////////////////////////////////////////
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(EN, OUTPUT);
  resetBEDPins(); // Set step, direction, microstep and enable pins to default states
  Serial.begin(9600); // Open Serial connection for debugging
  Serial.println("Begin motor control\n");

  // Print function list for user selection
  Serial.println("Enter number for control option:");
  Serial.println("1. Turn at default microstep mode.");
  Serial.println("2. Reverse direction at default microstep mode.");
  Serial.println("3. Turn at 1/16th microstep mode.");
  Serial.println("4. Step forward and reverse directions.");

  // Table Headers
  Serial.print("Distance\t");
  Serial.print("X\t");
  Serial.print("Y\t");
  Serial.print("Z\t\n");
}

void loop()
{
  // Time
  //time=millis();

  // Distance Sensor
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) // phase failures have incorrect data
  {
    // Get a new sensor event
    sensors_event_t event;
    bno.getEvent(&event);
       
    // Table Values
    //Serial.print(time);                   Serial.print("\t"); 
    Serial.print(measure.RangeMilliMeter); Serial.print("\t"); 
    Serial.print(event.orientation.x, 4);  Serial.print("\t"); 
    Serial.print(event.orientation.y, 4);  Serial.print("\t"); 
    Serial.print(event.orientation.z, 4);  Serial.print("\t");

    // Optional: Display calibration status
    displayCalStatus();

    // Optional: Display sensor status (debug only)
    //displaySensorStatus();

    Serial.println(""); // New line for the next sample

    delay(BNO055_SAMPLERATE_DELAY_MS); // Wait the specified delay before requesting next data
  }
  else
  {
    // Get a new sensor event
    sensors_event_t event;
    bno.getEvent(&event);
  
    // Table Values
    Serial.print(" out of range \t");
    Serial.print(event.orientation.x, 4); Serial.print("\t"); 
    Serial.print(event.orientation.y, 4); Serial.print("\t"); 
    Serial.print(event.orientation.z, 4); Serial.print("\t");

    // Optional: Display calibration status
    displayCalStatus();

    // Optional: Display sensor status (debug only)
    //displaySensorStatus();

    Serial.println(""); // New line for the next sample
  
    delay(BNO055_SAMPLERATE_DELAY_MS); // Wait the specified delay before requesting next data
  } 

  // Motor Code //////////////////////////////////////////////////////////////
  while(Serial.available())
  {
    user_input = Serial.read(); // Read user input and trigger appropriate function
    digitalWrite(EN, LOW); // Pull enable pin low to activate FETs and allow motor control
    switch(user_input)
    {
      case '1': StepForwardDefault();   break;
      case '2': ReverseStepDefault();   break;
      case '3': SmallStepMode();        break;
      case '4': ForwardBackwardStep();  break;
      default: Serial.println("Invalid option entered.");
    }
    resetBEDPins();
  }
}

// Reset Big Easy Driver pins to default states
void resetBEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(EN, HIGH);
}

// Default microstep mode function
void StepForwardDefault()
{
  Serial.println("Moving forward at default step mode.");
  digitalWrite(dir, LOW); // Pull direction pin low to move "forward"
  for (int x = 0; x < 1000; x++) // Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp, HIGH); // Trigger one step forward
    delay(1);
    digitalWrite(stp, LOW); // Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println("Enter new option\n");
}

// Reverse default microstep mode function
void ReverseStepDefault()
{
  Serial.println("Moving in reverse at default step mode.");
  digitalWrite(dir, HIGH); // Pull direction pin high to move in "reverse"
  for (int x = 0; x < 1000; x++) // Loop the stepping enough times for motion to be visible
  {
    digitalWrite(stp, HIGH); // Trigger one step
    delay(1);
    digitalWrite(stp, LOW); // Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println("Enter new option\n");
}

// 1/16th microstep foward mode function
void SmallStepMode()
{
  Serial.println("Stepping at 1/16th microstep mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, HIGH); //Pull MS1,MS2, and MS3 high to set logic to 1/16th microstep resolution
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
  for (int x = 0; x < 1000; x++)  // Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp, HIGH); // Trigger one step forward
    delay(1);
    digitalWrite(stp, LOW); // Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println("Enter new option\n");
}

// Forward/reverse stepping function
void ForwardBackwardStep()
{
  Serial.println("Alternate between stepping forward and reverse.");
  for (int x = 0; x < 5; x++) // Loop the forward stepping enough times for motion to be visible
  {
    // Read direction pin state and change it
    state = digitalRead(dir);
    if (state == HIGH) digitalWrite(dir, LOW);
    else if (state == LOW) digitalWrite(dir, HIGH);
    
    for (int y = 0; y < 1000; y++)
    {
      digitalWrite(stp, HIGH); // Trigger one step
      delay(1);
      digitalWrite(stp, LOW); // Pull step pin low so it can be triggered again
      delay(1);
    }
  }
  Serial.println("Enter new option\n");
}
