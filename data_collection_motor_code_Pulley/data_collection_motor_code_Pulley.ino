/* Daria Talaski
 *  December 31, 2018
 *  Objective: Output values for distance sensor and gyroscope, and run motor at the same time. 
 
 Edits:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 Maya Clinton 
 *June 11th, 2019
 *Added limit switches with interrupts
  
 *June 13th, 2019
 *Added current sensor code

 *June 17,2019
 *moved sensors into protothreads
 
 *June 18-19, 2019
 *moved motor commands into protothreads. A lot of original code commented out. 
 *added pin reset
 *going to try something with motor code to see if I can get it to read distance, take that number, then move motor until distance reads a stop number/stop distance, going to try for 500 millimeters
 
 *June 20, 2019
 *fixed bugs now code reads distance sensor values and compares it to user inputs then moves the motor forward or backward in relation to distance goal and actual distance. Code stops once it reaches distance goal (this part may take a while due to distance sensor fluctuation)   
 *new goal need to get it to stop within a window of the value input
 *took out old code which is saved in motor__protothread2 for clarity with the code and what I am doing 
 
 *June 21-24,2019
 *added protothreads for FRONT, MIDDLE, and BACK
 
 *June 25, 2019
 *added protothread for low battery error message 
 
 *November 15, 2019
 *created new file for pulley -- motor must turn oppostite direction for correct movement. 
 
 
 ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

// Set Up Timer ///////////////////////
  //unsigned long time

// Header for Distance sensor /////////////////////////////////////////////////////////////////////////
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Header for Gyroscope //////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_INA219.h> //current sensor library
#include <pt.h> //protothread library

#define BNO055_SAMPLERATE_DELAY_MS (100) //Set the delay between fresh samples

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void displaySensorDetails(){ //details about the gyro sensor
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------\n");
  delay(500);
}

void displaySensorStatus(){ //gyro sensor status
  // Get the system status values (mostly for debugging purposes)
  uint8_t systemStatus, selfTestResults, systemError;
  systemStatus = selfTestResults = systemError = 0;
  bno.getSystemStatus(&systemStatus, &selfTestResults, &systemError);

  // Display the results in the Serial Monitor
  Serial.print("\nSystem Status: 0x");
  Serial.println(systemStatus, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(selfTestResults, HEX);
  Serial.print("System Error:  0x");
  Serial.println(systemError, HEX);
  Serial.println("");
  delay(500);
}

void displayCalStatus(){
  /* Get the four calibration values (0..3)
   * Any sensor data reporting 0 should be ignored,
   * 3 means 'fully calibrated"
   */
  uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
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
//Declare pin functions on Arduino
#define stp 2
#define dir 3
#define MS1 4
#define MS2 5
#define MS3 6
#define EN  7

// Declare variables for functions
String userInput;
int userInputNum, distance;
//bool prompt = false;

bool front = false, middle = false, back = false, notANumber = false;

// Limit switch variable declaration
const byte interruptPin1 = 19; //limit switch 1
const byte interruptPin2 = 18; //limit switch 2
volatile byte allowForward = true; //boolean for forward limit
volatile byte allowReverse = true; //boolean for reverse limit 

Adafruit_INA219 ina219; //calling current sensor 

// Current sensor variables (getCurrent)
float shuntVoltage = 0, busVoltage = 0, current_mA = 0, loadVoltage = 0, power_mW = 0;

// Low battery message variables
float battery = 0;
bool lowBattery = false;

#define timeThresh 100 //declaring time (milliseconds) for sensor protothreads


static struct pt getGyroPT, getDistancePT, getCurrentPT, getNewInputPT, forwardRunPT, backwardRunPT, stopRunPT, goFrontPT, goMiddlePT, goBackPT, getSetPointPT; //lowBattPT, getPromptPT; //declaring protothreads

static int gyroProtothread(struct pt *pt){ //setting up protothread for BNO gyro sensor
  static unsigned long runtime = 0; //tracks amount of time since last trigger
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, millis() - runtime > timeThresh); //waits until it has been timeThresh ms since the last trigger
    getGyro();
    runtime = millis(); //sets runtime to the amount of time passed to restart timing
  PT_END(pt);
}

static int setPointProtothread(struct pt *pt){ //setting up protothread for BNO gyro sensor
  static unsigned long runtime = 0;
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, millis() - runtime > timeThresh);
    getSetPoint();
    runtime = millis();
  PT_END(pt);
}

static int distanceProtothread(struct pt *pt){
  static unsigned long runtime = 0;
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, millis() - runtime> timeThresh);
    getDistance();
    runtime = millis();
  PT_END(pt);
}

static int currentProtothread(struct pt *pt){
  static unsigned long runtime = 0;
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, millis() - runtime > timeThresh);
    getCurrent();
    runtime = millis();
  PT_END(pt);
}

static int newInputProtothread(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, Serial.available()); //waits until an input has been made 
    getNewInput(); //once triggered protothread will run fuction for reading and decifering user input
  PT_END(pt);
}

static int runBackwardProtothread(struct pt *pt){ //setting up protothread for frontward motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, distance > userInputNum && allowForward && !notANumber);  //waits until the distance of the mass is less than the distance the user gives it, the front limit switch is not pressed, and none of the FRONT, BACK, MIDDLE prototheads read true.
    forwardRun(); //once triggered protothread will run fuction to start forward motor run
  PT_END(pt);
}

static int runForwardProtothread(struct pt *pt){ //setting up protothread for backward motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, distance < userInputNum && allowReverse);  //waits until the distance of the mass is greater than the distance the user inputs. Also checks that the back limit switch is not pressed.  
    backwardRun(); //once triggered protothread will run fuction to start backward motor run
  PT_END(pt);
}

static int stopRunProtothread(struct pt *pt){ //setting up protothread for stopping motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, userInputNum + 1 >= distance && distance >= userInputNum - 1); //waits until the distance is within 1 millimeter from the distance the user gives.  
    stopRun(); //once triggered protothread will run fuction to stop motor step
  PT_END(pt);
}

//static int newPromptProtothread(struct pt *pt){ //setting up protothread for forward motor step
//  PT_BEGIN(pt);
//    PT_WAIT_UNTIL(pt,!prompt && !(allowForward && allowReverse)); //waits until there is a value for x_forward, in this case 1000
//    getPrompt(); //once triggered protothread will run fuction to start forward motor step
//  PT_END(pt);
//}


static int backProtothread(struct pt *pt){ //setting up protothread for user input "FRONT"
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, front); //waits until the user inputs "FRONT"
    goFront(); //once triggered protothread will run fuction to move to the "front"
  PT_END(pt);
}

static int middleProtothread(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, middle);
    goMiddle();
  PT_END(pt);
}

static int frontProtothread(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, back);
    goBack();
    PT_END(pt);
}

/*
static int lowBatteryProtothread(struct pt *pt){ //setting up protothread for low battery
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, lowBattery); //waits until lowBattery bool is true 
    Serial.println("Low battery! Please replace.\t\t\t\t\t\t\t");
  PT_END(pt);
}
*/



//Functions-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void resetBEDPins(){ //Reset Big Easy Driver pins to default states
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(EN, HIGH);
}

void getDistance(){  //function to grab and display distance sensor data
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); //pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4){ //phase failures have incorrect data
    distance = measure.RangeMilliMeter;  //sets distance variable to the data the distance sensor measures
    Serial.print(distance); Serial.print("\t\t\t"); //prints that distance
  }
  else Serial.print(" out of range \t\t\t"); //error message if distance too far for sensor to read
}

void getGyro(){ //function to grab and display BNO gyro sensor data
  sensors_event_t event;
  bno.getEvent(&event); //grabs sensor event
  Serial.print(event.orientation.x, 4); Serial.print("\t\t\t");
  Serial.print(event.orientation.y, 4); Serial.print("\t\t\t");
  Serial.print(event.orientation.z, 4); Serial.print("\t\t\t");
}

void getCurrent(){ //function to grab and display current sensor data
  // Current sensor loop code  //this code was moved up
  //float shuntVoltage = 0;
  //float busVoltage = 0;
  //float current_mA = 0;
  //float loadVoltage = 0;
  //float power_mW = 0;

  // Grabbing data for current sensor 
  shuntVoltage = ina219.getShuntVoltage_mV(); 
  busVoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadVoltage = busVoltage + shuntVoltage / 1000; //calculation for load voltage

  // Outputs sensor data
  //Serial.println("");
  //Serial.print("\t\t\t\t\t\t\tBus Voltage:   "); Serial.print(busVoltage); Serial.println(" V");
  //Serial.print("\t\t\t\t\t\t\tShunt Voltage: "); Serial.print(shuntVoltage); Serial.println(" mV");
  //Serial.print("\t\t\t\t\t\t\tLoad Voltage:  "); Serial.print(loadVoltage); Serial.println(" V");
  //Serial.print("\t\t\t\t\t\t\tCurrent:       "); Serial.print(current_mA); Serial.println(" mA");
  //Serial.print("\t\t\t\t\t\t\tPower:         "); Serial.print(power_mW); Serial.println(" mW\n");
  Serial.print(busVoltage); Serial.print("\t\t\t");
  Serial.print(shuntVoltage); Serial.print("\t\t\t");
  Serial.print(loadVoltage); Serial.print("\t\t\t");
  Serial.print(current_mA); Serial.print("\t\t\t");
  Serial.print(power_mW); Serial.print("\t\t\t\n");

  battery = busVoltage; // setup for low battery error message
  if (battery < 9.00) lowBattery = true;
  else lowBattery = false;
}

void getNewInput(){ //function for user input 
  userInput = ""; //clears user input
  while (Serial.available()) userInput += (char) Serial.read(); //appends serial input to userInput a char at a time
  Serial.print(userInput); Serial.print("\t");   
  front = false, back = false, middle = false;
  digitalWrite(EN, LOW);
  userInputNum = userInput.toInt(); //casts input to an int to be compared to the distance

  if (userInputNum == 0){ //userInput contains letters
    if (userInput.equals("FRONT")) front = true;
    else if (userInput.equals("MIDDLE")) middle = true;
    else if (userInput.equals("BACK")) back = true;
    else {
      Serial.print("invalid input");
      digitalWrite(EN, HIGH);
    }
  }
  notANumber = (front || middle|| back); //creates variable not a number so that these are not read at the very beginning.
}

void backwardRun(){ //function for forward motion
  //Serial.println("forwardRun");  
  digitalWrite(dir, LOW); //low direction means forward
  digitalWrite(stp, HIGH);
  delay(1);
  digitalWrite(stp, LOW);
  delay(1);
}

void forwardRun(){ //function for backward motion
  //Serial.println("backwardRun");
  digitalWrite(dir, HIGH);
  digitalWrite(stp, HIGH);
  delay(1);
  digitalWrite(stp, LOW);
  delay(1);
} 

void stopRun(){ //stop function
  //Serial.println("stop"); 
  digitalWrite(stp, LOW);  //step and enable pin set to "off" state so that mass is no longer moving 
  digitalWrite(EN, HIGH);
  resetBEDPins();// sets all motor pins to default state
}
  
//void getPrompt(){
//  Serial.println("enter new value\n");
//  prompt = true;
//}  

void goBack(){ //function for moving to the front
 //Serial.println("goFront");
  if (distance > 400){ //if distance greater than 400mm it will move forward until it reaches 400mm
    digitalWrite(dir, LOW);
    digitalWrite(stp, HIGH);
    delay(1);
    digitalWrite(stp, LOW);
    delay(1);
  }
  else {
    Serial.println("else ft");
    front = false;
  }
}

void goMiddle(){ //functions for moving to the middle 
  //Serial.println("goMiddle");
  if (distance > 500){ //if distance is greater then 500mm will move forward until reaches 500mm
    digitalWrite(dir, LOW);
    digitalWrite(stp, HIGH);
    delay(1);
    digitalWrite(stp, LOW);
    delay(1);
  }
  else if (distance < 500){ //if distance less than 500 will move backward until reaches 500
    digitalWrite(dir, HIGH);
    digitalWrite(stp, HIGH);
    delay(1);
    digitalWrite(stp, LOW);
    delay(1); 
  }
  else {
    Serial.println("else md");
    middle = false;
  }
}

void goFront(){ //function for moving to the back
  //Serial.println("goBack");
  if (distance < 650){ //if distance less than 650 the motor will move back until it reaches 650
    digitalWrite(dir, HIGH);
    digitalWrite(stp, HIGH);
    delay(1);
    digitalWrite(stp, LOW);
    delay(1);
  }
  else {
    Serial.println("else bk");
    back = false;
  }
}

void isrBackward(){ //limit switch disables forward movement function
  if (digitalRead(interruptPin1) == HIGH) allowReverse = false;
  else allowReverse = true;
}

void isrForward(){
  if (digitalRead(interruptPin2) == HIGH) allowForward = false;
  else allowForward = true;
}

void header(){
  Serial.println("-----------------------------------------\n");
  Serial.println("Begin motor control");
  Serial.println("Enter number between 300 and 700 millimeters");
  Serial.println("-----------------------------------------\n");
  Serial.print("Set point\tDistance(mm)\t\t");
  Serial.print("X\t\t\t\tY\t\t\tZ\t\t\t");
  Serial.print("Bus Vol(V)\t\t\t");
  Serial.print("Shunt Vol(mV)\t\t");
  Serial.print("Load Vol(V)\t\t");
  Serial.print("Current(mA)\t\t");
  Serial.print("Power(mW)\n");
  Serial.println("-----------------------------------------\n\n");
}

void getSetPoint(){
  if (userInput == "") Serial.print("0\t\t");
  else {Serial.print(userInputNum); Serial.print("\t\t");}
}



//setup-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup(){
  Serial.begin(9600);//500000
  Serial.print("hello");
  // Distance Sensor Checks//////////////////////////////////////////////////
  while (!Serial) delay(1); //wait until serial port opens for native USB devices
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()){
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  // Gyroscope Sensor Checks/////////////////////////////////////////////////
  Serial.println("Orientation Sensor Test\n");

  // Initialize the sensor
  if (!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  displaySensorDetails(); //display some basic information on this sensor
  displaySensorStatus(); //optional: display current status

  bno.setExtCrystalUse(true);

  header(); //creating header

  // Motor Sensor Checks/////////////////////////////////////////////////
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(EN, OUTPUT);

  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(EN, HIGH);
  
  //Serial.println("Begin motor control");
  //Serial.println(" ");
  //Serial.println("Enter number between 300 and 700 millimeters");
  //Serial.println("");
  //
  //// Table Headers
  //Serial.println("-----------------------------------------\n");
  //Serial.print("Distance (mm)\t");
  //Serial.print("X\tY\tZ\t");
  //Serial.print("Bus Voltage (V)\t");
  //Serial.print("Shunt Voltage(mV)\t");
  //Serial.print("Load Voltage(V)\t");
  //Serial.print("Current (mA)\t");
  //Serial.print("Power (mW)\t\n");

  //limit switch setup
  pinMode(interruptPin1, INPUT);
  pinMode(interruptPin2, INPUT);
  //digitalRead(interruptPin1);
  //digitalRead(interruptPin2);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), isrForward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), isrBackward, CHANGE); 

  // Current sensor set up
  uint32_t currentFrequency;
  
  /* Initialize the INA219.
   * By default the initialization will use the largest range (32V, 2A).  However
   * you can call a setCalibration function to change this range (see comments).
   */
  ina219.begin();
  //ina219.setCalibration_32V_1A(); //uses a 32V, 1A range (higher precision on amps)
  //ina219.setCalibration_16V_400mA(); //uses a 16V, 400mA range (higher precision on volts and amps)

  //Serial.println("Measuring voltage and current with INA219 ...");

  // Initialize protothreads
  //PT_INIT(&getGyroPT);
  //PT_INIT(&getDistancePT);
  //PT_INIT(&getCurrentPT);
  //PT_INIT(&getInputPT);
  //PT_INIT(&getfsPT);
  //PT_INIT(&getrsPT);
  //PT_INIT(&);
  //PT_INIT(&);
} 



//Loop--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop(){
  // Calling the protothreads in the loop
  setPointProtothread(&getSetPointPT);
  distanceProtothread(&getDistancePT);
  gyroProtothread(&getGyroPT);
  currentProtothread(&getCurrentPT);
  newInputProtothread(&getNewInputPT);
  runForwardProtothread(&forwardRunPT);
  runBackwardProtothread(&backwardRunPT);
  stopRunProtothread(&stopRunPT);
  //newPromptProtothread(&getPromptPT);
  frontProtothread(&goFrontPT);
  middleProtothread(&goMiddlePT);
  backProtothread(&goBackPT);
  //lowBatteryProtothread(&lowBattPT);
}
