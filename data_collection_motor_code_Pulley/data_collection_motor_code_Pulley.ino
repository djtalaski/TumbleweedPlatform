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
String user_input;
int user_inputnum, distance;
//bool prompt = false;

bool front = false, middle = false, back = false, notanumber = false;

// Limit switch variable declaration
const byte interruptpin1 = 19; //limit switch 1
const byte interruptpin2 = 18; //limit switch 2
volatile byte allowforward = true; //boolean for forward limit
volatile byte allowreverse = true; //boolean for reverse limit 

Adafruit_INA219 ina219; //calling current sensor 

// Current sensor variables (getcurrent)
float shuntvoltage = 0, busvoltage = 0, current_mA = 0, loadvoltage = 0, power_mW = 0;

// Low battery message variables
float battery = 0;
bool low_battery = false;

#define timethresh 100 //declaring time (milliseconds) for sensor protothreads


static struct pt getgyroPT, getdistancePT, getcurrentPT, getnewinputPT, forwardrunPT, backwardrunPT, stoprunPT, gofrontPT, gomiddlePT, gobackPT, getsetpointPT; //lowbattPT, getpromptPT; //declaring protothreads

static int gyroprotothread(struct pt *pt){ //setting up protothread for BNO gyro sensor
  static unsigned long runtime = 0; //tracks amount of time since last trigger
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, millis() - runtime > timethresh); //waits until it has been timethresh ms since the last trigger
    getgyro();
    runtime = millis(); //sets runtime to the amount of time passed to restart timing
  PT_END(pt);
}

static int setpointprotothread(struct pt *pt){ //setting up protothread for BNO gyro sensor
  static unsigned long runtime = 0;
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt,millis() - runtime > timethresh);
    getsetpoint();
    runtime = millis();
  PT_END(pt);
}

static int distanceprotothread(struct pt *pt){
  static unsigned long runtime = 0;
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, millis() - runtime> timethresh);
    getdistance();
    runtime = millis();
  PT_END(pt);
}

static int currentprotothread(struct pt *pt){
  static unsigned long runtime = 0;
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, millis() - runtime > timethresh);
    getcurrent();
    runtime = millis();
  PT_END(pt);
}

static int newinputprotothread(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, Serial.available()); //waits until an input has been made 
    getnewinput(); //once triggered protothread will run fuction for reading and decifering user input
  PT_END(pt);
}

static int runbackwardprotothread(struct pt *pt){ //setting up protothread for frontward motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, distance > user_inputnum && allowforward && !notanumber);  //waits until the distance of the mass is less than the distance the user gives it, the front limit switch is not pressed, and none of the FRONT, BACK, MIDDLE prototheads read true.
    forwardrun(); //once triggered protothread will run fuction to start forward motor run
  PT_END(pt);
}

static int runforwardprotothread(struct pt *pt){ //setting up protothread for backward motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, distance < user_inputnum && allowreverse);  //waits until the distance of the mass is greater than the distance the user inputs. Also checks that the back limit switch is not pressed.  
    backwardrun(); //once triggered protothread will run fuction to start backward motor run
  PT_END(pt);
}

static int stoprunprotothread(struct pt *pt){ //setting up protothread for stopping motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, user_inputnum + 1 >= distance && distance >= user_inputnum - 1); //waits until the distance is within 1 millimeter from the distance the user gives.  
    stoprun(); //once triggered protothread will run fuction to stop motor step
  PT_END(pt);
}

//static int newpromptprotothread(struct pt *pt){ //setting up protothread for forward motor step
//  PT_BEGIN(pt);
//    PT_WAIT_UNTIL(pt,!prompt && !(allowforward && allowreverse)); //waits until there is a value for x_forward, in this case 1000
//    getprompt(); //once triggered protothread will run fuction to start forward motor step
//  PT_END(pt);
//}


static int backprotothread(struct pt *pt){ //setting up protothread for user input "FRONT"
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, front); //waits until the user inputs "FRONT"
    gofront(); //once triggered protothread will run fuction to move to the "front"
  PT_END(pt);
}

static int middleprotothread(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, middle);
    gomiddle();
  PT_END(pt);
}

static int frontprotothread(struct pt *pt){
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, back);
    goback();
    PT_END(pt);
}

/*
static int lowbatteryprotothread(struct pt *pt){ //setting up protothread for low battery
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, low_battery); //waits until low_battery bool is true 
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

void getdistance(){  //function to grab and display distance sensor data
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); //pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4){ //phase failures have incorrect data
    distance = measure.RangeMilliMeter;  //sets distance variable to the data the distance sensor measures
    Serial.print(distance); Serial.print("\t\t\t"); //prints that distance
  }
  else Serial.print(" out of range \t\t\t"); //error message if distance too far for sensor to read
}

void getgyro(){ //function to grab and display BNO gyro sensor data
  sensors_event_t event;
  bno.getEvent(&event); //grabs sensor event
  Serial.print(event.orientation.x, 4); Serial.print("\t\t\t");
  Serial.print(event.orientation.y, 4); Serial.print("\t\t\t");
  Serial.print(event.orientation.z, 4); Serial.print("\t\t\t");
}

void getcurrent(){ //function to grab and display current sensor data
  // Current sensor loop code  //this code was moved up
  //float shuntvoltage = 0;
  //float busvoltage = 0;
  //float current_mA = 0;
  //float loadvoltage = 0;
  //float power_mW = 0;

  // Grabbing data for current sensor 
  shuntvoltage = ina219.getShuntVoltage_mV(); 
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + shuntvoltage / 1000; //calculation for load voltage

  // Outputs sensor data
  //Serial.println("");
  //Serial.print("\t\t\t\t\t\t\tBus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  //Serial.print("\t\t\t\t\t\t\tShunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  //Serial.print("\t\t\t\t\t\t\tLoad Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  //Serial.print("\t\t\t\t\t\t\tCurrent:       "); Serial.print(current_mA); Serial.println(" mA");
  //Serial.print("\t\t\t\t\t\t\tPower:         "); Serial.print(power_mW); Serial.println(" mW\n");
  Serial.print(busvoltage); Serial.print("\t\t\t");
  Serial.print(shuntvoltage); Serial.print("\t\t\t");
  Serial.print(loadvoltage); Serial.print("\t\t\t");
  Serial.print(current_mA); Serial.print("\t\t\t");
  Serial.print(power_mW); Serial.print("\t\t\t\n");

  battery = busvoltage; // setup for low battery error message
  if (battery < 9.00) low_battery = true;
  else low_battery = false;
}

void getnewinput(){ //function for user input 
  user_input = ""; //clears user input
  while (Serial.available()) user_input += (char) Serial.read(); //appends serial input to user_input a char at a time
  Serial.print(user_input); Serial.print("\t");   
  front = false, back = false, middle = false;
  digitalWrite(EN, LOW);
  user_inputnum = user_input.toInt(); //casts input to an int to be compared to the distance

  if (user_inputnum == 0){ //user_input contains letters
    if (user_input.equals("FRONT")) front = true;
    else if (user_input.equals("MIDDLE")) middle = true;
    else if (user_input.equals("BACK")) back = true;
    else {
      Serial.print("invalid input");
      digitalWrite(EN, HIGH);
    }
  }
  notanumber = (front || middle|| back); //creates variable not a number so that these are not read at the very beginning.
}

void backwardrun(){ //function for forward motion
  //Serial.println("forwardrun");  
  digitalWrite(dir, LOW); //low direction means forward
  digitalWrite(stp, HIGH);
  delay(1);
  digitalWrite(stp, LOW);
  delay(1);
}

void forwardrun(){ //function for backward motion
  //Serial.println("backwardrun");
  digitalWrite(dir, HIGH);
  digitalWrite(stp, HIGH);
  delay(1);
  digitalWrite(stp, LOW);
  delay(1);
} 

void stoprun(){ //stop function
  //Serial.println("stop"); 
  digitalWrite(stp, LOW);  //step and enable pin set to "off" state so that mass is no longer moving 
  digitalWrite(EN, HIGH);
  resetBEDPins();// sets all motor pins to default state
}
  
//void getprompt(){
//  Serial.println("enter new value\n");
//  prompt = true;
//}  

void goback(){ //function for moving to the front
 //Serial.println("gofront");
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

void gomiddle(){ //functions for moving to the middle 
  //Serial.println("gomiddle");
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

void gofront(){ //function for moving to the back
  //Serial.println("goback");
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

void ISR_backward(){ //limit switch disables forward movement function
  if (digitalRead(interruptpin1) == HIGH) allowreverse = false;
  else allowreverse=true;
}

void ISR_forward(){
  if (digitalRead(interruptpin2) == HIGH) allowforward=false;
  else allowforward = true;
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

void getsetpoint(){
  if (user_input == "") Serial.print("0\t\t");
  else {Serial.print(user_inputnum); Serial.print("\t\t");}
}



//setup-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup(){
  Serial.begin(500000);

  // Distance Sensor Checks//////////////////////////////////////////////////
  while (! Serial) delay(1); //wait until serial port opens for native USB devices
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()){
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  // Gyroscope Sensor Checks/////////////////////////////////////////////////
  Serial.println("Orientation Sensor Test"); Serial.println("");

  // Initialise the sensor
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
  pinMode(interruptpin1, INPUT);
  pinMode(interruptpin2, INPUT);
  //digitalRead(interruptpin1);
  //digitalRead(interruptpin2);
  attachInterrupt(digitalPinToInterrupt(interruptpin1), ISR_forward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptpin2), ISR_backward, CHANGE); 

  // Current sensor set up
  uint32_t currentFrequency;
  
  /* Initialize the INA219.
   * By default the initialization will use the largest range (32V, 2A).  However
   */you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  //ina219.setCalibration_32V_1A(); //uses a 32V, 1A range (higher precision on amps)
  //ina219.setCalibration_16V_400mA(); //uses a 16V, 400mA range (higher precision on volts and amps)

  //Serial.println("Measuring voltage and current with INA219 ...");

  // Initialize protothreads
  //PT_INIT(&getgyroPT);
  //PT_INIT(&getdistancePT);
  //PT_INIT(&getcurrentPT);
  //PT_INIT(&getinputPT);
  //PT_INIT(&getfsPT);
  //PT_INIT(&getrsPT);
  //PT_INIT(&);
  //PT_INIT(&);
} 



//Loop--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop(){
  // Calling the protothreads in the loop
  setpointprotothread(&getsetpointPT);
  distanceprotothread(&getdistancePT);
  gyroprotothread(&getgyroPT);
  currentprotothread(&getcurrentPT);
  newinputprotothread(&getnewinputPT);
  runforwardprotothread(&forwardrunPT);
  runbackwardprotothread(&backwardrunPT);
  stoprunprotothread(&stoprunPT);
  //newpromptprotothread(&getpromptPT);
  frontprotothread(&gofrontPT);
  middleprotothread(&gomiddlePT);
  backprotothread(&gobackPT);
  //lowbatteryprotothread(&lowbattPT);
}