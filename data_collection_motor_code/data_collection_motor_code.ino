/* Daria Talaski
 *  December 31, 2018
 *  Objective: Output values for distance sensor and gyroscope, and run motor at the same time. 
 
 Edits:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 Maya Clinton 
 *June 11th, 2019
 *Add limit switches with interrupts
  
 *June 13th, 2019
 *Add current sensor code

 *June 17,2019
 *moved sesors into protothreads
 
 *June 18-19, 2019
 *moved motor commands into proto threads. A lot of original code commented out. 
 *added pin reset
 *going to try something with motor code to see if I can get it to read distance take that number then move motor until distance reads a stop number stop distance going to  try for is 500 milimeters
 
 *June 20, 2019
 *fixed bugs now code reads distance sensor values and compares it to user inputs then moves the motor forward or backward in realtion to distance goal and actual distance. Code stops once it reaches distance goal (this part may take a while do to distance sensor fluctuation)   
 *new goal need to get it to stop within a window of the value input
 *took out old code which is saved in motor__protothread2 for clarity with the code and what I am doing 
 
 *June 21-24,2019
 *added protothreads for FRONT, MIDDLE, and BACK
 
 *June 25, 2019
 *added protothread for low battery error message 
 
 
 ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

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
#include <Adafruit_INA219.h>// current sensor library
#include <pt.h> //protothread library



/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void displaySensorDetails(void) //details about the gyo sensor
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

void displaySensorStatus(void) //gyro sensor status
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
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

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
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



//Declare variables for functions
String user_input;
int user_inputnum;
//bool prompt=false;
int distance;
int x;
int y;
int state;

bool front=false;
bool middle=false;
bool back=false;
bool notanumber=false;

// limit switch variable declaration
const byte interruptpin1 = 19; // limit switch 1
const byte interruptpin2 = 18; //limit switch 2
volatile byte allowforward = true; //boolean for forward limit
volatile byte allowreverse = true;// boolean for reverse limit 

// Calling current sensor 
Adafruit_INA219 ina219;

// Current sensor variables (getcurrent)
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

//low battery message variables
  float battery=0;
  float low_battery= false;

//Declaring time (milliseconds) for sensor protothreads
# define timethresh 100





static struct pt getgyroPT, getdistancePT, getcurrentPT, getnewinputPT, forwardrunPT, backwardrunPT, stoprunPT, gofrontPT, gomiddlePT, gobackPT, getsetpointPT;  //lowbattPT;    // decalring protothreads  //getpromptPT,

static int gyroprotothread(struct pt *pt){  //setting up protothread for BNO gyro sensor
  static unsigned long runtime=0; //creates variable runtime 
  PT_BEGIN(pt); // begin protothread
    PT_WAIT_UNTIL(pt,millis()-runtime> timethresh); //trigger for protothread. waits untill the amount of millisecons past minus the runtime is greater than the time we want the operation to be preformed at. 
    getgyro(); //once triggered protothread will run fuction to get gyo sensor data.
    runtime=millis(); //sets runtime to the amount of time past to restart timing
  PT_END(pt); //end protothread 
}

static int distanceprotothread(struct pt *pt){   //setting up protothread for distance sensor
  static unsigned long runtime=0;
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, millis()-runtime> timethresh);
    getdistance(); //once triggered protothread will run fuction to get distance sensor data.
    runtime=millis();
  PT_END(pt);
}

static int currentprotothread(struct pt*pt){ //setting up protothread for current sensor
  static unsigned long runtime=0;
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt,millis()-runtime> timethresh);
    getcurrent(); //once triggered protothread will run fuction to get current sensor data.
    runtime=millis();
  PT_END(pt);
}

static int newinputprotothread(struct pt*pt){  //setting up protothread for user input
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, Serial.available());  // waits until Serial. available meaning an input has been made 
    getnewinput(); //once triggered protothread will run fuction for reading and decifering user input
PT_END(pt);
}


static int runforwardprotothread(struct pt*pt){   //setting up protothread for frontward motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, (distance > user_inputnum)&& (allowforward==true)&& (!(notanumber)));  // waits until the distance of the mass is less than the distance the user gives it, also checks that front limit switch is not pressed and that none of the FRONT, BACK, MIDDLE prototheads read true.
    forwardrun(); //once triggered protothread will run fuction to start forward motor run
PT_END(pt);
}

static int runbackwardprotothread(struct pt*pt){   //setting up protothread for backward motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt,(distance < user_inputnum )&& (allowreverse==true));  // waits until the distance of the mass is greater than the distance the user inputs. Also checks that the back limit switch is not pressed.  
    backwardrun(); //once triggered protothread will run fuction to start backward motor run
PT_END(pt);
}

static int stoprunprotothread(struct pt*pt){   //setting up protothread for stopping motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt,(user_inputnum+1>=distance)&&(distance>=user_inputnum-1));  // waits until the distance the user inputs is either equal to or within 1 millimeter range from the distance the user gives.  
    stoprun(); //once triggered protothread will run fuction to stop motor step
PT_END(pt);
}



//static int newpromptprotothread(struct pt*pt){   //setting up protothread for forward motor step
//  PT_BEGIN(pt);
//    PT_WAIT_UNTIL(pt,!prompt && !(allowforward && allowreverse));// waits until there is a value for x_forward in this case 1000
//    getprompt(); //once triggered protothread will run fuction to start forward motor step
//PT_END(pt);
//}


static int frontprotothread(struct pt*pt){   //setting up protothread for user input "FRONT"
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, (front==true) );// waits until the bool front is true (user inputs "FRONT")
    gofront(); //once triggered protothread will run fuction to move to the "front"
PT_END(pt);
}


static int middleprotothread(struct pt*pt){   //setting up protothread for user input "MIDDLE"
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, (middle==true));// waits until middle bool is true (user inputs "MIDDLE")
    gomiddle(); //once triggered protothread will run fuction to move to the "middle"
PT_END(pt);
}


static int backprotothread(struct pt*pt){   //setting up protothread for user input "BACK"
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, (back==true));// waits until back bool is true (user inputs "BACK")
    goback(); //once triggered protothread will run fuction to move to the "back"
    PT_END(pt);
}

/*
static int lowbatteryprotothread(struct pt*pt){   //setting up protothread for low battery
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, (low_battery==true) );// waits until low_battery bool is true 
    lowbatt(); //once triggered protothread will run low battery error messsage
PT_END(pt);
}
*/
static int setpointprotothread(struct pt *pt){  //setting up protothread for BNO gyro sensor
  static unsigned long runtime=0; //creates variable runtime 
  PT_BEGIN(pt); // begin protothread
    PT_WAIT_UNTIL(pt,millis()-runtime> timethresh); //trigger for protothread. waits untill the amount of millisecons past minus the runtime is greater than the time we want the operation to be preformed at. 
    getsetpoint(); //once triggered protothread will run fuction to get gyo sensor data.
    runtime=millis(); //sets runtime to the amount of time past to restart timing
  PT_END(pt); //end protothread 
}







//Functions-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void resetBEDPins(){ //Reset Big Easy Driver pins to default states
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(EN, HIGH);
}



void getdistance(){  //fucntion to grab and display distance sensor data
VL53L0X_RangingMeasurementData_t measure;

 lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

if (measure.RangeStatus != 4) {  // phase failures have incorrect data
  distance = measure.RangeMilliMeter;  //sets distance variable to the data the distance sensor measures
   Serial.print(distance); Serial.print("\t\t\t");// prints that distance
}
else{
  Serial.print(" out of range ");Serial.print("\t\t\t"); // error message if distance too far for sensor to read
}

}




void getgyro(){  //fucntion to grab and display BNO gyro sensor data
 sensors_event_t event; //grabs sensor event
  bno.getEvent(&event); 
  
    Serial.print(event.orientation.x, 4); Serial.print("\t\t\t"); //prints x position data
    Serial.print(event.orientation.y, 4); Serial.print("\t\t\t"); //prints y position data
    Serial.print(event.orientation.z, 4); Serial.print("\t\t\t"); //prints z position data
   // Serial.println(""); 
  
}



void getcurrent(){   //fucntion to grab and display current sensor data
  //Current sensor loop code  // this code was moved up
// float shuntvoltage = 0;
//  float busvoltage = 0;
//  float current_mA = 0;
//  float loadvoltage = 0;
//  float power_mW = 0;

//grabbing data for current sensor 
  shuntvoltage = ina219.getShuntVoltage_mV(); 
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000); // calculation for load voltage

  //outputs sensor data
   //Serial.println("");Serial.print("\t\t\t\t\t\t\t");
//  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");Serial.print("\t\t\t\t\t\t\t");
//  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");Serial.print("\t\t\t\t\t\t\t");
//  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");Serial.print("\t\t\t\t\t\t\t");
//  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");Serial.print("\t\t\t\t\t\t\t");
//  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");Serial.print("\t\t\t\t\t\t\t");
//  Serial.println("");
 Serial.print(busvoltage); Serial.print("\t\t\t");
 Serial.print(shuntvoltage); Serial.print("\t\t\t");
 Serial.print(loadvoltage); Serial.print("\t\t\t");
 Serial.print(current_mA); Serial.print("\t\t\t");
 Serial.print(power_mW); Serial.print("\t\t\t");
 Serial.println(" ");

//setup for low battery error message
  battery= busvoltage;

  if(battery <9.00){
    low_battery=true;
  }
  else{
    low_battery=false;
  }
  


}



void getnewinput(){  //Function for user input 
  user_input=""; //clears uesr input at the start of each
   while(Serial.available()){   
      user_input += (char) Serial.read(); //Reads user input adds each character to build larger string (this is because other wise only last value would be read)   
  }
//sets bools to false
Serial.print(user_input); Serial.print("\t");   
front=false; 
back=false;
middle=false;
digitalWrite(EN, LOW);
user_inputnum=user_input.toInt(); //takes the string user input and turns it into an interger so that it can be compared to the distace which is an interger. this it then called user input number

if (user_inputnum==0){  //when letters given .toInt function returns a zero. so if a zero is returned it follows the following statments.
if (user_input.equals ("FRONT")){  //if the user input is FRONT then sets bool front to true (done with .equals because it is a string)
front=true;
}
else if (user_input.equals ("MIDDLE")){  //if the user input is MIDDLE then sets bool middle to true
middle=true;
}

else if (user_input.equals ("BACK")){ //if the user input is BACK then sets bool back to true
back=true;

}

else{
Serial.print("invalid input"); //other wise any other letter input returns error 
digitalWrite(EN, HIGH);

}

}
notanumber=(front || middle|| back); // creates variable not a number so that these are not read at the very begining.
}


void forwardrun(){   //function for forward motion
//Serial.println("forwardrun");  
digitalWrite(dir, LOW); //low direction means forward
digitalWrite(stp, HIGH);
delay(1);
digitalWrite(stp, LOW);
delay(1);

 }

void backwardrun(){ //function for backward motion
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
//  
//void getprompt(){
//Serial.println("enter new value");
//Serial.println(" ");
//prompt=true;
//}  

void gofront(){  //function for movint to the front
 //Serial.println("gofront");
if((distance > 400)){    //if distance greater than 400mm it will move forward until it reaches 400mm
digitalWrite(dir, LOW);
digitalWrite(stp, HIGH);
delay(1);
digitalWrite(stp, LOW);
delay(1);
}
else{    //otherwise front is false
  Serial.println("else ft");
    front=false;

}

}





void gomiddle(){  //functions for moving to the middle 
//Serial.println("gomiddle");
if((distance> 500)){  //if distance is greater then 500mm will move forward until reaches 500mm
digitalWrite(dir, LOW);
digitalWrite(stp, HIGH);
delay(1);
digitalWrite(stp, LOW);
delay(1);
}
else if((distance <500)){   //if distance less than 500 will move backward until reaches 500
 digitalWrite(dir, HIGH);
digitalWrite(stp, HIGH);
delay(1);
digitalWrite(stp, LOW);
delay(1); 
}
else{   // otherwise middle is false
  Serial.println("else md");
  middle=false;
 
  
  

}
 
}



void goback(){  //function for moving to the back
//Serial.println("goback");
if((distance <650)){    //if distance less than 650 the motor will move back untill it reaches 650
digitalWrite(dir, HIGH);
digitalWrite(stp, HIGH);
delay(1);
digitalWrite(stp, LOW);
delay(1);
}
else{
  Serial.println("else bk"); //otherwise bool back is false 
  back=false;
  
 


}

}


void lowbatt(){    //function to print out low battery message 
Serial.println("low battery! Please replace. "); Serial.print("\t\t\t\t\t\t\t");
}




void ISR_forward(){   //forward interrupt function
if(digitalRead(interruptpin1)==HIGH){ //if limit switch pressed
  allowforward=false;  //forward motion not allowed
    
}
else{
  allowforward=true;  //forward motion allowed
}
}

void ISR_reverse(){ //Backward interrupt function
if(digitalRead(interruptpin2)==HIGH){  //if limit switch pressed
  allowreverse=false; //backward motion not allowed 
}
else{
  allowreverse=true;   //backward motion allowed
}
}


void header(){
Serial.println("-----------------------------------------\n");
  Serial.println("Begin motor control");
  Serial.println("Enter number between 300 and 700 millimeters");
Serial.println("-----------------------------------------\n");
    Serial.print("Set point"); Serial.print("\t");
    Serial.print("Distance(mm)"); Serial.print("\t\t");
    Serial.print("X"); Serial.print("\t\t\t\t");
    Serial.print("Y"); Serial.print("\t\t\t");
    Serial.print("Z"); Serial.print("\t\t\t");
  Serial.print("Bus Vol(V)"); Serial.print("\t\t\t");
  Serial.print("Shunt Vol(mV)"); Serial.print("\t\t");
  Serial.print("Load Vol(V)"); Serial.print("\t\t");
  Serial.print("Current(mA)");  Serial.print("\t\t");
  Serial.print("Power(mW)");  
  Serial.println("");
  Serial.println("-----------------------------------------\n");
  Serial.println("");
}

void getsetpoint(){
if (user_input== ""){
Serial.print("0"); Serial.print("\t\t");
}
else {
  Serial.print(user_inputnum); Serial.print("\t\t");
}
}



//setup-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
   Serial.begin(500000);

   // Distance Sensor Checks//////////////////////////////////////////////////
     // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  // Gyroscope Sensor Checks/////////////////////////////////////////////////
    Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

  header(); // creating header

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
  
//
//  Serial.println("Begin motor control");
//  Serial.println(" ");
//  Serial.println("Enter number between 300 and 700 millimeters");
//  Serial.println("");
//
//
//// Table Headers
//Serial.println("-----------------------------------------\n");
//    Serial.print("Distance (mm)"); Serial.print("\t");
//    Serial.print("X"); Serial.print("\t");
//  Serial.print("Y"); Serial.print("\t");
//  Serial.print("Z"); Serial.print("\t");
//  Serial.print("Bus Voltage (V)");  Serial.print("\t");
//  Serial.print("Shunt Voltage(mV)"); Serial.print("\t");
//  Serial.print("Load Voltage(V)"); Serial.print("\t");
//  Serial.print("Current (mA) ");  Serial.print("\t");
//  Serial.print("Power (mW)");  Serial.print("\t");
//  Serial.println("");

    




//limit switch setup

pinMode(interruptpin1, INPUT);
pinMode(interruptpin2, INPUT);
//digitalRead(interruptpin1);
//digitalRead(interruptpin2);
attachInterrupt(digitalPinToInterrupt(interruptpin1), ISR_forward, CHANGE);
attachInterrupt(digitalPinToInterrupt(interruptpin2), ISR_reverse, CHANGE); 



//Current sensor set up

  uint32_t currentFrequency;
 // Serial.print("  ");Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t");  
  //Serial.println("Hello!");Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t");
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
 // ina219.setCalibration_16V_400mA();

  //Serial.println("Measuring voltage and current with INA219 ...");

// protothread
  //PT_INIT(&getgyroPT);
 // PT_INIT(&getdistancePT);
  //PT_INIT(&getcurrentPT);
  //PT_INIT(&getinputPT);
  //PT_INIT(&getfsPT);
  //PT_INIT(&getrsPT);
  //PT_INIT(&);
  //PT_INIT(&);
  
 

   
  } 


//Loop--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {

  //calling the protothreads in the loop
  setpointprotothread(&getsetpointPT);
  distanceprotothread(&getdistancePT);
  gyroprotothread(&getgyroPT);
  currentprotothread(&getcurrentPT);
  newinputprotothread(&getnewinputPT);
  runforwardprotothread(&forwardrunPT);
  runbackwardprotothread(&backwardrunPT);
  stoprunprotothread(&stoprunPT);
 // newpromptprotothread(&getpromptPT);
  frontprotothread(&gofrontPT);
  middleprotothread(&gomiddlePT);
  backprotothread(&gobackPT);
 // lowbatteryprotothread(&lowbattPT);

 }
