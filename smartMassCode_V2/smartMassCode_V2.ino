/* Daria Talaski
 *  August 21, 2019
 *  Objective: take set point, move forward or backwards, use limit switches, send position, orientation, current data 
 
 Edits:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(F(sensor.name));
  Serial.print  (F("Driver Ver:   ")); Serial.println(F(sensor.version));
  Serial.print  (F("Unique ID:    ")); Serial.println(F(sensor.sensor_id));
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" xxx"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" xxx"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" xxx"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

void displaySensorStatus(void) //gyro sensor status
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println(F(""));
  Serial.print(F("System Status: 0x"));
  Serial.println(system_status, HEX);
  Serial.print(F("Self Test:     0x"));
  Serial.println(self_test_results, HEX);
  Serial.print(F("System Error:  0x"));
  Serial.println(system_error, HEX);
  Serial.println(F(""));
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
  Serial.print(F("\t"));
  if (!system)
  {
    Serial.print(F("! "));
  }

  /* Display the individual values */
  Serial.print(F("Sys:"));
  Serial.print(system, DEC);
  Serial.print(F(" G:"));
  Serial.print(gyro, DEC);
  Serial.print(F(" A:"));
  Serial.print(accel, DEC);
  Serial.print(F(" M:"));
  Serial.print(mag, DEC);
}

// Header for Motor Driver //////////////////////////////////////////////////////////////////////////////
//Declare pin functions on Arduino
#define stp 6
#define dir 9
#define MS1 12
#define MS2 11
#define MS3 10
#define EN  13



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


// Calling current sensor 
Adafruit_INA219 ina219;

// Current sensor variables (getcurrent)
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

//Declaring time (milliseconds) for sensor protothreads
# define timethresh 100


static struct pt getgyroPT, getdistancePT, getcurrentPT, getnewinputPT, forwardrunPT, backwardrunPT, stoprunPT;   // decalring protothreads  //getpromptPT,

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
    PT_WAIT_UNTIL(pt, (distance > user_inputnum)&& (!(notanumber)));  // waits until the distance of the mass is less than the distance the user gives it, also checks that front limit switch is not pressed 
    forwardrun(); //once triggered protothread will run fuction to start forward motor run
PT_END(pt);
}

static int runbackwardprotothread(struct pt*pt){   //setting up protothread for backward motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt,(distance < user_inputnum ));  // waits until the distance of the mass is greater than the distance the user inputs. Also checks that the back limit switch is not pressed.  
    backwardrun(); //once triggered protothread will run fuction to start backward motor run
PT_END(pt);
}

static int stoprunprotothread(struct pt*pt){   //setting up protothread for stopping motor run
  PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt,(user_inputnum+1>=distance)&&(distance>=user_inputnum-1));  // waits until the distance the user inputs is either equal to or within 1 millimeter range from the distance the user gives.  
    stoprun(); //once triggered protothread will run fuction to stop motor step
PT_END(pt);
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


void getgyro(){  //fucntion to grab and display BNO gyro sensor data
 sensors_event_t event; //grabs sensor event
  bno.getEvent(&event); 
  
    Serial.print(event.orientation.x, 4); Serial.print(F("\t")); //prints x position data
    Serial.print(event.orientation.y, 4); Serial.print(F("\t")); //prints y position data
    Serial.print(event.orientation.z, 4); Serial.print(F("\t")); //prints z position data
    Serial.println(F("")); 
  
}

void getdistance(){  //fucntion to grab and display distance sensor data
VL53L0X_RangingMeasurementData_t measure;

 lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

if (measure.RangeStatus != 4) {  // phase failures have incorrect data
  distance = measure.RangeMilliMeter;  //sets distance variable to the data the distance sensor measures
   Serial.print(distance); Serial.print(F("\t"));// prints that distance
}
else{
  Serial.print(F(" out of range "));Serial.print(F("\t")); // error message if distance too far for sensor to read
}

}



void getcurrent(){   //fucntion to grab and display current sensor data

//grabbing data for current sensor 
  shuntvoltage = ina219.getShuntVoltage_mV(); 
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000); // calculation for load voltage

  //outputs sensor data
   //Serial.println("");//Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t");
  Serial.print(F("Bus Voltage:   ")); Serial.print(busvoltage); Serial.println(F(" V"));
  Serial.print(F("Shunt Voltage: ")); Serial.print(shuntvoltage); Serial.println(F(" mV"));
  Serial.print(F("Load Voltage:  ")); Serial.print(loadvoltage); Serial.println(F(" V"));
  Serial.print(F("Current:       ")); Serial.print(current_mA); Serial.println(F(" mA"));
  Serial.print(F("Power:         ")); Serial.print(power_mW); Serial.println(F(" mW"));
  Serial.println(F("")); 


}



void getnewinput(){  //Function for user input 
  user_input=""; //clears uesr input at the start of each
   while(Serial.available()){ 
      user_input += (char) Serial.read(); //Reads user input adds each character to build larger string (this is because other wise only last value would be read)
  }
//sets bools to false   
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
Serial.print(F("invalid input")); //other wise any other letter input returns error 
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




//setup-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
   Serial.begin(2000000);

   // Distance Sensor Checks//////////////////////////////////////////////////
     // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println(F("Adafruit VL53L0X test"));
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  // Gyroscope Sensor Checks/////////////////////////////////////////////////
    Serial.println(F("Orientation Sensor Test")); Serial.println(F(""));

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

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
  

  Serial.println(F("Begin motor control"));
  Serial.println();
  //Print function list for user selection
  Serial.println(F("Enter number between 300 and 700 millimeters"));


// Table Headers
Serial.print(F("-----------------------------------------\n"));
    Serial.print(F("Distance")); Serial.print(F("\t"));
    Serial.print(F("X")); Serial.print(F("\t"));
    Serial.print(F("Y")); Serial.print(F("\t"));
    Serial.print(F("Z")); Serial.print(F("\t"));
    Serial.println(F(""));



//Current sensor set up

  uint32_t currentFrequency;
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
 // ina219.setCalibration_16V_400mA();

  Serial.println(F("Measuring voltage and current with INA219 ..."));
  
 

   
  } 


//Loop--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {

  //calling the protothreads in the loop
  gyroprotothread(&getgyroPT);
  distanceprotothread(&getdistancePT);
  currentprotothread(&getcurrentPT);
  newinputprotothread(&getnewinputPT);
  runforwardprotothread(&forwardrunPT);
  runbackwardprotothread(&backwardrunPT);
  stoprunprotothread(&stoprunPT);

 }




