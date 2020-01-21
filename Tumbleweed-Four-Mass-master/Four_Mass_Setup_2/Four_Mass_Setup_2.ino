// PAURAV; FOUR MASS TUMBLEWEED ROLLING
// 24th JAN, 2018
// Initial Basic code for verifying Stepper and LIDAR working
// Accelerometer, XBee, Limit Switches not incorporated

#include <Wire.h>
#include <LIDARLite.h>
#include <stdarg.h>
#include <math.h>  

// Setpoint Angular Velocity
//*******************************

// Controller Physical Constants
//*******************************

// Defining Motor Driver Pins (Motors 1 to 4)
// Motor 1
#define dir1                    (24)      //Direction
#define stp1                    (26)      //Step
#define EN1                     (51)      //Enable
#define MS1_1                   (25)      //Finer Motor control
#define MS2_1                   (27)      //Finer Motor control
#define MS3_1                   (29)      //Finer Motor control
//Motor 2
#define dir2                    (52)
#define stp2                    (53)
#define EN2                     (34)
#define MS1_2                   (31)
#define MS2_2                   (33)
#define MS3_2                   (35)
 // Motor 3
#define dir3                    (36)
#define stp3                    (38)
#define EN3                     (40)
#define MS1_3                   (37)
#define MS2_3                   (39)
#define MS3_3                   (41)
 // Motor 4
#define dir4                    (42)
#define stp4                    (44)
#define EN4                     (46)
#define MS1_4                   (43)
#define MS2_4                   (45)
#define MS3_4                   (47)


// LIDAR Definitions
#define ZERO_ENABLE               (53) // port 54, may be left unused
#define ONE_ENABLE                (28) // port 51
#define TWO_ENABLE                (30) // port 52
#define THREE_ENABLE              (32) // port 53
#define LIDAR_ON                  (0)  // lidar power enabled
#define LIDAR_OFF                 (1)  // lidar power disabled
#define DEFAULT_LIDAR_ADDR        (0x62)
#define LIDAR1_ADDR               (0x64)
#define LIDAR2_ADDR               (0x42)
#define LIDAR3_ADDR               (0x32)
#define LIDAR4_ADDR               (0x52)
#define LIDAR1_OFFSET              33  // MEASURE AND CHANGE LATER
#define LIDAR2_OFFSET              33  // MEASURE AND CHANGE LATER
#define LIDAR3_OFFSET              33  // MEASURE AND CHANGE LATER
#define LIDAR4_OFFSET              33  // MEASURE AND CHANGE LATER
#define LIDAR_MODE                  2 // default range, faster acquisition
#define LIDAR_FILT                  0.87


// Controller Definitions (PID)
//******************************

// Accelerometer Definitions
//******************************

// Raw Ranges for Accelerometer
//******************************

// Structures

//Lidar Struct
struct lidarInfo
{
  LIDARLite interface;
  float d;                          // Distance 
  float o;                          // Offset
  uint8_t serial_high;              
  uint8_t serial_low;
  uint8_t address;
};

//Motor Info Struct
struct motorInfo
{
  int step_val;
  int step_pin;
  int dir_val;
  int dir_pin;
  int en_val;
  int en_pin;
  int MS1_val;
  int MS1_pin;
  int MS2_val;
  int MS2_pin;
  int MS3_val;
  int MS3_pin;
};

//PID InfoStruct
//***************

//Angle Data Struct
//***************

//Global Variables
lidarInfo lidar1;
lidarInfo lidar2;
lidarInfo lidar3;
lidarInfo lidar4;
motorInfo motor1;
motorInfo motor2;
motorInfo motor3;
motorInfo motor4;

byte temp[2];
//****************

void setup() 
{
    Serial.begin(19200);

 // init_motors();
  init_lidars();
//  init_accelerometer();
//  init_pid_controls(pid1, pid2);
//  init_angular_data(angular_data);
  delay(1000);

}

void loop()
{
read_lidars();  // gets and reads lidars
}

void init_motors()
{
  // setup motor control pins
  pinMode(dir1, OUTPUT);
  pinMode(stp1, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(MS1_1, OUTPUT);
  pinMode(MS2_1, OUTPUT);
  pinMode(MS3_1, OUTPUT);

  pinMode(dir2, OUTPUT);
  pinMode(stp2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(MS1_2, OUTPUT);
  pinMode(MS2_2, OUTPUT);
  pinMode(MS3_2, OUTPUT);
  
  pinMode(dir3, OUTPUT);
  pinMode(stp3, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(MS1_3, OUTPUT);
  pinMode(MS2_3, OUTPUT);
  pinMode(MS3_3, OUTPUT);
  
  pinMode(dir4, OUTPUT);
  pinMode(stp4, OUTPUT);
  pinMode(EN4, OUTPUT);
  pinMode(MS1_4, OUTPUT);
  pinMode(MS2_4, OUTPUT);
  pinMode(MS3_4, OUTPUT);

 // resetBEDPins(); //Set step, direction, microstep and enable pins to default states

  motor1.step_pin = stp1;
  motor1.step_val = LOW;
  motor1.dir_pin  = dir1;
  motor1.dir_val  = HIGH;
  motor1.en_val   = LOW;
  motor1.en_pin   = EN1;
  motor1.MS1_val  = LOW;
  motor1.MS1_pin  = MS1_1;
  motor1.MS2_val  = HIGH;
  motor1.MS2_pin  = MS2_1;
  motor1.MS3_val  = LOW;
  motor1.MS3_pin  = MS3_1;

  
  digitalWrite(motor1.step_pin, motor1.step_val);
  digitalWrite(motor1.dir_pin, motor1.dir_val);
  digitalWrite(motor1.en_pin, motor1.en_val);
  digitalWrite(motor1.MS1_pin, motor1.MS1_val);
  digitalWrite(motor1.MS2_pin, motor1.MS2_val);
  digitalWrite(motor1.MS3_pin, motor1.MS3_val);

  motor2.step_pin = stp2;
  motor2.step_val = LOW;
  motor2.dir_pin  = dir2;
  motor2.dir_val  = HIGH;
  motor2.en_val   = LOW;
  motor2.en_pin   = EN2;
  motor2.MS1_val  = LOW;
  motor2.MS1_pin  = MS1_2;
  motor2.MS2_val  = HIGH;
  motor2.MS2_pin  = MS2_2;
  motor2.MS3_val  = LOW;
  motor2.MS3_pin  = MS3_2;

  
  digitalWrite(motor2.step_pin, motor2.step_val);
  digitalWrite(motor2.dir_pin, motor2.dir_val);
  digitalWrite(motor2.en_pin, motor2.en_val);
  digitalWrite(motor2.MS1_pin, motor2.MS1_val);
  digitalWrite(motor2.MS2_pin, motor2.MS2_val);
  digitalWrite(motor2.MS3_pin, motor2.MS3_val);

  motor3.step_pin = stp3;
  motor3.step_val = LOW;
  motor3.dir_pin  = dir3;
  motor3.dir_val  = HIGH;
  motor3.en_val   = LOW;
  motor3.en_pin   = EN3;
  motor3.MS1_val  = LOW;
  motor3.MS1_pin  = MS1_3;
  motor3.MS2_val  = HIGH;
  motor3.MS2_pin  = MS2_3;
  motor3.MS3_val  = LOW;
  motor3.MS3_pin  = MS3_3;

  
  digitalWrite(motor3.step_pin, motor3.step_val);
  digitalWrite(motor3.dir_pin, motor3.dir_val);
  digitalWrite(motor3.en_pin, motor3.en_val);
  digitalWrite(motor3.MS1_pin, motor3.MS1_val);
  digitalWrite(motor3.MS2_pin, motor3.MS2_val);
  digitalWrite(motor3.MS3_pin, motor3.MS3_val);

  motor4.step_pin = stp4;
  motor4.step_val = LOW;
  motor4.dir_pin  = dir4;
  motor4.dir_val  = HIGH;
  motor4.en_val   = LOW;
  motor4.en_pin   = EN4;
  motor4.MS1_val  = LOW;
  motor4.MS1_pin  = MS1_4;
  motor4.MS2_val  = HIGH;
  motor4.MS2_pin  = MS2_4;
  motor4.MS3_val  = LOW;
  motor4.MS3_pin  = MS3_4;

  
  digitalWrite(motor4.step_pin, motor4.step_val);
  digitalWrite(motor4.dir_pin, motor4.dir_val);
  digitalWrite(motor4.en_pin, motor4.en_val);
  digitalWrite(motor4.MS1_pin, motor4.MS1_val);
  digitalWrite(motor4.MS2_pin, motor4.MS2_val);
  digitalWrite(motor4.MS3_pin, motor4.MS3_val);
  
}


void init_lidars() 
{

  Serial.println("Initializing LIDARS");
  pinMode(ZERO_ENABLE, OUTPUT);
  pinMode(ONE_ENABLE, OUTPUT);
  pinMode(TWO_ENABLE, OUTPUT);
  pinMode(THREE_ENABLE, OUTPUT);

   
  digitalWrite(ZERO_ENABLE, LIDAR_ON);
  digitalWrite(ONE_ENABLE, LIDAR_ON);
  digitalWrite(TWO_ENABLE, LIDAR_ON);
  digitalWrite(THREE_ENABLE, LIDAR_ON);
    
  delay(1000);

  // disable all lidars
  digitalWrite(ZERO_ENABLE, LIDAR_OFF);
  digitalWrite(ONE_ENABLE, LIDAR_OFF);
  digitalWrite(TWO_ENABLE, LIDAR_OFF);
  digitalWrite(THREE_ENABLE, LIDAR_OFF);
  delay(5); // allow devices to power down gracefully

  // power the first lidar
  digitalWrite(ZERO_ENABLE, LIDAR_ON);
  delay(25); // wait for device to come online

  lidar1.interface.begin(0, true, DEFAULT_LIDAR_ADDR);

  // does not seem to get the actual address
  lidar1.interface.read(0x16, 2, temp, false, DEFAULT_LIDAR_ADDR); // read high serial byte
  Serial.print(temp[0]); Serial.print(temp[1]);
  lidar1.serial_high = temp[0];
  lidar1.interface.read(0x17, 2, temp, false, DEFAULT_LIDAR_ADDR); // read low serial byte
  Serial.print(temp[0]); Serial.print(temp[1]);
  lidar1.serial_low = temp[0];


  // seems to be required
  lidar1.interface.write(0x18, lidar1.serial_high, DEFAULT_LIDAR_ADDR); // write high serial byte
  lidar1.interface.write(0x19, lidar1.serial_low, DEFAULT_LIDAR_ADDR); // write low serial byte to unlock


  lidar1.interface.write(0x1a, LIDAR1_ADDR, DEFAULT_LIDAR_ADDR); // set desired address
  lidar1.interface.write(0x1e, 0x08, DEFAULT_LIDAR_ADDR);  // disable default address

  delay(1000);



  
//Serial.println("DISTANCE 1");
// Serial.println(lidar1.d);
  // power the second lidar
  digitalWrite(ONE_ENABLE, LIDAR_ON);

  // wait at least 25 ms
  delay(25);
  lidar2.interface.begin(0, true, DEFAULT_LIDAR_ADDR);

  lidar2.interface.read(0x16, 2, temp, false, DEFAULT_LIDAR_ADDR); // read high serial byte
  Serial.print(temp[0]); Serial.print(temp[1]);
  lidar2.serial_high = temp[0];
  lidar2.interface.read(0x17, 2, temp, false, DEFAULT_LIDAR_ADDR); // read low serial byte
  Serial.print(temp[0]); Serial.print(temp[1]);
  lidar2.serial_low = temp[0];


  // seems to be required
  lidar2.interface.write(0x18, lidar2.serial_high, DEFAULT_LIDAR_ADDR); // write high serial byte
  lidar2.interface.write(0x19, lidar2.serial_low, DEFAULT_LIDAR_ADDR); // write low serial byte to unlock


  lidar2.interface.write(0x1a, LIDAR2_ADDR, DEFAULT_LIDAR_ADDR); // set desired address
  lidar2.interface.write(0x1e, 0x08, DEFAULT_LIDAR_ADDR);  // disable default address

  
 delay(1000);

  digitalWrite(TWO_ENABLE, LIDAR_ON);

  // wait at least 25 ms
  delay(25);
  lidar3.interface.begin(0, true, DEFAULT_LIDAR_ADDR);

  lidar3.interface.read(0x16, 2, temp, false, DEFAULT_LIDAR_ADDR); // read high serial byte
  Serial.print(temp[0]); Serial.print(temp[1]);
  lidar3.serial_high = temp[0];
  lidar3.interface.read(0x17, 2, temp, false, DEFAULT_LIDAR_ADDR); // read low serial byte
  Serial.print(temp[0]); Serial.print(temp[1]);
  lidar3.serial_low = temp[0];


  // seems to be required
  lidar3.interface.write(0x18, lidar3.serial_high, DEFAULT_LIDAR_ADDR); // write high serial byte
  lidar3.interface.write(0x19, lidar3.serial_low, DEFAULT_LIDAR_ADDR); // write low serial byte to unlock


  lidar3.interface.write(0x1a, LIDAR3_ADDR, DEFAULT_LIDAR_ADDR); // set desired address
  lidar3.interface.write(0x1e, 0x08, DEFAULT_LIDAR_ADDR);  // disable default address

  
 delay(1000);

   digitalWrite(THREE_ENABLE, LIDAR_ON);

  // wait at least 25 ms
  delay(25);
  lidar4.interface.begin(0, true, DEFAULT_LIDAR_ADDR);

  lidar4.interface.read(0x16, 2, temp, false, DEFAULT_LIDAR_ADDR); // read high serial byte
  Serial.print(temp[0]); Serial.print(temp[1]);
  lidar4.serial_high = temp[0];
  lidar4.interface.read(0x17, 2, temp, false, DEFAULT_LIDAR_ADDR); // read low serial byte
  Serial.print(temp[0]); Serial.print(temp[1]);
  lidar4.serial_low = temp[0];


  // seems to be required
  lidar4.interface.write(0x18, lidar4.serial_high, DEFAULT_LIDAR_ADDR); // write high serial byte
  lidar4.interface.write(0x19, lidar4.serial_low, DEFAULT_LIDAR_ADDR); // write low serial byte to unlock


  lidar4.interface.write(0x1a, LIDAR4_ADDR, DEFAULT_LIDAR_ADDR); // set desired address
  lidar4.interface.write(0x1e, 0x08, DEFAULT_LIDAR_ADDR);  // disable default address

  
 delay(1000);

 

  lidar1.interface.configure(LIDAR_MODE, LIDAR1_ADDR);
  lidar2.interface.configure(LIDAR_MODE, LIDAR2_ADDR);
  lidar3.interface.configure(LIDAR_MODE, LIDAR3_ADDR);
  lidar4.interface.configure(LIDAR_MODE, LIDAR4_ADDR);

  lidar1.o = LIDAR1_OFFSET;
  lidar2.o = LIDAR2_OFFSET;
  lidar3.o = LIDAR3_OFFSET;
  lidar4.o = LIDAR4_OFFSET;

  delay(15);
}

void read_lidars()
{
  static unsigned int count = 0;
  unsigned int time1 = 0;
  unsigned int time2 = 0;
  unsigned int time3 = 0;
  unsigned int time4 = 0;
  unsigned int timec = millis();

  if (++count > 100)
  {
    count = 0;

    //lidar1.d = lidar1.interface.distance(true, LIDAR1_ADDR);
    lidar1.d = (1 - LIDAR_FILT) * (lidar1.interface.distance(true, LIDAR1_ADDR) - lidar1.o) + LIDAR_FILT * lidar1.d;
    time1 = millis();

    //lidar2.d = lidar2.interface.distance(true, LIDAR2_ADDR);
    lidar2.d = (1 - LIDAR_FILT) * (lidar2.interface.distance(true, LIDAR2_ADDR) - lidar2.o) + LIDAR_FILT * lidar2.d;
    time2 = millis();

//    //lidar3.d = lidar3.interface.distance(true, LIDAR3_ADDR);
    lidar3.d = (1 - LIDAR_FILT) * (lidar3.interface.distance(true, LIDAR3_ADDR) - lidar3.o) + LIDAR_FILT * lidar3.d;
    time3 = millis();
//
//    //lidar4.d = lidar4.interface.distance(true, LIDAR4_ADDR);
    lidar4.d = (1 - LIDAR_FILT) * (lidar4.interface.distance(true, LIDAR4_ADDR) - lidar4.o) + LIDAR_FILT * lidar4.d;
     time4 = millis();
  }
  else
  {
    //lidar1.d = lidar1.interface.distance(false, LIDAR1_ADDR);
    lidar1.d = (1 - LIDAR_FILT) * (lidar1.interface.distance(false, LIDAR1_ADDR) - lidar1.o) + LIDAR_FILT * lidar1.d;
    time1 = millis();

    //lidar2.d = lidar2.interface.distance(false, LIDAR2_ADDR);
    lidar2.d = (1 - LIDAR_FILT) * (lidar2.interface.distance(false, LIDAR2_ADDR) - lidar2.o) + LIDAR_FILT * lidar2.d;
    time2 = millis();

//    //lidar3.d = lidar3.interface.distance(true, LIDAR3_ADDR);
    lidar3.d = (1 - LIDAR_FILT) * (lidar3.interface.distance(false, LIDAR3_ADDR) - lidar3.o) + LIDAR_FILT * lidar3.d;
    time3 = millis();
//
//    //lidar4.d = lidar4.interface.distance(true, LIDAR4_ADDR);
    lidar4.d = (1 - LIDAR_FILT) * (lidar4.interface.distance(false, LIDAR4_ADDR) - lidar4.o) + LIDAR_FILT * lidar4.d;
    time4 = millis();
  }



  Serial.print("Count = "); Serial.print(count); Serial.print('\t');
  Serial.print("T0 = "); Serial.print(timec); Serial.print('\t');
  Serial.print("D1 = "); Serial.print(lidar1.d); /*Serial.print('\t'); Serial.print("T1 = "); Serial.print(time1 - timec);*/ Serial.print('\t');
  Serial.print("D2 = "); Serial.print(lidar2.d); /*Serial.print('\t'); Serial.print("T2 = "); Serial.print(time2 - time1);*/ Serial.print('\t');
  Serial.print("D3 = "); Serial.print(lidar3.d); /*Serial.print('\t'); Serial.print("T3 = "); Serial.print(time3 - time2); */Serial.print('\t');
  Serial.print("D4 = "); Serial.print(lidar4.d); /*Serial.print('\t'); Serial.print("T2 = "); Serial.print(time4 - time3); */Serial.print('\n');
}



//void init_pid_controls(pidInfo &p1, pidInfo &p2)
//***********************************************

//void control_mass(pidInfo &control, lidarInfo &sensor, motorInfo &motor)
//***********************************************************************

//void init_angular_data(angleData &data)
//***********************************

//void calculate_angular_data()
//*************************

//void init accelerometer()
//**********************

//void accell()
//*********************

//int readaxis(int axisPins)
//*************************

// void setpoint_from_angle()
//**************************

// void printfunc()
// ******************






































