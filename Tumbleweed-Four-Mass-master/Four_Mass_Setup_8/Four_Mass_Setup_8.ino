// PAURAV; FOUR MASS TUMBLEWEED ROLLING
// 3rd Aug, 2019
// Main Code
// With Accelerometer and Limit Switches

//************#### Determine Motor direction

//#include <AccelStepper.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <stdarg.h>
#include <math.h>  
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

int k;
float temp_ang;
//=============================CHANGE VASRIABLES HERE========================================================================

#define SP_angvel                60.00f  // deg/s                  // Setpoint Angular Velocity
#define velocity_source          1       // 1 = calc from fused angle data        2 = raw gyro velocity
#define K_const                  1.75f   // 2 //2.25 //2.5 //2.75 //3


//============================================================================================================================
#define kP_vel                   K_const/SP_angvel * PI/180


int tempvar = 1;
#define STEP_INT_MAX             330

//Motor 1
#define dir1                    (43)      //Direction
#define stp1                    (37)      //Step
//Motor 2
#define dir2                    (49)
#define stp2                    (22)
 // Motor 3
#define dir3                    (35)
#define stp3                    (27)
 // Motor 4
#define dir4                    (41)
#define stp4                    (42)


// LIDAR Definitions
#define ZERO_ENABLE               (53) // port 54, may be left unused
#define ONE_ENABLE                (28) // port 51
#define TWO_ENABLE                (32) // port 52
#define THREE_ENABLE              (30) // port 53
#define LIDAR_ON                  (0)  // lidar power enabled
#define LIDAR_OFF                 (1)  // lidar power disabled
#define DEFAULT_LIDAR_ADDR        (0x62)
#define LIDAR1_ADDR               (0x64)
#define LIDAR2_ADDR               (0x42)
#define LIDAR3_ADDR               (0x32)
#define LIDAR4_ADDR               (0x52)
#define LIDAR1_OFFSET            25 // 21.5  // MEASURE AND CHANGE LATER
#define LIDAR2_OFFSET            20 // 26.5  // MEASURE AND CHANGE LATER
#define LIDAR3_OFFSET            28 // 20.5  // MEASURE AND CHANGE LATER
#define LIDAR4_OFFSET            30.5 // 20  // MEASURE AND CHANGE LATER
#define LIDAR_MODE                  2 // default range, faster acquisition
#define LIDAR_FILT                  0.87


// Controller Definitions (PID)
#define Kp1                 1// 2.28   //Marginally Stable at 6       Kp 3.6   
#define Ki1                 0.00// .1 // .25                                 0.3
#define Kd1                 0.00// 0 // .05 // .1667                         0.075
#define Kp2                 1
#define Ki2                 0.00
#define Kd2                 0.00
#define Kp3                 1
#define Ki3                 0.00
#define Kd3                 0.0
#define Kp4                 1
#define Ki4                 0.00
#define Kd4                 0.00
#define MIN_SETPOINT        25.0f // 10.0f   ******MEASURE AND CHANGE THESE NUMBERS
#define MAX_SETPOINT        55.0f // 40.0f   ******MEASURE AND CHANGE THESE NUMBERS
#define SP_mean             (MAX_SETPOINT + MIN_SETPOINT)/2
#define MAX_AMPLITUDE       15.0f

// Accelerometer Definitions
#define xInput              (A2)    // x acceleration, 0-1023 returned
#define yInput              (A1)    // y acceleration, 0-1023 returned
#define zInput              (A0)    // z acceleration, 0-1023 returned
#define CONE_WIDTH           22      // cone angle from vertical
#define sampleSize           2


Adafruit_BNO055 bno = Adafruit_BNO055(75);

int cone_state = 0;

int t2=0;
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
  int dir_val;     // dir high ==== clockwise
  int dir_pin;
//  int en_val;
//  int en_pin;
//  int MS1_val;
//  int MS1_pin;
//  int MS2_val;
//  int MS2_pin;
//  int MS3_val;
//  int MS3_pin;
  int step_interval;                 //(500 - step_interval) Microseconds
  int no;
};

//PID InfoStruct
struct pidInfo
{
  int pt;                            // previoustime
  int ct;                            // previous time
  int dt;                            // time delta
  float ce;                          // current error
  float pe;                          // previous error
  float sp;                          // setpoint
  float p;                           // proportional control
  float i;                           // integral control
  float d;                           // derivative control
  float u;                           // control signal
  int   tempU;
  float kp;                          // propotional gain
  float ki;                          // integral gain
  float kd;                          // derivative gain
};


//Angle Data Struct
struct angleData
{
  float curr_angle;
  float curr_time;
  float prev_angle;
  float prev_time;
  float next_angle;
  float velocity;
  float acceleration;
  float gyro_z_roll;
  float velocity_error;
  int rot_count;
  
  
  // 10 angle values for moving average
  float a1;
  float a2;
  float a3;
  float a4;
  float a5;
  float a6;
  float a7;
  float a8;
  float a9;
  float a0;
  int cntr;

};

//Global Variables
lidarInfo lidar1;
lidarInfo lidar2;
lidarInfo lidar3;
lidarInfo lidar4;
motorInfo motor1;
motorInfo motor2;
motorInfo motor3;
motorInfo motor4;
pidInfo   pid1;
pidInfo   pid2;
pidInfo   pid3;
pidInfo   pid4;
angleData angular_data;

char d;
byte temp[2];
//****************
// ********************************************************************************************************
void setup() 
{
  Serial.begin(19200);
  init_motors();
  init_lidars();
  init_accelerometer();
  init_pid_controls(pid1, pid2, pid3, pid4);
  init_angular_data(angular_data);
  delay(1000);
  Serial.println("Func, Time(ms), Ang(deg), AngVel(deg/s), S1(cm), D1(cm), S2(cm), D2(cm), S3(cm), D3(cm), S4(cm), D4(cm), No.Rot");
  delay(1000);
}

void loop()
{
   IMU();

   read_lidars();        
           
   // gets and reads lidars

  // =======CASE 1: Start -> Accelerate -> 6(n) Rotations -> Brake -> Stop  
  //===================================== O N E ==================================// 

//  if (angular_data.rot_count < 10)  
//   {
//    // Acceleration
//    setpoint_from_angle(pid1,pid2,pid3,pid4,angular_data);  // Accelerates TW
//   }
//   if (angular_data.rot_count >= 10)
//   {
//    // Braking
//   braking_setpoint_from_angle(pid1,pid2,pid3,pid4,angular_data); // Brakes TW and Stops
//   }

   //=========================================================================// 


   

  // =======CASE 2: Start -> Accelerate -> Set Velocity -> 
  // -> Brake if velocity exceeds/Accelerate if velocity reduces -> 
  // -> Stop after 10 revolutions 
  //===================================== T W O ==================================// 

 // if ((angular_data.rot_count < 10)&&(angular_data.velocity <= SP_angvel))  // 
 //  {
 //   // Acceleration
 //   setpoint_from_angle(pid1,pid2,pid3,pid4,angular_data);
 //  }
//
 //  if ((angular_data.rot_count < 10)&&(angular_data.velocity > SP_angvel))  // 
 //  {
 //   // Brake Momentarily
 //   intermittent_braking_setpoint_from_angle(pid1,pid2,pid3,pid4,angular_data);
 //  }
 //  
 //  if (angular_data.rot_count >= 10) 
 //  {
 //   // Brake and Stop
 //  braking_setpoint_from_angle(pid1,pid2,pid3,pid4,angular_data);
 //  }

   //=========================================================================// 


   

  // =======CASE 3: Start -> Accelerate to set velocity (linear path) ->
  // -> Maintain set velocity by small amplitude motions ->  Stop after 10 revolutions( stopping function not yet written)
  //===================================== T H R E E ==================================// 

//  // Constant Amplitude Acceleration
//    amplitude_from_angle(pid1,pid2,pid3,pid4,angular_data);

 //=========================================================================//
 
 
 
   // =======CASE 4: Start -> Accelerate to set velocity (linear path) ->
  // -> Maintain set velocity by small amplitude motions ->  Stop after 10 revolutions( stopping function not yet written)
  //===================================== F O U R ==================================// 

//  // Constant Amplitude Acceleration
   continuous_setpoint_from_angle(pid1,pid2,pid3,pid4,angular_data);

 //=========================================================================//
 
   control_mass(pid1,lidar1,motor1);
   control_mass(pid2,lidar2,motor2);
   control_mass(pid3,lidar3,motor3);
   control_mass(pid4,lidar4,motor4);
   
   printfunc();

}
// ********************************************************************************************************
void init_motors()
{
  // setup motor control pins
  pinMode(dir1, OUTPUT);
  pinMode(stp1, OUTPUT);

  pinMode(dir2, OUTPUT);
  pinMode(stp2, OUTPUT);
  
  pinMode(dir3, OUTPUT);
  pinMode(stp3, OUTPUT);
  
  pinMode(dir4, OUTPUT);
  pinMode(stp4, OUTPUT);

  resetBEDPins(); //Set step, direction, microstep and enable pins to default states

  motor1.step_pin = stp1;
  motor1.step_val = LOW;
  motor1.dir_pin  = dir1;
  motor1.dir_val  = HIGH;
  motor1.step_interval = 1;                            //(500 - step_interval) Microseconds
  motor1.no = 1; 
  
  digitalWrite(motor1.step_pin, motor1.step_val);
  digitalWrite(motor1.dir_pin, motor1.dir_val);

  motor2.step_pin = stp2;
  motor2.step_val = LOW;
  motor2.dir_pin  = dir2;
  motor2.dir_val  = HIGH;
  motor2.step_interval = 1;                              //(500 - step_interval) Microseconds
  motor2.no = 2; 
  
  digitalWrite(motor2.step_pin, motor2.step_val);
  digitalWrite(motor2.dir_pin, motor2.dir_val);

  motor3.step_pin = stp3;
  motor3.step_val = LOW;
  motor3.dir_pin  = dir3;
  motor3.dir_val  = HIGH;
  motor3.step_interval = 1;                               //(500 - step_interval) Microseconds
  motor3.no = 3; 
  
  digitalWrite(motor3.step_pin, motor3.step_val);
  digitalWrite(motor3.dir_pin, motor3.dir_val);

  motor4.step_pin = stp4;
  motor4.step_val = LOW;
  motor4.dir_pin  = dir4;
  motor4.dir_val  = HIGH;
  motor4.step_interval = 1;                                //(500 - step_interval) Microseconds
  motor4.no = 4; 
  
  digitalWrite(motor4.step_pin, motor4.step_val);
  digitalWrite(motor4.dir_pin, motor4.dir_val);
}

void resetBEDPins()
{
  digitalWrite(stp1, LOW);
  digitalWrite(dir1, LOW);
  
  digitalWrite(stp2, LOW);
  digitalWrite(dir2, LOW);
  
  digitalWrite(stp3, LOW);
  digitalWrite(dir3, LOW);
  
  digitalWrite(stp4, LOW);
  digitalWrite(dir4, LOW);
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


  
  Serial.println("DISTANCE 1");
  Serial.println(lidar1.d);

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

  Serial.println("DISTANCE 2");
  Serial.println(lidar2.d);

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

  Serial.println("DISTANCE 3");
  Serial.println(lidar3.d);

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

  Serial.println("DISTANCE 4");
  Serial.println(lidar4.d);

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

   //lidar3.d = lidar3.interface.distance(true, LIDAR3_ADDR);
    lidar3.d = (1 - LIDAR_FILT) * (lidar3.interface.distance(true, LIDAR3_ADDR) - lidar3.o) + LIDAR_FILT * lidar3.d;
    time3 = millis();
//    
 //  //lidar4.d = lidar4.interface.distance(true, LIDAR4_ADDR);
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

    //lidar3.d = lidar3.interface.distance(true, LIDAR3_ADDR);
    lidar3.d = (1 - LIDAR_FILT) * (lidar3.interface.distance(false, LIDAR3_ADDR) - lidar3.o) + LIDAR_FILT * lidar3.d;
    time3 = millis();

//    //lidar4.d = lidar4.interface.distance(true, LIDAR4_ADDR);
    lidar4.d = (1 - LIDAR_FILT) * (lidar4.interface.distance(false, LIDAR4_ADDR) - lidar4.o) + LIDAR_FILT * lidar4.d;
    time4 = millis();
  }

}


//*********************************************

//gain scheduler() (Assign gains based on angular velocity) (GS var=angvel)
//**********************************************

void init_pid_controls(pidInfo &p1, pidInfo &p2, pidInfo &p3, pidInfo &p4)
{
  p1.pt = 0; p1.ct = 0; p1.dt = 0;
  p1.ce = 0; p1.pe = 0;
  p1.p = 0; p1.i = 0; p1.d = 0; p1.u = 0;

  p1.sp = MIN_SETPOINT;
  p1.kp = Kp1; p1.ki = Ki1; p1.kd = Kd1;
//----------------------------------------
  p2.pt = 0; p2.ct = 0; p2.dt = 0;
  p2.ce = 0; p2.pe = 0;
  p2.p = 0; p2.i = 0; p2.d = 0; p2.u = 0;

  p2.sp = MIN_SETPOINT;
  p2.kp = Kp2; p2.ki = Ki2; p2.kd = Kd2;
//----------------------------------------
  p3.pt = 0; p3.ct = 0; p3.dt = 0;
  p3.ce = 0; p3.pe = 0;
  p3.p = 0; p3.i = 0; p3.d = 0; p3.u = 0;

  p3.sp = MIN_SETPOINT;
  p3.kp = Kp3; p3.ki = Ki3; p3.kd = Kd3;
//----------------------------------------
  p4.pt = 0; p4.ct = 0; p4.dt = 0;
  p4.ce = 0; p2.pe = 0;
  p4.p = 0; p4.i = 0; p4.d = 0; p4.u = 0;

  p4.sp = MIN_SETPOINT;
  p4.kp = Kp4; p4.ki = Ki4; p4.kd = Kd4;
  
}


 void control_mass(pidInfo &control, lidarInfo &sensor, motorInfo &motor)
 {
  control.ct = millis();                    // get current time
 // Serial.print("ct = "); Serial.print(control.ct); Serial.print('\t');//delay(500);
  control.dt = control.ct - control.pt;     // calculate time delta
 //  Serial.print("no = "); Serial.print(motor.no); Serial.print('\t');//delay(500);
  control.ce = sensor.d - control.sp;       // calculate error from setpoint
 //  Serial.print("ce = "); Serial.print(control.ce); Serial.print('\t');//delay(500);
  control.p = control.kp * control.ce;      // calculate pid control output
 //  Serial.print("P = "); Serial.print(control.p); Serial.print('\t');//delay(500);
 // control.i = control.ki * (control.dt*.5*(control.ce + control.pe)+control.i);
  control.i = control.ki * (control.dt * .5 * (control.ce + control.pe)) + control.i;
//   Serial.print("I = "); Serial.print(control.i); Serial.print('\t');//delay(500);
  control.d = control.kd * ((control.ce - control.pe)/control.dt);
//   Serial.print("D = "); Serial.print(control.d); Serial.print('\t');//delay(500);
  control.u = control.p + control.i + control.d;
 //  Serial.print("U = "); Serial.print(control.u); Serial.print('\t');//delay(500);

  if(control.u >= 0)                     
  {
    motor.dir_val = LOW;
   // delay(60);
    motor.step_interval=1;
    char d='h';
  //  Serial.print(motor.dir_val);
  }
  else
  {
    motor.dir_val = HIGH;
    //delay(60);
    motor.step_interval=1;
    char d='l';
  //  Serial.print(motor.dir_val);
  }
//Serial.print("DIR = "); Serial.print(motor.dir_val); Serial.print('\n');
   motor.step_interval = abs(control.u);          //***!!!#### Motor RPM vs delay (See recorded values)
  if(motor.step_interval > STEP_INT_MAX)              //***!!!#### Change PWMval to step delay time val
  {
    motor.step_interval = STEP_INT_MAX;
  }

  control.tempU=control.u;
  digitalWrite(motor.dir_pin,motor.dir_val); // actuate motor
  
  Wire.beginTransmission(motor.no); // transmit to particular motor
  Wire.write(abs(control.tempU));              // sends x 
  Wire.endTransmission();    // stop transmitting
//delay(2000);
  control.pe = control.ce;                  // save previous error
  control.pt = control.ct;                  // save previous time
 
}

void init_angular_data(angleData &data)
{
  data.curr_angle = 0;
  data.curr_time = millis();

  data.rot_count = 0;
  
  data.prev_time = data.curr_time;
  data.prev_angle = data.curr_angle;

  data.a0 = 0;
  data.a1 = 0;
  data.a2 = 0;
  data.a3 = 0;
  data.a4 = 0;
  data.a5 = 0;
  data.a6 = 0;
  data.a7 = 0;
  data.a8 = 0;
  data.a9 = 0;
  data.cntr = 0;
}

void calculate_angular_data(angleData &data)
{
  data.curr_time = millis();
  // old code 
  // data.velocity = 1000*(data.prev_angle - data.curr_angle)/((data.prev_time - data.curr_time));
  float vel = 1000*(data.prev_angle - data.curr_angle)/((data.prev_time - data.curr_time));
  if (((data.prev_angle-data.curr_angle) > 0)&&(abs(data.curr_angle) > 355))
  {
    data.rot_count = data.rot_count + 1;
  }
  data.prev_time = data.curr_time;
  data.prev_angle = data.curr_angle;

//  Serial.print("w = "); Serial.print(data.velocity); Serial.print('\t');
   if ( abs(vel) > 200 || data.curr_angle==359.94 || data.prev_angle==359.94 )
   {
    vel=data.velocity;
   }
  // calculate average velocity
  switch (data.cntr) 
  {
    case 0:
      data.a0 = vel;
      data.cntr = data.cntr + 1;
      break;
    case 1:
      data.a1 = vel;
      data.cntr = data.cntr + 1;
      break;
    case 2:
      data.a2 = vel;
      data.cntr = data.cntr + 1;
      break;
    case 3:
      data.a3 = vel;
      data.cntr = data.cntr + 1;
      break;
    case 4:
      data.a4 = vel;
      data.cntr = data.cntr + 1;
      break;
    case 5:
      data.a5 = vel;
      data.cntr = data.cntr + 1;
      break;
    case 6:
      data.a6 = vel;
      data.cntr = data.cntr + 1;
      break;
    case 7:
      data.a7 = vel;
      data.cntr = data.cntr + 1;
      break;
    case 8:
      data.a8 = vel;
      data.cntr = data.cntr + 1;
      break;
    case 9:
      data.a9 = vel;
      data.cntr = 0;
      break;
  }
  // moving average
  
  if (velocity_source ==1)
  {
  data.velocity = (data.a0 + data.a1 + data.a2 + data.a3 + data.a4 + data.a5 + data.a6 + data.a7 + data.a8 + data.a9)/10;
  }
  if (velocity_source ==2)
  {
  data.velocity = data.gyro_z_roll;
  }
}

//
void init_accelerometer()
{

 if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
}

void IMU()
{

 imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  sensors_event_t event;
  bno.getEvent(&event);
//  double angDEG;
  // int xRaw = ReadAxis(xInput);
  

  // Print the orientation filter output
  float roll = event.orientation.z;
  
  // IMU Low Pass Filter
 // double ang = (1 - 0.3) * roll + 0.3 * ang;
 
 imu::Vector<3> gyro_data = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
 float gyro_z = gyro_data(3);  // gyro_data(1) =  about X, gyro_data(2) = about Y, gyro_data(3) = about(Z)
  if (roll == -0.06)
   {
    Serial.print("ERROR");
   }
   else
   {
    if (roll < 0)
    {
    roll = roll + 360;
    }
     // else 
     // {
     //  angDEG=ang;
     // }

     angular_data.curr_angle =roll;
     angular_data.gyro_z_roll = gyro_z;

      // angular_data.curr_angle = angDEG;
     calculate_angular_data(angular_data);  
   }
}

void continuous_setpoint_from_angle(pidInfo &pid1, pidInfo &pid2, pidInfo &pid3, pidInfo &pid4, angleData &data)
{

  data.velocity_error = (SP_angvel - abs(data.velocity));
  
	if (abs(data.velocity)<=SP_angvel)
	{
     if (abs(data.velocity) >= 0.034)   // near zero ang vel in radian 
     {
		    pid1.sp = min(SP_mean + kP_vel*data.velocity_error*15*cos(data.curr_angle*PI/180), MAX_SETPOINT);
        pid1.sp = max(pid1.sp,MIN_SETPOINT);
        pid2.sp = min(SP_mean - kP_vel*data.velocity_error*15*cos(data.curr_angle*PI/180), MAX_SETPOINT);
        pid1.sp = max(pid1.sp,MIN_SETPOINT);
        pid3.sp = min(SP_mean + kP_vel*data.velocity_error*15*sin(data.curr_angle*PI/180), MAX_SETPOINT);
        pid1.sp = max(pid1.sp,MIN_SETPOINT);
        pid4.sp = min(SP_mean - kP_vel*data.velocity_error*15*sin(data.curr_angle*PI/180), MAX_SETPOINT);
        pid1.sp = max(pid1.sp,MIN_SETPOINT);
     }
     if (abs(data.velocity) < 0.034)   // near zero ang vel in radian
     {
        pid1.sp = SP_mean + 15*cos(data.curr_angle*PI/180);
        pid2.sp = SP_mean - 15*cos(data.curr_angle*PI/180);
        pid3.sp = SP_mean + 15*sin(data.curr_angle*PI/180); 
        pid4.sp = SP_mean - 15*sin(data.curr_angle*PI/180);
     }
     
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
	}
/*	
	if (abs(data.velocity)>SP_angvel)    // Reverse mass positions when angvel exceeds SP 
	{
		    pid1.sp = SP_mean - kP_vel*data.velocity_error*15*cos(data.curr_angle*PI/180);
        pid2.sp = SP_mean + kP_vel*data.velocity_error*15*cos(data.curr_angle*PI/180);;
        pid3.sp = SP_mean - kP_vel*data.velocity_error*15*sin(data.curr_angle*PI/180);; // assume pid1 is left motor and up
        pid4.sp = SP_mean + kP_vel*data.velocity_error*15*sin(data.curr_angle*PI/180);;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
	}
	*/
		
}

void setpoint_from_angle(pidInfo &pid1, pidInfo &pid2, pidInfo &pid3, pidInfo &pid4, angleData &data)
// ASSUMES THAT YOU WANT TO SPIN CLOCKWISE!!!!
{


    if (abs(data.velocity)<=SP_angvel)   // When current angvel less than desired angvel
    {
      // lower angle check
      if ((270 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (270 - CONE_WIDTH))
      {
//        pid1.kp=1;
//        pid1.ki=0.0001;
//        pid1.kd=0.0001;
        pid1.sp = MAX_SETPOINT;
        pid2.sp = MIN_SETPOINT;
        pid3.sp = MIN_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MAX_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }

		if ((180 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (180 - CONE_WIDTH))
      {
//        pid1.kp=1;
//        pid1.ki=0.0001;
//        pid1.kd=0.0001;
        pid1.sp = MIN_SETPOINT;
        pid2.sp = MAX_SETPOINT;
        pid3.sp = MIN_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MAX_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
      

      // upper angle check
		if ((90 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (90 - CONE_WIDTH))
      {
//        pid1.kp=1;                                  
//        pid1.ki=0.0001;                                      
//        pid1.kd=0.0001;                                   
        pid1.sp = MIN_SETPOINT;
        pid2.sp = MAX_SETPOINT;
        pid3.sp = MAX_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MIN_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
    
		if (((0 + CONE_WIDTH) > data.curr_angle && data.curr_angle > 0)||(361 > data.curr_angle && data.curr_angle > (360 - CONE_WIDTH)))
      {
//        pid1.kp=1;                                  
//        pid1.ki=0.0001;                                      
//        pid1.kd=0.0001;                                   
        pid1.sp = MAX_SETPOINT;
        pid2.sp = MIN_SETPOINT;
        pid3.sp = MAX_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MIN_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
    }
  
    if (abs(data.velocity) > SP_angvel*1.7)
    {
//     pid1.kp = 0;                                        // 2.28   //Marginally Stable at 6       Kp 3.6   
//     pid1.ki = 0.0;                                      // .1 // .25                                 0.3
//     pid1.kd = 0;
      pid1.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
      pid2.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
      pid3.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
      pid4.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
    }
    Serial.print("Accel,  ");
   }

void braking_setpoint_from_angle(pidInfo &pid1, pidInfo &pid2, pidInfo &pid3, pidInfo &pid4, angleData &data)
// ASSUMES THAT YOU WANT TO BRAKE BY SPINNING ANTICLOCKWISE!!!!
{

  // IF OUTSIDE THE CONE, DO NOTHING!!!
//  if (((90 + CONE_WIDTH) < (data.curr_angle) && (data.curr_angle) < (270 - CONE_WIDTH))||((data.curr_angle) > (270 + CONE_WIDTH) || (data.curr_angle) < (90 - CONE_WIDTH)))
//    {
////        pid1.kp=1;
////      pid1.ki=0.0001;
////      pid1.kd=0.0001;
//    }
//
//
//  // IF INSIDE THE CONE, DO SOMETHING!!!
//  else
//    {
      
    if ((abs(data.velocity)<=SP_angvel)&&(abs(data.velocity) > 0.9))   // When current angvel less than desired angvel
    {
      // lower angle check
      if ((270 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (270 - CONE_WIDTH))
      {
//        pid1.kp=1;
//        pid1.ki=0.0001;
//        pid1.kd=0.0001;
        pid1.sp = MIN_SETPOINT;
        pid2.sp = MAX_SETPOINT;
        pid3.sp = MAX_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MIN_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }

if ((180 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (180 - CONE_WIDTH))
      {
//        pid1.kp=1;
//        pid1.ki=0.0001;
//        pid1.kd=0.0001;
        pid1.sp = MAX_SETPOINT;
        pid2.sp = MIN_SETPOINT;
        pid3.sp = MAX_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MIN_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
      

      // upper angle check
if ((90 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (90 - CONE_WIDTH))
      {
//        pid1.kp=1;                                  
//        pid1.ki=0.0001;                                      
//        pid1.kd=0.0001;                                   
        pid1.sp = MAX_SETPOINT;
        pid2.sp = MIN_SETPOINT;
        pid3.sp = MIN_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MAX_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
    
if (((0 + CONE_WIDTH) > data.curr_angle && data.curr_angle > 0)||(361 > data.curr_angle && data.curr_angle > (360 - CONE_WIDTH)))
      {
//        pid1.kp=1;                                  
//        pid1.ki=0.0001;                                      
//        pid1.kd=0.0001;                                   
        pid1.sp = MIN_SETPOINT;
        pid2.sp = MAX_SETPOINT;
        pid3.sp = MIN_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MAX_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
    }
  
    if (abs(data.velocity) > SP_angvel*1.7)
    {
//     pid1.kp = 0;                                        // 2.28   //Marginally Stable at 6       Kp 3.6   
//     pid1.ki = 0.0;                                      // .1 // .25                                 0.3
//     pid1.kd = 0;
      pid1.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
      pid2.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
      pid3.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
      pid4.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
    }
    if (abs(data.velocity) <= 0.9)
    {
      pid1.sp = MIN_SETPOINT;
      pid2.sp = MIN_SETPOINT;
      pid3.sp = MIN_SETPOINT;
      pid4.sp = MIN_SETPOINT;
    }
    Serial.print("CBreak,  ");
   }

void intermittent_braking_setpoint_from_angle(pidInfo &pid1, pidInfo &pid2, pidInfo &pid3, pidInfo &pid4, angleData &data)
// ASSUMES THAT YOU WANT TO BRAKE BY SPINNING ANTICLOCKWISE!!!!
{

  // IF OUTSIDE THE CONE, DO NOTHING!!!
//  if (((90 + CONE_WIDTH) < (data.curr_angle) && (data.curr_angle) < (270 - CONE_WIDTH))||((data.curr_angle) > (270 + CONE_WIDTH) || (data.curr_angle) < (90 - CONE_WIDTH)))
//    {
////        pid1.kp=1;
////      pid1.ki=0.0001;
////      pid1.kd=0.0001;
//    }
//
//
//  // IF INSIDE THE CONE, DO SOMETHING!!!
//  else
//    {
      
    if ((abs(data.velocity)<=SP_angvel)&&(abs(data.velocity) > 0.9))   // When current angvel less than desired angvel
    {
      // lower angle check
      if ((270 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (270 - CONE_WIDTH))
      {
//        pid1.kp=1;
//        pid1.ki=0.0001;
//        pid1.kd=0.0001;
        pid1.sp = MAX_SETPOINT;
        pid2.sp = MIN_SETPOINT;
        pid3.sp = MAX_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MIN_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }

if ((180 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (180 - CONE_WIDTH))
      {
//        pid1.kp=1;
//        pid1.ki=0.0001;
//        pid1.kd=0.0001;
        pid1.sp = MIN_SETPOINT;
        pid2.sp = MAX_SETPOINT;
        pid3.sp = MAX_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MIN_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
      

      // upper angle check
if ((90 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (90 - CONE_WIDTH))
      {
//        pid1.kp=1;                                  
//        pid1.ki=0.0001;                                      
//        pid1.kd=0.0001;                                   
        pid1.sp = MIN_SETPOINT;
        pid2.sp = MAX_SETPOINT;
        pid3.sp = MIN_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MAX_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
    
if (((0 + CONE_WIDTH) > data.curr_angle && data.curr_angle > 0)||(361 > data.curr_angle && data.curr_angle > (360 - CONE_WIDTH)))
      {
//        pid1.kp=1;                                  
//        pid1.ki=0.0001;                                      
//        pid1.kd=0.0001;                                   
        pid1.sp = MAX_SETPOINT;
        pid2.sp = MIN_SETPOINT;
        pid3.sp = MIN_SETPOINT; // assume pid1 is left motor and up
        pid4.sp = MAX_SETPOINT;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
    }
  
    if (abs(data.velocity) > SP_angvel*1.7)
    {
//     pid1.kp = 0;                                        // 2.28   //Marginally Stable at 6       Kp 3.6   
//     pid1.ki = 0.0;                                      // .1 // .25                                 0.3
//     pid1.kd = 0;
      pid1.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
      pid2.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
      pid3.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
      pid4.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
    }
    Serial.print("IBreak,  ");
   }



void amplitude_from_angle(pidInfo &pid1, pidInfo &pid2, pidInfo &pid3, pidInfo &pid4, angleData &data)
{

    if (abs(data.velocity)<=SP_angvel)   // When current angvel less than desired angvel
    {
      // lower angle check
      if ((270 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (270 - CONE_WIDTH))
      {
//        pid1.kp=1;
//        pid1.ki=0.0001;
//        pid1.kd=0.0001;

        pid1.sp = SP_mean + MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));
        pid2.sp = SP_mean - MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));
        pid3.sp = SP_mean - MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel)); // assume pid1 is left motor and up
        pid4.sp = SP_mean + MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }

if ((180 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (180 - CONE_WIDTH))
      {
//        pid1.kp=1;
//        pid1.ki=0.0001;
//        pid1.kd=0.0001;
        pid1.sp = SP_mean - MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));
        pid2.sp = SP_mean + MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));
        pid3.sp = SP_mean - MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel)); // assume pid1 is left motor and up
        pid4.sp = SP_mean + MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
      

      // upper angle check
if ((90 + CONE_WIDTH) > data.curr_angle && data.curr_angle > (90 - CONE_WIDTH))
      {
//        pid1.kp=1;                                  
//        pid1.ki=0.0001;                                      
//        pid1.kd=0.0001;                                   
        pid1.sp = SP_mean - MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));
        pid2.sp = SP_mean + MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));
        pid3.sp = SP_mean + MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel)); // assume pid1 is left motor and up
        pid4.sp = SP_mean - MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
    
if (((0 + CONE_WIDTH) > data.curr_angle && data.curr_angle > 0)||(361 > data.curr_angle && data.curr_angle > (360 - CONE_WIDTH)))
      {
//        pid1.kp=1;                                  
//        pid1.ki=0.0001;                                      
//        pid1.kd=0.0001;                                   
        pid1.sp = SP_mean + MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));;
        pid2.sp = SP_mean - MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));;
        pid3.sp = SP_mean + MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));; // assume pid1 is left motor and up
        pid4.sp = SP_mean - MAX_AMPLITUDE*(1 - (data.velocity/SP_angvel));;
        pid1.i = 0;
        pid1.pe = 0;
        pid2.i = 0;
        pid2.pe = 0;
        pid3.i = 0;
        pid3.pe = 0;
        pid4.i = 0;
        pid4.pe = 0;
      }
    }
  
//    if (abs(data.velocity) > SP_angvel*1.7)
//    {
////     pid1.kp = 0;                                        // 2.28   //Marginally Stable at 6       Kp 3.6   
////     pid1.ki = 0.0;                                      // .1 // .25                                 0.3
////     pid1.kd = 0;
//      pid1.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
//      pid2.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
//      pid3.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
//      pid4.sp = (MAX_SETPOINT + MIN_SETPOINT)/2;
//    }
  Serial.print("AmpVar,  ");
}


 void printfunc()
{

 // Serial.print("Time:");  
  Serial.print(millis());Serial.print(" ,");
  t2=millis();
 // Serial.print("Ang:");
  Serial.print(angular_data.curr_angle);Serial.print(" ,");
//  Serial.print("AngRate:");
  Serial.print(angular_data.velocity);Serial.print(" ,");
//  Serial.print("S1:");
  Serial.print(pid1.sp);Serial.print(" ,");
//  Serial.print("D1:");
  Serial.print(lidar1.d);Serial.print(" ,");
//  Serial.print("S2:");
  Serial.print(pid2.sp);Serial.print(" ,");
//  Serial.print("D2:");
  Serial.print(lidar2.d);Serial.print(" ,");
//  Serial.print("S3:");
  Serial.print(pid3.sp);Serial.print(" ,");
//  Serial.print("D3:");
  Serial.print(lidar3.d);Serial.print(" ,");
//  Serial.print("S4:");
  Serial.print(pid4.sp);Serial.print(" ,");
//  Serial.print("D4:");
  Serial.print(lidar4.d);Serial.print(" ,");
//  Serial.print("N:");
  Serial.print(angular_data.rot_count);Serial.print("\n");
}
