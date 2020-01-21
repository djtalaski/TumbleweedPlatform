
#include <Wire.h>

#define dir1                    (7)      //Direction
#define stp1                    (5)      //Step
#define EN1                     (6)      //Enable
#define MS1_1                   (2)      //Finer Motor control
#define MS2_1                   (3)      //Finer Motor control
#define MS3_1                   (4)      //Finer Motor control
int num=270;
int x=0;
int j=1;
int iter=0;

void setup() 
{
  Serial.begin(19200);
  pinMode(dir1, OUTPUT);
  pinMode(stp1, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(MS1_1, OUTPUT);
  pinMode(MS2_1, OUTPUT);
  pinMode(MS3_1, OUTPUT);

  digitalWrite(dir1, HIGH);
  digitalWrite(stp1, LOW);
  digitalWrite(EN1, LOW);
  digitalWrite(MS1_1, HIGH);
  digitalWrite(MS2_1, HIGH);
  digitalWrite(MS3_1, LOW);


  Wire.begin(2); 
  Wire.onReceive(receiveEvent); // Attach a function to trigger when something is received.

}

void receiveEvent(int howmany) 
{
 int y = Wire.read();    // read one character from the I2C
// int x=10;
 //int dir = Wire.read();  // Read Direction
// digitalWrite(dir1, dir);
if (y>=0)
{
  digitalWrite(dir1, LOW);
}
if (y<0)
{
  digitalWrite(dir1, HIGH);
}
Serial.println(y);
x=abs(y);
    if (x>55 || x<3)
    {
      digitalWrite(EN1, HIGH);
    
//    digitalWrite(stp1,HIGH); //Trigger one step forward
//   //  delayMicroseconds(num-x*5);
//    delay(1);
//    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
//  //   delayMicroseconds(num-x*5);
//     delay(1);
    }

 else
 {
      digitalWrite(EN1, LOW);
    
//    digitalWrite(stp1,HIGH); //Trigger one step forward
//  //   delayMicroseconds(num-x*5);
//  delayMicroseconds(50);
//   // delay(1);
//    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
// //    delayMicroseconds(num-x*5);
// delayMicroseconds(50);
  //   delay(1);
    }


}

void loop() 
{
//  Serial.print("\t");

//for(j= 1; j<17500; j++)  //Loop the forward stepping enough times for motion to be visible
 // {
//****************************************
//    if (x>50 || x<3)
//    {
//      digitalWrite(EN1, HIGH);
//    
    digitalWrite(stp1,HIGH); //Trigger one step forward
     delayMicroseconds(num-x*5);
   //  delayMicroseconds(900);
   // delay(1);
    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
     delayMicroseconds(num-x*5);
    //    delayMicroseconds(900);
   //  delay(1);
//    }
//
//
// else
// {
//      digitalWrite(EN1, LOW);
//    
//    digitalWrite(stp1,HIGH); //Trigger one step forward
//     delayMicroseconds(num-x*5);
//    delay(1);
//    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
//     delayMicroseconds(num-x*5);
//     delay(1);
//    }

//*******************************************************    
   // delay(1);
 // }



//  if (x<50 && x>0)
//  {
//    digitalWrite(stp1,HIGH); //Trigger one step forward
//     delayMicroseconds(num-x*5);
//    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
//     delayMicroseconds(num-x*5);
////    digitalWrite(stp1,HIGH); //Trigger one step forward
////     delayMicroseconds(num-x*5);
////    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
////     delayMicroseconds(num-x*5);
////         digitalWrite(stp1,HIGH); //Trigger one step forward
////     delayMicroseconds(num-x*5);
////    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
////     delayMicroseconds(num-x*5);
////         digitalWrite(stp1,HIGH); //Trigger one step forward
////     delayMicroseconds(num-x*5);
////    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
////     delayMicroseconds(num-x*5);
////         digitalWrite(stp1,HIGH); //Trigger one step forward
////     delayMicroseconds(num-x*5);
////    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
////     delayMicroseconds(num-x*5);
//  }
}













