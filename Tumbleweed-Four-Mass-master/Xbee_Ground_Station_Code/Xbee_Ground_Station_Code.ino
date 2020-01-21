#include <SoftwareSerial.h>
SoftwareSerial XBee(0, 1); // RX, TX
void setup()
{
 Serial.begin(19200);
 
}

void loop()
{
 if (Serial.available()) 
 {
   Serial.write(Serial.read());
  
 }
}
