#include <Wire.h>
#include <I2C_Anything.h>

#define SLAVE_ADDRESS 0x0C

long position= 0;
double velocity = 0;

void setup()
{
  Wire.begin(SLAVE_ADDRESS);
  Serial.begin(9600);
  Serial.println("---I am Slave2---");
  Wire.onRequest(requestEvent);
  delay(1000);
}

void loop() {
  for(int i=0;i<100;i++){
    position = 2;
    velocity = 2.2;
    delay(10);
    }
}

void requestEvent()
{
   Serial.print (velocity);
   Serial.print (",");
   Serial.println (position);
   
   I2C_writeAnything (position);
   I2C_writeAnything (velocity);
}
