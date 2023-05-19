#include <Wire.h>
#include <I2C_Anything.h>

#define SLAVE_ADDRESS 0x0A

long position= 0;
double velocity = 0;

void setup()
{
  Wire.begin(SLAVE_ADDRESS);
  Serial.begin(9600);
  Serial.println("---I am Slave3---");
  Wire.onRequest(requestEvent);
  delay(1000);
}

void loop() {
  for(int i=0;i<100;i++){
    position = 3;
    velocity = 3.3;
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
