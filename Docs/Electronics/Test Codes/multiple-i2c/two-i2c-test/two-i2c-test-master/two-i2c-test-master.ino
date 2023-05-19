#include <Wire.h>
#include <I2C_Anything.h>

# define SLAVE1_ADDRESS 11


void setup()
{
  Wire.begin();        
  Serial.begin(9600);
    
  Serial.println(F("---I am the Master---"));
  delay(10);
  
}

volatile boolean haveData = false;
volatile double position,velocity;

void loop()
{
  int n = Wire.requestFrom(SLAVE1_ADDRESS, 10);
  if (n = 10)
    {
    I2C_readAnything (position);
    I2C_readAnything (velocity);
    
    //Serial.print ("Received position= ");
    //Serial.println (position);
    Serial.print ("Received velocity= ");
    Serial.println (velocity);
    haveData = false;
    }  // end if haveData
}
