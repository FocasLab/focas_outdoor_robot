#include <Wire.h>
#include <I2C_Anything.h>

#define SLAVE_ADDRESS_1 0x0B
#define SLAVE_ADDRESS_2 0x0C
#define SLAVE_ADDRESS_3 0x0A
#define SLAVE_ADDRESS_4 0x0D

void setup() {
  Wire.begin();
  Serial.begin(9600);

  Serial.println(F("---I am the Master---"));
  delay(10);
}

volatile float velocity_1 = 0, velocity_2 = 0, velocity_3 = 0, velocity_4 = 0;
volatile long position_1 = 0, position_2 = 0, position_3 = 0, position_4 = 0;

void loop() {
  int n = Wire.requestFrom(SLAVE_ADDRESS_1, 10);
  if (n = 10) {
    I2C_readAnything(position_1);
    I2C_readAnything(velocity_1);
  }  // end if haveData*/

  int n2 = Wire.requestFrom(SLAVE_ADDRESS_2, 10);
  if (n2 = 10) {
    I2C_readAnything(position_2);
    I2C_readAnything(velocity_2);
  }  // end if haveData

  int n3 = Wire.requestFrom(SLAVE_ADDRESS_3, 10);
  if (n3 = 10) {
    I2C_readAnything(position_3);
    I2C_readAnything(velocity_3);
  }

  int n4 = Wire.requestFrom(SLAVE_ADDRESS_4, 10);
  if (n4 = 10) {
    I2C_readAnything(position_4);
    I2C_readAnything(velocity_4);
  }

  Serial.print("Received Value= ");
  Serial.print(position_1);
  Serial.print(",");
  Serial.print(velocity_1);
  Serial.print(",");
  Serial.print(position_2);
  Serial.print(",");
  Serial.print(velocity_2);
  Serial.print(",");
  Serial.print(position_3);
  Serial.print(",");
  Serial.print(velocity_3);
  Serial.print(",");
  Serial.print(position_4);
  Serial.print(",");
  Serial.println(velocity_4);
}