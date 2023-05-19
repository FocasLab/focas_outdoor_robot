#include <Wire.h>
#include <I2C_Anything.h>

#define SLAVE_ADDRESS 11


#define encoderPinA_L 11  // Quadrature encoder A pin
#define encoderPinB_L 12  // Quadrature encoder B pin

double position = 0, velocity = 0, output = 0, temp = 0;
unsigned long time_now, time_prev;
volatile long encoderPos = 0, encoder_now = 0, encoder_prev = 0;

void readEncoderA_L() {
  if (digitalRead(encoderPinA_L) != digitalRead(encoderPinB_L)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

void readEncoderB_L() {
  if (digitalRead(encoderPinA_L) == digitalRead(encoderPinB_L)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

void setup()
{
  Wire.begin(SLAVE_ADDRESS);
  Serial.begin(9600);
  Serial.println("---I am Slave1---");
  Wire.onRequest(requestEvent);
  delay(1000);

  pinMode(encoderPinA_L, INPUT_PULLUP);  // quadrature encoder input A
  pinMode(encoderPinB_L, INPUT_PULLUP);  // quadrature encoder input B
  attachInterrupt(digitalPinToInterrupt(encoderPinA_L), readEncoderA_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB_L), readEncoderB_L, CHANGE);
  //TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise

}

void loop() {
  encoder_now = encoderPos;
  time_now = millis();
  if ((time_now - time_prev) > 0) {
    velocity = (6.28 * 1000 * (encoder_now - encoder_prev) / (3450 * (time_now - time_prev))); // This calculates velocity in rad/sec
    position = 360 * encoder_now / 3450; // position in degrees
  }
  else
    velocity = 0;

  time_prev = time_now;
  encoder_prev = encoder_now;
  
  //Serial.println(position);
  Serial.println(velocity);
}

void requestEvent()
{
   I2C_writeAnything (position);
}
