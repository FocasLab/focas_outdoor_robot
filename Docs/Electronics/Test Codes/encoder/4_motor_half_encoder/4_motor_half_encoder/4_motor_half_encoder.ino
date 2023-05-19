#include <digitalWriteFast.h>

volatile long rawTacho[3];      // interrupt 0 & 1 tachometers

void MotorISR1()
{
    int b = digitalReadFast(7);
    if (digitalReadFast(8))
        b ? rawTacho[0]-- : rawTacho[0]++;
    else
        b ? rawTacho[0]++ : rawTacho[0]--;
}


void MotorISR2()
{
    int b = digitalReadFast(9);
    if (digitalReadFast(8))
        b ? rawTacho[1]-- : rawTacho[1]++;
    else
        b ? rawTacho[1]++ : rawTacho[1]--;
}


void MotorISR3()
{
    int b = digitalReadFast(33);
    if (digitalReadFast(32))
        b ? rawTacho[2]-- : rawTacho[2]++;
    else
        b ? rawTacho[2]++ : rawTacho[2]--;
}



void setup()
{
    pinMode(6, INPUT_PULLUP);
    pinMode(8, INPUT_PULLUP);
    pinMode(32, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(6), MotorISR1, CHANGE);  // pin 2
    attachInterrupt(digitalPinToInterrupt(8), MotorISR2, CHANGE);  // pin 2
    attachInterrupt(digitalPinToInterrupt(32), MotorISR3, CHANGE);  // pin 2
    
    Serial.begin(115200);
}

void loop()
{
  Serial.print(rawTacho[0]);
  Serial.print(" ");
  Serial.print(rawTacho[1]);
  Serial.print(" ");
  Serial.println(rawTacho[2]);
  delay(10);
}
