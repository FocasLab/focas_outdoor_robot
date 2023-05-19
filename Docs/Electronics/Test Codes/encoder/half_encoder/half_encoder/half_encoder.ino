#include <digitalWriteFast.h>

volatile long rawTacho[2];      // interrupt 0 & 1 tachometers


void IRAM_ATTR MotorISR1()
{
    int b = digitalReadFast(23);
    if (digitalReadFast(22))
        b ? rawTacho[0]-- : rawTacho[0]++;
    else
        b ? rawTacho[0]++ : rawTacho[0]--;
}


void setup()
{
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(22), MotorISR1, CHANGE);  // pin 2
    Serial.begin(115200);
}

void loop()
{
  Serial.println(rawTacho[0]);
}
