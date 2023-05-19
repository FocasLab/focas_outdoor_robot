#include <PulsePosition.h>
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[] = { 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0, 0, 0, 0 };
int ChannelNumber = 0;
void read_receiver(void) {
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
    for (int i = 1; i <= ChannelNumber; i++) {
      ReceiverValue[i - 1] = ReceiverInput.read(i);
    }
  }
}
void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  ReceiverInput.begin(19);
}
void loop() {
  read_receiver();
  Serial.print("Number of channels: ");
  Serial.print(ChannelNumber);
  Serial.print(" ");
  Serial.print(ReceiverValue[0]);
  Serial.print(", ");
  Serial.print(ReceiverValue[1]);
  Serial.print(", ");
  Serial.print(ReceiverValue[2]);
  Serial.print(", ");
  Serial.print(ReceiverValue[3]);
  Serial.print(", ");
  Serial.print(ReceiverValue[4]);
  Serial.print(", ");
  Serial.print(ReceiverValue[5]);
  Serial.print(", ");
  Serial.print(ReceiverValue[6]);
  Serial.print(", ");
  Serial.print(ReceiverValue[7]);
  Serial.print(" ");
  Serial.print(ReceiverValue[8]);
  Serial.print(", ");
  Serial.print(ReceiverValue[9]);
  Serial.print(", ");
  Serial.print(ReceiverValue[10]);
  Serial.print(", ");
  Serial.print(ReceiverValue[11]);
  Serial.print(", ");
  Serial.print(ReceiverValue[12]);
  Serial.print(", ");
  Serial.print(ReceiverValue[13]);
  Serial.print(", ");
  Serial.print(ReceiverValue[14]);
  Serial.print(", ");
  Serial.println(ReceiverValue[15]);
  delay(50);
}