void setup() {
  // put your setup code here, to run once:
  pinMode(4, OUTPUT);
  pinMode(41, OUTPUT);
  

//  digitalWrite(4, LOW);/
  analogWriteResolution(8);

}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(41, HIGH);
  analogWrite(4, 20);
  delay(1000);
    digitalWrite(41, LOW);
    int pwm = -20
    analogWrite(4, abs(pwm));
  delay(1000);

}
