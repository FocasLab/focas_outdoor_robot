const int encoderPinA_L = 19;
const int encoderPinB_L = 18;
int motor_L = 0;
volatile long currentPosition_L = 0;

void setup() {

  Serial.begin(115200);
  pinMode(encoderPinA_L, INPUT_PULLUP);
  pinMode(encoderPinB_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_L), readEncoderA_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB_L), readEncoderB_L, CHANGE);

  TCCR1B = TCCR1B & 0b11111000 | 1;
}

void readEncoderA_L() {
  if (digitalRead(encoderPinA_L) != digitalRead(encoderPinB_L)) {
    currentPosition_L++;
  } else {
    currentPosition_L--;
  }
}

void readEncoderB_L() {
  if (digitalRead(encoderPinA_L) == digitalRead(encoderPinB_L)) {
    currentPosition_L++;
  } else {
    currentPosition_L--;
  }
}


void loop() {
  Serial.println(currentPosition_L);
}