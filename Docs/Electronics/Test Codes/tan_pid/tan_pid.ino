const int encoderPinA_L = 6;
const int encoderPinB_L = 7;
#define PWM 2
#define DIR 23

// Use the "volatile" directive for variables
// used in an interrupt
volatile long prevT = 0;
volatile long currentPosition_L = 0;
volatile long prevPosition_L = 0;

float vFilt = 0;
float vPrev = 0;

float aFilt = 0;
float aPrev = 0;

// Set a target
float target = 55;
//Serial.println(target);
float eprev = 0;

float kp = 0.2;
float kd = 0.01;
float ki = 0.2;
float eintegral = 0;
float u = 0;

int v_bar = 128;
float c = 1;

int y = 0.5;

float psi_0 = 100;
float psi_i = 0.01;
float tan_inv = 0;

int pwr = 0;

IntervalTimer SPEED_TIMER;

void setup() {
  Serial.begin(115200);

  pinMode(encoderPinA_L, INPUT_PULLUP);
  pinMode(encoderPinB_L, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA_L), readEncoderA_L, CHANGE);
  SPEED_TIMER.begin(SPEED_FUNC, 100000); // Check for change state every 0.1 seconds (10Hz)
  // CONTROL_TIMER.begin(SPEED_FUNC, 100000); // Check for change state every 0.1 seconds (10Hz)

}

void SPEED_FUNC() {
  float velocity = (currentPosition_L - prevPosition_L)/0.1;
  prevPosition_L = currentPosition_L;
  // Convert count/s to RPM
  float v = (velocity/1750)*60.0;

  // Low-pass filter (25 Hz cutoff)
  vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
  vPrev = v;
}

void ACCL_FUNC() {
  float accl = (vFilt - vPrev)/0.1;
  vPrev = vFilt;
  // Low-pass filter (25 Hz cutoff)
  aFilt = 0.854*aFilt + 0.0728*accl + 0.0728*aPrev;
  aPrev = accl;
}


void loop() {
  float e = vFilt - target;
  float r = 2*e + aFilt;

  long T = micros()/1.0e6;

  if (abs(e) > 0){
    noInterrupts();
    float si = ( psi_0 - psi_i ) * exp( -1 * y * T ) + psi_i;
    // float si = ( psi_0 - psi_i ) * pow(2.718 ,-0.5 * T) + psi_i;
    // float x = ( PI / ( 2 * c ) ) * ( tan( ( PI * e ) / ( 2 * si ) ) );
    // tan_inv = ( ( 2 * v_bar ) / PI ) * atan(x);    
    // float x = - 30*( tan( ( PI * e ) / ( 2 * si ) ) );

    float x = -100*( tan( ( PI * r ) / ( 2 * si ) ) );
    tan_inv = x;   
    interrupts();
  
    Serial.print(" r ");
    Serial.print(r);

    Serial.print(" exp ");
    Serial.print(pow(2.718 ,-0.5 * T) + psi_i);

    Serial.print(" | Time: ");
    Serial.print(T);

    Serial.print(" x: ");
    Serial.print(x);

    Serial.print(" | si: ");
    Serial.print(si);

    Serial.print(" | tan_inv: ");
    Serial.print(tan_inv);

    Serial.print(" | error: ");
    Serial.print(e);

    Serial.print(" | c: ");
    Serial.print(c);

    if (c>0.02){
      c = c - 0.01;
    }

  }

  // if (abs(e) < 5){
  //   eintegral = eintegral + e*0.1;
  //   tan_inv = kp*e + ki*eintegral;
  // } 
  eprev = e;
  
  pwr = tan_inv;
  setMotor(pwr);
  Serial.print(" ");
  Serial.print(target);
  Serial.print(" ");
  Serial.print(vFilt);
  Serial.println(" ");
  // Serial.println(currentPosition_L);
  delay(1);
}

void readEncoderA_L() {
  int b = digitalReadFast(encoderPinB_L);
  if (digitalReadFast(encoderPinA_L))
    b ? currentPosition_L-- : currentPosition_L++;
  else
    b ? currentPosition_L++ : currentPosition_L--;
}

void setMotor(int pwm){
  if (pwm > 255) {
    pwm = 255;
  } else if (pwm < -255) {
    pwm = -255;
  }
  
  if (pwm > 0) {
      digitalWrite(DIR, LOW);
      analogWrite(PWM, pwm);
    } else if (pwm < 0) {
      digitalWrite(DIR, HIGH);
      analogWrite(PWM, abs(pwm));
    } else {
      digitalWrite(DIR, LOW);
      analogWrite(PWM, 0);
    }
}
