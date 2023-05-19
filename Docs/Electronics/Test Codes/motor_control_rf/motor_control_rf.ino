#define CH1 16
#define CH2 17
#define CH3 18
#define CH4 19
#define CH5 3
#define CH6 2

// Integers to represent values from sticks and pots
int ch1Value;
int ch2Value;
int ch3Value;
int ch4Value;
int ch5Value;

// Boolean to represent switch value
bool ch6Value;

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

IntervalTimer myTimer;

void setup() {
  // Set up serial monitor
  Serial.begin(115200);

  // Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT_PULLUP);
  pinMode(0, OUTPUT);
  digitalWrite(0,HIGH);
//  attachInterrupt(digitalPinToInterrupt(CH4), Motor_Kill_Switch, HIGH);  
  myTimer.begin(Motor_Kill_Switch, 500000);
}

void Motor_Kill_Switch(){
  int state = readSwitch(CH4, false);
  if (state == 0)
    digitalWrite(0,HIGH);
  else if (state==1)
    digitalWrite(0,LOW);
  Serial.println("efef");
  }

void loop() {

  // Get values for each channel
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch3Value = readChannel(CH3, -1,1,0);
//  ch4Value = readSwitch(CH4, false);

  // Print to Serial Monitor
  Serial.print("Ch1: ");
  Serial.print(ch1Value);
  Serial.print(" | Ch2: ");
  Serial.print(ch2Value);
  Serial.print(" | Ch3: ");
  Serial.print(ch3Value);
  Serial.print(" | Ch4: ");
  Serial.println(ch4Value);
  delay(50);
}
