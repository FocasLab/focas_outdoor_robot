volatile int32_t encoder_position = 0;
volatile int8_t direction = 0;

const int front_right_motor_a = 22; 
const int front_right_motor_b = 23;

void IRAM_ATTR encoder_isr() {
  if (digitalRead(front_right_motor_b) == digitalRead(front_right_motor_a)) {
    encoder_position--;
    direction = -1;
  } else {
    encoder_position++;
    direction = 1;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(front_right_motor_a, INPUT_PULLUP); //Front Right A
  pinMode(front_right_motor_b, INPUT_PULLUP); //front Right B
  attachInterrupt(digitalPinToInterrupt(front_right_motor_a), encoder_isr, CHANGE); //ISR ONLY ONE PIN
}

void loop() {
  Serial.println(encoder_position);
  Serial.println(direction);
}
