#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <std_msgs/Float64.h>
#include "CytronMotorDriver.h"

#define encoderPinA_L 19  // Quadrature encoder A pin
#define encoderPinB_L 18  // Quadrature encoder B pin

// Configure the motor driver.
CytronMD motor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4.

ros::NodeHandle nh;

double position = 0, velocity = 0, output = 0, temp = 0;
unsigned long time_now,time_prev;
volatile long encoderPos = 0, encoder_now = 0, encoder_prev=0;

std_msgs::Float64 joint_state;
std_msgs::Float64 joint_speed;

void set_angle_cb(const rospy_tutorials::Floats& cmd_msg) {
  output = cmd_msg.data[0];
}

ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", set_angle_cb);
ros::Publisher pub1("/joint_states_from_arduino", &joint_state);
ros::Publisher pub2("/joint_speed_from_arduino", &joint_speed);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub1);
  nh.advertise(pub2);
  pinMode(encoderPinA_L, INPUT_PULLUP);  // quadrature encoder input A
  pinMode(encoderPinB_L, INPUT_PULLUP);  // quadrature encoder input B
  attachInterrupt(digitalPinToInterrupt(encoderPinA_L), readEncoderA_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB_L), readEncoderB_L, CHANGE);
  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
}

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

void loop() {
  encoder_now = encoderPos;
  time_now = millis();
  if ((time_now-time_prev) > 0){
    velocity = (6.28*1000*(encoder_now - encoder_prev)/(3450*(time_now - time_prev))); // This calculates velocity in rad/sec
    position = 360*encoder_now/3450; // position in degrees
  }
  else
    velocity = 0;
  
  time_prev = time_now;
  encoder_prev = encoder_now;
  
  joint_state.data = position;
  joint_speed.data = velocity;
  pub1.publish(&joint_state);
  pub2.publish(&joint_speed);
  pwmOut(output);
  // Serial.print("Velocity:- ");
  // Serial.println(velocity);
  // Serial.print("Position:- ");
  // Serial.println(position);
  nh.spinOnce();
  delay(100);
}

void pwmOut(float out) {
  motor.setSpeed(-out);
}