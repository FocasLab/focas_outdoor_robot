#include "usb_serial.h"
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

#include <digitalWriteFast.h>

#define NAME "focas_outdoor_robot Outdoor Robot"

// Robot Parameters
#define WHEEL_RADIUS 0.33           // meter
#define WHEEL_SEPARATION 0.122       // meter
#define TURNING_RADIUS 0.080         // meter
#define ROBOT_RADIUS 0.105           // meter

#define ENCODER_MIN -2147483648      // raw
#define ENCODER_MAX 2147483648       // raw
#define ENCODER_COUNTS_PER_REV 1700  // counts
#define MAX_RPM 300                  // RPM

#define LINEAR_X 0
#define ANGULAR 1

// #define PI 3.14159265359
#define DEG2RAD(x) (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x) (x * 57.2957795131)  // *180/PI

#define TICK2RAD 0.00394177  // 0.22584693[deg] * 3.14159265359 / 180 = 0.001533981f - 360/ENCODER_COUNTS = x deg/tick

#define MAX_LINEAR_VELOCITY (WHEEL_RADIUS * 2 * 3.14159265359 * MAX_RPM / 60)  // m/s
#define MAX_ANGULAR_VELOCITY (MAX_LINEAR_VELOCITY / TURNING_RADIUS)            // rad/s

#define MIN_LINEAR_VELOCITY -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY -MAX_ANGULAR_VELOCITY

#define WHEEL_NUM 4
#define PWM_RESOLUTION 8                  // Bit 8 = 0-255 , 9 = 0-511, 10 = 1023
#define CONTROL_MOTOR_SPEED_FREQUENCY 30  //hz
#define CONTROL_MOTOR_TIMEOUT 500         //ms

// Motor Index
// #define LEFT_FRONT 0   // 1
// #define RIGHT_FRONT 1  // 2
// #define RIGHT_REAR 2   // 3
// #define LEFT_REAR 3    // 4

#define RIGHT_FRONT 0  // 1
#define LEFT_FRONT 1   // 2
#define LEFT_REAR 2    // 3
#define RIGHT_REAR 3   // 4

// Motor Encoder Pin Definitions
#define RIGHT_FRONT_ENC_A 6
#define RIGHT_FRONT_ENC_B 7
#define RIGHT_FRONT_MOTOR_DIR 23
#define RIGHT_FRONT_PWM 2

#define LEFT_FRONT_ENC_A 8
#define LEFT_FRONT_ENC_B 9
#define LEFT_FRONT_MOTOR_DIR 22
#define LEFT_FRONT_PWM 3

#define LEFT_REAR_ENC_A 32
#define LEFT_REAR_ENC_B 33
#define LEFT_REAR_MOTOR_DIR 40
#define LEFT_REAR_PWM 4

#define RIGHT_REAR_ENC_A 36
#define RIGHT_REAR_ENC_B 37
#define RIGHT_REAR_MOTOR_DIR 41
#define RIGHT_REAR_PWM 5

// RF Pin Definitions
#define RF_CONTROL 0
#define CH1 30
#define CH2 24
#define CH3 18
#define CH4 19

int ch1Value, ch2Value, ch3Value;
volatile bool ch4Value;  // Boolean to represent switch value
IntervalTimer ESTOP_SWITCH_TIMER;
IntervalTimer SPEED_TIMER;

// debug printing on serial monitor
bool Debug = false;

// Function Prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

static uint32_t tTime[10];

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[] = "/base_link";
char odom_child_frame_id[] = "/odom";

char joint_state_header_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Odometry
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

std_msgs::Int64MultiArray motor_states;
ros::Publisher motor_states_pub("motor_states", &motor_states);
char dim0_label = "motor_state";

std_msgs::Float64MultiArray motor_velocity;
ros::Publisher motor_velocity_pub("motor_velocity", &motor_velocity);
char dim1_label = "motor_velocity";

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* Motor/Encoder Variables
*******************************************************************************/
bool init_encoder = true;

float goal_velocity[2] = { 0.0, 0.0};
float goal_velocity_motor[WHEEL_NUM] = {0.0, 0.0};

volatile long encoder_data[4];

volatile int32_t last_diff_tick[WHEEL_NUM] = { 0, 0, 0, 0 };

volatile int64_t current_tick[WHEEL_NUM] = { 0, 0, 0, 0 };
volatile int64_t last_tick[WHEEL_NUM] = { 0, 0, 0, 0 };

volatile float current_velocity[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };
float current_velocity_filtered[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };
volatile float last_velocity[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };
float last_velocity_filtered[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };

float dt[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };

long current_tick_time[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };
long last_tick_time[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };

volatile float last_rad[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };


/*******************************************************************************
* Controller Definitions
*******************************************************************************/
float ePrev[WHEEL_NUM] = { 0, 0, 0, 0 };

float Kp[WHEEL_NUM] = { 1, 1, 1, 1 };
float Ki[WHEEL_NUM] = { 0.2, 0.2, 0.2, 0.2 };
float Kd[WHEEL_NUM] = { 0.01, 0.01, 0.01, 0.01 };

float e[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };
float ederivative[WHEEL_NUM] = { 0.0, 0, 0, 0.0 };
float eintegral[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };

int u[WHEEL_NUM] = { 0, 0, 0, 0 };

int x[WHEEL_NUM] = { 0, 0, 0, 0 };
int y[WHEEL_NUM] = { 2, 2, 2, 2 };

int v_bar = pow(2, PWM_RESOLUTION)/2;  // Max Input

float c[WHEEL_NUM] = { 1.0, 1.0, 1.0, 1.0 };
float si[WHEEL_NUM] = { 0.0, 0.0, 0.0, 0.0 };

double psi_0[WHEEL_NUM] = { 300.0, 300.0, 300.0, 300.0 };

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
// float orientation =0;
double wheel_lf=0.0, wheel_rf=0.0, wheel_rr=0.0, wheel_lr=0.0, wheel_l=0.0,wheel_r=0.0;  // rotation value of wheel [rad]
double delta_s= 0.0, theta= 0.0, delta_theta= 0.0;
static double last_theta = 0.0;
double v = 0.0, w = 0.0;  // v = translational velocity [m/s], w = rotational velocity [rad/s]
double step_time = 0.0;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time = 0;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* RF Reciver Functions
*******************************************************************************/
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

/*******************************************************************************
* Motor Control Function
*******************************************************************************/
void MOTOR_PWM_CONTROL(int motor, int pwm)  //Function that control motors,pwm (range - to +);
{
  // Generalize for n bits
  if (pwm > pow(2, PWM_RESOLUTION)) {
    pwm = pow(2, PWM_RESOLUTION);
  } else if (pwm < -pow(2, PWM_RESOLUTION)) {
    pwm = -pow(2, PWM_RESOLUTION);
  }

  if (motor == LEFT_FRONT) {
    if (pwm > 0) {
      digitalWrite(LEFT_FRONT_MOTOR_DIR, HIGH);
      analogWrite(LEFT_FRONT_PWM, pwm);
    } else if (pwm < 0) {
      digitalWrite(LEFT_FRONT_MOTOR_DIR, LOW);
      analogWrite(LEFT_FRONT_PWM, abs(pwm));
    } else {
      digitalWrite(LEFT_FRONT_MOTOR_DIR, LOW);
      analogWrite(LEFT_FRONT_PWM, 0);
    }
  }

  else if (motor == RIGHT_FRONT) {
    if (pwm > 0) {
      digitalWrite(RIGHT_FRONT_MOTOR_DIR, HIGH);
      analogWrite(RIGHT_FRONT_PWM, pwm);
    } else if (pwm < 0) {
      digitalWrite(RIGHT_FRONT_MOTOR_DIR, LOW);
      analogWrite(RIGHT_FRONT_PWM, abs(pwm));
    } else {
      digitalWrite(RIGHT_FRONT_MOTOR_DIR, LOW);
      analogWrite(RIGHT_FRONT_PWM, 0);
    }
  }

  else if (motor == RIGHT_REAR) {
    if (pwm > 0) {
      digitalWrite(RIGHT_REAR_MOTOR_DIR, HIGH);
      analogWrite(RIGHT_REAR_PWM, pwm);
    } else if (pwm < 0) {
      digitalWrite(RIGHT_REAR_MOTOR_DIR, LOW);
      analogWrite(RIGHT_REAR_PWM, abs(pwm));
    } else {
      digitalWrite(RIGHT_REAR_MOTOR_DIR, LOW);
      analogWrite(RIGHT_REAR_PWM, 0);
    }
  }

  else if (motor == LEFT_REAR) {
    if (pwm > 0) {
      digitalWrite(LEFT_REAR_MOTOR_DIR, HIGH);
      analogWrite(LEFT_REAR_PWM, pwm);
    } else if (pwm < 0) {
      digitalWrite(LEFT_REAR_MOTOR_DIR, LOW);
      analogWrite(LEFT_REAR_PWM, abs(pwm));
    } else {
      digitalWrite(LEFT_REAR_MOTOR_DIR, LOW);
      analogWrite(LEFT_REAR_PWM, 0);
    }
  }
}
