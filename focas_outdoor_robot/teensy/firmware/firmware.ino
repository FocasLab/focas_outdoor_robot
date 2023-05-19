#include "robot_config.h"

void setup() {  
  // Encoder Pins and ISR Definition
  pinMode(LEFT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_REAR_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_REAR_ENC_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_FRONT_ENC_A), MotorISR_LEFT_FRONT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_FRONT_ENC_A), MotorISR_RIGHT_FRONT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_REAR_ENC_A), MotorISR_RIGHT_REAR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_REAR_ENC_A), MotorISR_LEFT_REAR, CHANGE);

  // Motor PWM Signal Definitions
  // Check how this is affecting the PWM values - wtf 
  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(LEFT_FRONT_PWM, 375000);
  analogWriteFrequency(RIGHT_FRONT_PWM, 375000);
  analogWriteFrequency(RIGHT_REAR_PWM, 375000);
  analogWriteFrequency(LEFT_REAR_PWM, 375000);

  pinMode(LEFT_FRONT_PWM,OUTPUT);
  pinMode(LEFT_FRONT_MOTOR_DIR,OUTPUT);

  pinMode(RIGHT_FRONT_PWM,OUTPUT);  
  pinMode(RIGHT_FRONT_MOTOR_DIR,OUTPUT);
  
  pinMode(RIGHT_REAR_PWM,OUTPUT);
  pinMode(RIGHT_REAR_MOTOR_DIR,OUTPUT);

  pinMode(LEFT_REAR_PWM,OUTPUT);
  pinMode(LEFT_REAR_MOTOR_DIR,OUTPUT);
 
  // RF Definitions
  pinMode(CH1, INPUT_PULLUP);
  pinMode(CH2, INPUT_PULLUP);
  pinMode(CH3, INPUT_PULLUP);
  pinMode(CH4, INPUT_PULLUP);
  pinMode(RF_CONTROL, OUTPUT);
  digitalWrite(RF_CONTROL, LOW);
  
  ESTOP_SWITCH_TIMER.begin(ESTOP_SWITCH_FUNC, 500000); // Check for change state every 0.5 seconds (2Hz)
  SPEED_TIMER.begin(SPEED_FUNC, 500000); // Check for change state every 0.1 seconds (10Hz)
  SPEED_TIMER.priority(50);

  // ROS Initialization
  Serial.begin(2000000);
  nh.initNode();
  nh.getHardware()->setBaud(2000000);

  // Subscribers Definition
  nh.subscribe(cmd_vel_sub);

  // Publisher Definition
  nh.advertise(odom_pub);
  nh.advertise(motor_states_pub);
  nh.advertise(motor_velocity_pub);

  tf_broadcaster.init(nh);

  // Initialize for SLAM and Navigation ( Odom , TF )
  initOdom();
  initMotorState();
  initMotorVel();
}

/*******************************************************************************
* MotorKill Timer ISR Function
*******************************************************************************/
void ESTOP_SWITCH_FUNC() {
  ch1Value = readChannel(CH1, -100, 100, 0); // Linear
  ch2Value = readChannel(CH2, -100, 100, 0); // Ang
  ch3Value = readChannel(CH3, -1,1,0);
  ch4Value = readSwitch(CH4, false);
  
  if (ch4Value == 0)
    digitalWrite(RF_CONTROL,HIGH);
  else if (ch4Value==1)
    digitalWrite(RF_CONTROL,LOW); 
}

/*******************************************************************************
* Motor State Update Timer ISR Function
*******************************************************************************/
void SPEED_FUNC() {
  // Timer Interrupt updating motor variables at specific time intervals

  noInterrupts();
  current_tick[LEFT_FRONT] = encoder_data[LEFT_FRONT];
  current_tick[RIGHT_FRONT] = encoder_data[RIGHT_FRONT];
  current_tick[RIGHT_REAR] = encoder_data[RIGHT_REAR];
  current_tick[LEFT_REAR] = encoder_data[LEFT_REAR];
  interrupts();

  for (int index = 0 ; index < WHEEL_NUM ; index++) {
    current_velocity[index] = (current_tick[index] - last_tick[index] ) * 60.0 / (0.5 * ENCODER_COUNTS_PER_REV);
    last_tick[index] = current_tick[index];
    last_tick_time[index] = current_tick_time[index];

    last_diff_tick[index] = current_tick[index] - last_tick[index];
    current_velocity_filtered[index] = 0.854 * current_velocity_filtered[index] + 0.0728 * current_velocity[index] + 0.0728 * last_velocity_filtered[index];
    last_velocity_filtered[index] = current_velocity_filtered[index];
  }
}

/*******************************************************************************
* Encoder ISR Functions Declarations
*******************************************************************************/
void MotorISR_LEFT_FRONT() {
  int b = digitalReadFast(LEFT_FRONT_ENC_B);
  if (digitalReadFast(LEFT_FRONT_ENC_A))
    b ? encoder_data[LEFT_FRONT]-- : encoder_data[LEFT_FRONT]++;
  else
    b ? encoder_data[LEFT_FRONT]++ : encoder_data[LEFT_FRONT]--;
}

void MotorISR_RIGHT_FRONT() {
  int b = digitalReadFast(RIGHT_FRONT_ENC_B);
  if (digitalReadFast(RIGHT_FRONT_ENC_A))
    b ? encoder_data[RIGHT_FRONT]-- : encoder_data[RIGHT_FRONT]++;
  else
    b ? encoder_data[RIGHT_FRONT]++ : encoder_data[RIGHT_FRONT]--;
}

void MotorISR_RIGHT_REAR() {
  int b = digitalReadFast(RIGHT_REAR_ENC_B);
  if (digitalReadFast(RIGHT_REAR_ENC_A))
    b ? encoder_data[RIGHT_REAR]-- : encoder_data[RIGHT_REAR]++;
  else
    b ? encoder_data[RIGHT_REAR]++ : encoder_data[RIGHT_REAR]--;
}

void MotorISR_LEFT_REAR() {
  int b = digitalReadFast(LEFT_REAR_ENC_B);
  if (digitalReadFast(LEFT_REAR_ENC_A))
    b ? encoder_data[LEFT_REAR]-- : encoder_data[LEFT_REAR]++;
  else
    b ? encoder_data[LEFT_REAR]++ : encoder_data[LEFT_REAR]--;
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime() {
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* Callback function for Subscribers 
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  goal_velocity[LINEAR_X] = cmd_vel_msg.linear.x;  // Update value in pos 0
  goal_velocity[ANGULAR] = cmd_vel_msg.angular.z;  // Update value in pos 1

  // Changed constraints
  goal_velocity[LINEAR_X] = constrain(goal_velocity[LINEAR_X], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity[ANGULAR] = constrain(goal_velocity[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}

/*******************************************************************************
* Initialize odometry data
*******************************************************************************/
void initOdom(void) {
  init_encoder = true;

  for (int index = 0; index < 3; index++) {
    odom_pose[index] = 0.0;
    odom_vel[index] = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Initialize motor_states data
*******************************************************************************/
void initMotorState(void) {

  motor_states.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  motor_states.layout.dim_length = 0;
  motor_states.data_length = 4;

  motor_states.layout.dim[0].label = dim0_label;
  motor_states.layout.dim[0].size = 4;
  motor_states.layout.dim[0].stride = 1*4;

  motor_states.layout.data_offset = 0;
  motor_states.data = (int64_t *)malloc(sizeof(int)*8);
  for(int i = 0; i < 4; i++){
    motor_states.data[i] = 0;
  }
}

void initMotorVel(void) {
  motor_velocity.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  motor_velocity.layout.dim_length = 0;
  motor_velocity.data_length = 4;

  motor_velocity.layout.dim[0].label = dim1_label;
  motor_velocity.layout.dim[0].size = 4;
  motor_velocity.layout.dim[0].stride = 1*4;

  motor_velocity.layout.data_offset = 0;
  motor_velocity.data = (float_t *)malloc(sizeof(float)*8);
  for(int i = 0; i < 4; i++){
    motor_velocity.data[i] = 0;
  }
}

/*******************************************************************************
* Update motor_state data
*******************************************************************************/
void updateMotorState(void) {
  for(int i = 0; i < 4; i++){
    motor_states.data[i] = last_tick[i];
  }
}

void updateMotorVel(void) {
  for(int i = 0; i < 4; i++){
    motor_velocity.data[i] = last_velocity_filtered[i];
  }
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void) {
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Calculate Updated TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf) {
  odom_tf.header.frame_id = odom_header_frame_id;
  odom_tf.child_frame_id = odom_child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;

  // odom_tf.transform.rotation.x = 0.0;
  // odom_tf.transform.rotation.y = 0.0; 
  // odom_tf.transform.rotation.z = 0.0; 
  // odom_tf.transform.rotation.w = 1.0; 
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time) {
  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_lf = last_velocity_filtered[LEFT_FRONT] * 2 * PI * WHEEL_RADIUS ;
  wheel_rf = last_velocity_filtered[RIGHT_FRONT] * 2 * PI * WHEEL_RADIUS;
  wheel_rr = last_velocity_filtered[RIGHT_REAR] * 2 * PI * WHEEL_RADIUS;
  wheel_lr = last_velocity_filtered[LEFT_REAR] * 2 * PI * WHEEL_RADIUS;

  // wheel_lf = TICK2RAD * (double)last_diff_tick[LEFT_FRONT];
  // wheel_rf = TICK2RAD * (double)last_diff_tick[RIGHT_FRONT];
  // wheel_rr = TICK2RAD * (double)last_diff_tick[RIGHT_REAR];
  // wheel_lr = TICK2RAD * (double)last_diff_tick[LEFT_REAR];

  if (isnan(wheel_lf))
    wheel_lf = 0.0;

  if (isnan(wheel_rf))
    wheel_rf = 0.0;

  if (isnan(wheel_rr))
    wheel_rf = 0.0;

  if (isnan(wheel_lr))
    wheel_lr = 0.0;

  wheel_r = (wheel_rf + wheel_rr)/2.0;
  wheel_l = (wheel_lf + wheel_lr)/2.0;

  delta_theta = theta - last_theta;  // Find delta

  // compute odometric pose from model
  delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  theta = WHEEL_RADIUS * (wheel_r - wheel_l) / 2.0; 

  odom_pose[0] += delta_s * cos(odom_pose[2]) ;
  odom_pose[1] += delta_s * sin(odom_pose[2]);
  odom_pose[2] += theta;

  // compute odometric instantaneous velocity
  v = delta_s/step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  // last_velocity[LEFT_FRONT] = wheel_lf / step_time;
  // last_velocity[RIGHT_FRONT] = wheel_rf / step_time;
  // last_velocity[RIGHT_REAR] = wheel_rr / step_time;
  // last_velocity[LEFT_REAR] = wheel_lr / step_time;
  last_theta = theta;

  return true;
}

/*******************************************************************************
* Publish Function
*******************************************************************************/
void publishDriveInformation(void) {
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = nh.now();

  // calculate and Update Odometry
  calcOdometry((double)(step_time * 0.001));
  updateOdometry();
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // Update Raw Motor Encoder Ticks
  updateMotorState();
  motor_states_pub.publish(&motor_states);

  // Update Motor Velocity
  updateMotorVel();
  motor_velocity_pub.publish(&motor_velocity);
}

/*******************************************************************************
* RC Control Function
*******************************************************************************/
void RF_control() {
  // Receive data from RC
  int RightMotor = 0;
  int LeftMotor = 0;
  // Get values for each channel
  // ch1Value = readChannel(CH1, -100, 100, 0); // Linear
  // ch2Value = readChannel(CH2, -100, 100, 0); // Ang
  // ch3Value = readChannel(CH3, -1,1,0);

  // Read channel and Move the robot according to the kinematic model
  // Do Something using this values
  if ( ch2Value >= 0)
  {
    if( ch1Value >=0 )
    {
      RightMotor = ch2Value - ch1Value;
      LeftMotor = ch2Value + ch1Value;
    }

    if ( ch1Value <= 0 )
    {
      RightMotor = ch2Value - ch1Value;
      LeftMotor = ch2Value + ch1Value;
    }
  }
  
  if ( ch2Value < 0)
  {
    if( ch1Value >=0 )
    {
      RightMotor = ch2Value + ch1Value;
      LeftMotor = ch2Value - ch1Value;
    }

    if ( ch1Value <= 0 )
    {

    RightMotor = ch2Value + ch1Value;
    LeftMotor = ch2Value - ch1Value;

    }
  }


  if ( LeftMotor > 100){
    LeftMotor = 100;
  }
  if ( LeftMotor < -100){
    LeftMotor = -100;
  }

  if ( RightMotor > 100){
    RightMotor = 100;
  }
  if ( RightMotor < -100){
    RightMotor = -100;
  }

  MOTOR_PWM_CONTROL(LEFT_FRONT, -LeftMotor);
  MOTOR_PWM_CONTROL(LEFT_REAR, -LeftMotor);
  MOTOR_PWM_CONTROL(RIGHT_FRONT, RightMotor);
  MOTOR_PWM_CONTROL(RIGHT_REAR, RightMotor);
  
  // for(int i = 0; i < abs(LeftMotor); i++){
    // int a = LeftMotor;
    // MOTOR_PWM_CONTROL(LEFT_FRONT, -i);
    // MOTOR_PWM_CONTROL(LEFT_REAR, -i);
  // }
  
  // for(int i = 0; i < abs(RightMotor); i++){
  //   int b = RightMotor;
  //   MOTOR_PWM_CONTROL(RIGHT_FRONT, -b);
  //   MOTOR_PWM_CONTROL(RIGHT_REAR, -b);
  // }

  // if (ch3Value == 1){
  // MOTOR_PWM_CONTROL(LEFT_FRONT, -ch1Value);
  // MOTOR_PWM_CONTROL(LEFT_REAR, -ch1Value);
  
  // MOTOR_PWM_CONTROL(RIGHT_FRONT, ch1Value);
  // MOTOR_PWM_CONTROL(RIGHT_REAR, ch1Value);
  // }
  
  // if (ch3Value == 0){
  // MOTOR_PWM_CONTROL(LEFT_FRONT, -ch2Value);
  // MOTOR_PWM_CONTROL(LEFT_REAR, -ch2Value);

  // MOTOR_PWM_CONTROL(RIGHT_FRONT, -ch2Value);
  // MOTOR_PWM_CONTROL(RIGHT_REAR, -ch2Value);
  // }

  // Debug printing
  Debug = true;
  if(Debug = true){

    Serial.print(" left: ");
    Serial.print(LeftMotor);
    Serial.print(" right: ");
    Serial.print(RightMotor);
    Serial.print(" Ch1: ");
    Serial.print(ch1Value);
    Serial.print(" | Ch2: ");
    Serial.print(ch2Value);
    Serial.print(" | Ch3: ");
    Serial.println(ch3Value);
    // delay(50);
  }
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected) {
  static bool wait_flag = false;

  if (isConnected) {
    if (wait_flag == false) {
      delay(10);
      wait_flag = true;
    }
  } else {
    wait_flag = false;
  }
}

void MOTOR_VELOCITY_CONTROL(int MOTOR) {

  double psi_i[WHEEL_NUM] = {goal_velocity_motor[0] + 0.01, goal_velocity_motor[1] + 0.01, goal_velocity_motor[2] + 0.01, goal_velocity_motor[3] + 0.01};
  e[MOTOR] = goal_velocity_motor[MOTOR] - current_velocity_filtered[MOTOR];

  long current_time = micros()/1.0e6;

  // Controller - If error more use tan-inv else use pid at steady state
  if ( (abs(e[MOTOR]) >= 5) ) {

    // global time instead of current_tick_time
    noInterrupts();
    si[MOTOR] = ( psi_0[MOTOR] - psi_i[MOTOR] ) * exp( -y[MOTOR] * current_time) + psi_i[MOTOR];
    x[MOTOR] = (PI / (2 * c[MOTOR])) * (tan((PI * e[MOTOR]) / (2 * si[MOTOR])));
    u[MOTOR] = ((2 * v_bar) / PI) * atan(x[MOTOR]);
    interrupts();

    if ( (c[MOTOR] > 0.02) ) {
      c[MOTOR] = c[MOTOR] - 0.01;
    }
    ePrev[MOTOR] = e[MOTOR];
  }

  else {

    eintegral[MOTOR] = eintegral[MOTOR] + e[MOTOR] * dt[MOTOR];
    ederivative[MOTOR] = (e[MOTOR] - ePrev[MOTOR]) / (dt[MOTOR]);
    u[MOTOR] = Kp[MOTOR] * e[MOTOR] + Ki[MOTOR] * eintegral[MOTOR];
    c[MOTOR] = 1;
  }

  if(Debug=true){
    Serial.print(" | Current Vel ");
    Serial.print(current_velocity[MOTOR]);
    Serial.print(" | Signal ");
    Serial.print(u[MOTOR]);
    Serial.print(" | error ");
    Serial.println(e[MOTOR]);
  }

  // Give signal to motor
  MOTOR_PWM_CONTROL(MOTOR, u[MOTOR]);
}

void loop() {
  uint32_t t = millis();
  updateTime();

  // RC Controller Function - to execute the rc commands
  // if ((t - tTime[1]) >= (1000 / 10)) //JOYSTICK_PUBLISH_FREQUENCY))
  // {
  //   RF_control();
  //   tTime[1] = t;
  // }
  
  RF_control();
  delay(100);

  // Publish relevant robot information
  if ((t - tTime[2]) >= (1000 / 100)) //DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishDriveInformation();
    tTime[2] = t;
  }

  // // Send goal velocities to motors ( Not Needed Now )
  // if ((t-tTime[0]) >= (1000 / 100))  //CONTROL_MOTOR_SPEED_FREQUENCY))
  // {
  //   // updateGoalVelocity();
  //   if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) 
  //   {
  //     // Controller Function Call
  //     for(int i = 0; i < 4; i++){
  //       goal_velocity[i] = 0.0;
  //     }
  //     MOTOR_VELOCITY_CONTROL(LEFT_FRONT);
  //     MOTOR_VELOCITY_CONTROL(RIGHT_FRONT);
  //     MOTOR_VELOCITY_CONTROL(LEFT_REAR);
  //     MOTOR_VELOCITY_CONTROL(RIGHT_REAR);
  //   } 
  //   else {
  //     MOTOR_VELOCITY_CONTROL(LEFT_FRONT);
  //     MOTOR_VELOCITY_CONTROL(RIGHT_FRONT);
  //     MOTOR_VELOCITY_CONTROL(LEFT_REAR);
  //     MOTOR_VELOCITY_CONTROL(RIGHT_REAR);
  //   }
  //   tTime[0] = t;
  // }

  // // Call all the callbacks waiting to be called at that point in time
  // // Other Sensor Functionalities if added 
   
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}
