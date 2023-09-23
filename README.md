# Focas outdoor robot

### Introduction
Developed hardware-software stack for a four-wheel differential drive mobile robot for algorithm testing of perception and planning stack. Main motive behind developing this robot was to make it modular in terms of mechanical structure as well sensor/electrical suit while maintaining base level safety. ( Underdevelopment ).
<center>
<img src="/assets/img/researchrobot.gif" height="200px" width="200px">
</center>
<br>

### Technical Specifications 

#### Four Wheel Drive Differential Drive 

1) Power Distribution board with RF and manual switching <br>
2) Controller Board<br>
3) Jetson Xavier<br>
4) RF Control<br>
5) E-stop ( On-Board and RF )<br>
6) Max Linear Speed - 2 m/s, Angular Speed - 10 rad/s<br>
7) Payload Capacity - Upto 50 kg<br>
8) Operating Time - 1 Hr ( Customizable )<br>
9) Open Source and ROS Compatible<br>

### Architecture
<center>
<img src="/assets/img/projects/project_1/Robot_Architecture.png" height="800px">
</center>
<div class="caption">
    High level architecture of the robot's electronics and sensors.
</div>

### Hardware Stack
We Developed each component of this stack keeping in mind modularity thus each component is independent and can be used seperately.<br>

#### Power Distribution Board
We Designed a custom power ditribution board that provides a basic power on/off switch along with a estop switch. It has 4 distribution channels for motor/actuators and 4 outputs to power the electronics, on board computer and other sensors. This board was designed keeping in mind modularity of the robot thus we added additional ports which can be used as and when required.

<center>
<img src="/assets/img/projects/project_1/pcb.png" height="300px">
</center>
<div class="caption">
    Custom Power Distribution Board.
</div>

#### Controller Board
We designed a custom controller host board which works with a Teensy 4.1 and provided different ports to interface various peripherals ( CAN ports – 2, I2C ports – 2, Serial Comm. Ports – 4, SPI ports – 2 ) directly with teensy, additionally we have 4 ports ( encoder + pwm ) which are capable to control a given actuator with sensor feedback.This board was designed keeping in mind modularity of the robot thus we added additional ports which can be used as and when required.

<center>
<img src="/assets/img/projects/project_1/controller_pcb.png" height="300px">
</center>
<div class="caption">
    Custom Controller Board.
</div>

### Software Stack

Software Stack is divided into two sections -<br> 
1) <b> Low Level Control </b>( motor, power, rf, lights, etc.):
   This is handled by the power distribution board and teensy 4.1. It also provides sensor feedback to ROS.
<br>

2) <b>High Level Control</b> ( Task Planning, robot movement, etc.):
   Handled by Jetson Xavier or Other SBC, ROS runs on this machine.All highlevel controller/planners are implemented on this for performing high level task.<br>
