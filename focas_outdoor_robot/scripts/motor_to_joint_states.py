#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState

def motor_states_callback(data):
    # Create JointState message for joint_states
    joint_states = JointState()
    joint_states.name = ["joint1", "joint2", "joint3","joint4"]
    joint_states.position = data.data

    # Publish joint_states message
    joint_states_pub.publish(joint_states)

def motor_velocity_callback(data):
    # Create JointState message for joint_velocities
    joint_velocities = JointState()
    joint_velocities.name = ["joint1", "joint2", "joint3","joint4"]
    joint_velocities.velocity = data.data
    
    # Publish joint_velocities message
    joint_velocities_pub.publish(joint_velocities)

if __name__ == '__main__':

    rospy.init_node('motor_to_joint_states')

    # Create publishers for joint_states and joint_velocities
    joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    joint_velocities_pub = rospy.Publisher('joint_velocities', JointState, queue_size=10)

    # Subscribe to the motor_states and motor_velocity topics
    rospy.Subscriber('motor_states', Int64MultiArray, motor_states_callback)
    rospy.Subscriber('motor_velocity', Float64MultiArray, motor_velocity_callback)

    # Spin ROS
    rospy.spin()
