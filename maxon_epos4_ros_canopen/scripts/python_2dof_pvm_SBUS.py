#!/usr/bin/env python3

# @file python_example_1dof_ppm.py
# @author Felipe Moraes
# @date Oct 05, 2023
# @brief Example code to control one EPOS4 with ROS1 ros_canopen
# This program sends SBUS command to the EPOS4 motors. 
# Contact: lfelipem@poli.ufrj.br

import rospy
import math
from std_msgs.msg import Float64
from sbus_serial.msg import Sbus

scale_low = 1
scale_high = 2

def sbus_callback(sbus_data):
    # Process the SBUS data to extract motor control values
    # Implement your SBUS parsing logic here

    # Enable/Scale switch (Ch5)
    scale_button = sbus_data.mappedChannels[4]
    # Thrust and Maneuver (Ch3 and Ch4)
    thrust_command = sbus_data.mappedChannels[2]   # Thrust command channel, left "up/down" stick
    diff_command = sbus_data.mappedChannels[3]     # Diff command channel, left "left/right" stick

    if scale_button<-50:
        if abs(diff_command)>20:
            v1 = (diff_command)   # v1 => port (bombordo)
            v2 = -(diff_command)  # v2 => starbord (estibordo)
        else:
            v1 = 0
            v2 = 0
    elif scale_button<50:
        v1 = scale_low*((diff_command/100 * thrust_command) + thrust_command)   # v1 => port (bombordo)
        v2 = scale_low*((-diff_command/100 * thrust_command) + thrust_command)  # v2 => starbord (estibordo)
    else:
        v1 = scale_high*((diff_command/100 * thrust_command) + thrust_command)   # v1 => port (bombordo)
        v2 = scale_high*((-diff_command/100 * thrust_command) + thrust_command)  # v2 => starbord (estibordo)
    
    print("Thrust: " + str (thrust_command) + " | Diff: " + str(diff_command) 
          + " | V1: " + str(v1) + " : " + str(v2))

    # Publish the motor control command
    pub_vel1.publish(v1)  # Publish motor velocity
    pub_vel2.publish(v2)


if __name__ == '__main__':
    rospy.init_node('maxon_epos4_ros_canopen_python_example', anonymous=True) # initialize the ROS node
    
    # Create a publisher for motor control commands
    pub_vel1 = rospy.Publisher('/maxon/canopen_motor/base_link1_joint_velocity_controller/command', Float64, queue_size=1) # define a publisher for link1
    pub_vel2 = rospy.Publisher('/maxon/canopen_motor/base_link2_joint_velocity_controller/command', Float64, queue_size=1) # define a publisher for link2

    # Subscribe to the SBUS data topic
    rospy.Subscriber('sbus', Sbus, sbus_callback)

    rospy.spin()