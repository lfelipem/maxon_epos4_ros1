#!/usr/bin/env python3

# @file python_example_1dof_ppm.py
# @author Felipe Moraes
# @date Ago 08, 2023
# @brief Example code to control one EPOS4 with ROS1 ros_canopen
# This program sends a discrete sinusoidal wave of target position, to be used in PPM 
# Contact: lfelipem@poli.ufrj.br

import rospy
import math
from std_msgs.msg import Float64 

def epos4_cmd():
    rospy.init_node('maxon_epos4_ros_canopen_python_example', anonymous=True) # initialize the ROS node
    pub_vel1 = rospy.Publisher('/maxon/canopen_motor/base_link1_joint_velocity_controller/command', Float64, queue_size=1) # define a publisher for link1
    pub_vel2 = rospy.Publisher('/maxon/canopen_motor/base_link2_joint_velocity_controller/command', Float64, queue_size=1) # define a publisher for link2
    loop_rate = 1.0 # hz
    period_sec = 60.0 # s
    amplitude = 200 # inc
    step = 0
    rate = rospy.Rate(loop_rate) 
    while not rospy.is_shutdown():
        cmd_pos = amplitude*math.sin((step/loop_rate)*2*math.pi/period_sec) # calculate a new target position
        pub_vel1.publish(cmd_pos) # publish the new target position to the command topic
        pub_vel2.publish(cmd_pos)
        print(cmd_pos)
        step += 1 # increment step
        rate.sleep() # sleep to enforce a 50 Hz loop

if __name__ == '__main__':
    try:
        epos4_cmd() # call the loop function
    except rospy.ROSInterruptException:
        pass
