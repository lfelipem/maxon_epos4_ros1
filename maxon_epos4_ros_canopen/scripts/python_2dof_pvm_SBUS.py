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
import pigpio

scale_low = 40
scale_high = 65

#Use pin 12 (BCM 18) for PWM signal
servo = 18

def rudder_init():
    # Initialize GPIO for Rudder PWM Pin.
    pwm = pigpio.pi() # Connect to local Pi.
    
    pwm.set_mode(servo, pigpio.OUTPUT)
    pwm.set_PWM_frequency(servo, 50)
    pwm.set_servo_pulsewidth(servo, 650)
    
    return (pwm)

def servo_rudder(angle, pwm):
    start = 500
    end = 1800
    ratio = (end - start)/200 #Calculate ratio from angle to percent
    angle_as_percent = angle * ratio
    #print(angle)
    #print(angle_as_percent)
    pwm.set_servo_pulsewidth(servo, 1150 + angle_as_percent)


def sbus_callback(sbus_data, args):
    # Process the SBUS data to extract motor control values

    if sbus_data.failsafe:                      #If True RC Controller is off or out of range!
        print("RC off!")
        v1 = 0
        v2 = 0
    else:
        print("RC on!")
        # Rudder Command (Ch1 - Leme)
        rudder_command = sbus_data.mappedChannels[0]
        servo_rudder(rudder_command, args[0])
        # Enable/Scale switch (Ch5)
        scale_button = sbus_data.mappedChannels[4]
        # Thrust and Maneuver (Ch3 and Ch4)
        thrust_command = sbus_data.mappedChannels[2]   # Thrust command channel, left "up/down" stick
        diff_command = sbus_data.mappedChannels[3]     # Diff command channel, left "left/right" stick

        if scale_button<-50:
            if abs(diff_command)>20:
                v1 = 3*(diff_command)   # v1 => port (bombordo)
                v2 = 3*(diff_command)   # v2 => starbord (estibordo)
            else:
                v1 = 0
                v2 = 0
        elif scale_button<50:
            v1 = scale_low*((diff_command/100 * thrust_command) + thrust_command)   # v1 => port (bombordo)
            v2 = -scale_low*((-diff_command/100 * thrust_command) + thrust_command)  # v2 => starbord (estibordo)
        else:
            v1 = scale_high*((diff_command/100 * thrust_command) + thrust_command)   # v1 => port (bombordo)
            v2 = -scale_high*((-diff_command/100 * thrust_command) + thrust_command)  # v2 => starbord (estibordo)
        
        print("Thrust: " + str (thrust_command) + " | Diff: " + str(diff_command) 
            + " | V1: " + str(v1) + "  V2: " + str(v2) + " | Leme: " + str(rudder_command))

    # Publish the motor control command (motor velocity)
    pub_vel1.publish(v1)
    pub_vel2.publish(v2)


if __name__ == '__main__':
    # Init Rudder
    pwm = rudder_init()

    rospy.init_node('maxon_epos4_ros_canopen_python_example', anonymous=True) # initialize the ROS node
    
    # Create a publisher for motor control commands
    pub_vel1 = rospy.Publisher('/maxon/canopen_motor/base_link1_joint_velocity_controller/command', Float64, queue_size=1) # define a publisher for link1
    pub_vel2 = rospy.Publisher('/maxon/canopen_motor/base_link2_joint_velocity_controller/command', Float64, queue_size=1) # define a publisher for link2

    # Subscribe to the SBUS data topic
    rospy.Subscriber('sbus', Sbus, sbus_callback, (pwm, 0))

    rospy.spin()
    
    # Exit procedure
    pwm.set_PWM_dutycycle(servo, 0)
    pwm.set_PWM_frequency(servo, 0)
    # Stop motors if not stoped
    pub_vel1.publish(0)
    pub_vel2.publish(0)

    pwm.stop()
