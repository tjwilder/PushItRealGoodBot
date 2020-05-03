#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  6 11:55:48 2020

@author: Peter Adamczyk, University of Wisconsin - Madison
"""

import rospy
from std_msgs.msg import Int32, Float32

# Here we will create one or more publishers for the processed topics.
# And the associated Messages.
# They are created outside the "listener" function
# because they will actually be called within the Callback function(s).
pub_A0_proc = rospy.Publisher('/sensors_A0_proc', Float32, queue_size=1)


def sensors_listener():
    # Initialize the Node
    rospy.init_node('sensors_processor', anonymous=False)

    # Set up a listener for the topic you want
    # Remember to name a "callback" function (must define below) to handle the
    # data
    rospy.Subscriber('/sensors_A0', Int32, cal_and_pub_A0)

    # spin() to prevent the function from exiting
    rospy.spin()


def cal_and_pub_A0(msg_in):
    analog_level = float(msg_in.data)

    # calibrate the relationship
    # between the signal that comes in and the signal in real units
    # This can be from a Data Sheet, or from measurements you make yourself.

    # Example: MaxBotix LV-EZ02 says:
    # Distance in Inches = Volts / (Vcc/512).
    # For A0, Vcc is 3.3 Volts
    # Therefore (Distance in Meters) = 0.0254 (m/in)*(distance in Inches)
    analog_volts = analog_level * (3.3 / 1023.)
    distance_meters = analog_volts / (3.3 / 512.) * 0.0254

    A0_proc = Float32()
    A0_proc.data = distance_meters
    pub_A0_proc.publish(A0_proc)


if __name__ == '__main__':
    try:
        sensors_listener()
    except rospy.ROSInterruptException:
        pass
