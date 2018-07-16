#!/usr/bin/env python

#created by Chenggu Wang 7/16/18, arduino program given by Benjamin Gillette
#this script is the ROS listener for moveit motion planning code
#waits for activation message and send output signal to arduino

#First, import the needed libraries
from arduino import Arduino, Message
import serial
import time
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("received [" + data.data + "]")
    if data.data == "on":
        laser_control_module.set_output(1)
    elif data.data == "off":
        laser_control_module.set_output(0)
    else:
        print("message not recognized")


def arduino_listener():
    rospy.init_node('arduino_listener', anonymous=True)
    rospy.Subscriber("laser_msg", String, callback)
    rospy.spin()

if __name__ == '__main__':
    #Connect to the laser control module's microcontroller.
    #Change 'COM20' to whatever port is actually being used.  In Windows, this
    #will be 'COM#', where # is some number.  In Linux, this will probably be
    #something like '/dev/ttyACM#', where # is some number.
    laser_control_module = Arduino('/dev/ttyACM0')
    #Wait a few seconds to make sure we're connected before sending messages
    time.sleep(2)

    arduino_listener()
