#This script shows how to listen for messages from the control module.  Each
#time the laser output turns on or off, the control module sends out a
#message over the serial line indicating that the laser state has changed.

#First, import the needed libraries
from arduino import Arduino, Message
import serial
import time

#Connect to the laser control module's microcontroller.
#Change 'COM20' to whatever port is actually being used.  In Windows, this
#will be 'COM#', where # is some number.  In Linux, this will probably be
#something like '/dev/ttyACM#', where # is some number.
laser_control_module = Arduino('/dev/ttyACM0')
#Wait a few seconds to make sure we're connected before sending messages
time.sleep(2)

#Turn the laser on
laser_control_module.set_output(1)
time.sleep(2)
print("Ready")

#Start listening for messages.  Since the control module is telling the laser
#to turn on, if you press and release the foot pedal, the laser should turn
#on and off.  Each time the laser turns on or off, you should recieve a
#message showing the new state.
while True:
    message = laser_control_module.read_message()
    if message != -1:
        print("Laser state: {}".format(message.laserOn))
