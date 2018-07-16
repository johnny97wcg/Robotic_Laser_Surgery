#This script shows how to send commands to the control module to change the
#laser output.  The script connects to the control module over the
#appropriate serial port and pulses the laser on and off several times (the
#pulses actually spell out "TEST TEST TEST" in Morse code).

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

#Start sending commands to the control module
for x in range(0, 3):
    time.sleep(1.75)
    #T (-)
    laser_control_module.set_output(1)
    time.sleep(0.75)
    laser_control_module.set_output(0)
    time.sleep(0.75)
    #E (*)
    laser_control_module.set_output(1)
    time.sleep(0.25)
    laser_control_module.set_output(0)
    time.sleep(0.75)
    #S (---)
    laser_control_module.set_output(1)
    time.sleep(0.25)
    laser_control_module.set_output(0)
    time.sleep(0.25)
    laser_control_module.set_output(1)
    time.sleep(0.25)
    laser_control_module.set_output(0)
    time.sleep(0.25)
    laser_control_module.set_output(1)
    time.sleep(0.25)
    laser_control_module.set_output(0)
    time.sleep(0.75)
    #T (-)
    laser_control_module.set_output(1)
    time.sleep(0.75)
    laser_control_module.set_output(0)
time.sleep(1)
