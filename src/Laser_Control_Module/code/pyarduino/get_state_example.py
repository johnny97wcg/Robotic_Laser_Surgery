#This script shows how to request the laser state (whether the laser is on
#or off) from the control module.  It pulses the laser output in the same way
#as in set_output_example.py, but in this case, we're also asking the control
#module to send us the state of the laser each time we change the output.

#First, import the needed libraries
from arduino import Arduino, Message
import serial
import time

#Connect to the laser control module's microcontroller
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
    #Wait a few milliseconds to make sure the control module has had time to
    #switch the laser on/off before asking for the state.
    time.sleep(0.01)
    #Ask for the laser state
    message = laser_control_module.get_laser_state()
    #Print the result.
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.75)
    laser_control_module.set_output(0)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.75)
    #E (*)
    laser_control_module.set_output(1)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.25)
    laser_control_module.set_output(0)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.75)
    #S (---)
    laser_control_module.set_output(1)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.25)
    laser_control_module.set_output(0)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.25)
    laser_control_module.set_output(1)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.25)
    laser_control_module.set_output(0)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.25)
    laser_control_module.set_output(1)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.25)
    laser_control_module.set_output(0)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.75)
    #T (-)
    laser_control_module.set_output(1)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
    time.sleep(0.75)
    laser_control_module.set_output(0)
    time.sleep(0.01)
    message = laser_control_module.get_laser_state()
    print("Laser state: {}".format(message.laserOn))
time.sleep(1)
