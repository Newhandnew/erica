#!/usr/bin/env python
# Using PEP 8: http://wiki.ros.org/PyStyleGuide
# pylint: disable=line-too-long
# Software License Agreement (BSD License)
#
# Author: Chris L8 https://github.com/chrisl8
# URL: https://github.com/chrisl8/ArloBot
#
# Derived from \opt\ros\hydro\lib\create_node\turtlebot_node.py
# This is based on turtlebot_node adapted to run on a Propeller Activity Board based ArloBot
#
# When upgrading to new versions of ROS,
# or when attempting to integrate new TurtleBot functions,
# please look at and compare turtlebot_node.py to the new version
# to see what you may need to add/improve/replace
# to make things work.
#
# Special thanks to arduino.py by Dr. Rainer Hessmer
# https://code.google.com/p/drh-robotics-ros/
# Much of my code below is based on or copied from his work.
#
# NOTE: This script REQUIRES parameters to be loaded from param/encoders.yaml!

import rospy
import time
import subprocess
import os
import struct

from std_msgs.msg import String

from SerialDataGateway import SerialDataGateway


class erica_node(object):
    """
    Helper class for communicating with a Propeller board over serial port
    """

    def __init__(self):
        rospy.init_node('erica_node')

        self._Counter = 0

        self._serialAvailable = False
        self._serialTimeout = 0

        # # Subscriptions
        rospy.Subscriber("cmd_erica", String, self._handle_erica_command)  # Is this line or the below bad redundancy?

        # # Publishers
        self._SerialPublisher = rospy.Publisher('serialReceive', String, queue_size=10)

        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baudRate", 115200))

        rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baud_rate))
        self.ericaSerial = SerialDataGateway(port, baud_rate, self._handle_received_line)


    def _handle_received_line(self, line):  # This is Propeller specific
        """
        This will run every time a line is received over the serial port (USB)
        from the Propeller board and will send the data to the correct function.
        """
        self._Counter += 1

        print line
        # self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))

        if len(line) > 0:
            line_parts = line.split('\t')
            self._SerialPublisher.publish(String(line_parts))


    # handle erica command 
    def _handle_erica_command(self, cmd):
        print cmd
        # inter communicate type : length.type1.type2.angle1(parameter).angle2.angle3.angle4.angle5 
        cmdData = map(int, cmd.data.split(".")) # chnage string to integer
        # barcode format must meet 4 column
        # set time interval to 3 seconds
        output = []
        output.append(0xc9) # start byte
        # length = type1 + type2 + parameter
        if(cmdData[0] == 7):
            output.append(12)       # add 5 byte for high byte and low byte
        else:
            output.append(cmdData[0])   # length
        output.append(cmdData[1])    # type1 (Gesture, Angle of motor, Macro, Config arm speed, Config hand speed)
        output.append(cmdData[2])    # type2 (left, right, both)
        if(cmdData[0] == 3):        # seperate into 2 form, 1 parameter and 5 parameters
            output.append(int(cmdData[3]))
        elif(cmdData[0] == 7):
            output.append(cmdData[3] / 200)
            output.append(cmdData[3] % 200)
            output.append(cmdData[4] / 200)
            output.append(cmdData[4] % 200)
            output.append(cmdData[5] / 200)
            output.append(cmdData[5] % 200)
            output.append(cmdData[6] / 200)
            output.append(cmdData[6] % 200)
            output.append(cmdData[7] / 200)
            output.append(cmdData[7] % 200)

        check_sum = 0
        for num in range(2, output[1] + 2):    # check sum from type1 to length + (type1 tpye2)
            check_sum ^= output[num]

        output.append(check_sum)
        output.append(0xca) # end byte

        print output
        
        string = ''
        for i in output:
            string += struct.pack('!B',i)
        self._write_serial(string) 


    def _reset_serial_connection(self):
        rospy.loginfo("Serial Data Gateway stopping . . .")
        try:
            self.ericaSerial.Stop()
        except AttributeError:
            rospy.loginfo("Attempt to start nonexistent Serial device.")
        rospy.loginfo("Serial Data Gateway stopped.")
        rospy.loginfo("5 second pause to let Activity Board settle after serial port reset . . .")
        time.sleep(5)  # Give it time to settle.
        self.startSerialPort()


        # self._InfraredPublisher.publish(infrared_scan)

    def _write_serial(self, message):
        # self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
        self.ericaSerial.Write(message)

    def start(self):
        self.startSerialPort()

    def startSerialPort(self):
        rospy.loginfo("Serial Data Gateway starting . . .")
        try:
            self.ericaSerial.Start()
        except:
            rospy.loginfo("SERIAL PORT Start Error")
            reset_usb_script = os.path.expanduser("~/metatron/scripts/callRestUSB.sh")
            if os.path.isfile(reset_usb_script):
                rospy.loginfo("RESETING USB PORTS")
                rospy.loginfo(reset_usb_script)
                devnull = open(os.devnull, 'w')
                subprocess.call(["/bin/bash", reset_usb_script], stdout=devnull, stderr=devnull)

                # At this point we have to restart the node.
                # The respawn atribute in the launch file should handle this.
                raise SystemExit(0)
        rospy.loginfo("Serial Data Gateway started.")
        self._serialAvailable = True


    def stop(self):
        """
        Called by ROS on shutdown.
        Shut off motors, record position and reset serial port.
        """
        rospy.loginfo("Stopping")
        self._serialAvailable = False
        rospy.loginfo("ericaSerial stopping . . .")
        try:
            self.ericaSerial.Stop()
        except AttributeError:
            rospy.loginfo("Attempt to start nonexistent Serial device.")
        rospy.loginfo("ericaSerial stopped.")

 

    def watchDog(self):
        while not rospy.is_shutdown():
            if self._serialAvailable:
                self._serialTimeout += 1
            else:
                self._serialTimeout = 0


if __name__ == '__main__':
    erica = erica_node()
    #rospy.on_shutdown(erica.stop)
    try:
        erica.start()
        rospy.loginfo("Erica_node has started.")
        # erica.watchDog()
        rospy.spin()

    except rospy.ROSInterruptException:
        erica.stop()