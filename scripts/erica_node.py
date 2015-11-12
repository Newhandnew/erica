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
        self._SerialPublisher = rospy.Publisher('serial', String, queue_size=10)

        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baudRate", 115200))

        rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baud_rate))
        self.nfcReceiver = SerialDataGateway(port, baud_rate, self._handle_received_line)


    def _handle_received_line(self, line):  # This is Propeller specific
        """
        This will run every time a line is received over the serial port (USB)
        from the Propeller board and will send the data to the correct function.
        """
        self._Counter += 1

        print line
        self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))

        if len(line) > 0:
            line_parts = line.split('\t')
            # We should broadcast the odometry no matter what. Even if the motors are off, or location is useful!
            if line_parts == "trigger":
                self._write_serial('a');
                return

    # handle erica command 
    def _handle_erica_command(self, cmd):
        print cmd
        if (cmd.data == "a"):
            self._write_serial('a');
        elif (cmd.data == "b"):
            self._write_serial('b');
        elif (cmd.data == "c"):
            self._write_serial('c');
        elif (cmd.data == "d"):
            self._write_serial('d');
        elif (cmd.data == "e"):
            self._write_serial('e');



    def _reset_serial_connection(self):
        rospy.loginfo("Serial Data Gateway stopping . . .")
        try:
            self.nfcReceiver.Stop()
        except AttributeError:
            rospy.loginfo("Attempt to start nonexistent Serial device.")
        rospy.loginfo("Serial Data Gateway stopped.")
        rospy.loginfo("5 second pause to let Activity Board settle after serial port reset . . .")
        time.sleep(5)  # Give it time to settle.
        self.startSerialPort()


        self._InfraredPublisher.publish(infrared_scan)

    def _write_serial(self, message):
        self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
        self.nfcReceiver.Write(message)

    def start(self):
        self.startSerialPort()

    def startSerialPort(self):
        rospy.loginfo("Serial Data Gateway starting . . .")
        try:
            self.nfcReceiver.Start()
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
        rospy.loginfo("nfcReceiver stopping . . .")
        try:
            self.nfcReceiver.Stop()
        except AttributeError:
            rospy.loginfo("Attempt to start nonexistent Serial device.")
        rospy.loginfo("nfcReceiver stopped.")

 

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