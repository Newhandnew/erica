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
        # self.r = rospy.Rate(1) # 1hz refresh rate
        # self._Counter = 0  # For Propeller code's _HandleReceivedLine and _write_serial
        # self._motorsOn = False  # Set to 1 if the motors are on, used with USB Relay Control board
        # self._safeToGo = False  # Use arlobot_safety to set this
        # self._SafeToOperate = False  # Use arlobot_safety to set this
        # self._acPower = True # Track AC power status internally
        # self._unPlugging = False # Used for when arlobot_safety tells us to "UnPlug"!
        # self._wasUnplugging = False # Track previous unplugging status for motor control
        # self._SwitchingMotors = False  # Prevent overlapping calls to _switch_motors
        self._serialAvailable = False
        self._serialTimeout = 0
        # self._leftMotorPower = False
        # self._rightMotorPower = False
        # self._laptop_battery_percent = 100


        # Get motor relay numbers for use later in _HandleUSBRelayStatus if USB Relay is in use:
        # self.relayExists = rospy.get_param("~usbRelayInstalled", False)
        # if self.relayExists:
        #     # I think it is better to get these once than on every run of _HandleUSBRelayStatus
        #     self.usbLeftMotorRelayLabel = rospy.get_param("~usbLeftMotorRelayLabel", "")
        #     self.usbRightMotorRelayLabel = rospy.get_param("~usbRightMotorRelayLabel", "")
        #     rospy.loginfo("Waiting for USB Relay find_relay service to start . . .")
        #     rospy.wait_for_service('/arlobot_usbrelay/find_relay')
        #     rospy.loginfo("USB Relay find_relay service started.")
        #     try:
        #         find_relay = rospy.ServiceProxy('/arlobot_usbrelay/find_relay', FindRelay)
        #         self.leftMotorRelay = find_relay(self.usbLeftMotorRelayLabel)
        #         self.rightMotorRelay = find_relay(self.usbRightMotorRelayLabel)
        #         if self.leftMotorRelay.foundRelay and self.leftMotorRelay.foundRelay:
        #             rospy.loginfo("Left = " + str(self.leftMotorRelay.relayNumber) + " & Right = " + str(
        #                 self.rightMotorRelay.relayNumber))
        #         else:
        #             self.relayExists = False
        #     except rospy.ServiceException as e:
        #         rospy.loginfo("Service call failed: %s" % e)
        #     rospy.Subscriber("arlobot_usbrelay/usbRelayStatus", usbRelayStatus,
        #                      self._handle_usb_relay_status)  # Safety Shutdown

        # # Subscriptions
        rospy.Subscriber("cmd_erica", String, self._handle_erica_command)  # Is this line or the below bad redundancy?
        # rospy.Subscriber("arlobot_safety/safetyStatus", arloSafety, self._safety_shutdown)  # Safety Shutdown

        # # Publishers
        self._SerialPublisher = rospy.Publisher('serial', String, queue_size=10)
        # self._pirPublisher = rospy.Publisher('~pirState', Bool, queue_size=1)  # for publishing PIR status
        # self._arlo_status_publisher = rospy.Publisher('arlo_status', arloStatus, queue_size=1)

       
        # You can use the ~/metatron/scripts/find_propeller.sh script to find this, and
        # You can set it by running this before starting this:
        # rosparam set /arlobot/port $(~/metatron/scripts/find_propeller.sh)
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
        #self._serialTimeout = 0
        # rospy.logdebug(str(self._Counter) + " " + line)
        # if self._Counter % 50 == 0:
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
    #         #rospy.loginfo("Serial Timeout = " + str(self._serialTimeout))
    #         if self._serialTimeout > 19:
    #             rospy.loginfo("Watchdog Timeout Reset initiated")
    #             self._reset_serial_connection()
    #         if self._unPlugging or self._wasUnplugging:
    #             self.UnplugRobot()

    #         old_track_width = self.track_width
    #         self.track_width = rospy.get_param("~driveGeometry/trackWidth", "0")
    #         if not old_track_width == self.track_width:
    #             self.robotParamChanged = True

    #         old_distance_per_count = self.distance_per_count
    #         self.distance_per_count = rospy.get_param("~driveGeometry/distancePerCount", "0")
    #         if not old_distance_per_count == self.distance_per_count:
    #             self.robotParamChanged = True

    #         old_ignore_proximity = self.ignore_proximity
    #         self.ignore_proximity = rospy.get_param("~ignoreProximity", False);
    #         if not old_ignore_proximity == self.ignore_proximity:
    #             self.robotParamChanged = True

    #         old_ignore_cliff_sensors = self.ignore_cliff_sensors
    #         self.ignore_cliff_sensors = rospy.get_param("~ignoreCliffSensors", False);
    #         if not old_ignore_cliff_sensors == self.ignore_cliff_sensors:
    #             self.robotParamChanged = True

    #         old_ignore_ir_sensors = self.ignore_ir_sensors
    #         self.ignore_ir_sensors = rospy.get_param("~ignoreIRSensors", False);
    #         if not old_ignore_ir_sensors == self.ignore_ir_sensors:
    #             self.robotParamChanged = True

    #         old_ignore_floor_sensors = self.ignore_floor_sensors
    #         self.ignore_floor_sensors = rospy.get_param("~ignoreFloorSensors", False);
    #         if not old_ignore_floor_sensors == self.ignore_floor_sensors:
    #             self.robotParamChanged = True

    #         if self.robotParamChanged:
    #             if (self.ignore_proximity):
    #                 ignore_proximity = 1
    #             else:
    #                 ignore_proximity = 0
    #             if (self.ignore_cliff_sensors):
    #                 ignore_cliff_sensors = 1
    #             else:
    #                 ignore_cliff_sensors = 0
    #             if (self.ignore_ir_sensors):
    #                 ignore_ir_sensors = 1
    #             else:
    #                 ignore_ir_sensors = 0
    #             if (self.ignore_floor_sensors):
    #                 ignore_floor_sensors = 1
    #             else:
    #                 ignore_floor_sensors = 0
    #             if (self._acPower):
    #                 ac_power = 1
    #             else:
    #                 ac_power = 0
    #             # WARNING! If you change this check the buffer length in the Propeller C code!
    #             message = 'd,%f,%f,%d,%d,%d,%d,%d\r' % (self.track_width, self.distance_per_count, ignore_proximity, ignore_cliff_sensors, ignore_ir_sensors, ignore_floor_sensors, ac_power)
    #             self._write_serial(message)
    #             self.robotParamChanged = False

    #         self.r.sleep()


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