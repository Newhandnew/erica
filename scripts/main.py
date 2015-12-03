#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from zbar_ros.msg import Marker


class mainSystem(object):
	def __init__(self):
		rospy.init_node('mainSystem')
		rospy.Subscriber("markers", Marker, self.handleBarcodeData)  # Is this line or the below bad redundancy?
		self.ericaCmdPublisher = rospy.Publisher('cmd_erica', String, queue_size=5)
		self.ttsPublisher = rospy.Publisher('ttsText', String, queue_size=5)
		rospy.Subscriber("serialReceive", String, self.handleSerialReceiver)
		rospy.loginfo("Main system started.")
		# self.startTime = 0
		self.action = ''
		self.available = 1

		self.cmd_length_angle = 7
		self.cmd_length_macro = 3
		self.cmd_type1_gesture = 1
		self.cmd_type1_angle = 2
		self.cmd_type1_macro = 3
		self.cmd_type2_none = 0
		self.cmd_type2_left = 1
		self.cmd_type2_right = 2
		self.cmd_type2_both = 3
		self.cmd_macro_salute = 2
		self.cmd_macro_ya = 3
		self.cmd_macro_hello = 4
		self.cmd_macro_love = 5
		self.cmd_macro_iceCream = 9
		self.cmd_macro_blowjob = 10

	def handleBarcodeData(self, data):
		rospy.loginfo("get barcode: " + data.data)
		self.dataInput = data.data.split(",")
		# barcode format must meet contain at least 4 column
		if (len(self.dataInput) >= 4):
			if (self.available):
				self.available = 0
				self.ttsPublisher.publish(String(str(self.dataInput[2])))
				# secret action for number 22158
				if (self.dataInput[0] == "22158"):
					self.ericaCmdPublisher.publish(String("%d.%d.%d.%d" % (self.cmd_length_macro, self.cmd_type1_macro, self.cmd_type2_none, self.cmd_macro_blowjob)))
				else:
					if (self.dataInput[3] == "a"):
						self.ericaCmdPublisher.publish(String("%d.%d.%d.%d" % (self.cmd_length_macro, self.cmd_type1_macro, self.cmd_type2_none, self.cmd_macro_salute)))
					elif (self.dataInput[3] == "b"):
						self.ericaCmdPublisher.publish(String("%d.%d.%d.%d" % (self.cmd_length_macro, self.cmd_type1_macro, self.cmd_type2_none, self.cmd_macro_ya)))
					elif (self.dataInput[3] == "c"):
						self.ericaCmdPublisher.publish(String("%d.%d.%d.%d" % (self.cmd_length_macro, self.cmd_type1_macro, self.cmd_type2_none, self.cmd_macro_hello)))
					elif (self.dataInput[3] == "d"):
						self.ericaCmdPublisher.publish(String("%d.%d.%d.%d" % (self.cmd_length_macro, self.cmd_type1_macro, self.cmd_type2_none, self.cmd_macro_love)))
					self.action = self.dataInput[4]

	# get action done acho
	def handleSerialReceiver(self, data):
		rospy.loginfo("get serial data: " + data.data)
		# check "done" ack
		if(len(data.data) > 4):
			if(self.action):
				self.ericaCmdPublisher.publish(String("%d.%d.%d.%d" % (self.cmd_length_macro, self.cmd_type1_macro, self.cmd_type2_none, self.cmd_macro_iceCream)))
				self.action = ''
			else:
				self.available = 1


if __name__ == '__main__':

	main = mainSystem()
	rospy.spin()