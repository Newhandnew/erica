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
		rospy.loginfo("Main system started.")
		self.startTime = 0

	def handleBarcodeData(self, data):
		rospy.loginfo("get barcode %d and %d : " + data.data, self.startTime, time.time())
		self.dataInput = data.data.split(",")
		# barcode format must meet 4 column
		# set time interval to 3 seconds
		if (len(self.dataInput) == 4):
			if ((time.time() - self.startTime) > 3):
				self.startTime = time.time();
				self.ttsPublisher.publish(String(str(self.dataInput[2])))
				self.ericaCmdPublisher.publish(String(str(self.dataInput[3])))

if __name__ == '__main__':

	main = mainSystem()
	rospy.spin()