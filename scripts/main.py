#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class mainSystem(object):
	def __init__(self):
		rospy.init_node('mainSystem')
		rospy.Subscriber("barcodeData", String, self.handleBarcodeData)  # Is this line or the below bad redundancy?
		self.ericaCmdPublisher = rospy.Publisher('cmd_erica', String, queue_size=5)
		self.ttsPublisher = rospy.Publisher('ttsText', String, queue_size=5)
    	rospy.loginfo("Main system started.")

	def handleBarcodeData(self, data):
		print data.data
		self.dataInput = data.data.split(",")
		if (len(self.dataInput) == 4):
			self.ttsPublisher.publish(String(str(self.dataInput[2])))
			self.ericaCmdPublisher.publish(String(str(self.dataInput[3])))

if __name__ == '__main__':

	main = mainSystem()
	rospy.spin()