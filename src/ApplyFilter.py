#!/usr/bin/env python

import rospy
import numpy as np
from scipy import signal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Filter:
	def __init__(self, filter_path, sub_topic, pub_topic, fast_convolve=False):
		self.filter = np.loadtxt(open(filter_path,'rb'),delimiter=',')
		if len(self.filter.shape) < 2:
			self.filter = self.filter[:,np.newaxis]
		self.sub = rospy.Subscriber(sub_topic, Image, self.apply_filter_cb, queue_size=1)
		self.pub = rospy.Publisher(pub_topic, Image, queue_size=1)
		self.bridge = CvBridge()
		self.fast_convolve = fast_convolve

	def apply_filter_cb(self, msg):
		try:
			in_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)
		
		if len(in_image.shape) < 3:
			in_image = in_image[:,:,np.newaxis]

		in_shape = in_image.shape
		
		out_shape = (in_shape[0]-self.filter.shape[0]+1, in_shape[1]-self.filter.shape[1]+1,in_shape[2])
		out_image = np.ndarray(shape=out_shape, dtype=in_image.dtype)

		for ch in xrange(out_shape[2]):
			if fast_convolve:
				out_image[:,:,ch] = signal.convolve2d(self.filter,in_image[:,:,ch],'valid')	
			else:		
				for row in xrange(out_shape[0]):
						for col in xrange(out_shape[1]):
							out_image[row,col,ch] = np.sum(self.filter*in_image[row:row+self.filter.shape[0],col:col+self.filter.shape[1],ch])
		
		try:
			self.pub.publish(self.bridge.cv2_to_imgmsg(out_image, encoding="passthrough"))
		except CvBridgeError as e:
			print(e)

if __name__ == '__main__':
	filter_path = None
	sub_topic = None
	pub_topic = None
	fast_convolve = False

	rospy.init_node('apply_filter', anonymous=True)
	
	# Populate param(s) with value(s) passed by launch file
	filter_path = rospy.get_param('~filter_path', None)
	sub_topic = rospy.get_param('~sub_topic', None)
	pub_topic = rospy.get_param('~pub_topic', None)
	fast_convolve = rospy.get_param('~fast_convolve', False)

	f = Filter(filter_path, sub_topic, pub_topic, fast_convolve)
	rospy.spin()
	
