#!/usr/bin/env python

import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped

BAG_TOPIC = '/vesc/low_level/ackermann_cmd_mux/input/teleop'
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
PUB_RATE = 20.0

def follow_bag(bag_path, follow_backwards=False):
	pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=1)
	pub_rate = rospy.Rate(PUB_RATE)
	bag = rosbag.Bag(bag_path)

	bag_msgs = []
	for topic, msg, t in bag.read_messages(topics=[BAG_TOPIC]):
		bag_msgs.append(msg)

	if follow_backwards:
		bag_msgs.reverse()
	
	for msg in bag_msgs:
		if follow_backwards:
			msg.drive.speed = -1.0*msg.drive.speed
		pub.publish(msg)
		pub_rate.sleep()

if __name__ == '__main__':
	bag_path = None
	follow_backwards = False
	rospy.init_node('bag_follower', anonymous=True)
	
	# Populate param(s) with value(s) passed by launch file
	bag_path = rospy.get_param('~bag_path', None)
	follow_backwards = rospy.get_param('~follow_backwards', False)

	follow_bag(bag_path, follow_backwards)
