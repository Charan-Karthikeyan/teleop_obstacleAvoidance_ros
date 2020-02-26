#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

global object_loc
global distance_value
distance_value = None
object_loc = None

def laserCallback(data):
	global distance_value
	global object_loc
	ranges = np.array(data.ranges)
	distance_value = min(ranges)
	distance_place = np.where(ranges==distance_value)
	if distance_place[0] >100 and distance_place[0] <=600 and distance_value < 1.5:
		object_loc = True
	else:
		object_loc = False
	# moveBot(object_loc)

def moveBot(object_loc, pub):
	twist = Twist()  
	print("Moving")
	if object_loc == True:
		twist.linear.x = 0.0
		twist.angular.z = 0.5
	elif object_loc == False:
		twist.linear.x = 0.2
		twist.angular.z = 0.0
	else:
		twist.linear.x = 0.0
		twist.angular.z = 0.0
	pub.publish(twist)

if __name__ == "__main__":
	rospy.init_node("obstcale_avoidance")
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	sub = rospy.Subscriber('scan',LaserScan,laserCallback)
	while not rospy.is_shutdown():
		moveBot(object_loc,pub)
	rospy.spin()

