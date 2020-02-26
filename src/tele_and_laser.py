#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
from sensor_msgs.msg import LaserScan
import tty, termios
import numpy as np


global distance_value 
global distance_place
global object_loc
object_loc = None
distance_value = None
distance_place = None

max_lin_vel = 0.22
max_ang_vel = 2.84

lin_step = 0.01
ang_step = 0.1


start_msg = """
The Direction to 
Control Your Bot
---------------------------
Moving around:
		w
   a    s    d
		x

w/x : increase/decrease linear velocity .
a/d : increase/decrease angular velocity .
s   : make the robot come to a complete stop.
space key: force stop
"""
def directionObstacle(inp):
	if inp[0] >=0 and inp[0] <=240:
		object_loc = 1
		print("Object on the right")
	elif inp[0] > 240  and inp[0] <= 480:
		object_loc = 2
		print("Object in the front")
	elif inp[0] > 480 and inp[0] <=720:
		object_loc = 3
		print("Object in the left")
	else:
		object_loc = 4
		print("No Object in search space")
	return object_loc

def laserCallBack(data):
	# range_angels = np.arange(len(data.ranges))
	ranges = np.array(data.ranges)
	# print("The minimum distance:", min(ranges))
	global distance_value
	global distance_place
	distance_value = min(ranges)
	distance_place = np.where(ranges == distance_value)

def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def vels(target_linear_vel, target_angular_vel):
	return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
	if input > output:
		output = min( input, output + slop )
	elif input < output:
		output = max( input, output - slop )
	else:
		output = input

	return output

def constrain(input, low, high):
	if input < low:
	  input = low
	elif input > high:
	  input = high
	else:
	  input = input

	return input

def checkLinearLimitVelocity(vel):
	vel = constrain(vel, -max_lin_vel,max_lin_vel)
	return vel

def checkAngularLimitVelocity(vel):
	vel = constrain(vel, -max_ang_vel, max_ang_vel)
	return vel

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('teleop_node')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	sub = rospy.Subscriber("scan", LaserScan,laserCallBack)
	# print("the sub is ",sub)
	turtlebot3_model = rospy.get_param("model", "burger")

	status = 0
	target_linear_vel   = 0.0
	target_angular_vel  = 0.0
	control_linear_vel  = 0.0
	control_angular_vel = 0.0

	try:
		print(start_msg)
		while(1):
			key = getKey()
			if key == 'w' :
				target_linear_vel = checkLinearLimitVelocity(target_linear_vel + lin_step)
				status = status + 1
				print(vels(target_linear_vel,target_angular_vel))
				print("the minimum distance is ",distance_value, distance_place)
				directionObstacle(distance_place)
			elif key == 'x' :
				target_linear_vel = checkLinearLimitVelocity(target_linear_vel - lin_step)
				status = status + 1
				print(vels(target_linear_vel,target_angular_vel))
				print("the minimum distance is ",distance_value, distance_place)
				directionObstacle(distance_place)
			elif key == 'd' :
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ang_step)
				status = status + 1
				print(vels(target_linear_vel,target_angular_vel))
				print("the minimum distance is ",distance_value, distance_place)
				directionObstacle(distance_place)
			elif key == 'a' :
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ang_step)
				status = status + 1
				print(vels(target_linear_vel,target_angular_vel))
				print("the minimum distance is ",distance_value, distance_place)
				directionObstacle(distance_place)
			elif key == ' ' or key == 's' :
				target_linear_vel   = 0.0
				control_linear_vel  = 0.0
				target_angular_vel  = 0.0
				control_angular_vel = 0.0
				print(vels(target_linear_vel, target_angular_vel))
				print("the minimum distance is ",distance_value, distance_place)
				directionObstacle(distance_place)
			else:
				if (key == '\x03'):
					break

			twist = Twist()

			control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (lin_step/2.0))
			twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

			control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ang_step/2.0))
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

			pub.publish(twist)

	except:
		print("Node not parsed with the bot, failed to communicate with the environment")
