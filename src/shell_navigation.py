#!/usr/bin/env python

"""
MIT License (modified)
Copyright (c) 2019 The Trustees of the University of Pennsylvania
Authors:
Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>
Rosalind Shinkle <rshinkle@seas.upenn.edu>
Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

# General ROS and Python imports
import roslib, rospy, math, tf, numpy
import scipy.io as sio
import time, message_filters

# ROS message imports
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt32, Float32MultiArray, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 


def publish_twist(Publisher,LinearCmd,AngularCmd):
	"""
	Function that publishes a Twist message
	
	Input:
		1) Publisher: Twist ROS publisher
		2) LinearCmd: Linear command
		3) AngularCmd: Angular command
	"""
    # Publish twist
	twist = Twist()
	twist.linear.x = LinearCmd
	twist.angular.z = AngularCmd
	Publisher.publish(twist)
	return


class PointNavigation():

	# Initialization
	def __init__(self):
		# Initialize node
		rospy.init_node('point_navigation', anonymous=True)

		# Find parameters
		self.pub_twist_topic = rospy.get_param('~pub_twist_topic')
		self.sub_laser_topic = rospy.get_param('~sub_laser_topic')
		self.sub_robot_topic = rospy.get_param('~sub_robot_topic')
		self.world_frame_id = rospy.get_param('~world_frame_id')
		self.laser_frame_id = rospy.get_param('~laser_frame_id')

		# Define publishers
		self.pub_twist = rospy.Publisher(self.pub_twist_topic, Twist, queue_size=1, latch=True)

		# Register the subscribers
		self.sub_laser = message_filters.Subscriber(self.sub_laser_topic, LaserScan, queue_size=1)
		self.sub_robot = message_filters.Subscriber(self.sub_robot_topic, Odometry, queue_size=1)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_laser, self.sub_robot], 1, 100000000000)
		self.ts.registerCallback(self.callback)

		# Start the robot
		self.startup()

		# Spin
		rospy.spin()


	def startup(self):
		"""
		Function that sets the robot up before the main loop
		"""
		# Send zeros initially
		publish_twist(self.pub_twist, 0., 0.)
		time.sleep(0.5)

		return


	def callback(self, lidar_data, robot_data):
		"""
		Callback function that implements the main part of the reactive planner
		
		Input:
			1) lidar_data: Data received from the LIDAR sensor
			2) robot_data: Data received from the robot odometry topic
		"""
		# Get robot position 
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([robot_data.pose.pose.orientation.x, robot_data.pose.pose.orientation.y, robot_data.pose.pose.orientation.z, robot_data.pose.pose.orientation.w])
		x_position_robot = robot_data.pose.pose.position.x
		y_position_robot = robot_data.pose.pose.position.y


		"""
		NAVIGATION CODE HERE 
		
		Insert your planning algorithm code here. 

		Populate the LinearCmd and AngularCmd variables, which will then be passed to the robot. 


		"""

		# Populate these output values: 
		LinearCmd = 1;
		AngularCmd = 0; 

		# Publish twist
		publish_twist(self.pub_twist, LinearCmd, AngularCmd)
		return


if __name__ == '__main__':
	try:
		PointNavigationObject = PointNavigation()
	except rospy.ROSInterruptException: pass 
