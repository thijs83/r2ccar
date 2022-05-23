#!/usr/bin/env python3

import rospy
from getkey import getkey
from std_msgs.msg import Float32
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PointStamped
import math




class aprildrive:
	def __init__(self):
		rospy.init_node('teleop_lidar', anonymous=False)
		
		self.pub_steering = rospy.Publisher('steering', Float32, queue_size=8)
		self.pub_distance = rospy.Publisher('distance', Float32, queue_size=8)
		self.pub_throttle = rospy.Publisher('throttle', Float32, queue_size=8)

		self.steer_gain = -5
		self.P_gain = 1
		self.P_gain_neg = 5
		self.reference = 0.3
		self.u = 0
		
		#Setup topics publishing and nodes
		sub = rospy.Subscriber('cluster_point', PointStamped, self.callback, queue_size=2)
		

		#Print contol hints
		print("ROS node is ready")

		rospy.spin()
		


	def callback(self,point):
		x = point.point.x #futher-closer
		y = point.point.y #left-right
		#z = point.point.z 
		
		# steering is from 40 degrees to -40 degrees
		range_steer = 30 * math.pi/180 
		
		gain = 0.6;
		
		steer = gain*math.atan(y/x) * (1 / range_steer)
		
		if steer < 1 and steer > -1:
			steer_msg = steer
		elif steer > 1:
			steer_msg = 1
		else:
			steer_msg = -1
			
		self.pub_steering.publish(steer_msg)
		
		distance = math.sqrt(x*x + y*y)
		
		self.pub_distance.publish(distance)
		
		error = x-self.reference
		
		if error>0:
			self.u = self.u + self.P_gain*error
		else:
			self.u = self.u + self.P_gain_neg*error
		self.u = min(max(0,self.u),0.18)
		
		#self.pub_throttle.publish(self.u)
		
		




if __name__ == '__main__':
	
	try:
		print('starting')
		drive = aprildrive()

	except rospy.ROSInterruptException:
		drive.pub_throttle.publish(0)
