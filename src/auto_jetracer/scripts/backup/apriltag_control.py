#!/usr/bin/env python3

import rospy
from getkey import getkey
from std_msgs.msg import Float32
from apriltag_ros.msg import AprilTagDetectionArray
import math




class aprildrive:
	def __init__(self):
		self.pub_throttle = rospy.Publisher('throttle', Float32, queue_size=8)
		self.pub_steering = rospy.Publisher('steering', Float32, queue_size=8)

		self.steer_gain = -5
		self.P_gain = 0.1
		self.reference = 0.3
		self.u = 0
		
		#Setup topics publishing and nodes
		sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.callback, queue_size=2)
		rospy.init_node('teleop', anonymous=False)

		#Print contol hints
		print("ROS node is ready")

		rospy.spin()
		


	def callback(self,DetectionArray):
		if DetectionArray.detections:
			x = DetectionArray.detections[0].pose.pose.pose.position.x #left-right
			y = DetectionArray.detections[0].pose.pose.pose.position.y #up-down
			z = DetectionArray.detections[0].pose.pose.pose.position.z #futher-closer
			
			# steering is from 40 degrees to -40 degrees
			range_steer = 40 
			
			steer = math.atan(x/z) * (1 / range_steer)
			
			if steer < 1 and steer > -1:
				steer_msg = steer
			elif steer > 1:
				steer_msg = 1
			else:
				steer_msg = -1
				
			self.pub_steering.publish(steer_msg)
			
			
			
			
			error = z-self.reference
			
			
			self.u = self.u + self.P_gain*error
			self.u = min(max(0,self.u),0.15)
			
			#self.pub_throttle.publish(self.u)
		
		




if __name__ == '__main__':
	
	try:
		print('starting')
		drive = aprildrive()

	except rospy.ROSInterruptException:
		drive.pub_throttle.publish(0)
