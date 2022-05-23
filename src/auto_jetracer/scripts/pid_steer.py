#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import time
from geometry_msgs.msg import PointStamped
import math

class PIDSteer:
	
	def __init__(self):
		# Setup node and topics subscription
		print("setup ros topics and node")
		# Car number
		number = 3
		rospy.init_node('pidsteer' + str(number), anonymous=True)
		rospy.Subscriber('cluster_point'+ str(number), PointStamped, self.callback_tag, queue_size=2)
		rospy.Subscriber("KpSteer" + str(number), Float32, self.callback_kp, queue_size=1)
		rospy.Subscriber("KiSteer" + str(number), Float32, self.callback_ki, queue_size=1)
		rospy.Subscriber("KdSteer" + str(number), Float32, self.callback_kd, queue_size=1)
		self.pub_steer = rospy.Publisher("steering" + str(number), Float32, queue_size=1)
		rospy.Subscriber("steering2", Float32, self.callback_steer, queue_size=1)
		
		#Tune parameters PID
		self.Kp = 1
		self.Ki = 0.00
		self.Kd = 0.005
		
		#Setup store variables PID
		self.error_prev = 0
		self.tprev = time.time()
		self.I = 0
		
		#Setup steering error
		self.DirectionError = 0
		self.SteerError = 0
		self.steer = 0

		self.steer_infront = 0
		
		rate = rospy.Rate(10)
		#Run the while loop for the controller
		print("starting the controller")
		while not rospy.is_shutdown():
			#Run the PID controller
			self.pid_control()
			# Sleep for the time set in the rate
			rate.sleep()

	#Tag detections callback
	def callback_tag(self, tag):
		x = tag.point.x  
		y = tag.point.y  
		#Calculate the radian error between direction and apriltag direction
		self.DirectionError = math.atan(y / x)*180/math.pi

	#Tuning subscribers	
	def callback_kp(self,kp):
		self.Kp = kp.data
		
	def callback_ki(self,ki):
		self.Ki = ki.data
	
	def callback_kd(self,kd):
		self.Kd = kd.data
		
	def callback_steer(self,steer):
		self.steer_infront = steer.data

	def pid_control(self):
		print("direction",self.DirectionError,"steer",self.steer)
		self.SteerError = self.DirectionError/30 - self.steer
		print("steererror",self.SteerError)
		#Determine each PID control value
		P = self.Kp*self.SteerError
		t = time.time()
		self.I = self.I + self.Ki*self.SteerError*(t - self.tprev)
		D = self.Kd*(self.SteerError-self.error_prev) / (t - self.tprev)
		
		#Store values
		self.tprev = t
		self.error_prev = self.SteerError
		
		#Determine control input
		self.steer = self.steer + P + self.I + D

		if self.steer < 1 and self.steer > -1:
			self.steer = self.steer
		elif self.steer > 1:
			self.steer = 1
		else:
			self.steer = -1

		steer_msg = 0.6*self.steer+0.4*self.steer_infront

		#publish steering message
		self.pub_steer.publish(steer_msg)

		

if __name__ == '__main__':
	print("Starting pid-controller for steering")
	try:
		pidsteering= PIDSteer()
	except rospy.ROSInterruptException:
		driving.car.throttle = 0
