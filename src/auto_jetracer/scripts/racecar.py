#!/usr/bin/env python3

import rospy
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from serial_read import Readline
import time
import serial


class racecar:
	
	def __init__(self):
		#Initialize car variable and tune settings
		print("setup nvidea racecar")
		self.car = NvidiaRacecar()
		self.car.steering_gain = 0.35		#do not change this value
		self.car.steering_offset = 0.0
		self.car.throttle_gain = 1
		self.car.steering = 0.0
		self.car.throttle = 0.0
		self.drive = False					#If the robot may drive
		
		#Setup node and topics subscription
		print("setup ros topics and node")
		# Car number
		number = 3
		rospy.init_node('racecar'+str(number), anonymous=True)
		rospy.Subscriber("steering"+str(number), Float32, self.callback_steering, queue_size=1)
		rospy.Subscriber("throttle"+str(number), Float32, self.callback_throttle, queue_size=1)
		rospy.Subscriber("drive", Bool, self.callback_start, queue_size=1)


		r = rospy.Rate(100)
		self.pub = rospy.Publisher('test_topic'+str(number), Float32, queue_size=1)
		self.test = 1

		#Run the while loop for the controller
		print("starting the controller")
		while not rospy.is_shutdown():
			test = 10 * 10;
			r.sleep()
		#rospy.spin()

	#Steering control callback function
	def callback_steering(self,steer):
		self.car.steering = steer.data
		
	#Reference velocity callback function
	def callback_throttle(self,throttle):
		if self.drive == True:
			self.car.throttle = throttle.data
		else:
			self.car.throttle = 0

		
	def callback_start(self,start_bool):
		if start_bool.data:
			self.drive = True
		else:
			self.drive = False
			self.car.throttle = 0
		

if __name__ == '__main__':
	print("Starting racecar")
	try:
		driving = racecar()
	except rospy.ROSInterruptException:
		driving.car.throttle = 0
