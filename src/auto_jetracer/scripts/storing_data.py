#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from serial_read import Readline
import time
import serial


class StoreData:
	
	def __init__(self):
		#Setup node and topics subscription
		print("setup ros topics and node")
		#Car number
		rospy.init_node('storing_vel_data, anonymous=True)
		rospy.Subscriber("vel_ref1", Float32, self.callback_vref1, queue_size=10)
		rospy.Subscriber("vel_ref2", Float32, self.callback_vref2, queue_size=10)
		rospy.Subscriber("vel_ref3", Float32, self.callback_vref3, queue_size=10)
		rospy.Subscriber("vel_ref4", Float32, self.callback_vref4, queue_size=10)
		rospy.Subscriber("velocity1", Float32, self.callback1, queue_size=10)
		rospy.Subscriber("velocity2", Float32, self.callback1, queue_size=10)
		rospy.Subscriber("velocity3", Float32, self.callback1, queue_size=10)
		rospy.Subscriber("velocity4", Float32, self.callback1, queue_size=10)

		
		#storing vectors
		self.time = []
		self.velref1 = []
		self.velref2 = []
		self.velref3 = []
		self.velref4 = []
		self.vel1 = []
		self.vel2 = []
		self.vel3 = []
		self.vel4 = []

		#Store values
		self.velrefStore1 = 0
		self.velrefStore2 = 0
		self.velrefStore3 = 0
		self.velrefStore4 = 0
		self.velStore2 = 0
		self.velStore3 = 0
		self.velStore4 = 0

		rospy.spin()
		
		
	#Reference velocity callback functions
	def callback_vref1(self,vref):
		self.velrefStore1 = vref.data
	def callback_vref2(self,vref):
		self.velrefStore2 = vref.data
	def callback_vref3(self,vref):
		self.velrefStore3 = vref.data
	def callback_vref4(self,vref):
		self.velrefStore4 = vref.data


	def callback4(self,vel):
		self.velStore4 = vel.data
	def callback3(self,vel):
		self.velStore3 = vel.data
	def callback2(self,vel):
		self.velStore2 = vel.data
	def callback1(self,vel):
		self.time.pushback()
		self.velref1.pushback()
		self.velref2 = []
		self.velref3 = []
		self.velref4 = []
		self.vel1 = []
		self.vel2 = []
		self.vel3 = []
		self.vel4 = []
		

		

		
		

		

if __name__ == '__main__':
	print("Starting pid-controller for velocity")
	try:
		pidvelocity = PIDVelocity()
	except rospy.ROSInterruptException:
		driving.car.throttle = 0
