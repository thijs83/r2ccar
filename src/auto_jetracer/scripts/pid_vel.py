#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from serial_read import Readline
import time
import serial


class PIDVelocity:
	
	def __init__(self):
		#Setup node and topics subscription
		print("setup ros topics and node")
		#Car number
		number = 3
		rospy.init_node('pidvelocity'+str(number), anonymous=True)
		rospy.Subscriber("vel_ref"+str(number), Float32, self.callback_vref, queue_size=1)
		rospy.Subscriber("KpVel"+str(number), Float32, self.callback_kp, queue_size=1)
		rospy.Subscriber("KiVel"+str(number), Float32, self.callback_ki, queue_size=1)
		rospy.Subscriber("KdVel"+str(number), Float32, self.callback_kd, queue_size=1)
		rospy.Subscriber("drive", Bool, self.callback_drive, queue_size=1)

		
		self.pub = rospy.Publisher("throttle"+str(number), Float32, queue_size=1)
		pub_vel = rospy.Publisher("velocity" + str(number), Float32, queue_size=1)
		
		#Start serial connection
		try:
			print("Setting up USB connection")
			arduino = serial.Serial(port="/dev/arduino", baudrate=115200, timeout=1)
			print("Setting up USB connection done")
			Rline = Readline(arduino)
		except:
			print('Please check the USB port connections')
		
		print('waiting for usb connection to be ready')
		r = rospy.Rate(0.2)
		r.sleep()

		#Tune parameters PID
		self.Kp = 0.05
		self.Ki = 0.00
		self.Kd = 0.01
		
		#Setup store variables PID
		self.error_prev = 0
		self.tprev = time.time()
		self.I = 0
		
		#Setup reference velocity for the pid controller and initial velocity
		self.VelReference = 0
		self.Vel = 0
		
		
		self.drive = False		#If the robot may drive
		self.throttle = 0		#Set first throttle output to zero
		data_prev = 0			#Check for new data from usb
		rate = rospy.Rate(20)	#frequency of controller
		
		print("starting the controller")
		# Run the while loop for the controller
		while not rospy.is_shutdown():
			
			#Check for new serial data
			data = Rline.readline()
			if data:
				data = float(data)
				if data != data_prev:
					self.Vel = data/1000
					data_prev = data
				#Publish the new measured velocity
				pub_vel.publish(self.Vel)
			
			#Run the PID controller if we may drive
			self.pid_control()

			# Sleep for the time set in the rate
			rate.sleep()
		
		
	#Reference velocity callback function
	def callback_vref(self,vref):
		self.VelReference = vref.data
		
	#Tuning subscribers	
	def callback_kp(self,kp):
		self.Kp = kp.data
		
	def callback_ki(self,ki):
		self.Ki = ki.data
	
	def callback_kd(self,kd):
		self.Kd = kd.data
		
	def callback_drive(self,start_bool):
		if start_bool.data:
			self.drive = True
		else:
			self.drive = False
		
		
		
		
	def pid_control(self):
		#Determine each PID control value
		error = self.VelReference - self.Vel
		P = self.Kp*error
		t = time.time()
		self.I = self.I + self.Ki*error*(t - self.tprev)
		D = self.Kd*(error-self.error_prev) / (t - self.tprev)
		
		#Store values
		self.tprev = t
		self.error_prev = error
		
		#Determine control input
		u = self.throttle + P + self.I + D
		
		if self.Vel == 0 and self.VelReference > 0:
			self.throttle = 0.15
		elif self.Vel > 0.1 and self.VelReference <= 0:
			self.throttle = -1
		elif self.Vel > 0.1:
			self.throttle = min(max(-1,u),0.3)
		else:
			self.throttle = min(max(-0.5,u),0.5)

		#check if we may drive else put throttle to zero
		if not self.drive:
			self.throttle = 0

		#Publish the throttle
		self.pub.publish(self.throttle)
		

if __name__ == '__main__':
	print("Starting pid-controller for velocity")
	try:
		pidvelocity = PIDVelocity()
	except rospy.ROSInterruptException:
		driving.car.throttle = 0
