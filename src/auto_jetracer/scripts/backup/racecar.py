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
		self.car.steering_gain = 0.4		#do not change this value
		self.car.steering_offset = 0.0
		self.car.throttle_gain = 1
		self.car.steering = 0.0
		self.car.throttle = 0.0
		
		#Setup node and topics subscription
		print("setup ros topics and node")
		rospy.init_node('racecar', anonymous=True)
		rospy.Subscriber("vel_ref2", Float32, self.callback_vref, queue_size=1)
		rospy.Subscriber("steering", Float32, self.callback_steering, queue_size=1)
		rospy.Subscriber("Kp", Float32, self.callback_kp, queue_size=1)
		rospy.Subscriber("Ki", Float32, self.callback_ki, queue_size=1)
		rospy.Subscriber("Kd", Float32, self.callback_kd, queue_size=1)
		rospy.Subscriber("stop", Bool, self.callback_start, queue_size=1)
		pub = rospy.Publisher("velocity2", Float32, queue_size=1)
		
		#Start serial connection
		try:
			print("Setting up USB connection")
			arduino = serial.Serial(port="/dev/ttyUSB1", baudrate=115200, timeout=1)
			print("Setting up USB connection done")
			Rline = Readline(arduino)
		except:
			print('Please check the USB port connections')
		
		#Tune parameters PID
		self.Kp = 0.02
		self.Ki = 0.00
		self.Kd = 0.001
		
		#Setup store variables PID
		self.error_prev = 0
		self.tprev = time.time()
		self.I = 0
		
		#Setup reference velocity for the pid controller and initial velocity
		self.VelReference = 0
		self.Vel = 0
		
		#If the robot may drive
		self.drive = False
		
		rate = rospy.Rate(100)
		#Run the while loop for the controller
		print("starting the controller")
		data_prev = 0
		print(self.Vel)
		while not rospy.is_shutdown():
			
			#Check for new serial data
			data = Rline.readline()
			if data:
				data = float(data)
				if data != data_prev:
					self.Vel = data/1000
					data_prev = data
				pub.publish(self.Vel)
			
			#if self.Vel > 0:
				#print(self.Vel)
			
			#Run the PID controller if we may drive
			if self.drive:
				self.pid_control()
			
			
			
			
			# Sleep for the time set in the rate
			rate.sleep()
			
		#if note is sut down set input zero
		self.car.throttle = 0
		
		
		
	#Steering control callback function
	def callback_steering(self,steer):
		self.car.steering = steer.data
		#rospy.loginfo("Steering: %s", str(steer.data))
		
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
		
	def callback_start(self,start_bool):
		if start_bool.data:
			self.drive = True
		else:
			self.drive = False
			self.car.throttle = 0
		
		
		
		
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
		u = self.car.throttle + P + self.I + D
		
		if self.Vel == 0 and self.VelReference > 0:
			self.car.throttle = 0.15
		elif self.Vel > 0.1 and self.VelReference <= 0:
			self.car.throttle = -1
		elif self.Vel > 0.1:
			self.car.throttle = min(max(-1,u),0.25)
		else:
			self.car.throttle = min(max(-0.5,u),0.5)
			
		#print("pid values: error=", error, " u=", u, " P=",P," I=", self.I, " D=", D)
		



if __name__ == '__main__':
	print("Starting racecar")
	try:
		driving = racecar()
	except rospy.ROSInterruptException:
		driving.car.throttle = 0
