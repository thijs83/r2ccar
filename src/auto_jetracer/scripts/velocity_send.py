#!/usr/bin/env python3

from std_msgs.msg import Float32
import rospy

rospy.init_node('vrefsend')
pub = rospy.Publisher("vel_ref2", Float32, queue_size=10)



while not rospy.is_shutdown():
	
	print("To shutdown type n and press enter")
	name = input("Enter reference velocity and press enter:")
	if name == 'n':
		rospy.signal_shutdown("n is entered to shutdown")
	try:
		velocity = float(name)
		if velocity > 4:
			velocity = 4
			print("velocity was too high, set to 4")
		
		if velocity < -2:
			velocity = -2
			print("velocity was too low, set to -2")
		
		pub.publish(velocity)
	except:
		print("not valid floating number")
		
	
		
		
#If ros shutdown set reference to 0
velocity = 0
pub.publish(velocity)
