#!/usr/bin/env python3

import rospy
import pygame
import time
from std_msgs.msg import Float32, Bool

#Initialize pygame and gamepad
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print ('Initialized Joystick : %s' % j.get_name())

def teleop_gamepad():
    #Car number you want to control
    carnumber = 4
    #If you can use throtthle
    bool_throttle = False
    bool_steer = False

    #Setup topics publishing and nodes
    pub_drive = rospy.Publisher('drive', Bool, queue_size=8)
    pub_steering = rospy.Publisher('steering'+str(carnumber), Float32, queue_size=8)
    pub_throttle = rospy.Publisher('throttle'+str(carnumber), Float32, queue_size=8)
    pub_mpc = rospy.Publisher('mpc_start', Bool, queue_size=8)
    rospy.init_node('teleop_gamepad', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    store = False

    while not rospy.is_shutdown():
        pygame.event.pump()
        
        #Obtain gamepad values
        steering = -j.get_axis(2)   #Right thumbstick X
        throttle = -j.get_axis(1)   #Right thumbstick X
        drive = j.get_button(9)       #right arrow press
        mpc = j.get_button(8)       #press button to start dmpc controller
        
        
        #output values
        print("Steering2 :", steering)
        print("throttle2 :", throttle)
        print("Drive :", drive)  
        print("mpc :", mpc)
        
        #start or stop mpc
        if mpc:
            if store:
                store = False
            else:
                store = True
                
            pub_mpc.publish(store)
                
        
        #Pubblish gamepad values
        pub_drive.publish(drive)
        if bool_steer:
            pub_steering.publish(steering)	
        
        if bool_throttle:
            pub_throttle.publish(throttle)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        teleop_gamepad()
    except rospy.ROSInterruptException:
        pass
