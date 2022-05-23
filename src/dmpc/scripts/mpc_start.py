#!/usr/bin/env python3

import rospy
from getkey import getkey
from std_msgs.msg import Float32
from std_msgs.msg import Bool

def teleop():
    #Setup topics publishing and nodes
    pub_start = rospy.Publisher('mpc_start', Bool, queue_size=8)
    rospy.init_node('mpc_start_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #Print contol hints
    print("press q to start mpc")

    start_bool = False


    while not rospy.is_shutdown():
        #Get key press
        key = getkey()

        #Throttle
        if key == 'q':
            if start_bool:
                start_bool = False
                pub_start.publish(False)
                print("nothing")
            else:
                start_bool = True
                pub_start.publish(True)
                print("starting dmpc")
        else:
            print("wrong key")
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
