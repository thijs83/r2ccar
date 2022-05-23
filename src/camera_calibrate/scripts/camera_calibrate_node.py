#!/usr/bin/env python

import time
import cv2
import numpy as np 
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 
 

capture_device = 0
capture_width = 3264
capture_height = 1848
capture_fps = 28
# scaling the capturing image using same ratio (scaled down by 0.6 multiply)
width = 1088 #768
height = 616 #432


def _gst_str(self):
        return 
 
# Gstreamer code for improvded Raspberry Pi Camera Quality
camSet = 'nvarguscamerasrc sensor-id='+str(capture_device)+' ! video/x-raw(memory:NVMM), width='+str(capture_width)+', height='+str(capture_height)+', format=(string)NV12, framerate=(fraction)'+str(capture_fps)+'/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)'+str(width)+', height=(int)'+str(height)+', format=I420 ! videobalance contrast=1.5 brightness=-0.25 ! appsink max-buffers=1 drop=true'

cam=cv2.VideoCapture(camSet, cv2.CAP_GSTREAMER)


rospy.init_node('csi_cam', anonymous=False)
image_pub = rospy.Publisher("/csi_cam/image_raw", Image, queue_size=1)
bridge = CvBridge()
rate = rospy.Rate(capture_fps)


while not rospy.is_shutdown():
    ret, img = cam.read()
    img = img[1:616,:]
    image_pub.publish(bridge.cv2_to_imgmsg(img, encoding="mono8"))
    print(img.shape)
    
    if cv2.waitKey(10)==ord('q'):
        break
    #rate.sleep()

cam.release()
