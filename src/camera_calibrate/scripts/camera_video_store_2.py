#!/usr/bin/env python

import time
import cv2
import numpy as np 
import rospy
 
 

capture_device = 0
capture_width = 1280
capture_height = 720
capture_fps = 60
# scaling the capturing image using same ratio (scaled down by 0.6 multiply)
width = 1280
height = 720


def _gst_str(self):
        return 
 
# Gstreamer code for improvded Raspberry Pi Camera Quality
camSet = 'nvarguscamerasrc sensor-id='+str(capture_device)+' ! video/x-raw(memory:NVMM), width='+str(capture_width)+', height='+str(capture_height)+', format=(string)NV12, framerate=(fraction)'+str(capture_fps)+'/1 ! nvvidconv ! video/x-raw, width=(int)'+str(width)+', height=(int)'+str(height)+', format=(string)BGRx ! videoconvert ! appsink'

cam=cv2.VideoCapture(camSet, cv2.CAP_GSTREAMER)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out=cv2.VideoWriter('video.avi', fourcc, capture_fps, (width,height))

rospy.init_node('csi_cam', anonymous=False)


rate = rospy.Rate(capture_fps)


while True:
    ret, img = cam.read()

    if ret == True:
	out.write(img)
    
    if cv2.waitKey(1)==ord('q'):
        break
    rate.sleep()

cam.release()
cv2.destroyAllWindows()
