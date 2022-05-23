#!/usr/bin/env python

import cv2
import numpy as np 
 
 

capture_device = 0
capture_width = 1280
capture_height = 720
capture_fps = 120
# scaling the capturing image using same ratio (scaled down by 0.6 multiply)
width = 1280 
height = 720 


def _gst_str(self):
        return 
 
# Gstreamer code for improvded Raspberry Pi Camera Quality
camSet = 'nvarguscamerasrc sensor-id='+str(capture_device)+' ! video/x-raw(memory:NVMM), width='+str(capture_width)+', height='+str(capture_height)+', format=(string)NV12, framerate=(fraction)'+str(capture_fps)+'/1 ! nvvidconv ! video/x-raw, width=(int)'+str(width)+', height=(int)'+str(height)+', format=(string)BGRx ! videoconvert ! appsink'

cam=cv2.VideoCapture(camSet, cv2.CAP_GSTREAMER)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out=cv2.VideoWriter('video.avi', fourcc, 120, (1280,720))



while True:
    ret, img = cam.read()
    
    if ret == True:
	out.write(img)
    
    if cv2.waitKey(1)==ord('q'):
        break


cam.release()
out.release()
cv2.destroyAllWindows()
