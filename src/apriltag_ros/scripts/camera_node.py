#!/usr/bin/env python

import time
import cv2
import numpy as np 
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
 


def q_set():
    q = CameraInfo()
    q.header.frame_id = 'csi_cam'
    q.height = 616
    q.width = 1088
    q.D = [-0.2647365650562701, 0.048655516368306714, 0.0037943248154841494, -8.068042653969374e-05, 0.0]
    q.K = [548.4914049314959, 0.0, 541.0631222400909, 0.0, 548.357719565556, 266.9416673033522, 0.0, 0.0, 1.0]
    q.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    q.P = [361.26739501953125, 0.0, 541.678137623785, 0.0, 0.0, 494.10791015625, 260.22868089348776, 0.0, 0.0, 0.0, 1.0, 0.0]
    q.binning_x = 0
    q.binning_y = 0
    q.roi.x_offset = 0
    q.roi.y_offset = 0
    q.roi.height = 0
    q.roi.width = 0
    q.roi.do_rectify = False
    return q



q = q_set()
capture_fps = 28/1

crop_y = 0
crop_height = 300
 
# Gstreamer code for improvded Raspberry Pi Camera Quality
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=1848, format=NV12, framerate=28/1 ! nvvidconv flip-method=2 ! video/x-raw, width='+str(q.width)+', height='+str(q.height)+', format=I420 ! videobalance contrast=1.5 brightness=-0.25 ! appsink max-buffers=1 drop=true'


cam=cv2.VideoCapture(camSet, cv2.CAP_GSTREAMER)

rospy.init_node('camera', anonymous=False)
image_pub = rospy.Publisher("/camera_rect3/image_rect", Image, queue_size=1)
info_pub = rospy.Publisher("/camera_rect3/camera_info", CameraInfo, queue_size=1)
bridge = CvBridge()
rate = rospy.Rate(capture_fps)

D = np.mat(q.D)
K = np.mat(q.K)
K = K.reshape(3,3)

cy_new = q.K[6]-crop_y
q.K[6] = cy_new
q.height = crop_height

while not rospy.is_shutdown():
    ret, img = cam.read()
    img_rect = cv2.undistort(img, K, D)
    img_crop = img_rect[crop_y : crop_y+crop_height, :]
    
    image_pub.publish(bridge.cv2_to_imgmsg(img_crop, encoding="8UC1"))   
    info_pub.publish(q)
    
    if cv2.waitKey(1)==ord('q'):
        break
    #rate.sleep()

cam.release()
cv2.destroyAllWindows()
        
        

    
        
        
