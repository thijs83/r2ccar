#!/usr/bin/env python

import cv2
import numpy as np
print(cv2.__version__)
dispW=1088
dispH=616
flip=2
#Uncomment These next Two Line for Pi Camera wbmode=3 tnr-mode=2 tnr-strength=1 ee-mode=2 ee-strength=1
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=1848, format=NV12, framerate=28/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=I420 ! videobalance contrast=1.5 brightness=-0.25 ! appsink max-buffers=1 drop=true'

cam= cv2.VideoCapture(camSet)

#nomal lens
#D = np.array([-0.2647365650562701, 0.048655516368306714, 0.0037943248154841494, -8.068042653969374e-05, 0.0])
#K = np.array([[548.4914049314959, 0.0, 541.0631222400909], [0.0, 548.357719565556, 266.9416673033522], [0.0, 0.0, 1.0]])
#R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
#P = [361.26739501953125, 0.0, 541.678137623785, 0.0, 0.0, 494.10791015625, 260.22868089348776, 0.0, 0.0, 0.0, 1.0, 0.0]


#fish-eye lens
D = [-0.017674241899362658, 0.17137802660743642, 0.20056120800559066, -0.15131438229408253]
K = [408.64876603348665, -2.313759168826387, 545.1673945872614, 0.0, 407.7371331655002, 307.189104083113, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [408.64876603348665, -2.313759168826387, 545.1673945872614, 0.0, 0.0, 407.7371331655002, 307.189104083113, 0.0, 0.0, 0.0, 1.0, 0.0]

#newcameramtx, roi = cv2.getOptimalNewCameraMatrix(np.float32(K), np.float32(D), (dispW,dispH), 1, (dispW,dispH))
D = np.mat(D)
K = np.mat(K)
K = K.reshape(3,3)

while True:
    ret, frame = cam.read()
    #cv2.imshow('nanoCam',frame)
    #cv2.moveWindow('nanoCam',0,0)
    frame = frame[1:616,:]
    dst = cv2.undistort(frame, K, D)
    cropped_frame = dst[0:300,:]
    cv2.imshow('croppedCam',dst)
    cv2.moveWindow('croppedCam',0,0)
    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
cv2.destroyAllWindows()
