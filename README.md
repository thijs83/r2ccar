# jetracer pro build

## Build
building the jetracer for this package

(make seperate docs folder with info and images)

## Setup

follow the instructions on:
https://www.waveshare.com/wiki/JetRacer_AI_Kit
to setup the beginning of the jetracer pro

--
install ros melodic (this build is build upon melodic)
http://wiki.ros.org/melodic/Installation/Ubuntu

--
install driver for the lidar
https://github.com/YDLIDAR/YDLidar-SDK

if dev/ydlidar not working then reattach the usb cables to the jetson nano

--
if camera not good quality
follow the instructions on: https://www.waveshare.com/wiki/IMX219-160_Camera

--
install Apriltag github

-clone apriltag to Downloads
cd
mkdir apriltag
cd apriltag
mkdir build
-copy content from Downloads/apriltag to build folder
cmake -B build -DCMAKE_BUILD_TYPE=Release
cd ..
sudo cmake --build apriltag --target install



sudo apt-get install ros-indigo-image-geometry

--

install : from github jetson_csi_cam
change in jetson_csi_cam.launch in Define the GSCAM pipeline: nvcamerasrc to nvarguscamera and remove the end video/x-raw, format=(string)BGR !

apriltag needs a calibrated camera so use :
roscore
roslaunch jetson_csi_cam jetson_csi_cam.launch
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/csi_cam/image_raw camera:=/csi_cam --no-service-check
to calibrate the camera
(needs way more info)
--
copy this github and use catkin_make to build all packages


if an error accors during make (No rule to make target '/usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.3.2.0), Opencv was not installed correctly, then use the command:
sudo apt install libopencv3.2


--
if problems with cvbridge occur during catkin make use in terminal:
sudo ln -s /usr/include/opencv4/opencv2/ /usr/include/opencv


-- 
flann::search error go to:
cd /usr/include/pcl-1.8/pcl/kdtree/
sudo gedit kdtree_flann.h

and change:
      /** \brief The KdTree search parameters for K-nearest neighbors. */
      ::flann::SearchParams param_k_;

      /** \brief The KdTree search parameters for radius search. */
      ::flann::SearchParams param_radius_;

to:
      /** \brief The KdTree search parameters for K-nearest neighbors. */
      ::flann::SearchParams *param_k_;

      /** \brief The KdTree search parameters for radius search. */
      ::flann::SearchParams *param_radius_;

## packages in this build
