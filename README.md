# R2C car (Modified Jetracer Pro AI)

This is the build and code for the cooperative driving platform R2C platform. This platform exists of multiple modified JetRacer Pro AI's. The platform is a lowcost alternative for cooperative driving experiments.




**Note: 


### The Car

The R2C car utilises the 8MP 160 deg FoV camera and an added YDLIDAR X4 for a 2D pointcloud for object recognition and distance measurements. 2 encoders are utilised to measure the RPM of the main gear, which is converted to a longitudinal velocity of the main body. 


### Code

The code is tested on:
* Ubuntu 18.04 LTS
* ROS Melodic

First follow the Setup below and then download this github page. Documentation for each of the packages is presented below.

## Setup

### Drivers
To setup the JetRacer Pro AI or R2C car, follow the instructions on: [JetRacer Setup Instructions](https://www.waveshare.com/wiki/JetRacer_AI_Kit).

The Ubuntu version used by the Waveshare image at the time of building this project is 18.04 LTS and this could change in the future. For the Ubuntu 18.04 LTS version, the ROS Melodic version should be used. Installation instructions can be found [here] (http://wiki.ros.org/melodic/Installation/Ubuntu).

The driver should be downloaded for the LiDAR, which is in our case the YDLIDAR X4. The installation instructions can be found [here](https://github.com/YDLIDAR/YDLidar-SDK).

**Note: if dev/ydlidar not working then reattach the usb cables to the jetson nano.**


### AprilTag


installallation of the AprilTag github

$ cd
$ mkdir apriltag
$ cd apriltag
$ mkdir build

Download the github folder to the Downloads folder. Then, copy the content in the Downloads/apriltag/ to the build folder.

$ cd
$ cd apriltag/build
$ cmake -B build -DCMAKE_BUILD_TYPE=Release
$ cd ..
$ sudo cmake --build apriltag --target install

And finally the image geometry needed for publishing the images to the apriltag detector

$ sudo apt-get install ros-melodic-image-geometry

### Download repository and build




### Check camera image

There exists a problem with the waveshare cameras where the camera can give a reddish poor quality image. Follow the instructions [here](follow the instructions on: https://www.waveshare.com/wiki/IMX219-160_Camera) to resolve this problem. 

### Calibration

apriltag needs a calibrated camera so use :


rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/csi_cam/image_raw camera:=/csi_cam --no-service-check
to calibrate the camera
(needs way more info)
--


### Common errors

> If an error accors during make (No rule to make target '/usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.3.2.0), Opencv was not installed correctly, then use the command:
$ sudo apt install libopencv3.2

> If problems with cvbridge occur during catkin make use the command:
$sudo ln -s /usr/include/opencv4/opencv2/ /usr/include/opencv

> If during catkin_make the build process gives an error about flann::search than use commands:
$ cd /usr/include/pcl-1.8/pcl/kdtree/
$ sudo gedit kdtree_flann.h

and change:
'''
      /** \brief The KdTree search parameters for K-nearest neighbors. */
      ::flann::SearchParams param_k_;

      /** \brief The KdTree search parameters for radius search. */
      ::flann::SearchParams param_radius_;
'''
to:
'''
      /** \brief The KdTree search parameters for K-nearest neighbors. */
      ::flann::SearchParams *param_k_;

      /** \brief The KdTree search parameters for radius search. */
      ::flann::SearchParams *param_radius_;
'''
## Docs




#### Cite
