# R2C car (Modified Jetracer Pro AI)

This is the build and code for the cooperative driving platform R2C platform. This platform exists of multiple modified JetRacer Pro AI's. The platform is a lowcost alternative for cooperative driving experiments.




**Note: The code used is free to be modified and used. The author is not responsible for any damages accoring during the use of this code.**


### The Car

The R2C car utilises the 8MP 160 deg FoV camera and an added YDLIDAR X4 for a 2D pointcloud for object recognition and distance measurements. 2 encoders are added to measure the RPM of the main gear, which is converted to a longitudinal velocity of the main body. 


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


installallation of the [AprilTag github](https://github.com/AprilRobotics/apriltag)

```bash
cd
mkdir apriltag
cd apriltag
mkdir build
```

Download the [AprilTag github](https://github.com/AprilRobotics/apriltag) folder to the Downloads folder. Then, copy the content in the Downloads/apriltag/ to the build folder.

```bash
cd
cd apriltag/build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cd ..
sudo cmake --build apriltag --target install
```

And finally the image geometry needed for publishing the images to the apriltag detector

```bash
sudo apt-get install ros-melodic-image-geometry
```

### Download repository and build

Create a new folder:
```bash
cd
mkdir r2ccar
cd r2ccar
git clone https://github.com/thijs83/r2ccar.git
catkin_make
```


### Check camera image

There exists a problem with the waveshare cameras where the camera can give a reddish poor quality image. Follow the instructions [here](follow the instructions on: https://www.waveshare.com/wiki/IMX219-160_Camera) to resolve this problem. 

### Calibration

Apriltag needs a rectified image, therefore the camera should be calibrated. The guidelines for calibration can be found in the documentation below for the camera_calibrate package.


### Common errors

> If an error accors during make (No rule to make target '/usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.3.2.0), Opencv was not installed correctly, then use the command:
```bash
sudo apt install libopencv3.2
```

> If problems with cvbridge occur during catkin make use the command:
```bash
sudo ln -s /usr/include/opencv4/opencv2/ /usr/include/opencv
```

> If during catkin_make the build process gives an error about flann::search than use commands:
```bash
cd /usr/include/pcl-1.8/pcl/kdtree/
sudo gedit kdtree_flann.h
```

and change:
```cpp
      /** \brief The KdTree search parameters for K-nearest neighbors. */
      ::flann::SearchParams param_k_;

      /** \brief The KdTree search parameters for radius search. */
      ::flann::SearchParams param_radius_;
```
to:
```cpp
      /** \brief The KdTree search parameters for K-nearest neighbors. */
      ::flann::SearchParams *param_k_;

      /** \brief The KdTree search parameters for radius search. */
      ::flann::SearchParams *param_radius_;
```

## Arduino

The arduino is connected to the encoders using the two digital interrupt pins of the Arduino Nano. For the first time connecting the arduino to the jetson nano, run the script startup_arduino.sh in a terminal.
```bash
cd
cd r2ccar/arduino
./startup_arduino
```

Now the USB port to the arduino is called arduino. verify with:
```bash
cd
ls /dev/a*
```
You should see the file /dev/arduino. If the arduino is connected and /dev/arduino is not visible than you have to modify the ID in the script to the correct microcontroller you are using.

In the arduino folder you can also find the code that needs to be uploaded to the arduino for the use of the two encoders. The code measures the average period between detections inside a constant time window and converts this to a velocity measurement using the gear ratio and wheel radius. This velocity measurement is send to the Jetson Nano using the USB connection.




## Docs

Each of the packages is discussed in their own section

### apriltag_ros

**Note: This is a modified version of the package obtained from [AprilRobotics/apriltag_ros](https://github.com/AprilRobotics/apriltag_ros)**

The continuous apriltag detector can be launched using:
```bash
roslaunch apriltag_ros continuous_detection.launch
```
This will launch the AprilTag detector and a camera node. The camera node found in the scripts folder contains all the camera calibration data and camera capture settings. The images received from the camera module using Gstreamer are rectified and cropped to finally be published. The Gstreamer gets the image from the camera module in the NV12 format, and this needs to be changed to a format that opencv can use. Most used format is RGB but the problem is that conversion is done on the CPU and is heavy for this format. With the I420 format the conversion process is less heavy and we can take out the grey image directly for the apriltag algorithm. 

To view the camera feed, start the image_node.py node and use:
```bash
rqt_image_view
```
and subscribe to the image publisher to view the output.

The config folder contains two files, one for the apriltag settings and another for the tags that you want to detect in the image feed. For the settings and more info on the detector go to [AprilRobotics/apriltag_ros](https://github.com/AprilRobotics/apriltag_ros). In the tags file you have to specify the ID of the used tag or tags, to find the tags and more information see [AprilRobotics/apriltag_ros](https://github.com/AprilRobotics/apriltag_ros).


### auto_jetracer


### camera_calibrate

To find the intrinsic values 

```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/csi_cam/image_raw camera:=/csi_cam --no-service-check
```
to calibrate the camera
(needs way more info)
--

### cytron_jetracer

An easy package to start basic commands on the JetRacer Pro AI or R2C car. The original code can be found on [CytronTechnologies/cytron_jetracer github](https://github.com/CytronTechnologies/cytron_jetracer). Some minor modification are made. One of the most important changes is the steering gain in the racecar.py file. The original setting of 1 distroyed two of the original servo motors. The wheels only turn between the values -0.3 to 0.3, to still have the -1 to 1 range as input a gain of 0.3 is used. This will cause the servo motor to only work in the reachable range of the steering. More information and examples to start driving can be found in 


**Note: The JetRacer Pro AI steering can change in the future, therefore check for yourself the best steering_gain to achieve the maximum reachable range for the steering.**

### dmpc



### lidar_ros



# Cite
