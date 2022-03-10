# PS4_Stereo_ROS_SLAM
Development of a ROS node for the PS4 VR usb camera. This will allow SLAM to be performed using the VR camera and ROS Noetic. In short, creating a basic 3D scanner for less than $50.

![PS4 VR camera + Handle](https://i.imgur.com/ChRJjp5.jpg)
## Purpose
This project served as a refresher for ROS Noetic and to give me the opportunity to educate myself about the `stereo_image_proc` and `rtab-map` ROS nodes. The code and results from this project will be rolled into further thesis work. I will be applying similar SLAM techniques to autonomous navigation in offroad 'hiking trail' environments in the near future. To my knowledge this is the first implementation of RTAB-Map using the Playstation Four VR camera (definetly first in ROS Noetic).
## Requirements
The following packages and softwares are required. A good understanding of how each of them function will also help mitigate any issues.

 1. Ubuntu 20.04
 2. ROS Noetic
 3. PS4 Camera (modified to connect through USB 3.0)
 4. Camset ([git repo](https://github.com/azeam/camset))
 5. camera_calibration ROS package ([ROS page](http://wiki.ros.org/camera_calibration))
 6. rtabmap_ros ROS package ([ROS page](http://wiki.ros.org/rtabmap_ros))

## Setup

 1. Fork this repository and insatll all requirements. This assume you already have a ROS environment setup and have a intermediate understanding of how ROS functions.
 2. Load the usb firmware on to the PS4 camera using the python script found in the *Firmware_loader* folder `sudo python3 ps4eye_init.py`. The output should confirm the camera has been setup.
 3. Run camset (`camset`) and mark down the input device number of the PS4 camera. Set the cameras exposure to *shutter priority* instead of *automatic*.
 4. In `psvr_cam_publisher.py` set the input device number to the number marked down from step 3. Only the variable at the top requires changing.
 5. Complete camera calibration. A sample calibration grid has been provided, be sure to mount it to a flat surface like cardboard or a binder.
 6. Run the `psvr_launch` launch file. `roslaunch PS4_Stereo_ROS_SLAM psvr_launch.launch`

## Node Map
If setup correctly the node map should look like the following.

![example nodemap here](https://i.imgur.com/3PNwTHu.png)
## Calibration
The following steps should be taken to calibrate your PS4 camera. The repository comes with calibrations files but more accurate results can be achieved after calibrations.

 1. Download, print, and build a calibration board
 2. Run the camera_calibration ROS package and follow their calibration tutorial :wink:.
 3. Instead of commiting the calibration parameters to the camera, save the calibration parameters to a file (the file will be save to the tmp/ folder)
 4. Copy and paste the left and right camera calibration parameters to their respective .txt files found in the calibration folder. Be sure to read the included comments in each file.

## Example Results
TO BE POSTED
## Extras
A few extras have been included in this package.

 1. The printable handle as an .stl file. 
 2. A small 8.5"x11" calibration board (each square is approximately 22mm at 100% size)
 3. Two rqt_reconfigure tunning files. One for StereoBM and one for StereoSGBM
