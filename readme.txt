This demo consists of the object learner and detector
The detector requires less modules so we start the installation guide with the detector.

==================================================

      ***﻿Raptor Installation and Configuration Instructions***

#install ros fuerte

#see guide under http://wiki.ros.org/fuerte/Installation/Ubuntu

#install git
sudo apt-get install git

#install fftw3
sudo apt-get install fftw3
sudo apt-get install libfftw3-dev

#install openni+usb_cam
sudo apt-get install ros-fuerte-openni-* ros-fuerte-bosch-drivers

git clone https://github.com/BVLC/raptor.git

cd into that folder, then...

```
rosws init
rosws merge /opt/ros/fuerte
rosws merge workspace.rosinstall
source setup.bash
rosmake -a

```
#verify that this was done correctly by running the following line:
rosversion ros
#the computer should output the version number.

#Go to /data/demo-office.config.template, data/runDemo.sh, and raptor/demo-office.config, and raptor/src/transfer_images_torecord.cpp.  Change “goehring” with your login name, and adjust the paths if necessary.

#To use a PrimeSense kinect instead of a webcam, follow the PrimeSense installation instructions on the following page; then edit the launch files by removing usb_cam..., and replacing it with "/camera/rgb/image_color"

#Test the full detection setup, after execution of these 4 commands you should see an image with detection boxes:

1. roscore
2. rosrun usb_cam usb_cam_node 
3. roslaunch raptor monitor.launch 
4. roslaunch raptor detection.launch 







                      ***PrimeSense Installation***

#Go to http://mitchtech.net/ubuntu-kinect-openni-primesense/ and follow the instructions for the first and third download.

#For the second, DO NOT use the git clone git://github.com/avin2/SensorKinect.git download, instead, use https://github.com/PrimeSense/Sensor, which is a newer version.  However, the installation instructions are the same.
