RAPTOR - Realtime adAPtive detecTOR
======================================

Implementation of the following paper
Daniel Göhring, Judy Hoffman, Erik Rodner, Kate Saenko, and Trevor Darrell.
Interactive Adaptation of Real-Time Object Detectors. International Conference on Robotics and Automation (ICRA). 2014

Details and paper: http://raptor.berkeleyvision.org/

Implemented by Daniel Göhring and Erik Rodner. Reorganized, cleaned, corrected by Eric McCann

Installation on Ubuntu
=======================

This demo consists of the object learner and detector.
The detector requires less modules so we start the installation guide with the detector. The current description
focuses on Ubuntu.

1. install ros fuerte (see guide under http://wiki.ros.org/fuerte/Installation/Ubuntu)
2. install git

    sudo apt-get install git

3. install fftw3

    sudo apt-get install fftw3
    sudo apt-get install libfftw3-dev

4. install ``openni+usb_cam``

    sudo apt-get install ros-fuerte-openni-* ros-fuerte-bosch-drivers
    
5. get raptor

    git clone https://github.com/BVLC/raptor.git

6. cd into that folder, then...

```
rosws init
rosws merge /opt/ros/fuerte
rosws merge workspace.rosinstall
source setup.bash
rosmake raptor

```

7. verify that this was done correctly by running the following line:

    rosversion ros

   the computer should output the version number.

8. Go to /data/demo-office.config.template, data/runDemo.sh, and raptor/demo-office.config, and raptor/src/transfer_images_torecord.cpp.  Change “goehring” with your login name, and adjust the paths if necessary.

9. To use a PrimeSense kinect instead of a webcam, follow the PrimeSense installation instructions on the following page; then edit the launch files by removing usb_cam..., and replacing it with "/camera/rgb/image_color"

10. Test the full detection setup, after execution of these 4 commands you should see an image with detection boxes:
```
roscore
rosrun usb_cam usb_cam_node 
roslaunch raptor monitor.launch 
roslaunch raptor detection.launch 
```

PrimeSense Installation
========================

1. Go to http://mitchtech.net/ubuntu-kinect-openni-primesense/ and follow the instructions for the first and third download.

2. For the second, DO NOT use the git clone git://github.com/avin2/SensorKinect.git download, instead, use https://github.com/PrimeSense/Sensor, which is a newer version.  However, the installation instructions are the same.
