RAPTOR - Realtime adAPtive detecTOR
======================================

Implementation of the following paper
Daniel Göhring, Judy Hoffman, Erik Rodner, Kate Saenko, and Trevor Darrell.
Interactive Adaptation of Real-Time Object Detectors. International Conference on Robotics and Automation (ICRA). 2014

Details and paper: http://raptor.berkeleyvision.org/

Implemented by Daniel Göhring and Erik Rodner. Reorganized, cleaned, corrected by Eric McCann. Catkinized and updated for ROS Indigo by Nick Hawes

Installation on Ubuntu
=======================

This demo consists of the object learner and detector.
The detector requires less modules so we start the installation guide with the detector. The current description
focuses on Ubuntu.

1. install ROS Indigo (see guide under http://wiki.ros.org/indigo/Installation/Ubuntu)
2. install git if you haven't already got it

    sudo apt-get install git

3. clone raptor into your catkin workspace, e.g. git clone https://github.com/BVLC/raptor.git

4. install deps

5. build

6. Go to raptor/demo-office.config and adjust the paths.

10. Test the full detection setup, after execution of these 4 commands you should see an image with detection boxes:
```
roscore
rosrun usb_cam usb_cam_node 
roslaunch raptor monitor.launch img_topic:=/usb_cam/image_raw
roslaunch raptor detection.launch img_topic:=/usb_cam/image_raw
```

