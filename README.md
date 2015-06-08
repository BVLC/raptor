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

1. Install ROS Indigo (see guide under http://wiki.ros.org/indigo/Installation/Ubuntu)

2. Clone raptor into your catkin workspace, e.g. git clone https://github.com/BVLC/raptor.git

3. Install deps

```bash
rosdep install --from-paths raptor  --ignore-src
```

4. Build with catkin_make 

5. Go to raptor/demo-office.config and adjust the paths to point to the correct plan on your file system

6. Test the full detection setup, after execution of these 4 commands you should see an image with detection boxes:

```
roscore
rosrun usb_cam usb_cam_node 
roslaunch raptor monitor.launch img_topic:=/usb_cam/image_raw
roslaunch raptor detection.launch img_topic:=/usb_cam/image_raw
```

If you get errors fromt the cam node, you could try the launch file which sets some different parameters

```
roslaunch raptor usb_cam.launch
```
