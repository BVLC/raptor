This demo consists of the object learner and detector
The detector requires less modules so we start the installation guide with the detector.

==================================================

1. Installation guide to run the detector only:

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
source setup.bash
rosws merge workspace.rosinstall
rosws up
source setup.bash
rosmake -a
```

#Test the full detection setup, after execution of these 4 commands you should see an image with detection boxes:

1. roscore
2. ** start any webcam node or openni_launch, and modify monitor.launch and detection.launch appropriately **
3. roslaunch raptor monitor.launch 
4. roslaunch raptor detection.launch 

==================================================
