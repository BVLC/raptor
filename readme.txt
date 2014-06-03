This demo consists of the object learner and detector
The detector requires less modules so we start the installation guide with the detector.

==================================================

1. Installation guide to run the detector only:

#install ros fuerte

#see guide under http://wiki.ros.org/fuerte/Installation/Ubuntu

#install git
sudo apt-get install git

#create a folder, e.g. raptor.berkeleyvision, cd into that folder, then execute:
git clone https://github.com/BVLC/raptor.git

copy the content under ros_workspace

#install fftw3
sudo apt-get install fftw3
sudo apt-get install libfftw3-dev

cd ffld
cd build
cmake ..
make

#change permission to folder cfg and its subfiles
chmod 777 cfg
chmod 777 cfg/*

compile bosch/usb_cam  (under ros_workspace, you can also checkout the usb-cam-stack yourself under: 

svn http://svn.code.sf.net/p/bosch-ros-pkg/code/trunk/stacks/bosch_drivers
)


#therefore create a build folder 'bosch_drivers/usb_cam/build', cd into 'build' and execute 'cmake ..' and make

#test if the camera works, you should see an image: 
roscore
rosrun usb_cam usb_cam_node
rosrun image_view image_view image:=/usb_cam/image_raw

#adjust paths (change 'goehring' into your login name plus make sure the rest of the folders match your environment) in data/demo-office.config.template and runDemo.sh

#Test the full detection setup, after execution of these 4 commands you should see an image with detection boxes:

1. roscore
2. rosrun usb_cam usb_cam_node 
3. roslaunch raptor monitor.launch 
4. roslaunch raptor detection.launch 

==================================================
