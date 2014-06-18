rostopic pub /bolt/vision/record std_msgs/String "test"
../who-imagenet-adapt/who_imagenet_adapt.sh 1.0 $*
cp model.ffld.*.txt /home/icsi/aya/work/ros_workspace/ffld/outoffice/
CATEGORYNAME=`echo $* | sed 's/ /_/g'`
sed "s/newObject/$CATEGORYNAME/g" demo-office.config.template > $HOME/aya/work/ros_workspace/bolt/ros/vision/kinect/demo-office.config 




