rostopic pub /bolt/vision/record std_msgs/String "test"
../who-imagenet-adapt/who_imagenet_adapt.sh $*
cp model.ffld.*.txt /home/james/ros_workspace/raptor/ffld_ros/ffld/outoffice/
CATEGORYNAME=`echo $* | sed 's/ /_/g'`
sed "s/newObject/$CATEGORYNAME/g" demo-office.config.template > $HOME/ros_workspace/bolt/ros/vision/kinect/demo-office.config 




