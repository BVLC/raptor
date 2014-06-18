#include "ros/package.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <iostream>
#include <time.h>

#include "std_msgs/String.h"
 
//rostopic pub /bolt/vision/record std_msgs/String "test"





using namespace std;
namespace enc = sensor_msgs::image_encodings;




class TransferImagesToRecord
{
    ros::NodeHandle nh_;
    ros::NodeHandle pt_;

    
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    ros::Subscriber image_sub2;

    image_transport::Publisher image_pub;
   // image_transport::Publisher image_pub_debug;
    image_transport::Publisher depth_pub;
    ros::Publisher dpmpoints_pub;
    ros::Publisher pclpoints_pub;

    cv_bridge::CvImagePtr cv_ptr, cv_ptr_orig;

    bool recordNow;
    std::string content;
    int numOfImages;

    int recordEveryNthFrame;
    int recordKFrames;
    std::string dataDir;


public:


    TransferImagesToRecord()
        : it_(nh_)
    {
        image_sub = it_.subscribe("image_in", 1, &TransferImagesToRecord::imageSCb, this);
        image_sub2 = pt_.subscribe("/bolt/vision/record", 1, &TransferImagesToRecord::imageSCb2, this);

        image_pub = it_.advertise("/bolt/vision/image", 1);
	numOfImages = 0;
        
	recordNow = false;
	recordEveryNthFrame = 60;
	recordKFrames = 5;
	dataDir = "/home/james/ros_workspace/data";


/*	string path = "";
	path.append(ros::package::getPath("raptor"));
	path.append("/../data");

	dataDir = path;*/




    }

    ~TransferImagesToRecord()
    {
    }



    void imageSCb2(const std_msgs::String &msg)
    {
		ROS_INFO("I heard: [%s]", msg.data.c_str());

		if (!recordNow) {

		recordNow = true;
		content = msg.data.c_str();
		std::cerr << "received" << msg << std::endl;
		numOfImages = 0;

				std::string path = dataDir;  //ros::package::getPath("kinect");
				ofstream myfile;
				
				std::stringstream strstr2;
				strstr2 << path << "/newexamples.cfg";
				
				myfile.open (strstr2.str().c_str());
				myfile.close();





		}
    }




    /********************************************************************
    * get the image, write out image, execute DPM code on it and read back the two bouding box coordinates
    */

    void imageSCb(const sensor_msgs::ImageConstPtr& msg)
    {
        std::cerr << "+++++++++ new image data received, No  " << numOfImages << std::endl;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8); //get the image
            cv_ptr_orig = cv_bridge::toCvCopy(msg, enc::BGR8); //get the image

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
 	


		int x0, y0, x1, y1;
		x0 = 120;
		y0 = 40;
		x1 = 519;
		y1 = 439;



		cv::Point pointResult1(x0,y0);
		cv::Point pointResult2(x1, y1);		
		cv::rectangle(cv_ptr->image, pointResult1, pointResult2, CV_RGB(255,0,0), 2);




////////////////////////////BEGIN

//paint backfround transparent

cv::Vec3b bgr;        
	for (int i = 0; i < 639; i++) {
	        for (int j = 0; j < 479; j++) {

			if ((i < x0) || (j < y0) || (i > x1) || (j > y1)) {
			cv::Point pointSrc(i,j);
			bgr = cv_ptr->image.at<cv::Vec3b>(pointSrc);
			cv::Point pointDst(i,j);
			cv::rectangle(cv_ptr->image, pointDst, pointDst, CV_RGB(bgr[2]*0.5,bgr[1]*0.5,bgr[0]*0.5));
			
			if (recordNow) {
				double percentageToNextRecord = ((double)(numOfImages % recordEveryNthFrame)) / recordEveryNthFrame;
				
				if ((479-j) < percentageToNextRecord * 480 ) {
			cv::rectangle(cv_ptr->image, pointDst, pointDst, CV_RGB(bgr[2],bgr[1],bgr[0]));

				} else {
			cv::rectangle(cv_ptr->image, pointDst, pointDst, CV_RGB(255-bgr[2],255-bgr[1],255-bgr[0]));

				}

				
			}


			}
		}
	}
	




//////////////////////////////END









	



	if (recordNow && ((numOfImages % recordEveryNthFrame) == 0)) {

				std::cerr << " record .............................. " << std::endl;

			 	std::string path = dataDir;   //ros::package::getPath("kinect");
				std::stringstream strstr;
				strstr << path << "/newObject"<< (int)(numOfImages / recordEveryNthFrame) <<".ppm";
				cv::imwrite(strstr.str().c_str(), cv_ptr_orig->image);


				ofstream myfile;
				
				std::stringstream strstr2;
				strstr2 << path << "/newexamples.cfg";
				
				myfile.open (strstr2.str().c_str(), ofstream::app);

				myfile << "newObject"<< (int)(numOfImages / recordEveryNthFrame) <<".ppm" << " " << content << " " << x0 << " " << y0 << " " << x1 << " " << y1 << "\n"; 
				myfile.close();




	
	for (int i = 0; i < 639; i++) {
	        for (int j = 0; j < 479; j++) {

			cv::Point pointSrc(i,j);
			bgr = cv_ptr->image.at<cv::Vec3b>(pointSrc);
			cv::Point pointDst(i,j);
			if ((i < x0) || (j < y0) || (i > x1) || (j > y1)) {
				cv::rectangle(cv_ptr->image, pointDst, pointDst, CV_RGB(255,0,0));
			} else {
				cv::rectangle(cv_ptr->image, pointDst, pointDst, CV_RGB(255,bgr[1]*0.5,bgr[0]*0.5));
			}
			
		}
	}



		

		std::cerr << "Recording new image " << std::endl;


		//content
	}



	if (numOfImages >= (recordEveryNthFrame * (recordKFrames - 1))) {
		recordNow = false;
		//numOfImages = -1;
	} 

	image_pub.publish(cv_ptr->toImageMsg());

	numOfImages++;

}

};






int main(int argc, char** argv)
{
    ros::init(argc, argv, "TransferImagesToRecord");

    TransferImagesToRecord tip;
    ros::spin();
    return 0;
}

