#include "ros/package.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32MultiArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <time.h>
#include <ffld/opencvToJPEGImage.h>
#include <ffld/DPMDetection.h>

#include <map>




using namespace std;
namespace enc = sensor_msgs::image_encodings;




class TransferImagesFFLD
{
    ros::NodeHandle nh_;
    ros::NodeHandle pt_;
    ros::NodeHandle dpmboxh;
    ros::NodeHandle pclboxh;

    ros::Subscriber cloud_sub;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
   // image_transport::Publisher image_pub_debug;
    image_transport::Publisher depth_pub;
    ros::Publisher dpmpoints_pub;
    ros::Publisher pclpoints_pub;

    cv_bridge::CvImagePtr cv_ptr, cv_ptr_dest;


    int frameNumber;
    int numOfReceivedImageFrames;
    int lastModelId;
    double lastEntropy;
    double lastDetectionScore;

    double detectionThreshold;

    int oldX0, oldY0, oldX1, oldY1;
    int consecutiveFramesThresholdMet;
    int consecutiveFramesThresholdNotMet;

    int SCALE;
    FFLD::DPMDetection* dpmdetect;
    map<string,int> colors;


public:


    TransferImagesFFLD()
        : it_(nh_)
    {
        image_sub = it_.subscribe("image_in", 1, &TransferImagesFFLD::imageSCb, this);
        image_pub = it_.advertise("/bolt/vision/image", 1);

        numOfReceivedImageFrames = 0;
        frameNumber = 0;


        dpmdetect = new FFLD::DPMDetection ( false );

	string path = "";
	path.append(ros::package::getPath("raptor"));
	path.append("/demo-office.config");

	dpmdetect->addModels(path);

	
	colors["bottle"] = 2;
	//colors["expo2"] = 3;
	//colors["listerine"] = 3;
	colors["minute_maid"] = 3;
	colors["mug"] = 5;
	//colors["pasta_box"] = 6;
	//colors["pringles"] = 0;
	//colors["soap_bottle"] = 4;
	//colors["toilet_paper"] = 5;
	//colors["vitamin_water"] = 7;
	//colors["Walgreens"] = 2;
	colors["bowl"] = 0;
	colors["clock"] = 1;
	colors["coca_cola"] = 3;
	colors["scissors"] = 4;
	colors["tupperware"] = 6;
	colors["wipes"] = 7;

    }

    ~TransferImagesFFLD()
    {
    }



    /********************************************************************
    * get the image, write out image, execute DPM code on it and read back the two bouding box coordinates
    */

    void imageSCb(const sensor_msgs::ImageConstPtr& msg)
    {

        //time_t ticks0 = time(NULL);
	//std::cerr << "time0 " << (int)ticks0 << std::endl;


	struct timespec start, stop;
	double accum;

	if ( clock_gettime(CLOCK_REALTIME, &start) == -1) {
		perror("clock gettime");
		//return EXIT_FAILURE;
	}




        std::cerr << "+++++++++ new image data received, model geladen " << std::endl;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8); //get the image
            //cv_ptr_dest = cv_bridge::toCvCopy(msg, enc::BGR8); //get the image

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }




        cv::Mat img(cv_ptr->image);


	cv::Mat roi(img, cv::Rect(0, 0, 319, 239));


		if(!roi.empty())
		{
		cv_bridge::CvImage destImg;
		destImg.encoding = sensor_msgs::image_encodings::BGR8;
		destImg.image = roi;
		sensor_msgs::ImagePtr rosMsg = destImg.toImageMsg();
		rosMsg->header.frame_id = "camera";
		rosMsg->header.stamp = ros::Time::now();
		//rosPublisher_.publish(rosMsg);

		cv_ptr_dest = cv_bridge::toCvCopy(rosMsg, enc::BGR8);
		}


		

        // create smaller image: - only if scale > 1
	cv::Vec3b bgr;        
	for (int i = 0; i < 319; i++) {
	        for (int j = 0; j < 239; j++) {
			cv::Point pointSrc(i*2,j*2);
			bgr = cv_ptr->image.at<cv::Vec3b>(pointSrc);
			cv::Point pointDst(i,j);
			cv::rectangle(cv_ptr_dest->image, pointDst, pointDst, CV_RGB(bgr[2],bgr[1],bgr[0]));
		}
	}



//smaller image end

 cv::Mat imgSmall(cv_ptr_dest->image);



	FFLD::JPEGImage *image = opencvToJPEGImage ( imgSmall );


	
	std::vector< FFLD::Detection > detections;
        int errcode = dpmdetect->detect ( *image, detections );


	std::cerr << "Detection " << detections.size() << " Errcode " << errcode << std::endl;

	int x = 0;
	int y = 0;

	 for (int i = 0; i < std::min(10,(int)detections.size()); i++ ) {

		int SCALE = 2;

		cv::Point pointResult1(detections[i].left()*SCALE,detections[i].top()*SCALE);
		cv::Point pointResult2(detections[i].right()*SCALE,detections[i].bottom()*SCALE);		


	x = max(30, min(320, detections[i].left()*SCALE));
	y = max(40, min(420, detections[i].top()*SCALE-10));

	cv::Point foo(x,y);


		int color = 0;
		if (colors.find(detections[i].classname)!=colors.end()) {
			color = colors[detections[i].classname];
		} else {
			color = 2;
		}
 
		
		cv::Scalar colorScale;

		
		if (color == 0) {colorScale = CV_RGB(255,0,0);} else
		if (color == 1) {colorScale = CV_RGB(255,63,0);} else
		if (color == 2) {colorScale = CV_RGB(255,128,0);} else
		if (color == 3) {colorScale = CV_RGB(255,255,0);} else
		if (color == 4) {colorScale = CV_RGB(0,255,0);} else
		if (color == 5) {colorScale = CV_RGB(0,128,128);} else
		if (color == 6) {colorScale = CV_RGB(0,0,255);} else
		 {colorScale = CV_RGB(128,0,255);}



		cv::rectangle(cv_ptr->image, pointResult1, pointResult2, colorScale,3);
        	cv::putText(cv_ptr->image, detections[i].classname, foo, 1, 2, colorScale, 3, 8);
		


	}

		


	

//detections[i].left() + 1) << ' '
//          << (detections[i].top() + 1) << ' ' << (detections[i].right() + 1) << ' '
//          << (detections[i].bottom() + 1) << endl;




	
	image_pub.publish(cv_ptr->toImageMsg());


     int BILLION = 1000000000;

     //time_t ticks1 = time(NULL);
	//std::cerr << "time1 " << (int)ticks1 << std::endl;


	if ( clock_gettime(CLOCK_REALTIME, &stop) == -1) {
		perror("clock gettime");
		//return EXIT_FAILURE;
	}

	accum = (stop.tv_sec - start.tv_sec) + (double)(stop.tv_nsec - start.tv_nsec) / (double)BILLION;
	std::cerr << "TIME IN SECONDS " << accum << " Frequency " << 1.0/accum  << std::endl;




}

};






int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_img_pcl");

    TransferImagesFFLD tip;
    ros::spin();
    return 0;
}

