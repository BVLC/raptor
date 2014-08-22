#include "ros/package.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32MultiArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

/////////////

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <fstream>
#include <time.h>


///+1
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
///-1





using namespace std;
namespace enc = sensor_msgs::image_encodings;

class ObjectTrack{
public:

    pcl::PointXYZRGB content;


    bool recentlyUpdated;
    int lastFrameNumber;
    int lastFrameNumberPropagated;
    int trackId;
    int cyclesSinceLastUpdate;
    int cyclesAlive;
    int frameNumberSinceAlive;


    ObjectTrack(pcl::PointXYZRGB content, int frameNumber) {
        this->content = content;
        this->lastFrameNumber = frameNumber;
        this->lastFrameNumberPropagated = frameNumber;
        frameNumberSinceAlive = frameNumber;
        this->trackId = (int)((double)rand() / RAND_MAX *16384);
        cyclesSinceLastUpdate = 0;
        cyclesAlive = 0;
    }
    void update(pcl::PointXYZRGB cog, int frameNumber) {
       // this->lastFrameNumber = frameNumber;
        this->content = cog;
        cyclesSinceLastUpdate = 0;
    }
    void resetUpdate() {
        recentlyUpdated = false;
    }

    void propagate(int frameNumber) {
        cyclesSinceLastUpdate++;
        cyclesAlive++;
        lastFrameNumberPropagated = frameNumber;
    }

    int getTrackLifeSpanInFrameNumbers(){
        return (lastFrameNumberPropagated - lastFrameNumber);
    }

    int getTrackId() {
        return trackId;
    }
};



class TransferImages
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




public:


    TransferImages()
        : it_(nh_)
    {
        image_sub = it_.subscribe("image_in", 1, &TransferImages::imageSCb, this);
        image_pub = it_.advertise("/bolt/vision/image", 1);

        numOfReceivedImageFrames = 0;
        frameNumber = 0;

	lastModelId = -1;
        lastEntropy = 10;
	oldX0 = oldY0 = 5;
        oldX1 = 634; oldY1 = 474;
	lastDetectionScore = -1.0;

        detectionThreshold = -1.0;
        SCALE = 1;
	


    }

    ~TransferImages()
    {
    }



    /********************************************************************
    * get the image, write out image, execute DPM code on it and read back the two bouding box coordinates
    */

    void imageSCb(const sensor_msgs::ImageConstPtr& msg)
    {
        std::cerr << "+++++++++ new image data received " << std::endl;

//	std::cerr << ros::package::getPath("raptor") << std::endl;


        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8); //get the image
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }


//write smaller image
	cv::Mat img(cv_ptr->image);
       	//cv::Mat roi(img, cv::Rect(0,0,639,479));

	std::cerr << "Send XA " << oldX0 << " YA " << oldY0 << " XE " << oldX1 << " YE " << oldY1 << std::endl;

	//cv::Mat roi(img, cv::Rect(oldX0,oldY0,oldX1 - oldX0,oldY1 - oldY0));








	cv::Mat roi(img, cv::Rect(oldX0 / SCALE, oldY0 / SCALE, (oldX1 / SCALE - oldX0 / SCALE - 1), (oldY1 / SCALE - oldY0 / SCALE - 1)));


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


		
	if (SCALE > 1) {	

        // create smaller image: - only if scale > 1
	cv::Vec3b bgr;        
	for (int i = (oldX0 / SCALE); i < (oldX1 / SCALE); i++) {
	        for (int j = (oldY0 / SCALE); j < (oldY1 / SCALE); j++) {
			cv::Point pointSrc(i*SCALE,j*SCALE);
			bgr = cv_ptr->image.at<cv::Vec3b>(pointSrc);
			cv::Point pointDst(i,j);
			cv::rectangle(cv_ptr_dest->image, pointDst, pointDst, CV_RGB(bgr[2],bgr[1],bgr[0]));
		}
	}
	//
	}








/*
	cv::Mat roi(img, cv::Rect(0,0, (640/SCALE - 1), (480/SCALE - 1)));


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
	for (int i = 0; i < (640 / SCALE); i++) {
	        for (int j = 0; j < (480 / SCALE); j++) {
			cv::Point pointSrc(i*SCALE,j*SCALE);
			bgr = cv_ptr->image.at<cv::Vec3b>(pointSrc);
			cv::Point pointDst(i,j);
			cv::rectangle(cv_ptr_dest->image, pointDst, pointDst, CV_RGB(bgr[2],bgr[1],bgr[0]));
		}
	}
	//


        //get the path to the raptor package
        std::string path = ros::package::getPath("raptor");
        std::stringstream strstr, strstr_dest;
        strstr << path << "/../bottleX.ppm";
        strstr_dest << path << "/../bottleY.ppm";


        //write the image into the DPM folder - it's important, where the path is


//!!!!!!!!COMMENT BACK IN 2013-07-10
//        cv::imwrite(strstr.str().c_str(), cv_ptr->image);
//        cv::imwrite(strstr_dest.str().c_str(), cv_ptr_dest->image);
//

        //std::cerr << "Executing Bottle recognition." << std::endl;
        //std::stringstream sys_call;

        //execute the DPM code - it's important, where the path is again
        if (true) {
            sys_call << "cd " << path << "/../../../irobot_lib_iSpot_2.0.beta2_x86_64; ./cudafelz_example bottle1.ppm out.ppm";
            int dpmdet = system (sys_call.str().c_str());
            std::cerr << "after execution of dpm - return value " << dpmdet << "\n" << std::cerr;
        }

*/

        


///2+

bool connectToServer = true;

if (connectToServer)
{
    int sockfd = 0, n = 0;
    char recvBuff[1024];
    struct sockaddr_in serv_addr; 

   
    memset(recvBuff, '0',sizeof(recvBuff));
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error : Could not create socket \n");
        return;
    } 

    memset(&serv_addr, '0', sizeof(serv_addr)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(5000); 

    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
        return;
    } 

    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\n Error : Connect Failed \n");
       return;
    } 
	
//    char* MSG;
//    *MSG = "FOO";

     char buf[1024];
     memset(buf, '0', sizeof(buf));

/////////////////////////////////////////////////////////////66666666


std::stringstream aux;


std::cerr << "Execution... " << std::endl;


	//select Models, not resolution yet:
	if ((true) && ((lastEntropy > 2) || (lastModelId < 0) || (lastDetectionScore < detectionThreshold))) {
		std::cerr << "FOO... " << std::endl;

		for (int i = 0; i < 64; i++) {
			if (i%4 == 0) {		
				aux << "1,";
			} else {
				aux << "0,";
			}
		}
	}

	else /*if (lastEntropy > 2)*/ {
std::cerr << "KRA " << lastModelId << std::endl;
	for (int i = 0; i < 64; i++) {

		if (((i / 16) == (lastModelId / 16)) && ((i % 4) == 0)) {		
			aux << "1,";
		} else {
			aux << "0,";
		}
	} } /*else {  //grids
std::cerr << "BAR " << lastModelId << std::endl;
	for (int i = 0; i < 64; i++) {

		if ((i / 4) == (lastModelId / 4)) {
		//std::cerr << " 1 -"  << std::endl;
					
			aux << "1,";
		} else {
//std::cerr << " 0 -"  << std::endl;
			aux << "0,";
		}
	} 
}
*/

     snprintf(buf, sizeof(buf), "%s", aux.str().c_str());


 //snprintf(buf, sizeof(buf), " Das Pferd ");



     time_t ticks0 = time(NULL);
	//std::cerr << "time0 " << (int)ticks0 << std::endl;


	struct timespec start, stop;
	double accum;

	if ( clock_gettime(CLOCK_REALTIME, &start) == -1) {
		perror("clock gettime");
		//return EXIT_FAILURE;
	}


//struct timespec r;
//  clock_gettime(CLOCK_MONOTONIC, &r);



///+1
     write(sockfd, buf, sizeof(buf));


///-1

    while ( (n = read(sockfd, recvBuff, sizeof(recvBuff)-1)) > 0)
    {
        recvBuff[n] = 0;
  /*      if(fputs(recvBuff, stderr) == EOF)
        {
            printf("\n Error : Fputs error\n");
        }*/
    } 

//read data
   std::string token;//, text("Here,is,some,text");
   std::istringstream iss(recvBuff);


  int counter = 0;
  double resultField[10];
  while ( (getline(iss, token, ',')) && (counter < 10) )
   {

      std::cerr << token << std::endl;
      resultField[counter] = atof(token.c_str());
	counter++;
   }

   
   // For small image 2013-08-12
   cv::Point pointP0((int)(resultField[0]*SCALE+oldX0),(int)(resultField[2]*SCALE+oldY0));
   cv::Point pointP1((int)(resultField[1]*SCALE+oldX0),(int)(resultField[3]*SCALE+oldY0));

   //cv::Point pointP0((int)(resultField[0]+oldX0),(int)(resultField[2]+oldY0));
   //cv::Point pointP1((int)(resultField[1]+oldX0),(int)(resultField[3]+oldY0));



   cv::rectangle(cv_ptr->image, pointP0, pointP1, CV_RGB(255, 0, 0));

	std::cerr << "Detect XA " << resultField[0]*SCALE+oldX0 << " YA " << resultField[2]*SCALE+oldY0 << " XE " << resultField[1]*SCALE+oldX0 << " YE " << resultField[3]*SCALE+oldY0 << std::endl;



   int increaseArea = 100;   

   oldX1 = min(639,(int)(resultField[1]*SCALE+oldX0)+increaseArea);
   oldY1 = min(479,(int)(resultField[3]*SCALE+oldY0)+increaseArea);

   oldX0 = max(0,(int)(resultField[0]*SCALE+oldX0)-increaseArea);
   oldY0 = max(0,(int)(resultField[2]*SCALE+oldY0)-increaseArea);



   lastModelId = resultField[4]; 
   lastEntropy = resultField[5];
   lastDetectionScore = resultField[6];

	if (lastDetectionScore < detectionThreshold) {
		consecutiveFramesThresholdMet = 0;
		consecutiveFramesThresholdNotMet++;
	} else {
		consecutiveFramesThresholdMet++;
		consecutiveFramesThresholdNotMet = 0;
	}

	if (consecutiveFramesThresholdMet > 2) {
		SCALE = min(4, SCALE*2);
	}
	else
	if (consecutiveFramesThresholdNotMet < 2) {
		SCALE = max(1, SCALE/2);
	}

   if (((true) &&(lastDetectionScore < detectionThreshold)) || (SCALE > 1)) {
	oldX0 = 0;
	oldY0 = 0;
        oldX1 = 639;
        oldY1 = 439;

	}




   image_pub.publish(cv_ptr->toImageMsg());

     int BILLION = 1000000000;

     time_t ticks1 = time(NULL);
	//std::cerr << "time1 " << (int)ticks1 << std::endl;


	if ( clock_gettime(CLOCK_REALTIME, &stop) == -1) {
		perror("clock gettime");
		//return EXIT_FAILURE;
	}

	accum = (stop.tv_sec - start.tv_sec) + (double)(stop.tv_nsec - start.tv_nsec) / (double)BILLION;
	std::cerr << "TIME IN SECONDS " << accum << std::endl;




    if(n < 0)
    {
        printf("\n Read error \n");
    } 



///////////////////////////////////////////////////////////////77777777777


     //time_t ticks = time(NULL);
/*
	std::cerr << "sending stuff to socket" << std::endl;

     snprintf(buf, sizeof(buf), "Der PR2 frisst keinen Gurkensalat");


     write(sockfd, buf, sizeof(buf));
	

    while ( (n = read(sockfd, recvBuff, sizeof(recvBuff)-1)) > 0)
    {
        recvBuff[n] = 0;

	std::cerr << recvBuff << std::endl;

        if(fputs(recvBuff, stdout) == EOF)
        {
            printf("\n Error : Fputs error\n");
        }
    } 

    if(n < 0)
    {
        printf("\n Read error \n");
    } 


*/





 }


///2-




    } //imageScb


};






int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_img_pcl");

    TransferImages tip;
    ros::spin();
    return 0;
}

