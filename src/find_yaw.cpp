//Size of the image is 640*360
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ardrone_autonomy/navdata_altitude.h"
#include "image_process.h"

using namespace cv;
using namespace std;

class FindYaw
{
public: 
	FindYaw();
private:
	ros::NodeHandle n;
	ros::Subscriber image_sub;

	IplImage *source_image;
	IplImage *source_image_resized;
	IplImage *corners1;
	IplImage *corners2;
	IplImage temp;
	IplConvKernel * myModel;
	int cornersCount;
	CvPoint2D32f *corners;
	double quality_level; 
    double min_distance;

	void imageCallback(const sensor_msgs::Image &msg);
};

FindYaw::FindYaw()
{
	image_sub = n.subscribe("/videofile/image_raw", 1, &FindYaw::imageCallback,this);
	source_image_resized = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
	corners1 = cvCreateImage(cvSize(640,360),IPL_DEPTH_32F, 1);
	corners1 = cvCreateImage(cvSize(640,360),IPL_DEPTH_32F, 1);
	myModel = cvCreateStructuringElementEx(30,30,2,2,CV_SHAPE_RECT);
	corners = new CvPoint2D32f[50];
	quality_level=0.05;
	min_distance=10;
}

void FindYaw::imageCallback(const sensor_msgs::Image &msg)
{

	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImage cv_to_ros;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	temp = (IplImage)(cv_ptr->image);
	source_image = &temp;
	cvResize(source_image, source_image_resized);
	
	//detect the yellow region
	IplImage *image_threshold = cvCreateImage(cvGetSize(source_image_resized),IPL_DEPTH_8U, 1);

	Color_Detection(source_image_resized, image_threshold, 0, 90, 0.1, 1, 0, 1);	
	//cvCopy(image_threshold,image_threshold_origin);

	cvDilate(image_threshold, image_threshold, myModel, 1);
	cvErode(image_threshold, image_threshold, myModel, 1);


	cvShowImage("Threshold Image", image_threshold);
	waitKey(1);
	cvShowImage("Origin Image", source_image_resized);
	waitKey(1);

	cvReleaseImage(&image_threshold);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_yaw");
	FindYaw Find_yaw;
	ros::spin();
}