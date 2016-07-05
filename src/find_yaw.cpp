//Size of the image is 640*360
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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
	ros::Publisher drone_pub;

	IplImage *source_image;
	IplImage *source_image_resized;
	IplImage temp;
	IplConvKernel * myModel;

	void imageCallback(const sensor_msgs::Image &msg);
};

FindYaw::FindYaw()
{
	image_sub = n.subscribe("/videofile/image_raw", 1, &FindYaw::imageCallback,this);
	drone_pub = n.advertise<std_msgs::Float32>("/ardrone/yaw", 1);
	source_image_resized = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
	myModel = cvCreateStructuringElementEx(30,30,2,2,CV_SHAPE_RECT);
}

void FindYaw::imageCallback(const sensor_msgs::Image &msg)
{
	float theta;
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImage cv_to_ros;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	temp = (IplImage)(cv_ptr->image);
	source_image = &temp;
	cvResize(source_image, source_image_resized);
	
	IplImage *image_threshold = cvCreateImage(cvGetSize(source_image_resized),IPL_DEPTH_8U, 1);

	Color_Detection(source_image_resized, image_threshold, 0, 90, 0.1, 1, 0, 255);	
	//cvCopy(image_threshold,image_threshold_origin);
	cvDilate(image_threshold, image_threshold, myModel, 1);
	cvErode(image_threshold, image_threshold, myModel, 1);

	cvCanny(image_threshold,image_threshold,200,240,3);

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	lines = cvHoughLines2(image_threshold, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 20, 120, 30);
	float angle[lines->total];
	float sum = 0;
	int count = 0;
	cvZero(image_threshold);
	for (int i=0; i<lines->total; i++)  
	{  
		CvPoint *line = (CvPoint *)cvGetSeqElem(lines,i);  
		cvLine(image_threshold,line[0],line[1],CV_RGB(255,255,255),3,CV_AA,0);  
		angle[i] = atan2((line[0].y - line[1].y) , (line[0].x - line[1].x));
		if(angle[i] < 0){
			angle[i] = angle[i] + M_PI;
		}
		angle[i] = angle[i] - M_PI/2;
		if((fabs(angle[i]) < M_PI/4) && (fabs(angle[i]) > 0.0001))
		{
			sum = sum + angle[i];
			count++ ;
		}
	}
	if(count > 0){
		std_msgs::Float32 msg;
		theta = sum/count/M_PI*180;
		msg.data = theta;
		drone_pub.publish(msg);
		//ROS_INFO("Angle:%f", theta);
	}	
	// cvShowImage("Origin Image", image_threshold);
	// waitKey(1);

	cvReleaseImage(&image_threshold);
	cvReleaseMemStorage(&storage);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_yaw");
	FindYaw Find_yaw;
	ros::spin();
}