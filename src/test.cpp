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

class Test
{
public: 
	Test();
private:
	ros::NodeHandle n;
	ros::Subscriber image_sub;
	Mat source_image;

	void imageCallback(const sensor_msgs::Image &msg);
};

Test::Test()
{
	image_sub = n.subscribe("/videofile/image_raw", 1, &Test::imageCallback,this);
}

void Test::imageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	source_image = cv_ptr->image;
	resize(source_image,source_image,Size(640,360),0,0,CV_INTER_LINEAR);
	GaussianBlur(source_image,source_image,Size(5,5),0,0);
	imshow("Origin Image", source_image);
	waitKey(1);
	Mat result;
	Canny(source_image, result, 150, 220);
	imshow("Result Image", result);
	waitKey(1);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_test");
	int cornersCount=50;//得到的角点数目
	CvPoint2D32f corners[50];//输出角点集合
	IplImage *image = cvLoadImage("/home/chenjie/catkin_ws/src/image_process/image/calibration3.jpg");
	cvSmooth(image,image,CV_MEDIAN,5,5);
	IplImage *grayImage = cvCreateImage(cvGetSize(image),IPL_DEPTH_8U,1);
	IplImage *corners1 = cvCreateImage(cvGetSize(image),IPL_DEPTH_32F,1);
	IplImage *corners2 = cvCreateImage(cvGetSize(image),IPL_DEPTH_32F,1);
	cvCvtColor(image,grayImage,CV_BGR2GRAY);
	cvThreshold(grayImage,grayImage,120,255,CV_THRESH_BINARY);
	cvShowImage("Threshold",grayImage);

	CvRect rect;
	rect.x = 200;
	rect.y = 50;
	rect.width = 200;
	rect.height = 200;
	IplImage *ROI_image = cvCreateImage(cvSize(200, 200), IPL_DEPTH_8U, 1);
	//get the ROI region
	cvSetImageROI(grayImage,rect);
	cvCopy(grayImage,ROI_image);  
	cvResetImageROI(grayImage); 

	cvCanny(ROI_image,ROI_image,200,240,3);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	lines = cvHoughLines2(ROI_image, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 5, 20, 50);
	float angle[lines->total];
	float sum = 0;
	int count = 0;
	for (int i=0; i<lines->total; i++)  
	{  
		CvPoint *line = (CvPoint *)cvGetSeqElem(lines,i);  
		cvLine(ROI_image,line[0],line[1],CV_RGB(255,255,255),3,CV_AA,0);  
	}


	cvShowImage("image",ROI_image);
	waitKey(0);

	ros::spin();
}