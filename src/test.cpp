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
	IplImage *image = cvLoadImage("/home/chenjie/Desktop/1.png");
	IplImage *src_float = cvCreateImage(cvGetSize(image),IPL_DEPTH_32F, 3);
	cvConvertScale(image, src_float, 1.0, 0.0);
	IplImage *hsv_img = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F , 3);
	cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
	int step = hsv_img->widthStep/sizeof(float);
	int channels = hsv_img->nChannels;
	float * datafloat = (float *)hsv_img->imageData;
	float max_V = 0;
	float max_H = 0;
	float max_S = 0;
	for(int i = 0; i < hsv_img->height; i++)
	{
		for(int j = 0; j < hsv_img->width; j++)
		{
			printf("H:%f\n", datafloat[i*step + j*channels]);
			printf("S:%f\n", datafloat[i*step + j*channels + 1]);
			printf("V:%f\n", datafloat[i*step + j*channels + 2]);
			if(datafloat[i*step + j*channels + 2]>max_V) max_V=datafloat[i*step + j*channels + 2];
			if(datafloat[i*step + j*channels + 1]>max_S) max_S=datafloat[i*step + j*channels + 1];
			if(datafloat[i*step + j*channels]>max_H) max_H=datafloat[i*step + j*channels];
		}
	}
	printf("H:%f\n", max_H);
	printf("S:%f\n", max_S);
	printf("V:%f\n", max_V);
	cvReleaseImage(&hsv_img);
	cvReleaseImage(&src_float);
	ros::spin();
}