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
	cv::Mat source_image;

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
	
	imshow("Origin Image", source_image);
	waitKey(1);
	aa
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");
	Test test;
	ros::spin();
}