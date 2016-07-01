//Size of the image is 640*360
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ardrone_autonomy/navdata_altitude.h"
#include "image_process/drone_info.h"
#include "image_process.h"
#include "Eigen/Dense"

using namespace cv;
using namespace std;
using namespace Eigen;

class FindPosition
{
public: 
	FindPosition();
private:
	ros::NodeHandle n;
	ros::Subscriber image_sub;
	// ros::Subscriber altitude_sub;
	// ros::Subscriber yaw_sub;
	ros::Publisher drone_pub;

	IplImage *source_image;
	IplImage *source_image_resized;
	IplImage temp;
	IplConvKernel * myModel;
	float distance;
	float yaw;
	Matrix<float, 3, 3> R_body;
	Vector3f pos;
//	Vector3f pos_field;

	void imageCallback(const sensor_msgs::Image &msg);
	// void altitudeCallback(const ardrone_autonomy::navdata_altitude &msg);
	// void yawCallback(const std_msgs::Float32 &msg);
	// void get_R_body(float yaw);
};

FindPosition::FindPosition()
{
	image_sub = n.subscribe("/ardrone/image_raw", 1, &FindPosition::imageCallback,this);
	// altitude_sub = n.subscribe("/ardrone/navdata_altitude", 1, &FindPosition::altitudeCallback,this);
	// yaw_sub = n.subscribe("/ardrone/yaw", 1, &FindPosition::yawCallback,this);
	drone_pub = n.advertise<image_process::drone_info>("/ardrone/position_reset_info", 1);
	source_image_resized = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
	myModel = cvCreateStructuringElementEx(10,10,2,2,CV_SHAPE_ELLIPSE);
}

// void FindPosition::altitudeCallback(const ardrone_autonomy::navdata_altitude &msg)
// {
// 	distance = msg.altitude_vision/1000.0;
// }

// void FindPosition::yawCallback(const std_msgs::Float32 &msg)
// {
// 	yaw = msg.data;
// 	get_R_body(yaw/57.3);
// }

// void FindPosition::get_R_body(float yaw)
// {
// 	R_body(0,0) = cos(yaw);
// 	R_body(0,1) = sin(-yaw);
// 	R_body(0,2) = 0;
// 	R_body(1,0) = sin(yaw);
// 	R_body(1,1) = cos(yaw);
// 	R_body(1,2) = 0;
// 	R_body(2,0) = 0;
// 	R_body(2,1) = 0;
// 	R_body(2,2) = 1;
// }

void FindPosition::imageCallback(const sensor_msgs::Image &msg)
{
	bool is_origin = false;
	double pixel_x,pixel_y,area;


	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImage cv_to_ros;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	temp = (IplImage)(cv_ptr->image);
	source_image = &temp;
	cvResize(source_image, source_image_resized);
	
	IplImage *image_threshold = cvCreateImage(cvGetSize(source_image_resized),IPL_DEPTH_8U, 1);

	Color_Detection(source_image_resized, image_threshold, 215, 255, 1, 1, 55, 255);	
	cvErode(image_threshold, image_threshold, myModel, 1);
	//cvDilate(image_threshold, image_threshold, myModel, 1);
	
	cvShowImage("Threshold Image", image_threshold);
	is_origin = find_circle_center(source_image_resized, image_threshold, pixel_x, pixel_y, area);

	cvShowImage("Origin Image", source_image_resized);
	waitKey(1);
	//ratio = pixel/real
	//ratio = -0.7196*d + 1.4297

	if(is_origin && (area > 5000))
	{
		image_process::drone_info msg;
		// float ratio = -0.7196 * distance + 1.4297;
		// pos(0) = (pixel_y - 180) / ratio;
		// pos(1) = (pixel_x - 320) / ratio;
		// pos_field = R_body * pos;
		msg.pose.x = pixel_x;
		msg.pose.y = pixel_y;
		// msg.pose.theta = yaw;
		drone_pub.publish(msg);
		ROS_INFO("x:%f, y:%f",msg.pose.x,msg.pose.y);
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_position");
	FindPosition find_position;
	ros::spin();
}