//Size of the image is 640*360
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "image_process.h"
#include "image_process/robot_info.h"
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_autonomy/Navdata.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"
#include "libxl/libxl.h"

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace libxl;

struct test
{
	double h;
	double length;
	
};

class ATTITUDE_CORR
{
public: 
	ATTITUDE_CORR();
private:
	ros::NodeHandle n;
	ros::Subscriber image_sub;
	ros::Subscriber altitude_sub;
	ros::Subscriber navdata_sub;
	ros::Subscriber imu_sub;
	ros::Publisher robot_pub;

	IplImage *source_image;
	IplImage *source_image_resized;
	IplImage temp;
	IplConvKernel * myModel;

	double height;
	double tag_length;
	double image_pos[2];
	double correct_pos[2];
	double actual_pos[2];
	double roll;
	double pitch;
	double yaw;

	std::vector<test> v;
	bool writen;

	void imageCallback(const sensor_msgs::Image &msg);
	void altitudeCallback(const ardrone_autonomy::navdata_altitude &msg);
	void navdataCallback(const ardrone_autonomy::Navdata &msg);
	void imuCallback(const sensor_msgs::Imu &msg);
	void createXLS();
};

ATTITUDE_CORR::ATTITUDE_CORR()
{
	image_sub = n.subscribe("/ardrone/image_raw", 1, &ATTITUDE_CORR::imageCallback,this);
	altitude_sub = n.subscribe("/ardrone/navdata_altitude", 1 , &ATTITUDE_CORR::altitudeCallback, this);
	navdata_sub = n.subscribe("/ardrone/navdata", 1, &ATTITUDE_CORR::navdataCallback, this);
	imu_sub = n.subscribe("ardrone/imu", 1, &ATTITUDE_CORR::imuCallback, this);

	robot_pub = n.advertise<image_process::robot_info>("/ardrone/robot_info", 1);

	source_image_resized = cvCreateImage(cvSize(640,360),IPL_DEPTH_8U, 3);
	myModel = cvCreateStructuringElementEx(3,3,2,2,CV_SHAPE_ELLIPSE);
	tag_length = 0.0;
	writen = false;
	roll = 0.0;
	pitch = 0.0;
	yaw = 0.0;
}

void ATTITUDE_CORR::altitudeCallback(const ardrone_autonomy::navdata_altitude &msg)
{
	height = msg.altitude_vision/1000.0 * cos(roll) * cos(pitch);
	correct_pos[0] = msg.altitude_vision/1000.0 * sin(roll);
	correct_pos[1] = msg.altitude_vision/1000.0 * sin(pitch);
}

void ATTITUDE_CORR::navdataCallback(const ardrone_autonomy::Navdata &msg)
{
	roll = msg.rotX / 180.0 * 3.14; 
	pitch = msg.rotY / 180.0 * 3.14; 
	yaw = msg.rotZ / 180.0 * 3.14; 
}

void ATTITUDE_CORR::imuCallback(const sensor_msgs::Imu &msg)
{
// 	double q0,q1,q2,q3;
// 	q0 = msg.orientation.w;
// 	q1 = msg.orientation.x;
// 	q2 = msg.orientation.y;
// 	q3 = msg.orientation.z;
// 	pitch = atan2(2*(q2*q3-q0*q1), q0*q0-q1*q1-q2*q2+q3*q3);
// 	roll = asin(-2*(q0*q2+q1*q3));
// 	yaw = atan2(2*(q1*q2-q0*q3), q0*q0+q1*q1-q2*q2-q3*q3);
// 	ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll*57.3, pitch*57.3, yaw*57.3);
}

// void ATTITUDE_CORR::imageCallback(const sensor_msgs::Image &msg)
// {
// 	cv_bridge::CvImagePtr cv_ptr;
// 	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
// 	Mat image_input = cv_ptr->image;

// 	Size board_size = Size(7, 6);
// 	vector<Point2f> image_points_buf;
// 	vector<vector<Point2f> > image_points_seq;

// 	cvtColor(image_input, image_input, CV_RGB2GRAY);
// 	bool find = findChessboardCorners(image_input, board_size, image_points_buf);
// 	if (find == false)
// 	{

// 	} else {
// 		find4QuadCornerSubpix(image_input,image_points_buf,Size(11,11));
// 		image_points_seq.push_back(image_points_buf);
// 		drawChessboardCorners(image_input, board_size, image_points_buf, 1);
// 		imshow("camera calibration", image_input);
// 		waitKey(1);
// 	}

// 	if(image_points_seq[0].empty() != true)
// 	{
// 		double length = 0; 
// 		for(int i = 0; i < 6; i++)
// 		{
// 			length += sqrt((image_points_seq[0][6+7*i].x - image_points_seq[0][0+7*i].x) * (image_points_seq[0][6+7*i].x - image_points_seq[0][0+7*i].x)
// 			+ (image_points_seq[0][6+7*i].y - image_points_seq[0][0+7*i].y) * (image_points_seq[0][6+7*i].y - image_points_seq[0][0+7*i].y));
// 		}
		
// 		length = length/6;
// 		ROS_INFO("%f", length);

// 		test temp;
// 		temp.h = height*cos(roll)*cos(pitch);
// 		temp.length = length;

// 		v.push_back(temp);
// 	}

		

// 	if(v.size() > 300 && !writen)
// 	{
// 		createXLS();
// 		writen = true;
// 	}

// }


void ATTITUDE_CORR::imageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImage cv_to_ros;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	temp = (IplImage)(cv_ptr->image);
	source_image = &temp;
	cvResize(source_image, source_image_resized);
	
	IplImage *image_threshold = cvCreateImage(cvGetSize(source_image_resized),IPL_DEPTH_8U, 1);

	Color_Detection(source_image_resized, image_threshold, 324, 360, 0.3, 1, 0, 255);	
	cvErode(image_threshold, image_threshold, myModel, 1);
	cvDilate(image_threshold, image_threshold, myModel, 1);

	cvShowImage("Threshold Image", image_threshold);
	waitKey(1);

	bool find_tag = false;

	//find the contours
	CvMemStorage* storage = cvCreateMemStorage(0);  
	CvSeq* contour = 0;  
	cvFindContours(image_threshold, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cvZero(image_threshold);
	CvSeq* _contour = contour;
	

	for( ; contour != 0; contour = contour->h_next )
	{  
		double tmparea = fabs(cvContourArea(contour));  
	 	if (tmparea < 1000)  
	 	{  
	 		continue; 
	 	}
		CvRect aRect = cvBoundingRect(contour, 0 );
		cvRectangle(source_image_resized, cvPoint(aRect.x, aRect.y), cvPoint(aRect.x + aRect.width, aRect.y + aRect.height),CV_RGB(255,255,0), 3, 8, 0);
		tag_length = (aRect.width + aRect.height) / 2;
		image_pos[0] = aRect.x + aRect.width / 2 - 320;
		image_pos[1] = aRect.y + aRect.height / 2 - 180;


		image_pos[0] = (1.55 * height + 0.01078) / 1000 * image_pos[0];
		image_pos[1] = (1.55 * height + 0.01078) / 1000 * image_pos[1];
		// image_pos[0] = 0.05 / tag_length * image_pos[0];
		// image_pos[1] = 0.05 / tag_length * image_pos[1];

		actual_pos[0] = image_pos[0] - correct_pos[0];
		actual_pos[1] = image_pos[1] + correct_pos[1];

		find_tag = true;
	}

	if(find_tag)
	{
		image_process::robot_info robot_msg;
		robot_msg.whole = true;
		robot_msg.pose.x = actual_pos[0];
 		robot_msg.pose.y = actual_pos[1];
 		robot_pub.publish(robot_msg);
	}else
	{
		image_process::robot_info robot_msg;
		robot_msg.whole = false;
		robot_pub.publish(robot_msg);
	}
	
	cvShowImage("Origin Image", source_image_resized);
	waitKey(1);


	ROS_INFO("actual_pos: x:%f  y:%f", image_pos[0], image_pos[1]);
	ROS_INFO("actual_pos_corr: x:%f  y:%f", actual_pos[0], actual_pos[1]);
	cvReleaseImage(&image_threshold);
	cvReleaseMemStorage(&storage);
}


void ATTITUDE_CORR::createXLS()
{
	Book* book = xlCreateBook();
	book->setKey("chenjie", "linux-2c232e010dcbe60168b76d6da0f5hfga"); 
	if(book)
	{    
		Sheet* sheet = book->addSheet("Sheet1");
		if(sheet)
		{
			for(int i = 0; i < v.size(); i++)
			{
				sheet->writeNum(i, 0, v[i].h);
				sheet->writeNum(i, 1, v[i].length);
			}
		}

		if(book->save("result.xls")) 
		{
			std::cout << "File example.xls has been created." << std::endl;
		}
		book->release();
	}
}
	


int main(int argc, char **argv)
{
	ros::init(argc, argv, "attitude_correct");
	ATTITUDE_CORR Find_robot;
	ros::spin();
}