#ifndef   __IMAGE_PROCESS_H__ 
#define   __IMAGE_PROCESS_H__ 

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

float Color_Detection(IplImage* src, IplImage* dst, float h_1, float h_2, float s_1, float s_2, float v_1, float v_2);
float Color_Detection_Reverse(IplImage* src, IplImage* dst, float h_1, float h_2, float s_1, float s_2, float v_1, float v_2);
void Number_Detection(IplImage* src,IplImage* dst);
void edge_extracting(IplImage* src, IplImage* dst);
void edge_direction_extracting(IplImage* src, IplImage* dst, float theta);
float find_center(IplImage* src, double &x, double &y);
bool find_circle_center(IplImage* origin_src, IplImage* src, double &x, double &y, double &area);
bool find_boundary(IplImage* origin_src, IplImage* src, double &x, double &y);

#endif
