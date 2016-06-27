#include "image_process.h"

using namespace cv;
using namespace std;

float Color_Detection(IplImage* src, IplImage* dst, float h_1, float h_2, float s_1, float s_2, float v_1, float v_2)
{
	int sum = 0;
	float percent = 0;
	cvSmooth(src,src,CV_MEDIAN,5,5);
	IplImage *src_float = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F, 3);
	cvConvertScale(src, src_float, 1.0, 0.0);
	IplImage *hsv_img = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F , 3);
	cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
	int step = hsv_img->widthStep/sizeof(float);
	int channels = hsv_img->nChannels;
	float * datafloat = (float *)hsv_img->imageData;
	for(int i = 0; i < hsv_img->height; i++)
	{
		for(int j = 0; j < hsv_img->width; j++)
		{
			//H:0-360   S:0-1   V:0-255
			if((datafloat[i*step + j*channels + 2] >= v_1 && datafloat[i*step + j*channels + 2] <= v_2) && (datafloat[i*step + j*channels + 1] >= s_1 && datafloat[i*step + j*channels + 1] <= s_2) && (datafloat[i*step + j*channels] >= h_1 && datafloat[i*step + j*channels] <= h_2))
			//if((datafloat[i*step + j*channels + 1] > s_1 && datafloat[i*step + j*channels + 1] < s_2) && (datafloat[i*step + j*channels] > h_1 && datafloat[i*step + j*channels] < h_2))
			{
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=255;
				sum++;
			}
			else
			{
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=0;	
			}
		}
	}
	percent=1.0*sum/(hsv_img->height*hsv_img->width);
	cvReleaseImage(&hsv_img);
	cvReleaseImage(&src_float);
	return percent;
}

float Color_Detection_Reverse(IplImage* src, IplImage* dst, float h_1, float h_2, float s_1, float s_2, float v_1, float v_2)
{
	int sum = 0;
	float percent = 0;
	cvSmooth(src,src,CV_MEDIAN,5,5);
	IplImage *src_float = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F, 3);
	cvConvertScale(src, src_float, 1.0, 0.0);
	IplImage *hsv_img = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F , 3);
	cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
	int step = hsv_img->widthStep/sizeof(float);
	int channels = hsv_img->nChannels;
	float * datafloat = (float *)hsv_img->imageData;
	for(int i = 0; i < hsv_img->height; i++)
	{
		for(int j = 0; j < hsv_img->width; j++)
		{
			if((datafloat[i*step + j*channels] >= h_1 && datafloat[i*step + j*channels] <= h_2))
			{
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=0;	
			}
			else if((datafloat[i*step + j*channels + 1] >= s_1 && datafloat[i*step + j*channels + 1] <= s_2) && (datafloat[i*step + j*channels + 2] >= v_1 && datafloat[i*step + j*channels + 2] <= v_2))
			{
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=255;	
				sum++;
			}else
			{
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=0;	
			}
		}
	}
	percent=1.0*sum/(hsv_img->height*hsv_img->width);
	cvReleaseImage(&hsv_img);
	cvReleaseImage(&src_float);
	return percent;
}

void Number_Detection(IplImage* src,IplImage* dst)
{
	cvSmooth(src,src,CV_MEDIAN,5,5);
	IplImage *src_float = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F, 3);
	cvConvertScale(src, src_float, 1.0, 0.0);
	IplImage *hsv_img = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F , 3);
	cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
	int step = hsv_img->widthStep/sizeof(float);
	int channels = hsv_img->nChannels;
	float * datafloat = (float *)hsv_img->imageData;
	for(int i = 0; i < hsv_img->height; i++)
	{
		for(int j = 0; j < hsv_img->width; j++)
		{
			//if(datafloat[i*step + j*channels + 2]>10&&(datafloat[i*step + j*channels + 1]>0.1)&&(datafloat[i*step + j*channels]>130&&datafloat[i*step + j*channels]<310))
			if((datafloat[i*step + j*channels]>10&&datafloat[i*step + j*channels]<90))
			{
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=255;
			}
			else
			{
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=0;
			}
		}
	}

	cvReleaseImage(&hsv_img);
	cvReleaseImage(&src_float);
}

//Input: 8比特、单通道(二值)图像
void edge_extracting(IplImage* src, IplImage* dst)
{
	cvCanny(src,dst,200,240,3);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 20, 100, 30);
	for (int i=0;i<lines->total;i++)  
	{  
		CvPoint *line = (CvPoint *)cvGetSeqElem(lines,i);  
		cvLine(dst,line[0],line[1],CV_RGB(255,255,255),3,CV_AA,0);  
	}  

	cvShowImage("line Detector",dst);
	cvReleaseMemStorage(&storage);
}

void edge_direction_extracting(IplImage* src, IplImage* dst, float theta)
{
	cvCanny(src,dst,200,240,3);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 20, 100, 30);
	float angle[lines->total];
	float sum = 0;
	int count = 0;
	for (int i=0; i<lines->total; i++)  
	{  
		CvPoint *line = (CvPoint *)cvGetSeqElem(lines,i);  
		angle[i] = atan2((line[0].y - line[1].y) , (line[0].x - line[1].x));
		if(angle[i] < 0){
			angle[i] = angle[i] + M_PI;
		}
		angle[i] = angle[i] - M_PI/2;
		if((fabs(angle[i]) < M_PI/4) && (fabs(angle[i]) > 0))
		{
			sum = sum + angle[i];
			count++ ;
		}
	}
	if(count > 0){
		theta = sum/count;
	}
	cvReleaseMemStorage(&storage);
}

//Input: 8比特、单通道(二值)图像
float find_center(IplImage* src, double &x, double &y)
{
	int sum=0,xsum=0,ysum=0;
	float percent=0;
	unsigned char* data=(unsigned char *)src->imageData;  
	int step = src->widthStep/sizeof(unsigned char);  

	for(int i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			if(data[i*step + j] > 100)
			{
				sum++;
				xsum+=(j+1);
				ysum+=(i+1);
			}
		}
	}
	if(sum > 0)
	{
		x = xsum/sum;
		y = ysum/sum;
		percent = 1.0 * sum / (src->height * src->width);
	}
	
	return percent;
}

//return the area of the max circle
bool find_circle_center(IplImage* origin_src, IplImage* src, double &x, double &y, double &area)
{
	//find the white contours
	CvMemStorage* storage = cvCreateMemStorage(0);  
	CvSeq* contour = 0;  
	double maxarea = 0;  
	int count = 0; 
	bool is_circle = false;

	cvFindContours(src, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	//保存轮廓的首指针位置，随后contour将用来迭代
	CvSeq* _contour = contour; 

	for( ; contour != 0; contour = contour->h_next )
	{  
		double tmparea = fabs(cvContourArea(contour));  
        if (tmparea > maxarea)  
        {  
            maxarea = tmparea;  
            continue;  
        } 	
	}
	contour = _contour;
	for( ; contour != 0; contour = contour->h_next )
	{  
		double tmparea = fabs(cvContourArea(contour));
        if (tmparea == maxarea)  
        {  
            CvRect aRect = cvBoundingRect(contour, 0 );
            double width = aRect.width;
            double height = aRect.height;
            if(width/height > 0.9 && height/width > 0.9)
            {
            	x = (aRect.x + aRect.x + aRect.width) / 2;
				y = (aRect.y + aRect.y + aRect.height) / 2;
				area = maxarea;
				is_circle = true;
				cvRectangle(origin_src, cvPoint(aRect.x, aRect.y), cvPoint(aRect.x + aRect.width, aRect.y + aRect.height),CV_RGB(0,255,0), 3, 8, 0);
            }
        }	
	}

	cvReleaseMemStorage(&storage);
	if(is_circle){
		return true;
	} 
	else {
		return false;
	}
}
