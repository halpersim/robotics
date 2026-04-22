#ifndef _THREAD_TEST_H
#define _THREAD_TEST_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

struct res_xy{
	int x; 
	int y; 
}; 

res_xy camera_func(const cv::Mat& frame);
void* cameraThread(void* arg);

#endif