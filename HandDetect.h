#pragma once

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>

#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_core2411d.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_imgproc2411d.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_highgui2411d.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_core2411.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_imgproc2411.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_highgui2411.lib")
#endif

class HandDetect
{
public:
	HandDetect(double nearThreshold, double farThreshold);
	std::vector<cv::Point> getTipData(cv::Mat depth, cv::Mat color);
	cv::Mat colorMarked;
	~HandDetect();
private:
	cv::Mat colorImage, depthImage, depthBinaryImage;
	double _nearThreshold, _farThreshold;
	cv::Mat getBinaryImage(void);
};

