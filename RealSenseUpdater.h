#pragma once

#include <windows.h>
#include "RealSense/SenseManager.h"
#include "RealSense/SampleReader.h"
#include <conio.h>
#include <opencv2\opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv/highgui.h>
#include <opencv\cv.h>
#include <direct.h>
#include "pxchandmodule.h"
#include "pxcsensemanager.h"
#include "pxchandconfiguration.h"
#include "pxcprojection.h"
#include <opencv2\highgui\highgui.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/fast_bilateral.h>

#include "wcolorIO.h"
#include "HandDetect.h"

#include "PointCloud2Mesh.h"


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

#define _USE_MATH_DEFINES
#include <math.h>

#define SHOW_DEPTH_ROW 7
#define AVERAGE_FRAME_NUM 10
#define CLOUD_SCALE 1000
#define CLOUD_PITCH 1
#define DIFF_EXCLUDE_THRESHOLD 20
#define GAUSS_EXCLUDE_THRESHOLD 10
#define CONTOUR_SIZE_THRESHOLD 10
//#define __DEBUG_MODE__

using namespace Intel::RealSense;

class RealSenseUpdater
{
	//private:
public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_point_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_point_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_joint_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr near_point_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tip_point_cloud_ptr;

	RealSenseUpdater();
	~RealSenseUpdater();
	int init(int num);
	int run(void);
	void setCamera(int numCam);
	Status setLaserPower(int num);
	bool saveData(std::string directory, std::string name);
	void setEnableHandTracking(bool _enableHandTracking);
	void changeThreshold(bool isIncr);
	void showFPS();

	enum
	{
		RSU_NO_ERROR = 0,
		RSU_DEVICE_REMOVED = -2,
		RSU_ERROR_OCCURED = -1,
		RSU_COLOR_IMAGE_UNAVAILABLE = -3,
		RSU_DEPTH_IMAGE_UNAVAILABLE = -4,
		RSU_USER_INTERRUPTED = -5
	};

private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	Status ppInit(int num);
	void showStatus(Status sts);
	void realsenseHandStatus(PXCHandData *handAnalyzer);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updatePointCloud(bool isHandDataArrived);
	bool acqireImage(PXCImage* cameraFrame, cv::Mat &mat, PXCImage::PixelFormat pixelFormat);
	int countMat(cv::Mat mat, cv::Vec4b elm);
	int countMat(cv::Mat mat, unsigned char elm);
	int countMat(cv::Mat mat, float elm);
		bool isOutliers(float rawDepthElem, float rawDepthPrevElem);
	int detC(cv::Mat src);
	void calcDepthMark();
	void setTipCloud();

	Status sts;
	bool isContinue;
	bool isUserInterrupt;
	bool isExit = false;
	int cameraNum;
	double nearThreshold = 0.15;
	double farThreshold = 0.6;

	SenseManager *pp;

	enum
	{
		CLOUD_HAND,
		CLOUD_CAMERA,
		CLOUD_JOINT,
		CLOUD_NEAR,
		CLOUD_NUM,
	};

	enum
	{
		CV_WAITKEY_CURSORKEY_TOP = 2490368,
		CV_WAITKEY_CURSORKEY_BOTTOM = 2621440,
		CV_WAITKEY_CURSORKEY_RIGHT = 2555904,
		CV_WAITKEY_CURSORKEY_LEFT = 2424832,
	};



	//bool isCloudArrived[CLOUD_NUM];

	static const int COLOR_WIDTH = 1920;
	static const int COLOR_HEIGHT = 1080;
	static const int COLOR_FPS = 30;

	static const int DEPTH_WIDTH = 640;
	static const int DEPTH_HEIGHT = 480;
	static const int DEPTH_FPS = 30;


	//画像表示の記述
	cv::Mat colorImage;
	//cv::Mat depthImage;
	cv::Mat rawDepthImage;
	cv::Mat rawDepthImagePrev;
	cv::Mat depthmarked;
	cv::Mat colorMappedToDepth;

	pxcI32 numberOfHands;

	//std::vector<unsigned short> depthBuffer;
	//const std::string windowName[1] = { "handimage" };

	/*const int COLOR_WIDTH = 640;
	const int COLOR_HEIGHT = 480;
	const int COLOR_FPS = 30;

	const int DEPTH_WIDTH = 640;
	const int DEPTH_HEIGHT = 480;
	const int DEPTH_FPS = 30;*/

	//クラス内変数
	/*int colorImageNum = 0;
	int depthImageNum = 0;
	int imageNum = 0;
	int dataNum = 0;*/
	//float bilateralS = 5;
	//float bilateralR = 0.05;
	//bool enableBilateral = false;

	wchar_t directoryName[20];
	char nallowDirectoryName[20];
	std::string dataFileName;
	std::ofstream dataFile;
	char windowTitle[20];

	//double sigmaG = 1.0;
	//int gSize = 1;

	//int pointCloudNum[CLOUD_NUM];

	PXCHandConfiguration* config = NULL;
	PXCHandData* handData = NULL;
	PXCProjection *projection = nullptr;

	PXCHandModule *handAnalyzer = NULL;

	//ここからdptviewer

	// init()を実行する前に設定する　許可：1
	bool enableReadColor = false; // カラー画像の取得許可
	bool enableReadDepth = false; // depth8i画像の取得許可
	bool enableHandTracking = false; // ハンドトラッキングの許可
	bool enableMirror = false; // ミラー表示の許可

	bool isSaveMirror = false; // ミラーでの画像保存

							   // 手の位置の点のサイズ
	int handSize = true;

	const std::string extension = "dpt";

	enum
	{
		d1_mirror
	};

	// dptデータのヘッダデータ
	typedef struct
	{
		unsigned char size = 0x10;
		unsigned char identifier = 0;
		unsigned short width = 0;
		unsigned short height = 0;
		unsigned short resoHori = 0;
		unsigned short resoVert = 0;
		unsigned char type = 0;
		// | 予約 | 予約 | 予約 | 予約 | 予約 | 予約 | 予約 | ミラー |
		unsigned char data1 = 0x00;
	}dptHeader;

	// depth32f画像をファイルに書き込む
	void writeDepth(const std::string name);
	//cv::Mat drawGuide(const cv::Mat& input, int num);
	//void printText(int hrgn, int num);
	//void shorGuideImage(const cv::Mat depth, int num);
	cv::Mat RealSenseUpdater::readDepth(const std::string name);

	const int numMax = 9; // 保存する一文字の数
	const int distMin = 375; // 手前の距離
	const int distMax = 425; // 奥の距離
	const int sizeLine = 20; // 線の太さ

	int num = 0; // 番号格納用
	int hrgn = 0; // 文字格納用

	//std::string _time;

	int morph_elem = 0;
	int morph_size = 1;
	int const max_elem = 2;

	//int cloudAlphaCh = 0;

	PointCloud2Mesh::gpParameters param;

	double fps = 0;

	std::chrono::system_clock::time_point nowTime, prevTime;
};

