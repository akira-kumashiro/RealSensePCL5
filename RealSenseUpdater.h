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
//Debug���[�h�̏ꍇ
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_core2411d.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_imgproc2411d.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_highgui2411d.lib")
#else
//Release���[�h�̏ꍇ
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
		RSU_DEPTH_IMAGE_UNAVAILABLE = -11,
		RSU_USER_INTERRUPTED = -12,
		RSU_MAPPING_UNAVAILABLE = 1
	};

	double fps = 0;
	Status sts;
private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	void ppInit(int num);
	void showStatus(Status sts);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updatePointCloud(bool isHandDataArrived);
	bool acqireImage(PXCImage* cameraFrame, cv::Mat &mat, PXCImage::PixelFormat pixelFormat);
	/*int countMat(cv::Mat mat, cv::Vec4b elm);
	int countMat(cv::Mat mat, unsigned char elm);
	int countMat(cv::Mat mat, float elm);*/
	//bool isOutliers(float rawDepthElem, float rawDepthPrevElem);
	//int detC(cv::Mat src);
	void calcDepthMark();
	void setTipCloud();
	cv::Mat drawMappedImage(void);
	void debugPrint(int line);

	bool isContinue;
	bool isUserInterrupt;
	bool isExit = false;
	int cameraNum;
	double nearThreshold = 0.15;
	double farThreshold = 0.6;

	SenseManager *pp;

	/*enum
	{
		CLOUD_HAND,
		CLOUD_CAMERA,
		CLOUD_JOINT,
		CLOUD_NEAR,
		CLOUD_NUM,
	};*/

	/*enum
	{
		CV_WAITKEY_CURSORKEY_TOP = 2490368,
		CV_WAITKEY_CURSORKEY_BOTTOM = 2621440,
		CV_WAITKEY_CURSORKEY_RIGHT = 2555904,
		CV_WAITKEY_CURSORKEY_LEFT = 2424832,
	};*/

	std::vector<cv::Size> colorSizes = {
		cv::Size(320,180),//0
		cv::Size(320,240),//1
		cv::Size(424,240),//2
		cv::Size(640,360),//3
		cv::Size(848,480),//4
		cv::Size(960,540),//5
		cv::Size(1280,720),//6
		cv::Size(1920,1080)//7//���d������color�����Ƃ�Ȃ��Ȃ����肷��
	};

	std::vector<cv::Size> depthSizes = {
		cv::Size(640,240),//0
		cv::Size(640,480)//1
	};

	cv::Size colorSize;
	static const int COLOR_FPS = 30;

	cv::Size depthSize;
	static const int DEPTH_FPS = 30;


	//�摜�\���̋L�q
	cv::Mat colorImage;
	//cv::Mat depthImage;
	cv::Mat rawDepthImage;
	cv::Mat rawDepthImagePrev;
	cv::Mat depthmarked;
	cv::Mat colorMappedToDepth;

	//�N���X���ϐ�
	wchar_t directoryName[20];
	char nallowDirectoryName[20];
	std::string dataFileName;
	std::ofstream dataFile;
	char windowTitle[20];

	PXCHandConfiguration* config = NULL;
	PXCHandData* handData = NULL;
	PXCProjection *projection = nullptr;

	PXCHandModule *handAnalyzer = NULL;

	//��������dptviewer

	// init()�����s����O�ɐݒ肷��@���F1
	bool enableReadColor = false; // �J���[�摜�̎擾����
	bool enableReadDepth = false; // depth8i�摜�̎擾����
	bool enableHandTracking = false; // �n���h�g���b�L���O�̋���
	bool enableMirror = false; // �~���[�\���̋���

	bool isSaveMirror = false; // �~���[�ł̉摜�ۑ�

							   // ��̈ʒu�̓_�̃T�C�Y
	int handSize = true;

	const std::string extension = "dpt";

	enum
	{
		d1_mirror
	};

	// dpt�f�[�^�̃w�b�_�f�[�^
	typedef struct
	{
		unsigned char size = 0x10;
		unsigned char identifier = 0;
		unsigned short width = 0;
		unsigned short height = 0;
		unsigned short resoHori = 0;
		unsigned short resoVert = 0;
		unsigned char type = 0;
		// | �\�� | �\�� | �\�� | �\�� | �\�� | �\�� | �\�� | �~���[ |
		unsigned char data1 = 0x00;
	}dptHeader;

	// depth32f�摜���t�@�C���ɏ�������
	void writeDepth(const std::string name);
	//cv::Mat drawGuide(const cv::Mat& input, int num);
	//void printText(int hrgn, int num);
	//void shorGuideImage(const cv::Mat depth, int num);
	cv::Mat RealSenseUpdater::readDepth(const std::string name);

	const int numMax = 9; // �ۑ�����ꕶ���̐�
	const int distMin = 375; // ��O�̋���
	const int distMax = 425; // ���̋���
	const int sizeLine = 20; // ���̑���

	int num = 0; // �ԍ��i�[�p
	int hrgn = 0; // �����i�[�p

	//std::string _time;

	int morph_elem = 0;
	int morph_size = 1;
	int const max_elem = 2;

	PointCloud2Mesh::gpParameters param;
	
	std::chrono::system_clock::time_point nowTime, prevTime;
};

