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

#include "PointCloud2Mesh.h"
#include "RealSenseUpdater.h"
#include "PCL_Regist.h"

#define NUM 3
#define POWER 3
#define TIME_STANDBY 50


using namespace Intel::RealSense;

class RealSenseProcessor
{
public:
	RealSenseProcessor();
	~RealSenseProcessor();
	bool run(void);
	std::vector<RealSenseUpdater> rsu;
private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	std::vector<Eigen::Matrix4f> transformMat;
	std::vector<PCL_Regist> regist_tip,regist_near;

	void updateViewerText(void);
	void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);
	bool keyboardCallBackSettings(int key);
	inline void initializeViewer(const std::string &id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloudPtr, double pointSize);
	bool setReInit(void);
	Eigen::Matrix4f getPresetTransMat(float radius, float theta);

	bool isContinue;
	bool isUserInterrupt;
	bool isExit = false;
	enum
	{
		CV_WAITKEY_CURSORKEY_TOP = 2490368,
		CV_WAITKEY_CURSORKEY_BOTTOM = 2621440,
		CV_WAITKEY_CURSORKEY_RIGHT = 2555904,
		CV_WAITKEY_CURSORKEY_LEFT = 2424832,
	};

	//ÉNÉâÉXì‡ïœêî
	wchar_t directoryName[20];
	char nallowDirectoryName[20];
	std::vector<std::string> cloud_id;
	std::vector<std::string> tip_cloud_id;
	std::string dataFileName;
	std::ofstream dataFile;
	inline std::string makeNameFolder(int hrgn);
	inline std::string makeNameFail(int hrgn, int num);
	inline void printText(int hrgn, int num);
	inline std::string getTime(void);

	const int numMax = 9; // ï€ë∂Ç∑ÇÈàÍï∂éöÇÃêî

	const std::string dataFolderName = "Data";

	int num = 0; // î‘çÜäiî[óp
	int hrgn = 0; // ï∂éöäiî[óp

	std::string _time;

	Eigen::Matrix4f zRev;
};

