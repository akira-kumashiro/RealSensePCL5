#include "RealSenseUpdater.h"

RealSenseUpdater::RealSenseUpdater() :
	hand_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	camera_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	hand_joint_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	near_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	tip_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>)//PCL�֘A�̕ϐ��̏�����
{

	wColorIO(wColorIO::PRINT_INFO, L"RSU>");
	wColorIO(wColorIO::PRINT_INFO, L"Start\n");

	pp = SenseManager::CreateInstance();
	if (!pp)
	{
		wprintf_s(L"Unable to create the SenseManager\n");
	}

	colorSize = colorSizes[6];
	depthSize = depthSizes[1];

	rawDepthImage = cv::Mat::zeros(depthSize, CV_32FC1);
	depthmarked = cv::Mat::zeros(depthSize, CV_8UC3);

	isContinue = false;
	isUserInterrupt = false;
	nowTime = std::chrono::system_clock::now();
}


RealSenseUpdater::~RealSenseUpdater()
{
	// Releases lock so pipeline can process next frame 
	pp->ReleaseFrame();

	cv::destroyAllWindows();

	/*if (projection == nullptr)
	{
		projection->Release();
		projection = nullptr;
	}*/

	wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
	wColorIO(wColorIO::PRINT_SUCCESS, L"Exiting\n");
}

int RealSenseUpdater::init(int num)
{
	cameraNum = num;

	setCamera(num);

	if (sts < Status::STATUS_NO_ERROR)
	{
		showStatus(sts);
		return RSU_ERROR_OCCURED;
	}

	ppInit(num);

	if (sts < Status::STATUS_NO_ERROR)
	{
		showStatus(sts);
		return RSU_ERROR_OCCURED;
	}

	return RSU_NO_ERROR;
}

int RealSenseUpdater::run(void)
{
	bool isMappingSucceed = true;

	prevTime = nowTime;
	nowTime = std::chrono::system_clock::now();

	auto def = nowTime - prevTime;

	fps = 1000 / std::chrono::duration<double, std::milli>(def).count();

	if (sts < Status::PXC_STATUS_NO_ERROR)
	{
		showStatus(sts);
		return RSU_ERROR_OCCURED;
	}

	//Waits until new frame is available and locks it for application processing
	sts = pp->AcquireFrame(false);//������true�ɂ���Ƃ��܂ɂ߂����᎞�Ԃ�������

	if (!pp->IsConnected())
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"device removed\n");
		pp->Close();
		isContinue = true;
		//break;
		return RSU_DEVICE_REMOVED;
	}
#ifdef __DEBUG_MODE__
	else
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_SUCCESS, L"Device checked.\n");
	}
#endif

	if (sts < Status::STATUS_NO_ERROR)
	{
		showStatus(sts);
		return RSU_ERROR_OCCURED;
	}

	const PXCCapture::Sample *sample = pp->QuerySample();
	if (sample != nullptr)
	{
		bool errorFlag = false;
		if (sample->color)
		{
			if (acqireImage(sample->color, colorImage, PXCImage::PixelFormat::PIXEL_FORMAT_RGB32))
			{
#ifdef __DEBUG_MODE__
				wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
				wColorIO(wColorIO::PRINT_SUCCESS, L"colorImage catched\n");
#endif
			}
			else
				errorFlag = true;
		}
		else
			errorFlag = true;

		if (errorFlag)
			return RSU_COLOR_IMAGE_UNAVAILABLE;
		else
			errorFlag = false;

		if (sample->depth)
		{
			if (acqireImage(sample->depth, rawDepthImage, PXCImage::PixelFormat::PIXEL_FORMAT_DEPTH_F32))
			{
#ifdef __DEBUG_MODE__
				wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
				wColorIO(wColorIO::PRINT_SUCCESS, L"depthImage catched\n");
#endif
			}
			else
				errorFlag = true;

		}
		else
			errorFlag = true;

		if (errorFlag)
			return RSU_COLOR_IMAGE_UNAVAILABLE;
		else
			errorFlag = false;


		/*if (sample->color && !acqireImage(sample->color, colorImage, PXCImage::PIXEL_FORMAT_RGB32))//updateCameraImage(sample->color, false)
		{
			return RSU_COLOR_IMAGE_UNAVAILABLE;
		}
#ifdef __DEBUG_MODE__
		else
		{
			wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
			wColorIO(wColorIO::PRINT_SUCCESS, L"colorImage catched\n");
		}
#endif
		if (sample->depth && !acqireImage(sample->depth, rawDepthImage, PXCImage::PIXEL_FORMAT_DEPTH_F32))//updateCameraImage(sample->depth, true)
		{
			return RSU_DEPTH_IMAGE_UNAVAILABLE;
		}
#ifdef __DEBUG_MODE__
		else
		{
			wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
			wColorIO(wColorIO::PRINT_SUCCESS, L"depthImage catched\n");
		}
#endif*/
		projection = pp->QueryCaptureManager()->QueryDevice()->CreateProjection();
		if (projection != nullptr)
		{
			if (sample->depth && sample->color)
			{
				PXCImage* projectionImage = projection->CreateColorImageMappedToDepth(sample->depth, sample->color);
				if (projectionImage != nullptr)
				{
					acqireImage(projectionImage, colorMappedToDepth, PXCImage::PixelFormat::PIXEL_FORMAT_RGB24);
					projectionImage->Release();
				}
				else
					errorFlag = true;
			}
			else
				errorFlag = true;

			if (errorFlag)
			{
				//wprintf_s(L"-7\n");
				isMappingSucceed = false;
				colorMappedToDepth = drawMappedImage().clone();
			}
			projection->Release();
		}

		camera_point_cloud_ptr = updatePointCloud(false);
		calcDepthMark();
		debugPrint(__LINE__);
		setTipCloud();
		debugPrint(__LINE__);

		cv::imshow("img(" + std::to_string(cameraNum) + ")", colorMappedToDepth);

		//imshow("img(" + std::to_string(cameraNum) + ") color", colorImage);

		rawDepthImagePrev = rawDepthImage.clone();
	}

	// Releases lock so pipeline can process next frame 
#ifdef __DEBUG_MODE__
	wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
	wColorIO(wColorIO::PRINT_SUCCESS, L"Pipeline release in progress\n");
#endif
	pp->ReleaseFrame();
	/*else
	{
		pp->Release();
		pp = nullptr;
	}*/

	/*if (projection == nullptr)
	{
		projection->Release();
		projection = nullptr;
	}*/
#ifdef __DEBUG_MODE__
	wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
	wColorIO(wColorIO::PRINT_SUCCESS, L"Pipeline release successful\n");
#endif
	// �E�B���h�E��������Ă��ĕ\������
	//cv::namedWindow("�J����");
	//cv::namedWindow("�ۑ��ς�");

	//shorGuideImage(rawDepthImage, num);

	int c = cv::waitKey(1);
	if (c == 27 || c == 'q' || c == 'Q')
	{
		isUserInterrupt = true;
		return RSU_USER_INTERRUPTED;
	}
	else if (c != -1)
	{
		return c;
	}

	/*if (_kbhit())
	{ // Break loop
		c = _getch() & 255;
		if (c == 27 || c == 'q' || c == 'Q')
		{
			isUserInterrupt = true;
			//c = 0;
			return RSU_USER_INTERRUPTED;
		} // ESC|q|Q for Exit
	}*/
	//	}
	isExit = true;

	return isMappingSucceed ? RSU_NO_ERROR : RSU_MAPPING_UNAVAILABLE;
	//}

}

void RealSenseUpdater::ppInit(int num)
{
	//isContinue = false;
	isExit = false;
	isUserInterrupt = false;

	// �X�g���[����L���ɂ���
	sts = pp->EnableStream(PXCCapture::StreamType::STREAM_TYPE_COLOR, colorSize.width, colorSize.height, COLOR_FPS);

	if (sts < Status::STATUS_NO_ERROR)
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"Create Color-Image pipeline has been unsuccessful.\n");
		return;// sts;
	}
	else
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_SUCCESS, L"Create Color-Image pipeline has been successful.\n");
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_INFO, L"Width:");
		wColorIO(wColorIO::PRINT_VALUE, L"%dpx ", colorSize.width);
		wColorIO(wColorIO::PRINT_INFO, L"Height:");
		wColorIO(wColorIO::PRINT_VALUE, L"%dpx ", colorSize.height);
		wColorIO(wColorIO::PRINT_INFO, L"FPS:");
		wColorIO(wColorIO::PRINT_VALUE, L"%d\n", COLOR_FPS);
	}

	sts = pp->EnableStream(PXCCapture::StreamType::STREAM_TYPE_DEPTH, depthSize.width, depthSize.height, DEPTH_FPS);
	if (sts < Status::STATUS_NO_ERROR)
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"Create Depth-Image pipeline has been unsuccessful.\n");
		return;// sts;
	}
	else
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_SUCCESS, L"Create Depth-Image pipeline has been successful.\n");
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_INFO, L"Width:");
		wColorIO(wColorIO::PRINT_VALUE, L"%dpx ", depthSize.width);
		wColorIO(wColorIO::PRINT_INFO, L"Height:");
		wColorIO(wColorIO::PRINT_VALUE, L"%dpx ", depthSize.height);
		wColorIO(wColorIO::PRINT_INFO, L"FPS:");
		wColorIO(wColorIO::PRINT_VALUE, L"%d\n", DEPTH_FPS);
	}

	// �p�C�v���C��������������
	sts = pp->Init();

	if (sts >= PXC_STATUS_NO_ERROR)
	{
		PXCCapture::Device *device = pp->QueryCaptureManager()->QueryDevice();
		PXCCapture::DeviceInfo dinfo;

		if (device != NULL)// �~���[�\���ɂ���
		{
			device->SetMirrorMode(Capture::Device::MirrorMode::MIRROR_MODE_HORIZONTAL);

			wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
			wColorIO(wColorIO::PRINT_INFO, L"MirrorMode has been enabled.\n");
		}

		//projection�����O��setCamera���Ȃ���projection�����������Ȃ�
		setCamera(num);

		//projection = device->CreateProjection();

		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_SUCCESS, L"Create pipeline has been successful.\n");
	}
	else
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"Pipeline initializing has been failed.\n");
	}
}

bool RealSenseUpdater::acqireImage(PXCImage* cameraFrame, cv::Mat &mat, PXCImage::PixelFormat pixelFormat)
{
	PXCImage::ImageData data;

	if (cameraFrame == nullptr)
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"cameraFrame throws nullptr.\n");
		return false;
	}

	sts = cameraFrame->AcquireAccess(PXCImage::Access::ACCESS_READ, pixelFormat, &data);

	if (sts < PXC_STATUS_NO_ERROR)
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"Cannot reach the cameraFrame\n");
		return false;
	}

	// �f�[�^���R�s�[����
	PXCImage::ImageInfo info = cameraFrame->QueryInfo();
	int cvPixelFormat;

	switch (pixelFormat)
	{
	case PXCImage::PIXEL_FORMAT_RGB32:
		cvPixelFormat = CV_8UC4; break;
	case PXCImage::PIXEL_FORMAT_DEPTH_F32:
		cvPixelFormat = CV_32FC1; break;
	case PXCImage::PIXEL_FORMAT_RGB24:
		cvPixelFormat = CV_8UC3; break;
	case PXCImage::PIXEL_FORMAT_DEPTH:
		cvPixelFormat = CV_8U; break;
	default:
		return false;
	}
	mat = cv::Mat::zeros(info.height, info.width, cvPixelFormat);
	memcpy(mat.data, data.planes[0], data.pitches[0] * info.height);

	// �f�[�^���������
	cameraFrame->ReleaseAccess(&data);
	return true;
}

/*bool RealSenseUpdater::isOutliers(float rawDepthElem, float rawDepthPrevElem)
{
	if (abs(rawDepthElem - rawDepthPrevElem) > DIFF_EXCLUDE_THRESHOLD)
	{
		return true;
	}
	else
	{
		return false;
	}
}*/

/*int RealSenseUpdater::detC(cv::Mat img)
{
	cv::Mat src;
	cv::Mat element = cv::getStructuringElement(morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));
	if (!(src = img).data)
		return -1;

	cv::Mat dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

	cv::morphologyEx(src, dst, CV_MOP_TOPHAT, element);

	rawDepthDiffImageFilterd = dst.clone();
	return 0;
}*/

void RealSenseUpdater::calcDepthMark()
{
	//rawDepthImage.convertTo(depthmarked, CV_8UC3, 1.0);// 0x60 / 0x100
	for (int y = 0; y < rawDepthImage.rows; y++)
	{
		float *rawDepthImagePtr = rawDepthImage.ptr<float>(y);
		cv::Vec3b *depthmarkedPtr = depthmarked.ptr<cv::Vec3b>(y);

		for (int x = 0; x < rawDepthImage.cols; x++)
		{
			if (rawDepthImagePtr[x] == 0.0)
				depthmarkedPtr[x] = cv::Vec3b(0, 0, 0);
			else if (rawDepthImagePtr[x] < farThreshold * CLOUD_SCALE && rawDepthImagePtr[x] > nearThreshold * CLOUD_SCALE)
				depthmarkedPtr[x] = cv::Vec3b(0, 0, 255);
			else
				depthmarkedPtr[x] = cv::Vec3b(rawDepthImagePtr[x] * 0x60 / 0x100);
			//else
				//depthmarkedPtr[x] = cv::Vec3b(255, 255, 255);

		}
	}

	cv::Mat dst_img, work_img;
	dst_img = depthmarked.clone();
	cv::cvtColor(depthmarked, work_img, CV_BGR2GRAY);

	// Hough�ϊ��̂��߂̑O�����i�摜�̕��������s�Ȃ�Ȃ��ƌ댟�o���������₷���j
	cv::GaussianBlur(work_img, work_img, cv::Size(11, 11), 2, 2);

	// Hough�ϊ��ɂ��~�̌��o�ƌ��o�����~�̕`��
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(work_img, circles, CV_HOUGH_GRADIENT, 1, 25, 100, 12, 10, 25);

	std::vector<cv::Vec3f>::iterator it = circles.begin();
	for (; it != circles.end(); ++it) {
		cv::Point center(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));
		if (work_img.at<uchar>(center) == 0)
			continue;
		int radius = cv::saturate_cast<int>((*it)[2]);
		cv::circle(depthmarked, center, radius, cv::Scalar(255, 255, 255), 2);
	}
}

void RealSenseUpdater::setTipCloud()
{
	HandDetect det(nearThreshold * CLOUD_SCALE, farThreshold * CLOUD_SCALE);
	cv::Mat colorMappedToDepthTemp = colorMappedToDepth.clone();
	cv::Mat tipPosMat = cv::Mat::zeros(rawDepthImage.size(), CV_8U);

	std::vector<cv::Point> tipPos = det.getTipData(rawDepthImage.clone(), colorMappedToDepth.clone());
	colorMappedToDepth = det.colorMarked.clone();
	cv::Mat handMask = det.contourMask.clone();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tip_cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cv::imshow("colorMapped(" + std::to_string(cameraNum) + ")", det.colorMarked);
	if (tipPos[0] == cv::Point(-1, -1))
	{
		tip_point_cloud_ptr = tip_cloud_temp;
		hand_point_cloud_ptr = hand_cloud_temp;
		return;
	}

	for (int i = 0; i < tipPos.size(); i++)
	{
		uchar *tipPosMatPtr = tipPosMat.ptr<uchar>(tipPos[i].y);
		tipPosMatPtr[tipPos[i].x] = 255;
	}

	for (int y = 0; y < rawDepthImage.rows; y++)
	{
		float *rawDepthImagePtr = rawDepthImage.ptr<float>(y);
		uchar *handMaskPtr = handMask.ptr<uchar>(y);
		cv::Vec3b *colorMappedToDepthTempPtr = colorMappedToDepthTemp.ptr<cv::Vec3b>(y);
		uchar* tipPosMatPtr = tipPosMat.ptr<uchar>(y);

		for (int x = 0; x < rawDepthImage.cols; x++)
		{
			if (tipPosMatPtr[x] != 255 && handMaskPtr[x] != 255)
				continue;
			PXCPoint3DF32 dDepthPoint;
			PXCPoint3DF32 cDepthPoint;
			pcl::PointXYZRGB point;//Unit:m

			dDepthPoint.x = x;
			dDepthPoint.y = y;
			dDepthPoint.z = rawDepthImagePtr[x];

			projection->ProjectDepthToCamera(1, &dDepthPoint, &cDepthPoint);

			//�P�ʕϊ��Fmm��m
			point.x = cDepthPoint.x / CLOUD_SCALE;
			point.y = cDepthPoint.y / CLOUD_SCALE;
			point.z = cDepthPoint.z / CLOUD_SCALE;
			if (handMaskPtr[x] == 255)
			{
				point.r = colorMappedToDepthTempPtr[x][2];
				point.g = colorMappedToDepthTempPtr[x][1];
				point.b = colorMappedToDepthTempPtr[x][0];

				hand_cloud_temp->points.push_back(point);
			}
			if (tipPosMatPtr[x] == 255)
			{
				point.r = cameraNum == 0 ? 255 : 0;
				point.g = cameraNum == 1 ? 255 : 0;
				point.b = cameraNum == 2 ? 255 : 0;

				tip_cloud_temp->points.push_back(point);
			}
		}
	}


	/*for (int i = 0; i < tipPos.size(); i++)
	{
		float *rawDepthImagePtr = rawDepthImage.ptr<float>(tipPos[i].y);

		if (rawDepthImagePtr[tipPos[i].x] != 0)// && !(grayHandImagePtr[x] != 255 && isHandDataArrived)
		{
			PXCPoint3DF32 dDepthPoint;
			PXCPoint3DF32 cDepthPoint;
			pcl::PointXYZRGB point;//Unit:m

			dDepthPoint.x = tipPos[i].x;
			dDepthPoint.y = tipPos[i].y;
			dDepthPoint.z = rawDepthImagePtr[tipPos[i].x];

			projection->ProjectDepthToCamera(1, &dDepthPoint, &cDepthPoint);

			//�P�ʕϊ��Fmm��m
			point.x = cDepthPoint.x / CLOUD_SCALE;
			point.y = cDepthPoint.y / CLOUD_SCALE;
			point.z = cDepthPoint.z / CLOUD_SCALE;
			point.r = cameraNum == 0 ? 255 : 0;
			point.g = cameraNum == 1 ? 255 : 0;
			point.b = cameraNum == 2 ? 255 : 0;

			tip_cloud_temp->points.push_back(point);
		}
	}*/

	tip_point_cloud_ptr = tip_cloud_temp;
	hand_point_cloud_ptr = hand_cloud_temp;
}

cv::Mat RealSenseUpdater::drawMappedImage(void)
{
	cv::Mat colorMapped = cv::Mat::zeros(depthSize, CV_8UC3);
	for (int y = 0; y < rawDepthImage.rows; y++)
	{
		float *rawDepthImagePtr = rawDepthImage.ptr<float>(y);
		cv::Vec3b *colorMappedPtr = colorMapped.ptr<cv::Vec3b>(y);
		for (int x = 0; x < rawDepthImage.cols; x++)
		{
			if (rawDepthImagePtr[x] == 0)
				continue;

			PXCPointF32 dColorPoint;//Unit:mm
			PXCPoint3DF32 dDepthPoint;
			//PXCPoint3DF32 cDepthPoint;

			dDepthPoint.x = x;
			dDepthPoint.y = y;
			dDepthPoint.z = rawDepthImagePtr[x];

			if (projection != nullptr)
				sts = projection->MapDepthToColor(1, &dDepthPoint, &dColorPoint);
			else
			{
				showStatus(sts);
				colorMappedPtr[x] = cv::Vec3b(255, 0, 255);
				continue;
			}

			if (sts < Status::PXC_STATUS_NO_ERROR)
			{
				showStatus(sts);
				colorMappedPtr[x] = cv::Vec3b(255, 0, 255);
				continue;
			}
			//projection->ProjectDepthToCamera(1, &dDepthPoint, &cDepthPoint);

			for (auto i = 0; i < 3; i++)
			{
				//int p = y / 10 + x / 10;
				//int c = p % 2 == 1 ? 204 : 255;

				int c = rawDepthImagePtr[x] == 0 ? 0 : 255;
				cv::Vec4b colorPx = cv::Vec4b(c, c, c, 0);
				if (dColorPoint.x != -1.0 && dColorPoint.y != -1.0)
				{
					cv::Vec4b *colorImagePtr = colorImage.ptr<cv::Vec4b>((int)dColorPoint.y);
					colorPx = colorImagePtr[(int)dColorPoint.x];
				}
				colorMappedPtr[x][i] = colorPx[i];
			}
		}
	}
	return colorMapped;
}

void RealSenseUpdater::debugPrint(int line)
{
#ifdef __DEBUG_MODE__
	wprintf_s(L"hit at line %d\n", line);
#endif
}

void RealSenseUpdater::changeThreshold(bool isIncr)
{
	const double changeStep = 0.05;
	if (isIncr)
		farThreshold += changeStep;
	else
		farThreshold -= changeStep;
}

void RealSenseUpdater::showFPS()
{
	wColorIO(wColorIO::PRINT_INFO, L"FPS(");
	wColorIO(wColorIO::PRINT_VALUE, L"%d", cameraNum);
	wColorIO(wColorIO::PRINT_INFO, L"):");
	wColorIO(wColorIO::PRINT_VALUE, L"%lf\n", fps);
}

bool RealSenseUpdater::saveData(std::string directory, std::string name)
{
	cv::Mat tmp;

	flip(colorImage, tmp, 1); // ���]
	cv::imwrite(directory + "-Color" + name + ".tif", tmp); // color�摜�ۑ�
	flip(rawDepthImage, tmp, 1); // ���]
	imwrite(directory + "-Depth����p" + name + ".tif", tmp * 0x60 / 0x100); // depth����p�摜�ۑ�
	writeDepth(directory + "-Depth" + name); // depth�摜�ۑ�
	//if (isCloudArrived[CLOUD_CAMERA])
	if (camera_point_cloud_ptr->size() != 0)
		pcl::io::savePCDFileBinary(directory + "-PCLCamera" + name + ".pcd", *camera_point_cloud_ptr);
	//if (isCloudArrived[CLOUD_NEAR])
	if (near_point_cloud_ptr->size() != 0)
		pcl::io::savePCDFileBinary(directory + "-PCLNear" + name + ".pcd", *near_point_cloud_ptr);
	if (tip_point_cloud_ptr->size() != 0)
		pcl::io::savePCDFileBinary(directory + "-PCLTip" + name + ".pcd", *tip_point_cloud_ptr);


	tmp = readDepth(directory + "-Depth" + name) * 0x60 / 0x10000;

	cv::imshow("�ۑ��ς�", tmp); // �ۑ��������̂̕\��
	return true;
}

void RealSenseUpdater::setEnableHandTracking(bool _enableHandTracking)
{
	enableHandTracking = _enableHandTracking;
}

void RealSenseUpdater::setCamera(int numCam)
{
	// ��������������������������������HandsViewer��聫������������������������������
	// �d�g�݂͂悭�킩��Ȃ�
	// �J�����̐؂�ւ�
	{
		PXCSession* session = pp->QuerySession();
		PXCCapture *g_capture = nullptr;
		std::map<int, PXCCapture::DeviceInfo> g_deviceInfoMap;

		PXCSession::ImplDesc desc;
		memset(&desc, 0, sizeof(desc));
		desc.group = PXCSession::IMPL_GROUP_SENSOR;
		desc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
		int itemPosition = 0;
		for (int i = 0;; i++)
		{
			PXCSession::ImplDesc desc1;
			if (session->QueryImpl(&desc, i, &desc1) < PXC_STATUS_NO_ERROR) break;
			if (session->CreateImpl<PXCCapture>(&desc1, &g_capture) < PXC_STATUS_NO_ERROR) continue;

			for (int j = 0;; j++)
			{
				PXCCapture::DeviceInfo dinfo;
				if (g_capture->QueryDeviceInfo(j, &dinfo) < PXC_STATUS_NO_ERROR) break;
				g_deviceInfoMap[itemPosition++] = dinfo;
			}
		}
		PXCCapture::DeviceInfo* checkedDeviceInfo = &g_deviceInfoMap[numCam];
		pp->QueryCaptureManager()->FilterByDeviceInfo(checkedDeviceInfo);
		//�������ł���������senseManager->QueryCaptureManager()->FilterByDeviceInfo(checkedDeviceInfo->name, checkedDeviceInfo->did, checkedDeviceInfo->didx);
	}
	// ��������������������������������HandsViewer��聪������������������������������

	// ����������������������������������������������������������������
	// https://software.intel.com/en-us/forums/realsense/topic/561008
	//{
		//PXCSenseManager * psm = PXCSenseManager::CreateInstance();
		//PXCCapture::DeviceInfo dinfo;
		//dinfo.didx = numCam;
		//psm->QueryCaptureManager()->FilterByDeviceInfo(&dinfo);
		//psm->Release();
	//}
	// ����������������������������������������������������������������
	//
	// ����������������������������������������������������������������
	// https://software.intel.com/sites/landingpage/realsense/camera-sdk/v1.1/documentation/html/doc_essential_selecting_the_r200_enhanced_device.htmls
	//{
		//PXCCapture::DeviceInfo dinfo = {};
		//dinfo.model = PXCCapture::DEVICE_MODEL_SR300;
		//senseManager->QueryCaptureManager()->FilterByDeviceInfo(&dinfo);
	//}
	// ����������������������������������������������������������������
}

Status RealSenseUpdater::setLaserPower(int num)
{
	sts = pp->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower(num);
	if (sts < Status::PXC_STATUS_NO_ERROR)
		showStatus(sts);
	return sts;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RealSenseUpdater::updatePointCloud(bool isHandDataArrived)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr near_point_cloud_ptr_temp(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int y = 0; y < rawDepthImage.rows; y++)
	{
		float *rawDepthImagePtr = rawDepthImage.ptr<float>(y);
		cv::Vec3b *colorMappedToDepthPtr = colorMappedToDepth.ptr<cv::Vec3b>(y);

		for (int x = 0; x < rawDepthImage.cols; x++)
		{
			float *rawDepthImagePrevPtr = rawDepthImagePrev.ptr<float>(y);
			if (rawDepthImagePtr[x] != 0.0)
			{
				PXCPoint3DF32 dDepthPoint;//Unit:mm
				PXCPoint3DF32 cDepthPoint;
				pcl::PointXYZRGB point;//Unit:m
				//bool isSkip = false;

				dDepthPoint.x = x;
				dDepthPoint.y = y;
				dDepthPoint.z = rawDepthImagePtr[x];

				projection->ProjectDepthToCamera(1, &dDepthPoint, &cDepthPoint);

				//�P�ʕϊ��Fmm��m
				point.x = cDepthPoint.x / CLOUD_SCALE;
				point.y = cDepthPoint.y / CLOUD_SCALE;
				point.z = cDepthPoint.z / CLOUD_SCALE;
				point.r = colorMappedToDepthPtr[x][2];
				point.g = colorMappedToDepthPtr[x][1];
				point.b = colorMappedToDepthPtr[x][0];

				if (colorMappedToDepthPtr[x] == cv::Vec3b(0, 0, 0))
				{
					point.r = 255;
					point.g = 255;
					point.b = 255;
				}

				point_cloud_ptr->points.push_back(point);
				/*if (point.z < farThreshold&&point.z>nearThreshold)
				{
					near_point_cloud_ptr_temp->points.push_back(point);
				}
				else
					continue;*/
			}
		}
	}
	//near_point_cloud_ptr = near_point_cloud_ptr_temp;

	return(point_cloud_ptr);
}

/*int RealSenseUpdater::countMat(cv::Mat mat, cv::Vec4b elm)
{
	int matElmNum = 0;
	for (int y = 0; y > mat.rows; y++)
	{
		cv::Vec4b *matPtr = mat.ptr<cv::Vec4b>(y);
		for (int x = 0; x > mat.cols; x++)
		{
			if (matPtr[x] == elm)
				matElmNum++;
		}
	}
	return matElmNum;
}

int RealSenseUpdater::countMat(cv::Mat mat, unsigned char elm)
{
	int matElmNum = 0;
	for (int y = 0; y > mat.rows; y++)
	{
		unsigned char *matPtr = mat.ptr<uchar>(y);
		for (int x = 0; x > mat.cols; x++)
		{
			if (matPtr[x] == elm)
				matElmNum++;
		}
	}
	return matElmNum;
}

int RealSenseUpdater::countMat(cv::Mat mat, float elm)
{
	int matElmNum = 0;
	for (int y = 0; y > mat.rows; y++)
	{
		float *matPtr = mat.ptr<float>(y);
		for (int x = 0; x > mat.cols; x++)
		{
			if (matPtr[x] == elm)
				matElmNum++;
		}
	}
	return matElmNum;
}*/

void RealSenseUpdater::writeDepth(const std::string name)
{
	cv::Mat matDepth = rawDepthImage;
	std::fstream fs;

	std::string dir = name;

	// �g���q�����Ă��邩
	std::string suffix = "." + extension;
	if (dir.size() < suffix.size() || dir.find(suffix, dir.size() - suffix.size()) == std::string::npos)
		dir += suffix;

	fs.open(dir, std::ios::out | std::ios::binary);

	if (!fs.is_open())
		throw std::runtime_error("depth32f�摜�̏������݃t�@�C���̌Ăяo���Ɏ��s");

	if (enableMirror != isSaveMirror) // �~���[���邩
	{
		cv::flip(matDepth.clone(), matDepth, 1); // ���]
	}

	dptHeader header;
	header.width = matDepth.cols;
	header.height = matDepth.rows;
	header.type = matDepth.type();
	if (isSaveMirror) // �~���[���Ă��邩
	{
		header.data1 |= 1 << d1_mirror; // �w�b�_�Ƀ~���[��񏑂�����
	}

	if (!fs.write((const char*)(&header), sizeof(header)) // �w�b�_��������
		|| !fs.seekp(header.size) // �|�C���^�ʒu�ړ�
		|| !fs.write((const char*)(matDepth.data), matDepth.total() * matDepth.elemSize())) // �f�[�^��������
		throw std::runtime_error("depth32f�摜�̏������݂Ɏ��s");

	fs.close();
}


cv::Mat RealSenseUpdater::readDepth(const std::string name)
{
	std::fstream fs;

	std::string dir = name;

	// �g���q�����Ă��邩
	std::string suffix = "." + extension;
	if (dir.size() < suffix.size() || dir.find(suffix, dir.size() - suffix.size()) == std::string::npos)
		dir += suffix;

	fs.open(dir, std::ios::in | std::ios::binary);
	if (!fs.is_open())
		throw std::runtime_error("depth32f�摜�̓ǂݍ��݃t�@�C���̌Ăяo���Ɏ��s");

	dptHeader header;

	if (!fs.read((char*)(&header.size), sizeof(header.size))
		|| !fs.read((char*)(&header.identifier), sizeof(header) - sizeof(header.size))
		|| !fs.seekg(header.size))
		throw std::runtime_error("depth32f�摜�̃w�b�_�̓ǂݍ��݂Ɏ��s");

	cv::Mat img(header.height, header.width, header.type);

	if (!fs.read((char*)(img.data), img.total() * img.elemSize()))
		throw std::runtime_error("depth32f�摜�̃f�[�^�̓ǂݍ��݂Ɏ��s");

	fs.close();

	if (enableMirror != (header.data1 & 1 << d1_mirror)) // �~���[���邩
	{
		cv::flip(img.clone(), img, 1); // ���]
	}

	return img.clone();
}

void RealSenseUpdater::showStatus(Status sts)
{
	wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
	switch (sts)
	{
	case Intel::RealSense::NSStatus::STATUS_NO_ERROR:
		wColorIO(wColorIO::PRINT_SUCCESS, L"STATUS_NO_ERROR");
		break;
	case Intel::RealSense::NSStatus::STATUS_FEATURE_UNSUPPORTED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_FEATURE_UNSUPPORTED");
		break;
	case Intel::RealSense::NSStatus::STATUS_PARAM_UNSUPPORTED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_PARAM_UNSUPPORTED");
		break;
	case Intel::RealSense::NSStatus::STATUS_ITEM_UNAVAILABLE:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_ITEM_UNAVAILABLE");
		break;
	case Intel::RealSense::NSStatus::STATUS_HANDLE_INVALID:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_HANDLE_INVALID");
		break;
	case Intel::RealSense::NSStatus::STATUS_ALLOC_FAILED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_ALLOC_FAILED");
		break;
	case Intel::RealSense::NSStatus::STATUS_DEVICE_FAILED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_DEVICE_FAILED");
		break;
	case Intel::RealSense::NSStatus::STATUS_DEVICE_LOST:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_DEVICE_LOST");
		break;
	case Intel::RealSense::NSStatus::STATUS_DEVICE_BUSY:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_DEVICE_BUSY");
		break;
	case Intel::RealSense::NSStatus::STATUS_EXEC_ABORTED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_EXEC_ABORTED");
		break;
	case Intel::RealSense::NSStatus::STATUS_EXEC_INPROGRESS:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_EXEC_INPROGRESS");
		break;
	case Intel::RealSense::NSStatus::STATUS_EXEC_TIMEOUT:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_EXEC_TIMEOUT");
		break;
	case Intel::RealSense::NSStatus::STATUS_FILE_WRITE_FAILED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_FILE_WRITE_FAILED");
		break;
	case Intel::RealSense::NSStatus::STATUS_FILE_READ_FAILED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_FILE_READ_FAILED");
		break;
	case Intel::RealSense::NSStatus::STATUS_FILE_CLOSE_FAILED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_FILE_CLOSE_FAILED");
		break;
	case Intel::RealSense::NSStatus::STATUS_DATA_UNAVAILABLE:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_DATA_UNAVAILABLE");
		break;
	case Intel::RealSense::NSStatus::STATUS_DATA_NOT_INITIALIZED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_DATA_NOT_INITIALIZED");
		break;
	case Intel::RealSense::NSStatus::STATUS_INIT_FAILED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_INIT_FAILED");
		break;
	case Intel::RealSense::NSStatus::STATUS_STREAM_CONFIG_CHANGED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_STREAM_CONFIG_CHANGED");
		break;
	case Intel::RealSense::NSStatus::STATUS_POWER_UID_ALREADY_REGISTERED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_POWER_UID_ALREADY_REGISTERED");
		break;
	case Intel::RealSense::NSStatus::STATUS_POWER_UID_NOT_REGISTERED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_POWER_UID_NOT_REGISTERED");
		break;
	case Intel::RealSense::NSStatus::STATUS_POWER_ILLEGAL_STATE:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_POWER_ILLEGAL_STATE");
		break;
	case Intel::RealSense::NSStatus::STATUS_POWER_PROVIDER_NOT_EXISTS:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_POWER_PROVIDER_NOT_EXISTS");
		break;
	case Intel::RealSense::NSStatus::STATUS_CAPTURE_CONFIG_ALREADY_SET:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_CAPTURE_CONFIG_ALREADY_SET");
		break;
	case Intel::RealSense::NSStatus::STATUS_COORDINATE_SYSTEM_CONFLICT:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_COORDINATE_SYSTEM_CONFLICT");
		break;
	case Intel::RealSense::NSStatus::STATUS_NOT_MATCHING_CALIBRATION:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_NOT_MATCHING_CALIBRATION");
		break;
	case Intel::RealSense::NSStatus::STATUS_ACCELERATION_UNAVAILABLE:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_ACCELERATION_UNAVAILABLE");
		break;
	case Intel::RealSense::NSStatus::STATUS_WRONG_META_DATA_VALUE:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_WRONG_META_DATA_VALUE");
		break;
	case Intel::RealSense::NSStatus::STATUS_TIME_GAP:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_TIME_GAP");
		break;
	case Intel::RealSense::NSStatus::STATUS_PARAM_INPLACE:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_PARAM_INPLACE");
		break;
	case Intel::RealSense::NSStatus::STATUS_DATA_NOT_CHANGED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_DATA_NOT_CHANGED");
		break;
	case Intel::RealSense::NSStatus::STATUS_PROCESS_FAILED:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_PROCESS_FAILED");
		break;
	case Intel::RealSense::NSStatus::STATUS_VALUE_OUT_OF_RANGE:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_VALUE_OUT_OF_RANGE");
		break;
	case Intel::RealSense::NSStatus::STATUS_DATA_PENDING:
		wColorIO(wColorIO::PRINT_ERROR, L"STATUS_DATA_PENDING");
		break;
	default:
		break;
	}
	wColorIO(sts == Intel::RealSense::NSStatus::PXC_STATUS_NO_ERROR ? wColorIO::PRINT_SUCCESS : wColorIO::PRINT_ERROR, L"(code:%d)\n", sts);
}

