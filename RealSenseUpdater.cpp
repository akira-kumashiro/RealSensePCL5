#include "RealSenseUpdater.h"

RealSenseUpdater::RealSenseUpdater() :
	//viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
	hand_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	camera_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	hand_joint_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	near_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	tip_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>)//PCL関連の変数の初期化
{

	wColorIO(wColorIO::PRINT_INFO, L"RSU>");
	wColorIO(wColorIO::PRINT_INFO, L"Start\n");

	/*hand_point_cloud_ptr->width = DEPTH_WIDTH;
	hand_point_cloud_ptr->height = DEPTH_HEIGHT;

	camera_point_cloud_ptr->width = DEPTH_WIDTH;
	camera_point_cloud_ptr->height = DEPTH_HEIGHT;

	hand_joint_cloud_ptr->points.resize(DEPTH_WIDTH*DEPTH_HEIGHT);
	camera_point_cloud_ptr->points.resize(DEPTH_WIDTH*DEPTH_HEIGHT);

	if (hand_point_cloud_ptr->isOrganized())
		wColorIO(wColorIO::PRINT_INFO, L"organized\n");
	else
		wColorIO(wColorIO::PRINT_INFO, L"unorganized\n");

	if (camera_point_cloud_ptr->isOrganized())
		wColorIO(wColorIO::PRINT_INFO, L"organized\n");
	else
		wColorIO(wColorIO::PRINT_INFO, L"unorganized\n");*/

	pp = SenseManager::CreateInstance();
	if (!pp)
	{
		wprintf_s(L"Unable to create the SenseManager\n");
	}

	//cv::namedWindow(windowName[0], CV_WINDOW_AUTOSIZE);
	rawDepthImage = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_32FC1);
#ifdef __ENABLE_HAND_TRACKING__
	handImage1 = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC1);
	handImage2 = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC1);
	handImage = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC4);
	handPoint = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC4);
#endif
	/*rawDepthDiffImage = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC1);
	rawDepthDiffImageFilterd = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC3);*/

	depthmarked = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC3);

	//	viewer->registerKeyboardCallback(&RealSenseUpdater::keyboardCallback, *this);

	isContinue = false;
	isUserInterrupt = false;
	nowTime = std::chrono::system_clock::now();
	//_time = getTime();
}


RealSenseUpdater::~RealSenseUpdater()
{
	// Releases lock so pipeline can process next frame 
	pp->ReleaseFrame();
	//viewer->close();

	cv::destroyAllWindows();

#ifdef __ENABLE_HAND_TRACKING__
	if (enableHandTracking)
	{
		if (handAnalyzer == nullptr)
		{
			pp->Release();
			pp = nullptr;
		}
		if (handData == nullptr)
		{
			handData->Release();
			handData = nullptr;
		}
	}
#endif

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
	showStatus(sts);
	if (sts < Status::STATUS_NO_ERROR)
	{
		return RSU_ERROR_OCCURED;
	}
	sts = ppInit(num);
	showStatus(sts);
	if (sts < Status::STATUS_NO_ERROR)
	{
		return RSU_ERROR_OCCURED;
	}
	return RSU_NO_ERROR;
}

int RealSenseUpdater::run(void)
{
	prevTime = nowTime;
	nowTime = std::chrono::system_clock::now();

	auto def = nowTime - prevTime;

	fps = 1000 / std::chrono::duration<double, std::milli>(def).count();

	//sts = ppInit();
	//if (sts >= Status::PXC_STATUS_NO_ERROR)
	//{
	//	while (1)
	//	{
	if (sts < Status::PXC_STATUS_NO_ERROR)
	{
		showStatus(sts);
		return RSU_ERROR_OCCURED;
	}

	/*for (int i = 0; i < CLOUD_NUM; i++)
	{
		isCloudArrived[i] = false;
	}*/
	//viewer->spinOnce();

	//Waits until new frame is available and locks it for application processing
	sts = pp->AcquireFrame(false);//ここをtrueにするとたまにめっちゃ時間がかかる

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
		//break;
	}

	const PXCCapture::Sample *sample = pp->QuerySample();
	if (sample != nullptr)
	{
		if (sample->color && !acqireImage(sample->color, colorImage, PXCImage::PIXEL_FORMAT_RGB32))//updateCameraImage(sample->color, false)
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
#endif
		PXCImage* projectionImage = projection->CreateColorImageMappedToDepth(sample->depth, sample->color);
		if (projectionImage == nullptr)
			return RSU_COLOR_IMAGE_UNAVAILABLE + RSU_DEPTH_IMAGE_UNAVAILABLE;
		else if (acqireImage(projectionImage, colorMappedToDepth, PXCImage::PixelFormat::PIXEL_FORMAT_RGB24))
			projectionImage->Release();

		/*if (sample->color&&sample->depth)
		{

		}*/
		//detC(rawDepthDiffImage.clone());

#ifdef __ENABLE_HAND_TRACKING__
		if (enableHandTracking)
		{
			if (handData)
			{
				handData->Update();
				updateHandImage();
				//hand_point_cloud_ptr = updatePointCloud(true);

				//viewer->updatePointCloud(hand_point_cloud_ptr, "handcloud");
				//viewer->updatePointCloud(hand_joint_cloud_ptr, "handjoint");

				/*if (pointCloudNum[CLOUD_HAND] != 0)
					isCloudArrived[CLOUD_HAND] = true;*/

					//viewer = app.rgbVis(point_cloud_ptr);

				realsenseHandStatus(handData);
			}
			else
			{
				wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
				wColorIO(wColorIO::PRINT_ERROR, L"Hands couldn't be detected\n");
				releaseHandImage();
			}
		}
#endif
		calcDepthMark();
		setTipCloud();

		camera_point_cloud_ptr = updatePointCloud(false);

		//viewer->updatePointCloud(camera_point_cloud_ptr, "cameracloud");

		/*if (pointCloudNum[CLOUD_CAMERA] != 0)
			isCloudArrived[CLOUD_CAMERA] = true;
		if (pointCloudNum[CLOUD_NEAR] != 0)
			isCloudArrived[CLOUD_NEAR] = true;*/

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
#ifdef __ENABLE_HAND_TRACKING__
	if (enableHandTracking)
	{
		if (handAnalyzer == nullptr)
		{
			pp->Release();
			pp = nullptr;
		}
		if (handData == nullptr)
		{
			handData->Release();
			handData = nullptr;
		}
	}
#endif
	/*else
	{
		pp->Release();
		pp = nullptr;
	}*/

	if (projection == nullptr)
	{
		projection->Release();
		projection = nullptr;
	}
#ifdef __DEBUG_MODE__
	wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
	wColorIO(wColorIO::PRINT_SUCCESS, L"Pipeline release successful\n");
#endif
	// ウィンドウが消されても再表示する
	//cv::namedWindow("カメラ");
	cv::namedWindow("保存済み");

	//shorGuideImage(rawDepthImage, num);

	//keyboardCallBackSettings(cv::waitKey(1));

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

	return RSU_NO_ERROR;
	//}

	/*cv::destroyAllWindows();
	pp->Close();

	if (sts == Status::STATUS_STREAM_CONFIG_CHANGED)
	{
		wprintf_s(L"Stream configuration was changed, re-initilizing\n");
		isContinue = true;
		return(isContinue);
	}
	else if (isUserInterrupt)
	{
		wColorIO(wColorIO::PRINT_WARN, L"Stream has been stoped by user interrupt.\n");
	}
	else
	{
		wprintf_s(L"Stream has been stoped due to error(s)\nsts=");
		showStatus(sts);
	}
	wprintf_s(L"Do you want to re-initilizing?\nY/N\n");

	while (true)
	{
		int c = _getch();
		if (c == 'Y' || c == 'y')
		{
			isContinue = true;
			isExit = false;
			break;
		}
		else if (c == 'N' || c == 'n' || c == 27)
		{
			isContinue = false;
			break;
		}
	}
	//return(isContinue);
	return true;*/
}

/*bool RealSenseUpdater::keyboardCallBackSettings(int key)
{
	cv::Mat tmp;

	switch (key)
	{
	case CV_WAITKEY_CURSORKEY_TOP:num++;	break; // 数字 + 1
	case CV_WAITKEY_CURSORKEY_BOTTOM:num--;	break; // 数字 - 1
	case CV_WAITKEY_CURSORKEY_RIGHT:hrgn++;	break; // 文字 + 1
	case CV_WAITKEY_CURSORKEY_LEFT:hrgn--;	break; // 文字 - 1
	case 'r':
		for (int i = 0; i < HandJointData::NUM_OF_JOINT_DST; i++)
		{
			wColorIO(wColorIO::PRINT_INFO, L"right hand %d=%lf\n", i, handjointdata[0].jointDistance[i]);
		}
		break;
	case 'l':
		for (int i = 0; i < HandJointData::NUM_OF_JOINT_DST; i++)
		{
			wColorIO(wColorIO::PRINT_INFO, L"right hand %d=%lf\n", i, handjointdata[1].jointDistance[i]);
		}
		break;
	case 't':
		if (cloudAlphaCh == 2)
			cloudAlphaCh = 0;
		else
			cloudAlphaCh++;
		break;
	case 'w':
	case 'W':
		morph_size++;
		break;
	case 's':
	case 'S':
		if (morph_size > 1)
			morph_size--;
		else
			morph_size = 0;
		break;
	case 'd':
	case 'D':
		if (morph_elem >= 2)
			morph_elem = 2;
		else
			morph_elem++;
		break;
	case 'a':
	case 'A':
		if (morph_elem <= 0)
			morph_elem = 0;
		else
			morph_elem--;
		break;
		case 'z':
			if (gSize > 1)
				gSize -= 2;
			else
				gSize = 1;
			break;
		case 'x':
			gSize += 2;
			break;
		case 'c':
			if (sigmaG > 0)
				sigmaG -= 1;
			else
				sigmaG = 0;
			break;
		case 'v':
			sigmaG += 1;
			break;
	case ' ': // 保存
		CreateDirectory((_time).c_str(), NULL); // 大本のフォルダ作成（名前が時間）
		CreateDirectory((_time + "\\" + makeNameFolder(hrgn) + "-Color").c_str(), NULL); // color画像フォルダ作成
		CreateDirectory((_time + "\\" + makeNameFolder(hrgn) + "-Depth").c_str(), NULL); // depth画像フォルダ作成
		CreateDirectory((_time + "\\" + makeNameFolder(hrgn) + "-HandImage").c_str(), NULL); // HandImage画像フォルダ作成
		CreateDirectory((_time + "\\" + makeNameFolder(hrgn) + "-HandPoint").c_str(), NULL); // HandPoint画像フォルダ作成
		CreateDirectory((_time + "\\" + makeNameFolder(hrgn) + "-PCLHand").c_str(), NULL); // PCLHand画像フォルダ作成
		CreateDirectory((_time + "\\" + makeNameFolder(hrgn) + "-PCLJoint").c_str(), NULL); // PCLJoint画像フォルダ作成
		CreateDirectory((_time + "\\" + makeNameFolder(hrgn) + "-PCLCamera").c_str(), NULL); // PCLCamera画像フォルダ作成
		CreateDirectory((_time + "\\" + makeNameFolder(hrgn) + "-Depth見る用").c_str(), NULL); // depth見る用画像フォルダ作成

		flip(colorImage, tmp, 1); // 反転
		cv::imwrite(_time + "\\" + makeNameFolder(hrgn) + "-Color" + "\\" + makeNameFail(hrgn, num) + ".tif", tmp); // color画像保存
		flip(handImage, tmp, 1); // 反転
		cv::imwrite(_time + "\\" + makeNameFolder(hrgn) + "-HandImage" + "\\" + makeNameFail(hrgn, num) + ".tif", tmp); // handImage画像保存
		flip(handPoint, tmp, 1); // 反転
		cv::imwrite(_time + "\\" + makeNameFolder(hrgn) + "-HandPoint" + "\\" + makeNameFail(hrgn, num) + ".tif", tmp); // handPoint画像保存
		flip(rawDepthImage, tmp, 1); // 反転
		imwrite(_time + "\\" + makeNameFolder(hrgn) + "-Depth見る用" + "\\" + makeNameFail(hrgn, num) + ".tif", tmp * 0x60 / 0x100); // depth見る用画像保存
		writeDepth(_time + "\\" + makeNameFolder(hrgn) + "-Depth" + "\\" + makeNameFail(hrgn, num)); // depth画像保存
		if (isCloudArrived[CLOUD_HAND])
		{
			pcl::io::savePCDFileBinary(_time + "\\" + makeNameFolder(hrgn) + "-PCLHand" + "\\" + makeNameFail(hrgn, num) + ".pcd", *hand_point_cloud_ptr);
			PointCloud2Mesh(hand_point_cloud_ptr, _time + "\\" + makeNameFolder(hrgn) + "-PCLHand" + "\\" + makeNameFail(hrgn, num) + "s.obj", param, true);
		}
		if (isCloudArrived[CLOUD_CAMERA])
			pcl::io::savePCDFileBinary(_time + "\\" + makeNameFolder(hrgn) + "-PCLCamera" + "\\" + makeNameFail(hrgn, num) + ".pcd", *camera_point_cloud_ptr);
		if (isCloudArrived[CLOUD_JOINT])
			pcl::io::savePCDFileBinary(_time + "\\" + makeNameFolder(hrgn) + "-PCLJoint" + "\\" + makeNameFail(hrgn, num) + ".pcd", *hand_joint_cloud_ptr);
		tmp = readDepth(_time + "\\" + makeNameFolder(hrgn) + "-Depth" + "\\" + makeNameFail(hrgn, num)) * 0x60 / 0x10000;

		cv::imshow("保存済み", tmp); // 保存したものの表示
		num++;
		break;
	case 'q': // 終了
		cv::destroyAllWindows();
		isUserInterrupt = true;
		return false;
	default:
		return true;
	}

	if (numMax - 1 < num)
	{
		num = 0;
		hrgn++;
	}
	else if (num < 0)
	{
		num = numMax - 1;
		hrgn--;
	}
	if (hrgn < 0)
	{
		num = 0;
		hrgn = 0;
	}
	else if ('ﾝ' - 'ｱ' < hrgn)
	{
		num = numMax - 1;
		hrgn = 'ﾝ' - 'ｱ';
	}
	printText(hrgn, num);
	// ボタンが押されたときのみ表示
	return true;
}*/

Status RealSenseUpdater::ppInit(int num)
{
	//isContinue = false;
	isExit = false;
	isUserInterrupt = false;

	// ストリームを有効にする
	sts = pp->EnableStream(PXCCapture::StreamType::STREAM_TYPE_COLOR, COLOR_WIDTH, COLOR_HEIGHT, COLOR_FPS);

	if (sts < Status::STATUS_NO_ERROR)
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"Create Color-Image pipeline has been unsuccessful.\n");
		return sts;
	}
	else
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_SUCCESS, L"Create Color-Image pipeline has been successful.\n");
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_INFO, L"Width:");
		wColorIO(wColorIO::PRINT_VALUE, L"%dpx ", COLOR_WIDTH);
		wColorIO(wColorIO::PRINT_INFO, L"Height:");
		wColorIO(wColorIO::PRINT_VALUE, L"%dpx ", COLOR_HEIGHT);
		wColorIO(wColorIO::PRINT_INFO, L"FPS:");
		wColorIO(wColorIO::PRINT_VALUE, L"%d\n", COLOR_FPS);
	}

	sts = pp->EnableStream(PXCCapture::StreamType::STREAM_TYPE_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS);
	if (sts < Status::STATUS_NO_ERROR)
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"Create Depth-Image pipeline has been unsuccessful.\n");
		return sts;
	}
	else
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_SUCCESS, L"Create Depth-Image pipeline has been successful.\n");
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_INFO, L"Width:");
		wColorIO(wColorIO::PRINT_VALUE, L"%dpx ", DEPTH_WIDTH);
		wColorIO(wColorIO::PRINT_INFO, L"Height:");
		wColorIO(wColorIO::PRINT_VALUE, L"%dpx ", DEPTH_HEIGHT);
		wColorIO(wColorIO::PRINT_INFO, L"FPS:");
		wColorIO(wColorIO::PRINT_VALUE, L"%d\n", DEPTH_FPS);
	}

#ifdef __ENABLE_HAND_TRACKING__
	if (enableHandTracking)
	{
		sts = pp->EnableHand();

		if (sts < PXC_STATUS_NO_ERROR)
		{
			wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
			wColorIO(wColorIO::PRINT_ERROR, L"Failed to pair the hand module with I/O.\n");
			return sts;
		}
		else
		{
			wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
			wColorIO(wColorIO::PRINT_SUCCESS, L"Succeeded to pair the hand module with I/O.\n");
		}

		handAnalyzer = pp->QueryHand();

		if (handAnalyzer == NULL)
		{
			wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
			wColorIO(wColorIO::PRINT_ERROR, L"Failed to pair the hand module with I/O\n");
			return sts;
		}
		else
		{
			wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
			wColorIO(wColorIO::PRINT_SUCCESS, L"Succeeded to pair the hand module with I/O\n");
		}
	}
#endif
	// パイプラインを初期化する
	sts = pp->Init();

	if (sts >= PXC_STATUS_NO_ERROR)
	{
		PXCCapture::Device *device = pp->QueryCaptureManager()->QueryDevice();
		PXCCapture::DeviceInfo dinfo;

		if (device != NULL)// ミラー表示にする
		{
			device->SetMirrorMode(Capture::Device::MirrorMode::MIRROR_MODE_HORIZONTAL);

			wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
			wColorIO(wColorIO::PRINT_INFO, L"MirrorMode has been enabled.\n");
		}

		//projectionを作る前にsetCameraしないとprojectionがおかしくなる
		setCamera(num);

		projection = device->CreateProjection();

#ifdef __ENABLE_HAND_TRACKING__
		if (enableHandTracking)
		{
			device->QueryDeviceInfo(&dinfo);
			if (dinfo.model == PXCCapture::DEVICE_MODEL_IVCAM)
			{
				device->SetDepthConfidenceThreshold(1);
				device->SetIVCAMFilterOption(6);
			}


			if (handAnalyzer)
			{
				handData = handAnalyzer->CreateOutput();
				config = handAnalyzer->CreateActiveConfiguration();
				//config->SetTrackingMode(PXCHandData::TRACKING_MODE_EXTREMITIES);//高速・輪郭モード時にコメントアウト
				//config->EnableNormalizedJoints(showNormalizedSkeleton);
				config->EnableSegmentationImage(true);
				config->DisableAllGestures();
			}
			else
			{
				wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
				wColorIO(wColorIO::PRINT_ERROR, L"Failed to set up handData\n");
				return sts;
			}
			config->ApplyChanges();
			config->Update();
		}
#endif
	}
	else
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"Pipeline initializing has been failed.\n");
		return sts;
	}

	if (sts >= PXC_STATUS_NO_ERROR)
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_SUCCESS, L"Create pipeline has been successful.\n");
	}

	//viewer->setBackgroundColor(0, 0, 0);
	/*if (enableHandTracking)
	{
		initializeViewer("handcloud", hand_point_cloud_ptr);
		initializeViewer("handjoint", hand_joint_cloud_ptr, 10);
	}*/
	//initializeViewer("cameracloud", camera_point_cloud_ptr, 0.1);
	//viewer->addCoordinateSystem(0.01);
	//viewer->initCameraParameters();

	return sts;
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
	}

	// データをコピーする
	PXCImage::ImageInfo info = cameraFrame->QueryInfo();

	if (pixelFormat == PXCImage::PIXEL_FORMAT_RGB32)
		mat = cv::Mat(info.height, info.width, CV_8UC4);
	else if (pixelFormat == PXCImage::PIXEL_FORMAT_DEPTH_F32)
		mat = cv::Mat(info.height, info.width, CV_32FC1);
	else if (pixelFormat == PXCImage::PIXEL_FORMAT_RGB24)
		mat = cv::Mat(info.height, info.width, CV_8UC3);
	else
		return false;

	memcpy(mat.data, data.planes[0], data.pitches[0] * info.height);

	// データを解放する
	cameraFrame->ReleaseAccess(&data);
	return true;
}

bool RealSenseUpdater::updateCameraImage(PXCImage* cameraFrame, bool isDepthImage)
{
	if (cameraFrame == nullptr)
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
		wColorIO(wColorIO::PRINT_ERROR, L"cameraFrame throws nullptr.\n");
		return false;
	}

	if (isDepthImage)
	{
		/*if (!acqireImage(cameraFrame, depthImage, PXCImage::PIXEL_FORMAT_RGB32))
			return false;*/
		if (!acqireImage(cameraFrame, rawDepthImage, PXCImage::PIXEL_FORMAT_DEPTH_F32))
			return false;
		/*if (rawDepthImagePrev.total() < 1)
			return true;*/

			//cv::GaussianBlur(rawDepthImage, rawDepthImageGauss, cv::Size(gSize, gSize), sigmaG, sigmaG);

			/*for (int y = 0; y < rawDepthImage.rows; y++)
			{
				float *rawDepthImagePtr = rawDepthImage.ptr<float>(y);
				float *rawDepthImagePrevPtr = rawDepthImagePrev.ptr<float>(y);
				unsigned char *rawDepthDiffImagePtr = rawDepthDiffImage.ptr<uchar>(y);
				float *rawDepthImageGaussPtr = rawDepthImageGauss.ptr<float>(y);

				for (int x = 0; x < rawDepthImage.cols; x++)
				{
					if (abs(rawDepthImagePtr[x] - rawDepthImagePrevPtr[x]) > DIFF_EXCLUDE_THRESHOLD)
					{
						rawDepthDiffImagePtr[x] = 255;
					}
					else if (abs(rawDepthImagePtr[x] - rawDepthImageGaussPtr[x]) > GAUSS_EXCLUDE_THRESHOLD)
					{
						rawDepthDiffImagePtr[x] = 128;
					}
					else
					{
						rawDepthDiffImagePtr[x] = 0;
					}
				}
			}*/
	}
	else
	{
		if (!acqireImage(cameraFrame, colorImage, PXCImage::PIXEL_FORMAT_RGB32))
			return false;
	}
	return true;
}

#ifdef __ENABLE_HAND_TRACKING__
bool RealSenseUpdater::updateHandImage(void)
{
	PXCImage::ImageData data;
	pointCloudNum[CLOUD_JOINT] = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_joint_cloud_ptr_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (handData != nullptr&&handData)
	{
		numberOfHands = handData->QueryNumberOfHands();

		handImage = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC4);
		grayHandImage = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC1);
		handPoint = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC4);

		for (int i = 0; i < numberOfHands; i++)
		{
			//pxcUID handID;
			PXCHandData::IHand* hand;

			sts = handData->QueryHandData(PXCHandData::AccessOrderType::ACCESS_ORDER_BY_ID, i, hand);
			if (sts < PXC_STATUS_NO_ERROR)
			{
				wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
				wColorIO(wColorIO::PRINT_ERROR, L"Couldn't update hand frame.\n");
				continue;
			}

			PXCImage* image = 0;
			sts = hand->QuerySegmentationImage(image);
			if (sts < PXC_STATUS_NO_ERROR)
			{
				wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
				wColorIO(wColorIO::PRINT_ERROR, L"Couldn't acquire hand image.\n");
				continue;
			}

			sts = image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_Y8, &data);
			if (sts < PXC_STATUS_NO_ERROR)
			{
				wColorIO(wColorIO::PRINT_INFO, L"RSU#%d>", cameraNum);
				wColorIO(wColorIO::PRINT_ERROR, L"Couldn't acquire masked hand image.\n");
				continue;
			}

			PXCImage::ImageInfo info = image->QueryInfo();

			auto& handImageTemp = i ? handImage2 : handImage1;
			memcpy(handImageTemp.data, data.planes[0], info.height*info.width);

			for (int y = 0; y < handImage.rows; y++)
			{
				cv::Vec4b *handImagePtr = handImage.ptr<cv::Vec4b>(y);
				unsigned char *handImage1Ptr = handImage1.ptr<uchar>(y);
				unsigned char *handImage2Ptr = handImage2.ptr<uchar>(y);
				unsigned char *grayHandImagePtr = grayHandImage.ptr<uchar>(y);
				for (int x = 0; x < handImage.cols; x++)
				{
					if ((i ? handImage2Ptr[x] : handImage1Ptr[x]) == 255)
					{
						handImagePtr[x] = cv::Vec4b(255, 255, 255, 255);
						grayHandImagePtr[x] = 255;
					}
				}
			}

			for (int j = 0; j < PXCHandData::NUMBER_OF_JOINTS; j++)
			{
				//int x, y = 0;
				PXCHandData::JointData jointData;
				sts = hand->QueryTrackedJoint((PXCHandData::JointType)j, jointData);
				if (sts < PXC_STATUS_NO_ERROR)
				{
					continue;
				}
				cv::circle(handPoint, cv::Point(jointData.positionImage.x, jointData.positionImage.y), 5, cv::Scalar(128 * (1 - i), 128 * i, 0, 0));

				pcl::PointXYZRGB point;

				point.x = jointData.positionWorld.x;
				point.y = jointData.positionWorld.y;
				point.z = jointData.positionWorld.z;
				point.r = 255 * (1 - i);
				point.g = 255 * i;
				point.b = 10 * j;
				//point.a = 0;

				handjointdata[i].x[j] = jointData.positionWorld.x;
				handjointdata[i].y[j] = jointData.positionWorld.y;
				handjointdata[i].z[j] = jointData.positionWorld.z;

				isCloudArrived[CLOUD_JOINT] = true;
				pointCloudNum[CLOUD_JOINT]++;

				hand_joint_cloud_ptr_tmp->points.push_back(point);
			}
			image->ReleaseAccess(&data);

			auto center = hand->QueryMassCenterImage();
			cv::circle(handPoint, cv::Point(center.x, center.y), 5, cv::Scalar(0, 255 * i, 255 * (1 - i)), -1);
		}

		for (int y = 0; y < handImage.rows; y++)
		{
			unsigned char *grayHandImagePtr = grayHandImage.ptr<uchar>(y);
			float *rawDepthImagePtr = rawDepthImage.ptr<float>(y);
			cv::Vec4b *handImagePtr = handImage.ptr<cv::Vec4b>(y);

			for (int x = 0; x < handImage.cols; x++)
			{
				if (grayHandImagePtr[x] == 255)
				{
					PXCPointF32 dColorPoint;
					PXCPoint3DF32 dDepthPoint;

					dDepthPoint.x = x;
					dDepthPoint.y = y;
					dDepthPoint.z = rawDepthImagePtr[x];

					projection->MapDepthToColor(1, &dDepthPoint, &dColorPoint);

					if (dColorPoint.x != -1.0 && dColorPoint.y != -1.0)
					{
						cv::Vec4b *colorImagePtr = colorImage.ptr<cv::Vec4b>((int)dColorPoint.y);

						cv::Vec4b colorPx = colorImagePtr[(int)dColorPoint.x];

						handImagePtr[x] = colorPx;
					}
				}
			}
		}
		if (handjointdata[0].isHandSurfaceVartical(1))
			handjointdata[0].getJointLength();

		if (handjointdata[1].isHandSurfaceVartical(1))
			handjointdata[1].getJointLength();

		cv::imshow(windowName[0], handImage);

		hand_joint_cloud_ptr = hand_joint_cloud_ptr_tmp;

		return(true);
	}
	else
	{
		wColorIO(wColorIO::PRINT_INFO, L"RSU>");
		wColorIO(wColorIO::PRINT_ERROR, L"handData is nullptr.\n");
		//return(point_cloud_ptr);
		return(false);
	}
}
#endif

bool RealSenseUpdater::isOutliers(float rawDepthElem, float rawDepthPrevElem)
{
	if (abs(rawDepthElem - rawDepthPrevElem) > DIFF_EXCLUDE_THRESHOLD)
	{
		return true;
	}
	else
	{
		return false;
	}
}

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
			else if (rawDepthImagePtr[x] < farThreshold*CLOUD_SCALE&&rawDepthImagePtr[x]>nearThreshold*CLOUD_SCALE)
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

	// Hough変換のための前処理（画像の平滑化を行なわないと誤検出が発生しやすい）
	cv::GaussianBlur(work_img, work_img, cv::Size(11, 11), 2, 2);

	// Hough変換による円の検出と検出した円の描画
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
	HandDetect det(nearThreshold*CLOUD_SCALE, farThreshold*CLOUD_SCALE);
	//std::vector<cv::Point> tipPos;
	/*cv::Mat colorMapped = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC3);
	for (int y = 0; y < rawDepthImage.rows; y++)
	{
		float *rawDepthImagePtr = rawDepthImage.ptr<float>(y);
		cv::Vec3b *colorMappedPtr = colorMapped.ptr<cv::Vec3b>(y);
		for (int x = 0; x < rawDepthImage.cols; x++)
		{
			PXCPointF32 dColorPoint;//Unit:mm
			PXCPoint3DF32 dDepthPoint;
			PXCPoint3DF32 cDepthPoint;

			dDepthPoint.x = x;
			dDepthPoint.y = y;
			dDepthPoint.z = rawDepthImagePtr[x];

			projection->MapDepthToColor(1, &dDepthPoint, &dColorPoint);
			projection->ProjectDepthToCamera(1, &dDepthPoint, &cDepthPoint);


			for (auto i = 0; i < 3; i++)
			{
				int p = y / 10 + x / 10;
				int c = p % 2 == 1 ? 204 : 255;

				cv::Vec4b colorPx = cv::Vec4b(c, c, c, 0);
				if (dColorPoint.x != -1.0 && dColorPoint.y != -1.0)
				{
					cv::Vec4b *colorImagePtr = colorImage.ptr<cv::Vec4b>((int)dColorPoint.y);
					colorPx = colorImagePtr[(int)dColorPoint.x];
				}
				colorMappedPtr[x][i] = colorPx[i];
			}
		}
	}*/



	//wColorIO(wColorIO::PRINT_INFO, L"Mat created.\n");

	std::vector<cv::Point> tipPos = det.getTipData(rawDepthImage.clone(), colorMappedToDepth.clone());

	//wColorIO(wColorIO::PRINT_SUCCESS, L"Tipdata catched\n");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tip_cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cv::imshow("colorMapped(" + std::to_string(cameraNum) + ")", det.colorMarked);
	if (tipPos[0] == cv::Point(-1, -1))
		return;

	for (int i = 0; i < tipPos.size(); i++)
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

			//単位変換：mm→m
			point.x = cDepthPoint.x / CLOUD_SCALE;
			point.y = cDepthPoint.y / CLOUD_SCALE;
			point.z = cDepthPoint.z / CLOUD_SCALE;
			point.r = 255;
			point.g = 0;
			point.b = 0;

			tip_cloud_temp->points.push_back(point);
		}
		/*cv::circle(colorMapped, tipPos[i], 5, CV_RGB(0, 255, 0), 2, 8, 0);
		cv::line(colorMapped, *(defectArray[i].start), *(defectArray[i].depth_point), CV_RGB(255, 255, 0), 1, CV_AA, 0);
		cvCircle(src, *(defectArray[i].depth_point), 5, CV_RGB(0, 0, 255), 2, 8, 0);
		cvLine(src, *(defectArray[i].depth_point), *(defectArray[i].end), CV_RGB(0, 255, 255), 1, CV_AA, 0);
		cvDrawContours(src, defects, CV_RGB(0, 0, 0), CV_RGB(255, 0, 0), -1, CV_FILLED, 8);*/

	}

	tip_point_cloud_ptr = tip_cloud_temp;
}

void RealSenseUpdater::changeThreshold(bool isIncr)
{
	const double changeStep = 0.05;
	if (isIncr)
		farThreshold += changeStep;
	else
		farThreshold -= changeStep;
}

void RealSenseUpdater::showFPS(std::chrono::system_clock::time_point time1, std::chrono::system_clock::time_point time2)
{




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

	flip(colorImage, tmp, 1); // 反転
	cv::imwrite(directory + "-Color" + name + ".tif", tmp); // color画像保存
	flip(rawDepthImage, tmp, 1); // 反転
	imwrite(directory + "-Depth見る用" + name + ".tif", tmp * 0x60 / 0x100); // depth見る用画像保存
	writeDepth(directory + "-Depth" + name); // depth画像保存
#ifdef __ENABLE_HAND_TRACKING__
	if (enableHandTracking)
	{
		//if (isCloudArrived[CLOUD_HAND])
		if (pointCloudNum[CLOUD_HAND] != 0)
		{
			pcl::io::savePCDFileBinary(directory + "-PCLHand" + name + ".pcd", *hand_point_cloud_ptr);
			PointCloud2Mesh(hand_point_cloud_ptr, directory + "-PCLHand" + name + "s.obj", param, true);
		}
		//if (isCloudArrived[CLOUD_JOINT])
		if (pointCloudNum[CLOUD_JOINT] != 0)
			pcl::io::savePCDFileBinary(directory + "-PCLJoint" + name + ".pcd", *hand_joint_cloud_ptr);
		flip(handImage, tmp, 1); // 反転
		cv::imwrite(directory + "-HandImage" + name + ".tif", tmp); // handImage画像保存
		flip(handPoint, tmp, 1); // 反転
		cv::imwrite(directory + "-HandPoint" + name + ".tif", tmp); // handPoint画像保存
	}
#endif
	//if (isCloudArrived[CLOUD_CAMERA])
	if (camera_point_cloud_ptr->size() != 0)
		pcl::io::savePCDFileBinary(directory + "-PCLCamera" + name + ".pcd", *camera_point_cloud_ptr);
	//if (isCloudArrived[CLOUD_NEAR])
	if (near_point_cloud_ptr->size() != 0)
		pcl::io::savePCDFileBinary(directory + "-PCLNear" + name + ".pcd", *near_point_cloud_ptr);
	if (tip_point_cloud_ptr->size() != 0)
		pcl::io::savePCDFileBinary(directory + "-PCLTip" + name + ".pcd", *tip_point_cloud_ptr);


	tmp = readDepth(directory + "-Depth" + name) * 0x60 / 0x10000;

	cv::imshow("保存済み", tmp); // 保存したものの表示
	return true;
}

void RealSenseUpdater::setEnableHandTracking(bool _enableHandTracking)
{
	enableHandTracking = _enableHandTracking;
}

void RealSenseUpdater::setCamera(int numCam)
{
	// ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓HandsViewerより↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
	// 仕組みはよくわからない
	// カメラの切り替え
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
		//こっちでもいいかもsenseManager->QueryCaptureManager()->FilterByDeviceInfo(checkedDeviceInfo->name, checkedDeviceInfo->did, checkedDeviceInfo->didx);
	}
	// ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑HandsViewerより↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

	// ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
	// https://software.intel.com/en-us/forums/realsense/topic/561008
	//{
		//PXCSenseManager * psm = PXCSenseManager::CreateInstance();
		//PXCCapture::DeviceInfo dinfo;
		//dinfo.didx = numCam;
		//psm->QueryCaptureManager()->FilterByDeviceInfo(&dinfo);
		//psm->Release();
	//}
	// ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
	//
	// ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
	// https://software.intel.com/sites/landingpage/realsense/camera-sdk/v1.1/documentation/html/doc_essential_selecting_the_r200_enhanced_device.htmls
	//{
		//PXCCapture::DeviceInfo dinfo = {};
		//dinfo.model = PXCCapture::DEVICE_MODEL_SR300;
		//senseManager->QueryCaptureManager()->FilterByDeviceInfo(&dinfo);
	//}
	// ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
}

Status RealSenseUpdater::setLaserPower(int num)
{
	return pp->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower(num);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RealSenseUpdater::updatePointCloud(bool isHandDataArrived)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr near_point_cloud_ptr_temp(new pcl::PointCloud<pcl::PointXYZRGB>);

	/*if (isHandDataArrived)
		pointCloudNum[CLOUD_HAND] = 0;
	else
	{
		pointCloudNum[CLOUD_CAMERA] = 0;
		pointCloudNum[CLOUD_NEAR] = 0;
	}*/

	for (int y = 0; y < rawDepthImage.rows; y++)
	{
		//unsigned char *grayHandImagePtr = grayHandImage.ptr<uchar>(y);
		float *rawDepthImagePtr = rawDepthImage.ptr<float>(y);
		cv::Vec3b *colorMappedToDepthPtr = colorMappedToDepth.ptr<cv::Vec3b>(y);
		//cv::Vec4b *handImagePtr = handImage.ptr<cv::Vec4b>(y);
		//unsigned char *rawDepthDiffImagePtr = rawDepthDiffImage.ptr<uchar>(y);
		//unsigned char *rawDepthDiffImageFilterdPtr = rawDepthDiffImageFilterd.ptr<uchar>(y);

		for (int x = 0; x < rawDepthImage.cols; x++)
		{
			if (rawDepthImagePrev.total() <= 10)
				continue;
			float *rawDepthImagePrevPtr = rawDepthImagePrev.ptr<float>(y);
			if (rawDepthImagePtr[x] != 0)// && !(grayHandImagePtr[x] != 255 && isHandDataArrived)
			{
				//PXCPointF32 dColorPoint;
				PXCPoint3DF32 dDepthPoint;//Unit:mm
				PXCPoint3DF32 cDepthPoint;
				pcl::PointXYZRGB point;//Unit:m
				//bool isSkip = false;

				dDepthPoint.x = x;
				dDepthPoint.y = y;
				dDepthPoint.z = rawDepthImagePtr[x];

				//projection->MapDepthToColor(1, &dDepthPoint, &dColorPoint);
				projection->ProjectDepthToCamera(1, &dDepthPoint, &cDepthPoint);

				//単位変換：mm→m
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

				/*if (dColorPoint.x != -1.0 && dColorPoint.y != -1.0)
				{
					cv::Vec4b *colorImagePtr = colorImage.ptr<cv::Vec4b>((int)dColorPoint.y);

					cv::Vec4b colorPx = colorImagePtr[(int)dColorPoint.x];

					//if (colorPx[0] == 255 && colorPx[1] == 255 && colorPx[2] == 255)
					//{
					//	isSkip = true;
					//}

					//if (isHandDataArrived)
					//	handImagePtr[x] = colorPx;
					point.r = colorPx[2];
					point.g = colorPx[1];
					point.b = colorPx[0];

				}
				else
				{
					point.r = 255;
					point.g = 255;
					point.b = 255;
				}*/

				//point.a = (2 - cloudAlphaCh) * 127;
				//point.a = 0;

				/*if (abs(rawDepthImagePtr[x] - rawDepthImagePrevPtr[x]) > EXCLUDE_THRESHOLD)
				{
				point.r = (rawDepthDiffImagePtr[x] + point.r) >= 255 ? 255 : rawDepthDiffImagePtr[x] + point.r;
				//point.r += rawDepthDiffImagePtr[x];
				//point.g = 0;
				//point.b = 0;
				point.b = (rawDepthDiffImageFilterdPtr[x] + point.b) >= 255 ? 255 : rawDepthDiffImageFilterdPtr[x] + point.b;
				}
				if (rawDepthDiffImagePtr[x])
				{
				point.r = 255;
				point.g = 0;
				point.b = 0;
				}*/
				/*if (rawDepthDiffImageFilterdPtr[x] == 255)
				{
					point.r = 255;
				}
				if (rawDepthDiffImagePtr[x] == 128)
				{
					point.b = 255;
				}*/

				/*if (!isSkip)
				{*/
				point_cloud_ptr->points.push_back(point);
				if (point.z < farThreshold&&point.z>nearThreshold)
				{
					near_point_cloud_ptr_temp->points.push_back(point);
					//pointCloudNum[CLOUD_NEAR]++;
				}
				//}
				else
					continue;
				/*if (isHandDataArrived)
					pointCloudNum[CLOUD_HAND]++;
				else
					pointCloudNum[CLOUD_CAMERA]++;*/
			}
		}
	}

	near_point_cloud_ptr = near_point_cloud_ptr_temp;

	return(point_cloud_ptr);
		}

#ifdef __ENABLE_HAND_TRACKING__
void RealSenseUpdater::releaseHandImage(void)
{
	handImage = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC4);
	cv::imshow(windowName[0], handImage);
}
#endif

/*void RealSenseUpdater::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.keyDown())
	{
		int key = event.getKeyCode();
		if (key != 0)
			keyboardCallBackSettings(key);
		else
		{
			std::string str = event.getKeySym();
			int key;
			if (str == "Right")
				key = CV_WAITKEY_CURSORKEY_RIGHT;
			else if (str == "Left")
				key = CV_WAITKEY_CURSORKEY_LEFT;
			else if (str == "Up")
				key = CV_WAITKEY_CURSORKEY_TOP;
			else if (str == "Down")
				key = CV_WAITKEY_CURSORKEY_BOTTOM;
			keyboardCallBackSettings(key);
		}
	}
}*/

int RealSenseUpdater::countMat(cv::Mat mat, cv::Vec4b elm)
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
}

void RealSenseUpdater::writeDepth(const std::string name)
{
	cv::Mat matDepth = rawDepthImage;
	std::fstream fs;

	std::string dir = name;

	// 拡張子がついているか
	std::string suffix = "." + extension;
	if (dir.size() < suffix.size() || dir.find(suffix, dir.size() - suffix.size()) == std::string::npos)
		dir += suffix;

	fs.open(dir, std::ios::out | std::ios::binary);

	if (!fs.is_open())
		throw std::runtime_error("depth32f画像の書き込みファイルの呼び出しに失敗");

	if (enableMirror != isSaveMirror) // ミラーするか
	{
		cv::flip(matDepth.clone(), matDepth, 1); // 反転
	}

	dptHeader header;
	header.width = DEPTH_WIDTH;
	header.height = DEPTH_HEIGHT;
	header.type = matDepth.type();
	if (isSaveMirror) // ミラーしているか
	{
		header.data1 |= 1 << d1_mirror; // ヘッダにミラー情報書き込み
	}

	if (!fs.write((const char*)(&header), sizeof(header)) // ヘッダ書き込み
		|| !fs.seekp(header.size) // ポインタ位置移動
		|| !fs.write((const char*)(matDepth.data), matDepth.total() * matDepth.elemSize())) // データ書き込み
		throw std::runtime_error("depth32f画像の書き込みに失敗");

	fs.close();
}


cv::Mat RealSenseUpdater::readDepth(const std::string name)
{
	std::fstream fs;

	std::string dir = name;

	// 拡張子がついているか
	std::string suffix = "." + extension;
	if (dir.size() < suffix.size() || dir.find(suffix, dir.size() - suffix.size()) == std::string::npos)
		dir += suffix;

	fs.open(dir, std::ios::in | std::ios::binary);
	if (!fs.is_open())
		throw std::runtime_error("depth32f画像の読み込みファイルの呼び出しに失敗");

	dptHeader header;

	if (!fs.read((char*)(&header.size), sizeof(header.size))
		|| !fs.read((char*)(&header.identifier), sizeof(header) - sizeof(header.size))
		|| !fs.seekg(header.size))
		throw std::runtime_error("depth32f画像のヘッダの読み込みに失敗");

	cv::Mat img(header.height, header.width, header.type);

	if (!fs.read((char*)(img.data), img.total() * img.elemSize()))
		throw std::runtime_error("depth32f画像のデータの読み込みに失敗");

	fs.close();

	if (enableMirror != (header.data1 & 1 << d1_mirror)) // ミラーするか
	{
		cv::flip(img.clone(), img, 1); // 反転
	}

	return img.clone();
}
/*
std::string RealSenseUpdater::makeNameFolder(int hrgn)
{
	std::string nameFolder = "ｱ";
	nameFolder[0] = 'ｱ' + hrgn;
	return nameFolder;
}

std::string RealSenseUpdater::makeNameFail(int hrgn, int num)
{
	std::string nameFail = "ｱ-00";
	nameFail[0] = 'ｱ' + hrgn;
	nameFail[2] = '0' + num / 10;
	nameFail[3] = '0' + num % 10;
	return nameFail;
}

cv::Mat RealSenseUpdater::drawGuide(const cv::Mat& input, int num)
{
	cv::Point senter = cv::Point(input.cols / 2, input.rows / 2);
	cv::Mat mat = cv::Mat::zeros(input.rows, input.cols, CV_8UC1) * 0xff;
	cv::circle(mat, senter, input.rows / 2 * 0.75, cv::Scalar::all(0xff), 2);
	cv::circle(mat, senter + cv::Point(input.rows / 2 * 0.75 * cos(num * CV_PI / 4), input.rows / 2 * 0.75 * sin(num * CV_PI / 4)), input.rows / 2 * 0.25, cv::Scalar::all(0xff), 2);
	cv::circle(mat, senter + cv::Point(input.rows / 2 * 0.75 * cos(num * CV_PI / 4), input.rows / 2 * 0.75 * sin(num * CV_PI / 4)), input.rows / 2 * 0.25 - 2, cv::Scalar::all(0x00), -1);
	return mat.clone();
}


// 文字出力
void RealSenseUpdater::printText(int hrgn, int num)
{
	//system("cls");

	cout << "ファイルについて" << endl;
	cout << "文字：" + makeNameFolder(hrgn) << "(" << hrgn << ")" << "  ";
	cout << "番号：" << num << "  ";
	cout << "ディレクトリ：" << makeNameFolder(hrgn) + "\\" + makeNameFail(hrgn, num) << "  ";
	cout << endl;
	cout << "操作方法" << endl;
	cout << "文字：+1…w  -1…s" << endl;
	cout << "数字：+1…d  -1…a" << endl;
	cout << "保存：スペースキー(自動で次の文字に移行)" << endl;
	cout << "終了：q" << endl;
	cout << endl;
	cout << "・ピンクに手を重ねて撮る" << endl;
	cout << "・なるべく手が黄色い範囲で撮る" << endl;
	cout << "・白い枠内で撮る" << endl;
	cout << endl;
}

void RealSenseUpdater::shorGuideImage(const cv::Mat depth, int num)
{
	const int T = 70;
	const int B = 70;
	const int L = 50;
	const int R = 0;

	cv::vector<cv::Mat> colors(3);
	cv::Mat image;

	// 範囲指定処理
	colors[0] = depth.clone();
	colors[1] = depth.clone();
	colors[2] = depth.clone();
	cv::Mat tmp;
	threshold(depth, tmp, distMax, 0, CV_THRESH_TOZERO_INV);
	colors[1] = tmp.clone();
	colors[2] = tmp.clone();
	threshold(depth, tmp, distMin, 0, CV_THRESH_TOZERO);
	colors[1] &= tmp.clone();
	threshold(depth, tmp, distMax, 0, CV_THRESH_TOZERO);
	colors[0] = tmp.clone();
	merge(colors, image);
	image.clone().convertTo(image, CV_16UC3); // ビット演算用に変換

	cv::Mat guide(depth.rows, depth.cols, CV_8UC3, cv::Scalar::all(0)); // ガイド用行列
	const int x = (guide.cols - R - L - sizeLine) / 3, y = (guide.rows - T - B - sizeLine) / 3; // 計算用の定数
	cv::Point p(x * (2 - num % 3) + L + sizeLine / 2, y * ((num / 3) % 3) + T + sizeLine / 2); // 文字に応じての位置の計算
	circle(guide, p + cv::Point(x / 2, y / 2), 10, cv::Scalar(0xff, 0x00, 0xff), -1, CV_AA); // 丸
	rectangle(guide, cv::Point(L, T), cv::Point(guide.cols - R, guide.rows - B), cv::Scalar(0xff, 0xff, 0xff), sizeLine); // 映る範囲
	guide.clone().convertTo(guide, CV_16UC3); // 16bitに型を変換
	guide |= guide * (1 << 8); // 16bitに値を変換

	image = image.clone() ^ guide.clone(); // ガイドと画像の合成

	image.clone().convertTo(image, CV_32FC3); // カラー表示用に変換
	imshow("カメラ", image * 0x60 / 0x10000); // 表示
}

// http://rinov.sakura.ne.jp/wp/cpp-date
// 時刻取得
std::string RealSenseUpdater::getTime(void)
{
	//現在日時を取得する
	time_t t = time(nullptr);

	//形式を変換する
	const tm* lt = localtime(&t);

	//sに独自フォーマットになるように連結していく
	std::stringstream s;
	s << "20";
	s << lt->tm_year - 100; // 100を引くことで20xxのxxの部分になる
	s << "年";
	s << lt->tm_mon + 1; // 月を0からカウントしているため
	s << "月";
	s << lt->tm_mday; // そのまま
	s << "日";
	s << lt->tm_hour; // 時間
	s << "時";
	s << lt->tm_min; // 分
	s << "分";

	return s.str();
}
*/

void RealSenseUpdater::realsenseHandStatus(PXCHandData *handAnalyzer)
{
	PXCHandData::AlertData alertData;
	//wColorIO(wColorIO::PRINT_INFO, L"RSU>");
	for (int i = 0; i < handAnalyzer->QueryFiredAlertsNumber(); ++i)
	{
		pxcStatus sts = handAnalyzer->QueryFiredAlertData(i, alertData);
		if (sts == PXC_STATUS_NO_ERROR)
		{
			switch (alertData.label)
			{
			case PXCHandData::ALERT_HAND_DETECTED:
			{
				wColorIO(wColorIO::PRINT_SUCCESS, L"Hand Detected.\n");
				break;
			}
			case PXCHandData::ALERT_HAND_NOT_DETECTED:
			{
				wColorIO(wColorIO::PRINT_ERROR, L"Hand Not Detected.\n");
				break;
			}
			case PXCHandData::ALERT_HAND_CALIBRATED:
			{
				wColorIO(wColorIO::PRINT_SUCCESS, L"Hand Calibrated.\n");
				break;
			}
			case PXCHandData::ALERT_HAND_NOT_CALIBRATED:
			{
				wColorIO(wColorIO::PRINT_ERROR, L"Hand Not Calibrated.\n");
				break;
			}
			case PXCHandData::ALERT_HAND_INSIDE_BORDERS:
			{
				wColorIO(wColorIO::PRINT_ERROR, L"Hand Inside Borders.\n");
				break;
			}
			case PXCHandData::ALERT_HAND_OUT_OF_BORDERS:
			{
				wColorIO(wColorIO::PRINT_SUCCESS, L"Hand Out Of Borders.\n");
				break;
			}
			}
		}
	}
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

void RealSenseUpdater::initializeViewer(const std::string & id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloudPtr, double pointSize)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> hand_rgb(pointCloudPtr);
	viewer->addPointCloud<pcl::PointXYZRGB>(pointCloudPtr, hand_rgb, id);
	//viewer->addPointCloud(pointCloudPtr, id);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, id);
}
