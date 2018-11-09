#include "RealSenseProcessor.h"

RealSenseProcessor::RealSenseProcessor() :
	viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
	rsu(std::vector<RealSenseUpdater>(NUM)),
	regist_near(std::vector<PCL_Regist>(NUM, PCL_Regist(1e-2, 0.01, 1000, 5, 1.0e-3))),
	regist_tip(std::vector<PCL_Regist>(NUM, PCL_Regist(1e-2, 0.2, 1000, 100, 0.0))),
	transformMat(std::vector<Eigen::Matrix4f>(NUM, Eigen::Matrix4f::Identity()))
{
	wColorIO(wColorIO::PRINT_INFO, L"RSP>");
	wColorIO(wColorIO::PRINT_INFO, L"Start\n");

	viewer->registerKeyboardCallback(&RealSenseProcessor::keyboardCallback, *this);

	isContinue = false;
	isUserInterrupt = false;
	_time = getTime();

	zRev << -1.0, 0.0, 0.0, 0.0,
		0.0, -1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0;


	viewer->setBackgroundColor(0, 0, 0);
	for (auto i = 0; i < NUM; i++)
	{
		cloud_id.push_back("cloud(" + std::to_string(i) + ")");
		tip_cloud_id.push_back("tip_cloud(" + std::to_string(i) + ")");
		initializeViewer(cloud_id[i], rsu[i].hand_point_cloud_ptr, 1.0);
		initializeViewer(tip_cloud_id[i], rsu[i].tip_point_cloud_ptr, 20.0);
	}
	viewer->addCoordinateSystem(0.01);
	viewer->initCameraParameters();
}


RealSenseProcessor::~RealSenseProcessor()
{
	viewer->close();

	cv::destroyAllWindows();

	wColorIO(wColorIO::PRINT_INFO, L"RSP>");
	wColorIO(wColorIO::PRINT_SUCCESS, L"Exiting\n");
}

bool RealSenseProcessor::run(void)
{
	for (int i = 0; i < NUM; i++)
	{
		/*wColorIO(wColorIO::PRINT_INFO, L"RSP>");
		wColorIO(wColorIO::PRINT_SUCCESS, L"Prosessing #%d device.\n",i);*/

		//rsu[i].setEnableHandTracking(false);

		if (rsu[i].init(i) < RealSenseUpdater::RSU_NO_ERROR)
		{
			return setReInit();
		}
	}

	wColorIO(wColorIO::PRINT_INFO, L"RSP>");
	wColorIO(wColorIO::PRINT_SUCCESS, L"init finish\n");

	while (1)
	{
		for (int i = 0; i < NUM; i++)
		{
			//wColorIO(wColorIO::PRINT_INFO, L"RSP>");
			//wColorIO(wColorIO::PRINT_SUCCESS, L"Prosessing #%d device.\n", i);
			rsu[i].setLaserPower(POWER);
			keyboardCallBackSettings(cv::waitKey(TIME_STANDBY / 2));
			int callback = rsu[i].run();
			if (callback < RealSenseUpdater::RSU_NO_ERROR)
			{
				wprintf_s(L"%d\n", callback);
				return setReInit();
			}
			else if (callback != RealSenseUpdater::RSU_NO_ERROR)
			{
				keyboardCallBackSettings(callback);
			}
			//rsu[i].showFPS();

			//printf_s("\n%d\n",rsu[i].run());
			if (NUM > 1)
				rsu[i].setLaserPower(0);

			updateViewerText();
			viewer->updatePointCloud(regist_tip[i].transformPointcloud(rsu[i].hand_point_cloud_ptr), cloud_id[i]);
			viewer->updatePointCloud(regist_tip[i].transformPointcloud(rsu[i].tip_point_cloud_ptr), tip_cloud_id[i]);
			viewer->spinOnce();
			keyboardCallBackSettings(cv::waitKey(TIME_STANDBY / 2));

			if (_kbhit())
			{ // Break loop
				char c = _getch() & 255;
				if (c == 27 || c == 'q' || c == 'Q')
				{
					isUserInterrupt = true;
					//c = 0;
					return true;
				} // ESC|q|Q for Exit
			}

		}
	}
	return true;
}

bool RealSenseProcessor::keyboardCallBackSettings(int key)
{
	cv::Mat tmp;

	switch (key)
	{
	case CV_WAITKEY_CURSORKEY_TOP:num++;	break; // 数字 + 1
	case CV_WAITKEY_CURSORKEY_BOTTOM:num--;	break; // 数字 - 1
	case CV_WAITKEY_CURSORKEY_RIGHT:hrgn++;	break; // 文字 + 1
	case CV_WAITKEY_CURSORKEY_LEFT:hrgn--;	break; // 文字 - 1
	case ' ': // 保存
		CreateDirectory(dataFolderName.c_str(), NULL);
		CreateDirectory((dataFolderName + "\\" + _time).c_str(), NULL); // 大本のフォルダ作成（名前が時間）
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Color").c_str(), NULL); // color画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth").c_str(), NULL); // depth画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandImage").c_str(), NULL); // HandImage画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandPoint").c_str(), NULL); // HandPoint画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLHand").c_str(), NULL); // PCLHand画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLJoint").c_str(), NULL); // PCLJoint画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLCamera").c_str(), NULL); // PCLCamera画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLNear").c_str(), NULL); // PCLNear画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth見る用").c_str(), NULL); // depth見る用画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLTip").c_str(), NULL); // depth見る用画像フォルダ作成
		for (int i = 0; i < NUM; i++)
		{
			rsu[i].saveData(dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn), "\\" + makeNameFail(hrgn, num) + "(" + std::to_string(i) + ")");

		}
		num++;
		break;
	case 'q': // 終了
		//viewer->close();
		cv::destroyAllWindows();
		isUserInterrupt = true;

		return false;
	case '+':
		for (int i = 0; i < NUM; i++)
		{
			rsu[i].changeThreshold(true);
		}
		break;
	case '-':
		for (int i = 0; i < NUM; i++)
		{
			rsu[i].changeThreshold(false);
		}
		break;
	case 't':
		for (int i = 1; i < NUM; i++)
		{
			if (rsu[0].tip_point_cloud_ptr->size() >= 5 && rsu[i].tip_point_cloud_ptr->size() >= 5)
			{
				transformMat[i] = transformMat[i] * regist_tip[i].getTransformMatrix(rsu[0].tip_point_cloud_ptr, rsu[i].tip_point_cloud_ptr, Eigen::Matrix4f::Identity());//, transformMat[i]
				transformMat[i] = transformMat[i] * regist_near[i].getTransformMatrix(rsu[0].hand_point_cloud_ptr, rsu[i].hand_point_cloud_ptr, transformMat[i]);
			}
			else
			{
				if (rsu[0].tip_point_cloud_ptr->size() < 5)
					wColorIO(wColorIO::PRINT_ERROR, L"The number of points #0 is too small!\n");
				if (rsu[i].tip_point_cloud_ptr->size() < 5)
					wColorIO(wColorIO::PRINT_ERROR, L"The number of points #%d is too small!\n", i);
			}
		}
		break;
	default:
		/*if (key != -1)
		wColorIO(wColorIO::PRINT_VALUE, L"%d\n", key);*/
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
}

inline void RealSenseProcessor::initializeViewer(const std::string & id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloudPtr, double pointSize)
{
	viewer->addPointCloud(pointCloudPtr, id);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, id);
}

bool RealSenseProcessor::setReInit(void)
{
	wprintf_s(L"Stream has been stoped due to error(s)\n");
	wprintf_s(L"Do you want to re-initilizing?\nY/N\n");

	while (true)
	{
		int c = _getch();
		if (c == 'Y' || c == 'y')
		{
			return false;
		}
		else if (c == 'N' || c == 'n' || c == 27)
		{
			return true;
		}
	}
}

Eigen::Matrix4f RealSenseProcessor::getPresetTransMat(float radius, float theta)
//radius[mm]
//theta[deg]
{
	Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();//(double)std::cos(theta*2*std::_Pi/360.0),0, (double)std::cos(theta * 2 * std::_Pi / 360.0),

	return mat;
}

void RealSenseProcessor::updateViewerText(void)
{
	std::vector<boost::format> entries;
	for (int i = NUM - 1; i >= 0; i--)
	{
		entries.push_back(boost::format("Cam #%i FPS:%i Num of tip:%i") % i % rsu[i].fps % (int)(rsu[i].tip_point_cloud_ptr->size()));
	}

	const int dx = 5;
	const int dy = 14;
	const int fs = 10;
	boost::format name_fmt("text%i");

	//std::vector<boost::format> entries = app.viewerTextUpdate();

	for (size_t i = 0; i < entries.size(); ++i)
	{
		std::string name = boost::str(name_fmt % i);
		std::string entry = boost::str(entries[i]);
		if (!viewer->updateText(entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name))
			viewer->addText(entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name);
	}
}

void RealSenseProcessor::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
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
}

inline std::string RealSenseProcessor::makeNameFolder(int hrgn)
{
	std::string nameFolder = "ｱ";
	nameFolder[0] = 'ｱ' + hrgn;
	return nameFolder;
}

inline std::string RealSenseProcessor::makeNameFail(int hrgn, int num)
{
	std::string nameFail = "ｱ-00";
	nameFail[0] = 'ｱ' + hrgn;
	nameFail[2] = '0' + num / 10;
	nameFail[3] = '0' + num % 10;
	return nameFail;
}

// 文字出力
inline void RealSenseProcessor::printText(int hrgn, int num)
{
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

// http://rinov.sakura.ne.jp/wp/cpp-date
// 時刻取得
inline std::string RealSenseProcessor::getTime(void)
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