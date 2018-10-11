#include "RealSenseProcessor.h"

RealSenseProcessor::RealSenseProcessor() :
	viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
	rsu(std::vector<RealSenseUpdater>(NUM)),
	regist_near(std::vector<PCL_Regist>(NUM, PCL_Regist(1e-2, 0.02, 1000, 2))),
	regist_tip(std::vector<PCL_Regist>(NUM, PCL_Regist(1e-2, 0.2, 1000, 100))),
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
		initializeViewer(cloud_id[i], rsu[i].camera_point_cloud_ptr, 1.0);
		initializeViewer(tip_cloud_id[i], rsu[i].tip_point_cloud_ptr, 20.0);
	}
	viewer->addCoordinateSystem(0.01);
	viewer->initCameraParameters();
}


RealSenseProcessor::~RealSenseProcessor()
{
	/*for (int i = 0; i < NUM; i++)
	{

	}*/

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

		rsu[i].setEnableHandTracking(false);

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
			/*wColorIO(wColorIO::PRINT_INFO, L"RSP>");
			wColorIO(wColorIO::PRINT_SUCCESS, L"Prosessing #%d device.\n", i);*/
			rsu[i].setLaserPower(POWER);
			keyboardCallBackSettings(cv::waitKey(TIME_STANDBY / 2));
			int callback = rsu[i].run();
			if (callback < RealSenseUpdater::RSU_NO_ERROR)
			{
				return setReInit();
			}
			else if (callback != RealSenseUpdater::RSU_NO_ERROR)
			{
				keyboardCallBackSettings(callback);
			}
			rsu[i].showFPS();

			//printf_s("\n%d\n",rsu[i].run());
			if (NUM > 1)
				rsu[i].setLaserPower(0);

			//if (i != 0)
				//transformMat[i] = transformMat[i] * regist[i].getTransformMatrix(rsu[0].camera_point_cloud_ptr, rsu[i].camera_point_cloud_ptr);

			viewer->updatePointCloud(regist_tip[i].transformPointcloud(rsu[i].camera_point_cloud_ptr), cloud_id[i]);
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
	case CV_WAITKEY_CURSORKEY_TOP:num++;	break; // ���� + 1
	case CV_WAITKEY_CURSORKEY_BOTTOM:num--;	break; // ���� - 1
	case CV_WAITKEY_CURSORKEY_RIGHT:hrgn++;	break; // ���� + 1
	case CV_WAITKEY_CURSORKEY_LEFT:hrgn--;	break; // ���� - 1
	case ' ': // �ۑ�
		CreateDirectory(dataFolderName.c_str(), NULL);
		CreateDirectory((dataFolderName + "\\" + _time).c_str(), NULL); // ��{�̃t�H���_�쐬�i���O�����ԁj
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Color").c_str(), NULL); // color�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth").c_str(), NULL); // depth�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandImage").c_str(), NULL); // HandImage�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandPoint").c_str(), NULL); // HandPoint�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLHand").c_str(), NULL); // PCLHand�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLJoint").c_str(), NULL); // PCLJoint�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLCamera").c_str(), NULL); // PCLCamera�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLNear").c_str(), NULL); // PCLNear�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth����p").c_str(), NULL); // depth����p�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLTip").c_str(), NULL); // depth����p�摜�t�H���_�쐬
		for (int i = 0; i < NUM; i++)
		{
			rsu[i].saveData(dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn), "\\" + makeNameFail(hrgn, num) + "(" + std::to_string(i) + ")");

		}
		num++;
		break;
	case 'q': // �I��
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
				transformMat[i] = transformMat[i] * regist_near[i].getTransformMatrix(rsu[0].near_point_cloud_ptr, rsu[i].near_point_cloud_ptr, transformMat[i]);
			}
			else
			{
				if (rsu[0].tip_point_cloud_ptr->size() >= 5)
					wColorIO(wColorIO::PRINT_ERROR, L"The number of points #0 is too small!\n");
				if (rsu[i].tip_point_cloud_ptr->size() >= 5)
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
	else if ('�' - '�' < hrgn)
	{
		num = numMax - 1;
		hrgn = '�' - '�';
	}
	printText(hrgn, num);
	// �{�^���������ꂽ�Ƃ��̂ݕ\��
	return true;
}

void RealSenseProcessor::initializeViewer(const std::string & id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloudPtr, double pointSize)
{
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> hand_rgb(pointCloudPtr);
	//viewer->addPointCloud<pcl::PointXYZRGBA>(pointCloudPtr, hand_rgb, id);
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

void RealSenseProcessor::updateViewerText(void)
{
	std::vector<boost::format> entries;

	const int dx = 5;
	const int dy = 14;
	const int fs = 10;
	boost::format name_fmt("text%i");

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

std::string RealSenseProcessor::makeNameFolder(int hrgn)
{
	std::string nameFolder = "�";
	nameFolder[0] = '�' + hrgn;
	return nameFolder;
}

std::string RealSenseProcessor::makeNameFail(int hrgn, int num)
{
	std::string nameFail = "�-00";
	nameFail[0] = '�' + hrgn;
	nameFail[2] = '0' + num / 10;
	nameFail[3] = '0' + num % 10;
	return nameFail;
}

// �����o��
void RealSenseProcessor::printText(int hrgn, int num)
{
	//system("cls");

	cout << "�t�@�C���ɂ���" << endl;
	cout << "�����F" + makeNameFolder(hrgn) << "(" << hrgn << ")" << "  ";
	cout << "�ԍ��F" << num << "  ";
	cout << "�f�B���N�g���F" << makeNameFolder(hrgn) + "\\" + makeNameFail(hrgn, num) << "  ";
	cout << endl;
	cout << "������@" << endl;
	cout << "�����F+1�cw  -1�cs" << endl;
	cout << "�����F+1�cd  -1�ca" << endl;
	cout << "�ۑ��F�X�y�[�X�L�[(�����Ŏ��̕����Ɉڍs)" << endl;
	cout << "�I���Fq" << endl;
	cout << endl;
	cout << "�E�s���N�Ɏ���d�˂ĎB��" << endl;
	cout << "�E�Ȃ�ׂ��肪���F���͈͂ŎB��" << endl;
	cout << "�E�����g���ŎB��" << endl;
	cout << endl;
}

// http://rinov.sakura.ne.jp/wp/cpp-date
// �����擾
std::string RealSenseProcessor::getTime(void)
{
	//���ݓ������擾����
	time_t t = time(nullptr);

	//�`����ϊ�����    
	const tm* lt = localtime(&t);

	//s�ɓƎ��t�H�[�}�b�g�ɂȂ�悤�ɘA�����Ă���
	std::stringstream s;
	s << "20";
	s << lt->tm_year - 100; // 100���������Ƃ�20xx��xx�̕����ɂȂ�
	s << "�N";
	s << lt->tm_mon + 1; // ����0����J�E���g���Ă��邽��
	s << "��";
	s << lt->tm_mday; // ���̂܂�
	s << "��";
	s << lt->tm_hour; // ����
	s << "��";
	s << lt->tm_min; // ��
	s << "��";

	return s.str();
}