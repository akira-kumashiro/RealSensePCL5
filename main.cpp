#include "RealSenseProcessor.h"
#include <iostream>
#include <Windows.h>

int wmain(int argc, WCHAR* argv[])
{
	try
	{
		RealSenseProcessor *rsp;
		rsp = new RealSenseProcessor();

		while (1)
		{
			if (rsp->run())
			{
				delete rsp;
				return false;
			}
			else
			{
				delete rsp;
				
				rsp = new RealSenseProcessor();
			}
		}
	}
	catch (std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}
}
