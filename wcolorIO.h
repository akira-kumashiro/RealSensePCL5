#pragma once
#include <stdarg.h>
#include <stdio.h>
#include <Windows.h>

#ifndef COLOR_IO_INCLUDED
#define COLOR_IO_INCLUDED

//     色定義
/*
#define     COL_BLACK   0x00
#define     COL_DARK_BLUE       0x01
#define     COL_DARK_GREEN 0x02
#define     COL_DARK_CYAN       0x03
#define     COL_DARK_RED     0x04
#define     COL_DARK_VIOLET0x05
#define     COL_DARK_YELLOW   0x06
#define     COL_GRAY 0x07
#define     COL_LIGHT_GRAY      0x08
#define     COL_BLUE     0x09
#define     COL_GREEN   0x0a
#define     COL_CYAN     0x0b
#define     COL_RED      0x0c
#define     COL_VIOLET  0x0d
#define     COL_YELLOW 0x0e
#define     COL_WHITE   0x0f
#define     COL_INTENSITY     0x08     // 高輝度マスク
#define     COL_RED_MASK     0x04     // 赤色ビット
#define     COL_GREEN_MASK 0x02     // 緑色ビット
#define     COL_BLUE_MASK   0x01     //  青色ビット
*/
/*
#define PRINT_INFO 0
#define PRINT_VALUE 1
#define PRINT_ERROR 2
#define PRINT_SUCCESS 3
#define PRINT_HIGHLIGHT 4
#define PRINT_WARN 5
*/
#define WCIO_INFO(...) wColorIO(wColorIO::PRINT_INFO,...)
#define WCIO_VALUE(...) wColorIO(wColorIO::PRINT_VALUE,...)
#define WCIO_ERROR(...) wColorIO(wColorIO::PRINT_ERROR,...)
#define WCIO_SUCCESS(...) wColorIO(wColorIO::PRINT_SUCCESS,...)
#define WCIO_HIGHLIGHT(...) wColorIO(wColorIO::PRINT_HIGHLIGHT,...)
#define WCIO_WARN(...) wColorIO(wColorIO::PRINT_WARN,...)

#endif // !COLOR_IO_INCLUDED

#define WCHAR_LENGTH 2048



class wColorIO
{
public:
	wColorIO(int prtintType, const wchar_t* format, ...);
	~wColorIO();
	static void setPrintColor(int fg = COL_GRAY, int bg = COL_BLACK);

	static enum PRINT_TYPE
	{
		PRINT_INFO,
		PRINT_VALUE,
		PRINT_ERROR,
		PRINT_SUCCESS,
		PRINT_HIGHLIGHT,
		PRINT_WARN,
	};

	static enum COLOR_DEFINE
	{
		COL_BLACK,
		COL_DARK_BLUE,
		COL_DARK_GREEN,
		COL_DARK_CYAN,
		COL_DARK_RED,
		COL_DARK_VIOLET,
		COL_DARK_YELLOW,
		COL_GRAY,
		COL_LIGHT_GRAY,
		COL_BLUE,
		COL_GREEN,
		COL_CYAN,
		COL_RED,
		COL_VIOLET,
		COL_YELLOW,
		COL_WHITE,
		COL_INTENSITY = 8,
		COL_RED_MASK = 4,
		COL_GREEN_MASK = 2,
		COL_BLUE_MASK = 1,
	};

private:	
	static void outputColorStr(void);
	static wchar_t* _string;
	static int _printType;
};