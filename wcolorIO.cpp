#include "wcolorIO.h"

wchar_t* wColorIO::_string;
int wColorIO::_printType;

wColorIO::wColorIO(int printType, const wchar_t* format, ...)
{
	_string = new wchar_t[WCHAR_LENGTH];
	va_list ap;

	_printType = printType;

	va_start(ap, format);
	if (vswprintf(_string, format, ap) < 0)
	{
		_string = L"ConsoleOutputError\n";
		_printType = PRINT_ERROR;
	}
	va_end(ap);

	outputColorStr();
}

void wColorIO::setPrintColor(int fg, int bg)
{
	HANDLE hCons = GetStdHandle(STD_OUTPUT_HANDLE);
	WORD attr = 0;
	if (fg & COL_INTENSITY)
		attr |= FOREGROUND_INTENSITY;
	if (fg & COL_RED_MASK)
		attr |= FOREGROUND_RED;
	if (fg & COL_GREEN_MASK)
		attr |= FOREGROUND_GREEN;
	if (fg & COL_BLUE_MASK)
		attr |= FOREGROUND_BLUE;

	if (bg & COL_INTENSITY)
		attr |= BACKGROUND_INTENSITY;
	if (bg & COL_RED_MASK)
		attr |= BACKGROUND_RED;
	if (bg & COL_GREEN_MASK)
		attr |= BACKGROUND_GREEN;
	if (bg & COL_BLUE_MASK)
		attr |= BACKGROUND_BLUE;
	SetConsoleTextAttribute(hCons, attr);
}

void wColorIO::outputColorStr(void)
{
	switch (_printType)
	{
	case PRINT_INFO:
		setPrintColor(COL_WHITE); break;
	case PRINT_VALUE:
		setPrintColor(COL_CYAN); break;
	case PRINT_ERROR:
		setPrintColor(COL_RED); break;
	case PRINT_SUCCESS:
		setPrintColor(COL_DARK_GREEN); break;
	case PRINT_HIGHLIGHT:
		setPrintColor(COL_WHITE, COL_DARK_RED); break;
	case PRINT_WARN:
		setPrintColor(COL_WHITE,COL_DARK_YELLOW); break;
	default:
		break;
	}

	wprintf_s(L"%ws", _string);
	setPrintColor();
}

wColorIO::~wColorIO()
{
	delete[WCHAR_LENGTH] _string;
}