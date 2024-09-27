#ifndef _NVILIDAR_SERIAL_H
#define _NVILIDAR_SERIAL_H

#if defined(__linux__)
	#include "serial/nvilidar_serial_list_unix.h"
	#include "serial/nvilidar_serial_unix.h"
#elif defined(_WIN32)
	#include "serial/nvilidar_serial_list_win.h"
	#include "serial/nvilidar_serial_win.h"
#endif


#endif


