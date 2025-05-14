#pragma once 

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <inttypes.h>
#if defined(_WIN32)
	#include <mmsystem.h>
	#include <sysinfoapi.h>
	#include <WinSock2.h>
	#include <windows.h>
#else 
	#include <unistd.h>
#endif 


namespace vp100_lidar{
	class TimeStamp{
	public:
		//get time stamp 
		static uint64_t getStamp(){
			#if defined(_WIN32)
				FILETIME		t;
				GetSystemTimeAsFileTime(&t);		//get 100ns time (for 100ns min)
				return ((((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime)) *
					100;
			#else 
				struct timespec	tim;
				clock_gettime(CLOCK_REALTIME, &tim);
				return static_cast<uint64_t>(tim.tv_sec) * 1000000000LL + tim.tv_nsec;
			#endif 
		}
		//sleep ms 
		static void sleepMS(uint32_t ms) {
			#if defined(_WIN32)
				Sleep(ms);
			#else 
				usleep(ms*1000);
			#endif 
		}
	};
}