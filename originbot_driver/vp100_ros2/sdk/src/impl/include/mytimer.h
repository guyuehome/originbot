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
#endif 

#if defined(_WIN32)
	//get corrent ns
	inline uint64_t getStamp(void)
	{
		FILETIME		t;
		GetSystemTimeAsFileTime(&t);		//get 100ns time (for 100ns min)
		return ((((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime)) *
			100;
	}

	//get current ms 
	inline uint64_t getMS(void)
	{
		return GetTickCount();
	}

	//dalay for some time 
	inline void delayMS(uint32_t ms)
	{
		Sleep(ms);
	}

#else 
	//get current ns 
	inline uint64_t getStamp(void)
	{
		#if 1
			struct timespec	tim;
			clock_gettime(CLOCK_REALTIME, &tim);
			return static_cast<uint64_t>(tim.tv_sec) * 1000000000LL + tim.tv_nsec;
		#else
			struct timeval timeofday;
			gettimeofday(&timeofday, NULL);
			return static_cast<uint64_t>(timeofday.tv_sec) * 1000000000LL +
			static_cast<uint64_t>(timeofday.tv_usec) * 1000LL;
		#endif
	}

	//sleep for some ms 
	inline void delayMS(uint32_t ms)
	{
		usleep(ms*1000);
	}

#endif 