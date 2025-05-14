#pragma once

#include "nvilidar_def.h"
#include "nvilidar_protocol.h"

//---visual studio include lib file 
#ifdef WIN32
	#define NVILIDAR_FILTER_API __declspec(dllexport)
#else
	#define NVILIDAR_FILTER_API
#endif // ifdef WIN32


namespace vp100_lidar
{
    //lidar driver 
	class  NVILIDAR_FILTER_API LidarFilter
    {
		public:
			static LidarFilter *instance();

			void LidarFilterLoadPara(FilterPara cfg);		//load fit para 
			bool LidarNoiseFilter(std::vector<Nvilidar_Node_Info> in,std::vector<Nvilidar_Node_Info> &out);
    		bool LidarTailFilter(TailFilterPara para,std::vector<Nvilidar_Node_Info> in,std::vector<Nvilidar_Node_Info> &out);
    		bool LidarSlidingFilter(SlidingFilterPara para,std::vector<Nvilidar_Node_Info> in,std::vector<Nvilidar_Node_Info> &out);

		private:
			FilterPara     lidar_filter_cfg;				//lidar filter config parameter 
			LidarFilter();		
			~LidarFilter();

			static LidarFilter *_instance;
    };
}
