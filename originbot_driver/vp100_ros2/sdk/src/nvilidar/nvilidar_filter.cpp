#include "nvilidar_filter.h"
#include <list>
#include <string>
#include <iostream> 
#include <istream> 
#include <sstream>
#include <math.h>
#include <numeric>

namespace vp100_lidar
{
	LidarFilter *LidarFilter::_instance = nullptr;		//instanse to null 

	//instance
	LidarFilter *LidarFilter::instance() {
		if (!_instance){
			_instance = new LidarFilter();
		}
		return _instance;
	}

	LidarFilter::LidarFilter(){
	}

	LidarFilter::~LidarFilter(){
		_instance = nullptr;
	}

	//lidar config filter para   
	void LidarFilter::LidarFilterLoadPara(FilterPara cfg){
		lidar_filter_cfg = cfg;
	}

	//过滤
	bool LidarFilter::LidarNoiseFilter(std::vector<Nvilidar_Node_Info> in,std::vector<Nvilidar_Node_Info> &out){
		std::vector<Nvilidar_Node_Info> out_temp;

		out_temp = in;

		//shadow filter 
		if(lidar_filter_cfg.tail_filter.enable){
			LidarTailFilter(lidar_filter_cfg.tail_filter,out_temp,out_temp);
		}
		//sliding filter 
		if(lidar_filter_cfg.sliding_filter.enable){
			LidarSlidingFilter(lidar_filter_cfg.sliding_filter,out_temp,out_temp);
		}

		if( (false == lidar_filter_cfg.sliding_filter.enable) &&
			(false == lidar_filter_cfg.tail_filter.enable) ){
			out = in;
		}else{
			out = out_temp;
		}

		return true;
	}

	//trailing filter
	bool LidarFilter::LidarTailFilter(TailFilterPara para,std::vector<Nvilidar_Node_Info> in,std::vector<Nvilidar_Node_Info> &out){
		std::vector<size_t> in_index_list;
		std::vector<size_t> in_index_nocalc_list;       //Indexes not involved in the calculation
		std::vector<int> in_index_check_tail_list;   	//Trailing indexes detected
		double min_angle = para.level;
		double max_angle = 180.0 - para.level;
		double min_angle_tan_ = tan(min_angle*M_PI/180.0);
		double max_angle_tan_ = tan(max_angle*M_PI/180.0);
		//Consistency in quantity
		out = in;
		//point defense
		if(in.size() < 3){
			return false;
		}
		//Cut out everything that equals zero, and reorganize the array.
		in_index_list.clear();
		in_index_nocalc_list.clear();
		in_index_check_tail_list.clear();
		//遍历跳过的点和其它
		for(size_t i = 0; i< in.size(); i++){
			//距离为0
			if(in[i].lidar_distance == 0){
				in_index_nocalc_list.push_back(i);
				continue;
			}
			//超过该距离不做算法处理
			if((true == para.distance_limit_flag) && (in[i].lidar_distance > para.distance_limit_value)){
				in_index_nocalc_list.push_back(i);
				continue;
			}
			in_index_list.push_back(i);
		}
		//遍历不需要处理的 先赋值原始值
		for(size_t i= 0; i<in_index_nocalc_list.size(); i++){
			out[in_index_nocalc_list[i]] = in[in_index_nocalc_list[i]];
		}
		//遍历非0点数据
		for(size_t i = 0; i<in_index_list.size(); i++){
			//index < 1 无前点和后点
			if(i < 1){
				out[in_index_list[i]] = in[in_index_list[i]];
				continue;
			}
			//计算角度信息
			double r1 = in[in_index_list[i-1]].lidar_distance;
			double r2 = in[in_index_list[i]].lidar_distance;
			double a_dif = std::fabs((in[in_index_list[i]].lidar_angle - in[in_index_list[i-1]].lidar_angle)*M_PI/180.0);

			double perpendicular_y_ = r2 * sin(a_dif);
			double perpendicular_x_ = r1 - r2 * cos(a_dif);
			double perpendicular_tan_ = std::fabs(perpendicular_y_) / perpendicular_x_;

			//qDebug() << "min_angle_tan_:" << min_angle_tan_ << "max_angle_tan_:" << max_angle_tan_ << perpendicular_tan_;
			if (perpendicular_tan_ > 0){
				if (perpendicular_tan_ < min_angle_tan_){
					in_index_check_tail_list.push_back(in_index_list[i]);
				// qDebug() << "perpendicular_tan_:" << perpendicular_tan_ << out[in_index_list[i]].angle;
				}else{
					out[in_index_list[i]] = in[in_index_list[i]];
				}
			}
			else{
				if (perpendicular_tan_ > max_angle_tan_){
					in_index_check_tail_list.push_back(in_index_list[i]);
				// qDebug() << "perpendicular_tan_:" << perpendicular_tan_ << out[in_index_list[i]].angle;
				}else{
					out[in_index_list[i]] = in[in_index_list[i]];
				}
			}
		}

		//遍历 过滤中间一些点信息 (可以加过滤邻点)
		for(size_t i=0; i<in_index_check_tail_list.size(); i++){
			if(para.neighbors > 0){
				int start_index = std::max<int>(in_index_check_tail_list[i]-para.neighbors, 0);
				int stop_index = std::min<int>(in_index_check_tail_list[i]+para.neighbors, in.size() - 1);
				for(int j = start_index; j <= stop_index; j++){
					out[j].lidar_distance = 0;
					out[j].lidar_quality = 0;
				}
			}else{
				out[in_index_check_tail_list[i]].lidar_distance = 0;
				out[in_index_check_tail_list[i]].lidar_quality = 0;
			}
		}

		return false;
	}

	//滑动滤波
	bool LidarFilter::LidarSlidingFilter(SlidingFilterPara para,std::vector<Nvilidar_Node_Info> in,std::vector<Nvilidar_Node_Info> &out){
		std::vector<double>  filter_buf;      //滤波器buf
		double  filter_out;         //滤波器输出
		uint8_t filter_num;         //滑动窗口3个
		int16_t filter_error;       //滤波修正误差范围阈值

		filter_buf.resize(para.window);     //window
		out = in;

		for(size_t i = 0; i<in.size(); i++){
			double r = in[i].lidar_distance;

			if(0 != r){
				if(((r < para.max_range) && (true == para.max_range_flag)) ||
						(false == para.max_range_flag)){
					filter_num++;
					for(size_t i=0; i<para.window-1; i++){
						filter_buf[i] = filter_buf[i+1];
					}
					filter_buf[para.window-1] = r;
					filter_out = std::accumulate(filter_buf.begin(),filter_buf.end(),0)/filter_buf.size();  //滑动滤波N次

					if(r >= filter_out){
						filter_error = r - filter_out;
					}else{
						filter_error = filter_out - r; //校正绝对值
					}

					if(filter_num >= para.window){
						if(filter_error < para.jump_threshold){
							r = filter_out;//赋值
						}
						filter_num = para.window - 1;
					}
				}
				else{
					filter_num = 0;  //跳出
				}
			}

			out[i].lidar_distance = r;
		}

		return true;
	}
}







