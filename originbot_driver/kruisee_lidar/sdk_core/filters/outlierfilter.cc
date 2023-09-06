//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "filters/outlierfilter.h"

#include "search.h"

std::unique_ptr<OutlierFilter> OutlierFilter::Create(double search_radius, uint32_t min_neighbors, double max_distance)
{
    if (search_radius <= 0.0 || min_neighbors == 0 || max_distance <= 0)
    	return nullptr;

    std::unique_ptr<Search, Deleter<Search>> searcher(new Search(false), [] (Search *ptr) { delete ptr; });
    if (searcher == nullptr)
    	return nullptr;

    std::unique_ptr<OutlierFilter> filter(new OutlierFilter(std::move(searcher)));
    if (filter == nullptr)
    	return nullptr;

    filter->set_radius_search(search_radius);
    filter->set_min_neighbors_in_radius(min_neighbors);
    filter->set_max_distance(max_distance);

    return filter;
}

OutlierFilter::OutlierFilter(std::unique_ptr<Search, Deleter<Search>> searcher)
    : searcher_(std::move(searcher)) 
{
	search_flags_.resize(1440);
	std::fill(search_flags_.begin(), search_flags_.end(), 0);
}

bool OutlierFilter::Run(Mode mode, std::vector<Point2D> &output, std::vector<int> &removed)
{
	removed.clear();

	if (!Prepare())
		return false;

	if (input_ == &output) {  // cloud_in = cloud_out
		std::vector<Point2D> tmp;
		ApplyFilter(mode, tmp, removed);
		output.swap(tmp);
	} else {
		ApplyFilter(mode, output, removed);
	}

	return true;
}

bool OutlierFilter::Prepare()
{
	// [1] 检测输入
	if (input_ == nullptr)
		return false;

	// [2] 检测索引是否点云匹配, 不匹配这更新索引
	if (indices_.size() != input_->size()) {
		size_t size = indices_.size();

		//std::cout << "size:" << size << std::endl;

		try {
			indices_.resize(input_->size());
		} catch (const std::bad_alloc &) {
			return false;
		}

		for (size_t i = size; i < indices_.size(); ++i)
			indices_[i] = static_cast<int>(i);
	}

	return true;
}

void OutlierFilter::set_search_flags(int start, int end)
{
	for(int i=start; i<end; i++)
	{
		search_flags_[i] = 1;
	}
}

void OutlierFilter::ApplyFilter(Mode mode, std::vector<Point2D> &output, std::vector<int> &removed)
{
	std::vector<int> indices;
	ApplyFilter(mode, indices, removed);
	for (const auto &index : indices)
		output.emplace_back(input_->at(index));
}

void OutlierFilter::ApplyFilter(Mode mode, std::vector<int> &indices, std::vector<int> &removed)
{
	searcher_->SetCloud(input_);

	// The arrays to be used
	std::vector<int> nn_indices(indices_.size());
	std::vector<float> nn_dists(indices_.size());
	indices.resize(indices_.size());
	int effective = 0;

	// If the data is dense => use nearest-k search
	if (mode == Mode::Knn) {
		// Note: k includes the query point, so is always at least 1
		int mean_k = min_pts_radius_ + 1;	// 目标查询点数
		double nn_dists_max = search_radius_ * search_radius_;	// 最大搜索距离

		for (const auto &index : indices_) {
			//std::cout << "index:" << index << std::endl;

			if( (!(scan_->ranges[index] > 0)) || (scan_->ranges[index] > max_distance_) )
			{
				indices[effective++] = index;
				continue;
			}

			if(scan_->ranges[index] < min_distance_)
			{
				indices[effective++] = index;
				continue;
			}
			
			if(0 == search_flags_[index])
			{
				indices[effective++] = index;
				continue;
			}
			
			// 执行最临近K搜索算法
			int k = searcher_->NearestKSearch(index, mean_k, nn_indices, nn_dists);

			// k的取值0, mean_k(mean_k小于点云数量), 点云数量
			if (k == mean_k && nn_dists_max >= nn_dists[k - 1]) {
				indices[effective++] = index;
			} else {
				if (nn_dists[k - 1] > max_distance_)
					continue;

				removed.push_back(index);
			}
		}
	} else { // NaN or Inf values could exist => use radius search
		for (const auto &index : indices_) {
			int k = searcher_->RadiusSearch(index, search_radius_, nn_indices, nn_dists);

			// Points having too few neighbors are outliers and are passed to removed indices
			// Unless negative was set, then it's the opposite condition
			if (k <= min_pts_radius_) {
				removed.push_back(index);
				continue;
			}

			// Otherwise it was a normal point for output(inlier)
			indices[effective++] = index;
		}
	}

	// Resize the output arrays
	indices.resize(effective);
}