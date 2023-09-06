//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <memory>
#include <vector>

#include "filters/point2d.h"
#include "drivers/msg.h"

class Search;
class OutlierFilter {
public:
	enum Mode { Knn, Radius };
	static std::unique_ptr<OutlierFilter> Create(double search_radius, uint32_t min_neighbors, double max_distance);
	~OutlierFilter() = default;

	void SetCloud(const std::vector<Point2D> *cloud) { this->input_ = cloud; }
	void SetLScan(const LScan *scan) { this->scan_ = scan; }

	bool Run(Mode mode, std::vector<Point2D> &output, std::vector<int> &removed);

	void set_radius_search(double radius) { search_radius_ = radius; }
	double get_radius_search() { return search_radius_; }

	void set_min_neighbors_in_radius(int min_pts) { min_pts_radius_ = min_pts; }
	int get_min_neighbors_in_radius() { return min_pts_radius_; }

	void set_max_distance(double distance) { max_distance_ = distance; }
	double get_max_distance(void) { return max_distance_; }

	void set_min_distance(double distance) { min_distance_ = distance; }
	double get_min_distance(void) { return min_distance_; }

	void set_search_flags(int start, int end);

private:
	template <class T>
	using Deleter = void (*)(T *);

	explicit OutlierFilter(std::unique_ptr<Search, Deleter<Search>> searcher);

	bool Prepare();

	void ApplyFilter(Mode mode, std::vector<Point2D> &output, std::vector<int> &removed);
	void ApplyFilter(Mode mode, std::vector<int> &indices, std::vector<int> &removed);

private:
	std::unique_ptr<Search, Deleter<Search>> searcher_;

	const std::vector<Point2D> *input_;
	std::vector<int> indices_;

	std::vector<int> search_flags_;

	const LScan *scan_;

	// 搜索半径
	double search_radius_;

	// 给定搜索半径内, 最少点数
	int min_pts_radius_;

	double max_distance_;

	double min_distance_;
};