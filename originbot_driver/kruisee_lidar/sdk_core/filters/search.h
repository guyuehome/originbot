//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <flann/flann.hpp>

#include "filters/point2d.h"

template<class T>
struct OL2 {
	using is_kdtree_distance = bool;
	using ElementType = T;
	using ResultType = typename flann::Accumulator<T>::Type;

	template<typename Unit1, typename Unit2>
	ResultType operator()(Unit1 a, Unit2 b, size_t size, ResultType /* worst_dist */ = -1) const {
		ResultType result = ResultType();
		ResultType diff;

		for (size_t i = 0; i < size; ++i) {
			diff = *a++ - *b++;
			result += diff * diff;
		}

		return result;
	}

	template<typename U, typename V>
	ResultType accum_dist(const U &a, const V &b, int) const { return (a - b) * (a - b); }
};

class Search {
public:
	using Index = flann::Index<OL2<float>>;

	explicit Search(bool sorted = false);
	~Search() = default;

	void SetCloud(const std::vector<Point2D> *cloud);

	// index 	点索引
	// k 		指定搜索半径内, 相邻点目标数量
	// indices 	指定半径内, 相邻点索引
	// distances到相邻点距离集合
	int NearestKSearch(int index, int k, std::vector<int> &indices, std::vector<float> &distances);

	// index 	点索引
	// radius	搜索半径
	// indices	指定半径内, 相邻点索引
	// distances到相邻点距离集合
	// max_nn	将最大返回相邻点限制为该值(为0或者大于点云数量, 则将返回半径内所有点)
	int RadiusSearch(int index, double radius, std::vector<int> &indices, std::vector<float> &distances, uint32_t max_nn = 0);

private:
	std::unique_ptr<Index> index_;

	// 树维数(点维数)
	int dim_;

	// 最邻近搜索算法参数
	flann::SearchParams param_k_;

	// 半径搜索算法参数
	flann::SearchParams param_radius_;

	const std::vector<Point2D> *cloud_;
};