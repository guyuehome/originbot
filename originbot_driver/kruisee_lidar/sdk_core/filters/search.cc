//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "search.h"

#include <iostream>

Search::Search(bool sorted)
    : dim_(2), param_k_(-1, 0.0), param_radius_(-1, 0.0, sorted), cloud_(nullptr) {}

void Search::SetCloud(const std::vector<Point2D> *cloud)
{
    assert(cloud != nullptr);
    assert(cloud->size() != 0);

    cloud_ = cloud;
    index_.reset(new Index(flann::Matrix<float>((float *)cloud->data(), cloud->size(), dim_),
                           flann::KDTreeSingleIndexParams(15))); // max 15 points/leaf
    index_->buildIndex();
}

int Search::NearestKSearch(int index, int k, std::vector<int> &indices, std::vector<float> &distances) {
	assert(index >= 0 && index < static_cast<int>(cloud_->size()) && "Out-of-bounds error in NearestKSearch!");
	if (k > cloud_->size())
		k = cloud_->size();

	indices.resize(k);
	distances.resize(k);

	if (k == 0)
		return 0;

	flann::Matrix<int> matrix_indices(&indices[0], 1, k);
	flann::Matrix<float> matrix_distances(&distances[0], 1, k);

	index_->knnSearch(flann::Matrix<float>((float *)&(*cloud_)[index], 1, dim_), matrix_indices, matrix_distances, k, param_k_);
	return k;
}

int Search::RadiusSearch(int index, double radius, std::vector<int> &indices, std::vector<float> &distances,
                         uint32_t max_nn)
{
	assert(index >= 0 && index < static_cast<int>(cloud_->size()) && "Out-of-bounds error in RadiusSearch!");

	if (max_nn == 0 || max_nn > cloud_->size())
		max_nn = cloud_->size();

	flann::SearchParams params(param_radius_);
	params.max_neighbors = (max_nn == cloud_->size()) ? (-1) : max_nn;

	std::vector<std::vector<int>> indices_array(1);
	std::vector<std::vector<float>> distances_array(1);
	int results = index_->radiusSearch(flann::Matrix<float>((float *)&(*cloud_)[index], 1, dim_), indices_array,
									   distances_array, static_cast<float>(radius * radius), params);

	indices = indices_array[0];
	distances = distances_array[0];

	return results;
}