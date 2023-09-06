//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "filters/smoother.h"

#include <iostream>
#include <cmath>

std::unique_ptr<Smoother> Smoother::Create(int level, float err)
{
    if (level < 0 || err < 0) {
        std::cerr << "Illegal level parameter detected" << std::endl;
        return nullptr;
    }

    std::unique_ptr<Smoother> smoother(new Smoother(level, err));
    if (smoother == nullptr)
        return nullptr;

    return smoother;
}

static inline void Memcpy(float *dst, const float *src, size_t length)
{
    for (size_t i = 0; i < length; i++)
        *dst++ = *src++;
}

float Smoother::Submap(float *data, size_t length)
{
    if (data[length] > 5)
        return data[length];

    if (data[length] == 0 || data[length] > 65)
        return 0;

    // 统计前后点与当前点的距离误差
    std::vector<float> err;
    for (size_t i = 0; i < 2 * length + 1; i++)
        err.push_back(fabs(data[length] - data[i]));

    int cnt = 0;
    float sum = 0;
    // smooth window size = 2 * length + 1
    for (size_t i = 0; i < 2 * length + 1; i++) {
        if (err.at(i) > err_)
            continue;

        sum += data[i];
        cnt++;
    }

    return sum / cnt;
}

bool Smoother::Run(std::vector<float> &out)
{
    if (data_ == nullptr) {
        std::cerr << "Failed to find data vector" << std::endl;
        return false;
    }

    if (level_ < 0 || level_ > data_->size() / 2) {
        std::cerr << "Illegal level parameter detected" << std::endl;
        return false;
    }

    // build calculation vector
    std::vector<float> data(data_->size() + 2 * level_);
    Memcpy(&data[0], &(*data_)[data_->size() - level_], level_);
    Memcpy(&data[level_], &(*data_)[0], data_->size());
    Memcpy(&data[data_->size() + level_], &(*data_)[0], level_);

    // smooth window size = 2 * level_ + 1
    for (size_t i = level_; i < data_->size() + level_; i++)
        data[i] = Submap(&data[i - level_], level_);

    for (size_t i = 0; i < data_->size(); i++)
        (*data_)[i] = data[i + level_];

    return true;
}

// TODO: 后续稳定后将参数移动到配置文件
#define dlt_int 6
#define dlt_rge 0.016
#define pt_cnt 6

bool Smoother::Run(std::vector<float> &out, std::vector<float> &inten)
{
	// build calculation vector
	std::vector<float> rges(data_->size() + 2 * pt_cnt);
	Memcpy(&rges[0], &(*data_)[data_->size() - pt_cnt], pt_cnt);
	Memcpy(&rges[pt_cnt], &(*data_)[0], data_->size());
	Memcpy(&rges[data_->size() + pt_cnt], &(*data_)[0], pt_cnt);

	std::vector<float> ints(data_->size() + 2 * pt_cnt);
	Memcpy(&ints[0], &inten[data_->size() - pt_cnt], pt_cnt);
	Memcpy(&ints[pt_cnt], &inten[0], data_->size());
	Memcpy(&ints[data_->size() + pt_cnt], &inten[0], pt_cnt);

	int s_index = 0;
	int e_index = pt_cnt;
	int lcnt = 0;
	int rcnt = 0;
	float avg_all = 0;
	float avg_l = 0;
	float avg_r = 0;
	int last = -1;

	float lmin = 0;
	float lmax = 0;
	float rmin = 0;
	float rmax = 0;

	//for(size_t i = pt_cnt; i < data_->size() + pt_cnt; i++)
	size_t i = pt_cnt;
	while (i < (data_->size()+ pt_cnt)) {
		if( 0 == rges[i]) {
			i += 1;
			continue;
		}

		s_index = -1;
		e_index = -1;

		lcnt = 0;
		rcnt = 0;
		avg_all = rges[i];

		lmin = rges[i];
		lmax = rges[i];

		rmin = rges[i];
		rmax = rges[i];

		int lbreak = 0;
		avg_l = 0;

		for (size_t j = i-pt_cnt; j < i; j++) {
			if (0 == rges[j]) {
				continue;
			}

			if ((fabs(rges[j] - lmin) < dlt_rge) && (fabs(rges[j] - lmax) < dlt_rge) &&
                (fabs(rges[j] - rges[i]) < 2*dlt_rge) &&  (fabs(ints[j] - ints[i]) < dlt_int)) {
				avg_all += rges[j];
				avg_l += rges[j];

				s_index = j;

				lcnt += 1;

				if (rges[j] < lmin) {
					lmin = rges[j];
				}

				if (rges[j] > lmax) {
					lmax = rges[j];
				}
			} else {
				lbreak += 1;
				if (lbreak > 2) {
					break;
				}
			}
		}

		int rbreak = 0;
		avg_r = 0;

		for (size_t k = i+1; k < i+pt_cnt+1; k++) {
			if (0 == rges[k]) {
				continue;
			}

			if ((fabs(rges[k] - rmin) < dlt_rge) && (fabs(rges[k] - rmax) < dlt_rge) &&
                (fabs(rges[k] - rges[i]) < 2*dlt_rge) &&  (fabs(ints[k] - ints[i]) < dlt_int)) {
				avg_all += rges[k];
				avg_r += rges[k];

				e_index = k;

				rcnt += 1;

				if (rges[k] < rmin) {
					rmin = rges[k];
				}

				if (rges[k] > rmax) {
					rmax = rges[k];
				}

			} else {
				rbreak += 1;
				if (rbreak > 2) {
					break;
				}
			}
		}

		if ((-1 != s_index)  && (-1 != e_index)) {
			avg_all = avg_all/(lcnt+rcnt+1);
			avg_l = avg_l/lcnt;
			avg_r = avg_r/rcnt;

			int mid = (s_index+e_index)/2;

			if (0 != ((s_index+e_index)%2)) {
				mid += 1;
			}

			for (size_t n = s_index; n < mid; n++) {
				if ((0 != rges[n]) && ( (n >= pt_cnt) && ( (n-pt_cnt) < 1440))) {
					float r = (float(n-s_index+1)/float(mid-s_index));
					if (fabs(rges[n]-avg_all) > 1) {
						r = r*0.6;
					}

					(*data_)[n-pt_cnt] = 0.6*r*rges[n] + (1.0-0.6*r)*avg_all;
				}
			}

			for (size_t n = mid; n < e_index+1; n++) {
				if ((0 != rges[n]) && ((n >= pt_cnt) && ((n-pt_cnt) <1440))) {
					float r = (float(n-mid+1)/float(e_index+1-mid));
					if (fabs(rges[n]-avg_all) > 1) {
						r = r*0.6;
					}

					(*data_)[n-pt_cnt] = 0.6*r*avg_all + (1-0.6*r)*rges[n];
				}
			}
		}


		i += 1;
	}

	//delete by int
	i = pt_cnt;
	while (i < (data_->size()+ pt_cnt)) {
		if (0 == rges[i]) {
			i += 1;
			continue;
		}

		s_index = -1;
		e_index = -1;

		int lbreak = 0;
		for (size_t j = i-1; j < i; j++) {
			if (0 == rges[j]) {
				continue;
			}

			if ((fabs(ints[j] - ints[i]) > 50) && (fabs(rges[j] - rges[i]) < 2*dlt_rge)) {
				s_index = j;
				break;
			} else {
				lbreak += 1;
				if (lbreak > 2) {
					break;
				}
			}
		}

		int rbreak = 0;
		for (size_t k = i+1; k < i+1+1; k++) {
			if (0 == rges[k]) {
				continue;
			}

			if ((fabs(ints[k] - ints[i]) > 50) && (fabs(rges[k] - rges[i]) < 2*dlt_rge)) {
				e_index = k;
				break;
			} else {
				rbreak += 1;
				if (rbreak > 2) {
					break;
				}
			}
		}

		if ((-1 != s_index) || (-1 != e_index)) {
			(*data_)[i-pt_cnt] = 0;
		}

		i += 1;
	}

	return true;
}
