//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include <functional>
#include <list>
#include <string>
#include <memory>

#include "drivers/msg.h"
#include "drivers/lidar.h"
#include "filters/outlierfilter.h"
#include "filters/smoother.h"
#include "filters/trailingfilter.h"
#include "drivers/iworker.h"

struct OutlierFilterNearParams {
    double front_angle;
    double side_angle;
    double lidar_angle;
    bool angle;
};

class SdkCore {
public:
    static std::unique_ptr<SdkCore> Create(const std::vector<std::string> &cfg_path, const std::string &main_file);
    ~SdkCore();

    enum Type {
        Raw = 0, Filtered
    };
    void RegisterDataDistributor(Type type, std::function<void(const LScan &)> handler);
    bool Run(bool detached = true);
    void RegisterCtlRunHandler(CtlRunHandler handler);
    void RegisterErrHandler(ErrHandler handler);

    bool Write(const std::string &cmd);

private:
    explicit SdkCore(std::unique_ptr<Lidar> lidar);

    void set_outlier_filter(std::unique_ptr<OutlierFilter> filter) { outlier_filter_ = std::move(filter); }
    void set_outlier_filter_near(std::unique_ptr<OutlierFilter> filter) { outlier_filter_near_ = std::move(filter); }
    void set_smoother(std::unique_ptr<Smoother> smoother) { smoother_ = std::move(smoother); }
    void set_trailing_filter(std::unique_ptr<TrailingFilter> filter) { trailing_filter_ = std::move(filter); }
    void set_outlier_filter_near_angles(double front_angle, double side_angle, double lidar_angle);

    void Handle(LScan &scan);
    void Publish(Type type, const LScan &scan);

private:
    std::unique_ptr<Lidar> lidar_;
    std::function<void(const LScan &)> handler_[2];

    std::unique_ptr<OutlierFilter> outlier_filter_;
    std::unique_ptr<OutlierFilter> outlier_filter_near_;
    OutlierFilterNearParams outlier_filter_near_params_;
    std::vector<std::tuple<int, int>> outlier_filter_near_angles_;
    std::unique_ptr<Smoother> smoother_;
    std::unique_ptr<TrailingFilter> trailing_filter_;
};