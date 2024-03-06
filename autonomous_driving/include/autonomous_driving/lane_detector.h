#ifndef LANE_DETECTOR_H_
#define LANE_DETECTOR_H_

#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>

namespace xycar
{
template <typename PREC>
class LaneDetector final
{
public:
    using Ptr = LaneDetector*;  // Pointer type of the class(It's up to you)

    // Scalar values of colors
    static inline const cv::Scalar RED = {0, 0, 255};
    static inline const cv::Scalar GREEN = {0, 255, 0};
    static inline const cv::Scalar BLUE = {255, 0, 0};

    LaneDetector(const YAML::Node& config)
    {
        setConfig(config);
        prev_left_ = cv::Point(0, moving_y_offset_);
        prev_right_ = cv::Point(img_width_, y_offset_);
        leftC_ = 0;
        rightC_ = 0;
    }

    std::pair<std::pair<int, int>, std::pair<bool, bool>> getLaneInfo(cv::Mat& frame);

    int32_t moving_y_offset_;
    void setYOffset(double speed){moving_y_offset_ = y_offset_ - speed * y_gain_;}

private:
    int32_t img_width_, img_height_;
    int32_t low_threshold_, high_threshold_;  // Canny params
    int32_t min_pixel_, min_line_, max_gap_;  // HoughlinesP params
    int32_t y_offset_, y_gap_;
    double y_gain_;
    bool is_debugging_;
    cv::Point prev_left_, prev_right_;
    int leftC_, rightC_;

    void setConfig(const YAML::Node& config);
    std::pair<std::vector<int>, std::vector<int>> divideLeftRight(std::vector<cv::Vec4f>& lines);
    std::tuple<cv::Point, cv::Point, bool, bool> getLinePosition(std::vector<int>& left_x_at_Y_offset,
                                                                 std::vector<int>& right_x_at_Y_offset);
};
}  // namespace xycar

#endif  // LANE_DETECTOR_H_
