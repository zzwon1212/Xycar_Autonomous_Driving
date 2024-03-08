#ifndef LANE_DETECTOR_H_
#define LANE_DETECTOR_H_

#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>

namespace xycar
{
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
        getConfig(config);
        prev_left_ = cv::Point(0, moving_y_offset_);
        prev_right_ = cv::Point(img_width_, y_offset_);
        leftC_ = 0;
        rightC_ = 0;
    }

    std::pair<std::pair<float, float>, std::pair<bool, bool>> getLaneInfo(cv::Mat& frame);

    uint16_t moving_y_offset_;
    void setYOffset(float speed){moving_y_offset_ = y_offset_ - speed * y_gain_;}

private:
    uint16_t img_width_, y_offset_, y_gap_;
    float y_gain_;
    uint16_t low_threshold_, high_threshold_;  // Canny params
    uint16_t min_pixel_, min_line_, max_gap_;  // HoughlinesP params
    bool is_debugging_;
    cv::Point2f prev_left_, prev_right_;
    uint32_t leftC_, rightC_;

    void getConfig(const YAML::Node& config);

    std::pair<std::vector<float>, std::vector<float>> divideLeftRight(std::vector<cv::Vec4f>& lines);
    std::tuple<cv::Point2f, cv::Point2f, bool, bool> getLinePosition(std::vector<float>& left_x_at_Y_offset,
                                                                 std::vector<float>& right_x_at_Y_offset);
};
}  // namespace xycar

#endif  // LANE_DETECTOR_H_
