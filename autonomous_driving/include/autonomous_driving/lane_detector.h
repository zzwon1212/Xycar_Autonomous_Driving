#ifndef LANE_DETECTOR_H_
#define LANE_DETECTOR_H_

#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>

namespace xycar
{
class LaneDetector final
{
public:
    using Ptr = LaneDetector*;  // Pointer type of the class

    LaneDetector(const YAML::Node& config);

    std::pair<std::pair<float, float>, std::pair<bool, bool>> getLaneInfo(cv::Mat& frame);

    void setYOffset(float speed);
    uint16_t moving_y_offset_;

private:
    void getConfig(const YAML::Node& config);

    std::pair<std::vector<float>, std::vector<float>> divideLeftRight(std::vector<cv::Vec4f>& lines);

    std::tuple<cv::Point2f, cv::Point2f, bool, bool> getLinePosition(
        std::vector<float>& left_x_at_Y_offset, std::vector<float>& right_x_at_Y_offset);

    // Configurations
    uint16_t IMG_WIDTH_, Y_OFFSET_, Y_GAP_;
    float Y_GAIN_;
    uint16_t LOW_THRESH_, HIGH_THRESH_;  // Canny params
    uint16_t MIN_PIXEL_, MIN_LINE_, MAX_GAP_;  // HoughlinesP params

    // Variables to get line position
    cv::Point2f prev_left_, prev_right_;
    uint32_t leftC_, rightC_;
};
}  // namespace xycar

#endif  // LANE_DETECTOR_H_
