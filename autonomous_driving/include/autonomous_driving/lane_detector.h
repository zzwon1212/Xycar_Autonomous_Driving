#ifndef LANE_DETECTOR_H_
#define LANE_DETECTOR_H_

#include "yaml-cpp/yaml.h"
#include "opencv2/opencv.hpp"

namespace xycar
{
/**
 * @brief Detecting lanes
 *
 * This class is for detecting lanes using Hough transformation.
 */
class LaneDetector final
{
public:
    using Ptr = LaneDetector*;  // Pointer type of the class

    /**
     * @brief Construct a new LaneDetector object
     */
    LaneDetector(const YAML::Node& config);

    /**
     * @brief Get lane information from image.
     *
     * @param[in] img The input image from camera.
     * @return A pair of pairs where the first pair contains left and right lane coordinate and
     *         the second pair contains boolean flags indicating the presence of left and right lanes.
     */
    std::pair<std::pair<float, float>, std::pair<bool, bool>> getLaneInfo(const cv::Mat& img);

    void setYOffset(float speed);
    uint16_t moving_y_offset_;

private:
    /**
     * @brief Get the parameters from config file
     *
     * @param[in] config Configuration file
     */
    void getConfig(const YAML::Node& config);

    /**
     * @brief Separate all points from given lines into left and right sides.
     *
     * @param[in] lines All lines detected by Hough transformation.
     * @return A pair of vector where the first vector contains points from lines at left side and
     *         the second vector contains points from lines at right side.
     */
    std::pair<std::vector<float>, std::vector<float>> separateLeftRight(const std::vector<cv::Vec4f>& lines);

    /**
     * @brief Get lane information from image.
     *
     * @param[in] left_x_at_Y_offset Vector which contains points from lines at left side.
     * @param[in] right_x_at_Y_offset Vector which contains points from lines at right side.
     * @return A tuple of 4 elements where the first and the second are lane coordinate of each side and
     *         the third and the fourth are boolean flag indicating the presence of lane at each side.
     */
    std::tuple<cv::Point2f, cv::Point2f, bool, bool> getLinePosition(
        const std::vector<float>& left_x_at_Y_offset, const std::vector<float>& right_x_at_Y_offset);

    // Configurations
    uint16_t IMG_WIDTH_;  // Width of image from camera
    uint16_t Y_OFFSET_;  // Initial y coordinate to detect lane candidates
    uint16_t Y_GAP_;  // Value to set ROI for detecting lines
    float Y_GAIN_;  // Value to adjust y coordinate based on the speed
    uint16_t LOW_THRESH_, HIGH_THRESH_;  // Canny parameters
    uint16_t MIN_PIXEL_, MIN_LINE_, MAX_GAP_;  // HoughlinesP parameters

    // Variables to get line position
    cv::Point2f prev_left_, prev_right_;  // The last lane coordinates for situation when no lines are detected
    uint32_t leftC_, rightC_;
};
}  // namespace xycar

#endif  // LANE_DETECTOR_H_
