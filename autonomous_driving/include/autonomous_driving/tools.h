#ifndef TOOLS_H_
#define TOOLS_H_

#include "yaml-cpp/yaml.h"
#include "opencv2/opencv.hpp"

#include <yolov3_trt_ros/BoundingBoxes.h>

namespace xycar
{
class Tools
{
public:
    using Ptr = Tools*;

    Tools(const YAML::Node& config);

    void undistortImg(const cv::Mat& input_img, cv::Mat& output_img);
    void show(
        const cv::Mat& input_img,
        cv::Mat& output_img,
        const std::pair<float, float>& lanes_position,
        const uint16_t y,
        std::vector<cv::Point>& undistorted_lanes_position,
        const yolov3_trt_ros::BoundingBoxes& predictions);

private:
    void getConfig(const YAML::Node& config);
    void undistortLanesPosition(
        const std::pair<float, float>& lanes_position,
        const uint16_t y,
        std::vector<cv::Point>& undistorted_lanes_position);

    /**
     * @brief draw bounding boxes detected by YOLO
     *
     * @param[in] img input and output image
     * @param[in] predictions predictions detected by YOLO
     */
    void drawBboxes(cv::Mat& img, const yolov3_trt_ros::BoundingBoxes& predictions);
    void drawLanes(const std::vector<cv::Point>& lanes_position, cv::Mat& img);

    std::vector<std::string> LABELS_;
    std::vector<cv::Scalar> COLORS_;
    uint16_t IMAGE_WIDTH_, IMAGE_HEIGHT_, YOLO_RESOLUTION_;
    float RESIZING_X_, RESIZING_Y_;

};
}

#endif