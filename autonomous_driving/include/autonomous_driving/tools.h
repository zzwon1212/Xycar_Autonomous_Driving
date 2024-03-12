#ifndef TOOLS_H_
#define TOOLS_H_

#include "yaml-cpp/yaml.h"
#include "opencv2/opencv.hpp"

#include <yolov3_trt_ros/BoundingBox.h>
#include <yolov3_trt_ros/BoundingBoxes.h>

namespace xycar
{
class Tools
{
public:
    using Ptr = Tools*;

    Tools(const YAML::Node& config);

    void undistortImg(const cv::Mat& input_img, cv::Mat& output_img);

    /**
     * @brief decide whether there is stop line
     *
     * @param[in] img input image
     * @param[in] is_stopline whether there is stopline in input image
     * @param[in] stoplines detected stoplines
     */
    void isStopline(const cv::Mat& img, bool& is_stopline, std::vector<cv::Vec4f>& stoplines);

    void drawStoplines(cv::Mat& img, const std::vector<cv::Vec4f>& stoplines);
    void undistortLanesPosition(
        const std::pair<float, float>& lanes_position,
        const uint16_t& y,
        std::vector<cv::Point>& undistorted_lanes_position);

    /**
     * @brief draw bounding boxes detected by YOLO
     *
     * @param[in] img input and output image
     * @param[in] predictions predictions detected by YOLO
     */
    void drawBboxes(cv::Mat& img, const yolov3_trt_ros::BoundingBoxes& predictions);
    void drawLanes(cv::Mat& img, std::vector<cv::Point>& lanes_position, const bool is_first_frame);

    /**
     * @brief get the closest object detected by YOLO
     *
     * @param[in] predictions predictions detected by YOLO
     * @param[in] closest_object closest object among predictions
     * @param[in] depth depth of closest object
     */
    void getClosestObject(
        const yolov3_trt_ros::BoundingBoxes& predictions,
        yolov3_trt_ros::BoundingBox& closest_object,
        float& depth);

private:
    void getConfig(const YAML::Node& config);

    std::vector<std::string> LABELS_;
    std::vector<cv::Scalar> COLORS_;
    uint16_t IMAGE_WIDTH_, IMAGE_HEIGHT_, YOLO_RESOLUTION_;
    float RESIZING_X_, RESIZING_Y_;

};
}

#endif