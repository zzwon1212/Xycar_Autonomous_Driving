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

    /**
     * @brief Undistort image.
     *
     * This function undistort image using camera matrix and distortion coefficients.
     *
     * @param[in] input_img input image which is distorted
     * @param[in] output_img output image which is undistorted
     */
    void undistortImg(const cv::Mat& input_img, cv::Mat& output_img);

    /**
     * @brief Undistort lanes coordinates.
     *
     * This function undistort given lanes coordinates using camera matrix and distortion coefficients.
     *
     * @param[in] lanes_position Input lanes x coordinates (left_x, right_x) which is distorted
     * @param[in] y Input lanes y coordinate which is distorted
     * @param[in] undistorted_lanes_position Output vector containing lanes coordinates (x, y) which is undistorted
     */
    void undistortLanesPosition(
        const std::pair<float, float>& lanes_position,
        const uint16_t& y,
        std::vector<cv::Point>& undistorted_lanes_position);

    /**
     * @brief Draw bounding boxes detected by YOLO.
     *
     * @param[in] img Input and output image
     * @param[in] predictions Predictions detected by YOLO
     */
    void drawBboxes(cv::Mat& img, const yolov3_trt_ros::BoundingBoxes& predictions);

    /**
     * @brief Draw detected lanes.
     *
     * @param[in] img Input and output image
     * @param[in] lanes_position Vector containing left and right lanes coordinates (x, y)
     * @param[in] is_first_frame Boolean flag for first frame
     */
    void drawLanes(cv::Mat& img, std::vector<cv::Point>& lanes_position, const bool is_first_frame);

    /**
     * @brief Decide whether there is stop line.
     *
     * @param[in] img Input image
     * @param[in] is_stopline Output boolean flag indicating whether there is stopline in input image
     * @param[in] stoplines Vector containing detected stoplines
     */
    void isStopline(const cv::Mat& img, bool& is_stopline, std::vector<cv::Vec4f>& stoplines);

    /**
     * @brief Draw detected stoplines
     *
     * @param[in] img Input and output image
     * @param[in] stoplines Vector containing detected stoplines
     */
    void drawStoplines(cv::Mat& img, const std::vector<cv::Vec4f>& stoplines);

    /**
     * @brief Get the closest object detected by YOLO
     *
     * @param[in] predictions Predictions detected by YOLO
     * @param[in] closest_object Closest object among predictions
     * @param[in] depth Depth of closest object
     */
    void getClosestObject(
        const yolov3_trt_ros::BoundingBoxes& predictions,
        yolov3_trt_ros::BoundingBox& closest_object,
        float& depth);

private:
    /**
     * @brief Get the parameters from config file.
     *
     * @param[in] config Configuration file
     */
    void getConfig(const YAML::Node& config);

    std::vector<std::string> LABELS_;  // Labels for drawing detected objects
    std::vector<cv::Scalar> COLORS_;  // Colors for drawing detected objects
    uint16_t IMAGE_WIDTH_;  // Width of image from camera
    uint16_t IMAGE_HEIGHT_;  // Height of image from camera
    uint16_t YOLO_RESOLUTION_;  // Resolution of image which is input of YOLO
    float RESIZING_X_;  // Resizing X value to draw YOLO output at original image resolution
    float RESIZING_Y_;  // Resizing Y value to draw YOLO output at original image resolution
};  // class Driving
}  // namespace xycar

#endif // TOOLS_H_