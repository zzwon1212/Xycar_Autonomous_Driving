#ifndef DRIVING_H_
#define DRIVING_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <yolov3_trt_ros/BoundingBox.h>
#include <yolov3_trt_ros/BoundingBoxes.h>
#include <xycar_msgs/xycar_motor.h>

#include "autonomous_driving/lane_detector.h"
#include "autonomous_driving/PID_controller.h"

namespace xycar
{
/**
 * @brief Lane Keeping System for searching and keeping Hough lines using Hough, Moving average and PID control
 *
 * @tparam Precision of data
 */

template <typename PREC>
class Driving
{
public:
    using Ptr = Driving*;  // Pointer type of this class
    using ControllerPtr = typename PIDController<PREC>::Ptr;  // Pointer type of PIDController
    using DetectorPtr = typename LaneDetector<PREC>::Ptr;  // Pointer type of LaneDetecter(It's up to you)

    static constexpr int32_t STEERING_ANGLE_LIMIT = 50;  // Xycar Steering Angle Limit
    static constexpr double FPS = 33.0;  // FPS
    /**
     * @brief Construct a new Driving object
     */
    Driving();

    /**
     * @brief Destroy the Driving object
     */
    virtual ~Driving();

    /**
     * @brief Run
     */
    void run();

private:
    /**
     * @brief Set the parameters from config file
     *
     * @param[in] config Configuration for searching and keeping Hough lines using Hough, Moving average and PID control
     */
    void setParams(const YAML::Node& config);

//     /**
//      * @brief Control the speed of xycar
//      *
//      * @param[in] steering_angle Angle to steer xycar. If over max angle, deaccelerate, otherwise accelerate
//      */
    void controlSpeed(PREC steering_angle);

    /**
     * @brief publish the motor topic message
     *
     * @param[in] steering_angle Angle to steer xycar actually
     */
    void drive(PREC steering_angle);

    ControllerPtr PID_;  // PID Class for Control
    DetectorPtr LaneDetector_;

    // ROS Variables
    uint32_t QUEUE_SIZE_;  // Max queue size for message
    ros::NodeHandle NodeHandler_;  // Node Hanlder for ROS. In this case Detector and Controller
    ros::Subscriber SubscriberImg_;  // Subscriber to receive topic from Camera
    ros::Subscriber SubscriberScan_;  // Subscriber to receive topic from LiDAR
    ros::Subscriber SubscriberYOLO_;  // Subscriber to receive topic from YOLO model
    ros::Publisher PublisherMotor_;  // Publisher to send topic to Xycar motor
    std::string SUB_TOPIC_IMG_;  // Topic name to subscribe from Camera
    std::string SUB_TOPIC_SCAN_;  // Topic name to subscribe from LiDAR
    std::string SUB_TOPIC_YOLO_;  // Topic name to subscribe from YOLO model
    std::string PUB_TOPIC_MOTOR_;  // Topic name to publish to Xycar motor

    // OpenCV Image processing Variables
    void imageCallback(const sensor_msgs::Image::ConstPtr& message);
    cv::Mat frame_;  // Image from camera. The raw image is converted into cv::Mat

    // Xycar Device variables
    PREC XYCAR_SPEED_, XYCAR_SPEED_MIN_, XYCAR_SPEED_MAX_;  // Speed of xycar
    PREC XYCAR_SPEED_CONTROL_THRESH_;  // Threshold of angular of xycar
    PREC DECELERATION_STEP_, ACCELERATION_STEP_;  // How much would (de)accelrate xycar depending on threshold
    PREC tmp_deceleration_step_;

    // LiDAR variables
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& message);
    std::vector<float> lidar_data_;
    PREC front_obs_cnt_, left_obs_cnt_, right_obs_cnt_;
    PREC FRONT_OBS_ANGLE_, FRONT_OBS_DEPTH_, FRONT_OBS_CNT_THRESH_;
    PREC SIDE_OBS_DEPTH_, SIDE_OBS_CNT_THRESH_;
    int last_obs_pos_ = -1;

    // Object Detection variables
    std::vector<std::string> LABELS_;
    void yoloCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& message);
    yolov3_trt_ros::BoundingBoxes predictions_;

    void undistortImg(const cv::Mat& input_img, cv::Mat& output_img);
    /**
     * @brief draw bounding boxes detected by YOLO
     *
     * @param[in] input_img input image
     * @param[in] output_img output image
     * @param[in] predictions predictions detected by YOLO
     */
    void drawBboxes(const cv::Mat& input_img, cv::Mat& output_img,
                    const yolov3_trt_ros::BoundingBoxes& predictions);
    std::vector<cv::Scalar> COLORS_;

    void undistortLanesPosition(const std::pair<int, int>& lanes_position,
                                const int32_t y,
                                std::vector<cv::Point>& undistorted_lanes_position);
    void drawLanes(cv::Mat& input_img, const std::vector<cv::Point>& lanes_position);

    /**
     * @brief get the closest object detected by YOLO
     *
     * @param[in] predictions predictions detected by YOLO
     * @param[in] closest_object closest object among predictions
     */
    void getClosestObject(const yolov3_trt_ros::BoundingBoxes& predictions,
                          yolov3_trt_ros::BoundingBox& closest_object);

    /**
     * @brief decide whether there is stop line
     *
     * @param[in] input_img input mage
     */
    bool isStopLine(const cv::Mat& input_img);

    int last_obj_class_ = -1;
    PREC RESIZING_X_, RESIZING_Y_;
    PREC OBJ_DEPTH_THRESH_;

    bool IS_DEBUGGING_;  // Debugging or not
};  // class Driving
}  // namespace xycar

#endif // DRIVING_H_