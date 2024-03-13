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
#include "autonomous_driving/tools.h"

namespace xycar
{
/**
 * @brief Lane Keeping System for searching and keeping Hough lines using Hough, Moving average and PID control
 */

class Driving
{
public:
    using Ptr = Driving*;  // Pointer type of this class

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
    using DetectorPtr = typename LaneDetector::Ptr;  // Pointer type of LaneDetecter
    using ControllerPtr = typename PIDController::Ptr;  // Pointer type of PIDController
    using ToolsPtr = typename Tools::Ptr;

    DetectorPtr LaneDetector_;
    ControllerPtr PID_;
    ToolsPtr Tools_;

    static constexpr double FPS = 30.0;  // FPS 30.007052(phone)
    static constexpr float STEERING_ANGLE_LIMIT = 50.0;  // Xycar Steering Angle Limit

    /**
     * @brief Set the parameters from config file
     *
     * @param[in] config Configuration for searching and keeping Hough lines using Hough, Moving average and PID control
     */
    void getConfig(const YAML::Node& config);

    /**
     * @brief Control the speed of xycar
     *
     * @param[in] steering_angle Angle to steer xycar. If over max angle, deaccelerate, otherwise accelerate
     */
    void controlSpeed(const float steering_angle);

    /**
     * @brief publish the motor topic message
     *
     * @param[in] steering_angle Angle to steer xycar actually
     */
    void drive(const float steering_angle);

    void imageCallback(const sensor_msgs::Image::ConstPtr& message);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& message);
    void yoloCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& message);

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

    // Image processing variables
    cv::Mat frame_;  // Image from camera. The raw image is converted into cv::Mat

    // Xycar Device variables
    float XYCAR_SPEED_, XYCAR_SPEED_MIN_, XYCAR_SPEED_MAX_;  // Speed of xycar
    float XYCAR_SPEED_CONTROL_THRESH_;  // Threshold of angular of xycar
    float DECELERATION_STEP_, ACCELERATION_STEP_;  // How much would (de)accelrate xycar depending on threshold
    float tmp_deceleration_step_;

    // PID variables
    float PID_P_, PID_I_, PID_D_;

    // LiDAR variables
    float FRONT_OBS_ANGLE_, FRONT_OBS_DEPTH_, SIDE_OBS_DEPTH_;
    uint16_t FRONT_OBS_CNT_THRESH_, SIDE_OBS_CNT_THRESH_;
    std::vector<float> lidar_data_;
    uint16_t front_obs_cnt_, left_obs_cnt_, right_obs_cnt_;
    int8_t last_obs_pos_ = -1;

    // Object Detection variables
    float OBJ_DEPTH_THRESH_;
    yolov3_trt_ros::BoundingBoxes predictions_;
    int8_t last_obj_class_ = -1;

    bool IS_DEBUGGING_;  // Debugging or not
};  // class Driving
}  // namespace xycar

#endif // DRIVING_H_