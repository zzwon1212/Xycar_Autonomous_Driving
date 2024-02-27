#ifndef DRIVING_H_
#define DRIVING_H_

// #include <cmath>
// #include <ctime>

// #include <algorithm>
// #include <vector>

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

//     // void bboxClassCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& bbox_class_data);
//     std::string getfilename();

private:  // @@@@@@@@@@@@@@@@@ TODO: Is this private necessary?
    ControllerPtr PID_;                      ///< PID Class for Control
    DetectorPtr LaneDetector_;

    // ROS Variables
    uint32_t queue_size_;  // Max queue size for message
    ros::NodeHandle NodeHandler_;  // Node Hanlder for ROS. In this case Detector and Controller
    ros::Subscriber SubscriberImg_;  // Subscriber to receive topic from Camera
    ros::Subscriber SubscriberScan_;  // Subscriber to receive topic from LiDAR
    ros::Subscriber SubscriberYOLO_;  // Subscriber to receive topic from YOLO model
    ros::Publisher PublisherMotor_;  // Publisher to send topic to Xycar motor
    std::string sub_topic_img_;  // Topic name to subscribe from Camera
    std::string sub_topic_scan_;  // Topic name to subscribe from LiDAR
    std::string sub_topic_yolo_;  // Topic name to subscribe from YOLO model
    std::string pub_topic_motor_;  // Topic name to publish to Xycar motor
    // xycar_msgs::xycar_motor mMotorMessage;  // @@@@@@@@@@@ TODO: Is necessary? Message for the motor of xycar

    // OpenCV Image processing Variables
    void imageCallback(const sensor_msgs::Image& message);
    cv::Mat frame_;  // Image from camera. The raw image is converted into cv::Mat

    // Xycar Device variables
    PREC xycar_speed_;  // Current speed of xycar
    PREC xycar_speed_min_;  // Min speed of xycar
    PREC xycar_speed_max_;  // Max speed of xycar
    PREC xycar_speed_control_thresh_;  // Threshold of angular of xycar
    PREC deceleration_step_;  // How much would deaccelrate xycar depending on threshold
    PREC tmp_deceleration_step_;
    PREC acceleration_step_;  // How much would accelrate xycar depending on threshold

    // LiDAR variables
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& message);
    std::vector<float> lidar_data_;
    PREC front_obs_cnt_;
    PREC left_obs_cnt_;
    PREC right_obs_cnt_;
    PREC front_obs_angle_;
    PREC front_obs_depth_;
    PREC front_obs_cnt_thresh_;
    PREC side_obs_depth_;
    PREC side_obs_cnt_thresh_;
    int last_obs_pos_ = -1;

    // Object Detection variables
    void yoloCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& predictions);
    std::vector<int> nearest_object_;
    void findStopLine(cv::Mat& frame);
    bool is_stopline_;
    int last_obj_class_ = -1;
    PREC obj_depth_thresh_;

//     // cv::VideoWriter outputVideo;
//     // Debug Flag
    bool is_debugging_;  // Debugging or not
};  // class Driving
}  // namespace xycar

#endif // DRIVING_H_