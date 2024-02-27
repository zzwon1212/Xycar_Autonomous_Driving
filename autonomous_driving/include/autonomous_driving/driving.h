#ifndef DRIVING_H_
#define DRIVING_H_

#include <cmath>
// #include <ctime>

// #include <algorithm>
// #include <vector>

#include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/LaserScan.h>
#include <xycar_msgs/xycar_motor.h>
// #include <yolov3_trt_ros/BoundingBox.h>
// #include <yolov3_trt_ros/BoundingBoxes.h>

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

    static constexpr int32_t kSteeringAngleLimit = 50;  // Xycar Steering Angle Limit
    static constexpr double kFPS = 33.0;  // FPS
    /**
     * @brief Construct a new Driving object
     */
    Driving();

    /**
     * @brief Destroy the Driving object
     */
    // virtual ~Driving();

    /**
     * @brief Run
     */
    void run();

private:
//     /**
//      * @brief Set the parameters from config file
//      *
//      * @param[in] config Configuration for searching and keeping Hough lines using Hough, Moving average and PID control
//      */
    void setParams(const YAML::Node& config);

//     /**
//      * @brief Control the speed of xycar
//      *
//      * @param[in] steeringAngle Angle to steer xycar. If over max angle, deaccelerate, otherwise accelerate
//      */
//     void speedControl(PREC steeringAngle);

//     /**
//      * @brief publish the motor topic message
//      *
//      * @param[in] steeringAngle Angle to steer xycar actually
//      */
//     void drive(PREC steeringAngle);
//     void findStopLine(cv::Mat& frame);
//     void imageCallback(const sensor_msgs::Image& message);
//     void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
//     // void bboxClassCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& bbox_class_data);
//     // void yoloCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& predictions);
//     std::string getfilename();

private:  // @@@@@@@@@@@@@@@@@ TODO: Is this private necessary?
    ControllerPtr PID_;                      ///< PID Class for Control
//     // FilterPtr mMovingAverage;                ///< Moving Average Filter Class for Noise filtering
    DetectorPtr LaneDetector_;

//     // ROS Variables
//     ros::NodeHandle mNodeHandler;          ///< Node Hanlder for ROS. In this case Detector and Controler
//     ros::Publisher mPublisher;             ///< Publisher to send message about
//     ros::Subscriber mSubscriber;           ///< Subscriber to receive image
//     ros::Subscriber mSubscriberScan;
//     ros::Subscriber mSubscriberBbox;
//     ros::Subscriber mSubscriberBboxClass;
//     std::string mPublishingTopicName;      ///< Topic name to publish
//     std::string mSubscribedTopicName;      ///< Topic name to subscribe
//     std::string mSubscribedTopicScan;
//     std::string mSubscribedTopicYolo;
//     uint32_t mQueueSize;                   ///< Max queue size for message
//     // xycar_msgs::xycar_motor mMotorMessage; ///< Message for the motor of xycar

//     // OpenCV Image processing Variables
    cv::Mat frame_;  // Image from camera. The raw image is converted into cv::Mat
//     std::vector<float> mScan;
//     int mLidarLastObsPos = -1;
    std::vector<int> nearest_object_;
//     int mLastPredClass = -1;
//     // bool mStopLine = false;
//     bool mStopLine;

//     // Xycar Device variables
//     PREC mXycarSpeed;                 ///< Current speed of xycar
//     PREC mXycarMaxSpeed;              ///< Max speed of xycar
//     PREC mXycarMinSpeed;              ///< Min speed of xycar
//     PREC mXycarSpeedControlThreshold; ///< Threshold of angular of xycar
//     PREC mAccelerationStep;           ///< How much would accelrate xycar depending on threshold
    PREC deceleration_step_;           ///< How much would deaccelrate xycar depending on threshold
    PREC tmp_deceleration_step_;

//     PREC mFrontObsCnt;
//     PREC mLeftObsCnt;
//     PREC mRightObsCnt;
//     PREC mLidarSideDepth;
//     PREC mLidarSideObsThresh;
//     PREC mLidarFrontAngle;
//     PREC mLidarFrontDepth;
//     PREC mLidarFrontObsThresh;

//     PREC mObjXDepthThresh;
//     PREC mObjXYDepthThresh;

//     // cv::VideoWriter outputVideo;
//     // Debug Flag
//     bool mDebugging = false; ///< Debugging or not
};  // class Driving
}  // namespace xycar

#endif // DRIVING_H_
