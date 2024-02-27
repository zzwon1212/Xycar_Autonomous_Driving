#ifndef LANE_KEEPING_SYSTEM_HPP_
#define LANE_KEEPING_SYSTEM_HPP_

#include <vector>
#include <ros/ros.h>
#include <yolov3_trt_ros/BoundingBox.h>
#include <yolov3_trt_ros/BoundingBoxes.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <xycar_msgs/xycar_motor.h>
#include <yaml-cpp/yaml.h>
#include <time.h>
#include <algorithm>
#include <cmath>

#include "LaneKeepingSystem/LaneDetector.hpp"
#include "LaneKeepingSystem/MovingAverageFilter.hpp"
#include "LaneKeepingSystem/PIDController.hpp"

namespace Xycar {
/**
 * @brief Lane Keeping System for searching and keeping Hough lines using Hough, Moving average and PID control
 *
 * @tparam Precision of data
 */
template <typename PREC>
class LaneKeepingSystem
{
public:
    using Ptr = LaneKeepingSystem*;                                     ///< Pointer type of this class
    using ControllerPtr = typename PIDController<PREC>::Ptr;            ///< Pointer type of PIDController
    using FilterPtr = typename MovingAverageFilter<PREC>::Ptr;          ///< Pointer type of MovingAverageFilter
    using DetectorPtr = typename LaneDetector<PREC>::Ptr;               ///< Pointer type of LaneDetecter(It's up to you)

    static constexpr int32_t kXycarSteeringAangleLimit = 50; ///< Xycar Steering Angle Limit
    static constexpr double kFrameRate = 33.0;               ///< Frame rate
    /**
     * @brief Construct a new Lane Keeping System object
     */
    LaneKeepingSystem();

    /**
     * @brief Destroy the Lane Keeping System object
     */
    virtual ~LaneKeepingSystem();

    /**
     * @brief Run Lane Keeping System
     */
    void run();

private:
    /**
     * @brief Set the parameters from config file
     *
     * @param[in] config Configuration for searching and keeping Hough lines using Hough, Moving average and PID control
     */
    void setParams(const YAML::Node& config);

    /**
     * @brief Control the speed of xycar
     *
     * @param[in] steeringAngle Angle to steer xycar. If over max angle, deaccelerate, otherwise accelerate
     */
    void speedControl(PREC steeringAngle);

    /**
     * @brief publish the motor topic message
     *
     * @param[in] steeringAngle Angle to steer xycar actually
     */
    void drive(PREC steeringAngle);
    void findStopLine(cv::Mat& frame);
    void imageCallback(const sensor_msgs::Image& message);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void bboxClassCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& bbox_class_data);
    void yoloCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& predictions);
    std::string getfilename();

private:
    ControllerPtr mPID;                      ///< PID Class for Control
    FilterPtr mMovingAverage;                ///< Moving Average Filter Class for Noise filtering
    DetectorPtr mLaneDetector;

    // ROS Variables
    ros::NodeHandle mNodeHandler;          ///< Node Hanlder for ROS. In this case Detector and Controler
    ros::Publisher mPublisher;             ///< Publisher to send message about
    ros::Subscriber mSubscriber;           ///< Subscriber to receive image
    ros::Subscriber mSubscriberScan;
    ros::Subscriber mSubscriberBbox;
    ros::Subscriber mSubscriberBboxClass;
    std::string mPublishingTopicName;      ///< Topic name to publish
    std::string mSubscribedTopicName;      ///< Topic name to subscribe
    std::string mSubscribedTopicScan;
    std::string mSubscribedTopicYolo;
    uint32_t mQueueSize;                   ///< Max queue size for message
    xycar_msgs::xycar_motor mMotorMessage; ///< Message for the motor of xycar

    // OpenCV Image processing Variables
    cv::Mat mFrame; ///< Image from camera. The raw image is converted into cv::Mat
    std::vector<float> mScan;
    int mLidarLastObsPos = -1;
    std::vector<int> mNearestPred;
    int mLastPredClass = -1;
    // bool mStopLine = false;
    bool mStopLine;

    // Xycar Device variables
    PREC mXycarSpeed;                 ///< Current speed of xycar
    PREC mXycarMaxSpeed;              ///< Max speed of xycar
    PREC mXycarMinSpeed;              ///< Min speed of xycar
    PREC mXycarSpeedControlThreshold; ///< Threshold of angular of xycar
    PREC mAccelerationStep;           ///< How much would accelrate xycar depending on threshold
    PREC mDecelerationStep;           ///< How much would deaccelrate xycar depending on threshold
    PREC temp_mDecelerationStep;

    PREC mFrontObsCnt;
    PREC mLeftObsCnt;
    PREC mRightObsCnt;
    PREC mLidarSideDepth;
    PREC mLidarSideObsThresh;
    PREC mLidarFrontAngle;
    PREC mLidarFrontDepth;
    PREC mLidarFrontObsThresh;

    PREC mObjXDepthThresh;
    PREC mObjXYDepthThresh;

    // cv::VideoWriter outputVideo;
    // Debug Flag
    bool mDebugging = false; ///< Debugging or not
};
} // namespace Xycar

#endif // LANE_KEEPING_SYSTEM_HPP_
