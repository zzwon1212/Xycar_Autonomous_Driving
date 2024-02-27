#include "autonomous_driving/driving.h"

namespace xycar
{
template <typename PREC>
Driving<PREC>::Driving()
{
    std::string config_path;
//     mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(config_path);

    LaneDetector_ = new LaneDetector<PREC>(config);
    PID_ = new PIDController<PREC>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
//     // mMovingAverage = new MovingAverageFilter<PREC>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());

//     // setParams(config);

//     // mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
//     mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
//     mSubscriberScan = mNodeHandler.subscribe(mSubscribedTopicScan, mQueueSize, &LaneKeepingSystem::scanCallback, this);
//     // mSubscriberBboxClass = mNodeHandler.subscribe(mSubscribedTopicYolo, mQueueSize, &LaneKeepingSystem::yoloCallback, this);
}

template <typename PREC>
void Driving<PREC>::SetParams(const YAML::Node& config)
{
// //     mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
// //     mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
// //     mSubscribedTopicScan = config["TOPIC"]["SUB_SCAN_NAME"].as<std::string>();
// //     mSubscribedTopicYolo = config["TOPIC"]["SUB_YOLO_NAME"].as<std::string>();
// //     mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
// //     mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
// //     mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
// //     mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
// //     mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
// //     mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    deceleration_step_ = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
// //     mDebugging = config["DEBUG"].as<bool>();

// //     mLidarSideDepth = config["LIDAR"]["SIDE_DEPTH"].as<PREC>();
// //     mLidarSideObsThresh = config["LIDAR"]["SIDE_OBS_THRESH"].as<PREC>();
// //     mLidarFrontAngle = config["LIDAR"]["FRONT_ANGLE"].as<PREC>();
// //     mLidarFrontDepth = config["LIDAR"]["FRONT_DEPTH"].as<PREC>();
// //     mLidarFrontObsThresh = config["LIDAR"]["FRONT_OBS_THRESH"].as<PREC>();

// //     mObjXDepthThresh = config["OBJECT"]["X_DEPTH"].as<PREC>();
// //     mObjXYDepthThresh = config["OBJECT"]["XY_DEPTH"].as<PREC>();
}

// template <typename PREC>
// LaneKeepingSystem<PREC>::~LaneKeepingSystem()
// {
//     // delete mPID;
//     // delete mMovingAverage;
//     delete mLaneDetector;
//     // delete your LaneDetector if you add your LaneDetector.
// }

template <typename PREC>
void Driving<PREC>::run()
{
    ros::Rate rate(kFPS);
    // int isFirstFrame = True;

    while (ros::ok()) {
        ros::spinOnce();

        // Wait until receiving image
        if (frame_.empty()) {
            continue;
        }

        // Wait until object detection starts
        if (nearest_object_.empty()) {
            continue;
        }

        // Get appropriate steering angle to drive at current frame
        double lane_center;
        bool is_left_detected, is_right_detected;
        std::tie(lane_center, is_left_detected, is_right_detected) = LaneDetector_ -> GetLaneInfo(frame_);
        double gap = (lane_center - frame_.cols/2);  // @@@@@@@@@@@@@@@@ TODO: Any error?
        auto steering_angle = std::max(std::min(kSteeringAngleLimit, (int32_t) PID_ -> GetPIDOutput(gap)), -1 * kSteeringAngleLimit);
        tmp_deceleration_step_ = std::round(std::abs(gap) / 10) * deceleration_step_;

        xycar_msgs::xycar_motor motorMessage;

        ros::Time end_time;

//         if (mFrontObsCnt > mLidarFrontObsThresh) {
//             // std::cout << "FRONT OBSTALCE, " << mFrontObsCnt << std::endl;
//             // motorMessage.speed = 0.0;
//             // mPublisher.publish(motorMessage);
//             continue;
//         } else if (mLeftObsCnt > mLidarSideObsThresh || mRightObsCnt > mLidarSideObsThresh) {
//             // std::cout << "SIDE OBSTACLE, " << mLeftObsCnt << std::endl;
//             end_time = ros::Time::now() + ros::Duration(1.2);
//             while (ros::Time::now() < end_time) {
//                 // motorMessage.angle = (2*(mLeftObsCnt > mLidarSideObsThresh) - 1) * 50.0;
//                 // motorMessage.speed = mXycarSpeed;
//                 // mPublisher.publish(motorMessage);
//             }
//             mLidarLastObsPos = (mRightObsCnt > mLidarSideObsThresh);  // left obs -> right avoid
//             continue;
//         }


//         int depth = sqrt(mNearestPred[6]*mNearestPred[6] + mNearestPred[7]*mNearestPred[7]);
//         // std::cout << "Last: " << mLastPredClass << ", Class: " << mNearestPred[5] << ", x: " << mNearestPred[6] << ", y: " << mNearestPred[7] << ", depth: " << depth << std::endl;

//         findStopLine(mFrame);  // Check whether there is stopline.
//         if (mStopLine) {
//             // std::cout << "STOPLINE" << std::endl;

//             // LEFT, RIGHT
//             if (mNearestPred[5] == 0 || mNearestPred[5] == 1) {
//                 // std::cout << "STOP and TURN" << std::endl;

//                 // motorMessage.speed = 0.0;
//                 // mPublisher.publish(motorMessage);
//                 ros::Duration(2).sleep();

//                 end_time = ros::Time::now() + ros::Duration(2);
//                 while (ros::Time::now() < end_time) {
//                     // motorMessage.angle = 7.5;
//                     // motorMessage.speed = mXycarSpeed;
//                     // mPublisher.publish(motorMessage);
//                 }

//                 if (mNearestPred[5] == 0) {
//                     end_time = ros::Time::now() + ros::Duration(3);
//                     while (ros::Time::now() < end_time) {
//                         // motorMessage.angle = -35.0;
//                         // motorMessage.speed = mXycarSpeed;
//                         // mPublisher.publish(motorMessage);
//                     }
//                     mLastPredClass = 0;
//                 } else if (mNearestPred[5] == 1) {
//                     end_time = ros::Time::now() + ros::Duration(3);
//                     while (ros::Time::now() < end_time) {
//                         // motorMessage.angle = 50.0;
//                         // motorMessage.speed = mXycarSpeed;
//                         // mPublisher.publish(motorMessage);
//                     }
//                     mLastPredClass = 1;
//                 }

//             // CROSSWALK, STOPSIGN
//             } else if (mNearestPred[5] == 2 || mNearestPred[5] == 3) {
//                 // std::cout << "STOP and GO" << std::endl;
//                 // motorMessage.speed = 0.0;
//                 // mPublisher.publish(motorMessage);
//                 ros::Duration(2).sleep();

//                 end_time = ros::Time::now() + ros::Duration(1);
//                 while (ros::Time::now() < end_time) {
//                     // motorMessage.speed = mXycarSpeed;
//                     // mPublisher.publish(motorMessage);
//                 }


//             // RED LIGHT
//             } else if (mNearestPred[5] == 5) {
//                 // std::cout << "RED LIGHT" << std::endl;
//                 // motorMessage.speed = 0.0;
//                 // mPublisher.publish(motorMessage);
//                 ros::Duration(1).sleep();

//             // GREEN LIGHT
//             } else if (mNearestPred[5] == 6) {
//                 // std::cout << "GREEN LIGHT" << std::endl;
//                 end_time = ros::Time::now() + ros::Duration(1);
//                 while (ros::Time::now() < end_time) {
//                     // motorMessage.speed = mXycarSpeed;
//                     // mPublisher.publish(motorMessage);
//                 }

//             // NO TRAFFIC SIGN
//             } else {
//                 // std::cout << "ONLY STOPLINE" << std::endl;
//                 // motorMessage.speed = 0.0;
//                 // mPublisher.publish(motorMessage);
//                 ros::Duration(1).sleep();
//             }

//             continue;
//         }

//         // If there is no object or object is far enough and,
//         // both lanes are not detected,
//         if ((mNearestPred[5] == -1 || depth >= mObjXYDepthThresh) &&
//             (is_left_detected && is_right_detected)) {
//             mLastPredClass = -1;
//             mLidarLastObsPos = -1;
//         }

//         // If there was obstacle detected by LIDAR,
//         if (mLidarLastObsPos == 0 || mLidarLastObsPos == 1) {
//             // std::cout << "0. LIDAR TURN" << std::endl;
//             // motorMessage.angle = (2*mLidarLastObsPos - 1) * 40.0;
//             // motorMessage.speed = mXycarSpeed;
//             // mPublisher.publish(motorMessage);
//             continue;
//         }

//         // If car remembers last turn sign,
//         if (mLastPredClass == 0 || mLastPredClass == 1) {
//             // If new turn sign is opposite to last turn sign and is near enough,
//             if ((mNearestPred[5] != -1 && mNearestPred[5] != mLastPredClass) &&
//                 (mNearestPred[6] < 20 && depth < mObjXYDepthThresh)) {
//                 // std::cout << "1. (NEW SIGN != LAST SIGN) TURN" << std::endl;
//                 // motorMessage.angle = (2*mNearestPred[5] - 1) * 50.0;
//                 mLastPredClass = mNearestPred[5];
//             } else {
//                 // std::cout << "2. (LAST SIGN) TURN" << std::endl;
//                 // motorMessage.angle = (2*mLastPredClass - 1) * 50.0;
//             }
//             // motorMessage.speed = mXycarSpeed;
//             // mPublisher.publish(motorMessage);
//             continue;
//         }

//         // After detecting both lanes, if new turn sign is detected,
//         // turn within current frame.
//         if (depth < mObjXYDepthThresh) {
//             if (mNearestPred[5] == 0 || mNearestPred[5] == 1) {
//                 // std::cout << "3. (NEW SIGN) TURN" << std::endl;
//                 // motorMessage.angle = (2*mNearestPred[5] - 1) * 50.0;
//                 // motorMessage.speed = mXycarSpeed;
//                 // mPublisher.publish(motorMessage);
//                 mLastPredClass = mNearestPred[5];
//                 continue;
//             }
//         }

//         // Drive only using lanes if there are no other situations.
//         // drive(steering_angle);

//         if (mDebugging) {
//             // cv::imshow("frame", mFrame);
//             // cv::imshow("Debug", mLaneDetector->getDebugFrame());
//             // cv::waitKey(1);
//         }

        // isFirstFrame = false;
    }
}

// template <typename PREC>
// void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
// {
//     cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
//     cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
// }

// template <typename PREC>
// void LaneKeepingSystem<PREC>::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
// {
//     mScan = scan -> ranges;

//     int front_idx = ((mLidarFrontAngle * 0.5) / 360) * mScan.size();  // 45 degree
//     int side_idx = 0.25 * mScan.size();  // 90 degree
//     mFrontObsCnt = 0;
//     mLeftObsCnt = 0;
//     mRightObsCnt = 0;
//     for (size_t i = 0; i < mScan.size(); ++i) {

//         if (((i < front_idx) || (i > mScan.size() - front_idx)) &&
//             ((mScan[i] > 0.01) && (mScan[i] < mLidarFrontDepth))) {
//             mFrontObsCnt++;
//         }

//         if ((i < side_idx) &&
//             ((mScan[i] > 0.01) && (mScan[i] < mLidarSideDepth))) {
//             mLeftObsCnt++;
//         }

//         if ((i > mScan.size() - side_idx) &&
//             ((mScan[i] > 0.01) && (mScan[i] < mLidarSideDepth))) {
//             mRightObsCnt++;
//         }
//     }
// }

// // template <typename PREC>
// // void LaneKeepingSystem<PREC>::yoloCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& preds)
// // {
// //     mNearestPred.assign(8, -1);

// //     for (const auto& pred : preds -> bounding_boxes) {
// //         if (pred.id == 4) {
// //             continue;
// //         }
// //         if ((mNearestPred[5] == -1) || (mNearestPred[7] > pred.ydepth)) {
// //             mNearestPred[0] = pred.probability;
// //             mNearestPred[1] = pred.xmin;
// //             mNearestPred[2] = pred.ymin;
// //             mNearestPred[3] = pred.xmax;
// //             mNearestPred[4] = pred.ymax;
// //             mNearestPred[5] = pred.id;
// //             mNearestPred[6] = pred.xdepth;
// //             mNearestPred[7] = pred.ydepth;
// //         }
// //     }
// // }

// // template <typename PREC>
// // void LaneKeepingSystem<PREC>::speedControl(PREC steering_angle)
// // {
// //     if (std::abs(steering_angle) > mXycarSpeedControlThreshold)
// //     {
// //         mXycarSpeed -= tmp_deceleration_step_;
// //         mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
// //         return;
// //     }

// //     mXycarSpeed += mAccelerationStep;
// //     mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
// // }

// // template <typename PREC>
// // void LaneKeepingSystem<PREC>::drive(PREC steering_angle)
// // {
// //     xycar_msgs::xycar_motor motorMessage;
// //     motorMessage.angle = std::round(steering_angle);
// //     speedControl(steering_angle);
// //     mLaneDetector->setYOffset(mXycarSpeed);
// //     motorMessage.speed = std::round(mXycarSpeed);
// //     mPublisher.publish(motorMessage);
// // }

// template <typename PREC>
// std::string LaneKeepingSystem<PREC>::getfilename(){
//     std::string str_buf;
//     time_t curTime = time(NULL);
//     struct tm* pLocal = localtime(&curTime);

//     str_buf="/home/nvidia/xycar_ws/src/LaneKeepingSystem/src/LaneKeepingSystem/"+std::to_string(pLocal->tm_year + 1900)+std::to_string(pLocal->tm_mon + 1)+std::to_string(pLocal->tm_mday)+ "_" + std::to_string(pLocal->tm_hour) + std::to_string(pLocal->tm_min) + std::to_string(pLocal->tm_sec)+".mp4";
//     return str_buf;
// }

// template <typename PREC>
// void LaneKeepingSystem<PREC>::findStopLine(cv::Mat& frame)
// {
//     cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 352.494189, 0.000000, 295.823760,
//                                                       0.000000, 353.504572, 239.649689,
//                                                       0.000000, 0.000000, 1.000000);
//     cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.318744, 0.088199, 0.000167, 0.000699, 0.000000);

//     cv::Mat map1, map2, undistorted;
//     cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::Mat(), cv::Size(frame.cols, frame.rows), CV_8UC1, map1, map2);
//     cv::remap(frame, undistorted, map1, map2, cv::INTER_LINEAR);

//     cv::Mat grayImg, blurImg, binImg, edgeImg, outputImg;
//     cv::Rect roiRect(140, 370, 360, 40);
//     cv::Mat croppedImg = undistorted(roiRect);
//     std::vector<cv::Vec4f> lines;
//     cv::cvtColor(croppedImg, grayImg, cv::COLOR_BGR2GRAY);
//     cv::GaussianBlur(grayImg, blurImg, cv::Size(), 1.0);
//     cv::threshold(blurImg, binImg, 190, 255, cv::THRESH_BINARY_INV);
//     cv::Canny(binImg, edgeImg, 50, 150);
//     // cv::HoughLinesP(edgeImg, lines, 1, CV_PI / 180, 40, 180, 50);
//     cv::HoughLinesP(edgeImg, lines, 1, CV_PI / 180, 40, 160, 50);

//     for (size_t i = 0; i < lines.size(); i++) {
//         cv::Vec4i l = lines[i];
//         cv::line(croppedImg, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(rand() % 256, rand() % 256, rand() % 256), 2, cv::LINE_AA);
//     }

//     if (mDebugging) {
//         cv::imshow("bin", binImg);
//         cv::imshow("edge", edgeImg);
//         cv::imshow("hough", croppedImg);
//         cv::waitKey(1);
//     }

//     float slope;
//     if (lines.empty()) {
//         slope = -1.0;
//     } else {
//         slope = (lines[0][3] - lines[0][1]) / (lines[0][0] - lines[0][2] + 0.001);
//     }

//     if (abs(slope) < 0.025) {
//         mStopLine = true;
//     } else {
//         mStopLine = false;
//     }
// }

template class Driving<float>;
template class Driving<double>;
}  // namespace xycar
