#include "autonomous_driving/driving.h"

namespace xycar
{
template <typename PREC>
Driving<PREC>::Driving()
{
    std::string config_path;
    NodeHandler_.getParam("config_path", config_path);
    YAML::Node config = YAML::LoadFile(config_path);

    LaneDetector_ = new LaneDetector<PREC>(config);
    PID_ = new PIDController<PREC>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());

    // setParams(config);

    SubscriberImg_ = NodeHandler_.subscribe(sub_topic_img_, queue_size_, &Driving::imageCallback, this);
    SubscriberScan_ = NodeHandler_.subscribe(sub_topic_scan_, queue_size_, &Driving::scanCallback, this);
    SubscriberYOLO_ = NodeHandler_.subscribe(sub_topic_yolo_, queue_size_, &Driving::yoloCallback, this);
    PublisherMotor_ = NodeHandler_.advertise<xycar_msgs::xycar_motor>(pub_topic_motor_, queue_size_);
}

template <typename PREC>
void Driving<PREC>::setParams(const YAML::Node& config)
{
    // Topic
    queue_size_ = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    sub_topic_img_ = config["TOPIC"]["SUB_IMG_NAME"].as<std::string>();
    sub_topic_scan_ = config["TOPIC"]["SUB_SCAN_NAME"].as<std::string>();
    sub_topic_yolo_ = config["TOPIC"]["SUB_YOLO_NAME"].as<std::string>();
    pub_topic_motor_ = config["TOPIC"]["PUB_MOTOR_NAME"].as<std::string>();

    // Xycar
    xycar_speed_ = config["XYCAR"]["START_SPEED"].as<PREC>();
    xycar_speed_min_ = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    xycar_speed_max_ = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    xycar_speed_control_thresh_ = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    deceleration_step_ = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    acceleration_step_ = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();

    // LiDAR
    front_obs_angle_ = config["LIDAR"]["FRONT_ANGLE"].as<PREC>();
    front_obs_depth_ = config["LIDAR"]["FRONT_DEPTH"].as<PREC>();
    front_obs_cnt_thresh_ = config["LIDAR"]["FRONT_OBS_THRESH"].as<PREC>();
    side_obs_depth_ = config["LIDAR"]["SIDE_DEPTH"].as<PREC>();
    side_obs_cnt_thresh_ = config["LIDAR"]["SIDE_OBS_THRESH"].as<PREC>();

    // Object Detection
    obj_depth_thresh_ = config["OBJECT"]["XY_DEPTH"].as<PREC>();

    is_debugging_ = config["DEBUG"].as<bool>();
}

template <typename PREC>
Driving<PREC>::~Driving()
{
    delete PID_;
    delete LaneDetector_;
}

template <typename PREC>
void Driving<PREC>::run()
{
    ros::Rate rate(FPS);

    while (ros::ok())
    {
        ros::spinOnce();

        // Wait until receiving image
        if (frame_.empty())
        {
            continue;
        }

        // Wait until object detection starts
        if (nearest_object_.empty())
        {
            continue;
        }

        // Get appropriate steering angle to drive at current frame
        double lane_center;
        bool is_left_detected, is_right_detected;
        std::tie(lane_center, is_left_detected, is_right_detected) = LaneDetector_ -> getLaneInfo(frame_);
        double gap = (lane_center - frame_.cols/2);  // @@@@@@@@@@@@@@@@ TODO: Any error?
        auto steering_angle = std::max(std::min(STEERING_ANGLE_LIMIT,
                                       (int32_t) PID_ -> getPIDOutput(gap)),
                                       -1 * STEERING_ANGLE_LIMIT);
        tmp_deceleration_step_ = std::round(std::abs(gap) / 10) * deceleration_step_;


        xycar_msgs::xycar_motor motor_message;
        ros::Time end_time;


        // Stop while there is obstacle in front of the car.
        if (front_obs_cnt_ > front_obs_cnt_thresh_)
        {
            // std::cout << "FRONT OBSTALCE, " << front_obs_cnt_ << std::endl;
            motor_message.speed = 0.0;
            PublisherMotor_.publish(motor_message);
            continue;  // Skip to next frame.
        }
        // If there is obstacle at left or right side, avoid it (turn for fixed time).
        // Then, return until detecting both lanes. (-> Code at # lines)
        else if (left_obs_cnt_ > side_obs_cnt_thresh_ || right_obs_cnt_ > side_obs_cnt_thresh_)
        {
            // std::cout << "SIDE OBSTACLE, " << left_obs_cnt_ << std::endl;
            end_time = ros::Time::now() + ros::Duration(1.2);
            while (ros::Time::now() < end_time)
            {
                motor_message.angle = (2*(left_obs_cnt_ > side_obs_cnt_thresh_) - 1) * 50.0;
                motor_message.speed = xycar_speed_;
                PublisherMotor_.publish(motor_message);
            }

            // Remember current obstacle position to return later.
            last_obs_pos_ = (right_obs_cnt_ > side_obs_cnt_thresh_);  // left obs -> right avoid
            continue;  // Skip to next frame.
        }


        // @@@@@@@@@@@@@@@@ TODO: Make struct for nearest_object_? .x, .y, .class etc.
        int depth = sqrt(nearest_object_[6]*nearest_object_[6] + nearest_object_[7]*nearest_object_[7]);
        // std::cout << "Last: " << last_obj_class_ << ", Class: " << nearest_object_[5] << ", x: " << nearest_object_[6] << ", y: " << nearest_object_[7] << ", depth: " << depth << std::endl;

        findStopLine(frame_);  // Check whether there is stopline.
        if (is_stopline_)
        {
            // std::cout << "STOPLINE" << std::endl;

            // LEFT, RIGHT: Stop -> Escape stopline -> Turn -> Detecting lanes
            if (nearest_object_[5] == 0 || nearest_object_[5] == 1)
            {
                // std::cout << "STOP and TURN" << std::endl;

                // Stop for 2 seconds.
                motor_message.speed = 0.0;
                PublisherMotor_.publish(motor_message);
                ros::Duration(2).sleep();

                // Escape stopline.
                // Angle is given because of xycar's hardware flaw.
                end_time = ros::Time::now() + ros::Duration(2);
                while (ros::Time::now() < end_time)
                {
                    motor_message.angle = 7.5;
                    motor_message.speed = xycar_speed_;
                    PublisherMotor_.publish(motor_message);
                }

                // Turn for fixed time.
                // Code for detecting lanes is located # lines.
                if (nearest_object_[5] == 0)
                {
                    end_time = ros::Time::now() + ros::Duration(3);
                    while (ros::Time::now() < end_time)
                    {
                        motor_message.angle = -35.0;
                        motor_message.speed = xycar_speed_;
                        PublisherMotor_.publish(motor_message);
                    }

                    last_obj_class_ = 0;  // Remember current turn sign to detect lanes later.
                }
                else if (nearest_object_[5] == 1)
                {
                    end_time = ros::Time::now() + ros::Duration(3);
                    while (ros::Time::now() < end_time)
                    {
                        motor_message.angle = 50.0;
                        motor_message.speed = xycar_speed_;
                        PublisherMotor_.publish(motor_message);
                    }

                    last_obj_class_ = 1;  // Remember current turn sign to detect lanes later.
                }
            }
            // CROSSWALK, STOPSIGN: Stop -> Escape stopline
            else if (nearest_object_[5] == 2 || nearest_object_[5] == 3)
            {
                // std::cout << "STOP and GO" << std::endl;
                motor_message.speed = 0.0;
                PublisherMotor_.publish(motor_message);
                ros::Duration(2).sleep();

                end_time = ros::Time::now() + ros::Duration(1);
                while (ros::Time::now() < end_time)
                {
                    motor_message.speed = xycar_speed_;
                    PublisherMotor_.publish(motor_message);
                }
            }
            // RED LIGHT: Stop while traffic light is RED.
            else if (nearest_object_[5] == 5)
            {
                // std::cout << "RED LIGHT" << std::endl;
                motor_message.speed = 0.0;
                PublisherMotor_.publish(motor_message);
                ros::Duration(1).sleep();
            }
            // GREEN LIGHT: Escape stopline.
            else if (nearest_object_[5] == 6)
            {
                // std::cout << "GREEN LIGHT" << std::endl;
                end_time = ros::Time::now() + ros::Duration(1);
                while (ros::Time::now() < end_time)
                {
                    motor_message.speed = xycar_speed_;
                    PublisherMotor_.publish(motor_message);
                }
            }
            // NO TRAFFIC SIGN: Stop while there is no traffic sign but stopline.
            else
            {
                // std::cout << "ONLY STOPLINE" << std::endl;
                motor_message.speed = 0.0;
                PublisherMotor_.publish(motor_message);
                ros::Duration(1).sleep();
            }

            continue;  // Skip to next frame.
        }

        // If there is no object or object is far enough and,
        // both lanes are not detected,
        // initialize last information of obstacle and object.
        if ((nearest_object_[5] == -1 || depth >= obj_depth_thresh_) &&
            (is_left_detected && is_right_detected))
        {
            last_obs_pos_ = -1;
            last_obj_class_ = -1;
        }

        // If there was obstacle at last frame and the car avoided it, (-> Code at # lines)
        // return until detecting both lanes.
        if (last_obs_pos_ == 0 || last_obs_pos_ == 1)
        {
            // std::cout << "0. LIDAR TURN" << std::endl;
            motor_message.angle = (2*last_obs_pos_ - 1) * 40.0;
            motor_message.speed = xycar_speed_;
            PublisherMotor_.publish(motor_message);
            continue;  // Skip to next frame.
        }

        // If car remembers last turn sign,
        if (last_obj_class_ == 0 || last_obj_class_ == 1)
        {
            // If new turn sign is opposite to last turn sign and is near enough,
            // follow and remember new sign.
            if ((nearest_object_[5] != -1 && nearest_object_[5] != last_obj_class_) &&
                (nearest_object_[6] < 20 && depth < obj_depth_thresh_))
            {
                // std::cout << "1. (NEW SIGN != LAST SIGN) TURN" << std::endl;
                motor_message.angle = (2*nearest_object_[5] - 1) * 50.0;
                last_obj_class_ = nearest_object_[5];
            }
            // Follow last turn sign.
            else
            {
                // std::cout << "2. (LAST SIGN) TURN" << std::endl;
                motor_message.angle = (2*last_obj_class_ - 1) * 50.0;
            }
            motor_message.speed = xycar_speed_;
            PublisherMotor_.publish(motor_message);
            continue;  // Skip to next frame.
        }

        // After detecting both lanes, if new turn sign is detected,
        // just follow new turn sign.
        if (depth < obj_depth_thresh_) {
            if (nearest_object_[5] == 0 || nearest_object_[5] == 1) {
                // std::cout << "3. (NEW SIGN) TURN" << std::endl;
                motor_message.angle = (2*nearest_object_[5] - 1) * 50.0;
                motor_message.speed = xycar_speed_;
                PublisherMotor_.publish(motor_message);
                last_obj_class_ = nearest_object_[5];
                continue;
            }
        }

        // Drive only using lanes if there are no other situations.
        drive(steering_angle);

        if (is_debugging_) {
            cv::imshow("frame", frame_);
            cv::imshow("Debug", LaneDetector_ -> getDebugFrame());
            cv::waitKey(1);
        }
    }
}

template <typename PREC>
void Driving<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

template <typename PREC>
void Driving<PREC>::scanCallback(const sensor_msgs::LaserScan::ConstPtr& message)
{
    lidar_data_ = message -> ranges;

    int front_idx = ((front_obs_angle_ * 0.5) / 360) * lidar_data_.size();  // 45 degree
    int side_idx = 0.25 * lidar_data_.size();  // 90 degree

    front_obs_cnt_ = 0;
    left_obs_cnt_ = 0;
    right_obs_cnt_ = 0;

    for (size_t i = 0; i < lidar_data_.size(); ++i)
    {
        if (((i < front_idx) || (i > lidar_data_.size() - front_idx)) &&
            ((lidar_data_[i] > 0.01) && (lidar_data_[i] < front_obs_depth_)))
        {
            front_obs_cnt_++;
        }

        if ((i < side_idx) &&
            ((lidar_data_[i] > 0.01) && (lidar_data_[i] < side_obs_depth_)))
        {
            left_obs_cnt_++;
        }

        if ((i > lidar_data_.size() - side_idx) &&
            ((lidar_data_[i] > 0.01) && (lidar_data_[i] < side_obs_depth_)))
        {
            right_obs_cnt_++;
        }
    }
}

template <typename PREC>
void Driving<PREC>::yoloCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& predictions)
{
    nearest_object_.assign(8, -1);

    for (const auto& pred : predictions -> bounding_boxes)
    {
        if (pred.id == 4)
        {
            continue;  // Skip if class is "car" which is unnecessary class.
        }

        if ((nearest_object_[5] == -1) || (nearest_object_[7] > pred.ydepth))
        {
            nearest_object_[0] = pred.probability;
            nearest_object_[1] = pred.xmin;
            nearest_object_[2] = pred.ymin;
            nearest_object_[3] = pred.xmax;
            nearest_object_[4] = pred.ymax;
            nearest_object_[5] = pred.id;
            nearest_object_[6] = pred.xdepth;
            nearest_object_[7] = pred.ydepth;
        }
    }
}

template <typename PREC>
void Driving<PREC>::controlSpeed(PREC steering_angle)
{
    if (std::abs(steering_angle) > xycar_speed_control_thresh_)
    {
        xycar_speed_ -= tmp_deceleration_step_;
        xycar_speed_ = std::max(xycar_speed_, xycar_speed_min_);
        return;
    }

    xycar_speed_ += acceleration_step_;
    xycar_speed_ = std::min(xycar_speed_, xycar_speed_max_);
}

template <typename PREC>
void Driving<PREC>::drive(PREC steering_angle)
{
    xycar_msgs::xycar_motor motor_message;
    motor_message.angle = std::round(steering_angle);
    controlSpeed(steering_angle);
    LaneDetector_ -> setYOffset(xycar_speed_);
    motor_message.speed = std::round(xycar_speed_);
    PublisherMotor_.publish(motor_message);
}

// template <typename PREC>
// std::string LaneKeepingSystem<PREC>::getfilename(){
//     std::string str_buf;
//     time_t curTime = time(NULL);
//     struct tm* pLocal = localtime(&curTime);

//     str_buf="/home/nvidia/xycar_ws/src/LaneKeepingSystem/src/LaneKeepingSystem/"+std::to_string(pLocal->tm_year + 1900)+std::to_string(pLocal->tm_mon + 1)+std::to_string(pLocal->tm_mday)+ "_" + std::to_string(pLocal->tm_hour) + std::to_string(pLocal->tm_min) + std::to_string(pLocal->tm_sec)+".mp4";
//     return str_buf;
// }

template <typename PREC>
void Driving<PREC>::findStopLine(cv::Mat& frame)
{
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 352.494189, 0.000000,   295.823760,
                                                       0.000000,   353.504572, 239.649689,
                                                       0.000000,   0.000000,   1.000000);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -0.318744, 0.088199, 0.000167, 0.000699, 0.000000);
    cv::Mat map1, map2, undistorted;
    cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), cv::Mat(),
                                cv::Size(frame.cols, frame.rows), CV_8UC1, map1, map2);
    cv::remap(frame, undistorted, map1, map2, cv::INTER_LINEAR);

    cv::Mat img_gray, img_blur, img_bin, img_edge;
    cv::Rect roiRect(140, 370, 360, 40);
    cv::Mat img_cropped = undistorted(roiRect);
    std::vector<cv::Vec4f> lines;
    cv::cvtColor(img_cropped, img_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(img_gray, img_blur, cv::Size(), 1.0);
    cv::threshold(img_blur, img_bin, 190, 255, cv::THRESH_BINARY_INV);
    cv::Canny(img_bin, img_edge, 50, 150);
    cv::HoughLinesP(img_edge, lines, 1, CV_PI/180, 40, 160, 50);

    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::line(img_cropped, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                 cv::Scalar(rand() % 256, rand() % 256, rand() % 256), 2, cv::LINE_AA);
    }

    if (is_debugging_)
    {
        cv::imshow("bin", img_bin);
        cv::imshow("edge", img_edge);
        cv::imshow("hough", img_cropped);
        cv::waitKey(1);
    }

    float slope;
    if (lines.empty())
    {
        slope = -1.0;
    }
    else
    {
        slope = (lines[0][3] - lines[0][1]) / (lines[0][0] - lines[0][2] + 0.001);
    }

    if (abs(slope) < 0.025)
    {
        is_stopline_ = true;
    }
    else
    {
        is_stopline_ = false;
    }
}

template class Driving<float>;
template class Driving<double>;
}  // namespace xycar
