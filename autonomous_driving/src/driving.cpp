#include "autonomous_driving/driving.h"

namespace xycar
{
Driving::Driving()
{
    std::string config_path;
    NodeHandler_.getParam("config_path", config_path);
    YAML::Node config = YAML::LoadFile(config_path);

    getConfig(config);
    LaneDetector_ = new LaneDetector(config);
    PID_ = new PIDController(PID_P_, PID_I_, PID_D_);
    Tools_ = new Tools(config);

    SubscriberImg_ = NodeHandler_.subscribe(SUB_TOPIC_IMG_, QUEUE_SIZE_, &Driving::imageCallback, this);
    SubscriberScan_ = NodeHandler_.subscribe(SUB_TOPIC_SCAN_, QUEUE_SIZE_, &Driving::scanCallback, this);
    SubscriberYOLO_ = NodeHandler_.subscribe(SUB_TOPIC_YOLO_, QUEUE_SIZE_, &Driving::yoloCallback, this);
    PublisherMotor_ = NodeHandler_.advertise<xycar_msgs::xycar_motor>(PUB_TOPIC_MOTOR_, QUEUE_SIZE_);
}

void Driving::getConfig(const YAML::Node& config)
{
    // Topic
    QUEUE_SIZE_ = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    SUB_TOPIC_IMG_ = config["TOPIC"]["SUB_IMG_NAME"].as<std::string>();
    SUB_TOPIC_SCAN_ = config["TOPIC"]["SUB_SCAN_NAME"].as<std::string>();
    SUB_TOPIC_YOLO_ = config["TOPIC"]["SUB_YOLO_NAME"].as<std::string>();
    PUB_TOPIC_MOTOR_ = config["TOPIC"]["PUB_MOTOR_NAME"].as<std::string>();

    // Xycar
    XYCAR_SPEED_ = config["XYCAR"]["START_SPEED"].as<float>();
    XYCAR_SPEED_MIN_ = config["XYCAR"]["MIN_SPEED"].as<float>();
    XYCAR_SPEED_MAX_ = config["XYCAR"]["MAX_SPEED"].as<float>();
    XYCAR_SPEED_CONTROL_THRESH_ = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();
    DECELERATION_STEP_ = config["XYCAR"]["DECELERATION_STEP"].as<float>();
    ACCELERATION_STEP_ = config["XYCAR"]["ACCELERATION_STEP"].as<float>();

    // PID
    PID_P_ = config["PID"]["P_GAIN"].as<float>();
    PID_I_ = config["PID"]["I_GAIN"].as<float>();
    PID_D_ = config["PID"]["D_GAIN"].as<float>();

    // LiDAR
    FRONT_OBS_ANGLE_ = config["LIDAR"]["FRONT_ANGLE"].as<float>();
    FRONT_OBS_DEPTH_ = config["LIDAR"]["FRONT_DEPTH"].as<float>();
    SIDE_OBS_DEPTH_ = config["LIDAR"]["SIDE_DEPTH"].as<float>();
    FRONT_OBS_CNT_THRESH_ = config["LIDAR"]["FRONT_OBS_THRESH"].as<uint16_t>();
    SIDE_OBS_CNT_THRESH_ = config["LIDAR"]["SIDE_OBS_THRESH"].as<uint16_t>();

    // Object Detection
    OBJ_DEPTH_THRESH_ = config["OBJECT"]["XY_DEPTH"].as<float>();

    IS_DEBUGGING_ = config["DEBUG"].as<bool>();
}

Driving::~Driving()
{
    delete PID_;
    delete LaneDetector_;
    delete Tools_;
}

void Driving::run()
{
    // cv::VideoWriter output(
    //     "/workspace/Programmers/xycar_ws/src/xycar.mp4",
    //     cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
    //     FPS, cv::Size(640, 480));

    // if (!output.isOpened())
    // {
    //     std::cout << "VideoWriter fails to open!" << std::endl;
    // }

    ros::Rate rate(FPS);

    bool is_first_frame = true;

    while (ros::ok())
    {
        ros::spinOnce();

        // Wait until receiving image
        if (frame_.empty())
        {
            continue;
        }

        // Wait until object detection starts
        if (SubscriberYOLO_.getNumPublishers() != 1)
        {
            continue;
        }

        // Get appropriate steering angle to drive at current frame
        std::pair<float, float> lanes_position;
        std::pair<bool, bool> is_each_lane_detected;
        std::tie(lanes_position, is_each_lane_detected) = LaneDetector_->getLaneInfo(frame_);

        float center_lane_position = (lanes_position.first + lanes_position.second) * 0.5 + 30;
        bool is_left_detected, is_right_detected;
        std::tie(is_left_detected, is_right_detected) = is_each_lane_detected;

        float gap = (center_lane_position - frame_.cols * 0.5);
        float steering_angle = PID_->getPIDOutput(gap);
        steering_angle = (steering_angle > 50.0) ? 50.0 : (steering_angle < -50.0) ? -50.0 : steering_angle;
        tmp_deceleration_step_ = std::round(std::abs(gap) * 0.1) * DECELERATION_STEP_;


        cv::Mat frame_undistorted;
        Tools_->undistortImg(frame_, frame_undistorted);

        bool is_stopline;
        std::vector<cv::Vec4f> stoplines;
        Tools_->isStopline(frame_undistorted, is_stopline, stoplines);

        if (IS_DEBUGGING_)
        {
            cv::Mat img_result = frame_undistorted.clone();
            // Tools_->drawStoplines(img_result, stoplines);
            std::vector<cv::Point> undistorted_lanes_position = {cv::Point(60, 400), cv::Point(580, 400)};
            Tools_->undistortLanesPosition(lanes_position, LaneDetector_->moving_y_offset_, undistorted_lanes_position);
            Tools_->drawLanes(img_result, undistorted_lanes_position, is_first_frame);
            Tools_->drawBboxes(img_result, predictions_);

            // cv::imshow("Result", img_result);
            // cv::waitKey(1);

            // output << img_result;
        }

        xycar_msgs::xycar_motor motor_message;
        ros::Time end_time;

        // // WARNING: Need LiDAR rosbag!
        // // Stop while there is obstacle in front of the car.
        // if (front_obs_cnt_ > FRONT_OBS_CNT_THRESH_)
        // {
        //     std::cout << "FRONT OBSTALCE, " << front_obs_cnt_ << std::endl;
        //     motor_message.speed = 0.0;
        //     PublisherMotor_.publish(motor_message);
        //     rate.sleep();
        //     continue;  // Skip to next frame.
        // }
        // // If there is obstacle at left or right side, avoid it (turn for fixed time).
        // // Then, return until detecting both lanes. (-> Code at # lines)
        // else if (left_obs_cnt_ > SIDE_OBS_CNT_THRESH_ || right_obs_cnt_ > SIDE_OBS_CNT_THRESH_)
        // {
        //     std::cout << "SIDE OBSTACLE, " << left_obs_cnt_ << std::endl;
        //     end_time = ros::Time::now() + ros::Duration(1.2);
        //     while (ros::Time::now() < end_time)
        //     {
        //         motor_message.angle = (2*(left_obs_cnt_ > SIDE_OBS_CNT_THRESH_) - 1) * 50.0;
        //         motor_message.speed = XYCAR_SPEED_;
        //         PublisherMotor_.publish(motor_message);
        //     }

        //     // Remember current obstacle position to return later.
        //     last_obs_pos_ = (right_obs_cnt_ > SIDE_OBS_CNT_THRESH_);  // left obs -> right avoid
        //     rate.sleep();
        //     continue;  // Skip to next frame.
        // }


        yolov3_trt_ros::BoundingBox closest_object;
        float depth;
        Tools_->getClosestObject(predictions_, closest_object, depth);

        // Check whether there is stopline.
        if (is_stopline)
        {
            std::cout << "STOPLINE" << std::endl;

            // LEFT, RIGHT: Stop -> Escape stopline -> Turn -> Detecting lanes
            if (closest_object.id == 0 || closest_object.id == 1)
            {
                std::cout << "STOP and TURN" << std::endl;

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
                    motor_message.speed = XYCAR_SPEED_;
                    PublisherMotor_.publish(motor_message);
                }

                // Turn for fixed time.
                // Code for detecting lanes is located # lines.
                if (closest_object.id == 0)
                {
                    end_time = ros::Time::now() + ros::Duration(3);
                    while (ros::Time::now() < end_time)
                    {
                        motor_message.angle = -35.0;
                        motor_message.speed = XYCAR_SPEED_;
                        PublisherMotor_.publish(motor_message);
                    }

                    last_obj_class_ = 0;  // Remember current turn sign to detect lanes later.
                }
                else if (closest_object.id == 1)
                {
                    end_time = ros::Time::now() + ros::Duration(3);
                    while (ros::Time::now() < end_time)
                    {
                        motor_message.angle = 50.0;
                        motor_message.speed = XYCAR_SPEED_;
                        PublisherMotor_.publish(motor_message);
                    }

                    last_obj_class_ = 1;  // Remember current turn sign to detect lanes later.
                }
            }
            // CROSSWALK, STOPSIGN: Stop -> Escape stopline
            else if (closest_object.id == 2 || closest_object.id == 3)
            {
                std::cout << "STOP and GO" << std::endl;
                motor_message.speed = 0.0;
                PublisherMotor_.publish(motor_message);
                ros::Duration(2).sleep();

                end_time = ros::Time::now() + ros::Duration(1);
                while (ros::Time::now() < end_time)
                {
                    motor_message.speed = XYCAR_SPEED_;
                    PublisherMotor_.publish(motor_message);
                }
            }
            // RED LIGHT: Stop while traffic light is RED.
            else if (closest_object.id == 5)
            {
                std::cout << "RED LIGHT" << std::endl;
                motor_message.speed = 0.0;
                PublisherMotor_.publish(motor_message);
                // ros::Duration(1).sleep();
            }
            // GREEN LIGHT: Escape stopline.
            else if (closest_object.id == 6)
            {
                std::cout << "GREEN LIGHT" << std::endl;
                end_time = ros::Time::now() + ros::Duration(1);
                while (ros::Time::now() < end_time)
                {
                    motor_message.speed = XYCAR_SPEED_;
                    PublisherMotor_.publish(motor_message);
                }
            }
            // NO TRAFFIC SIGN: Stop while there is no traffic sign but stopline.
            else
            {
                std::cout << "ONLY STOPLINE" << std::endl;
                motor_message.speed = 0.0;
                PublisherMotor_.publish(motor_message);
                // ros::Duration(1).sleep();
            }

            rate.sleep();
            continue;  // Skip to next frame.
        }

        // If there is no object or object is far enough and,
        // both lanes are not detected,
        // initialize last information of obstacle and object.
        if ((closest_object.id == -1 || depth >= OBJ_DEPTH_THRESH_) &&
            (is_left_detected && is_right_detected))
        {
            // // last_obs_pos_ = -1; // WARNING: Need LiDAR rosbag!
            last_obj_class_ = -1;
        }

        // // WARNING: Need LiDAR rosbag!
        // // If there was obstacle at last frame and the car avoided it, (-> Code at # lines)
        // // return until detecting both lanes.
        // if (last_obs_pos_ == 0 || last_obs_pos_ == 1)
        // {
        //     std::cout << "0. LIDAR TURN" << std::endl;
        //     motor_message.angle = (2*last_obs_pos_ - 1) * 40.0;
        //     motor_message.speed = XYCAR_SPEED_;
        //     PublisherMotor_.publish(motor_message);
        //     rate.sleep();
        //     continue;  // Skip to next frame.
        // }

        // If car remembers last turn sign,
        if (last_obj_class_ == 0 || last_obj_class_ == 1)
        {
            // If new turn sign is opposite to last turn sign and is near enough,
            // follow and remember new sign.
            if ((closest_object.id != -1 && closest_object.id != last_obj_class_) &&
                (closest_object.xdepth < 20 && depth < OBJ_DEPTH_THRESH_))
            {
                std::cout << "1. (NEW SIGN != LAST SIGN) TURN" << std::endl;
                motor_message.angle = (2*closest_object.id - 1) * 50.0;
                last_obj_class_ = closest_object.id;
            }
            // Follow last turn sign.
            else
            {
                std::cout << "2. (LAST SIGN) TURN" << std::endl;
                motor_message.angle = (2*last_obj_class_ - 1) * 50.0;
            }
            motor_message.speed = XYCAR_SPEED_;
            PublisherMotor_.publish(motor_message);

            rate.sleep();
            continue;  // Skip to next frame.
        }

        // After detecting both lanes, if new turn sign is detected,
        // just follow new turn sign.
        if (depth < OBJ_DEPTH_THRESH_)
        {
            if (closest_object.id == 0 || closest_object.id == 1)
            {
                std::cout << "3. (NEW SIGN) TURN" << std::endl;
                motor_message.angle = (2*closest_object.id - 1) * 50.0;
                motor_message.speed = XYCAR_SPEED_;
                PublisherMotor_.publish(motor_message);
                last_obj_class_ = closest_object.id;

                rate.sleep();
                continue;
            }
        }

        // Drive only using lanes if there are no other situations.
        drive(steering_angle);

        is_first_frame = false;

        rate.sleep();
    }

    // output.release();
    // cv::destroyAllWindows();
}

void Driving::imageCallback(const sensor_msgs::Image::ConstPtr& message)
{
    cv::Mat src = cv::Mat(message->height, message->width, CV_8UC3,
                          const_cast<uint8_t*>(&message->data[0]), message->step);
    cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

void Driving::scanCallback(const sensor_msgs::LaserScan::ConstPtr& message)
{
    lidar_data_ = message->ranges;  // size = 505

    uint16_t front_idx = static_cast<uint16_t>(((FRONT_OBS_ANGLE_ * 0.5) / 360) * 505);  // 45 degree
    uint16_t side_idx = static_cast<uint16_t>(0.25 * 505);  // 90 degree

    front_obs_cnt_ = 0;
    left_obs_cnt_ = 0;
    right_obs_cnt_ = 0;

    for (uint16_t i = 0; i < 505; ++i)
    {
        if (((i < front_idx) || (i > 505 - front_idx)) &&
            ((lidar_data_[i] > 0.01) && (lidar_data_[i] < FRONT_OBS_DEPTH_)))
        {
            front_obs_cnt_++;
        }

        if ((i < side_idx) &&
            ((lidar_data_[i] > 0.01) && (lidar_data_[i] < SIDE_OBS_DEPTH_)))
        {
            left_obs_cnt_++;
        }

        if ((i > 505 - side_idx) &&
            ((lidar_data_[i] > 0.01) && (lidar_data_[i] < SIDE_OBS_DEPTH_)))
        {
            right_obs_cnt_++;
        }
    }
}

void Driving::yoloCallback(const yolov3_trt_ros::BoundingBoxes::ConstPtr& message)
{
    predictions_ = *message;
}

void Driving::controlSpeed(const float steering_angle)
{
    if (std::abs(steering_angle) > XYCAR_SPEED_CONTROL_THRESH_)
    {
        XYCAR_SPEED_ -= tmp_deceleration_step_;
        XYCAR_SPEED_ = std::max(XYCAR_SPEED_, XYCAR_SPEED_MIN_);
        return;
    }

    XYCAR_SPEED_ += ACCELERATION_STEP_;
    XYCAR_SPEED_ = std::min(XYCAR_SPEED_, XYCAR_SPEED_MAX_);
}

void Driving::drive(const float steering_angle)
{
    xycar_msgs::xycar_motor motor_message;
    motor_message.angle = std::round(steering_angle);
    controlSpeed(steering_angle);
    LaneDetector_->setYOffset(XYCAR_SPEED_);
    motor_message.speed = std::round(XYCAR_SPEED_);
    PublisherMotor_.publish(motor_message);
}
}  // namespace xycar
