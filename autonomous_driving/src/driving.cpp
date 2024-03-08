#include "autonomous_driving/driving.h"

namespace xycar
{
Driving::Driving()
{
    std::string config_path;
    NodeHandler_.getParam("config_path", config_path);
    YAML::Node config = YAML::LoadFile(config_path);

    LaneDetector_ = new LaneDetector(config);
    PID_ = new PIDController(config["PID"]["P_GAIN"].as<float>(),
                             config["PID"]["I_GAIN"].as<float>(),
                             config["PID"]["D_GAIN"].as<float>());
    getConfig(config);

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

    // LiDAR
    FRONT_OBS_ANGLE_ = config["LIDAR"]["FRONT_ANGLE"].as<float>();
    FRONT_OBS_DEPTH_ = config["LIDAR"]["FRONT_DEPTH"].as<float>();
    SIDE_OBS_DEPTH_ = config["LIDAR"]["SIDE_DEPTH"].as<float>();
    FRONT_OBS_CNT_THRESH_ = config["LIDAR"]["FRONT_OBS_THRESH"].as<uint16_t>();
    SIDE_OBS_CNT_THRESH_ = config["LIDAR"]["SIDE_OBS_THRESH"].as<uint16_t>();

    // Object Detection
    LABELS_ = config["OBJECT"]["LABELS"].as<std::vector<std::string>>();
    COLORS_ = {cv::Scalar(50, 200, 255),  // LEFT
               cv::Scalar(255, 0, 255),   // RIGHT
               cv::Scalar(255, 0, 0),     // CROSSWALK
               cv::Scalar(0, 0, 170),     // STOP SIGN
               cv::Scalar(0, 170, 0),     // CAR
               cv::Scalar(0, 0, 255),     // RED SIGN
               cv::Scalar(0, 100, 50),    // GREEN SIGN
               cv::Scalar(255, 255, 0)};  // YELLOW SIGN
    IMAGE_WIDTH_ = config["IMAGE"]["WIDTH"].as<uint16_t>();
    IMAGE_HEIGHT_ = config["IMAGE"]["HEIGHT"].as<uint16_t>();
    YOLO_RESOLUTION_ = config["OBJECT"]["YOLO_RESOLUTION"].as<uint16_t>();
    RESIZING_X_ = IMAGE_WIDTH_ / static_cast<float>(YOLO_RESOLUTION_);
    RESIZING_Y_ = IMAGE_HEIGHT_ / static_cast<float>(YOLO_RESOLUTION_);
    OBJ_DEPTH_THRESH_ = config["OBJECT"]["XY_DEPTH"].as<float>();

    IS_DEBUGGING_ = config["DEBUG"].as<bool>();
}

Driving::~Driving()
{
    delete PID_;
    delete LaneDetector_;
}

void Driving::run()
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
        if (SubscriberYOLO_.getNumPublishers() != 1)
        {
            continue;
        }

        // Get appropriate steering angle to drive at current frame
        std::pair<float, float> lanes_position;
        std::pair<bool, bool> is_each_lane_detected;
        std::tie(lanes_position, is_each_lane_detected) = LaneDetector_ -> getLaneInfo(frame_);

        float center_lane_position = (lanes_position.first + lanes_position.second) * 0.5 + 30;
        bool is_left_detected, is_right_detected;
        std::tie(is_left_detected, is_right_detected) = is_each_lane_detected;

        float gap = (center_lane_position - IMAGE_WIDTH_ * 0.5);
        float steering_angle = PID_ -> getPIDOutput(gap);
        steering_angle = (steering_angle > 50.0) ? 50.0 : (steering_angle < -50.0) ? -50.0 : steering_angle;
        tmp_deceleration_step_ = std::round(std::abs(gap) * 0.1) * DECELERATION_STEP_;

        cv::Mat img_undistorted;
        undistortImg(frame_, img_undistorted);

        if (IS_DEBUGGING_)
        {
            cv::Mat result = img_undistorted.clone();

            drawBboxes(img_undistorted, result, predictions_);

            std::vector<cv::Point> undistorted_lanes_position = {
                cv::Point(60, 400),
                cv::Point(580, 400)
            };
            undistortLanesPosition(lanes_position, LaneDetector_ -> moving_y_offset_, undistorted_lanes_position);
            drawLanes(result, undistorted_lanes_position);

            cv::imshow("Result", result);
            cv::waitKey(1);
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
        //     continue;  // Skip to next frame.
        // }


        yolov3_trt_ros::BoundingBox closest_object;
        getClosestObject(predictions_, closest_object);
        float depth = sqrt(closest_object.xdepth*closest_object.xdepth + closest_object.ydepth*closest_object.ydepth);

        // Check whether there is stopline.
        if (isStopLine(img_undistorted))
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
                continue;
            }
        }

        // Drive only using lanes if there are no other situations.
        drive(steering_angle);
    }
}

void Driving::imageCallback(const sensor_msgs::Image::ConstPtr& message)
{
    cv::Mat src = cv::Mat(message -> height, message -> width, CV_8UC3,
                          const_cast<uint8_t*>(&message -> data[0]), message -> step);
    cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

void Driving::scanCallback(const sensor_msgs::LaserScan::ConstPtr& message)
{
    lidar_data_ = message -> ranges;  // size = 505

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
    // std::cout << predictions_.bbox[0] << std::endl;
}

void Driving::drawBboxes(const cv::Mat& input_img, cv::Mat& output_img,
                               const yolov3_trt_ros::BoundingBoxes& predictions)
{
    for (const auto& pred : predictions.bbox)
    {
        // bounding box
        cv::Point top_left = cv::Point(pred.xmin * RESIZING_X_, pred.ymin * RESIZING_Y_);
        cv::Point bottom_right = cv::Point(pred.xmax * RESIZING_X_, pred.ymax * RESIZING_Y_);
        cv::rectangle(output_img, top_left, bottom_right, COLORS_[pred.id], 2);

        // class, score, depth
        std::string id = LABELS_[pred.id];
        std::string score = std::to_string(pred.prob + 0.005).substr(0, 4);
        std::string depth = std::to_string(
            static_cast<uint16_t>(sqrt(pred.xdepth*pred.xdepth + pred.ydepth*pred.ydepth) + 0.5)
        );
        std::string text = id + " " + score + " " + depth + "cm";
        cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, nullptr);

        cv::rectangle(output_img,
                      top_left, cv::Point(top_left.x + text_size.width, top_left.y - text_size.height - 3),
                      COLORS_[pred.id], -1);
        cv::putText(output_img, text,
                    cv::Point(top_left.x, top_left.y - 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    }
}

void Driving::drawLanes(cv::Mat& input_img, const std::vector<cv::Point>& lanes_position)
{
    int y = std::min(lanes_position[0].y, lanes_position[1].y);
    cv::Point center_position((lanes_position[0].x + lanes_position[1].x) * 0.5 - 15, y);
    cv::Point img_center(static_cast<uint16_t>(input_img.cols * 0.5), y);

    cv::circle(input_img, lanes_position[0], 6, cv::Scalar(0, 255, 0), -1);
    cv::circle(input_img, lanes_position[1], 6, cv::Scalar(0, 255, 0), -1);
    cv::circle(input_img, img_center, 6, cv::Scalar(50, 50, 255), -1);
    cv::putText(input_img, "Car",
                cv::Point(img_center.x - 20, img_center.y + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(50, 50, 255), 2);
    cv::circle(input_img, center_position, 6, cv::Scalar(255, 200, 0), -1);
    cv::putText(input_img, "Lane",
                cv::Point(center_position.x - 20, center_position.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 200, 0), 2);
}

void Driving::getClosestObject(const yolov3_trt_ros::BoundingBoxes& predictions,
                                     yolov3_trt_ros::BoundingBox& closest_object)
{
    // Initialize
    closest_object.prob = -1;
    closest_object.xmin = -1;
    closest_object.ymin = -1;
    closest_object.xmax = -1;
    closest_object.ymax = -1;
    closest_object.id = -1;
    closest_object.xdepth = -1;
    closest_object.ydepth = -1;

    // Assign
    for (const auto& pred : predictions.bbox)
    {
        if (pred.id == 4)
        {
            continue;  // Skip if class is "car" which is unnecessary class.
        }

        if ((closest_object.id == -1) || (closest_object.ydepth > pred.ydepth))
        {
            closest_object.prob = pred.prob;
            closest_object.xmin = pred.xmin * RESIZING_X_;
            closest_object.ymin = pred.ymin * RESIZING_Y_;
            closest_object.xmax = pred.xmax * RESIZING_X_;
            closest_object.ymax = pred.ymax * RESIZING_Y_;
            closest_object.id = pred.id;
            closest_object.xdepth = pred.xdepth;
            closest_object.ydepth = pred.ydepth;
        }
    }
}

void Driving::controlSpeed(float steering_angle)
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

void Driving::drive(float steering_angle)
{
    xycar_msgs::xycar_motor motor_message;
    motor_message.angle = std::round(steering_angle);
    controlSpeed(steering_angle);
    LaneDetector_ -> setYOffset(XYCAR_SPEED_);
    motor_message.speed = std::round(XYCAR_SPEED_);
    PublisherMotor_.publish(motor_message);
}

void Driving::undistortImg(const cv::Mat& input_img, cv::Mat& output_img)
{
    cv::Matx<float, 3, 3> camera_matrix(352.494189, 0.000000,   295.823760,
                             0.000000,   353.504572, 239.649689,
                             0.000000,   0.000000,   1.000000);
    cv::Matx<float, 1, 5> dist_coeffs(-0.318744, 0.088199, 0.000167, 0.000699, 0.000000);
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), cv::Mat(),
                                cv::Size(input_img.cols, input_img.rows), CV_8UC1, map1, map2);
    cv::remap(input_img, output_img, map1, map2, cv::INTER_LINEAR);
}

void Driving::undistortLanesPosition(const std::pair<float, float>& lanes_position,
                                           const int32_t y,
                                           std::vector<cv::Point>& undistorted_lanes_position)
{
    std::vector<cv::Point2f> distorted_pts = {cv::Point2f(lanes_position.first, y),
                                              cv::Point2f(lanes_position.second, y)};
    std::vector<cv::Point2f> normalized_undistorted_pts;
    cv::Matx<float, 3, 3> camera_matrix(352.494189, 0.000000,   295.823760,
                                        0.000000,   353.504572, 239.649689,
                                        0.000000,   0.000000,   1.000000);
    cv::Matx<float, 1, 5> dist_coeffs(-0.318744, 0.088199, 0.000167, 0.000699, 0.000000);
    cv::undistortPoints(distorted_pts, normalized_undistorted_pts,
                        camera_matrix, dist_coeffs, cv::noArray(), cv::noArray());

    for (int i = 0; i < 2; ++i)
    {
        undistorted_lanes_position[i].x = static_cast<uint16_t>(
            camera_matrix(0, 0) * normalized_undistorted_pts[i].x + camera_matrix(0, 2) + 30
        );
        undistorted_lanes_position[i].y = static_cast<uint16_t>(
            camera_matrix(1, 1) * normalized_undistorted_pts[i].y + camera_matrix(1, 2)
        );
        // undistorted_lanes_position.emplace_back(
        //     static_cast<uint16_t>(camera_matrix(0, 0) * normalized_undistorted_pts[i].x + camera_matrix(0, 2) + 30),
        //     static_cast<uint16_t>(camera_matrix(1, 1) * normalized_undistorted_pts[i].y + camera_matrix(1, 2))
        // );
    }
}

bool Driving::isStopLine(const cv::Mat& input_img)
{
    // cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 352.494189, 0.000000,   295.823760,
    //                                                    0.000000,   353.504572, 239.649689,
    //                                                    0.000000,   0.000000,   1.000000);
    // cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -0.318744, 0.088199, 0.000167, 0.000699, 0.000000);
    // cv::Mat map1, map2, undistorted;
    // cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), cv::Mat(),
    //                             cv::Size(input_img.cols, input_img.rows), CV_8UC1, map1, map2);
    // cv::remap(input_img, undistorted, map1, map2, cv::INTER_LINEAR);

    cv::Mat img_gray, img_blur, img_bin, img_edge;
    cv::Rect roi(140, 370, 360, 40);
    cv::Mat img_cropped = input_img(roi);
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

    if (IS_DEBUGGING_)
    {
        // cv::imshow("hough", img_cropped);
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
        return true;
    }
    else
    {
        return false;
    }
}
}  // namespace xycar
