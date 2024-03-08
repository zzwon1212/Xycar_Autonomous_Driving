#include "autonomous_driving/lane_detector.h"

namespace xycar
{
LaneDetector::LaneDetector(const YAML::Node& config)
{
    getConfig(config);
    prev_left_ = cv::Point(0, moving_y_offset_);
    prev_right_ = cv::Point(IMG_WIDTH_, Y_OFFSET_);
    leftC_ = 0;
    rightC_ = 0;
}

void LaneDetector::getConfig(const YAML::Node& config)
{
    // Image
    IMG_WIDTH_ = config["IMAGE"]["WIDTH"].as<uint16_t>();
    Y_OFFSET_ = config["IMAGE"]["Y_OFFSET"].as<uint16_t>();
    Y_GAP_ = config["IMAGE"]["Y_GAP"].as<uint16_t>();
    Y_GAIN_ = config["IMAGE"]["Y_GAIN"].as<float>();

    // Canny
    LOW_THRESH_ = config["CANNY"]["LOW_THRESHOLD"].as<uint16_t>();
    HIGH_THRESH_ = config["CANNY"]["HIGH_THRESHOLD"].as<uint16_t>();

    // Hough
    MIN_PIXEL_ = config["HOUGH"]["MIN_PIXEL"].as<uint16_t>();
    MIN_LINE_ = config["HOUGH"]["MIN_LINE"].as<uint16_t>();
    MAX_GAP_ = config["HOUGH"]["MAX_GAP"].as<uint16_t>();
}

std::pair<std::vector<float>, std::vector<float>> LaneDetector::divideLeftRight(std::vector<cv::Vec4f>& lines)
{
    std::vector<float> left_x_at_Y_offset, right_x_at_Y_offset;
    float slope;

    for (cv::Vec4f line_ : lines)
    {
        cv::Point pt1(line_[0], line_[1]);
        cv::Point pt2(line_[2], line_[3]);
        slope = static_cast<float>((pt2.y - pt1.y) / (pt2.x - pt1.x + 0.0001));

        if (abs(slope) > 0 && abs(slope) < 10)
        {
            float x_at_Y_offset;
            if (pt1.x != pt2.x)
            {
                x_at_Y_offset = (moving_y_offset_ - pt1.y) / slope + pt1.x;
            }
            else
            {
                x_at_Y_offset = pt1.x;
            }

            if (slope < 0 && x_at_Y_offset < 280)  //250
            {
                left_x_at_Y_offset.push_back(x_at_Y_offset);
            }
            else if
            (slope > 0 && x_at_Y_offset > 360)  //380
            {
                right_x_at_Y_offset.push_back(x_at_Y_offset);
            }
        }
    }
    return std::make_pair(left_x_at_Y_offset, right_x_at_Y_offset);
}

std::tuple<cv::Point2f, cv::Point2f, bool, bool> LaneDetector::getLinePosition(std::vector<float>& left_x_at_Y_offset,
                                                                           std::vector<float>& right_x_at_Y_offset)
{
    cv::Point2f left, right;
    bool is_left_detected, is_right_detected;

    if ((!left_x_at_Y_offset.empty()) && (left_x_at_Y_offset.size() < 6))
    {
        float left_lx = *min_element(left_x_at_Y_offset.begin(), left_x_at_Y_offset.end());
        float left_rx = *max_element(left_x_at_Y_offset.begin(), left_x_at_Y_offset.end());

        if ((left_rx - left_lx > 50.0) && (left_x_at_Y_offset.size() != 1)) left_lx = left_rx - 50.0;

        left = cv::Point2f((left_lx + left_rx) * 0.5, moving_y_offset_);
        prev_left_ = left;
        leftC_ = 0;
        is_left_detected = true;
    }
    else
    {
        left = prev_left_;
        leftC_ += 1;
        is_left_detected = false;
    }

    if ((!right_x_at_Y_offset.empty()) && (right_x_at_Y_offset.size() < 6))
    {
        float right_lx = *min_element(right_x_at_Y_offset.begin(), right_x_at_Y_offset.end());
        float right_rx = *max_element(right_x_at_Y_offset.begin(), right_x_at_Y_offset.end());

        if ((right_rx - right_lx > 50.0) && (right_x_at_Y_offset.size() != 1)) right_rx = right_lx + 50.0;

        right = cv::Point2f((right_lx + right_rx) * 0.5, moving_y_offset_);
        prev_right_ = right;
        rightC_ = 0;
        is_right_detected = true;
    }
    else
    {
        right = prev_right_;
        rightC_ += 1;
        is_right_detected = false;
    }

    // If points are too close
    if (abs(right.x - left.x) < 300.0)  // 300 100
    {
        if (rightC_ != 0)
        {
            right.x = 640.0;
            // right.x = left.x +300;
            prev_right_ = right;
        }

        if (leftC_ != 0)
        {
            left.x = 0.0;
            // left.x = right.x -300;
            prev_left_ = left;
        }
    }

    if (left.x > 240.0)
    {
        right.x += 50.0;
        prev_right_.x += 1.0;
        rightC_ = 0;
    }

    if (right.x < 400.0)
    {
        left.x -= 50.0;
        prev_left_.x -= 1.0;
        leftC_ = 0;
    }

    return std::make_tuple(left, right, is_left_detected, is_right_detected);
}

std::pair<std::pair<float, float>, std::pair<bool, bool>> LaneDetector::getLaneInfo(cv::Mat& frame)
{
    // Set ROI
    // @@@@@@@@@@@@@@@@@@@@@@@@2 TODO: Find more efficient code
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<cv::Point> square;
    square.push_back(cv::Point(0, moving_y_offset_ + Y_GAP_));
    square.push_back(cv::Point(0, moving_y_offset_ - Y_GAP_));
    square.push_back(cv::Point(frame.cols, moving_y_offset_ - Y_GAP_));
    square.push_back(cv::Point(frame.cols, moving_y_offset_ + Y_GAP_));
    cv::fillConvexPoly(mask, &square[0], 4, cv::Scalar(255));

    // Get HoughLines
    cv::Mat img_gray, img_blur, img_edge, output, img_binary;
    std::vector<cv::Vec4f> lines;
    cv::cvtColor(frame, img_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(img_gray, img_blur, cv::Size(), 2.0);
    cv::Canny(img_blur, img_edge, LOW_THRESH_, HIGH_THRESH_);
    cv::bitwise_and(img_edge, mask, output, mask=mask);  // @@@@@@@@@@ OUTPUT NAME ???
    cv::threshold(output, img_binary, 190, 255, cv::THRESH_BINARY);
    cv::HoughLinesP(img_binary, lines, 1, CV_PI/180, MIN_PIXEL_, MIN_LINE_, MAX_GAP_);

    std::vector<float> left_lines, right_lines;
    std::tie(left_lines, right_lines) = divideLeftRight(lines);

    cv::Point2f left, right;
    bool is_left_detected, is_right_detected;
    std::tie(left, right, is_left_detected, is_right_detected) = getLinePosition(left_lines, right_lines);

    std::pair<float, float> lanes_position = std::make_pair(left.x, right.x);
    std::pair<bool, bool> is_each_lane_detected = std::make_pair(is_left_detected, is_right_detected);

    return std::make_pair(lanes_position, is_each_lane_detected);
}

void LaneDetector::setYOffset(float speed)
{
    moving_y_offset_ = Y_OFFSET_ - speed * Y_GAIN_;
}

}  // namespace xycar
