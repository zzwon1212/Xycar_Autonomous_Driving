#include "autonomous_driving/lane_detector.h"

namespace xycar
{
template <typename PREC>
void LaneDetector<PREC>::getConfig(const YAML::Node& config)
{
    img_width_ = config["IMAGE"]["WIDTH"].as<int32_t>();
    img_height_ = config["IMAGE"]["HEIGHT"].as<int32_t>();
    low_threshold_ = config["CANNY"]["LOW_THRESHOLD"].as<int32_t>();
    high_threshold_ = config["CANNY"]["HIGH_THRESHOLD"].as<int32_t>();
    min_pixel_ = config["HOUGH"]["MIN_PIXEL"].as<int32_t>();
    min_line_ = config["HOUGH"]["MIN_LINE"].as<int32_t>();
    max_gap_ = config["HOUGH"]["MAX_GAP"].as<int32_t>();
    y_offset_ = config["IMAGE"]["Y_OFFSET"].as<int32_t>();
    y_gap_ = config["IMAGE"]["Y_GAP"].as<int32_t>();
    y_gain_ = config["IMAGE"]["Y_GAIN"].as<double>();
    is_debugging_ = config["DEBUG"].as<bool>();
}

template <typename PREC>
std::tuple<cv::Point, cv::Point, bool, bool> LaneDetector<PREC>::getLinePosition(std::vector<int>& left_x_at_Y_offset,
                                                                                 std::vector<int>& right_x_at_Y_offset)
{
    cv::Point left, right;
    bool is_left_detected, is_right_detected;

    if ((!left_x_at_Y_offset.empty()) && (left_x_at_Y_offset.size() < 6))
    {
        int left_lx = *min_element(left_x_at_Y_offset.begin(), left_x_at_Y_offset.end());
        int left_rx = *max_element(left_x_at_Y_offset.begin(), left_x_at_Y_offset.end());

        if ((left_rx-left_lx > 50) && (left_x_at_Y_offset.size() != 1)) left_lx = left_rx - 50;

        left = cv::Point((left_lx + left_rx)/2, moving_y_offset_);
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
        int right_lx = *min_element(right_x_at_Y_offset.begin(), right_x_at_Y_offset.end());
        int right_rx = *max_element(right_x_at_Y_offset.begin(), right_x_at_Y_offset.end());

        if ((right_rx-right_lx > 50) && (right_x_at_Y_offset.size() != 1)) right_rx = right_lx + 50;

        right = cv::Point((right_lx + right_rx)/2, moving_y_offset_);
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
    if (abs(right.x - left.x) < 300)  // 300 100
    {
        if (rightC_ != 0)
        {
            right.x = 640;
            // right.x = left.x +300;
            prev_right_ = right;
        }

        if (leftC_ != 0)
        {
            left.x = 0;
            // left.x = right.x -300;
            prev_left_ = left;
        }
    }

    if (left.x > 240)
    {
        right.x += 50;
        prev_right_.x += 1;
        rightC_ = 0;
    }

    if (right.x < 400)
    {
        left.x -= 50;
        prev_left_.x -=1;
        leftC_ = 0;
    }

    return std::make_tuple(left, right, is_left_detected, is_right_detected);
}

template <typename PREC>
std::pair<std::vector<int>, std::vector<int>> LaneDetector<PREC>::divideLeftRight(std::vector<cv::Vec4f>& lines)
{
    std::vector<int> left_x_at_Y_offset, right_x_at_Y_offset;
    double slope;

    for (cv::Vec4f line_ : lines)
    {
        cv::Point pt1(line_[0], line_[1]);
        cv::Point pt2(line_[2], line_[3]);
        slope = (double)(pt2.y - pt1.y)/(pt2.x - pt1.x + 0.0001);

        if (abs(slope) > 0 && abs(slope) < 10)
        {
            int x_at_Y_offset;
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

template <typename PREC>
std::pair<std::pair<int, int>, std::pair<bool, bool>> LaneDetector<PREC>::getLaneInfo(cv::Mat& frame)
{
    // Set ROI
    // @@@@@@@@@@@@@@@@@@@@@@@@2 TODO: Find more efficient code
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<cv::Point> square;
    square.push_back(cv::Point(0, moving_y_offset_ + y_gap_));
    square.push_back(cv::Point(0, moving_y_offset_ - y_gap_));
    square.push_back(cv::Point(frame.cols, moving_y_offset_ - y_gap_));
    square.push_back(cv::Point(frame.cols, moving_y_offset_ + y_gap_));
    cv::fillConvexPoly(mask, &square[0], 4, cv::Scalar(255));

    // Get HoughLines
    cv::Mat img_gray, img_blur, img_edge, output, img_binary;
    std::vector<cv::Vec4f> lines;
    cv::cvtColor(frame, img_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(img_gray, img_blur, cv::Size(), 2.0);
    cv::Canny(img_blur, img_edge, low_threshold_, high_threshold_);
    cv::bitwise_and(img_edge, mask, output, mask=mask);  // @@@@@@@@@@ OUTPUT NAME ???
    cv::threshold(output, img_binary, 190, 255, cv::THRESH_BINARY);
    cv::HoughLinesP(img_binary, lines, 1, CV_PI/180, min_pixel_, min_line_, max_gap_);

    std::vector<int> left_lines, right_lines;
    std::tie(left_lines, right_lines) = divideLeftRight(lines);

    cv::Point left, right;
    bool is_left_detected, is_right_detected;
    std::tie(left, right, is_left_detected, is_right_detected) = getLinePosition(left_lines, right_lines);

    std::pair<int, int> lanes_position = std::make_pair(left.x, right.x);
    std::pair<bool, bool> is_each_lane_detected = std::make_pair(is_left_detected, is_right_detected);

    return std::make_pair(lanes_position, is_each_lane_detected);
}

template class LaneDetector<float>;
template class LaneDetector<double>;
}  // namespace xycar
