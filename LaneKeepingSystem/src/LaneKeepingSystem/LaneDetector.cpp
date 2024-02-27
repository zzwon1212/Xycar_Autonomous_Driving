// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneDetector.cpp
 * @author Jeongmin Kim
 * @author Jeongbin Yim
 * @brief lane detector class source file
 * @version 2.1
 * @date 2023-10-13
 */

#include <numeric>
#include "LaneKeepingSystem/LaneDetector.hpp"

namespace Xycar {

template <typename PREC>
void LaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    low_threshold = config["CANNY"]["LOW_THRESHOLD"].as<int32_t>();
    high_threshold = config["CANNY"]["HIGH_THRESHOLD"].as<int32_t>();
    min_pixel = config["HOUGH"]["MIN_PIXEL"].as<int32_t>();
    min_line = config["HOUGH"]["MIN_LINE"].as<int32_t>();
    max_gap = config["HOUGH"]["MAX_GAP"].as<int32_t>();
    yOffset = config["IMAGE"]["Y_OFFSET"].as<int32_t>();
    yGap = config["IMAGE"]["Y_GAP"].as<int32_t>();
    yGain = config["IMAGE"]["Y_GAIN"].as<double>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
// std::pair<cv::Point, cv::Point> LaneDetector<PREC>::getLinePos(std::vector<int>& left_x_at_Y_offset, std::vector<int>& right_x_at_Y_offset)
std::tuple<cv::Point, cv::Point, bool, bool> LaneDetector<PREC>::getLinePos(std::vector<int>& left_x_at_Y_offset, std::vector<int>& right_x_at_Y_offset)
{
    cv::Point lpos, rpos;
    bool is_lpos_detected, is_rpos_detected;

    if ((!left_x_at_Y_offset.empty()) && (left_x_at_Y_offset.size() < 6)) {
        int lposl_x = *min_element(left_x_at_Y_offset.begin(), left_x_at_Y_offset.end());
        int lposr_x = *max_element(left_x_at_Y_offset.begin(), left_x_at_Y_offset.end());
        if ((lposr_x-lposl_x > 50) && (left_x_at_Y_offset.size() != 1)) lposl_x = lposr_x - 50;
        lpos = cv::Point((lposl_x + lposr_x) / 2, tempYOffset);
        prev_lpos = lpos;
        leftC = 0;
        is_lpos_detected = true;
    } else {
        lpos = prev_lpos;
        leftC += 1;
        is_lpos_detected = false;
    }
    if ((!right_x_at_Y_offset.empty()) && (right_x_at_Y_offset.size() < 6)) {
        int rposl_x = *min_element(right_x_at_Y_offset.begin(), right_x_at_Y_offset.end());
        int rposr_x = *max_element(right_x_at_Y_offset.begin(), right_x_at_Y_offset.end());
        if ((rposr_x-rposl_x > 50) && (right_x_at_Y_offset.size() != 1)) rposr_x = rposl_x + 50;
        rpos = cv::Point((rposl_x + rposr_x) / 2, tempYOffset);
        prev_rpos = rpos;
        rightC = 0;
        is_rpos_detected = true;
    } else {
        rpos = prev_rpos;
        rightC += 1;
        is_rpos_detected = false;
    }

    //when points are too close
    if (abs(rpos.x - lpos.x) < 300){//300 100
        if (rightC != 0) {
            rpos.x = 640;
            // rpos.x = lpos.x +300;
            prev_rpos = rpos;
        }
        if (leftC != 0) {
            lpos.x = 0;
            // lpos.x = rpos.x -300;
            prev_lpos = lpos;
        }
    }

    if (lpos.x > 240) {
        rpos.x += 50;
        prev_rpos.x += 1;
        rightC = 0;
    }
    if (rpos.x < 400) {
        lpos.x -= 50;
        prev_lpos.x -=1;
        leftC = 0;
    }

    // std::cout << is_lpos_detected << ", " << is_rpos_detected << std::endl;

    return std::make_tuple(lpos, rpos, is_lpos_detected, is_rpos_detected);
}

template <typename PREC>
std::pair<std::vector<int>, std::vector<int>> LaneDetector<PREC>::divideLeftRight(std::vector<cv::Vec4f>& lines)
{
    std::vector<int> left_x_at_Y_offset;
    std::vector<int> right_x_at_Y_offset;
    double slope;

    for (cv::Vec4f line_ : lines) {
        cv::Point pt1(line_[0], line_[1]);
        cv::Point pt2(line_[2], line_[3]);
        slope = (double)(pt2.y - pt1.y) / (pt2.x - pt1.x + 0.0001);

        if (abs(slope)>0 && abs(slope) < 10) {
            int x_at_Y_offset;
            if (pt1.x != pt2.x) {
                x_at_Y_offset = (tempYOffset - pt1.y) / slope + pt1.x;
            } else {
                x_at_Y_offset = pt1.x;
            }

            if (slope < 0 && x_at_Y_offset <280) {//250
                left_x_at_Y_offset.push_back(x_at_Y_offset);
            } else if (slope > 0 && x_at_Y_offset >360) {//380
                right_x_at_Y_offset.push_back(x_at_Y_offset);
            }
        }
    }
    return std::make_pair(left_x_at_Y_offset, right_x_at_Y_offset);
}

template <typename PREC>
// double LaneDetector<PREC>::processImage(cv::Mat& frame)
std::tuple<double, bool, bool> LaneDetector<PREC>::processImage(cv::Mat& frame)
{
    cv::Mat img_gray, img_histo, img_blur, img_edge, roi, thresframe;
    std::vector<cv::Vec4f> lines;
    cv::Mat output;
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);

    std::vector<cv::Point> square;
    square.push_back(cv::Point(0, tempYOffset + yGap));
    square.push_back(cv::Point(0, tempYOffset - yGap));
    square.push_back(cv::Point(frame.cols, tempYOffset - yGap));
    square.push_back(cv::Point(frame.cols,tempYOffset + yGap));

    cv::fillConvexPoly(mask, &square[0], 4, cv::Scalar(255));

    cv::cvtColor(frame, img_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(img_gray, img_blur, cv::Size(), 2.0);
    cv::Canny(img_blur, img_edge, low_threshold, high_threshold);
    cv::bitwise_and(img_edge, mask, output, mask=mask);
    cv::threshold(output,thresframe, 190, 255, cv::THRESH_BINARY);
    cv::HoughLinesP(thresframe, lines, 1, CV_PI / 180, min_pixel, min_line, max_gap);

    std::vector<int> leftLines, rightLines;
    std::tie(leftLines, rightLines) = divideLeftRight(lines);
    cv::Point lpos, rpos;
    bool is_lpos_detected, is_rpos_detected;
    std::tie(lpos, rpos, is_lpos_detected, is_rpos_detected) = getLinePos(leftLines, rightLines);

    if (mDebugging) {
        // draw parts
        mask.copyTo(mDebugROI);
        frame.copyTo(mDebugFrame);
        drawLines(lines);
        drawRectangle(lpos.x, rpos.x);
    }

    return std::make_tuple(static_cast<double>(lpos.x + rpos.x) / 2 + 30, is_lpos_detected, is_rpos_detected);
}

template <typename PREC>
void LaneDetector<PREC>::drawLines(std::vector<cv::Vec4f>& lines)
{
    for (auto& line : lines) {
        int x1, y1, x2, y2;
        std::tie(x1, y1, x2, y2) = std::make_tuple(line[0], line[1], line[2], line[3]);
        cv::line(mDebugFrame, cv::Point(x1, y1), cv::Point(x2, y2), kRed, 2);
    }
}

template <typename PREC>
void LaneDetector<PREC>::drawRectangle(int leftPositionX, int rightPositionX)
{
    int center = (leftPositionX + rightPositionX) / 2;
    cv::rectangle(mDebugFrame, cv::Point(leftPositionX - 5, tempYOffset-5), cv::Point(leftPositionX + 5, tempYOffset+5), kGreen, 2, cv::LINE_AA);
    putText(mDebugFrame, cv::format("(%d, %d,leftPositionX)", leftPositionX, tempYOffset), cv::Point(leftPositionX, tempYOffset-20), cv::FONT_HERSHEY_SIMPLEX, 0.5, kGreen, 1, cv::LINE_AA);
    cv::rectangle(mDebugFrame, cv::Point(rightPositionX - 5, tempYOffset-5), cv::Point(rightPositionX + 5,tempYOffset+5), kGreen, 2, cv::LINE_AA);
    putText(mDebugFrame, cv::format("(%d, %d,rightPositionX)", rightPositionX, tempYOffset), cv::Point(rightPositionX, tempYOffset-20), cv::FONT_HERSHEY_SIMPLEX, 0.5, kGreen, 1, cv::LINE_AA);
    cv::rectangle(mDebugFrame, cv::Point(center - 5, tempYOffset-5), cv::Point(center + 5, tempYOffset+5), kRed, 2, cv::LINE_AA);
    putText(mDebugFrame, cv::format("(%d, %d,lane_center)",center, tempYOffset), cv::Point(center, tempYOffset-20), cv::FONT_HERSHEY_SIMPLEX, 0.5, kRed, 1, cv::LINE_AA);
    cv::rectangle(mDebugFrame, cv::Point(mImageWidth/2 - 5, tempYOffset-5), cv::Point(mImageWidth/2 + 5, tempYOffset+5), kBlue, 2, cv::LINE_AA);
}
template class LaneDetector<float>;
template class LaneDetector<double>;
} // namespace Xycar
