#include "autonomous_driving/tools.h"

namespace xycar
{
cv::Matx<float, 3, 3> CAMERA_MATRIX_(352.494189, 0.000000,   295.823760,
                                     0.000000,   353.504572, 239.649689,
                                     0.000000,   0.000000,   1.000000);
cv::Matx<float, 1, 5> DIST_COEFFS_(-0.318744, 0.088199, 0.000167, 0.000699, 0.000000);

Tools::Tools(const YAML::Node& config)
{
    getConfig(config);
}

void Tools::getConfig(const YAML::Node& config)
{
    LABELS_ = config["OBJECT"]["LABELS"].as<std::vector<std::string>>();
    COLORS_ = {cv::Scalar(50, 200, 255),  // LEFT
               cv::Scalar(255, 0, 255),  // RIGHT
               cv::Scalar(255, 0, 0),   // CROSSWALK
               cv::Scalar(0, 0, 170),   // STOP SIGN
               cv::Scalar(0, 170, 0),   // CAR
               cv::Scalar(0, 0, 255),   // RED SIGN
               cv::Scalar(0, 100, 50),  // GREEN SIGN
               cv::Scalar(255, 255, 0)};  // YELLOW SIGN
    IMAGE_WIDTH_ = config["IMAGE"]["WIDTH"].as<uint16_t>();
    IMAGE_HEIGHT_ = config["IMAGE"]["HEIGHT"].as<uint16_t>();
    YOLO_RESOLUTION_ = config["OBJECT"]["YOLO_RESOLUTION"].as<uint16_t>();
    RESIZING_X_ = IMAGE_WIDTH_ / static_cast<float>(YOLO_RESOLUTION_);
    RESIZING_Y_ = IMAGE_HEIGHT_ / static_cast<float>(YOLO_RESOLUTION_);
}

void Tools::undistortImg(const cv::Mat& input_img, cv::Mat& output_img)
{
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(CAMERA_MATRIX_, DIST_COEFFS_, cv::Mat(), cv::Mat(),
                                cv::Size(input_img.cols, input_img.rows), CV_8UC1, map1, map2);
    cv::remap(input_img, output_img, map1, map2, cv::INTER_LINEAR);
}

void Tools::undistortLanesPosition(const std::pair<float, float>& lanes_position,
                                   const uint16_t y,
                                   std::vector<cv::Point>& undistorted_lanes_position)
{
    std::vector<cv::Point2f> distorted_pts = {cv::Point2f(lanes_position.first, y),
                                              cv::Point2f(lanes_position.second, y)};
    std::vector<cv::Point2f> normalized_undistorted_pts;

    // Undistort ditorted points. Output points are on normal plane.
    cv::undistortPoints(distorted_pts, normalized_undistorted_pts,
                        CAMERA_MATRIX_, DIST_COEFFS_, cv::noArray(), cv::noArray());

    // Project normal plane to image plane
    for (int i = 0; i < 2; ++i)
    {
        undistorted_lanes_position[i].x = static_cast<uint16_t>(
            CAMERA_MATRIX_(0, 0) * normalized_undistorted_pts[i].x + CAMERA_MATRIX_(0, 2) + 30);
        undistorted_lanes_position[i].y = static_cast<uint16_t>(
            CAMERA_MATRIX_(1, 1) * normalized_undistorted_pts[i].y + CAMERA_MATRIX_(1, 2));
        // undistorted_lanes_position.emplace_back(
        //     static_cast<uint16_t>(camera_matrix(0, 0) * normalized_undistorted_pts[i].x + camera_matrix(0, 2) + 30),
        //     static_cast<uint16_t>(camera_matrix(1, 1) * normalized_undistorted_pts[i].y + camera_matrix(1, 2))
        // );
    }
}

void Tools::drawBboxes(cv::Mat& img, const yolov3_trt_ros::BoundingBoxes& predictions)
{
    for (const auto& pred : predictions.bbox)
    {
        // bounding box
        cv::Point top_left = cv::Point(pred.xmin * RESIZING_X_, pred.ymin * RESIZING_Y_);
        cv::Point bottom_right = cv::Point(pred.xmax * RESIZING_X_, pred.ymax * RESIZING_Y_);
        cv::rectangle(img, top_left, bottom_right, COLORS_[pred.id], 2);

        // class, score, depth
        std::string id = LABELS_[pred.id];
        std::string score = std::to_string(pred.prob + 0.005).substr(0, 4);
        std::string depth = std::to_string(
            static_cast<uint16_t>(sqrt(pred.xdepth*pred.xdepth + pred.ydepth*pred.ydepth) + 0.5)
        );
        std::string text = id + " " + score + " " + depth + "cm";
        cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, nullptr);

        cv::rectangle(img,
                      top_left, cv::Point(top_left.x + text_size.width, top_left.y - text_size.height - 3),
                      COLORS_[pred.id], -1);
        cv::putText(img, text,
                    cv::Point(top_left.x, top_left.y - 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    }
}

void Tools::drawLanes(cv::Mat& img, const std::vector<cv::Point>& lanes_position)
{
    int y = std::min(lanes_position[0].y, lanes_position[1].y);
    cv::Point center_position((lanes_position[0].x + lanes_position[1].x) * 0.5 - 15, y);
    cv::Point img_center(static_cast<uint16_t>(img.cols * 0.5), y);

    cv::circle(img, lanes_position[0], 6, cv::Scalar(0, 255, 0), -1);
    cv::circle(img, lanes_position[1], 6, cv::Scalar(0, 255, 0), -1);
    cv::circle(img, img_center, 6, cv::Scalar(50, 50, 255), -1);
    cv::putText(img, "Car",
                cv::Point(img_center.x - 20, img_center.y + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(50, 50, 255), 2);
    cv::circle(img, center_position, 6, cv::Scalar(255, 200, 0), -1);
    cv::putText(img, "Lane",
                cv::Point(center_position.x - 20, center_position.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 200, 0), 2);
}

void Tools::isStopline(const cv::Mat img, bool& is_stopline, std::vector<cv::Vec4f>& stoplines)
{
    cv::Mat img_gray, img_blur, img_bin, img_edge;
    cv::Rect roi(140, 370, 360, 40);
    cv::Mat img_cropped = img(roi);
    // std::vector<cv::Vec4f> lines;
    cv::cvtColor(img_cropped, img_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(img_gray, img_blur, cv::Size(), 1.0);
    cv::threshold(img_blur, img_bin, 190, 255, cv::THRESH_BINARY_INV);
    cv::Canny(img_bin, img_edge, 50, 150);
    cv::HoughLinesP(img_edge, stoplines, 1, CV_PI/180, 40, 160, 50);

    // if (IS_DEBUGGING_)
    // {
        // cv::imshow("hough", img_cropped);
    // }

    float slope;
    if (stoplines.empty())
    {
        slope = -1;
    }
    else
    {
        slope = (stoplines[0][3] - stoplines[0][1]) / (stoplines[0][0] - stoplines[0][2] + 1e-6);
    }

    is_stopline = (abs(slope) < 0.025) ? true : false;
}

void Tools::drawStoplines(cv::Mat& img, const std::vector<cv::Vec4f> stoplines)
{
    for (size_t i = 0; i < stoplines.size(); i++)
    {
        cv::Vec4i l = stoplines[i];

        float slope = (l[3] - l[1]) / (l[0] - l[2] + 1e-6);

        if (abs(slope) < 0.025)
        {
            cv::line(img,
                     cv::Point(l[0] + 140, l[1] + 370), cv::Point(l[2] + 140, l[3] + 370),
                     cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
    }
}

void Tools::show(
    cv::Mat& img,
    const std::vector<cv::Vec4f> stoplines,
    const std::pair<float, float>& lanes_position,
    const uint16_t y,
    std::vector<cv::Point>& undistorted_lanes_position,
    const yolov3_trt_ros::BoundingBoxes& predictions)
{
    drawStoplines(img, stoplines);
    undistortLanesPosition(lanes_position, y, undistorted_lanes_position);
    drawLanes(img, undistorted_lanes_position);
    drawBboxes(img, predictions);
    cv::imshow("Result", img);
    cv::waitKey(1);
}
}  // namespace xycar
