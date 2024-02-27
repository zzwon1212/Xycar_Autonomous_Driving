#ifndef LANE_DETECTOR_HPP_
#define LANE_DETECTOR_HPP_

#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>

/// create your lane detecter
/// Class naming.. it's up to you.
namespace Xycar {

enum class Direction : uint8_t
{
    LEFT = 0,  ///< Line direction LEFT
    RIGHT = 1, ///< Line direction RIGHT
};

template <typename PREC>
class LaneDetector final
{
public:
    using Ptr = LaneDetector*; /// < Pointer type of the class(it's up to u)

    static inline const cv::Scalar kRed = {0, 0, 255}; /// Scalar values of Red
    static inline const cv::Scalar kGreen = {0, 255, 0}; /// Scalar values of Green
    static inline const cv::Scalar kBlue = {255, 0, 0}; /// Scalar values of Blue
    static inline const cv::Scalar kBlack = {0, 0, 0}; /// Scalar values of Blue
    static inline const cv::Scalar kWhite = {255, 255, 255}; /// Scalar values of Blue

    LaneDetector(const YAML::Node& config) {
        setConfiguration(config);
        prev_lpos=cv::Point(0, yOffset);
        prev_rpos=cv::Point(mImageWidth, yOffset);
        leftC=0;
        rightC=0;
    }
    // double processImage(cv::Mat& frame);
    std::tuple<double, bool, bool> processImage(cv::Mat& frame);
    void drawRectangle(int32_t leftPositionX, int32_t rightPositionX);

    const cv::Mat& getDebugFrame() const {return mDebugFrame;};
    const cv::Mat& getDebugROI() const {return mDebugROI;};
    int32_t getWidth(){return mImageWidth;};
    void setYOffset(double speed){tempYOffset=yOffset-speed*yGain;}
private:
    int32_t mImageWidth;
    int32_t mImageHeight;
    int32_t low_threshold;
    int32_t high_threshold;
    int32_t min_pixel;
    int32_t min_line;
    int32_t max_gap;
    int32_t yOffset;
    int32_t yGap;
    int32_t tempYOffset;
    double yGain;
    bool mDebugging;
    cv::Mat mDebugFrame;
    cv::Mat mDebugROI;
    cv::Point prev_lpos;
    cv::Point prev_rpos;
    int leftC;
    int rightC;

    std::pair<std::vector<int>, std::vector<int>> divideLeftRight(std::vector<cv::Vec4f>& lines);
    // std::pair<cv::Point, cv::Point> getLinePos(std::vector<int>& left_x_at_Y_offset, std::vector<int>& right_x_at_Y_offset);
    std::tuple<cv::Point, cv::Point, bool, bool> getLinePos(std::vector<int>& left_x_at_Y_offset, std::vector<int>& right_x_at_Y_offset);
    void drawLines(std::vector<cv::Vec4f>& lines);
    void setConfiguration(const YAML::Node& config);
};
}

#endif // LANE_DETECTOR_HPP_