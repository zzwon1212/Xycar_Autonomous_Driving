#include "autonomous_driving/PID_controller.h"

namespace xycar
{
float PIDController::getPIDOutput(float gap_between_lane_and_car_)
{
    float castError = static_cast<float>(gap_between_lane_and_car_);
    gain_error_D_ = castError - gain_error_P_;
    gain_error_P_ = castError;
    gain_error_I_ += castError;
    return gain_P_*gain_error_P_ + gain_I_*gain_error_I_ + gain_D_*gain_error_D_;
}
}  // namespace Xycar
