#include "autonomous_driving/PID_controller.h"

namespace xycar
{
PIDController::PIDController(float P, float I, float D) :
    gain_P_(P), gain_I_(I), gain_D_(D)
{
}

float PIDController::getPIDOutput(float gap_between_lane_and_car_)
{
    float castError = static_cast<float>(gap_between_lane_and_car_);
    gain_error_D_ = castError - gain_error_P_;
    gain_error_P_ = castError;
    gain_error_I_ += castError;
    return gain_P_*gain_error_P_ + gain_I_*gain_error_I_ + gain_D_*gain_error_D_;
}
}  // namespace Xycar
