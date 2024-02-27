#include "autonomous_driving/PID_controller.hpp"

namespace xycar {
template <typename PREC>
PREC PIDController<PREC>::GetPIDOutput(int32_t gap_between_laneNcar_)
{
    PREC castError = static_cast<PREC>(gap_between_laneNcar_);
    gain_error_D_ = castError - gain_error_P_;
    gain_error_P_ = castError;
    gain_error_I_ += castError;
    return gain_P_*gain_error_P_ + gain_I_*gain_error_I_ + gain_D_*gain_error_D_;
}

template class PIDController<float>;
template class PIDController<double>;
}  // namespace Xycar
