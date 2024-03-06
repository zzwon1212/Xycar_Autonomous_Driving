#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <cstdint>

namespace xycar
{
/**
 * @brief PID Controller Class
 * @tparam PREC Precision of data
 */
template <typename PREC>
class PIDController
{
public:
    using Ptr = PIDController*;  // Pointer type of this class

    /**
     * @brief Construct a new PID object
     *
     * @param[in] P Proportional control gain
     * @param[in] I Integral control gain to remove error of steady-state
     * @param[in] D Differential control gain to relieve overshoot and improve stability
     */
    PIDController(PREC P, PREC I, PREC D) :
        gain_P_(P), gain_I_(I), gain_D_(D)
    {
    }

    /**
     * @brief Compute and return the PID Control Output
     *
     * @param[in] gap_between_lane_and_car_ Error between estimated position x and half of the image
     * @return The result of PID controller
     */
    PREC getPIDOutput(int32_t gap_between_lane_and_car_);

private:
    const PREC gain_P_;  // Proportional control gain. The higher, the more powerful
    const PREC gain_I_;  // Integral control gain to remove error of steady-state
    const PREC gain_D_;  // Differential control gain to relieve overshoot and improve stability
    PREC gain_error_P_ = 0.0;  // Error to determine how much the proportional gain should be reflected
    PREC gain_error_I_ = 0.0;  // Error to determine how much the integral gain should be reflected
    PREC gain_error_D_ = 0.0;  // Error to determine how much the differential gain should be reflected
};
}  // namespace Xycar

#endif  // PID_CONTROLLER_H_
