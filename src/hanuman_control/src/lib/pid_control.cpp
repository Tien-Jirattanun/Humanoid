#include "pid_control.hpp"

double PIDControl::PIDControlFunction(double setpoint, double current_position) {
    double error = setpoint - current_position;

    // PID velocity form
    double delta_u = kp_ * (error - e_prev_one_) +
                     ki_ * error +
                     kd_ * (error - 2 * e_prev_one_ + e_prev_two_);

    double u_new = u_prev_;

    // Only update if not saturated
    if (!((u_prev_ >= u_max_ && delta_u > 0) || (u_prev_ <= u_min_ && delta_u < 0))) {
        u_new += delta_u;
    }

    // Clamp output
    if (u_new > u_max_) u_new = u_max_;
    if (u_new < u_min_) u_new = u_min_;

    // Update state
    e_prev_two_ = e_prev_one_;
    e_prev_one_ = error;
    u_prev_ = u_new;

    return u_new;
}