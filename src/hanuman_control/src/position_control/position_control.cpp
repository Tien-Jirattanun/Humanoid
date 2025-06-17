#include "position_control.hpp"

double PositionControl::PIDControl(double setpoint, double current_position) {
    double error = setpoint - current_position;
    double delta_u = 0.0;

    delta_u = kp_ * (error - e_prev_one_) +   
              ki_ * error +                    
              kd_ * (error - 2 * e_prev_one_ + e_prev_two_);  

    if (!((u_prev_ >= u_max_ && delta_u > 0) || (u_prev_ <= u_min_ && delta_u < 0))) {
        delta_u = u_prev_ + delta_u;  
    } else {
        delta_u = u_prev_;  
    }

    delta_u = (delta_u > u_max_) ? u_max_ : delta_u;
    delta_u = (delta_u < u_min_) ? u_min_ : delta_u;

    e_prev_two_ = e_prev_one_;
    e_prev_one_ = error;
    u_prev_ = delta_u;

    return delta_u;
}