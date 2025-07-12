#ifndef CASCADE_CONTROL_HPP
#define CASCADE_CONTROL_HPP

#include "pid_control.hpp"

class CascadeControl 
{
public:
    CascadeControl(double outer_kp, double outer_ki, double outer_kd, double outer_u_max, double outer_u_min,
                 double inner_kp, double inner_ki, double inner_kd, double inner_u_max, double inner_u_min)
        : outer_controller_(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min),
          inner_controller_(inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min)
    {}

    // Keeping original function name and parameters
    double CascadeControlFunction(double ref_position, double ref_velocity, double current_position, double current_velocity);

private:
    PIDControl outer_controller_;  // Position controller (outputs velocity reference)
    PIDControl inner_controller_;  // Velocity controller (outputs actual control signal)
};

#endif