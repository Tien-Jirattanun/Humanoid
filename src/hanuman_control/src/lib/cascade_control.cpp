#include "pid_control.hpp"
#include "cascade_control.hpp"

double CascadeControl::CascadeControlFunction(double ref_position, double ref_velocity, double current_position, double current_velocity)
{
    double outer_control_output = outer_controller_.PIDControlFunction(ref_position, current_position);
    double inner_control_output = inner_controller_.PIDControlFunction(ref_velocity + outer_control_output, current_velocity);
    return inner_control_output;
}