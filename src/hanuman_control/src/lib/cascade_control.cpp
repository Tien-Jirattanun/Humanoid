#include "pid_control.hpp"
#include "cascade_control.hpp"

double CascadeControl::CascadeControlFunction(double ref_position, double ref_velocity, double current_position, double current_velocity)
{
    // outer must be slower than inner
    double outer_loop = outer_controller_.PIDControlFunction(ref_position, current_position);
    if (iteration_ % 2 == 1)
    {
        outer_loop = prev_outer;
    }

    double inner_loop = inner_controller_.PIDControlFunction(outer_loop + ref_velocity, current_velocity);

    prev_outer = outer_loop;
    iteration_++;
    return inner_loop;
}