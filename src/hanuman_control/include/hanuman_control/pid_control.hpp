#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP

class PIDControl 
{
public:
    PIDControl(double kp, double ki, double kd, double u_max, double u_min) 
    : kp_(kp), ki_(ki), kd_(kd), u_max_(u_max), u_min_(u_min),
      e_prev_one_(0.0f), e_prev_two_(0.0f), u_prev_(0.0f) {}

    double PIDControlFunction(double setpoint, double current_position);

    void SetterParam(double kp, double ki, double kd);

    void SetterZero();

private:
    double kp_;        // Proportional gain
    double ki_;        // Integral gain
    double kd_;        // Derivative gain
    double u_max_;     // Maximum control output
    double u_min_;     // Minimum control output
    double e_prev_one_; // Previous error for derivative calculation
    double e_prev_two_; // Second previous error for derivative calculation
    double u_prev_;
};

#endif
