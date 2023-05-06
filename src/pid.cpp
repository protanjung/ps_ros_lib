#include "ps_ros_lib/pid.h"

pid::pid()
{
}

//------------------------------------------------------------------------------
//==============================================================================

void pid::init(double kp, double ki, double kd, double dt, double min_output, double max_output, double min_integral, double max_integral)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _dt = dt;
    _min_output = min_output;
    _max_output = max_output;
    _min_integral = min_integral;
    _max_integral = max_integral;
}

//------------------------------------------------------------------------------
//==============================================================================

double pid::update(double error)
{
    /* Calculating the time difference between the last time
    the function was called and the current time. */
    static ros::Time time_last, time_now;
    time_last = time_now;
    time_now = ros::Time::now();
    ros::Duration dt = time_now - time_last;

    /* Reset the error last and integral
    if the time difference between the last time the function was
    called and the current time is greater than 1 second. */
    if (time_now - time_last > ros::Duration(1.0))
    {
        _error_last = 0;
        _integral = 0;
    }

    /* Calculating the PID's each term. */
    _error = error;
    _propotional = _kp * _error;
    _derivative = _kd * (_error - _error_last) / _dt;
    _integral += _ki * _error * _dt;

    /* This is a saturation function. It is used to limit the integral term. */
    if (_integral > _max_integral)
        _integral = _max_integral;
    else if (_integral < _min_integral)
        _integral = _min_integral;

    /* This is the PID equation. */
    _output = _propotional + _derivative + _integral;

    /* This is a saturation function. It is used to limit the PID output. */
    if (_output > _max_output)
        _output = _max_output;
    else if (_output < _min_output)
        _output = _min_output;

    /* Saving the error from the last time the function was called. */
    _error_last = _error;

    return _output;
}

double pid::update(double error, float min_output, float max_output)
{
    _min_output = min_output;
    _max_output = max_output;
    _min_integral = min_output;
    _max_integral = max_output;

    return update(error);
}

double pid::update(double setpoint, double input)
{
    return update(setpoint - input);
}

double pid::update(double setpoint, double input, float min_output, float max_output)
{
    _min_output = min_output;
    _max_output = max_output;
    _min_integral = min_output;
    _max_integral = max_output;

    return update(setpoint, input);
}