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

double pid::update(double setpoint, double input)
{
    static ros::Time time_last, time_now;

    time_last = time_now;
    time_now = ros::Time::now();

    if (time_now - time_last > ros::Duration(1.0))
    {
        _error_last = 0;
        _integral = 0;
    }

    _error = setpoint - input;
    _propotional = _kp * _error;
    _derivative = _kd * (_error - _error_last) / _dt;
    _integral += _ki * _error * _dt;

    if (_integral > _max_integral)
        _integral = _max_integral;
    else if (_integral < _min_integral)
        _integral = _min_integral;

    _output = _propotional + _derivative + _integral;

    if (_output > _max_output)
        _output = _max_output;
    else if (_output < _min_output)
        _output = _min_output;

    _error_last = _error;

    return _output;
}