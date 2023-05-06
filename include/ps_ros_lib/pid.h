/**
 * @file pid.h
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-03-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PID_H
#define PID_H

#include "ros/ros.h"

class pid
{
private:
    double _kp, _ki, _kd, _dt;
    double _error, _error_last;
    double _propotional;
    double _derivative;
    double _integral, _min_integral, _max_integral;
    double _output, _min_output, _max_output;

public:
    pid();

    void init(double kp, double ki, double kd, double dt, double min_output = -1, double max_output = 1, double min_integral = -1, double max_integral = 1);

    double update(double error);
    double update(double error, float min_output, float max_output);
    double update(double setpoint, double input);
    double update(double setpoint, double input, float min_output, float max_output);
};

#endif