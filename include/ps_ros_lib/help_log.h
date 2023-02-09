/**
 * @file help_log.h
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef HELP_LOG_H
#define HELP_LOG_H

#include "boost/date_time.hpp"
#include "ps_ros_lib/log.h"
#include "ros/ros.h"

class help_log
{
private:
    ros::Publisher _pub_log;

    std::string _node_namespace;
    std::string _node_name;

    void log(const char *header, const char *format, va_list arg);

public:
    help_log();

    void init(ros::NodeHandle &NH);

    void debug(const char *format, ...);
    void info(const char *format, ...);
    void warn(const char *format, ...);
    void error(const char *format, ...);
    void fatal(const char *format, ...);
};

#endif