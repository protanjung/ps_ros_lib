#include "ps_ros_lib/help_log.h"

help_log::help_log()
{
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * @brief It is a private function that is used by the public functions to publish the log message.
 *
 * @param header The header of the log message.
 * @param format The format of the log message.
 * @param arg The arguments of the log message.
 */
void help_log::log(const char *header, const char *format, va_list arg)
{
    /* A way to print a string with a variable number of arguments. */
    char body[1024];
    vsnprintf(body, sizeof(body), format, arg);

    /* Getting the current time and converting it to a string. */
    boost::posix_time::ptime time_ptime = boost::posix_time::microsec_clock::local_time();
    std::string time_string = boost::posix_time::to_simple_string(time_ptime);

    /* Publishing the log message to the topic /log. */
    ps_ros_lib::log msg_log;
    msg_log.node_namespace = _node_namespace;
    msg_log.node_name = _node_name;
    msg_log.log_datetime = time_string.substr(12, 12);
    msg_log.log_header = std::string(header);
    msg_log.log_body = std::string(body);
    _pub_log.publish(msg_log);
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * @brief It is a public function that is used to initialize the help_log library.
 *
 * @param NH The node handle of the node that is using this library.
 */
void help_log::init(ros::NodeHandle &NH)
{
    /* Creating a publisher that publishes to the topic `/log` of type `ps_ros_lib::log`. */
    _pub_log = NH.advertise<ps_ros_lib::log>("/log", 0);

    /* Getting the namespace and name of the node that is using this library. */
    _node_namespace = ros::this_node::getNamespace();
    _node_name = ros::this_node::getName();
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * @brief It is a public function that is used to publish a debug log message.
 *
 * @param format The format of the log message.
 * @param ... The arguments of the log message.
 */
void help_log::debug(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    log("DEBUG", format, arg);
    va_end(arg);
}

/**
 * @brief It is a public function that is used to publish an info log message.
 *
 * @param format The format of the log message.
 * @param ... The arguments of the log message.
 */
void help_log::info(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    log("INFO", format, arg);
    va_end(arg);
}

/**
 * @brief It is a public function that is used to publish a warn log message.
 *
 * @param format The format of the log message.
 * @param ... The arguments of the log message.
 */
void help_log::warn(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    log("WARN", format, arg);
    va_end(arg);
}

/**
 * @brief It is a public function that is used to publish an error log message.
 *
 * @param format The format of the log message.
 * @param ... The arguments of the log message.
 */
void help_log::error(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    log("ERROR", format, arg);
    va_end(arg);
}

/**
 * @brief It is a public function that is used to publish a fatal log message.
 *
 * @param format The format of the log message.
 * @param ... The arguments of the log message.
 */
void help_log::fatal(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    log("FATAL", format, arg);
    va_end(arg);
}