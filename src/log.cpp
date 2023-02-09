#include "ps_ros_lib/log.h"
#include "boost/date_time.hpp"
#include "boost/filesystem.hpp"
#include "ps_ros_lib/color.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_sub_log(const ps_ros_lib::logConstPtr &msg);

int log_init();
int log_routine();

std::string folder_to_make();
std::string folder_to_remove();
void make_folder(std::string path);
void remove_folder(std::string path);

//=====Parameter
std::string log_path;
//=====Subscriber
ros::Subscriber sub_log;

//-----Log file
//=============
std::ofstream log_file;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "log");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("/log/path", log_path);
    //=====Subscriber
    sub_log = NH.subscribe("/log", 0, cllbck_sub_log);

    if (log_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_log(const ps_ros_lib::log::ConstPtr &msg)
{
    /* Making a folder for the current date and removing the folder for 180 days ago. */
    make_folder(folder_to_make());
    remove_folder(folder_to_remove());

    /* Getting the current time and date. */
    boost::posix_time::ptime time_ptime = boost::posix_time::microsec_clock::local_time();
    std::string time_string = boost::posix_time::to_simple_string(time_ptime);

    /* Coloring the log messages based on the log level. */
    std::string node_namespace = color::rize(msg->node_namespace, "Yellow");
    std::string node_name = color::rize(msg->node_name, "Yellow");
    std::string log_datetime = color::rize(msg->log_datetime, "Green");
    std::string log_header = color::rize(msg->log_header, "Blue");
    std::string log_body;
    if (msg->log_header.find("DEBUG") != std::string::npos)
        log_body = color::rize(msg->log_body, "Cyan");
    else if (msg->log_header.find("INFO") != std::string::npos)
        log_body = color::rize(msg->log_body, "White");
    else if (msg->log_header.find("WARN") != std::string::npos)
        log_body = color::rize(msg->log_body, "Yellow");
    else if (msg->log_header.find("ERROR") != std::string::npos)
        log_body = color::rize(msg->log_body, "Red");
    else if (msg->log_header.find("FATAL") != std::string::npos)
        log_body = color::rize(msg->log_body, "Magenta");

    /* Printing the log message to the terminal. */
    std::cout << log_datetime << " [" << log_header << "] [" << node_namespace << " | " << node_name << "] " << log_body << std::endl;

    /* Writing the log message to the log file. */
    log_file.open(folder_to_make() + "/" + time_string.substr(0, 11) + ".txt", std::ofstream::out | std::ofstream::app);
    log_file << msg->log_datetime << " [" << msg->log_header << "] [" << msg->node_namespace << " | " << msg->node_name << "] " << msg->log_body << std::endl;
    log_file.close();
}

//------------------------------------------------------------------------------
//==============================================================================

int log_init()
{
    if (!boost::filesystem::exists(log_path))
        boost::filesystem::create_directories(log_path);

    return 0;
}

int log_routine()
{
    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * @brief It returns the path of the folder to be created.
 *
 * @return std::string
 */
std::string folder_to_make()
{
    boost::posix_time::ptime time_ptime = boost::posix_time::second_clock::local_time();
    std::string time_string = boost::posix_time::to_simple_string(time_ptime);

    return log_path + "/" + time_string.substr(0, 8);
}

/**
 * @brief It returns the path of the folder to be deleted.
 *
 * @return std::string
 */
std::string folder_to_remove()
{
    boost::posix_time::ptime time_ptime = boost::posix_time::second_clock::local_time() - boost::posix_time::hours(180 * 24);
    std::string time_string = boost::posix_time::to_simple_string(time_ptime);

    return log_path + "/" + time_string.substr(0, 8);
}

/**
 * @brief It creates a folder if it doesn't exist.
 *
 * @param path Path of the folder to be created.
 */
void make_folder(std::string path)
{
    if (!boost::filesystem::exists(path))
        boost::filesystem::create_directories(path);
}

/**
 * @brief It removes a folder and all its contents.
 *
 * @param path Path of the folder to be deleted.
 */
void remove_folder(std::string path)
{
    if (boost::filesystem::exists(path))
        boost::filesystem::remove_all(path);
}