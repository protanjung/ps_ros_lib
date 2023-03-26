/**
 * @file help_marker.h
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef HELP_MARKER_H
#define HELP_MARKER_H

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

class help_marker
{
private:
    ros::Publisher _pub_marker;

public:
    help_marker();

    void init(ros::NodeHandle &NH);

    void arrow(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> points, float r, float g, float b, float a, float scale_shaft, float scale_head);

    void cube(std::string frame_id, std::string ns, int id, geometry_msgs::Point position, geometry_msgs::Quaternion orientation, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z);
    void sphere(std::string frame_id, std::string ns, int id, geometry_msgs::Point position, geometry_msgs::Quaternion orientation, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z);
    void cylinder(std::string frame_id, std::string ns, int id, geometry_msgs::Point position, geometry_msgs::Quaternion orientation, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z);

    void line_strip(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> points, float r, float g, float b, float a, float scale);
    void line_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> points, float r, float g, float b, float a, float scale);
    void cube_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> positions, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z);
    void sphere_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> positions, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z);

    void text_view_facing(std::string frame_id, std::string ns, int id, geometry_msgs::Point position, geometry_msgs::Quaternion orientation, float r, float g, float b, float a, float scale, std::string text);
};

#endif