#include "ps_ros_lib/help_marker.h"

help_marker::help_marker()
{
}

//------------------------------------------------------------------------------
//==============================================================================

void help_marker::init(ros::NodeHandle &NH)
{
    /* Creating a publisher that publishes to the topic `/marker` of type `visualization_msgs::Marker`. */
    _pub_marker = NH.advertise<visualization_msgs::Marker>("/marker", 0);
}

//------------------------------------------------------------------------------
//==============================================================================

void help_marker::arrow(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> points, float r, float g, float b, float a, float scale_shaft, float scale_head)
{
    if (id == 0)
        return;

    if (points.size() != 2)
        return;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    visualization_msgs::Marker msg_marker;
    //-----
    msg_marker.header.frame_id = frame_id;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = ns;
    msg_marker.id = abs(id);
    //-----
    msg_marker.type = visualization_msgs::Marker::ARROW;
    msg_marker.action = id > 0 ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    //-----
    msg_marker.pose.orientation.w = 1.0;
    msg_marker.scale.x = scale_shaft;
    msg_marker.scale.y = scale_head;
    msg_marker.color = color;
    msg_marker.frame_locked = true;
    //-----
    msg_marker.points = points;
    //-----
    _pub_marker.publish(msg_marker);
}

//------------------------------------------------------------------------------
//==============================================================================

void help_marker::cube(std::string frame_id, std::string ns, int id, geometry_msgs::Point position, geometry_msgs::Quaternion orientation, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z)
{
    if (id == 0)
        return;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    visualization_msgs::Marker msg_marker;
    //-----
    msg_marker.header.frame_id = frame_id;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = ns;
    msg_marker.id = abs(id);
    //-----
    msg_marker.type = visualization_msgs::Marker::CUBE;
    msg_marker.action = id > 0 ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    //-----
    msg_marker.pose.position = position;
    msg_marker.pose.orientation = orientation;
    msg_marker.scale.x = scale_x;
    msg_marker.scale.y = scale_y;
    msg_marker.scale.z = scale_z;
    msg_marker.color = color;
    msg_marker.frame_locked = true;
    //-----
    _pub_marker.publish(msg_marker);
}

void help_marker::sphere(std::string frame_id, std::string ns, int id, geometry_msgs::Point position, geometry_msgs::Quaternion orientation, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z)
{
    if (id == 0)
        return;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    visualization_msgs::Marker msg_marker;
    //-----
    msg_marker.header.frame_id = frame_id;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = ns;
    msg_marker.id = abs(id);
    //-----
    msg_marker.type = visualization_msgs::Marker::SPHERE;
    msg_marker.action = id > 0 ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    //-----
    msg_marker.pose.position = position;
    msg_marker.pose.orientation = orientation;
    msg_marker.scale.x = scale_x;
    msg_marker.scale.y = scale_y;
    msg_marker.scale.z = scale_z;
    msg_marker.color = color;
    msg_marker.frame_locked = true;
    //-----
    _pub_marker.publish(msg_marker);
}

void help_marker::cylinder(std::string frame_id, std::string ns, int id, geometry_msgs::Point position, geometry_msgs::Quaternion orientation, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z)
{
    if (id == 0)
        return;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    visualization_msgs::Marker msg_marker;
    //-----
    msg_marker.header.frame_id = frame_id;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = ns;
    msg_marker.id = abs(id);
    //-----
    msg_marker.type = visualization_msgs::Marker::CYLINDER;
    msg_marker.action = id > 0 ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    //-----
    msg_marker.pose.position = position;
    msg_marker.pose.orientation = orientation;
    msg_marker.scale.x = scale_x;
    msg_marker.scale.y = scale_y;
    msg_marker.scale.z = scale_z;
    msg_marker.color = color;
    msg_marker.frame_locked = true;
    //-----
    _pub_marker.publish(msg_marker);
}

//------------------------------------------------------------------------------
//==============================================================================

void help_marker::line_strip(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> points, float r, float g, float b, float a, float scale)
{
    if (id == 0)
        return;

    if (points.size() < 2)
        return;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    visualization_msgs::Marker msg_marker;
    //-----
    msg_marker.header.frame_id = frame_id;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = ns;
    msg_marker.id = abs(id);
    //-----
    msg_marker.type = visualization_msgs::Marker::LINE_STRIP;
    msg_marker.action = id > 0 ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    //-----
    msg_marker.pose.orientation.w = 1.0;
    msg_marker.scale.x = scale;
    msg_marker.color = color;
    msg_marker.frame_locked = true;
    //-----
    msg_marker.points = points;
    //-----
    _pub_marker.publish(msg_marker);
}

void help_marker::line_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> points, float r, float g, float b, float a, float scale)
{
    if (id == 0)
        return;

    if (points.size() < 2)
        return;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    visualization_msgs::Marker msg_marker;
    //-----
    msg_marker.header.frame_id = frame_id;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = ns;
    msg_marker.id = abs(id);
    //-----
    msg_marker.type = visualization_msgs::Marker::LINE_LIST;
    msg_marker.action = id > 0 ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    //-----
    msg_marker.pose.orientation.w = 1.0;
    msg_marker.scale.x = scale;
    msg_marker.color = color;
    msg_marker.frame_locked = true;
    //-----
    msg_marker.points = points;
    //-----
    _pub_marker.publish(msg_marker);
}

void help_marker::cube_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> positions, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z)
{
    if (id == 0)
        return;

    if (positions.size() < 1)
        return;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    visualization_msgs::Marker msg_marker;
    //-----
    msg_marker.header.frame_id = frame_id;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = ns;
    msg_marker.id = abs(id);
    //-----
    msg_marker.type = visualization_msgs::Marker::CUBE_LIST;
    msg_marker.action = id > 0 ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    //-----
    msg_marker.pose.orientation.w = 1.0;
    msg_marker.scale.x = scale_x;
    msg_marker.scale.y = scale_y;
    msg_marker.scale.z = scale_z;
    msg_marker.color = color;
    msg_marker.frame_locked = true;
    //-----
    msg_marker.points = positions;
    //-----
    _pub_marker.publish(msg_marker);
}

void help_marker::sphere_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::Point> positions, float r, float g, float b, float a, float scale_x, float scale_y, float scale_z)
{
    if (id == 0)
        return;

    if (positions.size() < 1)
        return;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    visualization_msgs::Marker msg_marker;
    //-----
    msg_marker.header.frame_id = frame_id;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = ns;
    msg_marker.id = abs(id);
    //-----
    msg_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    msg_marker.action = id > 0 ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    //-----
    msg_marker.pose.orientation.w = 1.0;
    msg_marker.scale.x = scale_x;
    msg_marker.scale.y = scale_y;
    msg_marker.scale.z = scale_z;
    msg_marker.color = color;
    msg_marker.frame_locked = true;
    //-----
    msg_marker.points = positions;
    //-----
    _pub_marker.publish(msg_marker);
}

//------------------------------------------------------------------------------

void help_marker::text_view_facing(std::string frame_id, std::string ns, int id, geometry_msgs::Point position, geometry_msgs::Quaternion orientation, float r, float g, float b, float a, float scale, std::string text)
{
    if (id == 0)
        return;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    visualization_msgs::Marker msg_marker;
    //-----
    msg_marker.header.frame_id = frame_id;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = ns;
    msg_marker.id = abs(id);
    //-----
    msg_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    msg_marker.action = id > 0 ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    //-----
    msg_marker.pose.position = position;
    msg_marker.pose.orientation = orientation;
    msg_marker.scale.z = scale;
    msg_marker.color = color;
    msg_marker.frame_locked = true;
    //-----
    msg_marker.text = text;
    //-----
    _pub_marker.publish(msg_marker);
}
