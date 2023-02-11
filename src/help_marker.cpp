#include "ps_ros_lib/help_marker.h"

help_marker::help_marker()
{
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * @brief It is a public function that is used to initialize the help_marker class.
 *
 * @param NH The node handle of the node that is using this library.
 */
void help_marker::init(ros::NodeHandle &NH)
{
    /* Creating a publisher that publishes to the topic `/marker` of type `visualization_msgs::Marker`. */
    _pub_marker = NH.advertise<visualization_msgs::Marker>("/marker", 0);
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * It publishes a marker of type ARROW to the topic /marker
 *
 * @param frame_id The frame in which the marker is to be displayed.
 * @param ns The namespace to which this marker belongs.
 * @param id The id of the marker. If the id is 0, the marker will not be published. If the id is
 * positive, the marker will be added. If the id is negative, the marker will be deleted.
 * @param points a vector of two points, the first point is the start of the arrow, the second point is
 * the end of the arrow.
 * @param r red
 * @param g green
 * @param b blue
 * @param a frame_id: the frame in which the marker is to be displayed.
 * @param scale_shaft The diameter of the arrow shaft.
 * @param scale_head The size of the arrow head.
 *
 * @return Nothing is being returned.
 */
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

/**
 * It publishes a marker of type CUBE to the topic /marker
 *
 * @param frame_id The frame in which the marker is to be displayed.
 * @param ns The namespace to which this marker belongs.
 * @param id The id of the marker. If the id is 0, the marker will not be published. If the id is
 * positive, the marker will be added. If the id is negative, the marker will be deleted.
 * @param position The position of the marker in the frame_id frame.
 * @param orientation the orientation of the marker in the frame of reference
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha (transparency)
 * @param scale_x The length of the cube along the x axis.
 * @param scale_y The length of the cube along the y axis.
 * @param scale_z The length of the cube along the z axis.
 *
 * @return Nothing.
 */
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

/**
 * It publishes a marker of type CUBE to the topic /marker
 *
 * @param frame_id The frame in which the marker is to be displayed.
 * @param ns The namespace to which this marker belongs.
 * @param id The id of the marker. If the id is 0, the marker will not be published. If the id is
 * positive, the marker will be added. If the id is negative, the marker will be deleted.
 * @param position The position of the marker in the frame_id frame.
 * @param orientation the orientation of the marker
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha (transparency)
 * @param scale_x The radius of the sphere.
 * @param scale_y The radius of the sphere.
 * @param scale_z The radius of the sphere.
 *
 * @return Nothing is being returned.
 */
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

/**
 * It publishes a marker of type CYLINDER to the topic /marker
 *
 * @param frame_id The frame in which the marker is to be displayed.
 * @param ns The namespace to which this marker belongs.
 * @param id The id of the marker. If the id is 0, the marker will not be published. If the id is
 * positive, the marker will be added. If the id is negative, the marker will be deleted.
 * @param position The position of the marker in the frame_id frame.
 * @param orientation the orientation of the marker in the frame of reference
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha (transparency)
 * @param scale_x radius of the cylinder
 * @param scale_y radius of the cylinder
 * @param scale_z The height of the cylinder
 *
 * @return Nothing is being returned.
 */
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

/**
 * It publishes a marker of type LINE_STRIP to the topic /marker
 *
 * @param frame_id The frame in which the marker is to be displayed.
 * @param ns The namespace to which this marker belongs.
 * @param id The id of the marker. If the id is 0, the marker will not be published. If the id is
 * positive, the marker will be added. If the id is negative, the marker will be deleted.
 * @param points a vector of geometry_msgs::Point objects
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha (transparency)
 * @param scale The thickness of the line.
 *
 * @return A visualization_msgs::Marker message.
 */
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

/**
 * It publishes a a marker of type LINE_LIST to the topic /marker
 *
 * @param frame_id The frame in which the marker is to be displayed.
 * @param ns The namespace to which this marker belongs.
 * @param id The id of the marker. If the id is 0, the marker will not be published. If the id is
 * positive, the marker will be added. If the id is negative, the marker will be deleted.
 * @param points a vector of geometry_msgs::Point objects
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha (transparency)
 * @param scale The scale of the line.
 *
 * @return Nothing is being returned.
 */
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

/**
 * It publishes a marker of type CUBE_LIST to the topic /marker
 *
 * @param frame_id The frame in which the marker is to be displayed.
 * @param ns The namespace to which this marker belongs.
 * @param id The id of the marker. If the id is 0, the marker will not be published. If the id is
 * positive, the marker will be added. If the id is negative, the marker will be deleted.
 * @param positions a vector of geometry_msgs::Point objects
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha (transparency)
 * @param scale_x The scale of the cube in the x direction.
 * @param scale_y The scale of the cube in the y direction.
 * @param scale_z The scale of the cube in the z direction.
 */
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

/**
 * It publishes a marker of type SPHERE_LIST to the topic /marker
 *
 * @param frame_id The frame in which the marker is to be displayed.
 * @param ns The namespace to which this marker belongs.
 * @param id The id of the marker. If the id is 0, the marker will not be published. If the id is
 * positive, the marker will be added. If the id is negative, the marker will be deleted.
 * @param positions a vector of geometry_msgs::Point objects
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha (transparency)
 * @param scale_x The scale of the sphere in the x direction.
 * @param scale_y The scale of the sphere in the y direction.
 * @param scale_z The scale of the sphere in the z direction.
 */
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