/**
 * @file pure_pursuit.h
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"

class pure_pursuit
{
private:
    geometry_msgs::Pose2D *_currentPose;
    std::vector<geometry_msgs::Point> *_currentRoute;

    void updateGoal();
    void updateICR();
    void updateSteering();
    void updateIndexNearest();

public:
    float wheelBase;
    float lookAheadDistance;
    uint32_t goalIndexStart;
    uint32_t goalIndexStop;
    float goalPositionX;
    float goalPositionY;
    float icrRadius;
    float icrPositionX;
    float icrPositionY;
    float steeringAngle;
    uint32_t indexNearest;

    pure_pursuit();

    void init(geometry_msgs::Pose2D *currentPose, std::vector<geometry_msgs::Point> *currentRoute, float wheelBase, float lookAheadDistance);

    void setCurrentPose(geometry_msgs::Pose2D *currentPose);
    void setCurrentRoute(std::vector<geometry_msgs::Point> *currentRoute);
    void setWheelBase(float wheelBase);
    void setLookAheadDistance(float lookAheadDistance);

    void updateAll();
};

#endif