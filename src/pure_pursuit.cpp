#include "ps_ros_lib/pure_pursuit.h"

pure_pursuit::pure_pursuit()
{
}

//------------------------------------------------------------------------------
//==============================================================================

void pure_pursuit::updateGoal()
{
    static uint8_t statusNormal = 0;
    static uint32_t indexNormal = 1;

    if (indexNormal >= _currentRoute->size())
    {
        statusNormal = 0;
        indexNormal = 1;
    }

    //----------------------------------

    // Initialize statusNormal to always
    // be 0 at the beginning of the function.
    statusNormal = 0;

    // Normal scan #1, continue from previous index until last index
    if (statusNormal == 0)
    {
        for (int i = indexNormal; i < _currentRoute->size(); i++)
        {
            float distanceBackward = sqrtf(powf(_currentRoute->at(i - 1).x - _currentPose->x, 2) +
                                           powf(_currentRoute->at(i - 1).y - _currentPose->y, 2));
            float distanceForward = sqrtf(powf(_currentRoute->at(i - 0).x - _currentPose->x, 2) +
                                          powf(_currentRoute->at(i - 0).y - _currentPose->y, 2));
            if (distanceBackward < lookAheadDistance &&
                distanceForward > lookAheadDistance)
            {
                statusNormal = 1;
                indexNormal = i;
                break;
            }
        }
    }

    // Normal scan #2, start from index 0 until previous index
    if (statusNormal == 0)
    {
        for (int i = 1; i < indexNormal; i++)
        {
            float distanceBackward = sqrtf(powf(_currentRoute->at(i - 1).x - _currentPose->x, 2) +
                                           powf(_currentRoute->at(i - 1).y - _currentPose->y, 2));
            float distanceForward = sqrtf(powf(_currentRoute->at(i - 0).x - _currentPose->x, 2) +
                                          powf(_currentRoute->at(i - 0).y - _currentPose->y, 2));
            if (distanceBackward < lookAheadDistance &&
                distanceForward > lookAheadDistance)
            {
                statusNormal = 1;
                indexNormal = i;
                break;
            }
        }
    }

    // Abnormal scan, find closest point
    if (statusNormal == 0)
    {
        float nearestDistance = __FLT_MAX__;

        for (int i = 1; i < _currentRoute->size(); i++)
        {
            float distance = sqrtf(powf(_currentRoute->at(i).x - _currentPose->x, 2) +
                                   powf(_currentRoute->at(i).y - _currentPose->y, 2));
            if (distance < nearestDistance)
            {
                nearestDistance = distance;

                statusNormal = 1;
                indexNormal = i;
            }
        }
    }

    //----------------------------------

    goalIndexStart = indexNormal - 1;
    goalIndexStop = indexNormal - 0;

    //----------------------------------

    float xStop_Minus_xStart = _currentRoute->at(goalIndexStop).x - _currentRoute->at(goalIndexStart).x;
    float yStop_Minus_yStart = _currentRoute->at(goalIndexStop).y - _currentRoute->at(goalIndexStart).y;
    float xStart_Minus_xPose = _currentRoute->at(goalIndexStart).x - _currentPose->x;
    float yStart_Minus_yPose = _currentRoute->at(goalIndexStart).y - _currentPose->y;

    float a = xStop_Minus_xStart * xStop_Minus_xStart + yStop_Minus_yStart * yStop_Minus_yStart;
    float b = 2 * (xStart_Minus_xPose * xStop_Minus_xStart + yStart_Minus_yPose * yStop_Minus_yStart);
    float c = xStart_Minus_xPose * xStart_Minus_xPose + yStart_Minus_yPose * yStart_Minus_yPose - lookAheadDistance * lookAheadDistance;

    float discriminant = b * b - 4 * a * c;

    //----------------------------------

    if (discriminant < 0)
    {
        goalPositionX = _currentRoute->at(goalIndexStop).x;
        goalPositionY = _currentRoute->at(goalIndexStop).y;
    }
    else
    {
        float t1 = (-b + sqrtf(discriminant)) / (2 * a);
        float t2 = (-b - sqrtf(discriminant)) / (2 * a);

        if (t1 >= 0 && t1 <= 1)
        {
            goalPositionX = _currentRoute->at(goalIndexStart).x + t1 * xStop_Minus_xStart;
            goalPositionY = _currentRoute->at(goalIndexStart).y + t1 * yStop_Minus_yStart;
        }
        else if (t2 >= 0 && t2 <= 1)
        {
            goalPositionX = _currentRoute->at(goalIndexStart).x + t2 * xStop_Minus_xStart;
            goalPositionY = _currentRoute->at(goalIndexStart).y + t2 * yStop_Minus_yStart;
        }
        else
        {
            goalPositionX = _currentRoute->at(goalIndexStop).x;
            goalPositionY = _currentRoute->at(goalIndexStop).y;
        }
    }
}

void pure_pursuit::updateICR()
{
    float alpha = atan2f(goalPositionY - _currentPose->y, goalPositionX - _currentPose->x) - _currentPose->theta;
    icrRadius = lookAheadDistance / (2 * sinf(alpha));
    icrPositionX = _currentPose->x + icrRadius * cosf(_currentPose->theta + M_PI_2);
    icrPositionY = _currentPose->y + icrRadius * sinf(_currentPose->theta + M_PI_2);
}

void pure_pursuit::updateSteering()
{
}

//------------------------------------------------------------------------------
//==============================================================================

void pure_pursuit::init(geometry_msgs::Pose2D *currentPose, std::vector<geometry_msgs::Point> *currentRoute, float wheelBase, float lookAheadDistance)
{
    this->_currentPose = currentPose;
    this->_currentRoute = currentRoute;
    this->wheelBase = wheelBase;
    this->lookAheadDistance = lookAheadDistance;
}

//------------------------------------------------------------------------------
//==============================================================================

void pure_pursuit::setCurrentPose(geometry_msgs::Pose2D *currentPose)
{
    this->_currentPose = currentPose;
}

void pure_pursuit::setCurrentRoute(std::vector<geometry_msgs::Point> *currentRoute)
{
    this->_currentRoute = currentRoute;
}

void pure_pursuit::setWheelBase(float wheelBase)
{
    this->wheelBase = wheelBase;
}

void pure_pursuit::setLookAheadDistance(float lookAheadDistance)
{
    this->lookAheadDistance = lookAheadDistance;
}

//------------------------------------------------------------------------------
//==============================================================================

void pure_pursuit::updateAll()
{
    if (_currentRoute->size() < 2)
        return;

    updateGoal();
    updateICR();
    updateSteering();
}