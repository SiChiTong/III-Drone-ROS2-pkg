#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <math.h>
#include <limits>
#include <eigen3/Eigen/Core>

#include "geometry.h"
#include "single_line_class.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define POINT_MATCH_DISTANCE_THRESHOLD 0.5

/*****************************************************************************/
// Class
/*****************************************************************************/

class Powerline
{
public:
    Powerline(float r, float q);

    std::vector<SingleLine> GetLines();
    float GetDirection();

    void UpdateLine(point_t point);
    void UpdateDirection(float direction);
    void UpdateOdometry(point_t position, quat_t quat);

private:
    std::mutex lines_mutex_;
    std::mutex direction_mutex_;
    std::mutex odometry_mutex_;
    std::mutex projection_plane_mutex_;

    std::vector<SingleLine> lines_;
    float direction_;
    point_t position_;
    quat_t quat_;
    point_t last_position_;
    quat_t last_quat_;
    plane_t projection_plane_;

    float r_, q_;

    void updateProjectionPlane();

    int findMatchingLine(point_t point);
    point_t projectPoint(point_t point);

    void predictLines();
    
};
