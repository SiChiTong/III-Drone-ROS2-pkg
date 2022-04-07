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
#include <rclcpp/rclcpp.hpp>

#include "geometry.h"
#include "single_line_class.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define POINT_MATCH_DISTANCE_THRESHOLD 2.

/*****************************************************************************/
// Class
/*****************************************************************************/

class Powerline
{
public:
    Powerline(float r, float q, rclcpp::Logger logger);

    std::vector<SingleLine> GetLines();
    quat_t GetDirection();
    plane_t GetProjectionPlane();
    //orientation_t GetPlaneOrientation();
    quat_t GetQuat();

    point_t UpdateLine(point_t point);
    void UpdateDirection(quat_t pl_direction);
    void UpdateOdometry(point_t position, quat_t quat);

    void CleanupLines();

private:
    std::mutex lines_mutex_;
    std::mutex direction_mutex_;
    std::mutex odometry_mutex_;
    std::mutex projection_plane_mutex_;

    std::vector<SingleLine> lines_;
    quat_t direction_;
    //float last_global_input_direction_;
    //float last_global_output_direction_;
    //float last_last_global_output_direction_;
    point_t position_;
    quat_t quat_;
    point_t last_position_;
    quat_t last_quat_;
    plane_t projection_plane_;
    //orientation_t plane_orientation_;

    float r_, q_;

    rclcpp::Logger logger_;

    void updateProjectionPlane();

    int findMatchingLine(point_t point);
    point_t projectPoint(point_t point);

    void predictLines();
    
};
