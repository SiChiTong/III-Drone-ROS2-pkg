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
#include <rclcpp/rclcpp.hpp>

#include "geometry.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

typedef struct {

    float state_est;
    float var_est;

} kf_est_t;

/*****************************************************************************/
// Class
/*****************************************************************************/

class SingleLine
{
public:
    SingleLine(int id, point_t initial_point, float r, float q, 
    rclcpp::Logger logger, int alive_cnt_low_thresh, int alive_cnt_high_thresh, int alive_cnt_ceiling);

    SingleLine GetCopy();
    point_t GetPoint();

    bool IsAlive();
    bool IsVisible();

    void Update(point_t point);
    void Predict(vector_t delta_position, quat_t delta_quat);

    int GetId();

private:
    point_t pl_point_;
    kf_est_t estimates[3];

    point_t projected_point_;

    int id_;

    int alive_cnt_;
    int alive_cnt_low_thresh_;
    int alive_cnt_high_thresh_;
    int alive_cnt_ceiling_;

    float r_;
    float q_;

    rclcpp::Logger logger_;
    
};
