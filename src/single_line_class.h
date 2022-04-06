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
    SingleLine(point_t initial_point, float r, float q, rclcpp::Logger logger);

    SingleLine GetCopy();
    point_t GetPoint();

    bool IsAlive(int max_cnt);

    void IncrementWeight(int diff);

    int GetWeight();

    bool IsAboveThreshold();

    void Update(point_t point);
    void Predict(vector_t delta_position, quat_t delta_quat);

private:
    point_t pl_point_;
    kf_est_t estimates[3];

    int alive_cnt_;
    int weight_cnt_;
    int line_weight_threshold_ = 50; // param?

    float r_;
    float q_;

    rclcpp::Logger logger_;
    
};
