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
    SingleLine(point_t initial_point, float r, float q);

    SingleLine GetCopy();

    point_t GetPoint();

    void Update(point_t point);
    void Predict(vector_t delta_position, quat_t delta_quat);

private:
    point_t pl_point_;
    kf_est_t estimates[3];

    float r_;
    float q_;
    
};
