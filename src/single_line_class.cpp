/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "single_line_class.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

SingleLine::SingleLine(point_t initial_point, float r, float q, rclcpp::Logger logger) : logger_(logger) {

    pl_point_ = point_t(
        initial_point(0),
        initial_point(1),
        initial_point(2)
    );

    for (int i = 0; i < 3; i++) {

        estimates[i] = { .state_est = initial_point[i], .var_est = 0};

    }

    r_ = r;
    q_ = q;

    alive_cnt_ = 0;

}

SingleLine SingleLine::GetCopy() {

    SingleLine sl(pl_point_, r_, q_, logger_);

    return sl;

}

point_t SingleLine::GetPoint() {

    return pl_point_;

}

bool SingleLine::IsAlive(int max_cnt) {

    bool in_FOV = estimates[2].state_est > 0.25 && estimates[2].state_est < 20;

    if (in_FOV && ++alive_cnt_ >= max_cnt) {

        return false;

    } else {

        return true;

    }
}


void SingleLine::Update(point_t point) {

    for (int i = 0; i < 3; i++) {

        float y_bar = point[i] - estimates[i].state_est;
        float s = estimates[i].var_est + r_;

        float k = estimates[i].var_est / s;

        estimates[i].state_est += k*y_bar;
        estimates[i].var_est *= 1-k;

        pl_point_(i) = estimates[i].state_est;

    }

    alive_cnt_ = 0;

}

void SingleLine::Predict(vector_t delta_position, quat_t delta_quat) {

    rotation_matrix_t R = quatToMat(delta_quat);

    RCLCPP_INFO(logger_, "Point before: (%f, %f, %f)", pl_point_(0), pl_point_(1), pl_point_(2));

    pl_point_ = (R * pl_point_) + delta_position;

    RCLCPP_INFO(logger_, "Delta position: (%f, %f, %f)", delta_position(0), delta_position(1), delta_position(2));

    RCLCPP_INFO(logger_, "Point after: (%f, %f, %f)", pl_point_(0), pl_point_(1), pl_point_(2));

    for (int i = 0; i < 3; i++) {

        estimates[i].state_est = pl_point_(i);
        estimates[i].var_est += q_;
    }

}