/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "single_line_class.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

SingleLine::SingleLine(int id, point_t initial_point, float r, float q, 
    rclcpp::Logger logger, int alive_cnt_low_thresh, int alive_cnt_high_thresh, int alive_cnt_ceiling) : logger_(logger) {

    id_ = id;

    projected_point_ = initial_point;

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

    alive_cnt_low_thresh_ = alive_cnt_low_thresh;
    alive_cnt_high_thresh_ = alive_cnt_high_thresh;
    alive_cnt_ceiling_ = alive_cnt_ceiling;

    alive_cnt_ = (alive_cnt_low_thresh_ + alive_cnt_high_thresh_) / 10;

}

int SingleLine::GetId() {

    return id_;

}

SingleLine SingleLine::GetCopy() {

    SingleLine sl(id_, pl_point_, r_, q_, 
        logger_, alive_cnt_low_thresh_, alive_cnt_high_thresh_, alive_cnt_ceiling_);

    return sl;

}

point_t SingleLine::GetPoint() {

    return pl_point_;

}

bool SingleLine::IsAlive() {

    bool in_FOV = estimates[2].state_est > 0.5 && estimates[2].state_est < 20;

    if (in_FOV && --alive_cnt_ <= alive_cnt_low_thresh_) {

        return false;

    } else {

        return true;

    }
}

bool SingleLine::IsVisible() {

    return alive_cnt_ >= alive_cnt_high_thresh_;

}


void SingleLine::Update(point_t point) {

    projected_point_ = point;

    for (int i = 0; i < 3; i++) {

        float y_bar = point[i] - estimates[i].state_est;
        float s = estimates[i].var_est + r_;

        float k = estimates[i].var_est / s;

        estimates[i].state_est += k*y_bar;
        estimates[i].var_est *= 1-k;

        pl_point_(i) = estimates[i].state_est;

    }

    alive_cnt_ += 2;

    if (alive_cnt_ > alive_cnt_ceiling_) {

        alive_cnt_ = alive_cnt_ceiling_;

    }

}

void SingleLine::Predict(vector_t delta_position, quat_t delta_quat, plane_t projection_plane) {

    rotation_matrix_t R = quatToMat(delta_quat);

    // RCLCPP_INFO(logger_, "Point before: (%f, %f, %f)", pl_point_(0), pl_point_(1), pl_point_(2));

    pl_point_ = (R * pl_point_) + delta_position;

    pl_point_ = projectPointOnPlane(pl_point_, projection_plane);

    // RCLCPP_INFO(logger_, "Delta position: (%f, %f, %f)", delta_position(0), delta_position(1), delta_position(2));

    // RCLCPP_INFO(logger_, "Point after: (%f, %f, %f)", pl_point_(0), pl_point_(1), pl_point_(2));

    for (int i = 0; i < 3; i++) {

        estimates[i].state_est = pl_point_(i);
        estimates[i].var_est += q_;
    }

}