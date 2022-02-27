/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "single_line_class.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

SingleLine::SingleLine(point_t initial_point, float r, float q) {

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

}

SingleLine SingleLine::GetCopy() {

    SingleLine sl(pl_point_, r_, q_);

    return sl;

}

point_t SingleLine::GetPoint() {

    return pl_point_;

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

}

void SingleLine::Predict(vector_t delta_position, quat_t delta_quat) {

    rotation_matrix_t R = quatToMat(delta_quat);

    pl_point_ = (point_t)(R * pl_point_) + (point_t)delta_position;

    for (int i = 0; i < 3; i++) {

        estimates[i].state_est = pl_point_(0);
        estimates[i].var_est += q_;
    }


}