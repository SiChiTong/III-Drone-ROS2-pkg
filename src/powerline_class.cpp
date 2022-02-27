/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "powerline_class.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

Powerline::Powerline(float r, float q) {

    direction_ = 0;
    position_ = point_t(0,0,0);
    quat_ = quat_t(0, 0, 0, 0);
    last_position_ = point_t(0, 0, 0);
    last_quat_ = quat_t(0, 0, 0, 0);
    projection_plane_ = { .p=point_t(0,0,0), .normal=vector_t(0,0,0) };

    r_ = r;
    q_ = q;

}

std::vector<SingleLine> Powerline::GetLines() {

    std::vector<SingleLine> ret_lines;

    lines_mutex_.lock(); {

        for (int i = 0; i < lines_.size(); i++) {

            ret_lines.push_back(lines_[i].GetCopy());

        }

    } lines_mutex_.unlock();

    return ret_lines;

}

float Powerline::GetDirection() {

    return direction_;

}

void Powerline::UpdateLine(point_t point) {

    point_t projectedPoint = projectPoint(point);

    int match_index = findMatchingLine(projectedPoint);

    if (match_index == -1) {

        lines_mutex_.lock(); {

            lines_.push_back(SingleLine(projectedPoint, r_, q_));

        } lines_mutex_.unlock();

    } else {

        lines_mutex_.lock(); {

            lines_[match_index].Update(projectedPoint);

        } lines_mutex_.unlock();

    }

}

void Powerline::UpdateDirection(float direction) {

    direction_mutex_.lock(); {

        direction_ = direction;

    } direction_mutex_.unlock();

    updateProjectionPlane();

}

void Powerline::UpdateOdometry(point_t position, quat_t quat) {

    odometry_mutex_.lock(); {

        last_quat_ = quat_;
        last_position_ = position_;

        quat_ = quat;
        position_ = position;

    } odometry_mutex_.unlock();

    updateProjectionPlane();

    predictLines();

}

void Powerline::updateProjectionPlane() {

    float direction_tmp;
    quat_t quat_tmp;
    point_t position_tmp;

    direction_mutex_.lock(); {

        direction_tmp = direction_;

    } direction_mutex_.unlock();

    odometry_mutex_.lock(); {

        quat_tmp = quat_t(quat_[0],quat_[1],quat_[2],quat_[3]);

        position_tmp(0) = position_(0);
        position_tmp(1) = position_(1);
        position_tmp(2) = position_(2);

    } odometry_mutex_.unlock();

    point_t plane_p = position_tmp;

    vector_t unit_x(1, 0, 0);

    orientation_t eul = quatToEul(quat_tmp);

    orientation_t rotation(0, -eul[1], direction_tmp);

    vector_t plane_normal = rotateVector(eulToR(rotation), unit_x);

    projection_plane_mutex_.lock(); {

        projection_plane_ = {

            .p = plane_p,
            .normal = plane_normal

        };

    } projection_plane_mutex_.unlock();

}

point_t Powerline::projectPoint(point_t point) {

    point_t projected_point;

    projection_plane_mutex_.lock(); {

        projected_point = projectPointOnPlane(point, projection_plane_);

    } projection_plane_mutex_.unlock();

    return projected_point;

}

int Powerline::findMatchingLine(point_t point) { 

    int best_idx = -1;
    float best_dist = std::numeric_limits<float>::infinity();

    lines_mutex_.lock(); {
        
        for (int i = 0; i < lines_.size(); i++) {

            vector_t vec = (vector_t)(point - lines_[i].GetPoint());

            float dist = vec.dot(vec);

            if (dist < POINT_MATCH_DISTANCE_THRESHOLD && dist < best_dist) {

                best_dist = dist;
                best_idx = i;

            }
        }

    } lines_mutex_.unlock();

    return best_idx;

}

void Powerline::predictLines() {

    vector_t delta_position;
    quat_t delta_quat;

    odometry_mutex_.lock(); {

        delta_position = last_position_ - position_;
        delta_quat = quatMultiply(quat_, quatInv(last_quat_));

    } odometry_mutex_.unlock();

    lines_mutex_.lock(); {

        delta_position = (vector_t)(projectPointOnPlane((point_t)delta_position, projection_plane_));

    } lines_mutex_.unlock();

    lines_mutex_.lock(); {

        for (int i = 0; i < lines_.size(); i++) {

            lines_[i].Predict(delta_position, delta_quat);

        }

    } lines_mutex_.unlock();

}