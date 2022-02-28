/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "powerline_class.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

Powerline::Powerline(float r, float q, rclcpp::Logger logger) : logger_(logger) {

    direction_ = 0;
    last_global_input_direction_ = 0;
    last_global_output_direction_ = 0;
    last_last_global_output_direction_ = 0;
    position_ = point_t(0,0,0);
    quat_ = quat_t(0, 0, 0, 0);
    last_position_ = point_t(0, 0, 0);
    last_quat_ = quat_t(0, 0, 0, 0);
    projection_plane_ = { .p=point_t(0,0,0), .normal=vector_t(0,0,0) };
    plane_orientation_ = orientation_t(0,0,0);

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

    float direction;

    direction_mutex_.lock(); {

        direction = direction_;

    } direction_mutex_.unlock();

    return direction;

}

plane_t Powerline::GetProjectionPlane() {

    plane_t plane;

    projection_plane_mutex_.lock(); {

        plane = projection_plane_;

    } projection_plane_mutex_.unlock();

    return plane;

}

orientation_t Powerline::GetPlaneOrientation() {

    orientation_t ori;

    projection_plane_mutex_.lock(); {

        ori = plane_orientation_;

    } projection_plane_mutex_.unlock();

    return ori;

}

quat_t Powerline::GetQuat() {

    quat_t quat;

    odometry_mutex_.lock(); {

        quat = quat_;

    } odometry_mutex_.unlock();

    return quat;

}

point_t Powerline::UpdateLine(point_t point) {

    point_t projected_point = projectPoint(point);

    int match_index = findMatchingLine(projected_point);

    if (match_index == -1) {

        lines_mutex_.lock(); {

            lines_.push_back(SingleLine(projected_point, r_, q_, logger_));

        } lines_mutex_.unlock();

    } else {

        lines_mutex_.lock(); {

            lines_[match_index].Update(projected_point);

        } lines_mutex_.unlock();

    }

    return projected_point;

}

void Powerline::UpdateDirection(float direction) {

    quat_t quat;

    odometry_mutex_.lock(); {

        quat = quat_;

    } odometry_mutex_.unlock();

    direction_mutex_.lock(); {

        float direction2 = (direction > 0) ? direction - M_PI : direction + M_PI;
        float new_direction = (abs(direction-direction_) < abs(direction2-direction_)) ? direction : direction2;

        //orientation_t direction_eul(0, 0, new_input_direction);
        //quat_t direction_quat = eulToQuat(direction_eul);

        //quat_t global_quat = quatMultiply(quat, direction_quat);

        //orientation_t global_eul = quatToEul(global_quat);

        //float direction_global = global_eul(2);

        orientation_t yaw_eul = quatToEul(quat);

        float new_global_input_direction = new_direction - yaw_eul(2);

        float new_global_output_direction = 0.1 * new_global_input_direction 
                + 0.45 * last_global_output_direction_ + 0.45 * last_last_global_output_direction_;

        last_global_input_direction_ = new_global_input_direction;
        last_last_global_output_direction_ = last_global_output_direction_;
        last_global_output_direction_ = new_global_output_direction;

        direction_ = new_global_output_direction + yaw_eul(2);

        //float curr_direction_global = yaw + direction_;

        //float new_input_direction = (abs(direction_global1-curr_direction_global) < abs(direction_global2-curr_direction_global)) 
        //        ? direction_global1 : direction_global2;

        //float new_direction_global = new_input_direction * 0.5 + last_input_direction_ * 0.5;
        //last_input_direction_ = new_input_direction;

        //direction_ = new_direction_global - yaw;
        //if (direction_ > M_PI/2) direction_ -= M_PI;
        //else if (direction_ < M_PI/2) direction_ += M_PI;


        //float direction2 = (direction > 0) ? direction - M_PI : direction + M_PI;
        //float new_input_direction = (abs(direction-direction_) < abs(direction2-direction_)) ? direction : direction2;
        //float temp_output_direction = direction_;
        //direction_ = 0.4 * new_input_direction + 0.3 * last_input_direction_ + 0.3 * last_output_direction_;
        //last_input_direction_ = new_input_direction;
        //last_output_direction_ = temp_output_direction;
        //float new_input_direction1 = direction;
        //float new_input_direction2 = (direction > 0) ? direction - M_PI : direction + M_PI;

        //float new_output_direction1 = 0.5 * new_input_direction1 + 0.5 * last_input_direction_;
        //float new_output_direction2 = 0.5 * new_input_direction2 + 0.5 * last_input_direction_;

        //if (abs(new_output_direction1-direction_) < abs(new_output_direction2-direction)) {

        //    direction_ = new_output_direction1;
        //    last_input_direction_ = new_input_direction1;

        //} else {

        //    direction_ = new_output_direction2;
        //    last_input_direction_ = new_input_direction2;

        //}

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

void Powerline::CleanupLines() {

    lines_mutex_.lock(); {

        std::vector<SingleLine> new_vec;

        for (int i = 0; i < lines_.size(); i++) {

            if (lines_[i].IsAlive(150)) {

                new_vec.push_back(lines_[i]);

            }
        }

        lines_ = new_vec;

    } lines_mutex_.unlock();

}

void Powerline::updateProjectionPlane() {

    float direction_tmp;
    quat_t quat_tmp;

    direction_mutex_.lock(); {

        direction_tmp = direction_;

    } direction_mutex_.unlock();

    odometry_mutex_.lock(); {

        quat_tmp = quat_t(quat_[0],quat_[1],quat_[2],quat_[3]);

    } odometry_mutex_.unlock();

    point_t plane_p(0, 0, 0);

    vector_t unit_x(1, 0, 0);

    orientation_t eul = quatToEul(quat_tmp);

    orientation_t rotation(0, -eul[1], direction_tmp);

    vector_t plane_normal = rotateVector(eulToR(rotation), unit_x);

    projection_plane_mutex_.lock(); {

        projection_plane_ = {

            .p = plane_p,
            .normal = plane_normal

        };

        plane_orientation_ = rotation;

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

        delta_position = position_ - last_position_;

        quat_t inv_quat = quatInv(last_quat_);
        delta_quat = quatMultiply(quat_, inv_quat);

    } odometry_mutex_.unlock();

    lines_mutex_.lock(); {

        for (int i = 0; i < lines_.size(); i++) {

            lines_[i].Predict(delta_position, delta_quat);

        }

    } lines_mutex_.unlock();

}