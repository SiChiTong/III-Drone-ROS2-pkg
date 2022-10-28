/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "pl_dir_computer_node.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineDirectionComputerNode::PowerlineDirectionComputerNode(const std::string & node_name, const std::string & node_namespace) : 
        rclcpp::Node(node_name, node_namespace) {

    r_ = 0.999;
    q_ = 1-r_;

    for (int i = 0; i < 3; i++) { pl_angle_est[i].state_est = 0; pl_angle_est[i].var_est = 1; }

    pl_direction_(0) = 1;
    pl_direction_(1) = 0;
    pl_direction_(2) = 0;
    pl_direction_(3) = 0;

    drone_quat_(0) = 1;
    drone_quat_(1) = 0;
    drone_quat_(2) = 0;
    drone_quat_(3) = 0;

    last_drone_quat_(0) = 1;
    last_drone_quat_(1) = 0;
    last_drone_quat_(2) = 0;
    last_drone_quat_(3) = 0;

    // pl_direction_sub_ = this->create_subscription<iii_interfaces::msg::PowerlineDirection>(
    //     "/hough_transformer/cable_yaw_angle", 10, std::bind(&PowerlineDirectionComputerNode::plDirectionCallback, this, std::placeholders::_1));
    pl_direction_sub_ = this->create_subscription<iii_interfaces::msg::PowerlineDirection>(
        "/hough_cable_yaw_angle", 10, std::bind(&PowerlineDirectionComputerNode::plDirectionCallback, this, std::placeholders::_1));

    pl_direction_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("powerline_direction", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	rclcpp::Rate rate(1000ms);
	rate.sleep();

    // Call on_timer function every second
    drone_tf_timer_ = this->create_wall_timer(
      25ms, std::bind(&PowerlineDirectionComputerNode::odometryCallback, this));

    RCLCPP_DEBUG(this->get_logger(), "Initialized PowerlineDirectionComputerNode");

}

void PowerlineDirectionComputerNode::odometryCallback() {

    RCLCPP_DEBUG(this->get_logger(), "Fetching odometry transform");

    geometry_msgs::msg::TransformStamped tf;

    try {

        tf = tf_buffer_->lookupTransform("drone", "world", tf2::TimePointZero);

    } catch(tf2::TransformException & ex) {

        RCLCPP_FATAL(this->get_logger(), "Could not get odometry transform, frame world to drone");
        return;

    }

    point_t position(
        tf.transform.translation.x,
        tf.transform.translation.y, 
        tf.transform.translation.z
    );

    quat_t quat(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z
    );

    last_drone_quat_ = drone_quat_;
    drone_quat_ = quat;

    if (!received_first_quat) {

        received_first_quat = true;
        return;

    }

    if (!received_second_quat) {

        received_second_quat = true;
        return;

    }

    if (!received_angle) {

        return;

    }

    // RCLCPP_INFO(this->get_logger(), "Received");


    predict();

    publishPowerlineDirection();

}

void PowerlineDirectionComputerNode::plDirectionCallback(const iii_interfaces::msg::PowerlineDirection::SharedPtr msg) {

    //RCLCPP_INFO(this->get_logger(), "Received powerline direction message");

    if (!received_first_quat) {
        return;
    }

    float pl_angle = msg->angle;

    update(pl_angle);
}

void PowerlineDirectionComputerNode::predict() {

    quat_t inv_drone_quat = quatInv(drone_quat_);
    quat_t inv_last_drone_quat = quatInv(last_drone_quat_);

    // quat_t delta_drone_quat = quatMultiply(drone_quat_, last_drone_quat_);
    // quat_t delta_drone_quat = quatMultiply(inv_drone_quat, last_drone_quat_);           
    quat_t delta_drone_quat = quatMultiply(drone_quat_, inv_last_drone_quat); /// Works!!!!! #impericalmethod
    // quat_t delta_drone_quat = quatMultiply(inv_drone_quat, inv_last_drone_quat);
    // quat_t delta_drone_quat = quatMultiply(last_drone_quat_, drone_quat_);
    // quat_t delta_drone_quat = quatMultiply(inv_last_drone_quat, drone_quat_);
    // quat_t delta_drone_quat = quatMultiply(last_drone_quat_, inv_drone_quat);
    // quat_t delta_drone_quat = quatMultiply(inv_last_drone_quat, inv_drone_quat);

    quat_t inv_delta_drone_quat = quatInv(delta_drone_quat);

    direction_mutex_.lock(); {

        kf_mutex_.lock(); {

            pl_direction_ = quatMultiply(delta_drone_quat, pl_direction_);   //// WORKS!!! #impericalmethodAKAnoideawhyitworks
            // pl_direction_ = quatMultiply(inv_delta_drone_quat, pl_direction_);
            // pl_direction_ = quatMultiply(pl_direction_, delta_drone_quat);
            // pl_direction_ = quatMultiply(pl_direction_, inv_delta_drone_quat);          

            orientation_t eul = quatToEul(pl_direction_);

            for (int i = 0; i < 3; i++) { pl_angle_est[i].var_est += q_; pl_angle_est[i].state_est = backmapAngle(eul(i)); }

        } kf_mutex_.unlock();

    } direction_mutex_.unlock();

}

void PowerlineDirectionComputerNode::update(float pl_angle) {

    pl_angle = - pl_angle;
    // RCLCPP_INFO(this->get_logger(), "pl_angle = %f", pl_angle);

    quat_t W_Q_D = drone_quat_;
    // RCLCPP_INFO(this->get_logger(), "W_Q_D = [%f, %f, %f, %f]", W_Q_D(0), W_Q_D(1), W_Q_D(2), W_Q_D(3));

    direction_mutex_.lock(); {

        kf_mutex_.lock(); {

            rotation_matrix_t W_R_D = quatToMat(W_Q_D);
            rotation_matrix_t D_R_W = W_R_D.transpose();

            vector_t unit_x(1,0,0);
            orientation_t pl_eul(0,0,pl_angle);

            vector_t D_v = eulToR(pl_eul) * unit_x;

            // RCLCPP_INFO(this->get_logger(), "D_v = [%f, %f, %f]", D_v(0), D_v(1), D_v(2));

            // vector_t W_v(
            //     -(D_v(1)*D_R_W(0,1) - D_v(0)*D_R_W(1,1))/(D_R_W(0,0)*D_R_W(1,1) - D_R_W(0,1)*D_R_W(1,0)),
            //     (D_v(1)*D_R_W(0,0) - D_v(0)*D_R_W(1,0))/(D_R_W(0,0)*D_R_W(1,1) - D_R_W(0,1)*D_R_W(1,0)),
            //     0
            // );

            vector_t W_v(
                -(D_v(1)*W_R_D(0,1) - D_v(0)*W_R_D(1,1))/(W_R_D(0,0)*W_R_D(1,1) - W_R_D(0,1)*W_R_D(1,0)),
                (D_v(1)*W_R_D(0,0) - D_v(0)*W_R_D(1,0))/(W_R_D(0,0)*W_R_D(1,1) - W_R_D(0,1)*W_R_D(1,0)),
                0
            );

            // RCLCPP_INFO(this->get_logger(), "W_v = [%f, %f, %f]", W_v(0), W_v(1), W_v(2));

            pl_angle = atan2(W_v[1], W_v[0]);
            // RCLCPP_INFO(this->get_logger(), "pl_angle = %f", pl_angle);
            // RCLCPP_INFO(this->get_logger(), "W_pl_yaw = %f", W_pl_yaw_);
            pl_angle = mapAngle(W_pl_yaw_, pl_angle);
            // RCLCPP_INFO(this->get_logger(), "mapped pl_angle = %f", pl_angle);
            W_pl_yaw_ = pl_angle;

            orientation_t W_pl_eul(0,0,pl_angle);
            W_v = eulToR(W_pl_eul) * unit_x;
            // W_v(1) = -W_v(1);
            // RCLCPP_INFO(this->get_logger(), "Mapped W_v = [%f, %f, %f]", W_v(0), W_v(1), W_v(2));

            W_v /= W_v.norm();

            // RCLCPP_INFO(this->get_logger(), "W_v = [%f, %f, %f]", W_v(0), W_v(1), W_v(2));

            orientation_t pi_2_yaw(0,0,M_PI_2);

            // RCLCPP_INFO(this->get_logger(), "pi_2_yaw = [%f, %f, %f]", pi_2_yaw(0), pi_2_yaw(1), pi_2_yaw(2));

            vector_t W_P_x = W_v;
            // RCLCPP_INFO(this->get_logger(), "W_P_x = [%f, %f, %f]", W_P_x(0), W_P_x(1), W_P_x(2));
            vector_t W_P_y = eulToR(pi_2_yaw)*W_P_x;
            // RCLCPP_INFO(this->get_logger(), "W_P_y = [%f, %f, %f]", W_P_y(0), W_P_y(1), W_P_y(2));
            vector_t W_P_z(0,0,1);
            // RCLCPP_INFO(this->get_logger(), "W_P_z = [%f, %f, %f]", W_P_z(0), W_P_z(1), W_P_z(2));

            rotation_matrix_t W_R_P;

            for (int i = 0; i < 3; i++) {

                W_R_P(i,0) = W_P_x(i);
                W_R_P(i,1) = W_P_y(i);
                W_R_P(i,2) = W_P_z(i);

            }

            // RCLCPP_INFO(this->get_logger(), "W_R_P = [%f, %f, %f\n%f, %f, %f\n%f, %f, %f]", W_R_P(0,0), W_R_P(0,1), W_R_P(0,2), W_R_P(1,0), W_R_P(1,1), W_R_P(1,2), W_R_P(2,0), W_R_P(2,1), W_R_P(2,2));

            quat_t W_Q_P = matToQuat(W_R_P);

            rotation_matrix_t D_R_P = W_R_D * W_R_P;

            quat_t D_Q_P = matToQuat(D_R_P);

            orientation_t D_eul_P = quatToEul(D_Q_P);
            // RCLCPP_INFO(this->get_logger(), "D_eul_P = [%f, %f, %f]", D_eul_P(0), D_eul_P(1), D_eul_P(2));

            for (int i = 0; i < 3; i++) {

                float angle = false ? mapAngle2(pl_angle_est[i].state_est, D_eul_P(i)) : D_eul_P(i);
                // RCLCPP_INFO(this->get_logger(), "angle = %f", angle);

                if (!received_angle) {
                // if(true) {

                    pl_angle_est[i].state_est = backmapAngle(angle);
                    // RCLCPP_INFO(this->get_logger(), "state_est[%d] = %f", i, pl_angle_est[i].state_est);

                    if (i==2) {

                        received_angle = true;

                    }

                } else {

                    float y_bar = angle - pl_angle_est[i].state_est;
                    // RCLCPP_INFO(this->get_logger(), "y_bar[%d] = %f", i, y_bar);
                    float s = pl_angle_est[i].var_est + r_;
                    // RCLCPP_INFO(this->get_logger(), "s[%d] = %f", i, s);

                    float k = pl_angle_est[i].var_est / s;
                    // RCLCPP_INFO(this->get_logger(), "k[%d] = %f", i, k);

                    pl_angle_est[i].state_est += k*y_bar;
                    // RCLCPP_INFO(this->get_logger(), "state_est[%d] = %f", i, pl_angle_est[i].state_est);
                    pl_angle_est[i].var_est *= 1-k;
                    // RCLCPP_INFO(this->get_logger(), "var_est[%d] = %f", i, pl_angle_est[i].var_est);

                    pl_angle_est[i].state_est = backmapAngle(pl_angle_est[i].state_est);
                    // RCLCPP_INFO(this->get_logger(), "state_est[%d] = %f", i, pl_angle_est[i].state_est);

                }

                D_eul_P(i) = pl_angle_est[i].state_est;
            }

            // RCLCPP_INFO(this->get_logger(), "D_eul_P = [%f, %f, %f]", D_eul_P(0), D_eul_P(1), D_eul_P(2));
            pl_direction_ = eulToQuat(D_eul_P);

            // RCLCPP_INFO(this->get_logger(), "pl_direction = [%f, %f, %f, %f]", pl_direction_(0), pl_direction_(1), pl_direction_(2), pl_direction_(3));
            // RCLCPP_INFO(this->get_logger(), "\n\n\n");


        } kf_mutex_.unlock();

    } direction_mutex_.unlock();

}

void PowerlineDirectionComputerNode::publishPowerlineDirection() {

    RCLCPP_DEBUG(this->get_logger(), "Publishing powerline direction");

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "drone"; //"drone";
    msg.header.stamp = this->get_clock()->now();

    direction_mutex_.lock(); {

        msg.pose.orientation.w = pl_direction_(0);
        msg.pose.orientation.x = pl_direction_(1);
        msg.pose.orientation.y = pl_direction_(2);
        msg.pose.orientation.z = pl_direction_(3);

        msg.pose.position.x = 0;
        msg.pose.position.y = 0;
        msg.pose.position.z = 0;

    } direction_mutex_.unlock();

    pl_direction_pub_->publish(msg);

}

float PowerlineDirectionComputerNode::backmapAngle(float angle) {

    if (angle > M_PI) {
        return angle-2*M_PI;
    } else if (angle < -M_PI) {
        return angle+2*M_PI;
    } else {
        return angle;
    }

}

float PowerlineDirectionComputerNode::mapAngle(float curr_angle, float new_angle) {

    //file << "Mapping angle: " << std::to_string(new_angle) << std::endl;

    float angle_candidates[5];
    angle_candidates[0] = new_angle;

    if (new_angle > 0) {

        angle_candidates[1] = new_angle - M_PI;
        angle_candidates[2] = new_angle - 2*M_PI;
        angle_candidates[3] = new_angle + M_PI;
        angle_candidates[4] = new_angle + 2*M_PI;

    } else {

        angle_candidates[1] = new_angle + M_PI;
        angle_candidates[2] = new_angle + 2*M_PI;
        angle_candidates[3] = new_angle - M_PI;
        angle_candidates[4] = new_angle - 2*M_PI;

    }

    //file << "Angle candidates: " << std::to_string(angle_candidates[0]) << " " << std::to_string(angle_candidates[1]) << " " << std::to_string(angle_candidates[2]) << " " << std::to_string(angle_candidates[3]) << std::endl;

    float best_angle = angle_candidates[0];
    float best_angle_diff = abs(angle_candidates[0]-curr_angle);

    for (int i = 0; i < 5; i++) {

        float diff = abs(angle_candidates[i]-curr_angle);
        if (diff < best_angle_diff) {
            best_angle_diff = diff;
            best_angle = angle_candidates[i];
        }

    }

    //file << "Best candidate: " << std::to_string(best_angle) << std::endl;

    return best_angle;

}

float PowerlineDirectionComputerNode::mapAngle2(float curr_angle, float new_angle) {

    //file << "Mapping angle: " << std::to_string(new_angle) << std::endl;

    float angle_candidates[3];
    angle_candidates[0] = new_angle;
    angle_candidates[1] = new_angle + M_PI;
    angle_candidates[2] = new_angle - M_PI;

    //file << "Angle candidates: " << std::to_string(angle_candidates[0]) << " " << std::to_string(angle_candidates[1]) << " " << std::to_string(angle_candidates[2]) << " " << std::to_string(angle_candidates[3]) << std::endl;

    float best_angle = angle_candidates[0];
    float best_angle_diff = abs(angle_candidates[0]-curr_angle);

    for (int i = 0; i < 3; i++) {

        float diff = abs(angle_candidates[i]-curr_angle);
        if (diff < best_angle_diff) {
            best_angle_diff = diff;
            best_angle = angle_candidates[i];
        }

    }

    RCLCPP_INFO(this->get_logger(), "curr_angle = %f", curr_angle);
    RCLCPP_INFO(this->get_logger(), "new_angle = %f", new_angle);
    RCLCPP_INFO(this->get_logger(), "best_angle = %f", best_angle);

    //file << "Best candidate: " << std::to_string(best_angle) << std::endl;

    return best_angle;

}

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<PowerlineDirectionComputerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}
