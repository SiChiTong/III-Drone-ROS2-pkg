/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "pl_dir_computer_node.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineDirectionComputerNode::PowerlineDirectionComputerNode(const std::string & node_name, const std::string & node_namespace) : 
        rclcpp::Node(node_name, node_namespace), file("/home/ffn/test.txt", std::ofstream::out) {

    r_ = 0.8;
    q_ = 1-r_;

    pl_angle_est.state_est = 0;
    pl_angle_est.var_est = 1;

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

    pl_direction_sub_ = this->create_subscription<iii_interfaces::msg::PowerlineDirection>(
        "/hough_transformer/cable_yaw_angle", 10, std::bind(&PowerlineDirectionComputerNode::plDirectionCallback, this, std::placeholders::_1));

    pl_direction_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("powerline_direction", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

    predict();

    publishPowerlineDirection();

}

void PowerlineDirectionComputerNode::plDirectionCallback(const iii_interfaces::msg::PowerlineDirection::SharedPtr msg) {

    RCLCPP_DEBUG(this->get_logger(), "Received powerline direction message");

    float pl_angle = msg->angle;

    update(pl_angle);
}

void PowerlineDirectionComputerNode::predict() {

    file_mutex.lock();

    file << "Prediction step:" << std::endl;

    quat_t inv_last_drone_quat = quatInv(last_drone_quat_);
    quat_t inv_drone_quat = quatInv(drone_quat_);
    quat_t delta_drone_quat = quatMultiply(drone_quat_, inv_last_drone_quat);

    orientation_t delta_drone_eul = quatToEul(delta_drone_quat);

    file << "delta_drone_eul: [" << std::to_string(delta_drone_eul(0)) << ", " << std::to_string(delta_drone_eul(1)) << ", " << std::to_string(delta_drone_eul(2)) << "]" << std::endl;

    float delta_yaw = delta_drone_eul(2);
    float pl_dir_yaw;

    kf_mutex_.lock(); {

        file << "Previous direction est: " << std::to_string(pl_angle_est.state_est) << std::endl;
        pl_angle_est.state_est = pl_angle_est.state_est + delta_yaw;
        pl_angle_est.var_est += q_;

        file << "New direction est before backmapping: " << std::to_string(pl_angle_est.state_est) << std::endl;
        pl_angle_est.state_est = backmapAngle(pl_angle_est.state_est);
        file << "New direction est after backmapping: " << std::to_string(pl_angle_est.state_est) << std::endl << std::endl;

        pl_dir_yaw = pl_angle_est.state_est;

    } kf_mutex_.unlock();

    orientation_t pl_dir_eul(
        0,
        -inv_drone_quat(1),
        pl_dir_yaw
    );

    quat_t pl_dir = eulToQuat(pl_dir_eul);

    direction_mutex_.lock(); {

        pl_direction_ = pl_dir;

    } direction_mutex_.unlock();

    file << std::to_string(pl_dir_yaw) << std::endl;

    file_mutex.unlock();

}

void PowerlineDirectionComputerNode::update(float pl_angle) {

    file_mutex.lock();

    file << "Update step:" << std::endl;

    kf_mutex_.lock(); {

        file << "Received angle: " << std::to_string(pl_angle) << std::endl;
        file << "Current est angle: " << std::to_string(pl_angle_est.state_est) << std::endl;

        pl_angle = mapAngle(pl_angle_est.state_est, pl_angle);

        file << "Received angle after mapping: " << std::to_string(pl_angle) << std::endl;

        float y_bar = pl_angle - pl_angle_est.state_est;
        float s = pl_angle_est.var_est + r_;

        float k = pl_angle_est.var_est / s;

        pl_angle_est.state_est += k*y_bar;
        pl_angle_est.var_est *= 1-k;

        file << "New est angle: " << std::to_string(pl_angle_est.state_est) << std::endl;

        pl_angle_est.state_est = backmapAngle(pl_angle_est.state_est);

        file << "New est angle after backmapping: " << std::to_string(pl_angle_est.state_est) << std::endl << std::endl;

        file << std::to_string(pl_angle_est.state_est) << std::endl;

    } kf_mutex_.unlock();

    file_mutex.unlock();

}

void PowerlineDirectionComputerNode::publishPowerlineDirection() {

    RCLCPP_DEBUG(this->get_logger(), "Publishing powerline direction");

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "drone";
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

    file << "Mapping angle: " << std::to_string(new_angle) << std::endl;

    float angle_candidates[4];
    angle_candidates[0] = new_angle;

    if (new_angle > 0) {

        angle_candidates[1] = new_angle - M_PI;
        angle_candidates[2] = new_angle - 2*M_PI;
        angle_candidates[3] = new_angle + M_PI;

    } else {

        angle_candidates[1] = new_angle + M_PI;
        angle_candidates[2] = new_angle + 2*M_PI;
        angle_candidates[3] = new_angle - M_PI;

    }

    file << "Angle candidates: " << std::to_string(angle_candidates[0]) << " " << std::to_string(angle_candidates[1]) << " " << std::to_string(angle_candidates[2]) << " " << std::to_string(angle_candidates[3]) << std::endl;

    float best_angle = angle_candidates[0];
    float best_angle_diff = abs(angle_candidates[0]-curr_angle);

    for (int i = 0; i < 4; i++) {

        float diff = abs(angle_candidates[i]-curr_angle);
        if (diff < best_angle_diff) {
            best_angle_diff = diff;
            best_angle = angle_candidates[i];
        }

    }

    file << "Best candidate: " << std::to_string(best_angle) << std::endl;

    return best_angle;

}

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<PowerlineDirectionComputerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}