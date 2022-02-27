/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "pl_mapper_node.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineMapperNode::PowerlineMapperNode(const std::string & node_name, const std::string & node_namespace) : 
        rclcpp::Node(node_name, node_namespace),
        powerline_(0.5, 0.5) {

    pl_direction_sub_ = this->create_subscription<iii_interfaces::msg::PowerlineDirection>(
        "/pl_direction", 10, std::bind(&PowerlineMapperNode::plDirectionCallback, this, std::placeholders::_1));

    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/vehicle_odometry/out", 10, std::bind(&PowerlineMapperNode::odometryCallback, this, std::placeholders::_1));

    mmwave_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mmwave_points", 10, std::bind(&PowerlineMapperNode::mmWaveCallback, this, std::placeholders::_1));

    char pub_name[100];
    sprintf(pub_name, "%s/powerline_est", node_namespace);

    powerline_pub_ = this->create_publisher<iii_interfaces::msg::Powerline>(pub_name, 10);

    R_NED_to_body_frame = eulToR(orientation_t(M_PI, 0, 0));

}

void PowerlineMapperNode::odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {

    point_t position(
        msg->x,
        msg->y, 
        msg->z
    );

    position = R_NED_to_body_frame * position;

    quat_t quat(
        msg->q[0],
        msg->q[1],
        msg->q[2],
        msg->q[3]
    );

    rotation_matrix_t R_quat = quatToMat(quat);
    R_quat = R_NED_to_body_frame * R_quat;
    quat = matToQuat(R_quat);

    powerline_.UpdateOdometry(position, quat);

    publishPowerline();

}

void PowerlineMapperNode::mmWaveCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // read PointCloud2 msg data
    int pcl_size = msg->width;
    uint8_t *ptr = msg->data.data();
    const uint32_t POINT_STEP = 12;

    for (size_t i = 0; i < pcl_size; i++) {

        point_t point(
            *(reinterpret_cast<float*>(ptr + 0)),
            *(reinterpret_cast<float*>(ptr + 4)),
            *(reinterpret_cast<float*>(ptr + 8))
        );

        powerline_.UpdateLine(point);

        ptr += POINT_STEP;

    }   

}

void PowerlineMapperNode::plDirectionCallback(const iii_interfaces::msg::PowerlineDirection::SharedPtr msg) {

    float direction = msg->angle;

    powerline_.UpdateDirection(direction);

}

void PowerlineMapperNode::publishPowerline() {

    std::vector<SingleLine> lines = powerline_.GetLines();

    auto msg = iii_interfaces::msg::Powerline();
    msg.angle = powerline_.GetDirection();

    for (int i = 0; i < lines.size(); i++) {
        auto point_msg = geometry_msgs::msg::Point32();

        point_t point = lines[i].GetPoint();

        point_msg.x = point(0);
        point_msg.y = point(1);
        point_msg.z = point(2);

        msg.positions.push_back(point_msg);

    }

    powerline_pub_->publish(msg);

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    auto node = std::make_shared<PowerlineMapperNode>();

    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}