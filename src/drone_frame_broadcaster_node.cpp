/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <memory>
#include <string>

#include "geometry.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

/*****************************************************************************/
// Class
/*****************************************************************************/

class DroneFrameBroadcasterNode : public rclcpp::Node {
public:
explicit
    DroneFrameBroadcasterNode(const std::string & node_name="drone_frame_broadcaster", const std::string & node_namespace="/drone_frame_broadcaster")
            : Node(node_name, node_namespace) {

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::ostringstream stream;

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/vehicle_odometry/out", 10,
            std::bind(&DroneFrameBroadcasterNode::odometryCallback, this, std::placeholders::_1));

        R_NED_to_body_frame = eulToR(orientation_t(M_PI, 0, 0));
    }

private:
    void odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg) {

        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = now;
        t.header.frame_id = "world";
        t.child_frame_id = "drone";

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

        //quat_t quat_offset(
        //    msg->q_offset[0],
        //    msg->q_offset[1],
        //    msg->q_offset[2],
        //    msg->q_offset[3]
        //);

        //rotation_matrix_t R_quat = quatToMat(quat);
        //R_quat = R_NED_to_body_frame * R_quat;
        //quat = matToQuat(R_quat);

        //rotation_matrix_t R_quat_offset = quatToMat(quat_offset);
        //R_quat_offset = R_NED_to_body_frame * R_quat_offset;
        //quat_offset = matToQuat(R_quat_offset);

        //quat = quat * quat_offset;

        t.transform.translation.x = position(0);
        t.transform.translation.y = position(1);
        t.transform.translation.z = position(2);

        //t.transform.rotation.w = quat_offset[0];
        //t.transform.rotation.x = quat_offset[1];
        //t.transform.rotation.y = quat_offset[2];
        //t.transform.rotation.z = quat_offset[3];

        t.transform.rotation.w = quat(0);
        t.transform.rotation.x = quat(1);
        t.transform.rotation.y = quat(2);
        t.transform.rotation.z = quat(3);

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rotation_matrix_t R_NED_to_body_frame;

};

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneFrameBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}