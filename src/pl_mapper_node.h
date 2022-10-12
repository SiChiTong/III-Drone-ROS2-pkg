#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "iii_interfaces/msg/powerline_direction.hpp"
#include "iii_interfaces/msg/powerline.hpp"

#include "powerline_class.h"

using namespace std::chrono_literals;

/*****************************************************************************/
// Defines
/*****************************************************************************/


/*****************************************************************************/
// Class
/*****************************************************************************/

class PowerlineMapperNode : public rclcpp::Node {
public:
explicit
    PowerlineMapperNode(const std::string & node_name="pl_mapper", const std::string & node_namespace="/pl_mapper");

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mmwave_sub_;

    rclcpp::Publisher<iii_interfaces::msg::Powerline>::SharedPtr powerline_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_est_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr projected_points_pub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr drone_tf_timer_{nullptr};

    Powerline powerline_;

    rotation_matrix_t R_drone_to_mmw;
    vector_t v_drone_to_mmw;
    quat_t pl_direction_; ////////

    void odometryCallback();
    void mmWaveCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void plDirectionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishPowerline();
    //void publishProjectionPlane();
    void publishPoints(std::vector<point_t> points, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);

};

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[]);
