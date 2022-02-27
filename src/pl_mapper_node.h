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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "iii_interfaces/msg/powerline_direction.hpp"
#include "iii_interfaces/msg/powerline.hpp"

#include "powerline_class.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

/*****************************************************************************/
// Class
/*****************************************************************************/

class PowerlineMapperNode : public rclcpp::Node
{
public:
    explicit
    PowerlineMapperNode(const std::string & node_name="pl_mapper", const std::string & node_namespace="");

private:
    rclcpp::Subscription<iii_interfaces::msg::PowerlineDirection>::SharedPtr pl_direction_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mmwave_sub_;
    rclcpp::Publisher<iii_interfaces::msg::Powerline>::SharedPtr powerline_pub_;

    Powerline powerline_;

    rotation_matrix_t R_NED_to_body_frame;

    void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void mmWaveCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void plDirectionCallback(const iii_interfaces::msg::PowerlineDirection::SharedPtr msg);
    void publishPowerline();

};

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[]);