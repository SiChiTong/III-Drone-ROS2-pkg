#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "iii_interfaces/msg/powerline_direction.hpp"

/*****************************************************************************/
// Defines
/*****************************************************************************/

//#define	IMAGE_X		480
//#define IMAGE_Y		640
//#define SIZE 		IMAGE_X*IMAGE_Y
//
///*****************************************************************************/
//// Class
///*****************************************************************************/
//
//class InvertImageNode : public rclcpp::Node
//{
//public:
//    explicit
//    InvertImageNode(const std::string & node_name="image_inverter", const std::string & node_namespace="/");
//    ~InvertImageNode();
//
//private:
//    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
//    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr inv_img_publisher_;
//    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr org_img_publisher_;
//
//    XInvert_image x_inv_img_;
//
//    void imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg);
//    void publishInvertedImage(const sensor_msgs::msg::Image inv_img_msg, const sensor_msgs::msg::Image org_img_msg);
//
//    int invertImage(const cv::Mat img_grey, cv::Mat *ptr_inv_img_grey);
//    int callIP(const uint8_t *ptr_img_data_in, const uint8_t *ptr_img_data_out);
//};
//
///*****************************************************************************/
//// Main
///*****************************************************************************/
//
int main(int argc, char *argv[]);