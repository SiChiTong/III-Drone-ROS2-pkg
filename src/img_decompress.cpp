// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>

// #include <image_transport/image_transport.hpp>
// #include <cv_bridge/cv_bridge.h>

// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/imgproc.hpp"
// #include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>



// using namespace std::chrono_literals;

// //creates a ImgDecompress class that subclasses the generic rclcpp::Node base class.
// class ImgDecompress : public rclcpp::Node
// {
// 	public:
// 		ImgDecompress(const std::string & node_name="img_decompress", const std::string & node_namespace="/img_decompress") 
// 						: Node(node_name, node_namespace) {

// 			img_decompress_publisher_ = this->create_publisher<sensor_msgs::msg::Image::SharedPtr>(
// 				"cable_camera/img_decompress", 10);


// 			image_transport::ImageTransport it(node);

// 			camera_subscription_ = it.subscribe("cable_camera/image_raw/compressed", 1, imageCallback);


// 		}

// 		~ImgDecompress() {
// 			RCLCPP_INFO(this->get_logger(),  "Shutting down img_decompress..");
// 		}


// 	private:
// 		rclcpp::Publisher<iii_interfaces::msg::PowerlineDirection>::SharedPtr img_decompress_publisher_;
// 		void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

// };


// // mmwave message callback function
// void ImgDecompress::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){

//   try {

//     cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//     cv::waitKey(10);

//   } catch (cv_bridge::Exception & e) {

//     RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

//   }

// }

			
// int main(int argc, char *argv[])
// {
// 	std::cout << "Starting img_decompress node..." << std::endl;
// 	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
// 	rclcpp::init(argc, argv);

// 	cv::namedWindow("view");
// 	cv::startWindowThread();

// 	rclcpp::spin(std::make_shared<ImgDecompress>());

// 	rclcpp::shutdown();
// 	return 0;
// }




  
// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  } catch (cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("/cable_camera/image_raw", 1, imageCallback);
  rclcpp::spin(node);
  cv::destroyWindow("view");
}