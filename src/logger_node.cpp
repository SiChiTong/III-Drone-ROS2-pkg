#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <sensor_msgs/image_encodings.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "iii_interfaces/msg/powerline_direction.hpp"

#include "geometry.h"

#include <cstdlib>
#include <stdlib.h> 
#include <iostream>   
#include <vector>
#include <string>
#include <chrono>
#include <fstream>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std::chrono_literals;

class IIILoggerNode : public rclcpp::Node
{
	public:
		IIILoggerNode(std::string logfiles_dir) : Node("iii_logger") {

			logfiles_dir_ = logfiles_dir;

			std::stringstream img_ss;
			img_ss << logfiles_dir_ << "/images";
			images_dir_ = img_ss.str();

			std::stringstream odom_ss;
			odom_ss << logfiles_dir_ << "/odometry.txt";
			odom_logfile_ = odom_ss.str();
			odom_ofs_ = new std::ofstream(odom_logfile_, std::ofstream::out);
			*odom_ofs_ << "t,x,y,z,q0,q1,q2,q3" << std::endl;

			std::stringstream cable_ss;
			cable_ss << logfiles_dir_ << "/cable_yaw.txt";
			cable_yaw_logsfile_ = cable_ss.str();
			cable_yaw_ofs_ = new std::ofstream(cable_yaw_logsfile_, std::ofstream::out);
			*cable_yaw_ofs_ << "t,theta" << std::endl;

			std::stringstream points_est_ss;
			points_est_ss << logfiles_dir_ << "/points_est.txt";
			points_est_logsfile_ = points_est_ss.str();
			points_est_ofs_ = new std::ofstream(points_est_logsfile_, std::ofstream::out);
			*points_est_ofs_ << "t,cnt,x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,x5,y5,z5,x6,y6,z6,x7,y7,z7" << std::endl;

			std::stringstream transformed_points_ss;
			transformed_points_ss << logfiles_dir_ << "/transformed_points.txt";
			transformed_points_logsfile_ = transformed_points_ss.str();
			transformed_points_ofs_ = new std::ofstream(transformed_points_logsfile_, std::ofstream::out);
			*transformed_points_ofs_ << "t,cnt,x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,x5,y5,z5,x6,y6,z6,x7,y7,z7" << std::endl;

			std::stringstream projected_points_ss;
			projected_points_ss << logfiles_dir_ << "/projected_points.txt";
			projected_points_logsfile_ = projected_points_ss.str();
			projected_points_ofs_ = new std::ofstream(projected_points_logsfile_, std::ofstream::out);
			*projected_points_ofs_ << "t,cnt,x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,x5,y5,z5,x6,y6,z6,x7,y7,z7" << std::endl;

			std::stringstream pl_direction_ss;
			pl_direction_ss << logfiles_dir_ << "/pl_direction.txt";
			pl_direction_logfile_ = pl_direction_ss.str();
			pl_direction_ofs_ = new std::ofstream(pl_direction_logfile_, std::ofstream::out);
			*pl_direction_ofs_ << "t,x,y,z" << std::endl;

			std::stringstream projection_plane_ss;
			projection_plane_ss << logfiles_dir_ << "/projection_plane.txt";
			projection_plane_logfile_ = projection_plane_ss.str();
			projection_plane_ofs_ = new std::ofstream(projection_plane_logfile_, std::ofstream::out);
			*projection_plane_ofs_ << "t,x,y,z" << std::endl;

			camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
				"/cable_camera/image_raw",	10,
				std::bind(&IIILoggerNode::onImageMsg, this, std::placeholders::_1));

			odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>( 
				"/fmu/vehicle_odometry/out",	10,
				std::bind(&IIILoggerNode::onOdometryMsg, this, std::placeholders::_1));

			cable_yaw_sub_ = this->create_subscription<iii_interfaces::msg::PowerlineDirection>( 
				"/hough_transformer/cable_yaw_angle",	10,
				std::bind(&IIILoggerNode::onCableYaw, this, std::placeholders::_1));

			points_est_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( 
				"/pl_mapper/points_est",	10,
				std::bind(&IIILoggerNode::onPointsEst, this, std::placeholders::_1));

			transformed_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( 
				"/pl_mapper/transformed_points",	10,
				std::bind(&IIILoggerNode::onTransformedPoints, this, std::placeholders::_1));

			projected_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( 
				"/pl_mapper/projected_points",	10,
				std::bind(&IIILoggerNode::onProjectedPoints, this, std::placeholders::_1));

			pl_direction_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( 
				"/pl_mapper/powerline_direction",	10,
				std::bind(&IIILoggerNode::onPLDirection, this, std::placeholders::_1));

			projection_plane_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( 
				"/pl_mapper/projection_plane",	10,
				std::bind(&IIILoggerNode::onProjectionPlane, this, std::placeholders::_1));

			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

			timer = this->create_wall_timer(
			100ms, std::bind(&IIILoggerNode::timerCallback, this));

			counter = 0;
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
		rclcpp::Subscription<iii_interfaces::msg::PowerlineDirection>::SharedPtr cable_yaw_sub_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_est_sub_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_points_sub_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr projected_points_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr projection_plane_sub_;

		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		bool img_rec_ = false;

		rclcpp::TimerBase::SharedPtr timer{nullptr};

		std::string logfiles_dir_;
		std::string images_dir_;
		std::string odom_logfile_;
		std::string cable_yaw_logsfile_;
		std::string points_est_logsfile_;
		std::string transformed_points_logsfile_;
		std::string projected_points_logsfile_;
		std::string pl_direction_logfile_;
		std::string projection_plane_logfile_;

		std::ofstream *odom_ofs_;
		std::ofstream *cable_yaw_ofs_;
		std::ofstream *points_est_ofs_;
		std::ofstream *transformed_points_ofs_;
		std::ofstream *projected_points_ofs_;
		std::ofstream *pl_direction_ofs_;
		std::ofstream *projection_plane_ofs_;

		sensor_msgs::msg::Image image_;
		px4_msgs::msg::VehicleOdometry odom_;
		iii_interfaces::msg::PowerlineDirection cable_yaw_;
		sensor_msgs::msg::PointCloud2 points_est_;
		sensor_msgs::msg::PointCloud2 transformed_points_;
		sensor_msgs::msg::PointCloud2 projected_points_;
		geometry_msgs::msg::PoseStamped pl_direction_;
		geometry_msgs::msg::PoseStamped projection_plane_;

		int counter;

		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			std::cout << "IMG CALLBACK" << std::endl;
			image_ = *msg;
			img_rec_ = true;
		}

		void onOdometryMsg(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
			std::cout << "ODOM CALLBACK" << std::endl;
			odom_ = *msg;
			counter++;
		}

		void  onCableYaw(const iii_interfaces::msg::PowerlineDirection::SharedPtr msg) {
			std::cout << "YAW CALLBACK" << std::endl;
			cable_yaw_ = *msg;
			counter++;
		}

		void onPointsEst(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			std::cout << "EST CALLBACK" << std::endl;
			points_est_ = *msg;
			counter++;
		}

		void onTransformedPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			std::cout << "TRANS CALLBACK" << std::endl;
			transformed_points_ = *msg;
			counter++;
		}

		void onProjectedPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			std::cout << "PROJ CALLBACK" << std::endl;
			projected_points_ = *msg;
			counter++;
		}

		void onPLDirection(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
			std::cout << "DIR CALLBACK" << std::endl;
			pl_direction_ = *msg;
			counter++;
		}

		void onProjectionPlane(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
			std::cout << "PLANE CALLBACK" << std::endl;
			projection_plane_ = *msg;
			counter++;
		}

		void streamPoints(std::ofstream *ofs, unsigned int time, sensor_msgs::msg::PointCloud2 msg) {
			// read PointCloud2 msg data
			int pcl_size = msg.width;
			uint8_t *ptr = msg.data.data();
			const uint32_t POINT_STEP = 12;

			std::vector<point_t> transformed_points;
			std::vector<point_t> projected_points;

			*ofs << time << "," << pcl_size;

			for (size_t i = 0; i < 8; i++) {

				point_t point;

				if (i < pcl_size) {
					point(0) = *(reinterpret_cast<float*>(ptr + 0));
					point(1) = *(reinterpret_cast<float*>(ptr + 4));
					point(2) = *(reinterpret_cast<float*>(ptr + 8));
				} else {
					point(0) = 0;
					point(1) = 0;
					point(2) = 0;
				}

				*ofs << "," << point(0) << "," << point(1) << "," << point(2);
				ptr += POINT_STEP;
			}   

			*ofs << std::endl;
		}

		void timerCallback() {

			unsigned int time = this->get_clock()->now().nanoseconds();

			//std::stringstream img_ss;
			//std::cout << cnt++ << std::endl;
			//img_ss << logfiles_dir_ << "/" << time << ".png";
			//std::cout << cnt++ << std::endl;
			//cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_);
			//std::cout << cnt++ << std::endl;
			//cv::Mat img = cv_ptr->image;
			//std::cout << cnt++ << std::endl;
			//cv::imwrite(img_ss.str(), img);
			//std::cout << cnt++ << std::endl;

			if (img_rec_){
				cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
				cv::Mat img = cv_ptr->image;
				std::string img_filename = logfiles_dir_ + "/" + std::to_string(time) + ".jpg";
				cv::imwrite(img_filename, img);
				std::cout << "Save image as " << logfiles_dir_ + img_filename << std::endl;
			}

			*odom_ofs_ << time << "," << odom_.position[0] << "," << odom_.position[1] << "," << odom_.position[2] << "," << odom_.q[0] << "," << odom_.q[1] << "," << odom_.q[2] << "," << odom_.q[3] << std::endl;

			*cable_yaw_ofs_ << time << "," << cable_yaw_.angle << std::endl;

			streamPoints(points_est_ofs_, time, points_est_);

			streamPoints(transformed_points_ofs_, time, transformed_points_);

			streamPoints(projected_points_ofs_, time, projected_points_);

			*pl_direction_ofs_ << time << "," << pl_direction_.pose.position.x << "," << pl_direction_.pose.position.y << "," << pl_direction_.pose.position.z << std::endl;

			*projection_plane_ofs_ << time << "," << projection_plane_.pose.position.x << "," << projection_plane_.pose.position.y << "," << projection_plane_.pose.position.z << std::endl;
			
		}
};

int main(int argc, char *argv[])
{

	if (argc != 2) {
		std::cout << "Please provide the logging directory as a single command line argument, full path" << std::endl;

		return 1;
	}

	std::cout << "Starting IIILoggerNode..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<IIILoggerNode>(argv[1]));

	rclcpp::shutdown();
	return 0;
}
