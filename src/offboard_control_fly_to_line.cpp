/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "iii_interfaces/msg/powerline_direction.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "iii_interfaces/msg/powerline.hpp"
#include <std_msgs/msg/int32.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stdint.h>
#include <math.h>  
#include <limits>
#include <mutex>
#include <chrono>
#include <iostream>

#include <eigen3/Eigen/Core>


typedef Eigen::Vector3f point_t;
typedef Eigen::Vector3f orientation_t;
typedef Eigen::Vector4f quat_t;
typedef Eigen::Vector3f vector_t;
typedef Eigen::Matrix3f rotation_matrix_t;
typedef Eigen::Matrix4f transform_t;


#define PI 3.14159265
#define NAN_ std::numeric_limits<double>::quiet_NaN()

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);

		this->declare_parameter<float>("yaw_frac", 0.1);
		this->declare_parameter<float>("pos_frac", 0.5);

		// VehicleStatus: https://github.com/PX4/px4_msgs/blob/master/msg/VehicleStatus.msg
		vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/vehicle_status/out",
            10,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
              arming_state_ = msg->arming_state;
              nav_state_ = msg->nav_state;
			});

		
		powerline_ID_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/typed_ID",
            10,
            [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
              pl_id_ = msg->data;
			});


		powerline_dir_sub_ = create_subscription<iii_interfaces::msg::PowerlineDirection>(
            "/hough_cable_yaw_angle",
            10,
            [this](iii_interfaces::msg::PowerlineDirection::ConstSharedPtr msg) {
			  pl_yaw_ = -msg->angle;
			});


		powerline_sub_ = create_subscription<iii_interfaces::msg::Powerline>(
            "/pl_mapper/powerline",
            10,
            [this](iii_interfaces::msg::Powerline::ConstSharedPtr msg) {
			  
			  // If user has typed an ID
			  if(pl_id_ != -1)
			  {
				  int pl_id_checked__temp = -1;
				  int idx = -1;

				  //Check if ID is valid
				  for (int i = 0; i < msg->count; i++)
				  {
					  if(msg->ids[i] == pl_id_){
						pl_id_checked__temp = pl_id_;
						idx = i;
					  }
				  }
				  
				  if (pl_id_checked__temp == -1)
				  {
					  pl_id_checked_ = -1;
				  } 
				  else // If valid, update powerline position
				  {
					powerline_location_mutex_.lock(); {
						pl_id_checked_ = pl_id_checked__temp;
						pl_x_ = float(msg->poses[idx].pose.position.x);
						pl_y_ = float(msg->poses[idx].pose.position.y);
						pl_z_ = float(msg->poses[idx].pose.position.z);
					} powerline_location_mutex_.unlock();
				  }
			  }
			});


		odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(  ////
			"/fmu/vehicle_odometry/out",	10,
            [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr _msg) {
				drone_location_mutex_.lock(); {
				drone_x_ = _msg->x;
				drone_y_ = _msg->y;
				drone_z_ = _msg->z;

				// float temp_yaw = 0.0;

				odo_quat_(0) = _msg->q[0];
				odo_quat_(1) = _msg->q[1];
				odo_quat_(2) = _msg->q[2];
				odo_quat_(3) = _msg->q[3];

				drone_roll_ = atan2(2.0 * (_msg->q[0] * _msg->q[1] + _msg->q[2] * _msg->q[3]) , - 1.0 + 2.0 * (_msg->q[1] * _msg->q[1] + _msg->q[2] * _msg->q[2]));
				drone_pitch_ = asin(2.0 * (_msg->q[0] * _msg->q[2] - _msg->q[3] * _msg->q[1]));
				drone_yaw_ = atan2(2.0 * (_msg->q[3] * _msg->q[0] + _msg->q[1] * _msg->q[2]) , - 1.0 + 2.0 * (_msg->q[0] * _msg->q[0] + _msg->q[1] * _msg->q[1]));
				
                // drone_yaw_ = temp_yaw;



				} drone_location_mutex_.unlock();
			});


		id_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("ID_point", 10);


		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});


		auto timer_callback = [this]() -> void {

			drone_location_mutex_.lock(); {
				powerline_location_mutex_.lock(); {
					world_to_points();
				} powerline_location_mutex_.unlock();
			} drone_location_mutex_.unlock();

			// If drone not armed (from external controller) and put in offboard mode, do nothing
			if (nav_state_ != 14 or pl_id_checked_ == -1) 
            {
                if (old_nav_state_ != nav_state_ && nav_state_ != 14)
                {				
                    RCLCPP_INFO(this->get_logger(), "nav_state: %d", nav_state_);
                    RCLCPP_INFO(this->get_logger(), "Waiting for offboard mode");
                }

				if (old_nav_state_ != nav_state_ && nav_state_ == 14)
                {				
                    RCLCPP_INFO(this->get_logger(), "nav_state: %d", nav_state_);
                    RCLCPP_INFO(this->get_logger(), "Offboard mode enabled");
                }

				if (old_pl_id_ != pl_id_ && pl_id_checked_ == -1)
                {
					RCLCPP_INFO(this->get_logger(), "Powerline ID: %d", pl_id_);
                    RCLCPP_INFO(this->get_logger(), "Waiting for valid powerline ID");
                }

				if (old_pl_id_ != pl_id_ && pl_id_checked_ != -1)
                {
                    RCLCPP_INFO(this->get_logger(), "Got valid powerline ID %d", pl_id_checked_);
                }


				publish_offboard_control_mode();
				publish_hold_setpoint();
                old_nav_state_ = nav_state_;
				old_pl_id_ = pl_id_;
				counter_ = 0;
				return;
			}

            if (!printed_offboard_)
            {
                RCLCPP_INFO(this->get_logger(), "\n \nEntering offboard control \n");
                printed_offboard_ = true;
				this->arm();
            }

			if(counter_ < 20){
				if(counter_ == 0){
					RCLCPP_INFO(this->get_logger(), "Waiting two seconds \n");
				}
				publish_offboard_control_mode();
				publish_hold_setpoint();
			}

			else if(counter_ < 100){
				if(drone_z_ > -0.5) // Probably on ground, takeoff to hover
				{
					if(counter_ == 21){
						RCLCPP_INFO(this->get_logger(), "Takeoff to hover \n");
					}
					publish_offboard_control_mode();
					publish_hover_setpoint();
				}
				else
				{	// probably in air, hold
					counter_ = 100;
					if(counter_ == 21){
						RCLCPP_INFO(this->get_logger(), "Holding position \n");
					}
					publish_offboard_control_mode();
					publish_hold_setpoint();
				}
			}

			else if(counter_ >= 100){
				if(counter_ == 101){
					RCLCPP_INFO(this->get_logger(), "Beginning alignment \n");
				}
				publish_offboard_control_mode();
				publish_test_setpoint();

			}

			counter_++;

		};


		timer_ = this->create_wall_timer(100ms, timer_callback);
		
	}


	~OffboardControl() {
		RCLCPP_INFO(this->get_logger(),  "Shutting down offboard control, landing..");
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND); 
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		
	}

	void arm() const;
	void disarm() const;
	


private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr id_point_pub_;

	rclcpp::Subscription<iii_interfaces::msg::PowerlineDirection>::SharedPtr powerline_dir_sub_;
	rclcpp::Subscription<iii_interfaces::msg::Powerline>::SharedPtr powerline_sub_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr powerline_ID_sub_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr vel_ctrl_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	int nav_state_, old_nav_state_ = 0;
	int arming_state_;
	int pl_id_ = -1; 
	int old_pl_id_ = 0;
	int pl_id_checked_ = -1;
	int counter_ = 0;
	float x_world_to_id_point_, y_world_to_id_point_, z_world_to_id_point_ = 0.0;
	Eigen::Vector4f odo_quat_;

    bool printed_offboard_ = false;

	std::mutex drone_location_mutex_;
	std::mutex powerline_location_mutex_;

	float pl_x_, pl_y_, pl_z_, pl_yaw_ = 0;
	float drone_x_, drone_y_, drone_z_, drone_yaw_, drone_pitch_, drone_roll_ = 0;

	float yaw_ = 0, yawspeed_ = 0;
	float vx_ = 0, vy_ = 0, vz_ = 0;
	float hover_height_ = 2;

	void publish_test_setpoint();
	void publish_offboard_control_mode() const;
	void publish_hover_setpoint() const;
	void publish_tracking_setpoint() const;
	void publish_hold_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
	void world_to_points();
};



orientation_t quatToEul(quat_t quat)
{
    orientation_t eul(
        atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]), 1-2*(quat[1]*quat[1] + quat[2]*quat[2])),
        asin(2*(quat[0]*quat[2] - quat[3]*quat[1])),
        atan2(2*(quat[0]*quat[3] + quat[1]*quat[2]), 1-2*(quat[2]*quat[2]+quat[3]*quat[3]))
    );

    return eul;
}


rotation_matrix_t eulToR(orientation_t eul) 
{
    float cos_yaw = cos(eul[2]);
    float cos_pitch = cos(eul[1]);
    float cos_roll = cos(eul[0]);
    float sin_yaw = sin(eul[2]);
    float sin_pitch = sin(eul[1]);
    float sin_roll = sin(eul[0]);

    rotation_matrix_t mat;

    mat(0,0) = cos_pitch*cos_yaw;
    mat(0,1) = sin_roll*sin_pitch*cos_yaw-cos_roll*sin_yaw;
    mat(0,2) = cos_roll*sin_pitch*cos_yaw+sin_roll*sin_yaw;
    mat(1,0) = cos_pitch*sin_yaw;
    mat(1,1) = sin_roll*sin_pitch*sin_yaw+cos_roll*cos_yaw;
    mat(1,2) = cos_roll*sin_pitch*sin_yaw-sin_roll*cos_yaw;
    mat(2,0) = -sin_pitch;
    mat(2,1) = sin_roll*cos_pitch;
    mat(2,2) = cos_roll*cos_pitch;

    return mat;
}


void OffboardControl::world_to_points()
{
	float x_w_t_d = drone_x_;
	float y_w_t_d = -drone_y_;
	float z_w_t_d = -drone_z_;

	// FIND TRANSFORM WORLD->ID_POINT

	vector_t pl_xyz;
	pl_xyz(0) = pl_x_;
	pl_xyz(1) = pl_y_;
	pl_xyz(2) = pl_z_;

	rotation_matrix_t rot_mat;
	vector_t rpy = quatToEul(odo_quat_);
	rpy(0) = rpy(0);
	rpy(1) = -rpy(1);
	rpy(2) = -rpy(2);
	rot_mat = eulToR(rpy);

	vector_t rot_pl_xyz = rot_mat * pl_xyz;

	x_world_to_id_point_ = rot_pl_xyz(0) + x_w_t_d; 
	y_world_to_id_point_ = rot_pl_xyz(1) + y_w_t_d;
	z_world_to_id_point_ = rot_pl_xyz(2) + z_w_t_d;

	geometry_msgs::msg::PointStamped msg;
	msg.header.frame_id = "world";
    msg.header.stamp = this->get_clock()->now();
	msg.point.x = x_world_to_id_point_;
	msg.point.y = y_world_to_id_point_;
	msg.point.z = z_world_to_id_point_;
	id_point_pub_->publish(msg);
}


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send\n");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send\n");
}


/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;	
	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Should go to align with cable of choice there
 */
void OffboardControl::publish_test_setpoint() {

	drone_location_mutex_.lock(); {

	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();

	float pos_frac;
	this->get_parameter("pos_frac", pos_frac);

	float x_frac = drone_x_ + (x_world_to_id_point_ - drone_x_)*pos_frac;
	float y_frac = drone_y_ + (y_world_to_id_point_ - drone_y_)*pos_frac;
	float z_frac = - (drone_z_ + ((z_world_to_id_point_-1.0) - drone_z_)*pos_frac);

	msg.x = x_frac; 		// in meters NED
	msg.y = y_frac;
	// msg.z = z_frac;
	// msg.x = x_world_to_id_point_; 		// in meters NED
	// msg.y = y_world_to_id_point_;
	msg.z = -(z_world_to_id_point_-1.0);

	float yaw_frac;
	this->get_parameter("yaw_frac", yaw_frac);
	
	msg.yaw = drone_yaw_-yaw_frac*pl_yaw_;

	// RCLCPP_INFO(this->get_logger(), "X:%f Y:%f Z:%f YAW:%f", msg.x, msg.y, msg.z, msg.yaw);

	trajectory_setpoint_publisher_->publish(msg);

	} drone_location_mutex_.unlock();
}


/**
 * @brief Publish a trajectory setpoint
 *        Drone should hover at hover_height_
 */
void OffboardControl::publish_hover_setpoint() const {

	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = drone_x_; 		// in meters NED
	msg.y = drone_y_;
	msg.z = -hover_height_;
	msg.yaw = drone_yaw_;

	trajectory_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Drone should hover in place
 */
void OffboardControl::publish_hold_setpoint() const {

	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = drone_x_; 		// in meters NED
	msg.y = drone_y_;
	msg.z = drone_z_;
	msg.yaw = drone_yaw_;

	trajectory_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Track with velocities and yawspeed.
 */
void OffboardControl::publish_tracking_setpoint() const {

	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = drone_x_; 		// in meters NED
	msg.y = drone_y_;
	msg.z = drone_z_;
	msg.yaw = drone_yaw_;
	msg.yawspeed = yawspeed_;	// rotational speed around z in radians/sec
	msg.vx = vx_;	// m/s NED
	msg.vy = vy_;
	msg.vz = vz_;
	trajectory_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
