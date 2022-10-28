from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    tf_drone_to_cable_drum = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.13", "-0.01", "-0.09", "3.1415", "0", "0", "drone", "cable_drum"]
    )

    tf_drone_to_cable_gripper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        #arguments=["0.0", "0.0", "0.13", "0", "0.0", "0.0", "drone", "cable_gripper"]
        arguments=["0.0", "0.0", "0.09", "1.57079632679", "0.0", "0.0", "drone", "cable_gripper"]
    )

    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.035", "-0.23", "0.28", "3.1415", "-1.57079632679", "0", "drone", "iwr6843_frame"] # Simulation
    )

    tf_drone_to_depth_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.035", "-0.23", "0.28", "-1.57079632679", "0", "0.0", "drone", "depth_cam"] # Simulation
    )

    world_to_drone = Node(
        package="iii_drone",
        executable="drone_frame_broadcaster"
    )


    return LaunchDescription([
        tf_drone_to_cable_drum,
        tf_drone_to_cable_gripper,
        tf_drone_to_iwr,
        tf_drone_to_depth_cam,
        world_to_drone
    ])
