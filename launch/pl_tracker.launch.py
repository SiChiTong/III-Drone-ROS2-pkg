from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.05", "0", "-1.57079632679", "0", "drone", "iwr6843_frame"]
    )

    world_to_drone = Node(
        package="iii_drone",
        executable="drone_frame_broadcaster"
    )

    hough = Node(
        package="iii_drone",
        executable="hough_pub"
    )

    pl_mapper = Node(
        package="iii_drone",
        executable="pl_mapper"
    )

    return LaunchDescription([
        tf_drone_to_iwr,
        world_to_drone,
        hough,
        pl_mapper
    ])
