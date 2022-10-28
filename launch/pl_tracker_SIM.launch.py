from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    sensor_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("iii_drone"),
                "launch/simulate_pointcloud_control_launch.py"
            ])
        ])
    )

    #tf_drone_to_iwr = Node(
    #    package="tf2_ros",
    #    executable="static_transform_publisher",
    #    arguments=["0", "0", "0.05", "3.1415", "-1.57079632679", "0", "drone", "iwr6843_frame"] # Simulation

    #)

    #world_to_drone = Node(
    #    package="iii_drone",
    #    executable="drone_frame_broadcaster"
    #)

    hough = Node(
        package="iii_drone",
        executable="hough_pub",
        remappings=[('/image_raw','/cable_camera/image_raw')]
    )

    pl_dir_computer = Node(
        package="iii_drone",
        executable="pl_dir_computer"
    )

    pl_mapper = Node(
        package="iii_drone",
        executable="pl_mapper"
    )

    return LaunchDescription([
        #sensor_simulation_launch,
        #tf_drone_to_iwr,
        #world_to_drone,
        hough,
        pl_dir_computer,
        pl_mapper
    ])
