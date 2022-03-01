from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    pl_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("iii_drone"),
                "launch/pl_tracker.launch.py"
            ])
        ])
    )

    config = os.path.join(
        get_package_share_directory('iii_drone'),
        'config',
        'params.yaml'
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="usb_cam",
        parameters=[config]
    )

    return LaunchDescription([
        camera_node,
        pl_tracker_launch
    ])
