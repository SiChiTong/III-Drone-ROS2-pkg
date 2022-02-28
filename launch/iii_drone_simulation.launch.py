from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    simulator_env_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iii_drone'),
                "launch/simulate_pointcloud_control_launch.py"
            ])
        ]),
        launch_arguments={}.items()
    )

    pl_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("iii_drone"),
                "launch/pl_tracker.launch.py"
            ])
        ])
    )

    return LaunchDescription([
        simulator_env_launch,
        pl_tracker_launch
    ])
