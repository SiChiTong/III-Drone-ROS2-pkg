from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    cam = Node(
        package='iii_drone',
        executable='img_3d_to_2d_proj',
        name='img_3d_to_2d_proj'
    )

    mmwave = Node(
        package='iii_drone',
        executable='depth_cam_to_mmwave',
        name='depth_cam_to_mmwave'
    )

    return LaunchDescription([
        cam,
        mmwave
    ])
