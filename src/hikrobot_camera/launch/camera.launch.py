import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("hikrobot_camera")
    default_params = os.path.join(
        package_share,
        "config",
        "camera.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to camera parameter YAML file",
            ),
            Node(
                package="hikrobot_camera",
                executable="hikrobot_camera_node",
                name="hikrobot_camera",
                output="screen",
                parameters=[params_file],
                emulate_tty=True,
            ),
        ]
    )
