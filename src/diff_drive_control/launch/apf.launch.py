from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
import time


def generate_launch_description():

    conf_file_path = "apf.yaml"

    apf_conf1 = PathJoinSubstitution(
        [get_package_share_directory("diff_drive_control"), conf_file_path]
    )
    apf_conf2 = os.path.join(
        get_package_share_directory("diff_drive_control"), "config", conf_file_path
    )

    return LaunchDescription(
        [
            Node(
                package="diff_drive_control",
                executable="apf_controller",
                name="apf_controller",
                output="screen",
                parameters=[apf_conf1],
            )
        ]
    )
