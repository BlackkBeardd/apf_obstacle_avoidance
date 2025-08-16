from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
import os


def declare_args(context, *args, **kwargs):
    robot_type_arg = DeclareLaunchArgument("robot_type", default_value="")
    return [robot_type_arg]


def launch_setup(context, *args, **kwargs):
    robot_type_value = context.perform_substitution(LaunchConfiguration("robot_type"))

    if robot_type_value == "base":
        xacro_path = "diff_drive.xacro"
    elif robot_type_value == "with_lidar":
        xacro_path = "diff_drive_with_lidar.xacro"
    elif robot_type_value == "with_control":
        xacro_path = "diff_drive_with_lidar_control.xacro"
    else:
        raise ValueError("Unknown robot type: " + robot_type_value)

    robot_description = PathJoinSubstitution(
        [get_package_share_directory("diff_drive_description"), xacro_path]
    )

    diff_drive_description_package = launch_ros.substitutions.FindPackageShare(
        package="diff_drive_description"
    ).find("diff_drive_description")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["xacro ", robot_description])}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("diff_drive_description"),
                "rviz",
                "diff_drive.rviz",
            ),
        ],
    )

    return [robot_state_publisher_node, joint_state_publisher_gui, rviz2_node]


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=declare_args), OpaqueFunction(function=launch_setup)]
    )
