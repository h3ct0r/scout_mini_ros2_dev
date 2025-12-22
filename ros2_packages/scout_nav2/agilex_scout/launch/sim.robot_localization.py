# python imports
import os
from ament_index_python.packages import get_package_share_directory
from math import pi

# ros2 imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch configuration variables specific to simulation
 
    robot_localization_conf_file = os.path.join(
        get_package_share_directory("agilex_scout"),
        "config",
        "sim.ekf.yaml",
    )
 
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_conf_file, {'use_sim_time': True}]
    )

    return LaunchDescription(
        [
            robot_localization_node,
        ]
    )
