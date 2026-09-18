import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    launch_apps = []

    launch_apps.append(Node(package='fcu_ros_connector', executable='set_origin', name='origin_setter'))

    return LaunchDescription(launch_apps)