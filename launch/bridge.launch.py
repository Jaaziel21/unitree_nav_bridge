import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('unitree_nav_bridge')
    default_params = os.path.join(pkg_share, 'config', 'bridge_params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Ruta al YAML de parámetros del puente.'),

        Node(
            package='unitree_nav_bridge',
            executable='unitree_bridge',
            name='nav_to_unitree_bridge',
            output='screen',
            parameters=[params_file],
        ),
    ])
