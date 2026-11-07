"""Start rosbridge, a static HTTP server for web/, and webxr_bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory('quest_webxr_bridge'), 'web')

    http_port = LaunchConfiguration('http_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')

    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('publish_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('frame_id', default_value='world'),
        DeclareLaunchArgument('stale_timeout_s', default_value='0.5'),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': ParameterValue(rosbridge_port, value_type=int),
                'address': '0.0.0.0',
            }],
        ),

        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', http_port,
                 '--bind', '0.0.0.0', '--directory', web_dir],
            name='webxr_http_server',
            output='screen',
        ),

        Node(
            package='quest_webxr_bridge',
            executable='webxr_bridge',
            name='webxr_bridge',
            output='screen',
            parameters=[{
                'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
                'frame_id': LaunchConfiguration('frame_id'),
                'stale_timeout_s': ParameterValue(LaunchConfiguration('stale_timeout_s'), value_type=float),
            }],
        ),
    ])
