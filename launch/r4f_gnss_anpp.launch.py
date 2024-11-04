from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='r4f_sensors',
            executable='gnss_sim',
            name='GnssNode',
            parameters=[{'data_dir': '/home/guilh/data_tese/vinha-11-07/run3'}],
            output='screen',
            arguments=['--ros-args', '--log-level', 'DEBUG'] # DEBUG
        ),
    ])
