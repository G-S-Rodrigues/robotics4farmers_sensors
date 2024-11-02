from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='r4f_realsense',
            executable='realsense_sim',
            name='realsense_node',
            parameters=[{'bag_dir': '/home/guilh/data_tese/vinha-11-07/run1'}],
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'] # DEBUG
        ),
    ])
