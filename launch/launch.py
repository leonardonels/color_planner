from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('color_planner'),
        'config',
        'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='color_planner',
            executable='color_planner_node',
            name='color_planner_node',
            output='screen',
            parameters=[params_file]
        )
    ])
