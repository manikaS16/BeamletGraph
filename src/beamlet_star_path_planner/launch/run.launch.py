from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    config = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share',
        'beamlet_star_path_planner',
        'config',
        'grid.yaml')

    return LaunchDescription([
        Node(
            package='beamlet_star_path_planner',
            executable='beamlet_star_node',
            parameters=[config]
        )
    ])
