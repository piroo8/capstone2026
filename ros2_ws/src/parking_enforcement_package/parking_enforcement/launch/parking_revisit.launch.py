"""Launch the revisit pass for the parking-enforcement demo.

Like the first-pass launch, this assumes MAVROS, the external pose bridge, and
the raw camera publishers already exist in the wider system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parking_enforcement',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            output='screen',
        ),
        Node(
            package='parking_enforcement',
            executable='local_costmap_node',
            name='local_costmap_node',
            output='screen',
        ),
        Node(
            package='parking_enforcement',
            executable='plate_reader_node',
            name='plate_reader_node',
            output='screen',
        ),
        Node(
            package='parking_enforcement',
            executable='parking_revisit_node',
            name='parking_revisit_node',
            output='screen',
        ),
    ])
