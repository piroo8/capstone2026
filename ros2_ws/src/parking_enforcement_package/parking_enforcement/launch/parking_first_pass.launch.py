"""Launch the parking-enforcement first pass.

MAVROS, the external pose bridge, and the raw camera publishers are expected to
be running separately. This launch file brings up the parking package's stereo
depth, mission, obstacle-avoidance, and plate-reading nodes.
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
            executable='parking_mission_node',
            name='parking_mission_node',
            output='screen',
        ),
    ])
