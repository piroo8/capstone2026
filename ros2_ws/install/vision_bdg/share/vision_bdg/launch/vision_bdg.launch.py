from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_vicon = DeclareLaunchArgument('use_vicon', default_value='true')
    use_realsense = DeclareLaunchArgument('use_realsense', default_value='false')

    return LaunchDescription([
        use_vicon,
        use_realsense,

        Node(
            package='vision_bdg',
            executable='vision_bdg',
            name='vision_bdg',
            output='screen',
            parameters=[
                {'use_vicon': LaunchConfiguration('use_vicon')},
                {'use_realsense': LaunchConfiguration('use_realsense')}
            ],
        ),
    ])
