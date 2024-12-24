import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    prefix = os.getenv('LAUNCH_PREFIX', '')
    return LaunchDescription([
        DeclareLaunchArgument(
            name='gaitCommandFile',
            default_value=get_package_share_directory(
                'CatModel2_v2_description') + '/config/ocs2/gait.info'
        ),

        Node(
            package='CatModel2_v2_dummy',
            executable='CatModel2_gait_command',
            name='CatModel2_gait_command',
            output='screen',
            prefix=prefix,
            parameters=[
                {
                    'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
                }
            ],
        )
    ])
