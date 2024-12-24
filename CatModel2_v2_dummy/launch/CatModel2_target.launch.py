import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    prefix = os.getenv('LAUNCH_PREFIX', '')
    return LaunchDescription([
        DeclareLaunchArgument(
            name='referenceFile',
            default_value=get_package_share_directory(
                'CatModel2_v2_description') + '/config/ocs2/reference.info'
        ),
        Node(
            package='CatModel2_v2_dummy',
            executable='CatModel2_target',
            name='CatModel2_target',
            output='screen',
            prefix=prefix,
            parameters=[
                {
                    'referenceFile': LaunchConfiguration('referenceFile')
                }
            ]
        )
    ])