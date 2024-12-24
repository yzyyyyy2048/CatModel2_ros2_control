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
            name='taskFile',
            default_value=get_package_share_directory(
                'CatModel2_v2_description') + '/config/ocs2/task.info'
        ),
        DeclareLaunchArgument(
            name='referenceFile',
            default_value=get_package_share_directory(
                'CatModel2_v2_description') + '/config/ocs2/reference.info'
        ),
        DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'CatModel2_v2_description') + '/urdf/CatModel2_v2.urdf'
        ),

        Node(
            package='CatModel2_v2_dummy',
            executable='CatModel2_dummy',
            name='CatModel2_dummy',
            output='screen',
            prefix=prefix,
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
            ]
        )
    ])
