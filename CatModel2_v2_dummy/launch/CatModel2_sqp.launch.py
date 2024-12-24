import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = get_package_share_directory('CatModel2_v2_dummy') + "/rviz/CatModel2.rviz"
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='description_name',
            default_value='legged_robot_description'
        ),
        launch.actions.DeclareLaunchArgument(
            name='multiplot',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'CatModel2_v2_description') + '/config/ocs2/task.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='referenceFile',
            default_value=get_package_share_directory(
                'CatModel2_v2_description') + '/config/ocs2/reference.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'CatModel2_v2_description') + '/urdf/CatModel2_v2.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gaitCommandFile',
            default_value=get_package_share_directory(
                'CatModel2_v2_description') + '/config/ocs2/gait.info'
        ),
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[launch.substitutions.LaunchConfiguration("urdfFile")],
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", rviz_config_file],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('rviz'))
        ),
        launch_ros.actions.Node(
            package='CatModel2_v2_dummy',
            executable='CatModel2_sqp_mpc',
            name='CatModel2_sqp_mpc',
            output='screen',
            prefix= "",
            parameters=[
                {
                    'multiplot': launch.substitutions.LaunchConfiguration('multiplot')
                },
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='CatModel2_v2_dummy',
            executable='CatModel2_dummy',
            name='CatModel2_dummy',
            output='screen',
            prefix="gnome-terminal --",
            parameters=[
                {
                    'multiplot': launch.substitutions.LaunchConfiguration('multiplot')
                },
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='CatModel2_v2_dummy',
            executable='CatModel2_target',
            name='CatModel2_target',
            output='screen',
            prefix="gnome-terminal --",
            parameters=[
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
            ]
        ),
        launch_ros.actions.Node(
            package='CatModel2_v2_dummy',
            executable='CatModel2_gait_command',
            name='CatModel2_gait_command',
            output='screen',
            prefix="gnome-terminal --",
            parameters=[
                {
                    'multiplot': launch.substitutions.LaunchConfiguration('multiplot')
                },
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
