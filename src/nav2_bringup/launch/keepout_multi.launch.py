import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)

from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    robots = [
        {'name': 'robot1'},
        {'name': 'robot2'}
        ]
    
    KEEPOUT_FILTER_LAUNCH = '/keepout.launch.py'

    mask_yaml_file = LaunchConfiguration('mask')

    bringup_dir = get_package_share_directory('nav2_bringup')


    declare_robot1_keepout_params_file_cmd = DeclareLaunchArgument(
        'robot1_keepout_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'keepout_robot1.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')
    
    declare_robot2_keepout_params_file_cmd = DeclareLaunchArgument(
        'robot2_keepout_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'keepout_robot2.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')
    
    declare_mask_yaml_cmd = DeclareLaunchArgument(
        'mask',
        default_value=os.path.join(
            bringup_dir, 
            'maps',
            'map_seibu_keepout.yaml'
            ),
        description='Full path to map file to load')

    keepout_instances_cmds = []

    for robot in robots:
        keepout_params_file = LaunchConfiguration(f"{robot['name']}_keepout_params_file")

        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), KEEPOUT_FILTER_LAUNCH]),
                launch_arguments={'namespace': robot['name'],
                                  'params_file': keepout_params_file,
                                  'mask': mask_yaml_file}.items(), #<<編集箇所-6
            ),
        ])

        keepout_instances_cmds.append(group)       


    ld = LaunchDescription()

    ld.add_action(declare_mask_yaml_cmd)
    
    #台数分追加すること
    ld.add_action(declare_robot1_keepout_params_file_cmd)
    ld.add_action(declare_robot2_keepout_params_file_cmd)

    for keepout_cmd in keepout_instances_cmds:
        ld.add_action(keepout_cmd)

    return ld