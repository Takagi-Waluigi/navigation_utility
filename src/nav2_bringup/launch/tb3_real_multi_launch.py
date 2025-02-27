# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    robots = [
        # {'name': 'povroid1'},
        # {'name': 'povroid2'},
         {'name': 'povroid3'},
         {'name': 'projectoroid1'},
         {'name': 'projectoroid2'},
        #  {'name': 'framelight1'},
        #  {'name': 'framelight2'},
        #  {'name': 'framelight3'}        
    ]

    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_povroid1_params_file_cmd = DeclareLaunchArgument(
        'povroid1_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_povroid1.yaml'),
        description='Full path to robot paramters')
    
    declare_povroid2_params_file_cmd = DeclareLaunchArgument(
        'povroid2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_povroid2.yaml'),
        description='Full path to robot paramters')
    
    declare_povroid3_params_file_cmd = DeclareLaunchArgument(
        'povroid3_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_povroid3.yaml'),
        description='Full path to robot paramters')
    
    declare_projectoroid1_params_file_cmd = DeclareLaunchArgument(
        'projectoroid1_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_projectoroid1.yaml'),
        description='Full path to robot paramters')
    
    declare_projectoroid2_params_file_cmd = DeclareLaunchArgument(
        'projectoroid2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_projectoroid2.yaml'),
        description='Full path to robot paramters')
    
    declare_framelight1_params_file_cmd = DeclareLaunchArgument(
        'framelight1_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_framelight1.yaml'),
        description='Full path to robot paramters')
    
    declare_framelight2_params_file_cmd = DeclareLaunchArgument(
        'framelight2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_framelight2.yaml'),
        description='Full path to robot paramters')

    declare_framelight3_params_file_cmd = DeclareLaunchArgument(
        'framelight3_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_framelight3.yaml'),
        description='Full path to robot paramters')


    bringup_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(f"{robot['name']}_params_file")

        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={
                                  'namespace': TextSubstitution(text=robot['name']),
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'bringup_launch.py')),
                launch_arguments={'namespace': robot['name'],
                          'use_namespace': 'true',
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items()
            )
        ])

        bringup_cmds.append(group)

    # bringup_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, 'bringup_launch.py')),
    #     launch_arguments={'namespace': namespace,
    #                       'use_namespace': 'true',
    #                       'map': map_yaml_file,
    #                       'use_sim_time': use_sim_time,
    #                       'params_file': params_file,
    #                       'autostart': autostart,
    #                       'use_composition': use_composition,
    #                       'use_respawn': use_respawn}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_povroid1_params_file_cmd)
    ld.add_action(declare_povroid2_params_file_cmd)
    ld.add_action(declare_povroid3_params_file_cmd)
    ld.add_action(declare_projectoroid1_params_file_cmd)
    ld.add_action(declare_projectoroid2_params_file_cmd)
    ld.add_action(declare_framelight1_params_file_cmd)
    ld.add_action(declare_framelight2_params_file_cmd)
    ld.add_action(declare_framelight3_params_file_cmd)


    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)

    #ld.add_action(bringup_cmd)
    for bringup_cmd in  bringup_cmds:
        ld.add_action(bringup_cmd)

    return ld
