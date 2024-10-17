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

"""
Example for spawning multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros') #追記

    KEEPOUT_FILTER_LAUNCH = '/keepout.launch.py'

    # それぞれのロボットの初期設定
    # name以外はGazeboの初期位置なので実機では設定不要
    robots = [
        {'name': 'robot1', 'x_pose': 7.0, 'y_pose': 2.5, 'z_pose': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'robot2', 'x_pose': 7.0, 'y_pose': -2.5, 'z_pose': 0.01,'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        ]

    # Simulation settings
    #world = LaunchConfiguration('world')
    use_gazebo = LaunchConfiguration('use_gazebo', default=True)

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='false')

    # Declare the launch arguments
    # ワールドの設定とGazeboサーバの設定は別で
    # declare_world_cmd = DeclareLaunchArgument(
    #     'world',
    #     default_value=os.path.join(
    #         get_package_share_directory('turtlebot3_gazebo'),
    #         'worlds',
    #         #'turtlebot3_world.world'
    #         'shibuya_seibu.world'
    #         ),
    #     description='Full path to world file to load')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 
            'maps',
            #'turtlebot3_world.yaml'
            'map_seibu.yaml'
            ),
        description='Full path to map file to load')

    #パラメータファイルの設定
    #以前はYamlRewrittenによって上書きされていたが、ファイルごとに固有のパラメータ
    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        'robot1_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_keepout_1.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    declare_robot2_params_file_cmd = DeclareLaunchArgument(
        'robot2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_keepout_2.yaml'),
        description='Full path to the ROS2 parameters file to use for robot2 launched nodes')
    
    #Keep out param関連
    declare_robot1_keepout_params_file_cmd = DeclareLaunchArgument(
        'robot1_keepout_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'keepout_robot1.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='false',
        description='Automatically startup the stacks')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'shibuya_seibu.world' #ワールドの指定
        #'turtlebot3_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(use_gazebo), #追加 
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(use_gazebo), #追加 
    )

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
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
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                           'launch',
                                                           'tb3_simulation_launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': map_yaml_file,
                                  'use_sim_time': 'False',
                                  'params_file': params_file,
                                  'autostart': autostart,
                                  'use_rviz': 'False',
                                  'use_simulator': 'False',
                                  'headless': 'False',
                                  'use_robot_state_pub': use_robot_state_pub,
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                  'roll': TextSubstitution(text=str(robot['roll'])),
                                  'pitch': TextSubstitution(text=str(robot['pitch'])),
                                  'yaw': TextSubstitution(text=str(robot['yaw'])),
                                  'robot_name':TextSubstitution(text=robot['name']), }.items(),
                condition=IfCondition(use_gazebo), #追加 
            ),


            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                           'launch',
                                                           'tb3_simulation_launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': map_yaml_file,
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'autostart': autostart,
                                  'use_rviz': 'False',
                                #  'use_simulator': 'False',
                                #  'headless': 'False',
                                  'use_robot_state_pub': use_robot_state_pub,
                                #   'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                #   'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                #   'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                #   'roll': TextSubstitution(text=str(robot['roll'])),
                                #   'pitch': TextSubstitution(text=str(robot['pitch'])),
                                #   'yaw': TextSubstitution(text=str(robot['yaw'])),
                                  'robot_name':TextSubstitution(text=robot['name']), }.items(),
                condition=UnlessCondition(use_gazebo), #追加 
            ),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), KEEPOUT_FILTER_LAUNCH]),
            #     launch_arguments={'namespace': robot['name'],
            #                       'params_file': keepout_params_file,
            #                       'mask': mask_yaml_file}.items(), #<<編集箇所-6
            # ),

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' map yaml: ', map_yaml_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' params yaml: ', params_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' using robot state pub: ', use_robot_state_pub]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' autostart: ', autostart])
        ])

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    #ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)    

    # ロボットの台数分必要なので注意
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)

    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # 実機を使う場合は不要
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to start gazebo, robots and simulations
    #ld.add_action(start_gazebo_cmd)

    ld.add_action(gzserver_cmd) #追加
    ld.add_action(gzclient_cmd) #追加

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
