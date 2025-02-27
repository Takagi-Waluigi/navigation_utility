import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 名前空間を指定するためのリスト
    robot_namespaces = [
         'projectoroid1', 
        # 'projectoroid2',
        # 'povroid1', 
        # 'povroid2', 
        # 'povroid3'
        ]

    # ローンチ記述を保持するリスト
    nodes = []

    for ns in robot_namespaces:
        # 各ロボットに対してNodeを作成
        nav2_unity_node = Node(
            package='chapter5',  # 使用するパッケージ名
            executable='nav2_unity_pipeline',  # 実行するノードの名前
            name='nav2_unity_pipeline',
            namespace=ns,  # 名前空間を指定
            output='screen',
            parameters=[{
                'use_sim_time': False  # 例：シミュレーション時間を使用するかどうか
            }]
        )
        # 各ノードをリストに追加
        nodes.append(nav2_unity_node)

    # ローンチファイルの定義
    return LaunchDescription([
        *nodes
    ])
