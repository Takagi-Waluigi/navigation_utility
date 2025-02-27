from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    
        Node(
             package='tf_encoder',
             namespace='projectoroid1',
             executable='map_converter',
             remappings=[('/tf','tf'), ('/tf_static','tf_static'),],
        ),

        Node(
            package='tf_encoder',
            namespace='projectoroid2',
            executable='map_converter',
            remappings=[('/tf','tf'), ('/tf_static','tf_static'),],
        ),

        Node(
            package='tf_encoder',
            namespace='povroid3',
            executable='map_converter',
            remappings=[('/tf','tf'), ('/tf_static','tf_static'),],
        ),

        # Node(
        #     package='tf_encoder',
        #     namespace='povroid3',
        #     executable='map_converter',
        #     remappings=[('/tf','tf'), ('/tf_static','tf_static'),],
        # ),

        
        
    ])
