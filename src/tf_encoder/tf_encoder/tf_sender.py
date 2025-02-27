import math

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf_encoder_B')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call callback_func
        # callback function on each message
        self.subscription = self.create_subscription(
            PoseStamped,
            '/processing_pose',
            self.callback_func,
            1)
        self.subscription  # prevent unused variable warning

    def callback_func(self, msg):
        self.get_logger().info('process -B')

        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        
        t.header.frame_id ='odom'  ##設定箇所.1
        t.child_frame_id ='base_footprint' ##設定箇所.1

        #値の代入
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y= msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w     


        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()