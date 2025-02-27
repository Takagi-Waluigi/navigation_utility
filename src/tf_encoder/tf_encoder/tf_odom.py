import math

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry

import numpy as np
import time

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster



class OdomEncoder(Node):

    def __init__(self):
        super().__init__('tf_odom_encoder')
        self.get_logger().info('Begin Node')

        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription_robot1 = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.robot1_func,
            1)
        
        self.subscription_robot2 = self.create_subscription(
            Odometry,
            '/robot2/odom',
            self.robot2_func,
            1)
        
        self.subscription_robot3 = self.create_subscription(
            Odometry,
            '/robot3/odom',
            self.robot3_func,
            1)
        
        self.subscription_robot4 = self.create_subscription(
            Odometry,
            '/robot4/odom',
            self.robot4_func,
            1)        

        self.subscription_robot1  
        self.subscription_robot2  
        self.subscription_robot3  
        self.subscription_robot4

        self.lastFrameTime = 0.0
        

    def odomEncodeFunc(self, odomMessage, topicName):
        #self.get_logger().info(topicName + ':get odometry message')

        t = TransformStamped()

        t.header.stamp = odomMessage.header.stamp
        
        t.header.frame_id = topicName + '/odom'  ##設定箇所.1
        t.child_frame_id = topicName + '/base_footprint' ##設定箇所.1

        #値の代入
        t.transform.translation.x = odomMessage.pose.pose.position.x
        t.transform.translation.y = odomMessage.pose.pose.position.y
        t.transform.translation.z = odomMessage.pose.pose.position.z

        t.transform.rotation.x = odomMessage.pose.pose.orientation.x
        t.transform.rotation.y= odomMessage.pose.pose.orientation.y
        t.transform.rotation.z = odomMessage.pose.pose.orientation.z
        t.transform.rotation.w = odomMessage.pose.pose.orientation.w   


        self.tf_broadcaster.sendTransform(t)
        

    def robot1_func(self, msg):
        diffTime = time.time() - self.lastFrameTime

        #self.get_logger().info('Odom Subsrcibe Rate:' + str(diffTime))
        self.odomEncodeFunc(msg, 'robot1')

        self.lastFrameTime = time.time()
    
    def robot2_func(self, msg):
        self.odomEncodeFunc(msg, 'robot2')
    
    def robot3_func(self, msg):
        self.odomEncodeFunc(msg, 'robot3')
    
    def robot4_func(self, msg):
        self.odomEncodeFunc(msg, 'robot4')


def main():
    rclpy.init()
    node = OdomEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()