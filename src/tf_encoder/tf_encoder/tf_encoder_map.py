#!/usr/bin/env python3 
 
"""
Description:
Publish the coordinate transformation between the map frame
and the base_link frame.
The output is [x,y,yaw]. yaw is -pi to pi
-------
Subscription Topics:
/tf - geometry_msgs/TransformStamped[]
-------
Publishing Topics:
/map_to_base_link_pose2d – std_msgs/Float64MultiArray
-------
Author: Addison Sears-Collins
Website: AutomaticAddison.com
Date: November 25, 2021
"""
 
# Import the ROS client library for Python 
import rclpy 
 
# Enables the use of rclpy's Node class
from rclpy.node import Node 
 
# Base class to handle exceptions
from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray 

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
 
# Math library
import math 
 
 
class FrameListener(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('tf_encoder_map')
 
    # Declare and acquire `target_frame` parameter
    self.declare_parameter('target_frame', 'odom_1') #ほしいフレーム情報を指定する
    self.target_frame = self.get_parameter(
      'target_frame').get_parameter_value().string_value
 
    self.tf_buffer = Buffer() #バッファ用
    self.tf_listener = TransformListener(self.tf_buffer, self)
      
    # This node publishes the 2d pose.
    # Maximum queue size of 1. 

    self.publisher = self.create_publisher(
      Pose, 
      'map_to_odom', 
      1)
   
 
    # Call on_timer function on a set interval
    timer_period = 0.5
    self.timer = self.create_timer(timer_period, self.on_timer)
    
     
  def on_timer(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    """
    # Store frame names in variables that will be used to
    # compute transformations   
    from_frame_rel = 'map'  #ターゲットフレーム
    to_frame_rel = 'odom'

    # from_frame_rel = 'map'  #ターゲットフレーム
    # to_frame_rel = 'odom'
   
    trans = None
     
    try:
        now = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform(
                  from_frame_rel,
                  to_frame_rel,
                  now)
        self.get_logger().info('Succeded to Lookup ---Map to Odom')

        # Publish the 2D pose
        msg = Pose()

        #msg.header.seq = trans.header.s
        #msg.header.stamp = trans.header.stamp

        msg.position.x = trans.transform.translation.x
        msg.position.y = trans.transform.translation.y
        msg.position.z = trans.transform.translation.z

        msg.orientation.x = trans.transform.rotation.x
        msg.orientation.y = trans.transform.rotation.y
        msg.orientation.z = trans.transform.rotation.z
        msg.orientation.w = trans.transform.rotation.w

        self.publisher.publish(msg)

        print("encoded!!!")

    except TransformException as ex:
        self.get_logger().info(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
    return
    

 
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  frame_listener_node = FrameListener()
  
  # Spin the node so the callback function is called.
  # Publish any pending messages to the topics.
  try:
    rclpy.spin(frame_listener_node)
  except KeyboardInterrupt:
    pass
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()