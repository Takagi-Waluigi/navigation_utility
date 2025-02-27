import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFSubscriber(Node):
    def __init__(self):
        
        super().__init__('tfsubscriber')

        self.subscriber = self.create_subscription(
            TFMessage,
            'tf',
            self.tf_callback,
            10
        )

        self.subscriber

    def tf_callback(self, msg):
        for transform in msg.transforms:
            self.get_logger().info(f'Received transform from {transform.header.frame_id} to {transform.child_frame_id}')

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  frame_listener_node = TFSubscriber()
  
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
