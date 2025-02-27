import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.clock import Clock, ClockType
from rclpy.time import Time


class ScanEncoderClass(Node):

    def __init__(self):
        super().__init__('scan_encoder')

        self.subscription_robot1 = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.robot1_func,
            1)
        self.subscription_robot1  # prevent unused variable warning

        self.subscription_robot2 = self.create_subscription(
            LaserScan,
            '/robot2/scan',
            self.robot2_func,
            1)
        self.subscription_robot2  # prevent unused variable warning

        self.subscription_robot3 = self.create_subscription(
            LaserScan,
            '/robot3/scan',
            self.robot3_func,
            1)
        self.subscription_robot3  # prevent unused variable warning

        self.subscription_robot4 = self.create_subscription(
            LaserScan,
            '/robot4/scan',
            self.robot4_func,
            1)
        self.subscription_robot4  # prevent unused variable warning

        self.publisher_robot1 = self.create_publisher(LaserScan, '/robot1/scan_encoded', 10)
        self.publisher_robot2 = self.create_publisher(LaserScan, '/robot2/scan_encoded', 10)
        self.publisher_robot3 = self.create_publisher(LaserScan, '/robot3/scan_encoded', 10)
        self.publisher_robot4 = self.create_publisher(LaserScan, '/robot4/scan_encoded', 10)

    def robot1_func(self, msg):
        scan = LaserScan()
        scan = msg
        scan.header.frame_id = 'robot1/base_scan'

        self.publisher_robot1.publish(scan)
    
    def robot2_func(self, msg):
        scan = LaserScan()        
        scan = msg
        scan.header.frame_id = 'robot2/base_scan'

        self.publisher_robot2.publish(scan)
    
    def robot3_func(self, msg):
        scan = LaserScan()
        scan = msg
        scan.header.frame_id = 'robot3/base_scan'

        self.publisher_robot3.publish(scan)
            
    def robot4_func(self, msg):
        scan = LaserScan()
        scan = msg
        scan.header.frame_id = 'robot4/base_scan'

        self.publisher_robot4.publish(scan)

def main():
    rclpy.init()
    node = ScanEncoderClass()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()