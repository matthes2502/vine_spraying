#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray

class SimpleTestPublisher(Node):
    def __init__(self):
        super().__init__('simple_test_publisher')
        
        # Publishers
        self.speed_pub = self.create_publisher(Float32, '/rovo/speed', 10)
        self.density_pub = self.create_publisher(Int32MultiArray, '/grid_density/left', 10)
        
        # CONSTANT Test parameters - CHANGE THESE VALUES FOR TESTING
        self.test_speed = 0.62  # m/s - change this to test different speeds
        self.test_density = [2900, 4408, 4500]  # [bottom, middle, top] - change these values
        
        # Calculate timer period based on speed (1 meter intervals)
        self.meter_interval = 1.0 / self.test_speed  # seconds per meter
        
        # Timers
        self.speed_timer = self.create_timer(0.1, self.publish_speed)  # 10Hz speed
        self.density_timer = self.create_timer(self.meter_interval, self.publish_density)  # Every meter
        
        self.get_logger().info(f"Simple test node started - Speed: {self.test_speed}m/s, Density interval: {self.meter_interval:.2f}s")
        self.get_logger().info(f"Constant density values: {self.test_density}")

    def publish_speed(self):
        """Publish constant speed"""
        msg = Float32()
        msg.data = self.test_speed
        self.speed_pub.publish(msg)

    def publish_density(self):
        """Publish constant density data every meter"""
        msg = Int32MultiArray()
        msg.data = self.test_density
        self.density_pub.publish(msg)
        # self.get_logger().info(f"Published constant density: {self.test_density}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()