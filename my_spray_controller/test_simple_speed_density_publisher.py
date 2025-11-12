#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray, Bool
from geometry_msgs.msg import Twist
from datetime import datetime

class SimpleTestPublisher(Node):
    def __init__(self):
        super().__init__('simple_test_publisher')
        
        # Publishers
        self.speed_pub = self.create_publisher(Twist, '/rovo/speed', 10)
        self.density_pub = self.create_publisher(Int32MultiArray, '/grid_density/left', 10)
        self.stop_pub = self.create_publisher(Bool, '/peripherie/stop', 10)
        
        # CONSTANT Test parameters - CHANGE THESE VALUES FOR TESTING
        self.test_speed = 0.62  # m/s - change this to test different speeds
        self.test_density = [100000, 4400, 0]  # [bottom, middle, top] - change these values
        self.test_duration = 15.0  # seconds - CHANGE THIS VALUE FOR TEST DURATION
        
        # Calculate timer period based on speed (1 meter intervals)
        self.meter_interval = 1.0 / self.test_speed  # seconds per meter
        
        # Timers
        self.speed_timer = self.create_timer(0.1, self.publish_speed)  # 10Hz speed
        self.density_timer = self.create_timer(self.meter_interval, self.publish_density)  # Every meter
        
        # Stop timer - triggers after test_duration
        self.stop_timer = self.create_timer(self.test_duration, self.publish_stop)
        
        # Flag to track if stop was already sent
        self.stop_sent = False
        
        self.get_logger().info(f"Simple test node started - Speed: {self.test_speed}m/s, Density interval: {self.meter_interval:.2f}s")
        self.get_logger().info(f"Constant density values: {self.test_density}")
        self.get_logger().info(f"Test will run for {self.test_duration} seconds")

        self.start_time = datetime.now()
        self.get_logger().info(f"=== START TIME: {self.start_time.strftime('%H:%M:%S.%f')[:-3]} ===")
        self.publish_density()

    def publish_speed(self):
        """Publish constant speed"""
        if not self.stop_sent:
            speed_msg = Twist()
            speed_msg.linear.x = self.test_speed
            self.speed_pub.publish(speed_msg)

    def publish_density(self):
        """Publish constant density data every meter"""
        if not self.stop_sent:
            msg = Int32MultiArray()
            msg.data = self.test_density
            self.density_pub.publish(msg)
            self.publish_time = datetime.now()
            self.get_logger().info(f"=== PUBLISHED POINTS AT: {self.publish_time.strftime('%H:%M:%S.%f')[:-3]} ===")

    def publish_stop(self):
        """Publish stop signal after test duration"""
        if not self.stop_sent:
            end_time = datetime.now()
            duration = (end_time - self.start_time).total_seconds()
            
            self.get_logger().info(f"=== PUBLISH STOP AT: {end_time.strftime('%H:%M:%S.%f')[:-3]} ===")
            self.get_logger().info(f"=== ACTUAL DURATION: {duration:.3f} seconds (configured: {self.test_duration}s) ===")
            
            msg = Bool()
            msg.data = True
            self.stop_pub.publish(msg)
            self.stop_sent = True
            self.get_logger().info(f"Test completed after {self.test_duration}s - Stop signal sent!")
            
            # Cancel other timers to stop publishing
            self.speed_timer.cancel()
            self.density_timer.cancel()
            self.stop_timer.cancel()

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