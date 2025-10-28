#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanTimeComparison(Node):
    def __init__(self):
        super().__init__('scan_time_comparison')
        
        self.last_callback_time = None
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.last_callback_time is not None:
            # Callback time delta (wie in deinem Code)
            callback_delta = (current_time - self.last_callback_time).nanoseconds / 1e9
            callback_freq = 1.0 / callback_delta if callback_delta > 0 else 0.0
            
            # msg.scan_time (wie in deinem Code)
            scan_time = msg.scan_time if hasattr(msg, 'scan_time') else None
            scan_time_freq = 1.0 / scan_time if scan_time and scan_time > 0 else None
            
            if scan_time:
                self.get_logger().info(f"Callback: {callback_delta:.3f}s ({callback_freq:.1f}Hz) | msg.scan_time: {scan_time:.3f}s ({scan_time_freq:.1f}Hz)")
            else:
                self.get_logger().info(f"Callback: {callback_delta:.3f}s ({callback_freq:.1f}Hz) | msg.scan_time: N/A")
        
        self.last_callback_time = current_time

def main():
    rclpy.init()
    node = ScanTimeComparison()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()