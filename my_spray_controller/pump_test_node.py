#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import time


class PumpTestNode(Node):
    def __init__(self):
        super().__init__('pump_test_node')
        
        # Publishers
        self.speed_pub = self.create_publisher(Float32, '/pump/speed_command', 10)
        self.stop_pub = self.create_publisher(Bool, '/spray/stop', 10)
        
        # Timer für Test-Sequenz
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Test-Parameter
        self.current_speed = 1200
        self.max_speed = 1300
        self.step_count = 0
        self.max_steps = 6  # 1200, 1220, 1240, 1260, 1280, 1300
        
        self.get_logger().info('Pump Test Node started')
        self.get_logger().info(f'Will test speeds from {self.current_speed} to {self.max_speed}')
        self.get_logger().info('Publishing every 2 seconds, then sending stop command')
        
    def timer_callback(self):
        if self.step_count < self.max_steps:
            # Speed Command senden
            speed_msg = Float32()
            speed_msg.data = float(self.current_speed)
            self.speed_pub.publish(speed_msg)
            
            self.get_logger().info(f'Step {self.step_count + 1}/{self.max_steps}: Speed = {self.current_speed}')
            
            # Nächste Geschwindigkeit
            self.current_speed += 20
            self.step_count += 1
            
        elif self.step_count == self.max_steps:
            # Stop Command senden
            self.get_logger().info('Test completed - sending STOP command')
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)
            
            self.step_count += 1  # Verhindert mehrfaches Senden
            
            # Timer stoppen
            self.timer.cancel()
            
            # Node nach 2 Sekunden beenden
            self.create_timer(2.0, self.shutdown_callback)
    
    def shutdown_callback(self):
        self.get_logger().info('Test node shutting down')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    test_node = None
    
    try:
        test_node = PumpTestNode()
        rclpy.spin(test_node)
        
    except KeyboardInterrupt:
        if test_node:
            test_node.get_logger().info('Ctrl+C received - sending emergency stop')
            stop_msg = Bool()
            stop_msg.data = True
            test_node.stop_pub.publish(stop_msg)
            time.sleep(0.5)  # Kurz warten damit Message ankommt
            
    except Exception as e:
        if test_node:
            test_node.get_logger().error(f'Error: {e}')
            
    finally:
        if test_node is not None:
            test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()