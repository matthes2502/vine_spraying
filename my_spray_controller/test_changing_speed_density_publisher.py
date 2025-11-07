#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray
import random

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        # Publishers
        self.speed_pub = self.create_publisher(Float32, '/rovo/speed', 10)
        self.density_pub = self.create_publisher(Int32MultiArray, '/grid_density/left', 10)
        
        # Density calibration (same as your controller)
        self.density_calibration = {
            0.16: {  # Gang 1 max speed
                'bottom': {60: 11545, 100: 22410},
                'middle': {60: 10583, 100: 22976}, 
                'top': {60: 10780, 100: 17457}
            },
            0.62: {  # Gang 2 max speed  
                'bottom': {60: 2897, 100: 5949},
                'middle': {60: 2716, 100: 6099},
                'top': {60: 2693, 100: 4582}
            },
            1.57: {  # Gang 3 max speed
                'bottom': {60: 1059, 100: 2440},
                'middle': {60: 1055, 100: 2549},
                'top': {60: 1070, 100: 1928}
            }
        }
        
        self.zones = ['bottom', 'middle', 'top']
        
        # Generate random speed once for entire session
        self.test_speed = random.uniform(0.16, 0.50)  # upper val 1.57 
        
        # Calculate timer period based on speed (1 meter intervals)
        self.meter_interval = 1.0 / self.test_speed
        
        # Timers
        self.speed_timer = self.create_timer(0.1, self.publish_speed)
        self.density_timer = self.create_timer(self.meter_interval, self.publish_density)
        
        self.get_logger().info(f"Test node started - Speed: {self.test_speed:.2f}m/s, Interval: {self.meter_interval:.2f}s")

    def linear_interpolate(self, x, x1, y1, x2, y2):
        """Linear interpolation"""
        if x1 == x2:
            return y1
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)

    def get_calibration_for_speed(self, current_speed, zone):
        """Get interpolated calibration data for current speed"""
        available_speeds = sorted(self.density_calibration.keys())
        
        # Clamp speed to available range
        if current_speed <= available_speeds[0]:
            return self.density_calibration[available_speeds[0]][zone]
        elif current_speed >= available_speeds[-1]:
            return self.density_calibration[available_speeds[-1]][zone]
        
        # Find surrounding speeds
        lower_speed = available_speeds[0]
        upper_speed = available_speeds[-1]
        
        for speed in available_speeds:
            if speed <= current_speed:
                lower_speed = speed
            if speed >= current_speed and speed > lower_speed:
                upper_speed = speed
                break
        
        if lower_speed == upper_speed:
            return self.density_calibration[lower_speed][zone]
        
        # Interpolate calibration data
        lower_cal = self.density_calibration[lower_speed][zone]
        upper_cal = self.density_calibration[upper_speed][zone]
        
        interpolated_cal = {}
        for density_percent in [60, 100]:
            points_lower = lower_cal[density_percent]
            points_upper = upper_cal[density_percent]
            
            interpolated_points = self.linear_interpolate(
                current_speed, lower_speed, points_lower, upper_speed, points_upper
            )
            interpolated_cal[density_percent] = int(interpolated_points)
        
        return interpolated_cal

    def density_percent_to_points(self, density_percent, zone):
        """Convert density percentage to LiDAR points for current speed"""
        calibration_data = self.get_calibration_for_speed(self.test_speed, zone)
        
        points_60 = calibration_data[60]
        points_100 = calibration_data[100]
        
        # Linear interpolation from percent to points
        points = self.linear_interpolate(
            density_percent, 60.0, points_60, 100.0, points_100
        )
        
        return int(points)

    def publish_speed(self):
        """Publish current speed"""
        msg = Float32()
        msg.data = self.test_speed
        self.speed_pub.publish(msg)

    def publish_density(self):
        """Publish random density data every meter"""
        # Generate random density percentages for each zone
        density_percentages = [random.uniform(50, 100) for _ in range(3)]
        
        # Convert percentages to points using interpolation
        density_points = []
        for i, percent in enumerate(density_percentages):
            points = self.density_percent_to_points(percent, self.zones[i])
            density_points.append(points)
        
        # Publish points
        msg = Int32MultiArray()
        msg.data = density_points
        self.density_pub.publish(msg)
        
        self.get_logger().info(
            f"Published density - Percent: {[f'{p:.1f}%' for p in density_percentages]}, "
            f"Points: {density_points}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()