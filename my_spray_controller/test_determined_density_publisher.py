#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray, Bool
from geometry_msgs.msg import Twist
import random
from datetime import datetime


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        # Publishers
        self.speed_pub = self.create_publisher(Twist, '/rovo/speed', 10)
        self.density_pub = self.create_publisher(Int32MultiArray, '/grid_density/left', 10)
        self.stop_pub = self.create_publisher(Bool, '/peripherie/stop', 10)
        self.density_percentage_pub = self.create_publisher(Int32MultiArray, '/density_percentages', 10)  # just for evaluating
        
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
        
        # PREDEFINED DENSITY PATTERNS (percentage values for each meter)
        # Each inner list represents one meter: [bottom%, middle%, top%]
        self.density_pattern = [
            [80, 80, 80],    # Meter 1
            [0, 80, 80],   # Meter 2
            [60, 0, 100],     # Meter 3
            [0, 0, 0],    # Meter 4
            [50, 70, 100],    # Meter 5
            # Add more patterns as needed
        ]
        
        # Calculate total distance from pattern length
        self.total_distance = len(self.density_pattern)
        self.current_meter = 0
        
        # Generate random speed once for entire session
        # self.test_speed = random.uniform(0.16, 0.50)  # upper val 1.57 
        self.test_speed = 0.62  # upper val 1.57 
        
        # Calculate timer period based on speed (1 meter intervals)
        self.meter_interval = 1.0 / self.test_speed
        
        # Timers
        self.speed_timer = self.create_timer(0.1, self.publish_speed)
        self.density_timer = self.create_timer(self.meter_interval, self.publish_density)
        
        self.get_logger().info(
            f"Test node started - Speed: {self.test_speed:.2f}m/s, "
            f"Interval: {self.meter_interval:.2f}s, "
            f"Total distance: {self.total_distance}m"
        )
        self.get_logger().info(f"Density pattern: {self.density_pattern}")

        self.start_time = datetime.now()
        self.get_logger().info(f"=== START TIME: {self.start_time.strftime('%H:%M:%S.%f')[:-3]} ===")
        self.publish_speed()
        self.publish_density()

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
        # Handle 0% density case
        if density_percent <= 0:
            return 0
            
        # Handle values below 60% by extrapolation
        calibration_data = self.get_calibration_for_speed(self.test_speed, zone)
        
        points_60 = calibration_data[60]
        points_100 = calibration_data[100]
        
        # For values below 60%, extrapolate from 60% down to 0%
        if density_percent < 60:
            # Extrapolate: assume linear relationship from 0% -> 0 points to 60% -> points_60
            points = self.linear_interpolate(
                density_percent, 0.0, 0, 60.0, points_60
            )
        else:
            # Regular interpolation between 60% and 100%
            points = self.linear_interpolate(
                density_percent, 60.0, points_60, 100.0, points_100
            )
        
        return int(max(0, points))  # Ensure non-negative

    def publish_speed(self):
        """Publish constant speed"""
        speed_msg = Twist()
        speed_msg.linear.x = self.test_speed
        self.speed_pub.publish(speed_msg)

    def publish_density(self):
        """Publish predefined density data every meter"""
        # Check if we've reached the end of the pattern
        if self.current_meter >= len(self.density_pattern):
            self.get_logger().info("Pattern completed - stopping density publishing")
            self.density_timer.cancel()
            msg = Bool()
            msg.data = True
            self.stop_pub.publish(msg)
            end_time = datetime.now()
            duration = (end_time - self.start_time).total_seconds()
            
            self.get_logger().info(f"=== PUBLISH STOP AT: {end_time.strftime('%H:%M:%S.%f')[:-3]} ===")
            self.get_logger().info(f"=== ACTUAL DURATION: {duration:.3f} ===")
            return
        
        # Get current meter's density percentages
        density_percentages = self.density_pattern[self.current_meter]
        
        # Convert percentages to points using interpolation
        density_points = []
        for i, percent in enumerate(density_percentages):
            points = self.density_percent_to_points(percent, self.zones[i])
            density_points.append(points)
        
        # Publish points
        msg = Int32MultiArray()
        msg.data = density_points
        self.density_pub.publish(msg)

        # Publish density percentages just for tracking and evaluation
        msg_percentage = Int32MultiArray()
        msg_percentage.data = density_percentages
        self.density_percentage_pub.publish(msg_percentage)
        
        self.get_logger().info(
            f"Meter {self.current_meter + 1}/{self.total_distance} - "
            f"Percent: {density_percentages}, "
            f"Points: {density_points}"
        )
        
        # Move to next meter
        self.current_meter += 1

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