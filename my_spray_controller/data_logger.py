#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32, Bool
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PointStamped, Twist
import csv
import os
from datetime import datetime

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')
        
        # Create logging directory if it doesn't exist
        self.log_dir = '/home/matthes/Projects/ros2_ws/src/my_spray_controller/value_logging'
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # CSV file paths
        self.meter_data_file = os.path.join(self.log_dir, f'meter_data_{timestamp}.csv')
        self.sensor_data_file = os.path.join(self.log_dir, f'sensor_data_{timestamp}.csv')
        
        # Initialize CSV files with headers
        self.init_meter_csv()
        self.init_sensor_csv()
        
        # Subscribers for meter-based data
        self.density_percentage_sub = self.create_subscription(
            Int32MultiArray, '/density_percentages', self.density_percentage_callback, 10)
        self.grid_density_sub = self.create_subscription(
            Int32MultiArray, '/grid_density/left', self.grid_density_callback, 10)
        self.valve_commands_sub = self.create_subscription(
            Float32MultiArray, '/valve/commands', self.valve_commands_callback, 10)
        self.pump_pwm_sub = self.create_subscription(
            Float32MultiArray, '/pump/pwm_command', self.pump_pwm_callback, 10)
            
        # Subscribers for continuous sensor data
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/pressure', self.pressure_callback, 10)
        self.flow_rate_sub = self.create_subscription(
            Float32, '/flow_sensor_node/flow_rate_l_per_min', self.flow_rate_callback, 10)
        self.speed_sub = self.create_subscription(
            Twist, '/rovo/speed', self.speed_callback, 10)
        
        # Subscriber for stop signal
        self.stop_sub = self.create_subscription(
            Bool, '/peripherie/stop', self.stop_callback, 10)
        
        # Data storage for meter-based logging
        self.current_meter = 0
        self.logging_active = False
        self.start_time = 0.0  # Initialize with 0.0 instead of None
        self.end_time = 0.0    # Initialize with 0.0 instead of None
        self.meter_times = []  # Already correctly initialized as empty list
        self.data_received_flags = {
            # 'density': False,  # Optional - might not be present
            'grid': False, 
            'valve': False,
            # 'pump': False  # Temporarily removed since pump is off
        }
        
        # Sensor data buffer - store latest values
        self.sensor_buffer = {
            'pressure': 0.0,
            'flow_rate': 0.0,
            'speed': 0.0
        }
        
        # Timer will be created when logging starts
        self.sensor_log_timer = None
        
        self.meter_data = {
            'meter': 0,
            'time_relative': 0.0,
            'density_percentage_unten': 0,
            'density_percentage_mitte': 0, 
            'density_percentage_oben': 0,
            'grid_density_unten': 0,
            'grid_density_mitte': 0,
            'grid_density_oben': 0,
            'valve_commands_unten': 0.0,
            'valve_commands_mitte': 0.0,
            'valve_commands_oben': 0.0,
            'pump_pwm_command': 0.0
        }
        
        self.get_logger().info(f'Data logger started. Saving to:')
        self.get_logger().info(f'  Meter data: {self.meter_data_file}')
        self.get_logger().info(f'  Sensor data: {self.sensor_data_file}')
    
    def init_meter_csv(self):
        """Initialize meter data CSV with headers"""
        with open(self.meter_data_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'meter', 'time_relative',
                'density_percentage_unten', 'density_percentage_mitte', 'density_percentage_oben',
                'grid_density_unten', 'grid_density_mitte', 'grid_density_oben', 
                'valve_commands_unten', 'valve_commands_mitte', 'valve_commands_oben',
                'pump_pwm_command'
            ])
    
    def init_sensor_csv(self):
        """Initialize sensor data CSV with headers"""
        with open(self.sensor_data_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time_relative', 'pressure', 'flow_rate_l_per_min', 'speed'])
    
    def density_percentage_callback(self, msg):
        """Callback for density percentage data"""
        self.get_logger().info(f'Received density_percentage: {msg.data}')
        if not self.logging_active:
            return  # Only log if already started by grid_density
            
        if len(msg.data) >= 3:
            self.meter_data['density_percentage_unten'] = msg.data[0]
            self.meter_data['density_percentage_mitte'] = msg.data[1] 
            self.meter_data['density_percentage_oben'] = msg.data[2]
            self.data_received_flags['density'] = True
            self.get_logger().info(f'Set density flag, flags: {self.data_received_flags}')
            self.check_and_save_meter_data()
    
    def grid_density_callback(self, msg):
        """Callback for grid density data"""
        self.get_logger().info(f'Received grid_density: {msg.data}')
        if len(msg.data) >= 3:
            # Start logging with first grid density message
            if not self.logging_active:
                self.logging_active = True
                self.start_time = self.get_clock().now().nanoseconds / 1e9
                # Create timer when logging starts
                self.sensor_log_timer = self.create_timer(0.5, self.log_sensor_data)
                self.get_logger().info(f'Logging started at time: {self.start_time}')
            
            self.current_meter += 1
            current_time = self.get_clock().now().nanoseconds / 1e9
            relative_time = current_time - self.start_time
            self.meter_times.append(relative_time)
            
            self.meter_data['meter'] = self.current_meter
            self.meter_data['time_relative'] = relative_time
            self.meter_data['grid_density_unten'] = msg.data[0]
            self.meter_data['grid_density_mitte'] = msg.data[1]
            self.meter_data['grid_density_oben'] = msg.data[2]
            self.data_received_flags['grid'] = True
            self.get_logger().info(f'Set grid flag for meter {self.current_meter}, flags: {self.data_received_flags}')
            self.check_and_save_meter_data()
    
    def valve_commands_callback(self, msg):
        """Callback for valve commands data"""
        self.get_logger().info(f'Received valve_commands: {msg.data}')
        if not self.logging_active:
            return
            
        # Parse valve commands: [1.0, pressure1, flow1, 2.0, pressure2, flow2, 3.0, pressure3, flow3]
        if len(msg.data) >= 9:
            # Extract flow percentages for each valve (indices 2, 5, 8)
            self.meter_data['valve_commands_unten'] = msg.data[2]  # valve 1 flow percent (unten)
            self.meter_data['valve_commands_mitte'] = msg.data[5]  # valve 2 flow percent (mitte)  
            self.meter_data['valve_commands_oben'] = msg.data[8]   # valve 3 flow percent (oben)
            self.data_received_flags['valve'] = True
            self.get_logger().info(f'Set valve flag, flags: {self.data_received_flags}')
            self.check_and_save_meter_data()
    
    def pump_pwm_callback(self, msg):
        """Callback for pump PWM command data"""
        self.get_logger().info(f'Received pump_pwm: {msg.data}')
        if not self.logging_active:
            return
            
        if len(msg.data) >= 1:
            self.meter_data['pump_pwm_command'] = msg.data[0]
            self.data_received_flags['pump'] = True
            self.get_logger().info(f'Set pump flag, flags: {self.data_received_flags}')
            self.check_and_save_meter_data()
    
    def check_and_save_meter_data(self):
        """Save meter data when all fields are populated"""
        # Check if all data types have been received for this meter
        if all(self.data_received_flags.values()):
            self.save_meter_data()
            # Reset flags for next meter
            self.data_received_flags = {
                # 'density': False,  # Optional - might not be present
                'grid': False, 
                'valve': False,
                # 'pump': False  # Temporarily removed since pump is off
            }
    
    def save_meter_data(self):
        """Save current meter data to CSV"""
        with open(self.meter_data_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                self.meter_data['meter'],
                self.meter_data['time_relative'],
                self.meter_data['density_percentage_unten'],
                self.meter_data['density_percentage_mitte'], 
                self.meter_data['density_percentage_oben'],
                self.meter_data['grid_density_unten'],
                self.meter_data['grid_density_mitte'],
                self.meter_data['grid_density_oben'],
                self.meter_data['valve_commands_unten'],
                self.meter_data['valve_commands_mitte'],
                self.meter_data['valve_commands_oben'],
                self.meter_data['pump_pwm_command']
            ])
        self.get_logger().info(f'Saved meter {self.meter_data["meter"]} data at time {self.meter_data["time_relative"]:.2f}s')
    
    def pressure_callback(self, msg):
        """Callback for pressure sensor data (2Hz)"""
        if self.logging_active:
            self.sensor_buffer['pressure'] = msg.fluid_pressure
    
    def flow_rate_callback(self, msg):
        """Callback for flow rate sensor data (2Hz)"""
        if self.logging_active:
            self.sensor_buffer['flow_rate'] = msg.data

    def speed_callback(self, msg):
        """Callback for speed data (Twist message)"""
        if self.logging_active:
            self.sensor_buffer['speed'] = msg.linear.x

    def log_sensor_data(self):
        """Periodically log current sensor buffer values"""
        if not self.logging_active or self.sensor_log_timer is None:
            return  # Don't log if not active
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        relative_time = current_time - self.start_time
        
        with open(self.sensor_data_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                relative_time,
                self.sensor_buffer['pressure'],
                self.sensor_buffer['flow_rate'],
                self.sensor_buffer['speed']
            ])

    def stop_callback(self, msg):
        """Callback for stop signal from coordinator"""
        self.get_logger().info('Received stop signal!')
        if self.logging_active:
            # Stop logging immediately
            self.logging_active = False
            
            # Properly destroy and nullify the timer
            if self.sensor_log_timer is not None:
                self.sensor_log_timer.destroy()
                self.sensor_log_timer = None
                self.get_logger().info('Timer destroyed')
            
            self.end_time = self.get_clock().now().nanoseconds / 1e9
            relative_end_time = self.end_time - self.start_time
            
            self.get_logger().info(f'Stop signal received. Logging ended at relative time: {relative_end_time:.2f}s')
            self.get_logger().info(f'Total logging duration: {relative_end_time:.2f}s')
            self.get_logger().info(f'Logged {self.current_meter} meters')
            
            # Add summary to meter data file
            with open(self.meter_data_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([])  # Empty line
                writer.writerow(['# Summary'])
                writer.writerow(['# start_time', 0])
                writer.writerow(['# end_time', relative_end_time])
                writer.writerow(['# total_meters', self.current_meter])


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()