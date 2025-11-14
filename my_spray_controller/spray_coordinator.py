#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Float32MultiArray, Int32, Int32MultiArray
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import Vector3, Twist
from threading import Lock
from typing import Dict, Tuple, List, Optional, Any
import threading
import time
from datetime import datetime
import os

class SprayCoordinator(Node):
    def __init__(self):
        super().__init__('spray_coordinator')
        
        self.get_logger().info("Spray Coordinator started")

        # Get parameters
        self.declare_parameter('selected_nozzle', 'purple')
        selected_nozzle = str(self.get_parameter('selected_nozzle').value)        

        # Status variables
        self.spray_enabled = False
        self.shutdown_requested = False
        self.data_lock = Lock()
        
        # VESC initialization
        self.vesc_ready = False
        self.vesc_init_start_time = None
        
        # Sensor data storage
        self.leaf_wall_density = [0, 0, 0]  # [bottom, middle, top]
        self.flow_rate = 0.0  # L/min
        self.pressure = 0.0   # bar
        self.orientation = None

        self.pressure_log = []  # [(timestamp, pressure), ...]
        self.pump_running = False

        # Zone mapping
        self.zones = ['bottom', 'middle', 'top']

        # Nozzle configuration
        self.nozzle_pressures = {
            'orange': 4.9,
            'green': 2.2,
            'purple': 5.4,
            'blue': 3.8
        }

        #  Validate nozzle parameter
        if selected_nozzle not in self.nozzle_pressures:
            self.get_logger().error(f"Invalid nozzle '{selected_nozzle}'. Valid options: {list(self.nozzle_pressures.keys())}")
            raise ValueError(f"Invalid nozzle selection: {selected_nozzle}")

        self.target_pressure = self.nozzle_pressures[selected_nozzle]
        self.get_logger().info(f"Using nozzle: {selected_nozzle} ({self.target_pressure} bar)")
                
        # Flag to track if we need to recalculate
        self.density_updated = False
        self.last_calculated_params = None

        # Rovo parameters
        self.rovo_speed = 0.0

        # Leaf density calibration (points at different speeds and densities)
        self.density_calibration = {
            # speed_m_s -> zone -> {density_percent: lidar_points}
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
        
        # Pump calibration table
        # Structure: {pressure: {density_percent: duty_cycle}}
        self.pump_calibration = {
            4.9: {50: 1319, 60: 1321, 70: 1325, 80: 1328, 90: 1331, 100: 1334},
            2.2: {50: 1240, 60: 1245, 70: 1249, 80: 1253, 90: 1257, 100: 1262},
            5.4: {50: 1395, 60: 1405, 70: 1417, 80: 1430, 90: 1443, 100: 1458},
            3.8: {50: 1341, 60: 1357, 70: 1372, 80: 1390, 90: 1411, 100: 1422}
        }
        
        # === SUBSCRIBERS ===
        # Spray control from Xbox controller
        self.spray_control_sub = self.create_subscription(
            String, '/spray_control', self.spray_control_callback, 10)
        
        # Peripherie stop signal
        self.peripherie_stop_sub = self.create_subscription(
            Bool, '/peripherie/stop', self.stop_callback, 10)
        
        # Rovo speed subscriber
        self.speed_sub = self.create_subscription(
            Twist, '/rovo/speed', self.rovo_speed_callback, 10)
        
        # Sensor data subscriptions
        self.leaf_density_sub = self.create_subscription(
            Int32MultiArray, '/grid_density/left', self.leaf_density_callback, 10)
        
        self.flow_rate_sub = self.create_subscription(
            Float32, '/flow_sensor_node/flow_rate_l_per_min', self.flow_rate_callback, 10)
        
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/pressure', self.pressure_callback, 10)
        
        self.orientation_sub = self.create_subscription(
            Vector3, '/bno085_node/orientation', self.orientation_callback, 10)
        
        # === PUBLISHERS ===
        # Relay control
        self.relay_pub = self.create_publisher(String, '/peripherie/relay_control', 10)
        
        # Actuator control publishers
        self.pump_speed_pub = self.create_publisher(Float32, '/pump/pwm_command', 10)
        self.propeller_speed_pub = self.create_publisher(Float32, '/propellers/speed_command', 10)
        
        # Single valve control publisher for all valves
        self.valve_commands_pub = self.create_publisher(Float32MultiArray, '/valve/commands', 10)
        
        # === CONTROL TIMER ===
        # Main control loop - runs at 10Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Spray Coordinator ready - waiting for sensor data...")
    
    def spray_control_callback(self, msg: String):
        """Handle spray start/pause from Xbox controller"""
        # MAYBE: also adding relais for inline pump
        with self.data_lock:
            if msg.data == "spray_start":
                if not self.spray_enabled:
                    # Turn on relay first
                    relay_msg = String()
                    relay_msg.data = 'main_pump_on'
                    self.relay_pub.publish(relay_msg)
                    self.get_logger().info("SPRAY ACTIVATED - Relay ON, waiting for VESC...")
                    
                    # Start VESC initialization
                    self.vesc_init_start_time = time.time()
                    self.vesc_ready = False
                    self.spray_enabled = False  # Will be enabled after VESC init
                    
            elif msg.data == "spray_pause":
                self.spray_enabled = False
                self.vesc_ready = False
                self.get_logger().info("SPRAY PAUSED")
                # On pause, immediately stop all actuators
                self.send_zero_commands()
                # Turn off relay
                relay_msg = String()
                relay_msg.data = 'main_pump_off'
                self.relay_pub.publish(relay_msg)
    
    def stop_callback(self, msg: Bool):
        """Handle peripherie stop signal"""
        if msg.data:
            self.get_logger().info("Stop signal received - shutting down spray coordinator.")
            stop_receive_time = datetime.now()
            self.get_logger().info(f"=== STOP RECEIVED AT: {stop_receive_time.strftime('%H:%M:%S.%f')[:-3]} ===")
            with self.data_lock:
                self.spray_enabled = False
                self.vesc_ready = False
            self.send_zero_commands()
            self.save_pressure_data()  # Save pressure
            # Turn off relay
            relay_msg = String()
            relay_msg.data = 'main_pump_off'
            self.relay_pub.publish(relay_msg)
            self.destroy_timer(self.control_timer)
            self.shutdown_requested = True
    
    def rovo_speed_callback(self, msg: Twist):
        """Update leaf wall density from lidar processing"""
        with self.data_lock:
            self.rovo_speed = msg.linear.x
    
    def leaf_density_callback(self, msg):
        """Update leaf wall density from lidar processing"""
        with self.data_lock:
            # Received leaf wall data for the first time
            if not self.pump_running:
                self.pump_running = True
                self.get_logger().info("First vegetation data received - Starting pressure logging")
            
            if len(msg.data) >= 3:
                self.leaf_wall_density = [msg.data[0], msg.data[1], msg.data[2]]
                self.density_updated = True

            density_receive_time = datetime.now()
            self.get_logger().info(f"=== DENSITIES RECEIVED AT: {density_receive_time.strftime('%H:%M:%S.%f')[:-3]} ===")

             # Calculate new parameters and send commands
            if self.spray_enabled:
                pump_speed, propeller_speed, valve_commands = self.calculate_spray_parameters(self.leaf_wall_density)
                self.send_actuator_commands(pump_speed, propeller_speed, valve_commands)

    def flow_rate_callback(self, msg: Float32):
        """Update flow rate from flow sensor"""
        with self.data_lock:
            self.flow_rate = msg.data
    
    def pressure_callback(self, msg: FluidPressure):
        """Update pressure from pressure sensor"""
        with self.data_lock:
            self.pressure = msg.fluid_pressure  # bar
    
    def orientation_callback(self, msg: Vector3):
        """Update orientation from gyroscope (roll, pitch, yaw in degrees)"""
        with self.data_lock:
            self.orientation = msg

    def linear_interpolate(self, x, x1, y1, x2, y2):
        """Linear interpolation between two points"""
        if x1 == x2:
            return y1
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)

    def get_calibration_for_speed(self, current_speed, zone):
        """
        Get calibration data for current speed and zone using interpolation
        Returns: {60: points_60, 100: points_100} for the given speed
        """
        available_speeds = sorted(self.density_calibration.keys())
        
        # Clamp speed to available range
        min_speed = min(available_speeds)
        max_speed = max(available_speeds)
        
        if current_speed <= min_speed:
            return self.density_calibration[min_speed][zone]
        elif current_speed >= max_speed:
            return self.density_calibration[max_speed][zone]
        
        # Find surrounding speeds for interpolation
        lower_speed = min_speed
        upper_speed = max_speed
        
        for speed in available_speeds:
            if speed <= current_speed:
                lower_speed = speed
            if speed >= current_speed and speed > lower_speed:
                upper_speed = speed
                break
        
        if lower_speed == upper_speed:
            return self.density_calibration[lower_speed][zone]
        
        # Interpolate between lower and upper speed calibrations
        lower_cal = self.density_calibration[lower_speed][zone]
        upper_cal = self.density_calibration[upper_speed][zone]
        
        # Interpolate for both 60% and 100% density points
        interpolated_cal = {}
        for density_percent in [60, 100]:
            points_lower = lower_cal[density_percent]
            points_upper = upper_cal[density_percent]
            
            interpolated_points = self.linear_interpolate(
                current_speed, lower_speed, points_lower, upper_speed, points_upper
            )
            interpolated_cal[density_percent] = interpolated_points
        
        return interpolated_cal

    def calculate_density_percentage(self, measured_points, current_speed, zone):
        """Calculate density percentage from measured LiDAR points using speed-based interpolation"""
        if zone not in self.zones:
            self.get_logger().error(f"Invalid zone: {zone}")
            return 0.0
        
        # Get interpolated calibration data for current speed
        calibration_data = self.get_calibration_for_speed(current_speed, zone)
        
        # Get calibration points for current speed
        points_60 = calibration_data[60]
        points_100 = calibration_data[100]
        
        # Always do linear interpolation first
        density_percent = self.linear_interpolate(
            measured_points, points_60, 60.0, points_100, 100.0
        )
        
        # Apply limits after interpolation
        if density_percent > 100:
            return 100.0
        elif density_percent < 50:  # Use 50% as minimum threshold
            return 0.0
        else:
            return density_percent

    def interpolate_valve_flow_percent(self, density_percent):
        """Convert density percentage to valve flow percentage"""
        # Simple linear mapping: density% = flow%
        # You can adjust this if needed
        return min(100.0, max(0.0, density_percent))

    def interpolate_pump_duty(self, avg_density_percent):
        """Interpolate pump duty cycle based on average density and current pressure"""
        # Find closest pressure value
        calibration_data = self.pump_calibration[self.target_pressure]
        
        # Clamp density to valid range
        avg_density_percent = max(50, min(100, avg_density_percent))
        
        # If exact match exists
        if avg_density_percent in calibration_data:
            return calibration_data[avg_density_percent]
        
        # Find surrounding points for interpolation
        densities = sorted(calibration_data.keys())
        
        # Find the two closest points
        lower_density = None
        upper_density = None
        
        for density in densities:
            if density <= avg_density_percent:
                lower_density = density
            if density >= avg_density_percent and upper_density is None:
                upper_density = density
                break
        
        if lower_density is None:
            return calibration_data[densities[0]]
        if upper_density is None:
            return calibration_data[densities[-1]]
        if lower_density == upper_density:
            return calibration_data[lower_density]
        
        # Linear interpolation
        return self.linear_interpolate(
            avg_density_percent,
            lower_density, calibration_data[lower_density],
            upper_density, calibration_data[upper_density]
        )

    def calculate_spray_parameters(self, density)-> Tuple[float, float, Dict[str, Dict[str, float]]]:
        """Calculate spray parameters based on current density and sensor data
        Returns [pump_speed, propeller_speed, valve_commands]
        """
        self.get_logger().info(f"\nCALCULATION OF SPRAY PARAMS \n")

        if not density or len(density) < 3:
            self.get_logger().warning("Invalid density data")
            return 1000.0, 0.0, self.get_default_valve_commands()
        
        # Convert raw LiDAR point counts to density percentages
        density_percentages = []
        for i, measured_points in enumerate(density):
            zone = self.zones[i]
            density_percent = self.calculate_density_percentage(
                measured_points, self.rovo_speed, zone
            )
            density_percentages.append(density_percent)
        
        # Calculate valve flow percentages for each zone
        valve_commands = {}
        for i, density_percent in enumerate(density_percentages):
            # Convert density percentage to flow percentage
            flow_percent = self.interpolate_valve_flow_percent(density_percent)
            
            valve_commands[f'valve_{i+1}'] = {
                'system_pressure': self.pressure,
                'flow_percent': flow_percent
            }
        
        # Calculate pump speed based on average density
        avg_density = sum(density_percentages) / len(density_percentages)
        if avg_density < 50.0:
            target_pump_duty = 0.0
        else:
            target_pump_duty = self.interpolate_pump_duty(avg_density)
        
        # Propeller speed (not used in your current setup)
        propeller_speed = 20.0

        # Delete later, just for showcase
        # Map pump duty (1000-2000) to propeller speed (20-70%)
        # pump_range = 1458 - 1240  # 1000
        # propeller_range = 70.0 - 20.0  # 50%
        # propeller_speed = 20.0 + (target_pump_duty - 1240) / pump_range * propeller_range
        
        valve_flows = [f"{cmd['flow_percent']:.1f}%" for cmd in valve_commands.values()]
        densities_formatted = [f"{d:.1f}%" for d in self.leaf_wall_density]

        self.get_logger().info(
            f"Current Speed: {self.rovo_speed:.2f} m/s, \n Points: {density}, \n "
            f"Densities: {[f'{d:.1f}%' for d in density_percentages]}, \n "
            f"Avg: {avg_density:.1f}%, Pump: {target_pump_duty}, \n "
            f"Valve flows: {valve_flows}"
        )

        return float(target_pump_duty), float(propeller_speed), valve_commands

    def get_default_valve_commands(self):
        """Return default valve commands when no valid density data"""
        return {
            'valve_1': {'system_pressure': self.pressure, 'flow_percent': 0.0},
            'valve_2': {'system_pressure': self.pressure, 'flow_percent': 0.0},
            'valve_3': {'system_pressure': self.pressure, 'flow_percent': 0.0}
        }

    def control_loop(self):
        """Optimized control loop - only recalculates when density changes"""
        if self.shutdown_requested:
            return
        
        # Handle VESC initialization
        if self.vesc_init_start_time and not self.vesc_ready:
            if time.time() - self.vesc_init_start_time >= 5.0:
                zero_msg = Float32()
                zero_msg.data = 1000.0  # 1000µs = 0%
                self.pump_speed_pub.publish(zero_msg)
                
                self.vesc_ready = True
                self.spray_enabled = True
                self.vesc_init_start_time = None
                self.get_logger().info("VESC ready - Spray enabled") 

        # log pressure only when pump is running
        if self.vesc_ready and self.pump_running:
            with self.data_lock:
                current_time = time.time()
                self.pressure_log.append((current_time, self.pressure))
    
    def send_actuator_commands(self, pump_speed, propeller_speed, valve_commands):
        """Send commands to all actuators"""        
        # Pump speed command --> in duty cycle (1300-2000µs)
        pump_msg = Float32()
        pump_msg.data = float(pump_speed)
        self.pump_speed_pub.publish(pump_msg)
        
        # Propeller speed command  --> in percent (0-100%)
        propeller_msg = Float32()
        propeller_msg.data = float(propeller_speed)
        self.propeller_speed_pub.publish(propeller_msg)
        
        # Valve commands - single topic for all valves
        self.send_valve_commands(valve_commands)        
        
        # Debug output (every 50 cycles = ~5 seconds)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        # if self.debug_counter % 50 == 0:
        #     self.get_logger().info(
        #         f"Pump: {pump_speed:.0f}RPM, Propeller: {propeller_speed:.0f}RPM, "
        #         f"Density: {self.leaf_wall_density:.1f}, Flow: {self.flow_rate:.1f}L/min"
        #     )
    
    def send_valve_commands(self, valve_commands):
        """
        Send valve commands - single topic for all valves
        Format: [valve_id, system_pressure, flow_percent, valve_id, system_pressure, flow_percent, ...]
        valve_1: bottom
        valve_2: middle
        valve_3: top
        """
        try:
            valve_msg = Float32MultiArray() # pressure in mbar
            valve_msg.data = [
                # Valve 1
                1.0, 
                float(valve_commands['valve_1']['system_pressure']), 
                float(valve_commands['valve_1']['flow_percent']),
                # Valve 2  
                2.0, 
                float(valve_commands['valve_2']['system_pressure']), 
                float(valve_commands['valve_2']['flow_percent']),
                # Valve 3
                3.0, 
                float(valve_commands['valve_3']['system_pressure']), 
                float(valve_commands['valve_3']['flow_percent'])
            ]
            self.valve_commands_pub.publish(valve_msg)
                
        except Exception as e:
            self.get_logger().error(f"Valve command error: {e}")
    
    def send_zero_commands(self):
        """Send zero commands to all actuators (emergency stop)"""
        # Stop pump and propeller
        zero_msg = Float32()
        zero_msg.data = 1000.0  # 1000µs = 0% for VESC
        self.pump_speed_pub.publish(zero_msg)
        
        zero_msg.data = 0.0
        self.propeller_speed_pub.publish(zero_msg)
        
        # Stop all valves
        zero_valve_msg = Float32MultiArray()
        zero_valve_msg.data = [
            1.0, 0.0, 0.0,  # Valve 1: pressure=0, flow=0
            2.0, 0.0, 0.0,  # Valve 2: pressure=0, flow=0
            3.0, 0.0, 0.0   # Valve 3: pressure=0, flow=0
        ]
        self.valve_commands_pub.publish(zero_valve_msg)
        
        self.get_logger().info("All actuators stopped")

    def save_pressure_data(self):
        """Save pressure data to specific directory"""
        if self.pressure_log:
            # Ordner erstellen falls nicht vorhanden
            log_dir = "/home/matthes/Projects/ros2_ws/src/my_spray_controller/pressure_logs"
            os.makedirs(log_dir, exist_ok=True)
            
            # Dateiname mit Zeitstempel
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{log_dir}/pressure_log_{timestamp}.csv"
            
            with open(filename, "w") as f:
                f.write("timestamp,pressure_bar\n")
                for timestamp, pressure in self.pressure_log:
                    f.write(f"{timestamp:.3f},{pressure:.3f}\n")
            
            self.get_logger().info(f"Saved {len(self.pressure_log)} pressure measurements to {filename}")

def main(args=None):
    rclpy.init(args=args)
    
    spray_coordinator = None
    
    try:
        spray_coordinator = SprayCoordinator()
        while rclpy.ok() and not spray_coordinator.shutdown_requested:
            rclpy.spin_once(spray_coordinator, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        if spray_coordinator:
            spray_coordinator.get_logger().info("Ctrl-C received - shutting down spray coordinator.")
    finally:
        if spray_coordinator is not None:
            spray_coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()