#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Float32MultiArray
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import Vector3
from threading import Lock

class SprayCoordinator(Node):
    def __init__(self):
        super().__init__('spray_coordinator')
        
        self.get_logger().info("Spray Coordinator started")
        
        # Status variables
        self.spray_enabled = False
        self.shutdown_requested = False
        self.data_lock = Lock()
        
        # Sensor data storage
        self.leaf_wall_density = 0.0
        self.flow_rate = 0.0  # L/min
        self.pressure = 0.0   # Pa
        self.orientation = None
        
        # Control parameters (TODO: Calibrate these later)
        self.base_pump_speed = 1000  # RPM
        self.base_propeller_speed = 800  # RPM
        self.max_pump_speed = 3000
        self.max_propeller_speed = 2000
        
        # === SUBSCRIBERS ===
        # Spray control from Xbox controller
        self.spray_control_sub = self.create_subscription(
            String, '/spray_control', self.spray_control_callback, 10)
        
        # Peripherie stop signal
        self.peripherie_stop_sub = self.create_subscription(
            Bool, '/peripherie/stop', self.stop_callback, 10)
        
        # Sensor data subscriptions
        self.leaf_density_sub = self.create_subscription(
            Float32, '/lidar/leaf_wall_density', self.leaf_density_callback, 10)
        
        self.flow_rate_sub = self.create_subscription(
            Float32, '/flow_sensor_node/flow_rate_l_per_min', self.flow_rate_callback, 10)
        
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/pressure_node/pressure', self.pressure_callback, 10)
        
        self.orientation_sub = self.create_subscription(
            Vector3, '/bno085_node/orientation', self.orientation_callback, 10)
        
        # === PUBLISHERS ===
        # Actuator control publishers
        self.pump_speed_pub = self.create_publisher(Float32, '/pump/speed_command', 10)
        self.propeller_speed_pub = self.create_publisher(Float32, '/propellers/speed_command', 10)
        
        # Single valve control publisher for all valves
        self.valve_commands_pub = self.create_publisher(Float32MultiArray, '/valves/commands', 10)
        
        # === CONTROL TIMER ===
        # Main control loop - runs at 10Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Spray Coordinator ready - waiting for sensor data...")
    
    def spray_control_callback(self, msg: String):
        """Handle spray start/pause from Xbox controller"""
        with self.data_lock:
            if msg.data == "spray_start":
                self.spray_enabled = True
                self.get_logger().info("SPRAY ACTIVATED")
            elif msg.data == "spray_pause":
                self.spray_enabled = False
                self.get_logger().info("SPRAY PAUSED")
                # On pause, immediately stop all actuators
                self.send_zero_commands()
    
    def stop_callback(self, msg: Bool):
        """Handle peripherie stop signal"""
        if msg.data:
            self.get_logger().info("Stop signal received - shutting down spray coordinator.")
            with self.data_lock:
                self.spray_enabled = False
            self.send_zero_commands()
            self.destroy_timer(self.control_timer)
            self.shutdown_requested = True
    
    def leaf_density_callback(self, msg: Float32):
        """Update leaf wall density from lidar processing"""
        with self.data_lock:
            self.leaf_wall_density = msg.data
    
    def flow_rate_callback(self, msg: Float32):
        """Update flow rate from flow sensor"""
        with self.data_lock:
            self.flow_rate = msg.data
    
    def pressure_callback(self, msg: FluidPressure):
        """Update pressure from pressure sensor"""
        with self.data_lock:
            self.pressure = msg.fluid_pressure  # Pa
    
    def orientation_callback(self, msg: Vector3):
        """Update orientation from gyroscope (roll, pitch, yaw in degrees)"""
        with self.data_lock:
            self.orientation = msg
    
    def control_loop(self):
        """Main control loop - calculates and sends new actuator commands"""
        if self.shutdown_requested:
            return
        
        with self.data_lock:
            if not self.spray_enabled:
                return  # If spray disabled, do nothing
            
            # Copy current sensor data for calculations
            density = self.leaf_wall_density
            flow = self.flow_rate
            pressure = self.pressure
        
        # === CALCULATION LOGIC (TODO: Implement real algorithms) ===
        pump_speed, propeller_speed, valve_commands = self.calculate_spray_parameters(
            density, flow, pressure)
        
        # === SEND COMMANDS ===
        self.send_actuator_commands(pump_speed, propeller_speed, valve_commands)
    
    def calculate_spray_parameters(self, leaf_density, flow_rate, pressure):
        """
        Calculate optimal spray parameters based on sensor data
        TODO: Implement real calculation algorithms
        """
        # PLACEHOLDER CALCULATIONS
        
        # Pump speed based on leaf wall density
        density_factor = min(leaf_density / 100.0, 2.0)  # Normalization
        target_pump_speed = self.base_pump_speed * (0.5 + density_factor)
        target_pump_speed = min(target_pump_speed, self.max_pump_speed)
        
        # Propeller speed based on pressure and density
        pressure_factor = min(pressure / 100000.0, 1.5)  # 100kPa as baseline
        target_propeller_speed = self.base_propeller_speed * (0.7 + pressure_factor * 0.5)
        target_propeller_speed = min(target_propeller_speed, self.max_propeller_speed)
        
        # Valve settings (3 valves)
        # TODO: Real calculations based on vehicle speed, wind, etc.
        valve_commands = {
            'valve_1': {
                'pressure_percent': min(50.0 + leaf_density * 0.5, 100.0),
                'flow_percent': min(30.0 + flow_rate * 2.0, 90.0)
            },
            'valve_2': {
                'pressure_percent': min(40.0 + leaf_density * 0.4, 100.0),
                'flow_percent': min(25.0 + flow_rate * 1.8, 85.0)
            },
            'valve_3': {
                'pressure_percent': min(45.0 + leaf_density * 0.45, 100.0),
                'flow_percent': min(35.0 + flow_rate * 2.2, 95.0)
            }
        }
        
        return target_pump_speed, target_propeller_speed, valve_commands
    
    def send_actuator_commands(self, pump_speed, propeller_speed, valve_commands):
        """Send commands to all actuators"""
        
        # Pump speed command
        pump_msg = Float32()
        pump_msg.data = pump_speed
        self.pump_speed_pub.publish(pump_msg)
        
        # Propeller speed command  
        propeller_msg = Float32()
        propeller_msg.data = propeller_speed
        self.propeller_speed_pub.publish(propeller_msg)
        
        # Valve commands - single topic for all valves
        self.send_valve_commands(valve_commands)
        
        # Debug output (every 50 cycles = ~5 seconds)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 50 == 0:
            self.get_logger().info(
                f"Pump: {pump_speed:.0f}RPM, Propeller: {propeller_speed:.0f}RPM, "
                f"Density: {self.leaf_wall_density:.1f}, Flow: {self.flow_rate:.1f}L/min"
            )
    
    def send_valve_commands(self, valve_commands):
        """
        Send valve commands - single topic for all valves
        Format: [valve_id, pressure_percent, flow_percent, valve_id, pressure_percent, flow_percent, ...]
        """
        try:
            valve_msg = Float32MultiArray()
            valve_msg.data = [
                # Valve 1
                1.0, valve_commands['valve_1']['pressure_percent'], valve_commands['valve_1']['flow_percent'],
                # Valve 2  
                2.0, valve_commands['valve_2']['pressure_percent'], valve_commands['valve_2']['flow_percent'],
                # Valve 3
                3.0, valve_commands['valve_3']['pressure_percent'], valve_commands['valve_3']['flow_percent']
            ]
            self.valve_commands_pub.publish(valve_msg)
                
        except Exception as e:
            self.get_logger().error(f"Valve command error: {e}")
    
    def send_zero_commands(self):
        """Send zero commands to all actuators (emergency stop)"""
        # Stop pump and propeller
        zero_msg = Float32()
        zero_msg.data = 0.0
        self.pump_speed_pub.publish(zero_msg)
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