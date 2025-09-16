#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
import can
import subprocess
import threading
import time
import struct

class ValveControlNode(Node):
    def __init__(self):
        super().__init__('valve_control_node')
        
        # CAN interface configuration
        self.can_interface = 'can1'
        self.can_bitrate = 250000  # IWN valves use 250 kBaud
        self.shutdown_can_on_exit = False  # Set to True in production, False for testing
        self.can_started_by_us = False  # Track if we started the interface
        
        # Valve command buffer (thread-safe)
        self.valve_commands = {
            1: {'pressure_absolute': 0.0, 'flow_percent': 0.0},
            2: {'pressure_absolute': 0.0, 'flow_percent': 0.0},
            3: {'pressure_absolute': 0.0, 'flow_percent': 0.0}
        }
        self.commands_lock = threading.Lock()
        
        # Control flags
        self.shutdown_requested = False
        self.sender_running = False
        self.sender_thread = None
        
        # Track last sent values to avoid unnecessary CAN traffic
        self.last_system_pressure = -1.0  # Initialize with impossible value
        
        # Setup CAN interface
        if not self.setup_can_interface():
            self.get_logger().error("CAN Interface setup failed!")
            return
            
        # Initialize CAN interface
        try:
            self.can_bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
            self.get_logger().info("CAN Interface initialized")
        except Exception as e:
            self.get_logger().error(f"CAN Interface error: {e}")
            return
            
        # ROS2 subscribers
        self.stop_subscriber = self.create_subscription(Bool, '/peripherie/stop', self.stop_callback, 10)
        
        self.valve_commands_subscriber = self.create_subscription(Float32MultiArray, '/valve/commands', self.valve_commands_callback, 10)
        
        # Start cyclic sender thread (starts immediately with 0% flow)
        self.sender_running = True
        self.sender_thread = threading.Thread(target=self.cyclic_sender)
        self.sender_thread.daemon = True
        self.sender_thread.start()
        
        # Wait a moment for cyclic sender to establish 0% flow rates
        time.sleep(1.0)
        
        # Set valve mode once (after flow rates are established)
        self.set_valve_modes()
                
        self.get_logger().info("Valve control node initialized")
        
    def setup_can_interface(self):
        """Setup CAN interface with automatic configuration"""
        try:
            # Check if interface already exists and is up
            result = subprocess.run(['ip', 'link', 'show', self.can_interface], 
                                  capture_output=True, text=True)
            
            if result.returncode != 0:
                self.get_logger().error(f"CAN Interface {self.can_interface} does not exist!")
                return False
            
            # Check if interface is already UP
            if 'state UP' in result.stdout:
                self.get_logger().info(f"CAN Interface {self.can_interface} is already active")
                return True
            
            # Interface exists but is DOWN - configure it
            self.get_logger().info(f"Configuring CAN Interface {self.can_interface}...")
            
            # Set bitrate and bring up interface
            commands = [
                ['sudo', 'ip', 'link', 'set', self.can_interface, 'type', 'can', 'bitrate', str(self.can_bitrate)],
                ['sudo', 'ip', 'link', 'set', 'up', self.can_interface]
            ]
            
            for cmd in commands:
                result = subprocess.run(cmd, capture_output=True, text=True)
                if result.returncode != 0:
                    self.get_logger().error(f"Error executing: {' '.join(cmd)}")
                    self.get_logger().error(f"Error: {result.stderr}")
                    return False
            
            self.can_started_by_us = True
            self.get_logger().info(f"CAN Interface {self.can_interface} successfully started (Bitrate: {self.can_bitrate})")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error setting up CAN Interface: {e}")
            return False
    
    def shutdown_can_interface(self):
        """Shutdown CAN interface if we started it and shutdown is enabled"""
        if self.shutdown_can_on_exit and self.can_started_by_us:
            try:
                result = subprocess.run(['sudo', 'ip', 'link', 'set', 'down', self.can_interface], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    pass
                else:
                    self.get_logger().warning(f"Error shutting down: {result.stderr}")
            except Exception as e:
                self.get_logger().error(f"Error shutting down CAN Interface: {e}")
        elif self.can_started_by_us:
            self.get_logger().info(f"CAN Interface {self.can_interface} remains active (shutdown_can_on_exit=False)")
        else:
            pass
    
    def set_valve_modes(self):
        """Set valve modes once after flow rates are established"""
        try:
            self.get_logger().info("Setting valve modes...")
            
            for valve_id in [1, 2, 3]:
                can_id = 0x100 + valve_id
                
                # Set valve mode (Sub-addr 2, Mode 126 = 0x7E)
                mode_data = [0x02, 0x7E]
                mode_msg = can.Message(arbitration_id=can_id, data=mode_data, is_extended_id=False)
                self.can_bus.send(mode_msg)
                
                time.sleep(0.1)
                
            self.get_logger().info("All valve modes set to 0x7E")
            
        except Exception as e:
            self.get_logger().error(f"Error setting valve modes: {e}")
    
    def stop_callback(self, msg: Bool):
        """Handle stop signal from ROS topic"""
        if msg.data:
            self.get_logger().info("Stop signal received → shutting down valve control node.")
            
            # Stop cyclic sender
            self.stop_cyclic_sender()
            
            # Trigger ROS shutdown
            self.shutdown_requested = True
    
    def valve_commands_callback(self, msg: Float32MultiArray):
        """Process incoming valve commands"""
        try:
            data = msg.data
            
            # Parse message: [valve_id, pressure_absolute, flow_percent, valve_id, ...]
            if len(data) % 3 != 0:
                self.get_logger().warning("Invalid valve command message length")
                return
            
            with self.commands_lock:
                for i in range(0, len(data), 3):
                    valve_id = int(data[i])
                    pressure_absolute = data[i + 1]
                    flow_percent = data[i + 2]
                    
                    if valve_id in self.valve_commands:
                        self.valve_commands[valve_id]['pressure_absolute'] = pressure_absolute
                        self.valve_commands[valve_id]['flow_percent'] = flow_percent
                        
                        self.get_logger().debug(f"Updated valve {valve_id}: pressure={pressure_absolute:.1f} bar, flow={flow_percent:.1f}%")
                    else:
                        self.get_logger().warning(f"Unknown valve ID: {valve_id}")
                        
        except Exception as e:
            self.get_logger().error(f"Error processing valve commands: {e}")
    
    def stop_cyclic_sender(self):
        """Stop the cyclic CAN sender thread"""
        if self.sender_running:
            self.sender_running = False
            if self.sender_thread:
                self.sender_thread.join(timeout=2.0)
            print("Cyclic CAN sender stopped")
    
    def cyclic_sender(self):
        """Main loop for cyclic CAN message sending"""
        cycle_time = 0.5  # 500ms cycle time
        
        while self.sender_running:
            try:
                with self.commands_lock:
                    # Check if system pressure changed and send once if it did
                    current_pressure = self.valve_commands[1]['pressure_absolute']
                    if current_pressure != self.last_system_pressure:
                        self.send_system_pressure(current_pressure * 1000)  # bar to mbar
                        self.last_system_pressure = current_pressure
                    
                    # Send flow rates cyclically for all valves
                    for valve_id in [1, 2, 3]:
                        flow = self.valve_commands[valve_id]['flow_percent']
                        self.send_flow_rate(valve_id, flow)
                
                time.sleep(cycle_time)
                
            except Exception as e:
                self.get_logger().error(f"Error in cyclic sender: {e}")
                time.sleep(cycle_time)
    
    def send_system_pressure(self, pressure_mbar):
        """Send system pressure to all valves (CAN ID 0x002)"""
        try:
            pressure_int = int(pressure_mbar)
            
            # Convert to 16-bit value (low byte, high byte)
            pressure_low = pressure_int & 0xFF
            pressure_high = (pressure_int >> 8) & 0xFF
            
            pressure_data = [pressure_low, pressure_high]
            
            msg = can.Message(arbitration_id=0x002, data=pressure_data, is_extended_id=False)
            self.can_bus.send(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error sending system pressure: {e}")
    
    def send_flow_rate(self, valve_id, flow_percent):
        """Send flow rate to individual valve (called cyclically)"""
        try:
            can_id = 0x100 + valve_id  # 0x101, 0x102, 0x103
            
            # Send flow rate (0-100% mapped to 0-255)
            flow_value = int(max(0, min(100, flow_percent)) * 2.55)  # Map 0-100% to 0-255
            flow_data = [0x0F, flow_value]  # Sub-addr 15
            flow_msg = can.Message(arbitration_id=can_id, data=flow_data, is_extended_id=False)
            self.can_bus.send(flow_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error sending flow rate to valve {valve_id}: {e}")
    
    def close_all_valves(self):
        """Close all valves by setting them to de-energized state"""
        try:
            print("Closing all valves...")
            
            for valve_id in [1, 2, 3]:
                can_id = 0x100 + valve_id
                
                # Send valve off command (mode 255 = de-energized/open for normally open valves)
                close_data = [0x02, 0xFF]  # Sub-addr 2, Mode 255
                close_msg = can.Message(arbitration_id=can_id, data=close_data, is_extended_id=False)
                self.can_bus.send(close_msg)
                
                time.sleep(0.1)  # Small delay between valves
                
            print("All valves closed")
            
        except Exception as e:
            self.get_logger().error(f"Error closing valves: {e}")
    
    def cleanup(self):
        """Clean up resources"""
        try:
            # Stop cyclic sender
            self.stop_cyclic_sender()
            
            # Close all valves
            self.close_all_valves()
            
            # Close CAN bus
            if hasattr(self, 'can_bus'):
                self.can_bus.shutdown()
                
            # Shutdown CAN interface if needed
            self.shutdown_can_interface()
            
        except Exception as e:
            print(f"Error during cleanup: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    valve_node = None
    
    try:
        valve_node = ValveControlNode()
        while rclpy.ok() and not valve_node.shutdown_requested:
            rclpy.spin_once(valve_node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        if valve_node:
            print("\nCtrl-C received → shutting down valve control node.")
            pass
    finally:
        if valve_node is not None:
            valve_node.cleanup()
            try:
                valve_node.destroy_node()
            except Exception:
                pass  # Ignore errors during node destruction

        # Only shutdown if not already shut down
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors
        
if __name__ == '__main__':
    main()