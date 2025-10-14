#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from gpiozero import OutputDevice

'''README
- all relays are NC by default --> pin state acitve_high=False
- for switching e.g. main pump: ros2 topic pub --once /peripherie/relay_control std_msgs/msg/String "data: 'main_pump_off'"
- for starting node with main pump switched on: ros2 run my_spray_controller relais_node --ros-args -p main_pump_init:='"on"'
'''

class RelayControlNode(Node):
    def __init__(self):
        super().__init__('relay_control_node')
        
        # Parameters for initial states
        self.declare_parameter('main_pump_init', 'off')  # 'on' or 'off'
        self.declare_parameter('transfer_pump_init', 'off')  # 'on' or 'off'
        self.declare_parameter('valve_init', 'off')  # 'off', 'fw', or 'psm'
        
        main_pump_init = self.get_parameter('main_pump_init').value
        transfer_pump_init = self.get_parameter('transfer_pump_init').value
        valve_init = self.get_parameter('valve_init').value
        
        # GPIO Pin Mapping
        self.relay_mapping = {
            'main_pump': 24,
            'transfer_pump': 25,
            'valve_direction_fw': 17,
            'valve_direction_psm': 27,
        }
        
        # Initialize all relays as OFF
        # Set active_high=False if your relays are LOW-active (relay switches on LOW signal)
        self.relays = {}
        for name, pin in self.relay_mapping.items():
            self.relays[name] = OutputDevice(pin, active_high=False, initial_value=False)
        
        # Set initial states
        if main_pump_init == 'on':
            self.relays['main_pump'].on()
        
        if transfer_pump_init == 'on':
            self.relays['transfer_pump'].on()
        
        if valve_init == 'fw':
            self.relays['valve_direction_fw'].on()
            self.relays['valve_direction_psm'].off()
        elif valve_init == 'psm':
            self.relays['valve_direction_fw'].off()
            self.relays['valve_direction_psm'].on()
        # else: both stay off
        
        self.get_logger().info("Relay Control Node started")
        
        # Subscribers
        self.control_sub = self.create_subscription(
            String,
            '/peripherie/relay_control',
            self.relay_control_callback,
            10
        )
        
        self.stop_sub = self.create_subscription(
            Bool,
            '/peripherie/stop',
            self.stop_callback,
            10
        )
        
        self.shutdown_requested = False
    
    def relay_control_callback(self, msg: String):
        """
        Commands: 
        - 'main_pump_on' / 'main_pump_off'
        - 'transfer_pump_on' / 'transfer_pump_off'
        - 'valve_off' / 'valve_fw' / 'valve_psm'
        """
        command = msg.data.lower()
        
        # Pump commands
        if command == 'main_pump_on':
            self.relays['main_pump'].on()
        elif command == 'main_pump_off':
            self.relays['main_pump'].off()
        elif command == 'transfer_pump_on':
            self.relays['transfer_pump'].on()
        elif command == 'transfer_pump_off':
            self.relays['transfer_pump'].off()
        
        # Valve commands
        elif command == 'valve_off':
            self.relays['valve_direction_fw'].off()
            self.relays['valve_direction_psm'].off()
        elif command == 'valve_fw':
            self.relays['valve_direction_fw'].on()
            self.relays['valve_direction_psm'].off()
        elif command == 'valve_psm':
            self.relays['valve_direction_fw'].off()
            self.relays['valve_direction_psm'].on()
        
        else:
            self.get_logger().warning(f"Unknown command: '{command}'")
    
    def stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Stop signal received")
            self.shutdown_requested = True
    
    def cleanup(self):
        print("Cleaning up - setting all relays to LOW")
        for relay in self.relays.values():
            relay.off()
            relay.close()


def main(args=None):
    rclpy.init(args=args)
    relay_node = None
    
    try:
        relay_node = RelayControlNode()
        
        while rclpy.ok() and not relay_node.shutdown_requested:
            rclpy.spin_once(relay_node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        if relay_node:
            print("\nCtrl-C received")
    
    finally:
        if relay_node is not None:
            relay_node.cleanup()
            relay_node.destroy_node()
        if rclpy.ok():  # Only shutdown if not already shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()