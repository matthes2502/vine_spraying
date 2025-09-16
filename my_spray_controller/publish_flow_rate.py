#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import time
from gpiozero import Button
import threading
import numpy as np

class FlowSensorNode(Node):
    def __init__(self):
        super().__init__('flow_sensor_node')
        
        # ROS2 Parameters
        self.declare_parameter('flow_gpio_pin', 24)
        self.declare_parameter('publish_rate', 2.0)  # Hz
        
        gpio_pin = self.get_parameter('flow_gpio_pin').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Calibration data (Pulse/s -> l/min)
        # From your measurement table: [Pulse [1/s], Flow rate l/min]
        self.calibration_data = np.array([
            [1.5, 0.825],
            [4.5, 1.08],
            [7.0, 1.24],
            [10.0, 1.478],
            [12.5, 1.65],
            [14.5, 1.883],
            [17.0, 2.07],
            [19.0, 2.288],
            [22.0, 2.465],
            [45.0, 4.5],
            [68.0, 6.258],
            [90.0, 8.19],
            [110.0, 9.66],
            [122.0, 10.553]
        ])
        
        # Sensor setup
        self.sensor = Button(gpio_pin, pull_up=True)
        self.pulse_count = 0
        self.total_pulses = 0
        self.flow_frequency = 0.0
        self.lock = threading.Lock()
        
        # Event for clean thread stopping
        self.stop_event = threading.Event()
        
        # Flag for shutdown request
        self.shutdown_requested = False
        
        # Register sensor callback
        self.sensor.when_pressed = self._pulse_callback
        
        # Timer thread for frequency calculation (every 1 second)
        self.timer_thread = threading.Thread(target=self._frequency_calculator)
        self.timer_thread.daemon = True
        self.timer_thread.start()
        
        # ROS2 Publishers
        self.flow_rate_l_per_s_pub = self.create_publisher(
            Float32, 
            '~/flow_rate_l_per_s', 
            10
        )
        self.flow_rate_l_per_min_pub = self.create_publisher(
            Float32, 
            '~/flow_rate_l_per_min', 
            10
        )
        self.pulse_frequency_pub = self.create_publisher(
            Float32, 
            '~/pulse_frequency_hz', 
            10
        )
        
        # ROS2 Subscriber for stop signal
        self.stop_sub = self.create_subscription(
            Bool,
            '/peripherie/stop',  # Global stop topic
            self.stop_callback,
            10
        )
        
        # Publisher timer
        self.publish_timer = self.create_timer(
            1.0 / publish_rate, 
            self.publish_flow_data
        )
        
        self.get_logger().info(f'Flow Sensor Node started on GPIO Pin {gpio_pin}')
        self.get_logger().info(f'Publishing Rate: {publish_rate} Hz')
        
    def _pulse_callback(self):
        """Callback for each pulse from the sensor"""
        with self.lock:
            self.pulse_count += 1
            self.total_pulses += 1
        
    def _frequency_calculator(self):
        """Calculates frequency every 1 second in separate thread"""
        while not self.stop_event.wait(1.0):  # Wait 1s or until event is set
            with self.lock:
                # Frequency = pulses per second
                self.flow_frequency = self.pulse_count
                self.pulse_count = 0  # Reset for next second
        
        # If we reach here, stop_event was set
        self.get_logger().info("Frequency calculator thread terminated")
    
    def interpolate_flow_rate(self, pulse_frequency):
        """
        Interpolates flow rate based on pulse frequency
        
        Args:
            pulse_frequency: Measured pulse frequency [Hz]
            
        Returns:
            tuple: (flow_rate_l_per_min, flow_rate_l_per_s)
        """
        if pulse_frequency <= 0:
            return 0.0, 0.0
        
        # Extract pulses and flow rates
        pulses = self.calibration_data[:, 0]
        flow_rates_l_min = self.calibration_data[:, 1]
        
        # Interpolation with numpy
        # Clip to calibrated range
        pulse_frequency = np.clip(pulse_frequency, pulses[0], pulses[-1])
        
        # Linear interpolation
        flow_l_min = np.interp(pulse_frequency, pulses, flow_rates_l_min)
        flow_l_s = flow_l_min / 60.0
        
        return flow_l_min, flow_l_s
    
    def get_current_flow(self):
        """Get current flow measurement"""
        with self.lock:
            current_frequency = self.flow_frequency
            current_total = self.total_pulses
        
        # Interpolate flow rate
        flow_l_min, flow_l_s = self.interpolate_flow_rate(current_frequency)
        
        return {
            'frequency_hz': current_frequency,
            'flow_l_per_min': flow_l_min,
            'flow_l_per_s': flow_l_s,
            'total_pulses': current_total
        }
    
    def publish_flow_data(self):
        """Publishes current flow data"""
        flow_data = self.get_current_flow()
        
        # Create and publish ROS2 messages
        freq_msg = Float32()
        freq_msg.data = float(flow_data['frequency_hz'])
        self.pulse_frequency_pub.publish(freq_msg)
        
        flow_l_min_msg = Float32()
        flow_l_min_msg.data = float(flow_data['flow_l_per_min'])
        self.flow_rate_l_per_min_pub.publish(flow_l_min_msg)
        
        flow_l_s_msg = Float32()
        flow_l_s_msg.data = float(flow_data['flow_l_per_s'])
        self.flow_rate_l_per_s_pub.publish(flow_l_s_msg)
        
        # Debug output (optional, can be commented out later)
        if flow_data['frequency_hz'] > 0:
            self.get_logger().debug(
                f"Flow: {flow_data['flow_l_per_min']:.3f} l/min | "
                f"{flow_data['flow_l_per_s']:.4f} l/s | "
                f"Freq: {flow_data['frequency_hz']:.1f} Hz"
            )
    
    def stop_callback(self, msg):
        """Callback for stop signal"""
        if msg.data:
            self.get_logger().info("Stop signal received. Shutting down node...")
            self.shutdown_requested = True
    
    def stop_sensor(self):
        """Stops the sensor cleanly"""
        self.get_logger().info("Stopping Flow Sensor...")
        
        # Set event to stop thread
        self.stop_event.set()  # Wakes up the waiting thread
        
        if hasattr(self, 'timer_thread'):
            self.timer_thread.join(timeout=2)  # Wait max 2s for thread termination
        
        if hasattr(self, 'publish_timer'):
            self.publish_timer.cancel()
            
        self.get_logger().info("Flow Sensor stopped.")
    
    def reset_total_pulses(self):
        """Service to reset the total pulse counter (optional)"""
        with self.lock:
            self.total_pulses = 0
        self.get_logger().info("Total pulses reset.")

def main(args=None):
    rclpy.init(args=args)
    
    flow_sensor_node = None
    
    try:
        flow_sensor_node = FlowSensorNode()
        
        # Run node with shutdown check
        while rclpy.ok() and not flow_sensor_node.shutdown_requested:
            rclpy.spin_once(flow_sensor_node, timeout_sec=0.1)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        if flow_sensor_node is not None:
            flow_sensor_node.stop_sensor()
            flow_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()