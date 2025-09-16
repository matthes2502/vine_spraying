#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Bool

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


class ADS1115Node(Node):
    def __init__(self):
        super().__init__('ads1115_node')

        # I2C setup
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c, address=0x48)
        ads.gain = 1  # ±4.096 V range

        # Analog channel A0
        self.chan = AnalogIn(ads, ADS.P0)

        # Publisher for fluid pressure messages
        self.publisher_ = self.create_publisher(FluidPressure, 'pressure', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.shutdown_requested = False

        # Subscriber for stop signal
        self.stop_sub = self.create_subscription(Bool, '/peripherie/stop', self.stop_callback, 10)

        self.get_logger().info(f'Pressure Sensor Node started on I2C')
        self.get_logger().info(f'Publishing Rate: 2 Hz')

    def timer_callback(self):
        voltage = self.chan.voltage  # Measured voltage (0–3.125 V with ADS1115 at gain=1)
        input_voltage = voltage / (1000 / (2200 + 1000))  # Recalculate to 0–10 V
        pressure_bar = (input_voltage / 10.0) * 10.0  # 0–10 V → 0–10 bar

        msg = FluidPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.fluid_pressure = pressure_bar  # * 1e5  # bar → Pascal
        msg.variance = 0.0

        self.publisher_.publish(msg)
        # self.get_logger().info(f"Voltage={input_voltage:.2f} V → Pressure={pressure_bar:.6f} bar")

    def stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Stop signal received → shutting down pressure node.")
            # Stop timer and release resources
            self.destroy_timer(self.timer)
            # Trigger ROS shutdown
            self.shutdown_requested = True

def main(args=None):
    rclpy.init(args=args)

    pressure_node = None

    try:
        pressure_node = ADS1115Node()
        while rclpy.ok() and not pressure_node.shutdown_requested:
            rclpy.spin_once(pressure_node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pressure_node.get_logger().info("Ctrl-C received → shutting down node.") # type: ignore
    finally:
        if pressure_node is not None:
            pressure_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
