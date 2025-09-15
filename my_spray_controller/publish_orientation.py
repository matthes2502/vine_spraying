#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
import math
import time

class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')
        
        # ROS2 Parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('i2c_address', 0x4A)   # Default BNO085 address
        
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        
        # Flag for shutdown request
        self.shutdown_requested = False
        
        # Calibration/zeroing variables
        self.is_calibrated = False
        self.offset_roll = 0.0
        self.offset_pitch = 0.0
        self.offset_yaw = 0.0
        self.calibration_samples = []
        self.calibration_sample_count = 50  # Number of samples for averaging
        
        # Initialize I2C and BNO085
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(self.i2c, address=i2c_address)
            
            # Enable rotation vector reports
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            
            self.get_logger().info(f"BNO085 initialized successfully at address 0x{i2c_address:02X}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BNO085: {e}")
            self.shutdown_requested = True
            return
        
        # ROS2 Publishers
        self.pitch_pub = self.create_publisher(Float32, '~/pitch', 10)
        self.yaw_pub = self.create_publisher(Float32, '~/yaw', 10)
        self.roll_pub = self.create_publisher(Float32, '~/roll', 10)
        
        # Combined orientation publisher (optional)
        self.orientation_pub = self.create_publisher(Vector3, '~/orientation', 10)
        
        # ROS2 Subscriber for stop signal
        self.stop_sub = self.create_subscription(
            Bool,
            '/sensors/stop',
            self.stop_callback,
            10
        )
        
        # Timer for publishing IMU data
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_imu_data)
        
        # Start calibration process
        self.get_logger().info("Starting sensor calibration - please keep sensor still...")
        self.start_calibration()
        
        self.get_logger().info(f'BNO085 Node started')
        self.get_logger().info(f'Publishing Rate: {publish_rate} Hz')
        
    def quaternion_to_euler(self, quat_i, quat_j, quat_k, quat_real):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        
        Args:
            quat_i, quat_j, quat_k, quat_real: Quaternion components
            
        Returns:
            tuple: (roll, pitch, yaw) in degrees
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (quat_real * quat_i + quat_j * quat_k)
        cosr_cosp = 1 - 2 * (quat_i * quat_i + quat_j * quat_j)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (quat_real * quat_j - quat_k * quat_i)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (quat_real * quat_k + quat_i * quat_j)
        cosy_cosp = 1 - 2 * (quat_j * quat_j + quat_k * quat_k)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert from radians to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        return roll_deg, pitch_deg, yaw_deg
    
    def start_calibration(self):
        """Start the calibration process"""
        self.is_calibrated = False
        self.calibration_samples = []
        self.get_logger().info(f"Collecting {self.calibration_sample_count} calibration samples...")
    
    def add_calibration_sample(self, roll, pitch, yaw):
        """Add a sample to calibration data"""
        self.calibration_samples.append((roll, pitch, yaw))
        
        if len(self.calibration_samples) >= self.calibration_sample_count:
            # Calculate average offset
            roll_samples = [s[0] for s in self.calibration_samples]
            pitch_samples = [s[1] for s in self.calibration_samples]
            yaw_samples = [s[2] for s in self.calibration_samples]
            
            self.offset_roll = sum(roll_samples) / len(roll_samples)
            self.offset_pitch = sum(pitch_samples) / len(pitch_samples)
            self.offset_yaw = sum(yaw_samples) / len(yaw_samples)
            
            self.is_calibrated = True
            self.get_logger().info(
                f"Calibration complete! Offsets - Roll: {self.offset_roll:.2f}°, "
                f"Pitch: {self.offset_pitch:.2f}°, Yaw: {self.offset_yaw:.2f}°"
            )
    
    def reset_calibration(self):
        """Reset calibration - can be called via service later"""
        self.get_logger().info("Resetting sensor calibration...")
        self.start_calibration()
    
    def publish_imu_data(self):
        """Read IMU data and publish orientation"""
        try:
            # Read quaternion from BNO085
            quat_data = self.bno.quaternion
            
            # Check if data is valid
            if quat_data is None or None in quat_data:
                self.get_logger().warn("No quaternion data available from BNO085")
                return
            
            quat_i, quat_j, quat_k, quat_real = quat_data
            
            # Convert quaternion to Euler angles
            roll, pitch, yaw = self.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
            
            # If not calibrated yet, collect calibration samples
            if not self.is_calibrated:
                self.add_calibration_sample(roll, pitch, yaw)
                # Don't publish during calibration
                return
            
            # Apply calibration offsets (zero the sensor)
            roll_calibrated = roll - self.offset_roll
            pitch_calibrated = pitch - self.offset_pitch
            yaw_calibrated = yaw - self.offset_yaw
            
            # Normalize yaw to -180 to +180 degrees
            while yaw_calibrated > 180:
                yaw_calibrated -= 360
            while yaw_calibrated < -180:
                yaw_calibrated += 360
            
            # Publish individual angles (calibrated values)
            pitch_msg = Float32()
            pitch_msg.data = pitch_calibrated
            self.pitch_pub.publish(pitch_msg)
            
            yaw_msg = Float32()
            yaw_msg.data = yaw_calibrated
            self.yaw_pub.publish(yaw_msg)
            
            roll_msg = Float32()
            roll_msg.data = roll_calibrated
            self.roll_pub.publish(roll_msg)
            
            # Publish combined orientation
            orientation_msg = Vector3()
            orientation_msg.x = roll_calibrated   # Roll
            orientation_msg.y = pitch_calibrated  # Pitch
            orientation_msg.z = yaw_calibrated    # Yaw
            self.orientation_pub.publish(orientation_msg)
            
            # Debug output (optional, can be commented out later)
            self.get_logger().debug(
                f"Calibrated Orientation - Roll: {roll_calibrated:.1f}°, "
                f"Pitch: {pitch_calibrated:.1f}°, Yaw: {yaw_calibrated:.1f}°"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error reading BNO085 data: {e}")
    
    def stop_callback(self, msg):
        """Callback for stop signal"""
        if msg.data:
            self.get_logger().info("Stop signal received → shutting down node.")
            # Stop timer and release resources
            if hasattr(self, 'timer'):
                self.destroy_timer(self.timer)
            # Set shutdown flag
            self.shutdown_requested = True

def main(args=None):
    rclpy.init(args=args)
    
    bno085_node = None
    
    try:
        bno085_node = BNO085Node()
        
        # Run node with shutdown check
        while rclpy.ok() and not bno085_node.shutdown_requested:
            rclpy.spin_once(bno085_node, timeout_sec=0.1)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        if bno085_node is not None:
            bno085_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()