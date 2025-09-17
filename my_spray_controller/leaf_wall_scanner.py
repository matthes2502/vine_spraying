#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension
import numpy as np
from typing import Optional, List, Tuple, Deque
import struct
import os
from datetime import datetime
from collections import deque
import threading


class VerticalLidarNode(Node):
    """
    ROS2 node for processing vertical 2D LiDAR scans into 3D point clouds.
    The LiDAR scans vertically (rotated 90°) while moving forward.
    """
    
    def __init__(self):
        super().__init__('vertical_lidar_node')
        
        # Shutdown control
        self.shutdown_requested = False
        self.shutdown_lock = threading.Lock()
        
        # Parameters
        self.declare_parameter('lateral_distance', 1.2)  # Distance to foliage wall (left/right)
        self.declare_parameter('grid_height_min', 0.4)  # Minimum height for grid (40cm)
        self.declare_parameter('grid_height_max', 2.0)  # Maximum height for grid (2m)
        self.declare_parameter('grid_length', 0.2)  # Grid length in driving direction (1m)
        self.declare_parameter('scan_history_distance', 1000.0)  # Keep last X meters of scans (set very high to keep all)
        self.declare_parameter('save_to_file', False)  # Enable/disable saving to file
        self.declare_parameter('output_directory', './lidar_scans')  # Save in current directory
        
        # Get parameters with type casting to ensure they're not None
        lateral_param = self.get_parameter('lateral_distance').value
        height_min_param = self.get_parameter('grid_height_min').value
        height_max_param = self.get_parameter('grid_height_max').value
        grid_length_param = self.get_parameter('grid_length').value
        scan_history_param = self.get_parameter('scan_history_distance').value
        save_to_file_param = self.get_parameter('save_to_file').value
        output_dir_param = self.get_parameter('output_directory').value
        
        # Ensure parameters are not None before casting
        self.lateral_distance = float(lateral_param) if lateral_param is not None else 1.2
        self.grid_height_min = float(height_min_param) if height_min_param is not None else 0.4
        self.grid_height_max = float(height_max_param) if height_max_param is not None else 2.0
        self.grid_length = float(grid_length_param) if grid_length_param is not None else 1.0
        self.scan_history_distance = float(scan_history_param) if scan_history_param is not None else 1000.0
        self.save_to_file = bool(save_to_file_param) if save_to_file_param is not None else False
        self.output_directory = str(output_dir_param) if output_dir_param is not None else './lidar_scans'
        
        # Grid configuration
        self.grid_height_step = (self.grid_height_max - self.grid_height_min) / 3  # 3 height levels
        self.grid_heights = [
            self.grid_height_min + i * self.grid_height_step 
            for i in range(4)  # 4 boundaries for 3 zones
        ]
        
        # State variables
        self.current_speed: float = 0.2  # m/s
        self.last_scan_time: Optional[rclpy.time.Time] = None
        self.distance_traveled: float = 0.0  # Total distance traveled
        self.last_grid_publish_distance: float = 0.0  # Distance at last grid publish
        self.laser_frame_id: str = "laser"  # Will be updated from laser scan messages
        
        # Data storage with type hints
        self.scan_buffer: Deque[Tuple[np.ndarray, float]] = deque()  # Store (points, y_position) tuples
        self.all_points: np.ndarray = np.array([])  # Current accumulated points
        
        # File handling
        self.output_filename: Optional[str] = None
        self.scan_timestamp: str = ""
        
        # Create output directory and filename only if saving is enabled
        if self.save_to_file:
            os.makedirs(self.output_directory, exist_ok=True)  # Creates directory if it doesn't exist
            self.scan_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_filename = os.path.join(
                self.output_directory, 
                f"scan_session_{self.scan_timestamp}.npy"
            )
            self.get_logger().info(f"Saving enabled - will save to: {self.output_filename}")
        else:
            self.output_filename = None
            self.get_logger().info("Saving disabled - data will not be saved to file")
        
        # Publishers
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, '/pointcloud', 10
        )
        self.grid_left_publisher = self.create_publisher(
            Float32MultiArray, '/grid_density/left', 10
        )
        self.grid_right_publisher = self.create_publisher(
            Float32MultiArray, '/grid_density/right', 10
        )
        
        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.speed_subscriber = self.create_subscription(
            Twist, '/rovo/speed', self.speed_callback, 10
        )
        self.stop_subscriber = self.create_subscription(
            Bool, '/peripherie/stop', self.stop_callback, 10
        )
        
        self.get_logger().info("Vertical LiDAR node initialized")
        self.get_logger().info(f"Grid configuration: {len(self.grid_heights)-1} height zones from {self.grid_height_min}m to {self.grid_height_max}m")
        self.get_logger().info(f"Lateral distance: {self.lateral_distance}m, Grid length: {self.grid_length}m")
    
    def stop_callback(self, msg: Bool):
        """Handle stop signal from /peripherie/stop topic."""
        if msg.data:
            with self.shutdown_lock:
                self.shutdown_requested = True
            self.get_logger().info("Received stop signal, initiating shutdown...")
    
    def speed_callback(self, msg: Twist):
        """Update current robot speed from Twist message."""
        self.current_speed = msg.linear.x  # Forward speed in m/s
    
    def scan_callback(self, msg: LaserScan):
        """Process incoming 2D laser scan and accumulate into 3D point cloud."""
        current_time = self.get_clock().now()
        
        # Store the frame_id from the laser scan for consistency
        self.laser_frame_id = msg.header.frame_id
        
        # Calculate distance traveled based on speed and scan rate
        # Using scan_time from LaserScan message if available, otherwise calculate from timestamps
        if self.last_scan_time is not None and self.current_speed != 0.0:
            # Option 1: Use scan_time from message (time between scans)
            if hasattr(msg, 'scan_time') and msg.scan_time > 0:
                time_delta = msg.scan_time
            else:
                # Option 2: Calculate from ROS timestamps
                time_delta = (current_time - self.last_scan_time).nanoseconds / 1e9
            
            distance_increment = abs(self.current_speed * time_delta)
            self.distance_traveled += distance_increment
            
            # Log scan rate occasionally for debugging
            if int(self.distance_traveled) % 10 == 0 and int(self.distance_traveled - distance_increment) % 10 != 0:
                scan_rate = 1.0 / time_delta if time_delta > 0 else 0.0
                self.get_logger().info(f"Scan rate: {scan_rate:.1f} Hz, Distance: {self.distance_traveled:.2f}m")
        
        self.last_scan_time = current_time
        
        # Convert LaserScan to points
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter valid measurements
        valid_indices = np.isfinite(ranges) & (ranges > 0)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        
        if len(ranges) == 0:
            return
        
        # Convert to 3D points (vertical scan plane)
        # X: forward (driving direction)
        # Y: lateral (left/right)
        # Z: vertical (up/down)
        y_points = ranges * np.cos(angles)  # Lateral distance (left/right)
        z_points = ranges * np.sin(angles)  # Vertical position (up/down)
        x_points = np.full_like(y_points, self.distance_traveled)  # Forward position (driving direction)
        
        points_3d = np.stack((x_points, y_points, z_points), axis=-1)
        
        # Add to buffer
        self.scan_buffer.append((points_3d, self.distance_traveled))
        
        # Remove old scans outside history window
        while self.scan_buffer and (self.distance_traveled - self.scan_buffer[0][1]) > self.scan_history_distance:
            self.scan_buffer.popleft()
        
        # Combine all buffered scans
        if self.scan_buffer:
            self.all_points = np.vstack([points for points, _ in self.scan_buffer])
            
            # Publish point cloud
            self.publish_pointcloud(self.all_points)
            
            # Check if we've traveled enough to publish grid densities
            if self.distance_traveled - self.last_grid_publish_distance >= self.grid_length:
                self.publish_grid_densities()
                self.last_grid_publish_distance = self.distance_traveled
    
    def publish_pointcloud(self, points):
        """Publish the 3D point cloud as PointCloud2 message."""
        if len(points) == 0:
            return
        
        # Create PointCloud2 message
        cloud_data = []
        for point in points:
            x, y, z = point
            cloud_data.append(struct.pack('fff', x, y, z))
        
        cloud_data = b''.join(cloud_data)
        
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.laser_frame_id  # Use the same frame as the laser scan
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_dense = True
        msg.data = cloud_data
        
        self.pointcloud_publisher.publish(msg)
        
        # Log first publish for debugging
        if not hasattr(self, '_first_publish_logged'):
            self.get_logger().info(f"Publishing point cloud with frame_id: {self.laser_frame_id}")
            self._first_publish_logged = True
    
    def publish_grid_densities(self):
        """Calculate and publish point densities for each grid cell."""
        if len(self.all_points) == 0:
            self.get_logger().warning("No points available for grid density calculation")
            return
        
        # Get points from last grid_length meters
        min_x = self.distance_traveled - self.grid_length
        # Ensure all_points is a numpy array for indexing
        points_array = np.asarray(self.all_points)
        grid_points = points_array[points_array[:, 0] >= min_x]  # Changed from [:, 1] to [:, 0]
        
        if len(grid_points) == 0:
            return
        
        # Separate left and right points (in right-handed coordinate system)
        left_points = grid_points[grid_points[:, 1] > 0]  # Positive Y = left
        right_points = grid_points[grid_points[:, 1] < 0]  # Negative Y = right
        
        # Calculate densities for left side (positive Y)
        left_densities = self.calculate_side_densities(left_points, self.lateral_distance)
        self.publish_density_array(left_densities, self.grid_left_publisher, "left")
        
        # Calculate densities for right side (negative Y)
        right_densities = self.calculate_side_densities(right_points, -self.lateral_distance)
        self.publish_density_array(right_densities, self.grid_right_publisher, "right")
        
        # Log summary
        self.get_logger().info(
            f"Grid densities at {self.distance_traveled:.2f}m - "
            f"Left: {left_densities}, Right: {right_densities}"
        )
    
    def calculate_side_densities(self, points: np.ndarray, target_y: float) -> List[int]:
        """
        Calculate point densities for one side (left or right) in 3 height zones.
        
        Args:
            points: Points for this side
            target_y: Expected Y coordinate for foliage wall (positive for left, negative for right)
        
        Returns:
            List of 3 integers representing point counts in each height zone
        """
        densities: List[int] = []
        
        # Define lateral tolerance (accept points near the expected wall distance)
        lateral_tolerance = 0.3  # ±30cm from expected wall position
        
        # Filter points near the target lateral distance
        if len(points) > 0:
            lateral_mask = np.abs(np.abs(points[:, 1]) - abs(target_y)) < lateral_tolerance  # Changed from [:, 0]
            filtered_points = points[lateral_mask]
        else:
            filtered_points = np.array([])
        
        # Calculate density for each height zone
        for i in range(len(self.grid_heights) - 1):
            height_min = self.grid_heights[i]
            height_max = self.grid_heights[i + 1]
            
            if len(filtered_points) > 0:
                height_mask = (filtered_points[:, 2] >= height_min) & (filtered_points[:, 2] < height_max)
                count = np.sum(height_mask)
            else:
                count = 0
            
            densities.append(int(count))
        
        return densities
    
    def publish_density_array(self, densities, publisher, side_name):
        """Publish density array as Float32MultiArray."""
        msg = Float32MultiArray()
        # Simply publish the 3 density values without complex metadata
        msg.data = [float(d) for d in densities]
        publisher.publish(msg)
    
    def save_scan_data(self, points: np.ndarray):
        """Save scan data to single session file if saving is enabled."""
        if not self.save_to_file or self.output_filename is None or len(points) == 0:
            return
            
        # Save/overwrite the complete point cloud to the session file
        np.save(self.output_filename, points)
        self.get_logger().debug(f"Updated scan file with {len(points)} points")
    
    def cleanup(self):
        """Clean up resources before shutdown."""
        # Save final scan data if available and saving is enabled
        if self.save_to_file and self.output_filename is not None and len(self.all_points) > 0:
            np.save(self.output_filename, self.all_points)
            print(f"Saved final scan session to {self.output_filename}")
            print(f"Total points collected: {len(self.all_points)}")
            print(f"Total distance traveled: {self.distance_traveled:.2f}m")
        elif not self.save_to_file and len(self.all_points) > 0:
            print(f"Session ended - Total points collected: {len(self.all_points)}")
            print(f"Total distance traveled: {self.distance_traveled:.2f}m")
            print("(Saving was disabled - no file created)")
        else:
            print("No scan data collected during this session")
        
        print("Vertical LiDAR node cleanup completed")


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = VerticalLidarNode()
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        if node:
            print("\nCtrl-C received → shutting down vertical LiDAR node.")
    
    finally:
        if node is not None:
            node.cleanup()
            try:
                node.destroy_node()
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