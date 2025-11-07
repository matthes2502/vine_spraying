#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray, String
import numpy as np
from typing import Optional, List, Tuple, Deque
import struct
import os
from datetime import datetime
from collections import deque
import threading
import time


class LeafWallLidarNode(Node):
    """
    ROS2 node for processing vertical 2D LiDAR scans into 3D point clouds.
    The LiDAR scans vertically (rotated 90°) while moving forward.
    """
    
    def __init__(self):
        super().__init__('leaf_wall_lidar_node')
        
        # Shutdown control
        self.shutdown_requested = False
        self.shutdown_lock = threading.Lock()
        
        # Parameters
        self.declare_parameter('lidar_height', 1.0)  # Height of lidar above ground
        self.declare_parameter('lateral_distance', 1.2)  # Distance to foliage wall (left/right)
        self.declare_parameter('wall_detection_tolerance', 0.25)  # Tolerence around foliage wall (left/right)
        self.declare_parameter('grid_height_min', 0.4)  # Minimum height for grid (40cm)
        self.declare_parameter('grid_height_max', 2.0)  # Maximum height for grid (2m)
        self.declare_parameter('grid_length', 0.2)  # Grid length in driving direction (1m)
        self.declare_parameter('scan_history_distance', 1000.0)  # Keep last X meters of scans (set very high to keep all)
        self.declare_parameter('save_to_file', False)  # Enable/disable saving to file
        self.declare_parameter('scan_output_directory', '/home/matthes/Projects/ros2_ws/src/my_spray_controller/lidar_scans')  # Save in current directory
        
        # Get parameters with type casting to ensure they're not None
        lidar_height_param = self.get_parameter('lidar_height').value
        lateral_param = self.get_parameter('lateral_distance').value
        tolerance_param = self.get_parameter('wall_detection_tolerance').value
        height_min_param = self.get_parameter('grid_height_min').value
        height_max_param = self.get_parameter('grid_height_max').value
        grid_length_param = self.get_parameter('grid_length').value
        scan_history_param = self.get_parameter('scan_history_distance').value
        save_to_file_param = self.get_parameter('save_to_file').value
        output_dir_param = self.get_parameter('scan_output_directory').value
        
        # Ensure parameters are not None before casting
        self.lidar_height = float(lidar_height_param) if lidar_height_param is not None else 1.0
        self.lateral_distance = float(lateral_param) if lateral_param is not None else 1.2
        self.wall_detection_tolerance = float(tolerance_param) if tolerance_param is not None else 0.25
        self.grid_height_min = float(height_min_param) if height_min_param is not None else 0.4
        self.grid_height_max = float(height_max_param) if height_max_param is not None else 2.0
        self.grid_length = float(grid_length_param) if grid_length_param is not None else 1.0
        self.scan_history_distance = float(scan_history_param) if scan_history_param is not None else 1000.0
        self.save_to_file = bool(save_to_file_param) if save_to_file_param is not None else False
        self.scan_output_directory = str(output_dir_param) if output_dir_param is not None else './lidar_scans'
        
        # Grid configuration
        self.grid_height_step = (self.grid_height_max - self.grid_height_min) / 3  # 3 height levels
        self.grid_heights = [
            self.grid_height_min + i * self.grid_height_step 
            for i in range(4)  # 4 boundaries for 3 zones
        ]
        # Indices of scans for each zone
        self.zone_indices = [[] for _ in range(len(self.grid_heights)-1)]
        self.grid_zone_counts = [0] * (len(self.grid_heights)-1)

        self.initial_scan_sub = self.create_subscription(
            LaserScan, "/picoScan_25420273/scan/all_segments_echo0", self.calculate_height_zone_indices, 10)
        
        # State variables
        self.current_speed: float = 0.0  # m/s - Received from ROS2 topic
        self.last_scan_time: Optional[rclpy.time.Time] = None
        self.distance_traveled: float = 0.0  # Total distance traveled
        self.last_grid_publish_distance: float = 0.0  # Distance at last grid publish
        self.laser_frame_id: str = "laser"  # Will be updated from laser scan messages
        self.actual_lateral_distance: Optional[float] = None  # will be updated in detect_wall_presence with actual value

        self.scanning_active: bool = False  # Control scanning state
        self.wall_detected = False  # Check if leaf wall is within range
        self.initial_wall_detection_done = False  # Just check for existance of leafwall at the beginning
        self.indices_calculated = False  # Check if indices have been already calculated

        self.logger_count = 0  # counter for logging
        
        # Data storage with type hints
        self.scan_buffer: Deque[Tuple[np.ndarray, float]] = deque()  # Store (points, y_position) tuples
        self.all_points: np.ndarray = np.array([])  # Current accumulated points
        self.grid_density_history = []

        
        # File handling
        self.output_filename: Optional[str] = ""
        self.scan_timestamp: str = ""
        
        self.density_output_directory = "/home/matthes/Projects/ros2_ws/src/my_spray_controller/densities"
        # Create output directory if saving is enabled
        if self.save_to_file:
            os.makedirs(self.scan_output_directory, exist_ok=True)  # Creates directory if it doesn't exist
            os.makedirs(self.density_output_directory, exist_ok=True)  # Creates directory if it doesn't exist
            self.get_logger().info(f"Saving enabled - files will be saved to directory: {self.scan_output_directory}")
        else:
            self.get_logger().info("Saving disabled - data will not be saved to file")
        
        # Publishers
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, '/pointcloud', 10
        )
        self.grid_left_publisher = self.create_publisher(
            Int32MultiArray, '/grid_density/left', 10
        )
        self.grid_right_publisher = self.create_publisher(
            Int32MultiArray, '/grid_density/right', 10
        )
        
        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/picoScan_25420273/scan/all_segments_echo0', self.scan_callback, 10
        )
        self.spray_subscriber = self.create_subscription(
            String, '/spray_control', self.spray_control_callback, 10
        )
        self.speed_subscriber = self.create_subscription(
            Twist, '/rovo/speed', self.speed_callback, 10
        )
        self.stop_subscriber = self.create_subscription(
            Bool, '/peripherie/stop', self.stop_callback, 10
        )
        
        self.get_logger().info("Vertical LiDAR node initialized")
        self.get_logger().info(f"Grid configuration: {len(self.grid_heights)-1} height zones from {self.grid_height_min}m to {self.grid_height_max}m")
        self.get_logger().info(f"Lateral distance: {self.lateral_distance}m, Grid length: {self.grid_length}m, LiDAR height: {self.lidar_height}m")
    
    def create_grid_visualization_points(self) -> np.ndarray:
        """
        Create artificial points to visualize the grid boxes for debugging.
        Creates complete box edges with points every 2cm.
        """
        if self.distance_traveled < 1.0:  # Mindestens 1 Meter gefahren
            return np.array([])
        
        # Berechne die Anzahl der vollständigen 1-Meter-Spalten
        num_columns = int(self.distance_traveled)
        
        # Left side parameters (positive Y)
        assert self.actual_lateral_distance is not None, "Wall distance should be measured by now"
        center_y = self.actual_lateral_distance  # Expected wall position
        y_min = center_y - 0.25  # -25cm from wall
        y_max = center_y + 0.25  # +25cm from wall
        
        visualization_points = []
        point_spacing = 0.02  # 2cm spacing
        
        # Für jede 1-Meter-Spalte
        for col in range(num_columns):
            min_x = float(col)  # Spalte beginnt bei col Metern
            max_x = float(col + 1)  # Spalte endet bei col+1 Metern
            
            # Create points for each height zone (3 Kästen pro Spalte)
            for i in range(len(self.grid_heights) - 1):
                z_min = self.grid_heights[i]
                z_max = self.grid_heights[i + 1]
                
                # 1. Horizontale Kanten (in X-Richtung)
                x_points = np.arange(min_x, max_x + point_spacing, point_spacing)
                
                # Untere horizontale Kanten
                for x in x_points:
                    visualization_points.extend([
                        [x, y_min, z_min],  # Vordere untere Kante
                        [x, y_max, z_min],  # Hintere untere Kante
                    ])
                
                # Obere horizontale Kanten
                for x in x_points:
                    visualization_points.extend([
                        [x, y_min, z_max],  # Vordere obere Kante
                        [x, y_max, z_max],  # Hintere obere Kante
                    ])
                
                # 2. Vertikale Kanten (in Z-Richtung)
                z_points = np.arange(z_min, z_max + point_spacing, point_spacing)
                
                for z in z_points:
                    visualization_points.extend([
                        [min_x, y_min, z],  # Linke vordere vertikale Kante
                        [min_x, y_max, z],  # Linke hintere vertikale Kante
                        [max_x, y_min, z],  # Rechte vordere vertikale Kante
                        [max_x, y_max, z],  # Rechte hintere vertikale Kante
                    ])
                
                # 3. Seitliche Kanten (in Y-Richtung)
                y_points = np.arange(y_min, y_max + point_spacing, point_spacing)
                
                for y in y_points:
                    visualization_points.extend([
                        [min_x, y, z_min],  # Linke untere seitliche Kante
                        [min_x, y, z_max],  # Linke obere seitliche Kante
                        [max_x, y, z_min],  # Rechte untere seitliche Kante
                        [max_x, y, z_max],  # Rechte obere seitliche Kante
                    ])
        
        return np.array(visualization_points)

    def add_grid_visualization_to_pointcloud(self, points: np.ndarray) -> np.ndarray:
        """
        Add grid visualization points to the main point cloud for debugging.
        Call this in spray_control_callback before publishing.
        """
        grid_points = self.create_grid_visualization_points()
        
        if len(grid_points) > 0 and len(points) > 0:
            # Combine real points with visualization points
            combined_points = np.vstack([points, grid_points])
            self.get_logger().info(f"Added {len(grid_points)} grid visualization points")
            return combined_points
        else:
            return points
    
    def spray_control_callback(self, msg: String):
        """Handle spray control commands to start/stop scanning."""
        command = msg.data.lower()
        
        if command == "spray_start":
            if not self.scanning_active:
                self.scanning_active = True
                self.indices_calculated = False  # Calibrate new indices
                self.wall_detected = False  # Reset wall detection
                self.actual_lateral_distance = None
                self.initial_wall_detection_done = False
                # Clear previous session data
                self.scan_buffer.clear()
                self.all_points = np.array([])
                self.last_grid_publish_distance = 0.0
                self.distance_traveled = 0.0 # Reset distance for new session
                self.get_logger().info("Scanning STARTED - collecting new session")
            
        elif command == "spray_pause":
            if self.scanning_active:
                self.scanning_active = False
                self.wall_detected = False
                self.get_logger().info("Scanning PAUSED - session completed")
                self.get_logger().info(f"Type buffer: {type(self.scan_buffer)}")
                self.get_logger().info(f"Length buffer: {len(self.scan_buffer)}")
                self.all_points = np.vstack([points for points, _ in self.scan_buffer])    # Stack buffered points together
                
                # Add grid visualization for debugging (remove later!)
                self.all_points = self.add_grid_visualization_to_pointcloud(self.all_points)

                # Save current session data
                # Generate NEW filename for this session
                filename = self.generate_daily_filename()
                self.scan_output_filename = os.path.join(self.scan_output_directory, f"scan_{filename}.npy")
                self.density_output_filename = os.path.join(self.density_output_directory, f"density_{filename}.npy")

                # Save point cloud if available
                if len(self.all_points) > 0:
                    self.publish_pointcloud(self.all_points)   # Optional: publish before saving
                    self.save_scan_data(self.all_points)
                else:
                    self.get_logger().warning("No points to save for this session")

                # Save grid density history
                if self.grid_density_history:
                    self.save_grid_density_history()
                else:
                    self.get_logger().warning("No grid densities to save for this session")
    
    def generate_daily_filename(self):
        """Generate filename with date and daily incrementing number."""
        today = datetime.now()
        date_str = today.strftime("%Y%m%d")
        
        # Find existing files for today
        existing_files = []
        if os.path.exists(self.scan_output_directory):
            for filename in os.listdir(self.scan_output_directory):
                if filename.startswith(f"scan_{date_str}_") and filename.endswith(".npy"):
                    existing_files.append(filename)
        
        # Determine next number
        if not existing_files:
            next_num = 1
        else:
            # Extract numbers from existing files
            numbers = []
            for filename in existing_files:
                try:
                    # Extract number between last underscore and .npy
                    num_str = filename.split('_')[-1].replace('.npy', '')
                    numbers.append(int(num_str))
                except ValueError:
                    continue
            next_num = max(numbers) + 1 if numbers else 1
        
        return f"{date_str}_{next_num:03d}"

    def stop_callback(self, msg: Bool):
        """Handle stop signal from /peripherie/stop topic."""
        if msg.data:
            with self.shutdown_lock:
                self.shutdown_requested = True
            self.get_logger().info("Received stop signal, initiating shutdown...")
    
    def speed_callback(self, msg: Twist):
        """Update current robot speed from Twist message."""
        self.current_speed = msg.linear.x  # Forward speed in m/s

    def calculate_actual_wall_distance(self, scan: LaserScan) -> Optional[float]:
        """Extract wall distance calculation as separate function."""
        # Desired angle window (degrees) and converted to rad for ROS2
        angle_min_deg = -100.0
        angle_max_deg = -80.0
        angle_min_rad = np.deg2rad(angle_min_deg)
        angle_max_rad = np.deg2rad(angle_max_deg)

        # Compute indices dynamically
        idx_start = int((angle_min_rad - scan.angle_min) / scan.angle_increment)
        idx_end   = int((angle_max_rad - scan.angle_min) / scan.angle_increment)
        idx_start = max(0, idx_start)
        idx_end   = min(len(scan.ranges), idx_end)

        # Zero-copy: no memory duplication
        ranges = np.frombuffer(scan.ranges, dtype=np.float32)
        segment = ranges[idx_start:idx_end]

        # Remove NaN/inf
        segment = segment[np.isfinite(segment)]
        if segment.size == 0:
            self.get_logger().info(f"Segment Size = 0 --> Return with None")
            return None

        expected_distance = abs(self.lateral_distance)
        min_distance = expected_distance - self.wall_detection_tolerance
        max_distance = expected_distance + self.wall_detection_tolerance

        # Check % of values inside tolerance
        valid_mask = (segment >= min_distance) & (segment <= max_distance)
        valid_segment = segment[valid_mask]
        
        if not self.initial_wall_detection_done and len(valid_segment) / len(segment) < 0.5:
            # self.get_logger().info(f"Less than 30% within distance +- tolerance --> Return None")
            return None
        
        # self.get_logger().info(f"Calculated wall distance: {float(np.mean(valid_segment))}")
        if len(valid_segment) == 0:
            self.get_logger().info("No valid measurements in tolerance range")
            return None
            
        return float(np.mean(valid_segment))

    def detect_wall_presence(self, scan: LaserScan) -> bool:
        """Check for wall at expected distance - simplified version."""
        distance = self.calculate_actual_wall_distance(scan)
        if distance is not None:
            self.actual_lateral_distance = distance
            self.get_logger().info(f"Wall detected at {self.actual_lateral_distance:.3f} m")
            return True
        return False

    def calculate_height_zone_indices(self, msg: LaserScan):
        """Calculate which scan indices correspond to each height zone."""
        if not self.wall_detected or self.indices_calculated:
            return

        # LaserScan data: scan.angle_min, scan.angle_increment, scan.ranges
        angles = msg.angle_min + np.arange(len(msg.ranges)//2) * msg.angle_increment
        assert self.actual_lateral_distance is not None, "actual_lateral_distance must be set before calculating indices"
        heights = self.lidar_height - self.actual_lateral_distance/np.tan(angles)

        # Reset zone_indices
        for i in range(len(self.zone_indices)):
            self.zone_indices[i].clear()

        for i, h in enumerate(heights):
            for z in range(len(self.grid_heights)-1):
                if self.grid_heights[z] <= h < self.grid_heights[z+1]:
                    self.zone_indices[z].append(i)
                    break

        self.indices_calculated = True

        # self.get_logger().info(f"Hight of zone: 1: {heights[self.zone_indices[0][0]-1]}")
        # for i in range(len(self.zone_indices)):
        #     self.get_logger().info(f"Length zone_indices individually: {len(self.zone_indices[i])}")
        #     self.get_logger().info(f"Hight zones: [{self.zone_indices[i][0]}; {self.zone_indices[i][-1]}]")
        #     self.get_logger().info(f"Hight of zone: {i}: {heights[self.zone_indices[i][-1]]}")
   
    def scan_callback(self, msg: LaserScan):
        """Process incoming 2D laser scan and accumulate into 3D point cloud."""
        # Only process scans when scanning is active
        if not self.scanning_active or self.current_speed == 0.0:
            return
        
        current_time = self.get_clock().now()
        
        # Wall detection - early return if no wall
        if not self.wall_detected:
            self.wall_detected = self.detect_wall_presence(msg)
            if not self.wall_detected:
                self.get_logger().info("-----------Nothing within lateral distance range detected--------------")
                return  # Leave callback if there are no close points available
            else:
                self.get_logger().info("Wall detected! Starting grid counting.")
                self.initial_wall_detection_done = True
                self.distance_traveled = 0.0
                self.last_grid_publish_distance = 0.0
                self.last_scan_time = current_time

        # Calculate indices if not happened before
        if not self.indices_calculated:
            self.calculate_height_zone_indices(msg)
            if not self.indices_calculated:  # Falls Berechnung fehlgeschlagen
                return
                        
        # Store the frame_id from the laser scan for consistency
        self.laser_frame_id = msg.header.frame_id
                
        # Calculate distance traveled based on speed and scan rate
        if self.last_scan_time is not None and self.current_speed != 0.0:
            time_delta = (current_time - self.last_scan_time).nanoseconds / 1e9
            
            distance_increment = abs(self.current_speed * time_delta)
            self.distance_traveled += distance_increment
            
            # Log scan rate occasionally for debugging
            if self.logger_count == 50:
            # if True:
                scan_rate = 1.0 / time_delta if time_delta > 0 else 0.0
                self.get_logger().info(f" Scan rate: {scan_rate:.1f} Hz, Travelled distance: {self.distance_traveled:.2f}m , Distance Increment: {distance_increment}m")
                self.logger_count = 0

            self.logger_count = self.logger_count + 1
        
        self.last_scan_time = current_time
        
        # Convert LaserScan to points
        ranges = np.frombuffer(msg.ranges, dtype=np.float32).copy()
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter valid measurements
        valid_indices = np.isfinite(ranges) & (ranges > 0)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        
        if len(ranges) == 0:
            self.get_logger().info(f"-----------FUCK-------: Len_Ranges: {len(ranges)}")
            return
        
        # Convert to 3D points (vertical scan plane)
        # X: forward (driving direction)
        # Y: lateral (left/right)
        # Z: vertical (up/down)
        y_points = - ranges * np.sin(angles) # Lateral distance (left/right)
        z_points = ranges * np.cos(angles) + self.lidar_height  # Vertical position (up/down) corrected for lidar height
        x_points = np.full_like(y_points, self.distance_traveled)  # Forward position (driving direction)

        points_3d_full = np.stack((x_points, y_points, z_points), axis=-1)
        self.calculate_grid_densities(points_3d_full)

        # Filter for lateral distance +- tolerance as well as hight
        wall_filter = (np.abs(y_points - self.lateral_distance) <= self.wall_detection_tolerance) & \
              (z_points <= (self.grid_height_max + 0.5))
        if np.any(wall_filter):
            x_filtered = x_points[wall_filter]
            y_filtered = y_points[wall_filter]
            z_filtered = z_points[wall_filter]
            
            points_3d_filtered = np.stack((x_filtered, y_filtered, z_filtered), axis=-1)
            self.scan_buffer.append((points_3d_filtered, self.distance_traveled))
        else:
            self.get_logger().debug("No points within lateral distance")
            
        # Check if we've traveled enough to publish grid densities
        if self.distance_traveled - self.last_grid_publish_distance >= self.grid_length:
            self.publish_grid_densities()
            self.last_grid_publish_distance = self.distance_traveled
            self.get_logger().info(f"Distance Travelled at publish_grid_densities: {self.distance_traveled}")

            new_lateral_distance = self.calculate_actual_wall_distance(msg)
            # self.get_logger().info(f"New lateral distance {new_lateral_distance}m ")
            if new_lateral_distance is not None and self.actual_lateral_distance is not None:
                lateral_distance_change = abs(new_lateral_distance - self.actual_lateral_distance)
                
                # Rekalibrierung wenn Abstand sich signifikant geändert hat
                if lateral_distance_change > 0.1:  # 10cm Schwellwert
                    self.get_logger().info(f"Distance changed by {lateral_distance_change:.3f}m - recalibrating")
                    self.actual_lateral_distance = new_lateral_distance
                    self.indices_calculated = False
                    self.calculate_height_zone_indices(msg)
            
        # Remove old scans outside history window
        while self.scan_buffer and (self.distance_traveled - self.scan_buffer[0][1]) > self.scan_history_distance:
            self.scan_buffer.popleft()

    def calculate_grid_densities(self, points_3d):
        """
        Calculate point densities for one side (left or right) in 3 height zones.
        
        Args:
            points: Points for this side        
        Returns:
            List of 3 integers representing point counts in each height zone
        """
        y_lower = self.lateral_distance - self.wall_detection_tolerance
        y_upper = self.lateral_distance + self.wall_detection_tolerance

        # Reset temp counts (optional, oder direkt aufs grid_zone_counts addieren)
        for z, indices in enumerate(self.zone_indices):
            # if not indices:
            #     continue

            # Nur die Y-Werte dieser Zone prüfen
            y_vals = points_3d[indices, 1]
            count = np.sum((y_vals >= y_lower) & (y_vals <= y_upper))
            self.grid_zone_counts[z] += count

    def publish_grid_densities(self):
        """Calculate and publish point densities for each grid cell."""
        if not any(self.grid_zone_counts):
            self.get_logger().warning("No points available for grid density calculation")
            return
        
        msg = Int32MultiArray()
        msg.data = self.grid_zone_counts.copy()  # [bottom, middle, top]
        self.grid_left_publisher.publish(msg)

        # Store in history
        self.grid_density_history.append(self.grid_zone_counts.copy())
        # self.get_logger().info(f"----------------Grid Densities: {self.grid_zone_counts}------------")

        # Reset for next grid
        self.grid_zone_counts = [0] * (len(self.grid_heights)-1)

    def save_grid_density_history(self):
        """Save the accumulated grid densities to a .npy file."""
        if not self.grid_density_history:
            self.get_logger().info("No densities to save")
            return  # nothing to save

        np.save(self.density_output_filename, np.array(self.grid_density_history))
        self.get_logger().info(f"Saved grid density history to {self.density_output_filename}")

        # Reset after saving
        self.grid_density_history = []
    
    def save_scan_data(self, points: np.ndarray):
        """Save scan data to new session file if saving is enabled."""
        if not self.save_to_file or len(points) == 0:
            self.get_logger().info(f"Saved deactivated or no scans collected.")
            return
        
        # Save the complete point cloud
        np.save(self.scan_output_filename, points)
        self.get_logger().info(f"Saved scan session to: {self.scan_output_filename}")
        self.get_logger().info(f"Session contained {len(points)} points, traveled {self.distance_traveled:.2f}m")

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
        msg.header.frame_id = self.laser_frame_id  # Use the same frame as the laser scaln
        msg.header.frame_id = 'world'  # Use the same frame as the laser scan
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
    
    def cleanup(self):
        """Clean up resources before shutdown."""        
        print("Vertical LiDAR node cleanup completed")

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = LeafWallLidarNode()
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