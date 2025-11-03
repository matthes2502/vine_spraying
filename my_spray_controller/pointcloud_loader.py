#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import os
import glob

class PointcloudLoader(Node):
    def __init__(self):
        super().__init__('pointcloud_loader')
        
        # Parameters (from old code - filtering functionality)
        self.declare_parameter('scan_directory', '/home/matthes/Projects/ros2_ws/src/my_spray_controller/lidar_scans')
        self.declare_parameter('index_from_last', 1)
        self.declare_parameter('filter_left_distance', -1.0)   # -1 = no filter
        self.declare_parameter('filter_right_distance', -1.0)  # -1 = no filter  
        self.declare_parameter('filter_tolerance', 0.3)        # ±30cm tolerance
        self.declare_parameter('center_points', True)          # Enable/disable centering
        self.declare_parameter('cyclic_publish', True)  # Default: off for comparisons
        
        scan_directory = self.get_parameter('scan_directory').value
        index_from_last = self.get_parameter('index_from_last').value
        self.filter_left_distance = self.get_parameter('filter_left_distance').value
        self.filter_right_distance = self.get_parameter('filter_right_distance').value
        self.filter_tolerance = self.get_parameter('filter_tolerance').value
        self.center_points = self.get_parameter('center_points').value
        self.cyclic_publish = self.get_parameter('cyclic_publish').value
        
        # Publisher (using working name from new code)
        topic_name = f"/pointcloud_scan_{index_from_last}"
        self.publisher_ = self.create_publisher(PointCloud2, topic_name, 10)
        # self.publisher_ = self.create_publisher(PointCloud2, '/pointcloud', 10)
        
        # Load points (using improved loading from new code)
        self.current_points = self.load_pointcloud(scan_directory, index_from_last)
        
        if self.current_points is not None and len(self.current_points) > 0:
            self.get_logger().info(f"Loaded {len(self.current_points)} points.")
            # First publish
            self.publish_pointcloud()
            # Timer for cyclic republishing only when needed
            if self.cyclic_publish:
                self.timer = self.create_timer(1.0, self.publish_pointcloud)
                self.get_logger().info("Cyclic publishing enabled")
            else:
                self.get_logger().info("One-time publishing only")
        else:
            self.get_logger().error("No valid points found!")

    def load_pointcloud(self, directory, index_from_last):
        """Load pointcloud file with improved error handling from new code."""
        try:
            # Find all scan files
            pattern = os.path.join(directory, "scan_*.npy")
            files = glob.glob(pattern)
            
            if not files:
                self.get_logger().error(f"No scan files found in {directory}")
                return None
            
            # Sort by modification time (newest first)
            files.sort(key=os.path.getmtime, reverse=True)
            
            if index_from_last >= len(files):
                self.get_logger().error(f"Index {index_from_last} too high, only {len(files)} files available")
                return None
            
            # Load the requested file
            selected_file = files[index_from_last-1]
            self.get_logger().info(f"Loading scan file: {selected_file}")
            
            points = np.load(selected_file)
            
            # Validate shape (from new code)
            if points.shape[1] != 3:
                self.get_logger().error(f"Invalid pointcloud shape: {points.shape}")
                return None
            
            # Force Float32 (from new code)
            points = points.astype(np.float32)
            
            # Apply distance filtering (from old code)
            points = self.filter_points_by_distance(points)
            
            if len(points) == 0:
                self.get_logger().warning("No points remaining after filtering")
                return None
            
            # Log point ranges (from new code)
            self.get_logger().info(f"x-range: {points[:,0].min():.2f} .. {points[:,0].max():.2f}")
            self.get_logger().info(f"y-range: {points[:,1].min():.2f} .. {points[:,1].max():.2f}")
            self.get_logger().info(f"z-range: {points[:,2].min():.2f} .. {points[:,2].max():.2f}")
            
            return points
            
        except Exception as e:
            self.get_logger().error(f"Error loading pointcloud: {str(e)}")
            return None

    def filter_points_by_distance(self, points):
        """Filter points based on left/right distance parameters (from old code)."""
        if len(points) == 0:
            return points
        
        # If both distances are -1, return all points (no filtering)
        if self.filter_left_distance == -1.0 and self.filter_right_distance == -1.0:
            self.get_logger().info("No distance filtering applied (both distances = -1)")
            return points
        
        filtered_points = []
        
        for point in points:
            x, y, z = point
            keep_point = False
            
            # Check left side (positive Y)
            if y > 0 and self.filter_left_distance != -1.0:
                if abs(y - self.filter_left_distance) <= self.filter_tolerance:
                    keep_point = True
            
            # Check right side (negative Y)  
            elif y < 0 and self.filter_right_distance != -1.0:
                if abs(abs(y) - self.filter_right_distance) <= self.filter_tolerance:
                    keep_point = True
            
            # If one side is disabled (-1), include all points from that side
            elif y > 0 and self.filter_left_distance == -1.0:
                keep_point = True
            elif y < 0 and self.filter_right_distance == -1.0:
                keep_point = True
            
            # Points at Y=0 (center line)
            elif y == 0:
                keep_point = True
                
            if keep_point:
                filtered_points.append(point)
        
        filtered_points = np.array(filtered_points) if filtered_points else np.array([]).reshape(0, 3)
        
        self.get_logger().info(
            f"Filtered points: {len(points)} → {len(filtered_points)} "
            f"(Left: {self.filter_left_distance}m, Right: {self.filter_right_distance}m, "
            f"Tolerance: ±{self.filter_tolerance}m)"
        )
        
        return filtered_points

    def publish_pointcloud(self):
        """Publish pointcloud using improved method from new code."""
        if self.current_points is None or len(self.current_points) == 0:
            self.get_logger().warn("No points to publish.")
            return

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"  # Using laser frame from old code
        msg.height = 1
        msg.width = len(self.current_points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        
        # Use tobytes() method from new code (more efficient than struct packing)
        msg.data = self.current_points.tobytes()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointcloudLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()