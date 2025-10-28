#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import os
import glob
import struct

class PointcloudLoader(Node):
    def __init__(self):
        super().__init__('pointcloud_loader')
        
        # Parameters
        self.declare_parameter('scan_directory', '/home/matthes/Projects/ros2_ws/src/my_spray_controller/lidar_scans')
        self.declare_parameter('index_from_last', 0)
        self.declare_parameter('filter_left_distance', -1.0)   # -1 = no filter
        self.declare_parameter('filter_right_distance', -1.0)  # -1 = no filter  
        self.declare_parameter('filter_tolerance', 0.3)        # ±30cm tolerance
        
        scan_directory = self.get_parameter('scan_directory').value
        index_from_last = self.get_parameter('index_from_last').value
        self.filter_left_distance = self.get_parameter('filter_left_distance').value
        self.filter_right_distance = self.get_parameter('filter_right_distance').value
        self.filter_tolerance = self.get_parameter('filter_tolerance').value
        
        # Publisher
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, '/pointcloud', 10
        )
        
        # Load and publish pointcloud
        self.load_and_publish_pointcloud(scan_directory, index_from_last)
        
        # Timer to republish every second
        self.timer = self.create_timer(1.0, self.republish_pointcloud)
        self.current_pointcloud = None
    
    def load_and_publish_pointcloud(self, directory, index_from_last):
        """Load pointcloud file and publish it."""
        try:
            # Find all scan files
            pattern = os.path.join(directory, "scan_*.npy")
            files = glob.glob(pattern)
            
            if not files:
                self.get_logger().error(f"No scan files found in {directory}")
                return
            
            # Sort by modification time (newest first)
            files.sort(key=os.path.getmtime, reverse=True)
            
            if index_from_last >= len(files):
                self.get_logger().error(f"Index {index_from_last} too high, only {len(files)} files available")
                return
            
            # Load the requested file
            selected_file = files[index_from_last]
            self.get_logger().info(f"Loading scan file: {selected_file}")
            
            points = np.load(selected_file)
            points = self.filter_points_by_distance(points)

            
            if len(points) == 0:
                self.get_logger().warning("Loaded file contains no points")
                return
            
            # Create and publish pointcloud
            self.current_pointcloud = self.create_pointcloud_message(points)
            self.pointcloud_publisher.publish(self.current_pointcloud)
            
            self.get_logger().info(f"Published pointcloud with {len(points)} points")
            
        except Exception as e:
            self.get_logger().error(f"Error loading pointcloud: {str(e)}")

    def filter_points_by_distance(self, points):
        """Filter points based on left/right distance parameters."""
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
    
    def create_pointcloud_message(self, points):
        """Convert numpy points to PointCloud2 message."""
        cloud_data = []
        for point in points:
            x, y, z = point
            cloud_data.append(struct.pack('fff', x, y, z))
        
        cloud_data = b''.join(cloud_data)
        
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
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
        
        return msg
    
    def republish_pointcloud(self):
        """Republish the pointcloud every second."""
        if self.current_pointcloud is not None:
            self.current_pointcloud.header.stamp = self.get_clock().now().to_msg()
            self.pointcloud_publisher.publish(self.current_pointcloud)

def main(args=None):
    rclpy.init(args=args)
    node = PointcloudLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()