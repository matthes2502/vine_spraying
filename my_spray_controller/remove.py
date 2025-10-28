#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import os, glob

class PointcloudLoader(Node):
    def __init__(self):
        super().__init__('pointcloud_loader')

        # Parameter
        self.declare_parameter('scan_directory', '/home/matthes/Projects/ros2_ws/src/my_spray_controller/lidar_scans')
        self.declare_parameter('index_from_last', 0)

        scan_directory = self.get_parameter('scan_directory').value
        index_from_last = self.get_parameter('index_from_last').value

        # Publisher
        self.publisher_ = self.create_publisher(PointCloud2, '/pointcloud', 10)

        # Load points
        self.current_points = self.load_pointcloud(scan_directory, index_from_last)

        if self.current_points is not None and len(self.current_points) > 0:
            self.get_logger().info(f"Loaded {len(self.current_points)} points.")
            # First publish
            self.publish_pointcloud()
            # Timer for cyclic republishing
            self.timer = self.create_timer(1.0, self.publish_pointcloud)
        else:
            self.get_logger().error("No valid points found!")

    def load_pointcloud(self, directory, index_from_last):
        # Random test points (remove this later)
        points = np.random.rand(1000, 3).astype(np.float32)
        points[:,0] *= 2.0
        points[:,1] *= 2.0
        points[:,2] *= 0.5
        self.current_points = points
        self.publish_pointcloud()

        pattern = os.path.join(directory, "scan_*.npy")
        files = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
        if not files:
            self.get_logger().error(f"No files found in {directory}")
            return None

        idx = min(index_from_last, len(files) - 1)
        selected_file = files[idx]
        self.get_logger().info(f"Loading: {selected_file}")

        points = np.load(selected_file)
        if points.shape[1] != 3:
            self.get_logger().error(f"Invalid pointcloud shape: {points.shape}")
            return None

        # Force Float32
        points = points.astype(np.float32)

        # Center points around origin (NEW ADDITION)
        mean_x = np.mean(points[:, 0])
        mean_y = np.mean(points[:, 1])
        mean_z = np.mean(points[:, 2])
        points[:, 0] -= mean_x
        points[:, 1] -= mean_y
        points[:, 2] -= mean_z
        self.get_logger().info(f"Centered points: moved by ({mean_x:.2f}, {mean_y:.2f}, {mean_z:.2f})")

        self.get_logger().info(f"x-range: {points[:,0].min()} .. {points[:,0].max()}")
        self.get_logger().info(f"y-range: {points[:,1].min()} .. {points[:,1].max()}")
        self.get_logger().info(f"z-range: {points[:,2].min()} .. {points[:,2].max()}")

        return points

    def publish_pointcloud(self):
        if self.current_points is None or len(self.current_points) == 0:
            self.get_logger().warn("No points to publish.")
            return

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
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
        msg.data = self.current_points.tobytes()

        self.publisher_.publish(msg)
        # self.get_logger().info(f"Published pointcloud with {len(self.current_points)} points")

def main(args=None):
    rclpy.init(args=args)
    node = PointcloudLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()